//! Minimal VirtIO GPU PCI driver for QEMU's 2D display backend.
//!
//! This is a modern VirtIO PCI driver (VERSION_1, split control queue). It
//! creates one host-visible 2D resource and backs it with a contiguous guest
//! framebuffer. Rendering stays in the existing software console; dirty
//! rectangles are transferred to the device and presented through VirtIO.

use core::ptr;
use core::sync::atomic::{fence, AtomicBool, Ordering};
use spin::Mutex;

const PCI_VENDOR_VIRTIO: u16 = 0x1af4;
const PCI_DEVICE_GPU: u16 = 0x1050;
const PCI_CAP_VENDOR: u8 = 0x09;
const VIRTIO_F_VERSION_1: u64 = 1 << 32;

const STATUS_ACKNOWLEDGE: u8 = 1;
const STATUS_DRIVER: u8 = 2;
const STATUS_DRIVER_OK: u8 = 4;
const STATUS_FEATURES_OK: u8 = 8;
const STATUS_FAILED: u8 = 128;

const CAP_COMMON: u8 = 1;
const CAP_NOTIFY: u8 = 2;
const CAP_DEVICE: u8 = 4;

const QUEUE_SIZE: u16 = 64;
const RESOURCE_ID: u32 = 1;
const PIXEL_FORMAT_B8G8R8X8_UNORM: u32 = 2;
const MAX_WIDTH: u32 = 4096;
const MAX_HEIGHT: u32 = 2160;

const GPU_CMD_GET_DISPLAY_INFO: u32 = 0x0100;
const GPU_CMD_RESOURCE_CREATE_2D: u32 = 0x0101;
const GPU_CMD_SET_SCANOUT: u32 = 0x0103;
const GPU_CMD_RESOURCE_FLUSH: u32 = 0x0104;
const GPU_CMD_TRANSFER_TO_HOST_2D: u32 = 0x0105;
const GPU_CMD_RESOURCE_ATTACH_BACKING: u32 = 0x0106;
const GPU_RESP_OK_NODATA: u32 = 0x1100;
const GPU_RESP_OK_DISPLAY_INFO: u32 = 0x1101;

const DESC_NEXT: u16 = 1;
const DESC_WRITE: u16 = 2;
const PAGE_SIZE: usize = 4096;
const COMMAND_PAGE_BYTES: usize = 4096;
const REQUEST_OFFSET: usize = 0;
const RESPONSE_OFFSET: usize = 2048;
const RESPONSE_CAPACITY: usize = COMMAND_PAGE_BYTES - RESPONSE_OFFSET;

#[derive(Clone, Copy)]
struct PciLocation {
    bus: u8,
    device: u8,
    function: u8,
}

#[derive(Clone, Copy, Default)]
struct PciRegion {
    base: u64,
    offset: u32,
    length: u32,
}

#[derive(Clone, Copy, Default)]
struct VirtioCaps {
    common: PciRegion,
    notify: PciRegion,
    notify_multiplier: u32,
    device: PciRegion,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Descriptor {
    address: u64,
    length: u32,
    flags: u16,
    next: u16,
}

struct DmaMemory {
    physical: u64,
    virtual_address: *mut u8,
}

unsafe impl Send for DmaMemory {}

struct VirtQueue {
    memory: DmaMemory,
    size: u16,
    avail_offset: usize,
    used_offset: usize,
    avail_index: u16,
    used_index: u16,
}

struct Driver {
    queue: VirtQueue,
    command: DmaMemory,
    framebuffer: DmaMemory,
    width: u32,
    height: u32,
    pitch: usize,
    common: *mut u8,
    notify: *mut u8,
    notify_multiplier: u32,
    notify_length: u32,
    notify_offset: u16,
}

unsafe impl Send for Driver {}

static GPU: Mutex<Option<Driver>> = Mutex::new(None);
static GPU_ACTIVE: AtomicBool = AtomicBool::new(false);

unsafe fn outl(port: u16, value: u32) {
    core::arch::asm!("out dx, eax", in("dx") port, in("eax") value, options(nomem, nostack, preserves_flags));
}

unsafe fn outw(port: u16, value: u16) {
    core::arch::asm!("out dx, ax", in("dx") port, in("ax") value, options(nomem, nostack, preserves_flags));
}

unsafe fn inl(port: u16) -> u32 {
    let value: u32;
    core::arch::asm!("in eax, dx", in("dx") port, out("eax") value, options(nomem, nostack, preserves_flags));
    value
}

fn config_address(location: PciLocation, offset: u8) -> u32 {
    (1 << 31)
        | ((location.bus as u32) << 16)
        | ((location.device as u32) << 11)
        | ((location.function as u32) << 8)
        | (offset as u32 & 0xfc)
}

fn read_config32(location: PciLocation, offset: u8) -> u32 {
    unsafe {
        outl(0xcf8, config_address(location, offset));
        inl(0xcfc)
    }
}

fn write_config16(location: PciLocation, offset: u8, value: u16) {
    unsafe {
        outl(0xcf8, config_address(location, offset));
        outw(0xcfc + (offset as u16 & 2), value);
    }
}

fn find_gpu() -> Option<PciLocation> {
    for bus in 0u16..256 {
        for device in 0u8..32 {
            let location = PciLocation {
                bus: bus as u8,
                device,
                function: 0,
            };
            let id = read_config32(location, 0);
            if id as u16 != PCI_VENDOR_VIRTIO {
                continue;
            }
            let header = (read_config32(location, 0x0c) >> 16) as u8;
            let functions = if header & 0x80 != 0 { 8 } else { 1 };
            for function in 0..functions {
                let candidate = PciLocation {
                    function,
                    ..location
                };
                let id = read_config32(candidate, 0);
                if id as u16 == PCI_VENDOR_VIRTIO && (id >> 16) as u16 == PCI_DEVICE_GPU {
                    return Some(candidate);
                }
            }
        }
    }
    None
}

fn bar_base(location: PciLocation, bar: u8) -> Option<u64> {
    if bar >= 6 {
        return None;
    }
    let low = read_config32(location, 0x10 + bar * 4);
    if low & 1 != 0 {
        return None;
    }
    let kind = (low >> 1) & 3;
    let address = (low & !0xf) as u64;
    if kind == 2 {
        if bar >= 5 {
            return None;
        }
        Some(address | ((read_config32(location, 0x10 + (bar + 1) * 4) as u64) << 32))
    } else if kind == 0 {
        Some(address)
    } else {
        None
    }
}

fn parse_caps(location: PciLocation) -> Option<VirtioCaps> {
    if read_config32(location, 0x04) & (1 << 20) == 0 {
        return None;
    }
    let mut pointer = (read_config32(location, 0x34) & 0xfc) as u8;
    let mut caps = VirtioCaps::default();
    let mut visited = 0usize;
    while pointer >= 0x40 && visited < 48 {
        visited += 1;
        let header = read_config32(location, pointer);
        let id = header as u8;
        let next = (header >> 8) as u8;
        let length = (header >> 16) as u8;
        if id == PCI_CAP_VENDOR && length >= 16 {
            let cap_type = (header >> 24) as u8;
            let bar = read_config32(location, pointer + 4) as u8;
            let offset = read_config32(location, pointer + 8);
            let region_length = read_config32(location, pointer + 12);
            let base = bar_base(location, bar)?;
            let region = PciRegion {
                base,
                offset,
                length: region_length,
            };
            match cap_type {
                CAP_COMMON => caps.common = region,
                CAP_DEVICE => caps.device = region,
                CAP_NOTIFY if length >= 20 => {
                    caps.notify = region;
                    caps.notify_multiplier = read_config32(location, pointer + 16);
                }
                _ => {}
            }
        }
        if next == 0 {
            break;
        }
        pointer = next & 0xfc;
    }
    if caps.common.base == 0
        || caps.common.length < 56
        || caps.notify.base == 0
        || caps.notify_multiplier == 0
        || caps.device.base == 0
        || caps.device.length < 12
    {
        None
    } else {
        Some(caps)
    }
}

fn map_region(region: PciRegion) -> Option<*mut u8> {
    region.offset.checked_add(region.length)?;
    if region.base == 0 || region.length == 0 {
        return None;
    }
    Some(crate::vmm::map_mmio(
        region.base.checked_add(region.offset as u64)?,
        region.length as u64,
    ))
}

fn allocate_dma(pages: usize) -> Option<DmaMemory> {
    let physical = crate::pmm::alloc_contiguous(pages)?;
    let virtual_address = crate::pmm::phys_to_virt(physical);
    unsafe { ptr::write_bytes(virtual_address, 0, pages.checked_mul(PAGE_SIZE)?) };
    Some(DmaMemory {
        physical,
        virtual_address,
    })
}

fn align_up(value: usize, alignment: usize) -> Option<usize> {
    value
        .checked_add(alignment - 1)
        .map(|v| v & !(alignment - 1))
}

impl VirtQueue {
    fn new() -> Option<Self> {
        let desc_bytes = QUEUE_SIZE as usize * core::mem::size_of::<Descriptor>();
        let avail_offset = desc_bytes;
        let avail_bytes = 4 + QUEUE_SIZE as usize * 2;
        let used_offset = align_up(avail_offset.checked_add(avail_bytes)?, PAGE_SIZE)?;
        let used_bytes = 4 + QUEUE_SIZE as usize * 8;
        let pages = align_up(used_offset.checked_add(used_bytes)?, PAGE_SIZE)? / PAGE_SIZE;
        let memory = allocate_dma(pages)?;
        Some(Self {
            memory,
            size: QUEUE_SIZE,
            avail_offset,
            used_offset,
            avail_index: 0,
            used_index: 0,
        })
    }

    unsafe fn descriptor(&mut self, index: usize) -> *mut Descriptor {
        self.memory
            .virtual_address
            .add(index * core::mem::size_of::<Descriptor>())
            .cast()
    }

    fn avail_index_ptr(&mut self) -> *mut u16 {
        unsafe {
            self.memory
                .virtual_address
                .add(self.avail_offset + 2)
                .cast()
        }
    }

    fn avail_ring_ptr(&mut self, index: usize) -> *mut u16 {
        unsafe {
            self.memory
                .virtual_address
                .add(self.avail_offset + 4 + index * 2)
                .cast()
        }
    }

    fn used_index_ptr(&mut self) -> *mut u16 {
        unsafe { self.memory.virtual_address.add(self.used_offset + 2).cast() }
    }
}

impl Driver {
    unsafe fn set_status(&self, value: u8) {
        ptr::write_volatile(self.common.add(20), value);
    }

    unsafe fn read_status(&self) -> u8 {
        ptr::read_volatile(self.common.add(20))
    }

    fn init_queue(&mut self) -> bool {
        unsafe {
            ptr::write_volatile(self.common.add(22).cast::<u16>(), 0);
            let max_size = ptr::read_volatile(self.common.add(24).cast::<u16>());
            if max_size < 2 {
                return false;
            }
            let size = max_size.min(QUEUE_SIZE);
            if !size.is_power_of_two() {
                return false;
            }
            self.queue.size = size;
            ptr::write_volatile(self.common.add(24).cast::<u16>(), size);
            let notify_offset = ptr::read_volatile(self.common.add(30).cast::<u16>());
            let notify_byte_offset = (notify_offset as u64) * self.notify_multiplier as u64;
            if notify_byte_offset + 2 > self.notify_length as u64 {
                return false;
            }
            self.notify_offset = notify_offset;
            ptr::write_volatile(
                self.common.add(32).cast::<u64>(),
                self.queue.memory.physical,
            );
            ptr::write_volatile(
                self.common.add(40).cast::<u64>(),
                self.queue.memory.physical + self.queue.avail_offset as u64,
            );
            ptr::write_volatile(
                self.common.add(48).cast::<u64>(),
                self.queue.memory.physical + self.queue.used_offset as u64,
            );
            fence(Ordering::SeqCst);
            ptr::write_volatile(self.common.add(28).cast::<u16>(), 1);
            ptr::read_volatile(self.common.add(28).cast::<u16>()) == 1
        }
    }

    fn submit(
        &mut self,
        request: &[u8],
        expected_response: u32,
        response_size: usize,
    ) -> Option<usize> {
        if request.len() > RESPONSE_OFFSET
            || response_size > RESPONSE_CAPACITY
            || request.len() < 24
        {
            return None;
        }
        unsafe {
            let cmd = self.command.virtual_address;
            ptr::write_bytes(cmd, 0, COMMAND_PAGE_BYTES);
            ptr::copy_nonoverlapping(request.as_ptr(), cmd.add(REQUEST_OFFSET), request.len());

            let desc0 = self.queue.descriptor(0);
            ptr::write_volatile(
                desc0,
                Descriptor {
                    address: self.command.physical + REQUEST_OFFSET as u64,
                    length: request.len() as u32,
                    flags: DESC_NEXT,
                    next: 1,
                },
            );
            let desc1 = self.queue.descriptor(1);
            ptr::write_volatile(
                desc1,
                Descriptor {
                    address: self.command.physical + RESPONSE_OFFSET as u64,
                    length: response_size as u32,
                    flags: DESC_WRITE,
                    next: 0,
                },
            );

            let ring_slot = self.queue.avail_index as usize % self.queue.size as usize;
            ptr::write_volatile(self.queue.avail_ring_ptr(ring_slot), 0);
            self.queue.avail_index = self.queue.avail_index.wrapping_add(1);
            fence(Ordering::Release);
            ptr::write_volatile(self.queue.avail_index_ptr(), self.queue.avail_index);
            fence(Ordering::SeqCst);

            let notify_addr = self
                .notify
                .add(self.notify_offset as usize * self.notify_multiplier as usize);
            ptr::write_volatile(notify_addr.cast::<u16>(), 0);

            let mut spins = 20_000_000usize;
            while ptr::read_volatile(self.queue.used_index_ptr()) == self.queue.used_index {
                if spins == 0 {
                    return None;
                }
                spins -= 1;
                core::hint::spin_loop();
            }
            fence(Ordering::Acquire);
            let used_slot = self.queue.used_index as usize % self.queue.size as usize;
            let used_len = ptr::read_volatile(
                self.queue
                    .memory
                    .virtual_address
                    .add(self.queue.used_offset + 4 + used_slot * 8 + 4)
                    .cast::<u32>(),
            )
            .to_le() as usize;
            self.queue.used_index = self.queue.used_index.wrapping_add(1);
            let response = cmd.add(RESPONSE_OFFSET);
            let response_type = ptr::read_unaligned(response.cast::<u32>()).to_le();
            if response_type != expected_response {
                return None;
            }
            Some(used_len)
        }
    }

    fn command_header(buffer: &mut [u8], command: u32) {
        buffer[..24].fill(0);
        buffer[0..4].copy_from_slice(&command.to_le_bytes());
    }

    fn get_display_info(&mut self) -> Option<(u32, u32)> {
        let mut request = [0u8; 24];
        Self::command_header(&mut request, GPU_CMD_GET_DISPLAY_INFO);
        let response_len = self.submit(&request, GPU_RESP_OK_DISPLAY_INFO, RESPONSE_CAPACITY)?;
        if response_len < 48 {
            return None;
        }
        unsafe {
            let response = self.command.virtual_address.add(RESPONSE_OFFSET);
            let enabled = ptr::read_unaligned(response.add(24 + 16).cast::<u32>()).to_le();
            let width = ptr::read_unaligned(response.add(24 + 8).cast::<u32>()).to_le();
            let height = ptr::read_unaligned(response.add(24 + 12).cast::<u32>()).to_le();
            if enabled == 0 || width == 0 || height == 0 {
                return None;
            }
            Some((width, height))
        }
    }

    fn setup_resource(&mut self) -> bool {
        let mut request = [0u8; 48];
        Self::command_header(&mut request, GPU_CMD_RESOURCE_CREATE_2D);
        request[24..28].copy_from_slice(&RESOURCE_ID.to_le_bytes());
        request[28..32].copy_from_slice(&PIXEL_FORMAT_B8G8R8X8_UNORM.to_le_bytes());
        request[32..36].copy_from_slice(&self.width.to_le_bytes());
        request[36..40].copy_from_slice(&self.height.to_le_bytes());
        if self
            .submit(&request[..40], GPU_RESP_OK_NODATA, 24)
            .is_none()
        {
            return false;
        }

        // One contiguous backing entry keeps attach and transfer simple.
        let mut attach = [0u8; 48];
        Self::command_header(&mut attach, GPU_CMD_RESOURCE_ATTACH_BACKING);
        attach[24..28].copy_from_slice(&RESOURCE_ID.to_le_bytes());
        attach[28..32].copy_from_slice(&1u32.to_le_bytes());
        attach[32..40].copy_from_slice(&self.framebuffer.physical.to_le_bytes());
        let bytes = self.pitch * self.height as usize;
        let Ok(bytes) = u32::try_from(bytes) else {
            return false;
        };
        attach[40..44].copy_from_slice(&bytes.to_le_bytes());
        if self.submit(&attach, GPU_RESP_OK_NODATA, 24).is_none() {
            return false;
        }

        let mut scanout = [0u8; 48];
        Self::command_header(&mut scanout, GPU_CMD_SET_SCANOUT);
        scanout[32..36].copy_from_slice(&self.width.to_le_bytes());
        scanout[36..40].copy_from_slice(&self.height.to_le_bytes());
        scanout[40..44].copy_from_slice(&0u32.to_le_bytes());
        scanout[44..48].copy_from_slice(&RESOURCE_ID.to_le_bytes());
        self.submit(&scanout, GPU_RESP_OK_NODATA, 24).is_some()
    }

    fn present(&mut self, x: usize, y: usize, width: usize, height: usize) -> bool {
        let x = x.min(self.width as usize);
        let y = y.min(self.height as usize);
        let width = width.min(self.width as usize - x);
        let height = height.min(self.height as usize - y);
        if width == 0 || height == 0 {
            return true;
        }

        let mut transfer = [0u8; 56];
        Self::command_header(&mut transfer, GPU_CMD_TRANSFER_TO_HOST_2D);
        transfer[24..28].copy_from_slice(&(x as u32).to_le_bytes());
        transfer[28..32].copy_from_slice(&(y as u32).to_le_bytes());
        transfer[32..36].copy_from_slice(&(width as u32).to_le_bytes());
        transfer[36..40].copy_from_slice(&(height as u32).to_le_bytes());
        transfer[48..52].copy_from_slice(&RESOURCE_ID.to_le_bytes());
        if self.submit(&transfer, GPU_RESP_OK_NODATA, 24).is_none() {
            return false;
        }

        let mut flush = [0u8; 48];
        Self::command_header(&mut flush, GPU_CMD_RESOURCE_FLUSH);
        flush[24..28].copy_from_slice(&(x as u32).to_le_bytes());
        flush[28..32].copy_from_slice(&(y as u32).to_le_bytes());
        flush[32..36].copy_from_slice(&(width as u32).to_le_bytes());
        flush[36..40].copy_from_slice(&(height as u32).to_le_bytes());
        flush[40..44].copy_from_slice(&RESOURCE_ID.to_le_bytes());
        self.submit(&flush, GPU_RESP_OK_NODATA, 24).is_some()
    }
}

unsafe fn initialize(location: PciLocation, caps: VirtioCaps) -> Option<Driver> {
    let command = read_config32(location, 0x04) as u16 | 0x0006;
    write_config16(location, 0x04, command);

    let common = map_region(caps.common)?;
    let notify = map_region(caps.notify)?;
    let device = map_region(caps.device)?;

    let mut driver = Driver {
        queue: VirtQueue::new()?,
        command: allocate_dma(1)?,
        framebuffer: DmaMemory {
            physical: 0,
            virtual_address: ptr::null_mut(),
        },
        width: 0,
        height: 0,
        pitch: 0,
        common,
        notify,
        notify_multiplier: caps.notify_multiplier,
        notify_length: caps.notify.length,
        notify_offset: 0,
    };

    driver.set_status(0);
    driver.set_status(STATUS_ACKNOWLEDGE);
    driver.set_status(STATUS_ACKNOWLEDGE | STATUS_DRIVER);
    ptr::write_volatile(common.add(0), 0);
    let features_low = ptr::read_volatile(common.add(4).cast::<u32>()) as u64;
    ptr::write_volatile(common.add(0), 1);
    let features_high = ptr::read_volatile(common.add(4).cast::<u32>()) as u64;
    let features = features_low | (features_high << 32);
    if features & VIRTIO_F_VERSION_1 == 0 {
        driver.set_status(STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FAILED);
        return None;
    }
    ptr::write_volatile(common.add(8), 0);
    ptr::write_volatile(common.add(12).cast::<u32>(), 0);
    ptr::write_volatile(common.add(8), 1);
    ptr::write_volatile(common.add(12).cast::<u32>(), 1);
    fence(Ordering::SeqCst);
    driver.set_status(STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK);
    if driver.read_status() & STATUS_FEATURES_OK == 0 || !driver.init_queue() {
        driver.set_status(STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FAILED);
        return None;
    }
    driver.set_status(STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK | STATUS_DRIVER_OK);

    let (width, height) = driver.get_display_info().unwrap_or((1280, 800));
    if width > MAX_WIDTH || height > MAX_HEIGHT || width == 0 || height == 0 {
        return None;
    }
    driver.width = width;
    driver.height = height;
    driver.pitch = width as usize * 4;
    let bytes = driver.pitch.checked_mul(height as usize)?;
    let pages = align_up(bytes, PAGE_SIZE)? / PAGE_SIZE;
    driver.framebuffer = allocate_dma(pages)?;
    if !driver.setup_resource() {
        return None;
    }

    // Keep the GPU configuration mapping alive by reading its scanout count;
    // this also verifies the advertised device configuration was accessible.
    if ptr::read_volatile(device.add(8).cast::<u32>()) == 0 {
        return None;
    }
    if !driver.present(0, 0, width as usize, height as usize) {
        return None;
    }
    Some(driver)
}

/// Find and initialize a modern VirtIO GPU. Failure leaves the Limine
/// framebuffer active, so unsupported physical machines still boot normally.
pub fn init() {
    let Some(location) = find_gpu() else {
        crate::log_ok!(
            "VirtIO GPU",
            "Init",
            "No VirtIO GPU found; keeping boot framebuffer"
        );
        return;
    };
    let Some(caps) = parse_caps(location) else {
        crate::log_fail!(
            "VirtIO GPU",
            "Init",
            "GPU found but required PCI capabilities are unavailable"
        );
        return;
    };
    let Some(driver) = (unsafe { initialize(location, caps) }) else {
        crate::log_fail!(
            "VirtIO GPU",
            "Init",
            "GPU initialization failed; keeping boot framebuffer"
        );
        return;
    };

    let addr = driver.framebuffer.virtual_address;
    let width = driver.width as u64;
    let height = driver.height as u64;
    let pitch = driver.pitch as u64;
    *GPU.lock() = Some(driver);
    crate::framebuffer::use_gpu_backing(addr, width, height, pitch);
    GPU_ACTIVE.store(true, Ordering::Release);
    crate::log_ok!(
        "VirtIO GPU",
        "Init",
        "2D display ready ({}x{}, dirty-rectangle updates)",
        width,
        height
    );
}

/// Submit a dirty rectangle after the console has copied its shadow pixels
/// into the GPU resource backing. Returns false when no VirtIO backend exists.
pub fn present_rect(x: usize, y: usize, width: usize, height: usize) -> bool {
    if !GPU_ACTIVE.load(Ordering::Acquire) {
        return false;
    }
    let mut guard = GPU.lock();
    let Some(driver) = guard.as_mut() else {
        return false;
    };
    let _ = driver.present(x, y, width, height);
    true
}
