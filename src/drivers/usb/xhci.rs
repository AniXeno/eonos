//! xHCI (Extensible Host Controller Interface) driver, keyboard-only.
//!
//! Deliberately the simplest thing that actually works on real
//! hardware, not a general USB stack:
//!
//! - **Polling, not interrupts.** This kernel's `idt`/`pic` only wire up
//!   the legacy 8259 (`drivers::ps2` uses it for IRQ1); there's no
//!   IOAPIC or MSI/MSI-X support to route an xHCI controller's
//!   interrupt through. Real xHCI hardware expects MSI-X and often
//!   doesn't route cleanly through legacy INTx at all, so guessing at
//!   PIC wiring here would be more likely to silently not work than to
//!   help. Every wait in this driver (command completion, port reset,
//!   new keyboard reports) is instead a bounded spin against the event
//!   ring / port status registers, driven by whoever calls `poll()`
//!   (currently `syscall::next_byte`, once per line-editor iteration --
//!   see that file). This is slower to notice a keypress than an
//!   interrupt would be, bounded by how often the caller polls, but at
//!   typing speed the difference isn't perceptible.
//! - **One device, boot protocol only.** This driver finds the first
//!   port with something HID-keyboard-shaped plugged in, addresses it,
//!   flips it into the USB HID *boot protocol* (a fixed 8-byte report:
//!   modifier byte, reserved byte, 6 simultaneous keycodes -- every USB
//!   keyboard is required to support this even if its normal reports
//!   are a different shape), and stops there. No hubs, no multiple
//!   keyboards, no mice, no report-descriptor parsing.
//! - **No hot-plug.** Ports are scanned once, at `init()`. Plugging a
//!   keyboard in after boot won't be noticed.
//!
//! Everything here is built around a single `Controller` -- multiple
//! xHCI controllers in one machine aren't supported, matching
//! `usb::init()` only probing the first xHCI-class device it finds.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering};

use crate::sync::IrqMutex;
use crate::{log_fail, log_ok};
use crate::{pmm, vmm};

use super::pci::UsbController;

// ---------------------------------------------------------------------
// Register layout (xHCI spec section 5). All offsets are from the BAR
// base; capability register length (`cap_length`) gives the offset
// where the operational registers begin.
// ---------------------------------------------------------------------

#[repr(C)]
struct CapRegs {
    cap_length: u8,
    _reserved: u8,
    hci_version: u16,
    hcs_params1: u32,
    hcs_params2: u32,
    hcs_params3: u32,
    hcc_params1: u32,
    dboff: u32,
    rtsoff: u32,
    hcc_params2: u32,
}

/// Operational registers (xHCI 5.4). Only the fields this driver
/// actually touches -- there are more (device notification, ...) a
/// fuller driver would need.
#[repr(C)]
struct OpRegs {
    usbcmd: u32,
    usbsts: u32,
    pagesize: u32,
    _rsvd1: [u32; 2],
    dnctrl: u32,
    crcr_lo: u32,
    crcr_hi: u32,
    _rsvd2: [u32; 4],
    dcbaap_lo: u32,
    dcbaap_hi: u32,
    config: u32,
    // Port register sets (one per port, 4 dwords each) follow at
    // offset 0x400 from the *capability* base, not immediately after
    // this struct -- xHCI leaves a gap here for extended-capabilities
    // pointers this driver doesn't use, so port registers are addressed
    // separately (see `port_regs`) rather than as a trailing array.
}

const USBCMD_RUN: u32 = 1 << 0;
const USBCMD_HCRST: u32 = 1 << 1;
// USBCMD_INTE (bit 2) deliberately never set -- see module docs.

const USBSTS_HCH: u32 = 1 << 0; // host controller halted
const USBSTS_CNR: u32 = 1 << 11; // controller not ready

/// One port's register set (xHCI 5.4.8): 4 dwords starting at
/// capability-base + 0x400 + 0x10 * (port - 1) (ports are 1-indexed in
/// the spec).
#[repr(C)]
struct PortRegs {
    portsc: u32,
    portpmsc: u32,
    portli: u32,
    porthlpmc: u32,
}

const PORTSC_CCS: u32 = 1 << 0; // current connect status
const PORTSC_PED: u32 = 1 << 1; // port enabled/disabled
const PORTSC_PR: u32 = 1 << 4; // port reset
const PORTSC_SPEED_MASK: u32 = 0xF << 10;
const PORTSC_PRC: u32 = 1 << 21; // port reset change (write-1-to-clear)
const PORTSC_CSC: u32 = 1 << 17; // connect status change (write-1-to-clear)
/// Bits that are write-1-to-clear status bits, not real state -- when
/// read-modify-writing PORTSC to change something else (like setting
/// PR), these must be masked out of the value we write back, or we'd
/// accidentally clear status the controller hasn't told us about yet.
const PORTSC_RW1C_MASK: u32 = (1 << 17) | (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 22) | (1 << 23);

/// Runtime registers (xHCI 5.5): interrupter 0's registers are what
/// this driver reads to find and dequeue event TRBs, even though
/// interrupts themselves stay disabled.
#[repr(C)]
struct RuntimeRegs {
    mfindex: u32,
    _rsvd: [u32; 7],
    ir0_iman: u32,
    ir0_imod: u32,
    ir0_erstsz: u32,
    _rsvd2: u32,
    ir0_erstba_lo: u32,
    ir0_erstba_hi: u32,
    ir0_erdp_lo: u32,
    ir0_erdp_hi: u32,
}

const ERDP_BUSY: u32 = 1 << 3; // event handler busy (write-1-to-clear)

// ---------------------------------------------------------------------
// TRBs (Transfer Request Blocks) -- the 16-byte structure every ring
// (command, event, transfer) is built from (xHCI 4.11).
// ---------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy)]
struct Trb {
    parameter: u64,
    status: u32,
    control: u32,
}

const TRB_CYCLE: u32 = 1 << 0;
const TRB_TOGGLE_CYCLE: u32 = 1 << 1; // link TRBs only
const TRB_IOC: u32 = 1 << 5; // interrupt on completion (harmless with INTE off; we poll instead)

fn trb_type(control: u32) -> u32 {
    (control >> 10) & 0x3F
}
fn make_type(t: u32) -> u32 {
    (t & 0x3F) << 10
}

const TRB_TYPE_NORMAL: u32 = 1;
const TRB_TYPE_SETUP_STAGE: u32 = 2;
const TRB_TYPE_DATA_STAGE: u32 = 3;
const TRB_TYPE_STATUS_STAGE: u32 = 4;
const TRB_TYPE_LINK: u32 = 6;
const TRB_TYPE_ENABLE_SLOT: u32 = 9;
const TRB_TYPE_ADDRESS_DEVICE: u32 = 11;
const TRB_TYPE_CONFIGURE_ENDPOINT: u32 = 12;
const TRB_TYPE_TRANSFER_EVENT: u32 = 32;
const TRB_TYPE_COMMAND_COMPLETION: u32 = 33;

const COMPLETION_SUCCESS: u32 = 1;
const COMPLETION_SHORT_PACKET: u32 = 13;

/// Entries per ring. 16 is plenty for command/event traffic this small
/// (a handful of setup commands at init, then one interrupt-IN
/// completion at a time); real drivers often use 256, but there is no
/// benefit to that here and it would just be more zeroed memory to
/// scan. One slot is reserved for the trailing Link TRB that wraps the
/// ring back to the start.
const RING_SIZE: usize = 16;

/// A single-page ring buffer of TRBs used for the command ring, the
/// event ring, and each endpoint's transfer ring. Software's enqueue
/// pointer and the hardware/software "cycle bit" convention (xHCI
/// 4.9.2) are tracked together since every ring user needs both.
struct Ring {
    trbs: *mut Trb,
    phys: u64,
    enqueue: usize,
    /// Which cycle-bit value currently marks a TRB as valid. Starts at
    /// 1 (a freshly zeroed ring has cycle bit 0 in every slot, so the
    /// producer must write 1 to mark real entries) and flips every
    /// time the enqueue pointer wraps via the Link TRB.
    cycle: u32,
}

impl Ring {
    fn new() -> Option<Ring> {
        let phys = pmm::alloc_frame_zeroed()?;
        let trbs = pmm::phys_to_virt(phys) as *mut Trb;
        // Install the Link TRB in the last slot up front: it never
        // moves, and always points back to slot 0 with TOGGLE_CYCLE
        // set so hardware flips its notion of the cycle bit when it
        // follows the link, matching software's own flip below.
        unsafe {
            let link = trbs.add(RING_SIZE - 1);
            core::ptr::write_volatile(
                link,
                Trb { parameter: phys, status: 0, control: make_type(TRB_TYPE_LINK) | TRB_TOGGLE_CYCLE | TRB_CYCLE },
            );
        }
        Some(Ring { trbs, phys, enqueue: 0, cycle: 1 })
    }

    /// Write one TRB (cycle bit filled in automatically) and advance,
    /// following the Link TRB and flipping `cycle` when the ring wraps.
    /// Returns the physical address the TRB was written at, which
    /// callers needing to correlate a later completion event against
    /// this specific submission compare against.
    unsafe fn push(&mut self, mut trb: Trb) -> u64 {
        trb.control = (trb.control & !TRB_CYCLE) | self.cycle;
        let slot_phys = self.phys + (self.enqueue as u64) * 16;
        core::ptr::write_volatile(self.trbs.add(self.enqueue), trb);
        self.enqueue += 1;
        if self.enqueue == RING_SIZE - 1 {
            // At the Link TRB: refresh its cycle bit to match this
            // wrap before jumping back to slot 0.
            let link = self.trbs.add(RING_SIZE - 1);
            let mut existing = core::ptr::read_volatile(link);
            existing.control = (existing.control & !TRB_CYCLE) | self.cycle;
            core::ptr::write_volatile(link, existing);
            self.enqueue = 0;
            self.cycle ^= 1;
        }
        slot_phys
    }
}

// ---------------------------------------------------------------------
// Device Context Base Address Array and Input/Device Contexts (xHCI 6.2)
// ---------------------------------------------------------------------

/// One Slot Context or Endpoint Context: 32 bytes (xHCI 6.2.1 / 6.2.2),
/// assuming the controller's HCCPARAMS1.CSZ bit is 0 (32-byte
/// contexts) -- checked at init and aborted otherwise rather than
/// silently laying out structures the controller would misinterpret.
#[repr(C)]
#[derive(Clone, Copy)]
struct SlotOrEndpointContext {
    dwords: [u32; 8],
}
const CONTEXT_SIZE: usize = core::mem::size_of::<SlotOrEndpointContext>();

/// Input Control Context plus a Slot Context and endpoint contexts
/// (xHCI 6.2.5) -- what ADDRESS_DEVICE and CONFIGURE_ENDPOINT commands
/// point at. Device Context Index (DCI) 1 is always EP0 (the control
/// endpoint every device has); DCI 2 and up are EP1 OUT, EP1 IN, EP2
/// OUT, ... in order. This driver only ever populates DCI 1 and DCI 3
/// (EP1 IN, the interrupt endpoint boot-protocol reports arrive on),
/// but the layout reserves the full DCI 2-31 range regardless, since
/// the Slot Context's own "context entries" field -- not how much of
/// this struct is populated -- is what tells the controller how many
/// endpoint contexts are actually meaningful.
#[repr(C)]
struct InputContext {
    control: SlotOrEndpointContext,
    slot: SlotOrEndpointContext,
    ep0: SlotOrEndpointContext,
    endpoints: [SlotOrEndpointContext; 30], // DCI 2 (endpoints[0]) through DCI 31 (endpoints[29])
}

fn slot_context_mut(ctx_phys: u64) -> *mut SlotOrEndpointContext {
    unsafe { core::ptr::addr_of_mut!((*(pmm::phys_to_virt(ctx_phys) as *mut InputContext)).slot) }
}
fn ep_context_mut(ctx_phys: u64, ep_dci: usize) -> *mut SlotOrEndpointContext {
    // DCI 1 (EP0) has its own dedicated field; DCI N for N >= 2 is
    // `endpoints[N - 2]`.
    unsafe {
        let input = pmm::phys_to_virt(ctx_phys) as *mut InputContext;
        if ep_dci == 1 {
            core::ptr::addr_of_mut!((*input).ep0)
        } else {
            core::ptr::addr_of_mut!((*input).endpoints[ep_dci - 2])
        }
    }
}

// ---------------------------------------------------------------------
// USB control-transfer setup packet (USB 2.0 spec 9.3), used verbatim
// as a Setup Stage TRB's parameter field.
// ---------------------------------------------------------------------

fn setup_packet(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> u64 {
    (request_type as u64) | ((request as u64) << 8) | ((value as u64) << 16) | ((index as u64) << 32) | ((length as u64) << 48)
}

const REQ_TYPE_STD_DEVICE_IN: u8 = 0x80;
const REQ_TYPE_STD_DEVICE_OUT: u8 = 0x00;
const REQ_TYPE_CLASS_IFACE_OUT: u8 = 0x21;

const REQ_GET_DESCRIPTOR: u8 = 6;
const REQ_SET_CONFIGURATION: u8 = 9;
const HID_REQ_SET_PROTOCOL: u8 = 0x0B;
const HID_REQ_SET_IDLE: u8 = 0x0A;

const DESC_TYPE_DEVICE: u16 = 1 << 8;
const HID_PROTOCOL_BOOT: u16 = 0;

// ---------------------------------------------------------------------
// The controller itself.
// ---------------------------------------------------------------------

struct Controller {
    cap: *const CapRegs,
    op: *mut OpRegs,
    rt: *mut RuntimeRegs,
    db: *mut u32, // doorbell array, one u32 per slot (0 = command ring doorbell)
    max_ports: u8,

    cmd_ring: Ring,
    event_ring: Ring,
    /// Software's copy of which cycle value currently marks a *valid*
    /// (not-yet-consumed) event TRB. Flips every time the event ring's
    /// dequeue pointer wraps, mirroring the producer side in `Ring`.
    event_cycle: u32,
    event_dequeue: usize,

    dcbaa_phys: u64,
    /// Physical address of the one device's Input Context, and the
    /// separate Device Context the controller writes live state back
    /// into (they must be different pages -- xHCI 4.2 -- with the
    /// DCBAA slot entry pointing at the *output* Device Context, never
    /// the Input Context used only for commands).
    input_ctx_phys: u64,
    output_ctx_phys: u64,
    ep0_ring_phys: u64,

    slot_id: u8,
    /// Interrupt-IN endpoint's transfer ring, once a keyboard is
    /// configured. `None` until a keyboard is actually found and set
    /// up in `try_setup_keyboard`.
    keyboard_ring: Option<Ring>,
    /// One 8-byte boot-protocol report buffer, reused for every polled
    /// transfer -- the controller DMAs into this on each successful
    /// interrupt-IN completion.
    report_phys: u64,
}

// Not Sync/Send by default (raw pointers); access is always through the
// single `CONTROLLER` behind an `IrqMutex`, same pattern as every other
// shared mutable driver state in this kernel (`ps2::ASCII_QUEUE`, etc).
unsafe impl Send for Controller {}

static CONTROLLER: IrqMutex<Option<Controller>> = IrqMutex::new(None);
static KEYBOARD_READY: AtomicBool = AtomicBool::new(false);
/// Tracks Caps Lock across polls the same way `drivers::ps2` does, so
/// USB and PS/2 keyboards produce identically-cased ASCII into whatever
/// reads `try_read_byte`.
static CAPS_LOCK: AtomicBool = AtomicBool::new(false);
/// Which of the up-to-6 simultaneous keys in the previous report were
/// already down, so a report only produces a keypress for keys that are
/// *newly* pressed. Without this, holding a key down would spam the
/// queue with one character per poll instead of one per press -- USB
/// boot reports give no separate press/release events the way PS/2
/// scancodes do; a release is just "this keycode is no longer listed".
static PREV_KEYS: IrqMutex<[u8; 6]> = IrqMutex::new([0; 6]);

fn reg_read32(ptr: *const u32) -> u32 {
    unsafe { core::ptr::read_volatile(ptr) }
}
fn reg_write32(ptr: *mut u32, val: u32) {
    unsafe { core::ptr::write_volatile(ptr, val) }
}
fn reg_write64_split(lo: *mut u32, hi: *mut u32, val: u64) {
    reg_write32(lo, val as u32);
    reg_write32(hi, (val >> 32) as u32);
}

/// Spin up to roughly `iters` times waiting for `cond` to become true,
/// yielding to other threads between checks so a slow or absent
/// controller can't hang the whole kernel. Every hardware wait in this
/// driver goes through this, mirroring `ps2.rs`'s bounded spins --
/// nothing here trusts a device to respond in any particular time.
fn spin_until(iters: u32, mut cond: impl FnMut() -> bool) -> bool {
    for _ in 0..iters {
        if cond() {
            return true;
        }
        crate::scheduler::yield_now();
    }
    false
}

/// Entry point from `usb::init()`. Brings the controller up far enough
/// to enumerate one keyboard, if one is plugged in. Every failure path
/// logs and returns rather than panicking -- absent, non-compliant, or
/// already-owned-by-firmware hardware is expected on some machines and
/// must not take down boot.
/// Extended Capability ID 1: USB Legacy Support (xHCI 7.1.1). Byte
/// layout of its first dword: capability ID (bits 0:7), next-capability
/// pointer in dwords (bits 8:15, 0 = end of list), BIOS-owned semaphore
/// (bit 16), OS-owned semaphore (bit 24).
const XECP_ID_USB_LEGACY: u32 = 1;
const USBLEGSUP_BIOS_OWNED: u32 = 1 << 16;
const USBLEGSUP_OS_OWNED: u32 = 1 << 24;

/// Walk the xHCI Extended Capabilities linked list looking for USB
/// Legacy Support, and if present, ask firmware to release ownership of
/// the controller.
///
/// On real hardware the BIOS/SMM can keep the xHCI controller for its
/// own USB keyboard emulation (so you can use a USB keyboard in the
/// boot menu, say) until the OS explicitly claims it here. Skipping
/// this is a common way for a driver to "work" in QEMU -- which has no
/// such firmware ownership to contend with -- while doing nothing at
/// all on real hardware, or worse, fighting the BIOS's SMI handler for
/// the controller's registers. `xecp_offset_dwords` is
/// HCCPARAMS1[31:16] from `probe`, a dword offset from `base` to the
/// first capability in the list.
fn request_bios_handoff(base: *mut u8, xecp_offset_dwords: usize) {
    let mut offset = xecp_offset_dwords;
    // Bounded rather than a `loop`: a corrupt or malicious-looking
    // capability list (an offset of 0 that isn't really "end of list",
    // or a cycle) must not hang boot. The xHCI spec doesn't bound the
    // list's length, but no real controller has anywhere near this
    // many extended capabilities.
    for _ in 0..256 {
        if offset == 0 {
            return; // end of list, no USB Legacy Support capability present
        }
        let cap_ptr = unsafe { (base as *mut u32).add(offset) };
        let cap_dword = reg_read32(cap_ptr);
        let cap_id = cap_dword & 0xFF;
        let next = (cap_dword >> 8) & 0xFF;

        if cap_id == XECP_ID_USB_LEGACY {
            if cap_dword & USBLEGSUP_BIOS_OWNED == 0 {
                log_ok!("USB", "xHCI", "Controller already OS-owned, no handoff needed");
                return;
            }
            log_ok!("USB", "xHCI", "Requesting BIOS-to-OS handoff");
            reg_write32(cap_ptr, cap_dword | USBLEGSUP_OS_OWNED);
            let handed_over = spin_until(500_000, || reg_read32(cap_ptr) & USBLEGSUP_BIOS_OWNED == 0);
            if !handed_over {
                // Some firmware never clears this bit despite otherwise
                // behaving correctly; log it and proceed anyway rather
                // than refusing to drive a controller that will most
                // likely work fine regardless -- an outright hang here
                // would take a merely-noncompliant BIOS and turn it
                // into "USB keyboard never works on this machine".
                log_fail!(
                    "USB",
                    "xHCI",
                    "BIOS did not release ownership in time -- proceeding anyway"
                );
            } else {
                log_ok!("USB", "xHCI", "BIOS released ownership");
            }
            return;
        }

        if next == 0 {
            return;
        }
        offset += next as usize;
    }
}

pub fn probe(controller: &UsbController) {
    if controller.mmio_base == 0 {
        log_fail!("USB", "xHCI", "Controller has no usable memory BAR -- cannot probe");
        return;
    }

    // Map generously past the capability/operational registers into
    // where the runtime and doorbell arrays live (`dboff`/`rtsoff` are
    // read below, but both are documented to sit within the first bank
    // of registers on essentially all implementations); 64 KiB covers
    // every real controller's register footprint with room to spare.
    let base = vmm::map_mmio(controller.mmio_base, 0x10000);
    let cap = base as *const CapRegs;

    let hci_version = reg_read32(base as *const u32) >> 16;
    let hcs_params1 = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*cap).hcs_params1)) };
    let hcc_params1 = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*cap).hcc_params1)) };
    let dboff = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*cap).dboff)) } & !0x3;
    let rtsoff = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*cap).rtsoff)) } & !0x1F;
    let cap_length = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*cap).cap_length)) };

    let max_slots = (hcs_params1 & 0xFF) as u8;
    let max_ports = ((hcs_params1 >> 24) & 0xFF) as u8;
    let context_size_64 = hcc_params1 & (1 << 2) != 0; // CSZ bit

    log_ok!(
        "USB",
        "xHCI",
        "Controller v{}.{}, {} slot(s), {} port(s), {}-byte contexts",
        (hci_version >> 8) & 0xFF,
        hci_version & 0xFF,
        max_slots,
        max_ports,
        if context_size_64 { 64 } else { 32 }
    );

    if context_size_64 {
        // A fuller driver would just use 64-byte contexts; this one
        // assumes 32-byte throughout (see `SlotOrEndpointContext`), so
        // bail rather than lay out structures the controller will
        // misinterpret.
        log_fail!("USB", "xHCI", "Controller wants 64-byte contexts, which this driver doesn't support yet");
        return;
    }

    let op = unsafe { base.add(cap_length as usize) } as *mut OpRegs;
    let rt = unsafe { base.add(rtsoff as usize) } as *mut RuntimeRegs;
    let db = unsafe { base.add(dboff as usize) } as *mut u32;

    let xecp_dwords = (hcc_params1 >> 16) & 0xFFFF;
    if xecp_dwords != 0 {
        request_bios_handoff(base, xecp_dwords as usize);
    }

    if !reset_controller(op) {
        log_fail!("USB", "xHCI", "Controller did not come out of reset in time");
        return;
    }

    let Some(cmd_ring) = Ring::new() else {
        log_fail!("USB", "xHCI", "Out of memory allocating the command ring");
        return;
    };
    let Some(event_ring) = Ring::new() else {
        log_fail!("USB", "xHCI", "Out of memory allocating the event ring");
        return;
    };
    let Some(dcbaa_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("USB", "xHCI", "Out of memory allocating the DCBAA");
        return;
    };
    let Some(input_ctx_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("USB", "xHCI", "Out of memory allocating the input context");
        return;
    };
    let Some(output_ctx_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("USB", "xHCI", "Out of memory allocating the device context");
        return;
    };
    let Some(erst_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("USB", "xHCI", "Out of memory allocating the event ring segment table");
        return;
    };
    let Some(ep0_ring) = Ring::new() else {
        log_fail!("USB", "xHCI", "Out of memory allocating EP0's transfer ring");
        return;
    };

    // Event Ring Segment Table: one entry describing our single-segment
    // event ring (xHCI 6.5). Sixteen bytes: base address (64-bit) then
    // size (in TRBs) then 4 reserved bytes.
    unsafe {
        let erst = pmm::phys_to_virt(erst_phys) as *mut u64;
        core::ptr::write_volatile(erst, event_ring.phys);
        core::ptr::write_volatile((erst as *mut u32).add(2), RING_SIZE as u32);
        core::ptr::write_volatile((erst as *mut u32).add(3), 0);
    }

    // Program DCBAAP, CRCR, and the interrupter's event ring pointers.
    // CRCR's cycle bit (bit 0) must be OR'd into the address to match
    // the ring's initial producer cycle state.
    reg_write64_split(unsafe { core::ptr::addr_of_mut!((*op).dcbaap_lo) }, unsafe { core::ptr::addr_of_mut!((*op).dcbaap_hi) }, dcbaa_phys);
    reg_write64_split(unsafe { core::ptr::addr_of_mut!((*op).crcr_lo) }, unsafe { core::ptr::addr_of_mut!((*op).crcr_hi) }, cmd_ring.phys | 1);
    reg_write32(unsafe { core::ptr::addr_of_mut!((*rt).ir0_erstsz) }, 1);
    reg_write64_split(
        unsafe { core::ptr::addr_of_mut!((*rt).ir0_erstba_lo) },
        unsafe { core::ptr::addr_of_mut!((*rt).ir0_erstba_hi) },
        erst_phys,
    );
    reg_write64_split(
        unsafe { core::ptr::addr_of_mut!((*rt).ir0_erdp_lo) },
        unsafe { core::ptr::addr_of_mut!((*rt).ir0_erdp_hi) },
        event_ring.phys,
    );
    // CONFIG: tell the controller how many device slots we intend to
    // use. One is plenty for a single keyboard.
    reg_write32(unsafe { core::ptr::addr_of_mut!((*op).config) }, 1.min(max_slots as u32));

    // Run the controller. Interrupts (USBCMD bit 2) deliberately left
    // off -- see module docs.
    reg_write32(unsafe { core::ptr::addr_of_mut!((*op).usbcmd) }, USBCMD_RUN);
    if !spin_until(100_000, || reg_read32(unsafe { core::ptr::addr_of!((*op).usbsts) }) & USBSTS_HCH == 0) {
        log_fail!("USB", "xHCI", "Controller did not leave Halted state after Run");
        return;
    }

    let mut controller = Controller {
        cap,
        op,
        rt,
        db,
        max_ports,
        cmd_ring,
        event_ring,
        event_cycle: 1,
        event_dequeue: 0,
        dcbaa_phys,
        input_ctx_phys,
        output_ctx_phys,
        ep0_ring_phys: ep0_ring.phys,
        slot_id: 0,
        keyboard_ring: None,
        report_phys: 0,
    };
    // `ep0_ring` itself goes out of scope here, which is fine: `Ring`
    // has no `Drop` impl (its backing page is intentionally never freed
    // -- USB structures live for the kernel's whole lifetime), and the
    // physical address that matters is already saved in
    // `ep0_ring_phys`. EP0's enqueue/cycle cursor is tracked separately
    // in `EP0_RING_STATE`, reconstructed into a fresh `Ring` value on
    // each call (see `control_transfer`) rather than kept live here.

    log_ok!("USB", "xHCI", "Controller running, scanning {} port(s) for a keyboard", max_ports);

    for port in 1..=max_ports {
        if try_setup_keyboard(&mut controller, port) {
            log_ok!("USB", "xHCI", "USB keyboard ready on port {}", port);
            KEYBOARD_READY.store(true, Ordering::Release);
            *CONTROLLER.lock() = Some(controller);
            return;
        }
    }

    log_ok!("USB", "xHCI", "No USB keyboard found on any port");
    *CONTROLLER.lock() = Some(controller);
}

fn reset_controller(op: *mut OpRegs) -> bool {
    // Stop the controller first if it's already running (firmware may
    // have left it in a running state), then reset.
    if reg_read32(unsafe { core::ptr::addr_of!((*op).usbcmd) }) & USBCMD_RUN != 0 {
        reg_write32(unsafe { core::ptr::addr_of_mut!((*op).usbcmd) }, 0);
        if !spin_until(100_000, || reg_read32(unsafe { core::ptr::addr_of!((*op).usbsts) }) & USBSTS_HCH != 0) {
            return false;
        }
    }
    reg_write32(unsafe { core::ptr::addr_of_mut!((*op).usbcmd) }, USBCMD_HCRST);
    if !spin_until(100_000, || reg_read32(unsafe { core::ptr::addr_of!((*op).usbcmd) }) & USBCMD_HCRST == 0) {
        return false;
    }
    spin_until(100_000, || reg_read32(unsafe { core::ptr::addr_of!((*op).usbsts) }) & USBSTS_CNR == 0)
}

fn port_regs(c: &Controller, port: u8) -> *mut PortRegs {
    let cap_base = c.cap as *const u8;
    unsafe { cap_base.add(0x400 + (port as usize - 1) * 0x10) as *mut PortRegs }
}

/// Submit `trb` on the command ring, ring the command doorbell, and
/// spin for the matching Command Completion Event. Returns
/// `(completion_code, slot_id)` on success. This "submit and wait" model
/// is what this driver uses everywhere instead of an asynchronous/
/// interrupt-driven design -- simple, and entirely adequate for the
/// handful of commands boot-up enumeration needs.
fn do_command(c: &mut Controller, trb: Trb) -> Option<(u32, u8)> {
    let slot_phys = unsafe { c.cmd_ring.push(trb) };
    reg_write32(c.db, 0); // doorbell 0, target 0 = command ring doorbell

    let mut result = None;
    let ok = spin_until(500_000, || {
        if let Some(ev) = poll_event_ring(c) {
            if trb_type(ev.control) == TRB_TYPE_COMMAND_COMPLETION && ev.parameter == slot_phys {
                let completion_code = (ev.status >> 24) & 0xFF;
                let slot_id = ((ev.control >> 24) & 0xFF) as u8;
                result = Some((completion_code, slot_id));
                return true;
            }
            // A different event (e.g. a port status change queued up in
            // between) -- not what this call is waiting for, but
            // `poll_event_ring` already advanced past it, so the ring
            // doesn't back up; just keep spinning for the one we want.
        }
        false
    });
    if !ok {
        return None;
    }
    result
}

/// Pop and return the next event TRB, if any, advancing the event
/// ring's dequeue pointer and telling the controller where it now is
/// (ERDP) so it knows how much ring space has been freed. Returns
/// `None` if the ring is empty (the TRB at the dequeue pointer doesn't
/// have the expected cycle bit set).
fn poll_event_ring(c: &mut Controller) -> Option<Trb> {
    let trb = unsafe { core::ptr::read_volatile(c.event_ring.trbs.add(c.event_dequeue)) };
    if trb.control & TRB_CYCLE != c.event_cycle {
        return None; // nothing new
    }

    c.event_dequeue += 1;
    if c.event_dequeue == RING_SIZE {
        // The event ring's single segment wraps without a Link TRB of
        // its own (xHCI 4.9.4: software just wraps the index); flip the
        // expected cycle bit to match.
        c.event_dequeue = 0;
        c.event_cycle ^= 1;
    }

    let erdp_phys = c.event_ring.phys + (c.event_dequeue as u64) * 16;
    reg_write64_split(
        unsafe { core::ptr::addr_of_mut!((*c.rt).ir0_erdp_lo) },
        unsafe { core::ptr::addr_of_mut!((*c.rt).ir0_erdp_hi) },
        erdp_phys | ERDP_BUSY as u64,
    );

    Some(trb)
}

/// Try to bring up whatever's on `port` as a HID boot keyboard. Returns
/// `true` and leaves the controller configured for polling on success;
/// on any failure (nothing plugged in, not a keyboard, a step the
/// device didn't respond to) just returns `false` so the caller moves
/// on to the next port.
fn try_setup_keyboard(c: &mut Controller, port: u8) -> bool {
    let regs = port_regs(c, port);

    let portsc = reg_read32(unsafe { core::ptr::addr_of!((*regs).portsc) });
    if portsc & PORTSC_CCS == 0 {
        return false; // nothing plugged in
    }

    // Reset the port (required before it's usable regardless of
    // whether it was already enabled by firmware) and wait for the
    // reset-complete status bit.
    let cleared = portsc & !PORTSC_RW1C_MASK;
    reg_write32(unsafe { core::ptr::addr_of_mut!((*regs).portsc) }, cleared | PORTSC_PR);
    if !spin_until(200_000, || reg_read32(unsafe { core::ptr::addr_of!((*regs).portsc) }) & PORTSC_PRC != 0) {
        return false;
    }
    // Clear the reset-change bit we just observed (write-1-to-clear).
    let after_reset = reg_read32(unsafe { core::ptr::addr_of!((*regs).portsc) });
    reg_write32(
        unsafe { core::ptr::addr_of_mut!((*regs).portsc) },
        (after_reset & !PORTSC_RW1C_MASK) | PORTSC_PRC | PORTSC_CSC,
    );

    if after_reset & PORTSC_PED == 0 {
        return false; // reset didn't leave the port enabled
    }
    let speed = (after_reset & PORTSC_SPEED_MASK) >> 10;

    // Enable Slot.
    let Some((code, slot_id)) = do_command(c, Trb { parameter: 0, status: 0, control: make_type(TRB_TYPE_ENABLE_SLOT) }) else {
        return false;
    };
    if code != COMPLETION_SUCCESS || slot_id == 0 {
        return false;
    }
    c.slot_id = slot_id;

    // Fill in the Input Context: Input Control Context says "apply slot
    // context + endpoint 0 context" (add-context bits A0 and A1), the
    // Slot Context describes the device's route/speed/port, and the
    // EP0 (control endpoint) context sets up its transfer ring.
    unsafe {
        let control_ctx = pmm::phys_to_virt(c.input_ctx_phys) as *mut SlotOrEndpointContext;
        (*control_ctx).dwords[1] = 0b11; // A0 (slot) | A1 (EP0)

        let slot_ctx = slot_context_mut(c.input_ctx_phys);
        // dword0: route string (0, direct on root hub) | speed (bits
        // 20:23) | context entries (bits 27:31, at least 1 for EP0).
        (*slot_ctx).dwords[0] = (speed << 20) | (1 << 27);
        // dword1 bits 16:23: root hub port number.
        (*slot_ctx).dwords[1] = (port as u32) << 16;

        let ep0_ctx = ep_context_mut(c.input_ctx_phys, 1);
        // EP type = 4 (control) in dword1 bits 3:5; max packet size
        // depends on speed (8 for low speed, 64 otherwise is a
        // reasonable default for full/high speed -- a fuller driver
        // reads this from the device descriptor's bMaxPacketSize0 and
        // reconfigures, which this one skips for simplicity).
        let max_packet0: u32 = if speed == 2 { 8 } else { 64 }; // speed 2 = low-speed
        (*ep0_ctx).dwords[1] = (4 << 3) | (max_packet0 << 16) | (3 << 1); // CErr=3
        (*ep0_ctx).dwords[2] = (c.ep0_ring_phys | 1) as u32; // TR dequeue ptr lo | DCS=1
        (*ep0_ctx).dwords[3] = (c.ep0_ring_phys >> 32) as u32;
    }

    // Point the DCBAA's slot entry at the Output Device Context so the
    // controller has somewhere to write the slot's live state back to.
    unsafe {
        let dcbaa = pmm::phys_to_virt(c.dcbaa_phys) as *mut u64;
        core::ptr::write_volatile(dcbaa.add(c.slot_id as usize), c.output_ctx_phys);
    }

    let Some((code, _)) = do_command(
        c,
        Trb {
            parameter: c.input_ctx_phys,
            status: 0,
            control: make_type(TRB_TYPE_ADDRESS_DEVICE) | ((c.slot_id as u32) << 24),
        },
    ) else {
        return false;
    };
    if code != COMPLETION_SUCCESS {
        return false;
    }

    // Device now has a USB address and EP0 is usable. Ask for the
    // device descriptor -- mostly just to confirm it's really there and
    // responsive. A fuller driver would check idVendor/idProduct or
    // walk the interface descriptors to confirm HID class 3 / boot
    // subclass 1 / protocol 1 (keyboard) before assuming boot-keyboard
    // reports mean anything; this one assumes "answered EP0 on this
    // port" is enough, which is a real simplification -- a USB mouse or
    // other class-3 boot device on this port would be treated the same
    // way and produce garbage keycodes.
    let Some(desc_phys) = pmm::alloc_frame_zeroed() else { return false };
    if !control_transfer_in(
        c,
        setup_packet(REQ_TYPE_STD_DEVICE_IN, REQ_GET_DESCRIPTOR, DESC_TYPE_DEVICE, 0, 18),
        desc_phys,
        18,
    ) {
        return false;
    }

    // SET_CONFIGURATION 1 -- assumes configuration value 1 exists and is
    // the one we want, true for essentially every simple USB HID
    // keyboard (a fuller driver would parse the configuration
    // descriptor and use its actual bConfigurationValue).
    if !control_transfer_out(c, setup_packet(REQ_TYPE_STD_DEVICE_OUT, REQ_SET_CONFIGURATION, 1, 0, 0)) {
        return false;
    }

    // HID SET_PROTOCOL(Boot) -- guarantees the fixed 8-byte report
    // layout this driver decodes. Interface 0 is assumed to be the
    // keyboard interface, true for the vast majority of plain USB
    // keyboards (a composite device with, say, a media-key interface
    // alongside would need real interface-descriptor parsing).
    if !control_transfer_out(c, setup_packet(REQ_TYPE_CLASS_IFACE_OUT, HID_REQ_SET_PROTOCOL, HID_PROTOCOL_BOOT, 0, 0)) {
        return false;
    }
    // SET_IDLE 0: ask the device to only send a report when something
    // changes rather than repeating on a timer. Not fatal if it fails
    // -- some devices are picky about it, and a keyboard that just
    // repeats its last report on a timer still works fine with this
    // driver's press/release diffing.
    let _ = control_transfer_out(c, setup_packet(REQ_TYPE_CLASS_IFACE_OUT, HID_REQ_SET_IDLE, 0, 0, 0));

    // Now configure the interrupt-IN endpoint (EP1 IN, DCI 3 by USB
    // convention -- IN endpoints are odd DCIs starting from 3) that
    // boot-protocol reports arrive on. This driver assumes endpoint
    // number 1 IN, true for the large majority of simple keyboards but
    // not guaranteed by spec; a fuller driver would read this out of
    // the HID interface's endpoint descriptor instead.
    let Some(kb_ring) = Ring::new() else { return false };
    unsafe {
        let control_ctx = pmm::phys_to_virt(c.input_ctx_phys) as *mut SlotOrEndpointContext;
        core::ptr::write_bytes(control_ctx, 0, 1); // fresh Input Control Context for this command
        (*control_ctx).dwords[1] = 1 << 3; // A3: add endpoint context at DCI 3 (EP1 IN)
        let slot_ctx = slot_context_mut(c.input_ctx_phys);
        (*slot_ctx).dwords[0] = (speed << 20) | (3 << 27); // context entries now includes DCI 3

        let ep1in_ctx = ep_context_mut(c.input_ctx_phys, 3);
        const EP_TYPE_INTERRUPT_IN: u32 = 7;
        let max_packet: u32 = 8; // boot-protocol reports are always 8 bytes
        (*ep1in_ctx).dwords[1] = (EP_TYPE_INTERRUPT_IN << 3) | (max_packet << 16) | (3 << 1);
        // Interval field (dword0 bits 16:23) as an exponent of 2 * 125us
        // frames; 4 (~1ms at full/high speed, near enough at low speed
        // too) is a safe, responsive-enough default without reading the
        // endpoint descriptor's actual bInterval.
        (*ep1in_ctx).dwords[0] = 4 << 16;
        (*ep1in_ctx).dwords[2] = (kb_ring.phys | 1) as u32;
        (*ep1in_ctx).dwords[3] = (kb_ring.phys >> 32) as u32;
    }

    let Some((code, _)) = do_command(
        c,
        Trb {
            parameter: c.input_ctx_phys,
            status: 0,
            control: make_type(TRB_TYPE_CONFIGURE_ENDPOINT) | ((c.slot_id as u32) << 24),
        },
    ) else {
        return false;
    };
    if code != COMPLETION_SUCCESS {
        return false;
    }

    let Some(report_phys) = pmm::alloc_frame_zeroed() else { return false };
    c.keyboard_ring = Some(kb_ring);
    c.report_phys = report_phys;

    // Prime the first transfer so `poll()` has something outstanding to
    // check on the very first call.
    submit_report_read(c);

    true
}

/// EP0's transfer-ring enqueue/cycle state, tracked separately from
/// `Controller` since EP0 is only ever used sequentially during setup
/// (one control transfer at a time, never interleaved with anything
/// else touching the ring) and every control transfer in this driver
/// goes through `control_transfer` below.
static EP0_RING_STATE: IrqMutex<(usize, u32)> = IrqMutex::new((0, 1));

/// Issue a control-IN transfer on EP0 (Setup -> Data(IN) -> Status(OUT))
/// and spin for its completion, copying the result into `buf_phys`.
/// Used for GET_DESCRIPTOR.
fn control_transfer_in(c: &mut Controller, setup: u64, buf_phys: u64, len: u32) -> bool {
    control_transfer(c, setup, Some((buf_phys, len)), true)
}

/// Issue a control-OUT (no data stage) transfer on EP0 (Setup ->
/// Status(IN)) and spin for its completion. Used for
/// SET_CONFIGURATION and the HID class requests.
fn control_transfer_out(c: &mut Controller, setup: u64) -> bool {
    control_transfer(c, setup, None, false)
}

fn control_transfer(c: &mut Controller, setup: u64, data: Option<(u64, u32)>, data_is_in: bool) -> bool {
    let (enqueue, cycle) = *EP0_RING_STATE.lock();
    let mut ring = Ring { trbs: pmm::phys_to_virt(c.ep0_ring_phys) as *mut Trb, phys: c.ep0_ring_phys, enqueue, cycle };

    let transfer_type: u32 = match &data {
        Some(_) if data_is_in => 3, // IN data stage present
        Some(_) => 2,               // OUT data stage present
        None => 0,                  // no data stage
    };

    let setup_trb = Trb {
        parameter: setup,
        status: 8, // TRB transfer length is always 8 for a Setup Stage TRB
        control: make_type(TRB_TYPE_SETUP_STAGE) | TRB_IOC | (transfer_type << 16) | (1 << 6), // IDT: immediate data
    };
    unsafe { ring.push(setup_trb) };

    if let Some((buf_phys, len)) = data {
        let data_trb = Trb {
            parameter: buf_phys,
            status: len,
            control: make_type(TRB_TYPE_DATA_STAGE) | TRB_IOC | if data_is_in { 1 << 16 } else { 0 },
        };
        unsafe { ring.push(data_trb) };
    }

    // Status stage direction is opposite the data stage's (or IN, if
    // there was no data stage at all -- a control-OUT-with-no-data
    // transfer still has an IN status stage).
    let status_is_in = !data_is_in || data.is_none();
    let status_trb = Trb {
        parameter: 0,
        status: 0,
        control: make_type(TRB_TYPE_STATUS_STAGE) | TRB_IOC | if status_is_in { 1 << 16 } else { 0 },
    };
    let status_slot_phys = unsafe { ring.push(status_trb) };

    *EP0_RING_STATE.lock() = (ring.enqueue, ring.cycle);

    // EP0 is DCI 1; doorbell target field = DCI.
    reg_write32(unsafe { c.db.add(c.slot_id as usize) }, 1);

    let mut saw_completion = false;
    let responded = spin_until(500_000, || {
        if let Some(ev) = poll_event_ring(c) {
            if trb_type(ev.control) == TRB_TYPE_TRANSFER_EVENT && ev.parameter == status_slot_phys {
                let code = (ev.status >> 24) & 0xFF;
                saw_completion = code == COMPLETION_SUCCESS || code == COMPLETION_SHORT_PACKET;
                return true;
            }
        }
        false
    });
    responded && saw_completion
}

/// Push one Normal TRB on the keyboard's interrupt-IN transfer ring to
/// receive the next report, and ring its doorbell. Called once at setup
/// and again after every report `poll()` picks up, so there's always
/// exactly one outstanding read.
fn submit_report_read(c: &mut Controller) {
    let Some(ring) = c.keyboard_ring.as_mut() else { return };
    let trb = Trb { parameter: c.report_phys, status: 8, control: make_type(TRB_TYPE_NORMAL) | TRB_IOC | (1 << 5) };
    unsafe { ring.push(trb) };
    // DCI 3 = EP1 IN, matching the endpoint configured in
    // `try_setup_keyboard`.
    reg_write32(unsafe { c.db.add(c.slot_id as usize) }, 3);
}

/// Service the event ring for a completed keyboard report, if any, and
/// translate it into ASCII bytes on the shared queue (see
/// `try_read_byte`). Cheap to call frequently -- it's just a couple of
/// volatile reads when nothing's happened -- and is what stands in for
/// this driver not having an interrupt handler; see module docs.
///
/// Called from `syscall::next_byte`'s poll loop, the same place that
/// already polls `drivers::ps2` every iteration while waiting for a
/// keypress.
pub fn poll() {
    if !KEYBOARD_READY.load(Ordering::Acquire) {
        return;
    }
    let mut guard = CONTROLLER.lock();
    let Some(c) = guard.as_mut() else { return };

    let Some(ev) = poll_event_ring(c) else { return };
    if trb_type(ev.control) != TRB_TYPE_TRANSFER_EVENT {
        return; // a port status change or stray event; nothing to do
    }
    let completion_code = (ev.status >> 24) & 0xFF;
    if completion_code == COMPLETION_SUCCESS || completion_code == COMPLETION_SHORT_PACKET {
        handle_report(c);
    }
    // Always queue the next read regardless of completion code -- a
    // transient error on one report shouldn't stop future ones.
    submit_report_read(c);
}

/// USB HID boot-keyboard usage IDs -> ASCII, US layout, unshifted.
/// Index is the raw HID keycode (0x04 = 'a' ... 0x27 = '0', etc, per the
/// HID Usage Tables spec, table 12); 0 marks keys with no ASCII meaning
/// this minimal driver doesn't represent, matching `drivers::ps2`'s
/// SET1 tables in spirit.
#[rustfmt::skip]
const HID_UNSHIFTED: [u8; 0x39] = [
    0,    0,    0,    0,    b'a', b'b', b'c', b'd', b'e', b'f', b'g', b'h', b'i', b'j', // 0x00-0x0D
    b'k', b'l', b'm', b'n', b'o', b'p', b'q', b'r', b's', b't', b'u', b'v',             // 0x0E-0x1B
    b'w', b'x', b'y', b'z', b'1', b'2', b'3', b'4', b'5', b'6', b'7', b'8',             // 0x1C-0x27
    b'9', b'0', b'\n', 0x1B, 0x08, b'\t', b' ', b'-', b'=', b'[', b']',                 // 0x28-0x32
    b'\\', 0,   b';', b'\'', b'`', b',', b'.', b'/',                                    // 0x33-0x38
];

#[rustfmt::skip]
const HID_SHIFTED: [u8; 0x39] = [
    0,    0,    0,    0,    b'A', b'B', b'C', b'D', b'E', b'F', b'G', b'H', b'I', b'J',
    b'K', b'L', b'M', b'N', b'O', b'P', b'Q', b'R', b'S', b'T', b'U', b'V',
    b'W', b'X', b'Y', b'Z', b'!', b'@', b'#', b'$', b'%', b'^', b'&', b'*',
    b'(', b')', b'\n', 0x1B, 0x08, b'\t', b' ', b'_', b'+', b'{', b'}',
    b'|', 0,    b':', b'"', b'~', b'<', b'>', b'?',
];

const HID_KEYCODE_CAPS_LOCK: u8 = 0x39;
/// Modifier byte bits (HID boot report byte 0): left/right Shift.
const MOD_LSHIFT: u8 = 1 << 1;
const MOD_RSHIFT: u8 = 1 << 5;

/// Ring buffer, identical in shape and role to `drivers::ps2`'s own --
/// see that module for why (both feed `syscall::next_byte` the same
/// way).
struct RingBuffer {
    buf: [u8; 256],
    head: usize,
    tail: usize,
    len: usize,
}
impl RingBuffer {
    const fn new() -> Self {
        RingBuffer { buf: [0; 256], head: 0, tail: 0, len: 0 }
    }
    fn push(&mut self, byte: u8) {
        if self.len == 256 {
            self.tail = (self.tail + 1) % 256;
            self.len -= 1;
        }
        self.buf[self.head] = byte;
        self.head = (self.head + 1) % 256;
        self.len += 1;
    }
    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let b = self.buf[self.tail];
        self.tail = (self.tail + 1) % 256;
        self.len -= 1;
        Some(b)
    }
}
static ASCII_QUEUE: IrqMutex<RingBuffer> = IrqMutex::new(RingBuffer::new());

fn handle_report(c: &mut Controller) {
    let report = unsafe { core::slice::from_raw_parts(pmm::phys_to_virt(c.report_phys), 8) };
    let modifiers = report[0];
    let keys = [report[2], report[3], report[4], report[5], report[6], report[7]];

    let shift = modifiers & (MOD_LSHIFT | MOD_RSHIFT) != 0;

    let mut prev = PREV_KEYS.lock();
    for &code in &keys {
        if code == 0 || prev.contains(&code) {
            continue; // no key in this slot, or already down last report
        }
        if code == HID_KEYCODE_CAPS_LOCK {
            let cur = CAPS_LOCK.load(Ordering::Relaxed);
            CAPS_LOCK.store(!cur, Ordering::Relaxed);
            continue;
        }
        if (code as usize) >= HID_UNSHIFTED.len() {
            continue;
        }
        let caps = CAPS_LOCK.load(Ordering::Relaxed);
        let mut ascii = if shift { HID_SHIFTED[code as usize] } else { HID_UNSHIFTED[code as usize] };
        if ascii == 0 {
            continue;
        }
        if caps && ascii.is_ascii_lowercase() {
            ascii = ascii.to_ascii_uppercase();
        } else if caps && ascii.is_ascii_uppercase() && shift {
            ascii = ascii.to_ascii_lowercase();
        }
        ASCII_QUEUE.lock().push(ascii);
    }
    *prev = keys;
}

/// Non-blocking: `Some(byte)` if a translated USB keystroke is waiting,
/// mirroring `drivers::ps2::try_read_byte` so `syscall::next_byte` can
/// poll both the same way.
pub fn try_read_byte() -> Option<u8> {
    ASCII_QUEUE.lock().pop()
}