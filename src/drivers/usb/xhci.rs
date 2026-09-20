//! xHCI (Extensible Host Controller Interface) -- the controller type
//! basically every USB 3 (and most USB 2) port is behind on modern
//! hardware. This is a stub: it maps the controller's MMIO region and
//! reads its capability registers far enough to log what's there, but
//! does not yet reset the controller, program the command/event rings,
//! or enumerate any device on any port. No HID input flows from here
//! yet -- PS/2 (`drivers::ps2`) is still the only working keyboard path.
//!
//! What a real driver still needs to do from here, roughly in order:
//! take ownership from firmware (the BIOS/OS handoff bit in the
//! extended capabilities list), reset the controller (USBCMD.HCRST),
//! allocate and program the Device Context Base Address Array, set up
//! a command ring and at least one event ring plus an MSI-X or
//! interrupt-pin IRQ to service it, reset each port, and run the
//! standard USB enumeration sequence (GET_DESCRIPTOR, SET_ADDRESS,
//! read the HID report descriptor) against whatever answers. Each of
//! those is its own chunk of work; this file is the foundation, not
//! the driver.

use crate::log_ok;
use super::pci::UsbController;

/// Capability register layout (xHCI spec, section 5.3). All controllers
/// expose at least this much at the base of their MMIO BAR.
#[repr(C)]
struct CapabilityRegisters {
    cap_length: u8,
    reserved: u8,
    hci_version: u16,
    hcs_params1: u32,
    hcs_params2: u32,
    hcs_params3: u32,
    hcc_params1: u32,
    dboff: u32,
    rtsoff: u32,
    hcc_params2: u32,
}

/// Read the capability registers and log what the controller reports.
/// Takes the physical MMIO base directly rather than a virtual address:
/// EonOS doesn't yet have a general-purpose "map this physical range
/// into kernel space" helper outside the direct map `pmm` already
/// maintains, so this leans on that (see the safety note below) instead
/// of introducing a one-off mapping path for a stub that doesn't act on
/// the data yet.
pub fn probe(controller: &UsbController) {
    if controller.mmio_base == 0 {
        log_ok!(
            "USB",
            "xHCI",
            "Controller has no usable memory BAR (I/O-mapped?) -- cannot probe further"
        );
        return;
    }

    // Safety: `mmio_base` came straight from the device's BAR0/BAR1, so
    // it's a real physical address the PCI device decodes; EonOS's
    // direct map covers all physical memory (see `pmm::phys_to_virt`),
    // and MMIO registers here are read-only observation (no writes),
    // so there's no risk of corrupting controller state by reading
    // capability registers that firmware already left in a defined
    // state at boot.
    let regs = unsafe { &*(crate::pmm::phys_to_virt(controller.mmio_base) as *const CapabilityRegisters) };

    let version = regs.hci_version;
    let max_slots = regs.hcs_params1 & 0xFF;
    let max_ports = (regs.hcs_params1 >> 24) & 0xFF;

    log_ok!(
        "USB",
        "xHCI",
        "Controller version {}.{} -- {} device slot(s), {} root port(s) (driver not implemented past this point yet)",
        (version >> 8) & 0xFF,
        version & 0xFF,
        max_slots,
        max_ports
    );
}