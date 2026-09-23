//! USB, staged in on top of PS/2.
//!
//! Getting from here to a working USB keyboard is a lot more machinery
//! than PS/2: find the host controller on the PCI bus, map its MMIO
//! registers, bring up an xHCI controller (command ring, event ring,
//! device context base array), enumerate whatever's plugged into each
//! port, and speak the HID boot-keyboard protocol to the result. Real
//! hardware may also require dropping USB legacy support (BIOS/SMM
//! keeps ownership of the controller for PS/2 emulation until the OS
//! explicitly asks for it) before any of that works at all.
//!
//! `init()` discovers PCI USB controllers and starts the first xHCI
//! controller for direct-attached HID boot keyboards. This remains a
//! small polling driver: it has no hub, hot-plug, or non-xHCI support.

pub mod pci;
pub mod xhci;

use crate::log_ok;

/// Discover USB host controllers and initialize the first xHCI controller.
pub fn init() {
    let controllers = pci::find_usb_controllers();

    if controllers.is_empty() {
        log_ok!("USB", "Init", "No USB host controller found on the PCI bus");
        return;
    }

    for c in &controllers {
        log_ok!(
            "USB",
            "Init",
            "Found {} controller at {:02x}:{:02x}.{} (BAR0 base {:#x})",
            c.kind.name(),
            c.location.bus,
            c.location.device,
            c.location.function,
            c.mmio_base
        );
    }

    if let Some(xhci_ctrl) = controllers
        .iter()
        .find(|c| c.kind == pci::ControllerKind::Xhci)
    {
        xhci::probe(xhci_ctrl);
    } else {
        log_ok!(
            "USB",
            "Init",
            "No xHCI controller among them -- only xHCI is supported so far, stopping here"
        );
    }
}
