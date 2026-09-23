//! Device drivers live here, one subsystem per module, grouped under
//! this directory instead of sitting loose in `src/` -- there's more
//! than one input driver now (PS/2 today, USB HID once it exists), and
//! more will land here later (storage, NIC, ...) rather than in the
//! kernel's core modules.
//!
//! Every driver in here follows the same shape as the core drivers that
//! predate this folder (`pic`, `pit`): an `init()` that programs the
//! hardware and registers its IRQ handler with `idt::register_irq` +
//! `interrupts::unmask_isa_irq`, and a `self_test()` that's safe to call once
//! interrupts are on. `main.rs` calls both in order.

pub mod ps2;
pub mod usb;
pub mod ahci;
pub mod nvme;
