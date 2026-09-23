//! Runtime selection between the legacy 8259 PIC and ACPI-discovered APICs.

use core::sync::atomic::{AtomicBool, Ordering};

static APIC_ROUTING: AtomicBool = AtomicBool::new(false);

/// Switch ISA IRQ requests and acknowledgements to the APIC backend.
/// Call only after the Local APIC and all I/O APICs are initialized.
pub fn use_apic() {
    APIC_ROUTING.store(true, Ordering::Release);
}

pub fn using_apic() -> bool {
    APIC_ROUTING.load(Ordering::Acquire)
}

pub fn unmask_isa_irq(irq: u8) {
    if using_apic() {
        crate::ioapic::unmask_isa_irq(irq);
    } else {
        crate::pic::unmask(irq);
    }
}

pub fn mask_isa_irq(irq: u8) {
    if using_apic() {
        crate::ioapic::mask_isa_irq(irq);
    } else {
        crate::pic::mask(irq);
    }
}

pub fn end_of_interrupt(irq: u8) {
    if using_apic() {
        crate::lapic::end_of_interrupt();
    } else {
        crate::pic::end_of_interrupt(irq);
    }
}
