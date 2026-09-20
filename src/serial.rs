use core::fmt;
use crate::sync::IrqMutex;

const COM1: u16 = 0x3F8;

pub static SERIAL1: IrqMutex<SerialPort> = IrqMutex::new(SerialPort::new(COM1));

pub struct SerialPort {
    port: u16,
}

impl SerialPort {
    pub const fn new(port: u16) -> Self {
        Self { port }
    }

    pub fn init(&mut self) {
        unsafe {
            outb(self.port + 1, 0x00); 
            outb(self.port + 3, 0x80); 
            outb(self.port + 0, 0x03); 
            outb(self.port + 1, 0x00); 
            outb(self.port + 3, 0x03); 
            outb(self.port + 2, 0xC7); 
            outb(self.port + 4, 0x0B); 
        }
    }

    fn is_transmit_empty(&self) -> bool {
        unsafe { inb(self.port + 5) & 0x20 != 0 }
    }

    pub fn write_byte(&mut self, byte: u8) {
        while !self.is_transmit_empty() {
            core::hint::spin_loop();
        }
        unsafe { outb(self.port, byte) };
    }

    /// Non-blocking: `Some(byte)` if the receiver has one waiting
    /// (LSR bit 0, "data ready"), `None` otherwise. This is the only
    /// input device EonOS has until a real keyboard driver (PS/2 or
    /// USB HID) exists -- in QEMU it's whatever is hooked up to COM1
    /// (usually the host terminal via `-serial stdio`).
    pub fn try_read_byte(&mut self) -> Option<u8> {
        unsafe {
            if inb(self.port + 5) & 0x01 != 0 {
                Some(inb(self.port))
            } else {
                None
            }
        }
    }
}

impl fmt::Write for SerialPort {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            match byte {
                b'\n' => {
                    self.write_byte(b'\r');
                    self.write_byte(b'\n');
                }
                b => self.write_byte(b),
            }
        }
        Ok(())
    }
}

unsafe fn outb(port: u16, val: u8) {
    core::arch::asm!("out dx, al", in("dx") port, in("al") val, options(nomem, nostack, preserves_flags));
}

unsafe fn inb(port: u16) -> u8 {
    let val: u8;
    core::arch::asm!("in al, dx", in("dx") port, out("al") val, options(nomem, nostack, preserves_flags));
    val
}