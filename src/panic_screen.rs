use core::fmt::Write;

use crate::console::{Console, CONSOLE};
use crate::log_critical;

pub fn force_unlock_all() {
    unsafe {
        crate::logger::LOGGER.force_unlock();
        crate::serial::SERIAL1.force_unlock();
        CONSOLE.force_unlock();
    }
}

fn rule(console: &mut Console, ch: char) {
    for _ in 0..console.cols() {
        let _ = console.write_char(ch);
    }
    let _ = console.write_char('\n');
}

fn centered(console: &mut Console, s: &str) {
    let pad = console.cols().saturating_sub(s.chars().count()) / 2;
    for _ in 0..pad {
        let _ = console.write_char(' ');
    }
    let _ = writeln!(console, "{}", s);
}

pub fn show<F>(title: &str, rgb: (u8, u8, u8), body: F) -> !
where
    F: FnOnce(&mut Console),
{
    crate::idt::disable_interrupts();
    force_unlock_all();

    if let Some(console) = CONSOLE.lock().as_mut() {
        let (r, g, b) = rgb;
        let _ = write!(console, "\x1b[48;2;{r};{g};{b}m\x1b[97m");
        console.clear();

        rule(console, '=');
        centered(console, title);
        rule(console, '=');
        let _ = writeln!(console);

        body(console);

        let _ = writeln!(console);
        rule(console, '-');
        centered(console, "System halted -- power off or reset the machine.");
        let _ = write!(console, "\x1b[0m");
    }

    log_critical!("Kernel", "Halt", "{} -- system halted", title);
    crate::hcf()
}