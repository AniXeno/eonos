//! Full-screen crash display.
//!
//! Fatal CPU exceptions and Rust panics already produce a detailed register
//! dump through `log_critical!` (serial + console, scrolling log lines).
//! This module draws a clean, final overlay on top of that: a solid-colour
//! screen with a title bar and a short summary, so the one thing left on
//! screen when the machine halts is readable at a glance instead of being
//! the tail end of a scrolling log.
//!
//! It reuses the existing text console (and its ANSI colour support,
//! including 24-bit `\x1b[48;2;r;g;bm` backgrounds) rather than drawing
//! pixels directly, so it can't get out of sync with how the console
//! actually renders text.

use core::fmt::Write;

use crate::console::{Console, CONSOLE};
use crate::log_critical;

/// Force-unlock every mutex a crash handler might need, in case the fault
/// happened while one of them was held (e.g. inside a `log_*!` call). Safe
/// to call even when nothing is locked. Must run before any of these are
/// touched again.
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

/// Draw a full-screen crash report and halt. `rgb` sets the background
/// colour (e.g. dark red for CPU exceptions, purple for Rust panics) so the
/// two kinds of fatal error are visually distinguishable at a glance.
/// `body` writes the details between the header and footer; never returns.
pub fn show<F>(title: &str, rgb: (u8, u8, u8), body: F) -> !
where
    F: FnOnce(&mut Console),
{
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