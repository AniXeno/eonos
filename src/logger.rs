use core::fmt::{self, Write};
use crate::sync::IrqMutex;

use crate::console::CONSOLE;
use crate::serial::SERIAL1;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Ok,
    Busy,
    Fail,
    Critical,
    Debug,
}

impl Status {
    pub const fn as_str(self) -> &'static str {
        match self {
            Status::Ok => "OK",
            Status::Busy => "BUSY",
            Status::Fail => "FAIL",
            Status::Critical => "CRITICAL",
            Status::Debug => "DEBUG",
        }
    }

    pub const fn color(self) -> &'static str {
        match self {
            Status::Ok => "\x1b[92m",       
            Status::Busy => "\x1b[93m",     
            Status::Fail => "\x1b[91m",     
            Status::Critical => "\x1b[97;41m", 
            Status::Debug => "\x1b[94m",   
        }
    }

    pub const fn text_color(self) -> &'static str {
        match self {
            Status::Fail | Status::Critical => "\x1b[91m",
            _ => "\x1b[37m",
        }
    }
}

const EARLY_LOG_CAPACITY: usize = 32;
const MAX_LINE_LEN: usize = 256; 

struct LogEntry {
    buf: [u8; MAX_LINE_LEN],
    len: usize,
}

pub struct Logger {
    pub use_color: bool,
    early_buffer: [LogEntry; EARLY_LOG_CAPACITY],
    early_count: usize,
}

pub static LOGGER: IrqMutex<Logger> = IrqMutex::new(Logger {
    use_color: true,
    early_buffer: [const { LogEntry { buf: [0; MAX_LINE_LEN], len: 0 } }; EARLY_LOG_CAPACITY],
    early_count: 0,
});

fn emit<W: Write>(
    w: &mut W,
    colors: bool,
    module: &str,
    function: &str,
    output: fmt::Arguments,
    status: Status,
) {
    let (dim, reset, modc, func, text, stc) = if colors {
        ("\x1b[90m", "\x1b[0m", "\x1b[96m", "\x1b[95m", status.text_color(), status.color())
    } else {
        ("", "", "", "", "", "")
    };

    let _ = write!(w, "{modc}{module}{dim}:{func}{function}{dim} > {reset}{text}");
    let _ = w.write_fmt(output);
    let _ = write!(w, "{reset} {dim}|{reset} {stc}{}{reset}\n", status.as_str());
}

impl Logger {
    pub fn log(&mut self, module: &str, function: &str, output: fmt::Arguments, status: Status) {
        {
            let mut serial = SERIAL1.lock();
            emit(&mut *serial, self.use_color, module, function, output, status);
        }

        if let Some(console) = CONSOLE.lock().as_mut() {
            emit(console, true, module, function, output, status);
        } else if self.early_count < EARLY_LOG_CAPACITY {
            let entry = &mut self.early_buffer[self.early_count];
            let mut writer = BufferWriter { buf: &mut entry.buf, len: 0 };
            emit(&mut writer, true, module, function, output, status);
            entry.len = writer.len;
            self.early_count += 1;
        }
    }

    fn flush(&mut self) {
        if let Some(console) = CONSOLE.lock().as_mut() {
            for i in 0..self.early_count {
                let entry = &self.early_buffer[i];
                if let Ok(s) = core::str::from_utf8(&entry.buf[..entry.len]) {
                    let _ = console.write_str(s);
                    let _ = console.write_str("\x1b[0m");
                }
            }
            self.early_count = 0;
        }
    }
}

pub fn flush_to_console() {
    LOGGER.lock().flush();
}

struct BufferWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> fmt::Write for BufferWriter<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let rem = self.buf.len() - self.len;
        let to_copy = bytes.len().min(rem);
        self.buf[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
        Ok(())
    }
}

#[macro_export]
macro_rules! klog {
    ($module:expr, $function:expr, $status:expr, $($arg:tt)*) => {{
        $crate::logger::LOGGER.lock().log($module, $function, format_args!($($arg)*), $status);
    }};
}

#[macro_export]
macro_rules! log_ok {
    ($module:expr, $function:expr, $($arg:tt)*) => {
        $crate::klog!($module, $function, $crate::logger::Status::Ok, $($arg)*)
    };
}

#[macro_export]
macro_rules! log_busy {
    ($module:expr, $function:expr, $($arg:tt)*) => {
        $crate::klog!($module, $function, $crate::logger::Status::Busy, $($arg)*)
    };
}

#[macro_export]
macro_rules! log_fail {
    ($module:expr, $function:expr, $($arg:tt)*) => {
        $crate::klog!($module, $function, $crate::logger::Status::Fail, $($arg)*)
    };
}

#[macro_export]
macro_rules! log_critical {
    ($module:expr, $function:expr, $($arg:tt)*) => {
        $crate::klog!($module, $function, $crate::logger::Status::Critical, $($arg)*)
    };
}

#[macro_export]
macro_rules! log_debug {
    ($module:expr, $function:expr, $($arg:tt)*) => {
        $crate::klog!($module, $function, $crate::logger::Status::Debug, $($arg)*)
    };
}