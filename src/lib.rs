extern crate libc;
use libc::c_char;

#[macro_use]
extern crate bitflags;

extern crate libfreenect_sys;
use libfreenect_sys::{
    freenect_context,
    freenect_init,
    freenect_shutdown,
    freenect_loglevel,
    freenect_set_log_level,
    freenect_process_events,
    freenect_num_devices,
    freenect_device_flags,
};

use std::ptr;
use std::ffi;
use std::rc::Rc;

enum FreenectError {
    LibraryReturnCode(i32),
    NullPtr,
}

pub type FreenectResult<T> = Result<T, FreenectError>;

pub enum LogLevel {
	Fatal,         // Log for crashing/non-recoverable errors
	Error,         // Log for major errors
	Warning,       // Log for warning messages
	Notice,        // Log for important messages
	Info,          // Log for normal messages
	Debug,         // Log for useful development messages
	Spew,          // Log for slightly less useful messages
	Flood,         // Log EVERYTHING. May slow performance.
}

impl LogLevel {
    fn to_lowlevel(self) -> freenect_loglevel {
        match self {
            LogLevel::Fatal => freenect_loglevel::FREENECT_LOG_FATAL,
            _ => freenect_loglevel::FREENECT_LOG_FATAL,
        }
    }

    fn from_lowlevel(lvl: freenect_loglevel) -> LogLevel {
        match lvl {
            freenect_loglevel::FREENECT_LOG_FATAL => LogLevel::Fatal,
            _ => LogLevel::Fatal
        }
    }
}

bitflags! {
    flags DeviceFlags: u32 {
        const DEVICE_MOTOR  = freenect_device_flags::FREENECT_DEVICE_MOTOR  as u32,
        const DEVICE_CAMERA = freenect_device_flags::FREENECT_DEVICE_CAMERA as u32,
        const DEVICE_AUDIO  = freenect_device_flags::FREENECT_DEVICE_AUDIO  as u32,
    }
}

pub struct Context {
    ctx: *mut freenect_context,
}

impl Context {
    pub fn new() -> FreenectResult<Context> {
        let mut ctx = Context{ctx: ptr::null_mut()};

        match unsafe { freenect_init(&mut ctx.ctx, ptr::null_mut()) } {
            0 => {
                if ctx.ctx != ptr::null_mut() {
                    Ok(ctx)
                } else {
                    Err(FreenectError::NullPtr)
                }
            },
            x => Err(FreenectError::LibraryReturnCode(x)),
        }
    }

    pub fn set_log_level(&mut self, level: LogLevel) {
        unsafe { freenect_set_log_level(self.ctx, level.to_lowlevel()); }
    }

    pub fn set_log_callback(&mut self) {

    }

    // Convert C callback parameters to rustified versions, then call user callback
    fn callback_trampoline(dev: *mut freenect_context, level: freenect_loglevel, msg: *const c_char) {
        let level = LogLevel::from_lowlevel(level);
        let msg = unsafe { ffi::CStr::from_ptr(msg) };
    }

    pub fn process_events(&mut self) -> FreenectResult<()> {
        match unsafe { freenect_process_events(self.ctx) } {
            0 => Ok(()),
            x => Err(FreenectError::LibraryReturnCode(x)),
        }
    }

    pub fn num_devices(&mut self) -> FreenectResult<u32> {
        let ret = unsafe { freenect_num_devices(self.ctx) };
        if ret < 0 {
            Err(FreenectError::LibraryReturnCode(ret))
        } else {
            Ok(ret as u32)
        }
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        let ret = unsafe { freenect_shutdown(self.ctx) };

        if ret < 0 {
            panic!(ret)
        }
    }
}
