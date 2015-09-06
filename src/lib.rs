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
    freenect_device_attributes,
    freenect_list_device_attributes,
    freenect_free_device_attributes,
    freenect_supported_subdevices,
    freenect_select_subdevices,
    freenect_enabled_subdevices,
    freenect_device,
    freenect_open_device,
    freenect_open_device_by_camera_serial,
    freenect_close_device,
};

use std::ptr;
use std::ffi;
use std::sync::Arc;

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

pub struct DeviceAttributes {
    camera_serial: String,
}

struct InnerContext {
    ctx: *mut freenect_context,
}

impl InnerContext {
    fn new() -> FreenectResult<InnerContext> {
        let mut ctx = InnerContext{ctx: ptr::null_mut()};

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
}

impl Drop for InnerContext {
    fn drop(&mut self) {
        let ret = unsafe { freenect_shutdown(self.ctx) };

        if ret < 0 {
            panic!(ret)
        }
    }
}

pub struct Context {
    ctx: Arc<InnerContext>,
}

impl Context {
    pub fn new() -> FreenectResult<Context> {
        let inner_ctx = try!(InnerContext::new());

        Ok(Context{ctx: Arc::new(inner_ctx)})
    }

    pub fn set_log_level(&mut self, level: LogLevel) {
        unsafe { freenect_set_log_level(self.ctx.ctx, level.to_lowlevel()); }
    }

    // pub fn set_log_callback(&mut self) {
    //
    // }
    //
    // // Convert C callback parameters to rustified versions, then call user callback
    // fn callback_trampoline(dev: *mut freenect_context, level: freenect_loglevel, msg: *const c_char) {
    //     let level = LogLevel::from_lowlevel(level);
    //     let msg = unsafe { ffi::CStr::from_ptr(msg) };
    // }

    pub fn process_events(&mut self) -> FreenectResult<()> {
        match unsafe { freenect_process_events(self.ctx.ctx) } {
            0 => Ok(()),
            x => Err(FreenectError::LibraryReturnCode(x)),
        }
    }

    pub fn num_devices(&mut self) -> FreenectResult<u32> {
        let ret = unsafe { freenect_num_devices(self.ctx.ctx) };
        if ret < 0 {
            Err(FreenectError::LibraryReturnCode(ret))
        } else {
            Ok(ret as u32)
        }
    }

    pub fn list_device_attributes(&mut self) -> FreenectResult<Vec<DeviceAttributes>> {
        let mut lowlevel_list: *mut freenect_device_attributes = ptr::null_mut();

        let ret = unsafe { freenect_list_device_attributes(self.ctx.ctx, &mut lowlevel_list) };
        if ret < 0 {
            return Err(FreenectError::LibraryReturnCode(ret));
        }

        let mut device_list: Vec<DeviceAttributes> = Vec::new();

        let mut curr_item = lowlevel_list;
        while curr_item != ptr::null_mut() {
            let serial_cstr = unsafe { ffi::CStr::from_ptr((*curr_item).camera_serial) };
            let serial = String::from_utf8_lossy(serial_cstr.to_bytes()).to_string();

            device_list.push(DeviceAttributes{camera_serial: serial});
            unsafe { curr_item = (*curr_item).next };
        }

        unsafe { freenect_free_device_attributes(self.ctx.ctx, lowlevel_list) };

        Ok(device_list)
    }

    fn select_subdevices(&mut self, subdevs: DeviceFlags) {
        unsafe { freenect_select_subdevices(self.ctx.ctx, subdevs.bits) };
    }

    fn enabled_subdevices(&mut self) -> DeviceFlags {
        let ret = unsafe { freenect_enabled_subdevices(self.ctx.ctx) };

        return DeviceFlags::from_bits(ret as u32).unwrap();
    }

    pub fn open_device(&mut self, index: u32, subdevs: DeviceFlags) -> FreenectResult<Device> {
        let mut dev: *mut freenect_device = ptr::null_mut();

        self.select_subdevices(subdevs);

        let ret = unsafe { freenect_open_device(self.ctx.ctx, &mut dev, index as i32) };
        if ret < 0 {
            return Err(FreenectError::LibraryReturnCode(ret))
        }

        return Ok(Device::from_raw_device(self.ctx.clone(), dev, self.enabled_subdevices()));
    }

    pub fn open_device_by_camera_serial(&mut self, serial: &str, subdevs: DeviceFlags) -> FreenectResult<Device> {
        let mut dev: *mut freenect_device = ptr::null_mut();

        let serial_cstring = ffi::CString::new(serial).unwrap();

        self.select_subdevices(subdevs);

        let ret = unsafe { freenect_open_device_by_camera_serial(self.ctx.ctx, &mut dev, serial_cstring.as_ptr()) };
        if ret < 0 {
            return Err(FreenectError::LibraryReturnCode(ret))
        }

        return Ok(Device::from_raw_device(self.ctx.clone(), dev, self.enabled_subdevices()));
    }
}

pub struct MotorSubdevice {
    dev: *mut freenect_device,
}

pub struct CameraSubdevice {
    dev: *mut freenect_device,
}

pub struct AudioSubdevice {
    dev: *mut freenect_device,
}

pub struct Device {
    ctx: Arc<InnerContext>, // Handle to prevent underlying context being free'd before device
    dev: *mut freenect_device,
    pub motor:  Option<MotorSubdevice>,
    pub camera: Option<CameraSubdevice>,
    pub audio:  Option<AudioSubdevice>,
}

impl Device {
    fn from_raw_device(ctx: Arc<InnerContext>, dev: *mut freenect_device, subdevs: DeviceFlags) -> Device {
        Device {
            ctx: ctx,
            dev: dev,
            motor:  if subdevs.contains(DEVICE_MOTOR)  { Some(MotorSubdevice{dev: dev})  } else { None },
            camera: if subdevs.contains(DEVICE_CAMERA) { Some(CameraSubdevice{dev: dev}) } else { None },
            audio:  if subdevs.contains(DEVICE_AUDIO)  { Some(AudioSubdevice{dev: dev})  } else { None },
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        let ret = unsafe { freenect_close_device(self.dev) };

        if ret < 0 {
            panic!(ret)
        }
    }
}

pub fn supported_subdevices() -> DeviceFlags {
    let bits = unsafe { freenect_supported_subdevices() as u32 };
    return DeviceFlags::from_bits(bits).unwrap();
}
