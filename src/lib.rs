use std::ffi;
use std::ptr;
use std::rc::Rc;
use std::cell::RefCell;
use std::slice;

extern crate libc;
use libc::{
    c_int,
    c_void,
    uint32_t,
    int32_t,
};

#[macro_use]
extern crate bitflags;

extern crate libfreenect_sys;
use libfreenect_sys as ft;

#[derive(Debug)]
enum FreenectError {
    LibraryReturnCode(i32),
    NullPtr,
    FrameFormatMismatch,
}

// Error type for the library
pub type FreenectResult<T> = Result<T, FreenectError>;

#[derive(Debug)]
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
    fn to_lowlevel(&self) -> ft::freenect_loglevel {
        match *self {
            LogLevel::Fatal     => ft::freenect_loglevel::FREENECT_LOG_FATAL,
            LogLevel::Error     => ft::freenect_loglevel::FREENECT_LOG_ERROR,
            LogLevel::Warning   => ft::freenect_loglevel::FREENECT_LOG_WARNING,
            LogLevel::Notice    => ft::freenect_loglevel::FREENECT_LOG_NOTICE,
            LogLevel::Info      => ft::freenect_loglevel::FREENECT_LOG_INFO,
            LogLevel::Debug     => ft::freenect_loglevel::FREENECT_LOG_DEBUG,
            LogLevel::Spew      => ft::freenect_loglevel::FREENECT_LOG_SPEW,
            LogLevel::Flood     => ft::freenect_loglevel::FREENECT_LOG_FLOOD,
        }
    }

    fn from_lowlevel(lvl: ft::freenect_loglevel) -> LogLevel {
        match lvl {
            ft::freenect_loglevel::FREENECT_LOG_FATAL   => LogLevel::Fatal,
            ft::freenect_loglevel::FREENECT_LOG_ERROR   => LogLevel::Error,
            ft::freenect_loglevel::FREENECT_LOG_WARNING => LogLevel::Warning,
            ft::freenect_loglevel::FREENECT_LOG_NOTICE  => LogLevel::Notice,
            ft::freenect_loglevel::FREENECT_LOG_INFO    => LogLevel::Info,
            ft::freenect_loglevel::FREENECT_LOG_DEBUG   => LogLevel::Debug,
            ft::freenect_loglevel::FREENECT_LOG_SPEW    => LogLevel::Spew,
            ft::freenect_loglevel::FREENECT_LOG_FLOOD   => LogLevel::Flood,
        }
    }
}

#[derive(Debug)]
pub enum Resolution {
    Low,
    Medium,
    High,
}

impl Resolution {
    fn to_lowlevel(&self) -> ft::freenect_resolution {
        match *self {
            Resolution::Low     => ft::freenect_resolution::FREENECT_RESOLUTION_LOW,
            Resolution::Medium  => ft::freenect_resolution::FREENECT_RESOLUTION_MEDIUM,
            Resolution::High    => ft::freenect_resolution::FREENECT_RESOLUTION_HIGH,
        }
    }

    fn from_lowlevel(res: &ft::freenect_resolution) -> Resolution {
        match *res {
            ft::freenect_resolution::FREENECT_RESOLUTION_LOW    => Resolution::Low,
            ft::freenect_resolution::FREENECT_RESOLUTION_MEDIUM => Resolution::Medium,
            ft::freenect_resolution::FREENECT_RESOLUTION_HIGH   => Resolution::High,
            _ => panic!("Unknown freenect_resolution enum")
        }
    }
}

#[derive(Debug)]
pub enum VideoFormat {
    Rgb,
    Bayer,
    Ir8Bit,
    Ir10Bit,
    Ir10BitPacked,
    YuvRgb,
    YuvRaw,
}

impl VideoFormat {
    fn to_lowlevel(&self) -> ft::freenect_video_format {
        match *self {
            VideoFormat::Rgb            => ft::freenect_video_format::FREENECT_VIDEO_RGB,
            VideoFormat::Bayer          => ft::freenect_video_format::FREENECT_VIDEO_BAYER,
            VideoFormat::Ir8Bit         => ft::freenect_video_format::FREENECT_VIDEO_IR_8BIT,
            VideoFormat::Ir10Bit        => ft::freenect_video_format::FREENECT_VIDEO_IR_10BIT,
            VideoFormat::Ir10BitPacked  => ft::freenect_video_format::FREENECT_VIDEO_IR_10BIT_PACKED,
            VideoFormat::YuvRgb         => ft::freenect_video_format::FREENECT_VIDEO_YUV_RGB,
            VideoFormat::YuvRaw         => ft::freenect_video_format::FREENECT_VIDEO_YUV_RAW,
        }
    }

    fn from_lowlevel_int(i: int32_t) -> VideoFormat {
        match i {
            0 => VideoFormat::Rgb,
            1 => VideoFormat::Bayer,
            2 => VideoFormat::Ir8Bit,
            3 => VideoFormat::Ir10Bit,
            4 => VideoFormat::Ir10BitPacked,
            5 => VideoFormat::YuvRgb,
            6 => VideoFormat::YuvRaw,
            _ => panic!("Unknown freenect_video_format enum"),
        }
    }
}

#[derive(Debug)]
pub enum DepthFormat {
    _11Bit,
    _10Bit,
    _11BitPacked,
    _10BitPacked,
    Registered,
    Mm
}

impl DepthFormat {
    fn to_lowlevel(&self) -> ft::freenect_depth_format {
        match *self {
            DepthFormat::_11Bit         => ft::freenect_depth_format::FREENECT_DEPTH_11BIT,
            DepthFormat::_10Bit         => ft::freenect_depth_format::FREENECT_DEPTH_10BIT,
            DepthFormat::_11BitPacked   => ft::freenect_depth_format::FREENECT_DEPTH_11BIT_PACKED,
            DepthFormat::_10BitPacked   => ft::freenect_depth_format::FREENECT_DEPTH_10BIT_PACKED,
            DepthFormat::Registered     => ft::freenect_depth_format::FREENECT_DEPTH_REGISTERED,
            DepthFormat::Mm             => ft::freenect_depth_format::FREENECT_DEPTH_MM,
        }
    }

    fn from_lowlevel_int(i: int32_t) -> DepthFormat {
        match i {
            0 => DepthFormat::_11Bit,
        	1 => DepthFormat::_10Bit,
        	2 => DepthFormat::_11BitPacked,
        	3 => DepthFormat::_10BitPacked,
        	4 => DepthFormat::Registered,
        	5 => DepthFormat::Mm,
        	_ => panic!("Unknown freenect_depth_format enum"),
        }
    }
}

#[derive(Debug)]
pub enum FrameModeFormat {
    Video(VideoFormat),
    Depth(DepthFormat),
}

#[derive(Debug)]
pub struct FrameMode {
    reserved: uint32_t, // Need to track contents of underlying freenect struct
    pub resolution: Resolution,
    pub format: FrameModeFormat,
    pub bytes: i32,
    pub width: i16,
    pub height: i16,
    pub data_bits_per_pixel: i8,
    pub padding_bits_per_pixel: i8,
    pub framerate: i8,
    pub is_valid: bool,
}

impl FrameMode {
    fn to_lowlevel(&self) -> ft::freenect_frame_mode {
        ft::freenect_frame_mode {
            reserved: self.reserved,
            resolution: self.resolution.to_lowlevel(),
            dummy: match self.format {
                FrameModeFormat::Video(ref x) => x.to_lowlevel() as int32_t,
                FrameModeFormat::Depth(ref y) => y.to_lowlevel() as int32_t,
            },
            bytes: self.bytes,
            width: self.width,
            height: self.height,
            data_bits_per_pixel: self.data_bits_per_pixel,
            padding_bits_per_pixel: self.padding_bits_per_pixel,
            framerate: self.framerate,
            is_valid: if self.is_valid { 1 } else { 0 },
        }
    }

    fn to_lowlevel_video(&self) -> Option<ft::freenect_frame_mode> {
        match self.format {
            FrameModeFormat::Video(_) => Some(self.to_lowlevel()),
            FrameModeFormat::Depth(_) => None,
        }
    }

    fn to_lowlevel_depth(&self) -> Option<ft::freenect_frame_mode> {
        match self.format {
            FrameModeFormat::Video(_) => None,
            FrameModeFormat::Depth(_) => Some(self.to_lowlevel()),
        }
    }

    fn from_lowlevel(mode: &ft::freenect_frame_mode, fmt: FrameModeFormat) -> FrameMode {
        FrameMode {
            reserved: mode.reserved,
            resolution: Resolution::from_lowlevel(&mode.resolution),
            format: fmt,
            bytes: mode.bytes as i32,
            width: mode.width as i16,
            height: mode.height as i16,
            data_bits_per_pixel: mode.data_bits_per_pixel as i8,
            padding_bits_per_pixel: mode.padding_bits_per_pixel as i8,
            framerate: mode.framerate as i8,
            is_valid: if mode.is_valid > 0 { true } else { false },
        }
    }

    fn from_lowlevel_video(mode: &ft::freenect_frame_mode) -> FrameMode {
        FrameMode::from_lowlevel(mode, FrameModeFormat::Video(VideoFormat::from_lowlevel_int(mode.dummy)))
    }

    fn from_lowlevel_depth(mode: &ft::freenect_frame_mode) -> FrameMode {
        FrameMode::from_lowlevel(mode, FrameModeFormat::Depth(DepthFormat::from_lowlevel_int(mode.dummy)))
    }
}

pub enum TiltStatus {
    Stopped,
    Limit,
    Moving,
}

impl TiltStatus {
    fn from_lowlevel(status: &ft::freenect_tilt_status_code) -> TiltStatus {
        match *status {
            ft::freenect_tilt_status_code::TILT_STATUS_STOPPED => TiltStatus::Stopped,
            ft::freenect_tilt_status_code::TILT_STATUS_LIMIT   => TiltStatus::Limit,
            ft::freenect_tilt_status_code::TILT_STATUS_MOVING  => TiltStatus::Moving,
        }
    }
}

pub struct RawTiltState {
    pub accelerometer_x: i16,
    pub accelerometer_y: i16,
    pub accelerometer_z: i16,
    pub tilt_angle: i8,
    pub tilt_status: TiltStatus,
}

impl RawTiltState {
    fn from_lowlevel(state: *const ft::freenect_raw_tilt_state) -> RawTiltState {
        let state = unsafe { &*state };
        RawTiltState{
            accelerometer_x: state.accelerometer_x,
            accelerometer_y: state.accelerometer_y,
            accelerometer_z: state.accelerometer_z,
            tilt_angle: state.tilt_angle,
            tilt_status: TiltStatus::from_lowlevel(&state.tilt_status),
        }
    }
}

pub enum Flag {
    AutoExposure,
    AutoWhiteBalance,
    RawColor,
    MirrorDepth,
    MirrorVideo,
}

impl Flag {
    fn to_lowlevel(&self) -> ft::freenect_flag {
        match *self {
            Flag::AutoExposure        => ft::freenect_flag::FREENECT_AUTO_EXPOSURE,
            Flag::AutoWhiteBalance    => ft::freenect_flag::FREENECT_AUTO_WHITE_BALANCE,
            Flag::RawColor            => ft::freenect_flag::FREENECT_RAW_COLOR,
            Flag::MirrorDepth         => ft::freenect_flag::FREENECT_MIRROR_DEPTH,
            Flag::MirrorVideo         => ft::freenect_flag::FREENECT_MIRROR_VIDEO,
        }
    }
}

bitflags! {
    flags DeviceFlags: u32 {
        const DEVICE_MOTOR  = ft::freenect_device_flags::FREENECT_DEVICE_MOTOR  as u32,
        const DEVICE_CAMERA = ft::freenect_device_flags::FREENECT_DEVICE_CAMERA as u32,
        const DEVICE_AUDIO  = ft::freenect_device_flags::FREENECT_DEVICE_AUDIO  as u32,
    }
}

#[derive(Debug)]
pub struct DeviceAttributes {
    pub camera_serial: String,
}

struct InnerContext {
    ctx: *mut ft::freenect_context,
}

// InnerContext separated from main Context so that 'Device' handles can hold a reference to the
// InnerContext to prevent premature release. Could also use lifetimes (probably) to statically
// enforce this.
impl InnerContext {
    fn new() -> FreenectResult<InnerContext> {
        let mut ctx = InnerContext{ctx: ptr::null_mut()};

        match unsafe { ft::freenect_init(&mut ctx.ctx, ptr::null_mut()) } {
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
        let ret = unsafe { ft::freenect_shutdown(self.ctx) };

        if ret < 0 {
            panic!(ret)
        }
    }
}

pub struct Context {
    ctx: Rc<InnerContext>,
}

impl Context {
    pub fn new() -> FreenectResult<Context> {
        let inner_ctx = try!(InnerContext::new());

        Ok(Context{ctx: Rc::new(inner_ctx)})
    }

    pub fn set_log_level(&mut self, level: LogLevel) {
        unsafe { ft::freenect_set_log_level(self.ctx.ctx, level.to_lowlevel()); }
    }

    pub fn process_events(&mut self) -> FreenectResult<()> {
        match unsafe { ft::freenect_process_events(self.ctx.ctx) } {
            0 => Ok(()),
            x => Err(FreenectError::LibraryReturnCode(x)),
        }
    }

    // FIXME: Implement process_events with timeout

    pub fn num_devices(&mut self) -> FreenectResult<u32> {
        let ret = unsafe { ft::freenect_num_devices(self.ctx.ctx) };
        if ret < 0 {
            Err(FreenectError::LibraryReturnCode(ret))
        } else {
            Ok(ret as u32)
        }
    }

    pub fn list_device_attributes(&mut self) -> FreenectResult<Vec<DeviceAttributes>> {
        let mut lowlevel_list: *mut ft::freenect_device_attributes = ptr::null_mut();

        let ret = unsafe { ft::freenect_list_device_attributes(self.ctx.ctx, &mut lowlevel_list) };
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

        unsafe { ft::freenect_free_device_attributes(lowlevel_list) };

        Ok(device_list)
    }

    // Internal use only
    fn select_subdevices(&mut self, subdevs: DeviceFlags) {
        unsafe { ft::freenect_select_subdevices(self.ctx.ctx, subdevs.bits) };
    }

    // Internal use only
    fn enabled_subdevices(&mut self) -> DeviceFlags {
        let ret = unsafe { ft::freenect_enabled_subdevices(self.ctx.ctx) };

        return DeviceFlags::from_bits(ret as u32).unwrap();
    }

    pub fn open_device(&mut self, index: u32, subdevs: DeviceFlags) -> FreenectResult<Device> {
        let mut dev: *mut ft::freenect_device = ptr::null_mut();

        self.select_subdevices(subdevs);

        let ret = unsafe { ft::freenect_open_device(self.ctx.ctx, &mut dev, index as i32) };
        if ret < 0 {
            return Err(FreenectError::LibraryReturnCode(ret))
        }

        return Ok(Device::from_raw_device(self.ctx.clone(), dev, self.enabled_subdevices()));
    }

    pub fn open_device_by_camera_serial(&mut self, serial: &str, subdevs: DeviceFlags) -> FreenectResult<Device> {
        let mut dev: *mut ft::freenect_device = ptr::null_mut();

        let serial_cstring = ffi::CString::new(serial).unwrap();

        self.select_subdevices(subdevs);

        let ret = unsafe { ft::freenect_open_device_by_camera_serial(self.ctx.ctx, &mut dev, serial_cstring.as_ptr()) };
        if ret < 0 {
            return Err(FreenectError::LibraryReturnCode(ret))
        }

        return Ok(Device::from_raw_device(self.ctx.clone(), dev, self.enabled_subdevices()));
    }
}

// Rust struct allowing methods to be attached to the underyling C struct
struct CDevice {
    dev: *mut ft::freenect_device,
}

impl Drop for CDevice {
    fn drop(&mut self) {
        let ret = unsafe { ft::freenect_close_device(self.dev) };

        if ret != 0 {
            panic!(ret)
        }
    }
}

impl CDevice {
    fn from_raw_device(dev: *mut ft::freenect_device) -> CDevice {
        CDevice{dev: dev}
    }

    fn set_user(&mut self, user: *mut c_void) {
        unsafe { ft::freenect_set_user(self.dev, user) };
    }

    fn set_depth_callback(&mut self, cb: ft::freenect_depth_cb) {
        unsafe { ft::freenect_set_depth_callback(self.dev, cb) };
    }

    fn set_video_callback(&mut self, cb: ft::freenect_video_cb) {
        unsafe { ft::freenect_set_video_callback(self.dev, cb) };
    }

    fn start_depth(&mut self) -> FreenectResult<()> {
        let ret = unsafe { ft::freenect_start_depth(self.dev) };

        if ret == 0 {
            Ok(())
        } else {
            Err(FreenectError::LibraryReturnCode(ret))
        }
    }

    fn start_video(&mut self) -> FreenectResult<()> {
        let ret = unsafe { ft::freenect_start_video(self.dev) };

        if ret == 0 {
            Ok(())
        } else {
            Err(FreenectError::LibraryReturnCode(ret))
        }
    }

    fn stop_depth(&mut self) -> FreenectResult<()> {
        let ret = unsafe { ft::freenect_stop_depth(self.dev) };

        if ret == 0 {
            Ok(())
        } else {
            Err(FreenectError::LibraryReturnCode(ret))
        }
    }

    fn stop_video(&mut self) -> FreenectResult<()> {
        let ret = unsafe { ft::freenect_stop_video(self.dev) };

        if ret == 0 {
            Ok(())
        } else {
            Err(FreenectError::LibraryReturnCode(ret))
        }
    }

    fn update_tilt_state(&mut self) -> FreenectResult<()> {
        let ret = unsafe { ft::freenect_update_tilt_state(self.dev) };

        if ret == 0 {
            Err(FreenectError::LibraryReturnCode(ret))
        } else {
            Ok(())
        }
    }

    fn get_tilt_state(&mut self) -> *mut ft::freenect_raw_tilt_state {
        unsafe { ft::freenect_get_tilt_state(self.dev) }
    }

    fn set_tilt_degs(&mut self, angle: f64) -> FreenectResult<()> {
        let ret = unsafe { ft::freenect_set_tilt_degs(self.dev, angle) };

        if ret == 0 {
            Err(FreenectError::LibraryReturnCode(ret))
        } else {
            Ok(())
        }
    }

    fn get_current_video_mode(&mut self) -> FrameMode {
        let lowlevel_video_mode = unsafe { ft::freenect_get_current_video_mode(self.dev) };
        FrameMode::from_lowlevel_video(&lowlevel_video_mode)
    }

    fn set_video_mode(&mut self, mode: FrameMode) -> FreenectResult<()> {
        let lowlevel_video_mode = try!(mode.to_lowlevel_video().ok_or(FreenectError::FrameFormatMismatch));
        unsafe { ft::freenect_set_video_mode(self.dev, lowlevel_video_mode) };
        Ok(())
    }

    fn get_current_depth_mode(&mut self) -> FrameMode {
        let lowlevel_depth_mode = unsafe { ft::freenect_get_current_depth_mode(self.dev) };
        FrameMode::from_lowlevel_depth(&lowlevel_depth_mode)
    }

    fn set_depth_mode(&mut self, mode: FrameMode) -> FreenectResult<()> {
        let lowlevel_depth_mode = try!(mode.to_lowlevel_depth().ok_or(FreenectError::FrameFormatMismatch));
        unsafe { ft::freenect_set_depth_mode(self.dev, lowlevel_depth_mode) };
        Ok(())
    }

    fn set_flag(&mut self, flag: Flag, set: bool) -> FreenectResult<()> {
        let flag_value = if set {
            ft::freenect_flag_value::FREENECT_ON
        } else {
            ft::freenect_flag_value::FREENECT_OFF
        };

        let ret = unsafe { ft::freenect_set_flag(self.dev, flag.to_lowlevel(), flag_value) };

        if ret == 0 {
            Err(FreenectError::LibraryReturnCode(ret))
        } else {
            Ok(())
        }
    }
}

#[allow(dead_code)]
pub struct Device {
    ctx: Rc<InnerContext>, // Handle to prevent underlying context being free'd before device
    dev: Rc<RefCell<CDevice>>,
    pub motor:  Option<MotorSubdevice>,
    pub camera: Option<CameraSubdevice>,
    pub audio:  Option<AudioSubdevice>,
}

impl Device {
    fn from_raw_device(ctx: Rc<InnerContext>, dev: *mut ft::freenect_device, subdevs: DeviceFlags) -> Device {
        let inner_dev = Rc::new(RefCell::new(CDevice::from_raw_device(dev)));

        Device {
            ctx: ctx,
            dev: inner_dev.clone(),
            motor:  if subdevs.contains(DEVICE_MOTOR)  { Some(MotorSubdevice::new(inner_dev.clone()))  } else { None },
            camera: if subdevs.contains(DEVICE_CAMERA) { Some(CameraSubdevice::new(inner_dev.clone())) } else { None },
            audio:  if subdevs.contains(DEVICE_AUDIO)  { Some(AudioSubdevice::new(inner_dev.clone()))  } else { None },
        }
    }
}

pub struct MotorSubdevice {
    dev: Rc<RefCell<CDevice>>,
}

impl MotorSubdevice {
    fn new(dev: Rc<RefCell<CDevice>>) -> MotorSubdevice {
        MotorSubdevice{dev: dev}
    }

    pub fn get_tilt_state(&mut self) -> FreenectResult<RawTiltState> {
        let mut cdev = self.dev.borrow_mut();

        try!(cdev.update_tilt_state());
        Ok(RawTiltState::from_lowlevel(cdev.get_tilt_state()))
    }
}

// Exists so it can be boxed (therefore fixing its memory address) and have its address handed as a
// C callback userdata void pointer
struct ClosureHolder {
    dev: Rc<RefCell<CDevice>>,
    depth_cb: Option<Box<FnMut(&FrameMode, &mut [u16], u32) + Send + 'static>>,
    video_cb: Option<Box<FnMut(&FrameMode, &mut [u8], u32) + Send + 'static>>,
    starting: bool,
}

impl ClosureHolder {
    fn new(dev: Rc<RefCell<CDevice>>) -> ClosureHolder {
        ClosureHolder{dev: dev, depth_cb: None, video_cb: None, starting: false}
    }
}

pub struct CameraSubdevice {
    dev: Rc<RefCell<CDevice>>,
    ch: Box<ClosureHolder>,
}

impl CameraSubdevice {
    fn new(dev: Rc<RefCell<CDevice>>) -> CameraSubdevice {
        let mut cam_sub = CameraSubdevice{ dev: dev.clone(), ch: Box::new(ClosureHolder::new(dev.clone()))};

        // Register all callbacks. We'll let Rust code decide if a user callback should be called.
        unsafe {
            let mut cdev = dev.borrow_mut();
            cdev.set_user(std::mem::transmute(&mut *cam_sub.ch));
            cdev.set_depth_callback(CameraSubdevice::depth_cb_trampoline);
            cdev.set_video_callback(CameraSubdevice::video_cb_trampoline);
        }

        return cam_sub;
    }

    pub fn set_depth_callback(&mut self, cb: Option<Box<FnMut(&FrameMode, &mut [u16], u32) + Send + 'static>>) {
        self.ch.depth_cb = cb;
    }

    pub fn set_video_callback(&mut self, cb: Option<Box<FnMut(&FrameMode, &mut [u8], u32) + Send + 'static>>) {
        self.ch.video_cb = cb;
    }

    pub fn start_depth(&mut self) -> FreenectResult<()> {
        (*self.ch).starting = true;
        let ret = self.dev.borrow_mut().start_depth();
        (*self.ch).starting = false;
        return ret;
    }

    pub fn start_video(&mut self) -> FreenectResult<()> {
        (*self.ch).starting = true;
        let ret = self.dev.borrow_mut().start_video();
        (*self.ch).starting = false;
        return ret;
    }

    pub fn stop_depth(&mut self) -> FreenectResult<()> {
        self.dev.borrow_mut().stop_depth()
    }

    pub fn stop_video(&mut self) -> FreenectResult<()> {
        self.dev.borrow_mut().stop_video()
    }

    pub fn set_tilt_degs(&mut self, angle: f64) -> FreenectResult<()> {
        self.dev.borrow_mut().set_tilt_degs(angle)
    }

    pub fn get_current_video_mode(&mut self) -> FrameMode {
        self.dev.borrow_mut().get_current_video_mode()
    }

    pub fn set_video_mode(&mut self, mode: FrameMode) -> FreenectResult<()> {
        self.dev.borrow_mut().set_video_mode(mode)
    }

    pub fn get_current_depth_mode(&mut self) -> FrameMode {
        self.dev.borrow_mut().get_current_depth_mode()
    }

    pub fn set_depth_mode(&mut self, mode: FrameMode) -> FreenectResult<()> {
        self.dev.borrow_mut().set_depth_mode(mode)
    }

    pub fn set_flag(&mut self, flag: Flag, set: bool) -> FreenectResult<()> {
        self.dev.borrow_mut().set_flag(flag, set)
    }

    extern "C" fn depth_cb_trampoline(dev: *mut ft::freenect_device, depth: *mut c_void, timestamp: uint32_t) {
        unsafe {
            let ch = ft::freenect_get_user(dev) as *mut ClosureHolder;

            // libfreenect end's up calling this callback when start_depth is called. This is an
            // issue as the cdev RefCell will be borrowed twice (causing a panic). Instead, check a
            // flag indicating we are starting, and if set, just ignore the frame.

            if !(*ch).starting {
                // Callback provides no information on frame buffer length. Retrieve the length by
                // directly asking for the current mode information
                let mode = (*ch).dev.borrow_mut().get_current_depth_mode();

                let frame = slice::from_raw_parts_mut(depth as *mut u16, mode.bytes as usize);
                let timestamp = timestamp as u32;

                match (*ch).depth_cb {
                    Some(ref mut cb) => cb(&mode, frame, timestamp),
                    None => return,
                };
            }
        }
    }

    extern "C" fn video_cb_trampoline(dev: *mut ft::freenect_device, video: *mut c_void, timestamp: uint32_t) {
        unsafe {
            let ch = ft::freenect_get_user(dev) as *mut ClosureHolder;

            if !(*ch).starting {
                // Callback provides no information on frame buffer length. Retrieve the length by
                // directly asking for the current mode information
                let mode = (*ch).dev.borrow_mut().get_current_video_mode();

                let frame = slice::from_raw_parts_mut(video as *mut u8, mode.bytes as usize);
                let timestamp = timestamp as u32;

                match (*ch).video_cb {
                    Some(ref mut cb) => cb(&mode, frame, timestamp),
                    None => return,
                };
            }
        }
    }
}

#[allow(dead_code)]
pub struct AudioSubdevice {
    dev: Rc<RefCell<CDevice>>,
}

impl AudioSubdevice {
    fn new(dev: Rc<RefCell<CDevice>>) -> AudioSubdevice {
        AudioSubdevice{dev: dev}
    }
}

pub fn supported_subdevices() -> DeviceFlags {
    let bits = unsafe { ft::freenect_supported_subdevices() as u32 };
    return DeviceFlags::from_bits(bits).unwrap();
}

pub fn find_video_mode(res: Resolution, fmt: VideoFormat) -> Option<FrameMode> {
    let frame_mode_lowlevel = unsafe { ft::freenect_find_video_mode(res.to_lowlevel(), fmt.to_lowlevel()) };

    let frame_mode = FrameMode::from_lowlevel_video(&frame_mode_lowlevel);

    if frame_mode.is_valid {
        Some(frame_mode)
    } else {
        None
    }
}

pub fn find_depth_mode(res: Resolution, fmt: DepthFormat) -> Option<FrameMode> {
    let frame_mode_lowlevel = unsafe { ft::freenect_find_depth_mode(res.to_lowlevel(), fmt.to_lowlevel()) };

    let frame_mode = FrameMode::from_lowlevel_depth(&frame_mode_lowlevel);

    if frame_mode.is_valid {
        Some(frame_mode)
    } else {
        None
    }
}

pub struct VideoModeIter {
    video_mode_count: c_int,
    next_mode: c_int,
}

impl VideoModeIter {
    fn new() -> VideoModeIter {
        VideoModeIter{
            video_mode_count: unsafe { ft::freenect_get_video_mode_count() },
            next_mode: 0,
        }
    }
}

impl Iterator for VideoModeIter {
    type Item = FrameMode;

    fn next(&mut self) -> Option<FrameMode> {
        if self.next_mode < self.video_mode_count {
            let lowlevel_frame_mode = unsafe { ft::freenect_get_video_mode(self.next_mode) };
            self.next_mode += 1;
            Some(FrameMode::from_lowlevel_video(&lowlevel_frame_mode))
        } else {
            None
        }
    }
}

impl ExactSizeIterator for VideoModeIter {
    fn len(&self) -> usize {
        self.video_mode_count as usize
    }
}

pub fn video_modes() -> VideoModeIter {
    VideoModeIter::new()
}

pub struct DepthModeIter {
    depth_mode_count: c_int,
    next_mode: c_int,
}

impl DepthModeIter {
    fn new() -> DepthModeIter {
        DepthModeIter{
            depth_mode_count: unsafe { ft::freenect_get_depth_mode_count() },
            next_mode: 0,
        }
    }
}

impl Iterator for DepthModeIter {
    type Item = FrameMode;

    fn next(&mut self) -> Option<FrameMode> {
        if self.next_mode < self.depth_mode_count {
            let lowlevel_frame_mode = unsafe { ft::freenect_get_depth_mode(self.next_mode) };
            self.next_mode += 1;
            Some(FrameMode::from_lowlevel_depth(&lowlevel_frame_mode))
        } else {
            None
        }
    }
}

impl ExactSizeIterator for DepthModeIter {
    fn len(&self) -> usize {
        self.depth_mode_count as usize
    }
}

pub fn depth_modes() -> DepthModeIter {
    DepthModeIter::new()
}
