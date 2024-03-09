#[allow(dead_code)]
pub mod registers {
    pub const GCONF: u8 = 0x00;
    pub const GSTAT: u8 = 0x01;
    pub const IOIN: u8 = 0x04;
    pub const X_COMPARE: u8 = 0x05;
    pub const OTP_PROG: u8 = 0x06;
    pub const OTP_READ: u8 = 0x07;
    pub const FACTORY_CONF: u8 = 0x08;
    pub const SHORT_CONF: u8 = 0x09;
    pub const DRV_CONF: u8 = 0x0A;
    pub const GLOBAL_SCALER: u8 = 0x0B;
    pub const OFFSET_READ: u8 = 0x0C;
    pub const IHOLD_IRUN: u8 = 0x10;
    pub const TPOWERDOWN: u8 = 0x11;
    pub const TSTEP: u8 = 0x12;
    pub const TPWMTHRS: u8 = 0x13;
    pub const TCOOLTHRS: u8 = 0x14;
    pub const THIGH: u8 = 0x15;
    pub const XDIRECT: u8 = 0x2D;
    pub const VDCMIN: u8 = 0x33;
    pub const MSLUT0: u8 = 0x60;
    pub const MSLUT1: u8 = 0x61;
    pub const MSLUT2: u8 = 0x62;
    pub const MSLUT3: u8 = 0x63;
    pub const MSLUT4: u8 = 0x64;
    pub const MSLUT5: u8 = 0x65;
    pub const MSLUT6: u8 = 0x66;
    pub const MSLUT7: u8 = 0x67;
    pub const MSLUTSEL: u8 = 0x68;
    pub const MSLUTSTART: u8 = 0x69;
    pub const MSCNT: u8 = 0x6A;
    pub const MSCURACT: u8 = 0x6B;
    pub const CHOPCONF: u8 = 0x6C;
    pub const COOLCONF: u8 = 0x6D;
    pub const DCCTRL: u8 = 0x6E;
    pub const DRV_STATUS: u8 = 0x6F;
    pub const PWMCONF: u8 = 0x70;
    pub const PWM_SCALE: u8 = 0x71;
    pub const PWM_AUTO: u8 = 0x72;
    pub const LOST_STEPS: u8 = 0x73;
}
