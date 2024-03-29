//? documentation https://www.rlocman.ru/i/File/2018/10/16/TMC2160-datasheet_Rev1.00.pdf

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

    /// Global scaling of Motor current. This value is multiplied
    /// to the current scaling in order to adapt a drive to a
    /// certain motor type. This value should be chosen before
    /// tuning other settings, because it also influences
    /// chopper hysteresis.
    /// - 0: Full Scale (or write 256)
    /// - 1 … 31: Not allowed for operation
    /// - 32 … 255: 32/256 … 255/256 of maximum current.
    ///
    /// Hint: Values >128 recommended for best results
    /// (Reset Default = 0)
    pub const GLOBAL_SCALER: u8 = 0x0B;
    pub const OFFSET_READ: u8 = 0x0C;

    /// # Bits 4..0 IHOLD
    ///
    /// Standstill current (0=1/32…31=32/32)
    /// In combination with stealthChop mode, setting
    /// IHOLD=0 allows to choose freewheeling or coil
    /// short circuit for motor stand still.
    ///
    /// # Bits 12..8 IRUN
    ///
    /// Motor run current (0=1/32…31=32/32)
    /// Hint: Choose sense resistors in a way, that normal
    /// IRUN is 16 to 31 for best microstep performance.
    ///
    /// # Bits 19..16 IHOLDDELAY
    /// Controls the number of clock cycles for motor
    /// power down after a motion as soon as standstill is
    /// detected (stst=1) and TPOWERDOWN has expired.
    /// The smooth transition avoids a motor jerk upon
    /// power down.
    /// - 0: instant power down
    /// - 1..15: Delay per current reduction step in multiple
    /// of 2^18 clocks
    pub const IHOLD_IRUN: u8 = 0x10;

    /// Sets the delay time after stand still (stst) of the
    /// motor to motor current power down. Time range is about 0 to
    /// 4 seconds.
    pub const TPOWERDOWN: u8 = 0x11;

    // Actual measured time between two 1/256 microsteps derived
    // from the step input frequency in units of 1/fCLK. Measured
    // value is (2^20)-1 in case of overflow or stand still.
    pub const TSTEP: u8 = 0x12;

    /// This is the upper velocity for stealthChop voltage PWM mode.
    /// TSTEP ≥ TPWMTHRS
    ///  - stealthChop PWM mode is enabled, if configured
    ///  - dcStep is disabled
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
    /// Chopper and driver configuration
    pub const CHOPCONF: u8 = 0x6C;
    pub const COOLCONF: u8 = 0x6D;
    pub const DCCTRL: u8 = 0x6E;
    pub const DRV_STATUS: u8 = 0x6F;
    /// Voltage PWM mode chopper configuration
    pub const PWMCONF: u8 = 0x70;
    pub const PWM_SCALE: u8 = 0x71;
    pub const PWM_AUTO: u8 = 0x72;
    pub const LOST_STEPS: u8 = 0x73;
}
