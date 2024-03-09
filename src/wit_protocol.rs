// quaternion
#![allow(dead_code)]

use defmt::Format;
extern crate alloc;
use alloc::vec::Vec;

// 16-bit signed integer (between −32,768 to 32,767), normalized by dividing by 32,768)
const NORM: f32 = u16::MAX as f32;
const ACC_FACTOR: f32 = 16.0 / NORM;
const GYRO_FACTOR: f32 = 2000.0 / NORM;
const QUAT_FACTOR: f32 = 1.0 / NORM;
const ANGLE_FACTOR: f32 = 180.0 / NORM;

#[derive(Format)]
pub struct SAcc {
    acc0: u16,
    acc1: u16,
    acc2: u16,
}

impl SAcc {
    fn new(data: Vec<u8>) -> Self {
        Self {
            acc0: u16::from_le_bytes([data[0], data[1]]),
            acc1: u16::from_le_bytes([data[2], data[3]]),
            acc2: u16::from_le_bytes([data[4], data[5]]),
        }
    }
}

#[derive(Format)]
pub struct SGyro {
    w0: u16,
    w1: u16,
    w2: u16,
}

impl SGyro {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            w0: u16::from_le_bytes([data[0], data[1]]),
            w1: u16::from_le_bytes([data[2], data[3]]),
            w2: u16::from_le_bytes([data[4], data[5]]),
        }
    }
}

#[derive(Format)]
pub struct SAngle {
    ang0: u16,
    ang1: u16,
    ang2: u16,
}

impl SAngle {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            ang0: u16::from_le_bytes([data[0], data[1]]),
            ang1: u16::from_le_bytes([data[2], data[3]]),
            ang2: u16::from_le_bytes([data[4], data[5]]),
        }
    }
}

#[derive(Format)]
pub struct SMag {
    h0: u16,
    h1: u16,
    h2: u16,
}

impl SMag {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            h0: u16::from_le_bytes([data[0], data[1]]),
            h1: u16::from_le_bytes([data[2], data[3]]),
            h2: u16::from_le_bytes([data[4], data[5]]),
        }
    }
}

#[derive(Format)]
pub struct SDStatus {
    s0: u16,
    s1: u16,
    s2: u16,
    s3: u16,
}

impl SDStatus {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            s0: u16::from_le_bytes([data[0], data[1]]),
            s1: u16::from_le_bytes([data[2], data[3]]),
            s2: u16::from_le_bytes([data[4], data[5]]),
            s3: u16::from_le_bytes([data[6], data[7]]),
        }
    }
}

#[derive(Format)]
pub struct SPress {
    l_pressure: u32,
    l_altitude: u32,
}

impl SPress {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            l_pressure: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            l_altitude: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        }
    }
}

#[derive(Format)]
pub struct SLonLat {
    l_lon: u32,
    l_lat: u32,
}

impl SLonLat {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            l_lon: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            l_lat: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        }
    }
}

#[derive(Format)]
pub struct SGPSV {
    s_gpsheight: u16,
    s_gpsyaw: u16,
    l_gpsvelocity: u32,
}

impl SGPSV {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            s_gpsheight: u16::from_le_bytes([data[0], data[1]]),
            s_gpsyaw: u16::from_le_bytes([data[2], data[3]]),
            l_gpsvelocity: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        }
    }
}

#[derive(Format)]
pub struct SQuat {
    q0: u16,
    q1: u16,
    q2: u16,
    q3: u16,
}

impl SQuat {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            q0: u16::from_le_bytes([data[0], data[1]]),
            q1: u16::from_le_bytes([data[2], data[3]]),
            q2: u16::from_le_bytes([data[4], data[5]]),
            q3: u16::from_le_bytes([data[6], data[7]]),
        }
    }
}

const TIME: u8 = 0x50;
const ACCELERATION: u8 = 0x51;
const ANGULAR_VELOCITY: u8 = 0x52;
const ANGLE: u8 = 0x53;
const MAGNETIC_FIELD: u8 = 0x54;
const PORT: u8 = 0x55;
const BAROMETRIC_ALTITUDE: u8 = 0x56;
const LATITUDE_LONGITUDE: u8 = 0x57;
const GROUND_SPEED: u8 = 0x58;
const QUATERNION: u8 = 0x59;
const GPS_LOCATION_ACCURACY: u8 = 0x5A;
const READ: u8 = 0x5F;

#[derive(Format)]
pub enum WITData {
    Acc(SAcc),
    Gyro(SGyro),
    Angle(SAngle),
    Mag(SMag),
    DStatus(SDStatus),
    Press(SPress),
    LonLat(SLonLat),
    Gpsv(SGPSV),
    Quat(SQuat),
}

pub fn get_wit_data(data_type: u8, data: Vec<u8>) -> Option<WITData> {
    defmt::debug!("Data type: {:02x}", data_type);

    use WITData::*;
    match data_type {
        TIME => Some(Acc(SAcc::new(data))),
        ANGULAR_VELOCITY => Some(Gyro(SGyro::new(data))),
        ANGLE => Some(Angle(SAngle::new(data))),
        MAGNETIC_FIELD => Some(Mag(SMag::new(data))),
        PORT => Some(DStatus(SDStatus::new(data))),
        BAROMETRIC_ALTITUDE => Some(Press(SPress::new(data))),
        LATITUDE_LONGITUDE => Some(LonLat(SLonLat::new(data))),
        GROUND_SPEED => Some(Mag(SMag::new(data))),
        QUATERNION => Some(Mag(SMag::new(data))),
        _ => None,
    }
}
