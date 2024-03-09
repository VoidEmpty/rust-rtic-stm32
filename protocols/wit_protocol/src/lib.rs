#![no_std]

use defmt::Format;
extern crate alloc;

mod parser;
use crate::parser::*;
use serde::Serialize;

// 16-bit signed integer (between −32,768 to 32,767), normalized by dividing by 32,768)
const NORM: f32 = i16::MAX as f32;
const ACC_FACTOR: f32 = (16.0 * 9.8) / NORM;
const GYRO_FACTOR: f32 = 2000.0 / NORM;
const QUAT_FACTOR: f32 = 1.0 / NORM;
const ANGLE_FACTOR: f32 = 180.0 / NORM;

pub struct SAcc {
    acc0: i16,
    acc1: i16,
    acc2: i16,
}

impl SAcc {
    fn new(data: &[u8]) -> Self {
        let (data, acc0) = take_i16_le(data).unwrap_or_default();
        let (data, acc1) = take_i16_le(data).unwrap_or_default();
        let (_, acc2) = take_i16_le(data).unwrap_or_default();
        Self { acc0, acc1, acc2 }
    }
}

impl defmt::Format for SAcc {
    fn format(&self, fmt: defmt::Formatter) {
        let acc_x = self.acc0 as f32 * ACC_FACTOR;
        let acc_y = self.acc1 as f32 * ACC_FACTOR;
        let acc_z = self.acc2 as f32 * ACC_FACTOR;

        defmt::write!(
            fmt,
            "acceleration X = {}, acceleration Y = {}, acceleration Z = {}",
            acc_x,
            acc_y,
            acc_z
        )
    }
}

pub struct SGyro {
    w0: i16,
    w1: i16,
    w2: i16,
}

impl SGyro {
    pub fn new(data: &[u8]) -> Self {
        let (data, w0) = take_i16_le(data).unwrap_or_default();
        let (data, w1) = take_i16_le(data).unwrap_or_default();
        let (_, w2) = take_i16_le(data).unwrap_or_default();
        Self { w0, w1, w2 }
    }
}

impl defmt::Format for SGyro {
    fn format(&self, fmt: defmt::Formatter) {
        let w_x = self.w0 as f32 * ACC_FACTOR;
        let w_y = self.w1 as f32 * ACC_FACTOR;
        let w_z = self.w2 as f32 * ACC_FACTOR;

        defmt::write!(
            fmt,
            "angular velocity X = {}, angular velocity  Y = {}, angular velocity  Z = {}",
            w_x,
            w_y,
            w_z
        )
    }
}

pub struct SAngle {
    ang0: i16,
    ang1: i16,
    ang2: i16,
}

impl defmt::Format for SAngle {
    fn format(&self, fmt: defmt::Formatter) {
        let roll = self.ang0 as f32 * ANGLE_FACTOR;
        let pitch = self.ang1 as f32 * ANGLE_FACTOR;
        let yaw = self.ang2 as f32 * ANGLE_FACTOR;

        defmt::write!(fmt, "roll = {}, pitch = {}, yaw = {}", roll, pitch, yaw)
    }
}

impl SAngle {
    pub fn new(data: &[u8]) -> Self {
        let (data, ang0) = take_i16_le(data).unwrap_or_default();
        let (data, ang1) = take_i16_le(data).unwrap_or_default();
        let (_, ang2) = take_i16_le(data).unwrap_or_default();
        Self { ang0, ang1, ang2 }
    }
}

#[derive(Format)]
pub struct SMag {
    h0: i16,
    h1: i16,
    h2: i16,
}

impl SMag {
    pub fn new(data: &[u8]) -> Self {
        let (data, h0) = take_i16_le(data).unwrap_or_default();
        let (data, h1) = take_i16_le(data).unwrap_or_default();
        let (_, h2) = take_i16_le(data).unwrap_or_default();
        Self { h0, h1, h2 }
    }
}

#[derive(Format)]
pub struct SDStatus {
    s0: i16,
    s1: i16,
    s2: i16,
    s3: i16,
}

impl SDStatus {
    pub fn new(data: &[u8]) -> Self {
        let (data, s0) = take_i16_le(data).unwrap_or_default();
        let (data, s1) = take_i16_le(data).unwrap_or_default();
        let (data, s2) = take_i16_le(data).unwrap_or_default();
        let (_, s3) = take_i16_le(data).unwrap_or_default();
        Self { s0, s1, s2, s3 }
    }
}

#[derive(Format)]
pub struct SPress {
    l_pres: i32,
    l_alt: i32,
}

impl SPress {
    pub fn new(data: &[u8]) -> Self {
        let (data, l_pres) = take_i32_le(data).unwrap_or_default();
        let (_, l_alt) = take_i32_le(data).unwrap_or_default();
        Self { l_pres, l_alt }
    }
}

#[derive(Format)]
pub struct SLonLat {
    l_lon: i32,
    l_lat: i32,
}

impl SLonLat {
    pub fn new(data: &[u8]) -> Self {
        let (data, l_lon) = take_i32_le(data).unwrap_or_default();
        let (_, l_lat) = take_i32_le(data).unwrap_or_default();
        Self { l_lon, l_lat }
    }
}

#[allow(clippy::upper_case_acronyms)]
#[derive(Format)]
pub struct SGPSV {
    s_gpsheight: i16,
    s_gpsyaw: i16,
    l_gpsvelocity: i32,
}

impl SGPSV {
    pub fn new(data: &[u8]) -> Self {
        let (data, s_gpsheight) = take_i16_le(data).unwrap_or_default();
        let (data, s_gpsyaw) = take_i16_le(data).unwrap_or_default();
        let (_, l_gpsvelocity) = take_i32_le(data).unwrap_or_default();
        Self {
            s_gpsheight,
            s_gpsyaw,
            l_gpsvelocity,
        }
    }
}

#[derive(Serialize)]
pub struct SQuat {
    pub q0: i16,
    pub q1: i16,
    pub q2: i16,
    pub q3: i16,
}

impl SQuat {
    pub fn new(data: &[u8]) -> Self {
        let (data, q0) = take_i16_le(data).unwrap_or_default();
        let (data, q1) = take_i16_le(data).unwrap_or_default();
        let (data, q2) = take_i16_le(data).unwrap_or_default();
        let (_, q3) = take_i16_le(data).unwrap_or_default();
        Self { q0, q1, q2, q3 }
    }
}

impl defmt::Format for SQuat {
    fn format(&self, fmt: defmt::Formatter) {
        let q0 = self.q0 as f32 * QUAT_FACTOR;
        let q1 = self.q1 as f32 * QUAT_FACTOR;
        let q2 = self.q2 as f32 * QUAT_FACTOR;
        let q3 = self.q3 as f32 * QUAT_FACTOR;

        defmt::write!(fmt, "q0 = {}, q1 = {}, q2 = {}, q3 = {}", q0, q1, q2, q3)
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
    Invalid,
}

pub struct WIT;

impl WIT {
    fn get_checksum(data: &[u8]) -> (u8, u8) {
        let mut checksum: u8 = 0;
        let expected_checksum = data[10];

        // calculate checksum
        for &byte in &data[..10] {
            checksum = checksum.wrapping_add(byte);
        }

        (checksum, expected_checksum)
    }

    pub fn parse_wit(packet_data: &[u8]) -> Option<WITData> {
        let (checksum, expected_checksum) = Self::get_checksum(packet_data);
        if checksum != expected_checksum {
            defmt::warn!(
                "Checksum doesn't match! got: {:02x}, expected: {:02x}",
                checksum,
                expected_checksum
            );
            return None;
        }

        let data_type = packet_data[1];
        let data = &packet_data[2..10];

        use WITData::*;
        let result = match data_type {
            TIME => Acc(SAcc::new(data)),
            ANGULAR_VELOCITY => Gyro(SGyro::new(data)),
            ANGLE => Angle(SAngle::new(data)),
            MAGNETIC_FIELD => Mag(SMag::new(data)),
            PORT => DStatus(SDStatus::new(data)),
            BAROMETRIC_ALTITUDE => Press(SPress::new(data)),
            LATITUDE_LONGITUDE => LonLat(SLonLat::new(data)),
            GROUND_SPEED => Gpsv(SGPSV::new(data)),
            QUATERNION => Quat(SQuat::new(data)),
            _ => Invalid,
        };

        Some(result)
    }
}
