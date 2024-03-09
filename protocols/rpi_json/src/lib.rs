#![no_std]

pub mod commands {
    extern crate alloc;

    use serde::Deserialize;
    #[allow(unused_imports)]
    use serde_json::{Result, Value};

    #[derive(Deserialize)]
    pub struct Command {
        pub cmd: u8,
        #[serde(flatten)]
        pub cmd_data: Option<CommandData>,
    }

    impl Command {
        pub fn from_json(slice: &[u8]) -> Option<Self> {
            let res = serde_json::from_slice(slice);
            if res.is_err() {
                defmt::warn!("Failed to parse command from JSON: {=[u8]:a}", slice);
            }
            res.ok()
        }
    }

    #[derive(Deserialize)]
    #[serde(untagged)]
    pub enum CommandData {
        #[serde(rename(deserialize = "delay_time"))]
        Delay(u32),
        #[serde(rename(deserialize = "angle"))]
        Angle(f32),
    }
}

pub mod data_types {
    extern crate alloc;
    use alloc::string::String;

    #[allow(unused_imports)]
    use serde::{Deserialize, Serialize};

    pub use nmea_protocol::GpsData;
    pub use wit_protocol::SQuat;

    #[derive(Serialize, Default)]
    pub enum MotorState {
        Running,
        #[default]
        Stopped,
    }

    #[derive(Serialize, Default)]
    pub struct MotorData {
        pub motor_state: MotorState,
        pub motor_angle: f32,
    }

    #[derive(Serialize, Default)]
    pub struct RegularData {
        pub gps_data: GpsData,
        pub quat: SQuat,
        pub motor_data: MotorData,
    }

    impl RegularData {
        pub fn to_json(&self) -> Option<String> {
            serde_json::to_string(self).ok()
        }
    }
}
