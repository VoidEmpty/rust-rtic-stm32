#![no_std]

pub mod data_types {
    extern crate alloc;
    use alloc::string::String;

    #[allow(unused_imports)]
    use serde::{Deserialize, Serialize};

    pub use nmea_protocol::GpsData;
    pub use wit_protocol::SQuat;

    #[derive(Serialize)]
    pub enum MotorState {
        Running,
        Stopped,
    }

    #[derive(Serialize)]
    pub struct MotorData {
        pub motor_state: MotorState,
        pub motor_angle: f32,
    }

    #[derive(Serialize)]
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