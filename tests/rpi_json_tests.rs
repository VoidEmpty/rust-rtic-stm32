#![no_std]
#![no_main]

use cortex_m_rt as _;

use stm32f4xx_hal as _;

// RTT and defmt logger setup
use defmt_rtt as _;

// Setup panic behaviour
use panic_probe as _;

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// add rust collections with custom allocator
extern crate alloc;
// See https://crates.io/crates/defmt-test/0.3.0 for more documentation (e.g. about the 'state'
// feature)
#[allow(unused_imports)]
#[defmt_test::tests]
mod tests {
    #[init]
    fn init() {
        // Initialize the allocator
        {
            use embedded_alloc::Heap;

            #[global_allocator]
            static HEAP: Heap = Heap::empty();

            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }
    }

    #[allow(unused_imports)]
    use defmt::{assert, assert_eq, assert_ne};

    use nmea_protocol::{Direction, GpsData};
    extern crate rpi_json;
    #[allow(unused_imports)]
    use rpi_json::data_types::RegularData;
    use rpi_json::data_types::{MotorData, MotorState};
    use wit_protocol::SQuat;

    #[test]
    fn check_regular_data() {
        let gps_data = GpsData {
            latitude: 5546.95900,
            lat_dir: Direction::North,
            longitude: 03740.69200,
            lon_dir: Direction::East,
            time: 123035.546,
        };

        defmt::info!("GPS data to send: {}", gps_data);

        let quat = SQuat {
            q0: 1,
            q1: 2,
            q2: 3,
            q3: 4,
        };

        let motor_data = MotorData {
            motor_angle: 60.0,
            motor_state: MotorState::Stopped,
        };

        let reg_data = RegularData {
            gps_data,
            quat,
            motor_data,
        };

        let json_string = reg_data.to_json();

        assert!(json_string.is_some());

        if let Some(s) = json_string {
            defmt::info!("JSON string: {=str}", s.as_str())
        }
    }
}
