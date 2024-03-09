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

    extern crate nmea_protocol;
    use nmea_protocol::{Direction, Nmea};

    #[test]
    fn parse_gga() {
        let data = b"$GNGGA,102030.000,5546.95900,N,03740.69200,E,1,08,2.0,142.0,M,0.0,M,,*";
        if let Some(gps_data) = Nmea::parse_nmea(data) {
            defmt::info!("{}", gps_data);
            assert_eq!(gps_data.latitude, 5546.959);
            assert!(gps_data.lat_dir == Direction::North);
            assert_eq!(gps_data.longitude, 3740.692);
            assert!(gps_data.lon_dir == Direction::East);
        } else {
            assert!(false, "Failed to parse GGA data!");
        }
    }

    #[test]
    fn parse_gll() {
        let data = b"$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*";
        if let Some(gps_data) = Nmea::parse_nmea(data) {
            defmt::info!("{}", gps_data);
            assert_eq!(gps_data.latitude, 5546.959);
            assert!(gps_data.lat_dir == Direction::North);
            assert_eq!(gps_data.longitude, 3740.692);
            assert!(gps_data.lon_dir == Direction::East);
        } else {
            assert!(false, "Failed to parse GGA data!");
        }
    }
}
