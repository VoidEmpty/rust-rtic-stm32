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
    use defmt::assert;

    extern crate nmea_protocol;
    use nmea_protocol::Nmea;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
