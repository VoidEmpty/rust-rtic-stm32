#![warn(clippy::std_instead_of_alloc, clippy::std_instead_of_core)]
#![no_main]
#![no_std]

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

mod parser;
mod wit_protocol;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI1])]
mod app {
    use alloc::collections::VecDeque;
    use alloc::vec::Vec;

    // Use HAL crate for stm32f407
    use stm32f4xx_hal::{
        pac::USART2,
        prelude::*,
        serial::{config::Config, Event, Rx, Serial, Tx},
    };

    use crate::wit_protocol::{get_wit_data, WITData};

    // Setup heap allocator for rust collections
    use embedded_alloc::Heap;

    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    #[shared]
    struct Shared {
        buf: VecDeque<u8>,
    }

    const PACKET_SIZE: usize = 11;

    #[local]
    struct Local {
        tx: Tx<USART2>,
        rx: Rx<USART2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::debug!("Init started");

        // Initialize the allocator
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .hclk(84.MHz())
            .require_pll48clk()
            .pclk2(21.MHz())
            .freeze();

        let gpioa = ctx.device.GPIOA.split();

        // Configure serial
        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;

        let mut serial: Serial<USART2, u8> = Serial::new(
            ctx.device.USART2,
            (tx_pin, rx_pin),
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
        .unwrap();

        serial.listen(Event::Rxne);

        let (tx, rx) = serial.split();

        let buf: VecDeque<u8> = VecDeque::new();
        (
            Shared {
                // Initialization of shared resources go here
                buf,
            },
            Local {
                // Initialization of local resources go here
                tx,
                rx,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("In idle");
        loop {
            continue;
        }
    }

    #[task(shared = [buf])]
    fn process_packet(mut ctx: process_packet::Context) {
        let mut packet: Vec<u8> = alloc::vec![];

        ctx.shared.buf.lock(|buf| {
            while !buf.is_empty() && buf[0] != 0x55 {
                buf.pop_front();
            }

            // whait untill packet is full
            if buf.len() < PACKET_SIZE {
                return;
            }

            packet = buf.drain(..PACKET_SIZE).collect();
        });

        defmt::debug!("Packet: {=[u8]:02x}", packet);

        if packet.len() != PACKET_SIZE {
            return;
        }

        let mut checksum: u8 = 0;
        let expected_checksum = packet[10];

        // calculate checksum
        for &byte in &packet[..10] {
            checksum = checksum.wrapping_add(byte);
        }

        if checksum == expected_checksum {
            let data_type = packet[1];
            let data = packet[2..10].to_vec();

            if let WITData::Angle(wit_data) = get_wit_data(data_type, data.as_slice()) {
                defmt::info!("{}", wit_data);
            }
        } else {
            defmt::warn!(
                "Checksum doesn't match! got: {:02x}, expected: {:02x}",
                checksum,
                expected_checksum
            );
        }
    }

    #[task(binds = USART2, local = [tx, rx], shared = [buf])]
    fn usart2(mut ctx: usart2::Context) {
        let rx = ctx.local.rx;

        rx.unlisten();

        // if tx.is_tx_empty() {
        //     defmt::println!("TX interrupt");
        //     let _ = tx.write(0xFF);
        //     tx.unlisten();
        // }

        if rx.is_rx_not_empty() {
            if let Ok(byte) = rx.read() {
                defmt::debug!("RX Byte value: {:02x}", byte);
                ctx.shared.buf.lock(|buf| {
                    // wait for header byte
                    if buf.is_empty() {
                        if byte == 0x55 {
                            buf.push_back(byte);
                        }
                    } else {
                        buf.push_back(byte);
                    }

                    // message complete
                    if buf.len() > PACKET_SIZE {
                        process_packet::spawn().unwrap();
                    }
                });
            }
        }

        rx.listen();
    }
}

// #[cfg(test)]
// #[defmt_test::tests]
// mod unit_tests {
//     use defmt::assert;
//     #[test]
//     fn it_works() {
//         assert!(true)
//     }
// }
