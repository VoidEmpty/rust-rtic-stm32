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

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI1, SPI2])]
mod app {
    use alloc::collections::VecDeque;
    use alloc::vec::Vec;

    use rtic_monotonics::systick::Systick;

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

    // Resources
    #[shared]
    struct Shared {
        read_buf: VecDeque<u8>,
        write_buf: VecDeque<u8>,
    }

    const PACKET_SIZE: usize = 11;

    #[local]
    struct Local {
        commands: VecDeque<Vec<u8>>,
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

        // Setup clocks
        let rcc = ctx.device.RCC.constrain();

        // Initialize the systick interrupt & obtain the token to prove that we did
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_mono_token); // default STM32F303 clock-rate is 36MHz

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .hclk(84.MHz())
            .require_pll48clk()
            .pclk2(21.MHz())
            .freeze();

        let gpioa = ctx.device.GPIOA.split();

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

        let read_buf: VecDeque<u8> = VecDeque::new();
        let write_buf: VecDeque<u8> = VecDeque::new();

        // Configure inclinometer
        let mut commands = VecDeque::new();

        // special unlock/enable command??? (for some reason not documented anywhere)
        commands.push_back([0xFF, 0xF0, 0xF0, 0xF0, 0xF0].into());
        // unlock config
        commands.push_back([0xff, 0xaa, 0x69, 0x88, 0xb5].into());
        // set data rate to 2Hz
        commands.push_back([0xff, 0xaa, 0x03, 0x01, 0x00].into());
        // save config
        commands.push_back([0xff, 0xaa, 0x00, 0x00, 0x00].into());

        // send_command::spawn_after(Duration::<u64, 1, 1000>::from_ticks(2000)).unwrap();

        (
            Shared {
                // Initialization of shared resources go here
                read_buf,
                write_buf,
            },
            Local {
                // Initialization of local resources go here
                commands,
                tx,
                rx,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("In idle");

        loop {}
    }

    #[task(local = [commands], shared= [write_buf])]
    fn send_command(mut ctx: send_command::Context) {
        if let Some(cmd) = ctx.local.commands.pop_front() {
            defmt::info!("Sending command: {=[u8]:02x}", cmd);
            for byte in cmd {
                ctx.shared
                    .write_buf
                    .lock(|write_buf| write_buf.push_back(byte));
            }
            send_command::spawn_after(Duration::<u64, 1, 1000>::from_ticks(2000)).unwrap();
        }
    }

    #[task(shared = [read_buf])]
    fn process_packet(mut ctx: process_packet::Context) {
        let mut packet: Vec<u8> = alloc::vec![];

        ctx.shared.read_buf.lock(|read_buf| {
            while !read_buf.is_empty() && read_buf[0] != 0x55 {
                read_buf.pop_front();
            }

            // wait untill packet is full
            if read_buf.len() < PACKET_SIZE {
                return;
            }

            packet = read_buf.drain(..PACKET_SIZE).collect();
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

    #[task(binds = USART2, local = [tx, rx], shared = [read_buf, write_buf])]
    fn usart2(mut ctx: usart2::Context) {
        let rx = ctx.local.rx;
        let tx = ctx.local.tx;

        // rx.unlisten();

        let mut start_read = false;

        if tx.is_tx_empty() {
            ctx.shared.write_buf.lock(|write_buf| {
                if let Some(byte) = write_buf.pop_front() {
                    if let Ok(_) = tx.write(byte) {
                        defmt::debug!("TX Byte value: {:02x}", byte);
                    }
                }
                start_read = write_buf.is_empty();
            })
        }

        if rx.is_rx_not_empty() && start_read {
            if let Ok(byte) = rx.read() {
                defmt::debug!("RX Byte value: {:02x}", byte);
                ctx.shared.read_buf.lock(|read_buf| {
                    // wait for header byte
                    if read_buf.is_empty() {
                        if byte == 0x55 {
                            read_buf.push_back(byte);
                        }
                    } else {
                        read_buf.push_back(byte);
                    }

                    // message complete
                    if read_buf.len() > PACKET_SIZE {
                        process_packet::spawn().unwrap();
                    }
                });
            }
        }

        // rx.listen();
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
