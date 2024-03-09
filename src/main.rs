#![warn(clippy::std_instead_of_alloc, clippy::std_instead_of_core)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

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

extern crate nmea_protocol;
extern crate wit_protocol;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI1])]
mod app {

    use alloc::collections::VecDeque;
    use alloc::vec::Vec;

    use rtic_monotonics::systick::Systick;

    // Use HAL crate for stm32f407
    use stm32f4xx_hal::{
        pac::USART1,
        pac::USART2,
        prelude::*,
        serial::{config::Config, Event, Rx, Serial, Tx},
    };

    use nmea_protocol::Nmea;
    use wit_protocol::WIT;

    // Setup heap allocator for rust collections
    use embedded_alloc::Heap;

    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    // Resources
    #[shared]
    struct Shared {
        read_buf1: VecDeque<u8>,
        read_buf2: VecDeque<u8>,
        write_buf2: VecDeque<u8>,
    }

    const PACKET_SIZE: usize = 11;

    #[local]
    struct Local {
        commands: VecDeque<Vec<u8>>,
        tx1: Tx<USART1>,
        rx1: Rx<USART1>,
        tx2: Tx<USART2>,
        rx2: Rx<USART2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
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
        Systick::start(ctx.core.SYST, 168_000_000, systick_mono_token); // default STM32F407 clock-rate is 16MHz

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .hclk(84.MHz())
            .require_pll48clk()
            .pclk2(21.MHz())
            .freeze();

        let gpioa = ctx.device.GPIOA.split();

        let tx_pin = gpioa.pa9;
        let rx_pin = gpioa.pa10;

        let mut serial1: Serial<USART1, u8> = Serial::new(
            ctx.device.USART1,
            (tx_pin, rx_pin),
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
        .unwrap();

        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;

        let mut serial2: Serial<USART2, u8> = Serial::new(
            ctx.device.USART2,
            (tx_pin, rx_pin),
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
        .unwrap();

        serial1.listen(Event::Rxne);
        serial2.listen(Event::Rxne);

        let (tx1, rx1) = serial1.split();
        let (tx2, rx2) = serial2.split();

        let read_buf1: VecDeque<u8> = VecDeque::new();
        let read_buf2: VecDeque<u8> = VecDeque::new();
        let write_buf2: VecDeque<u8> = VecDeque::new();

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
                read_buf1,
                read_buf2,
                write_buf2,
            },
            Local {
                // Initialization of local resources go here
                commands,
                tx1,
                rx1,
                tx2,
                rx2,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("In idle");

        loop {}
    }

    #[task(local = [commands], shared = [write_buf2], priority = 1)]
    async fn send_command(mut ctx: send_command::Context) {
        loop {
            if let Some(cmd) = ctx.local.commands.pop_front() {
                defmt::info!("Sending command: {=[u8]:02x}", cmd);
                for byte in cmd {
                    ctx.shared
                        .write_buf2
                        .lock(|write_buf| write_buf.push_back(byte));
                }
                Systick::delay(1000.millis()).await;
            }
        }
    }

    #[task(binds = USART1, local = [tx1, rx1], shared = [read_buf1])]
    fn usart1(mut ctx: usart1::Context) {
        let rx = ctx.local.rx1;

        let mut start_read = false;

        if rx.is_rx_not_empty() {
            if let Ok(byte) = rx.read() {
                defmt::debug!("RX Byte value: {:02x}", byte);
                ctx.shared.read_buf1.lock(|read_buf| {
                    read_buf.push_back(byte);

                    if read_buf.len() < 2 {
                        return;
                    }

                    // check last two elements
                    let mut it = read_buf.iter().rev().take(2);
                    let last2 = it.next().unwrap().clone();
                    let last1 = it.next().unwrap().clone();

                    let end = ['\r', '\n'].map(|x| x as u8);

                    if [last1, last2] == end {
                        if !start_read {
                            read_buf.clear();
                            start_read = true;
                        } else {
                            // message complete
                            // call process function
                            let data: Vec<u8> = read_buf.drain(..).collect();
                            defmt::debug!("Packet: {=[u8]:02x}", data);
                            if let Some(gps_data) = Nmea::parse_nmea(&data) {
                                defmt::info!("{}", gps_data);
                            }
                        }
                    }
                });
            }
        }
    }

    #[task(binds = USART2, local = [tx2, rx2], shared = [read_buf2, write_buf2])]
    fn usart2(mut ctx: usart2::Context) {
        let rx = ctx.local.rx2;
        let tx = ctx.local.tx2;

        let mut start_read = false;

        if tx.is_tx_empty() {
            ctx.shared.write_buf2.lock(|write_buf| {
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
                ctx.shared.read_buf2.lock(|read_buf| {
                    read_buf.push_back(byte);

                    while !read_buf.is_empty() && read_buf[0] != 0x55 {
                        read_buf.pop_front();
                    }

                    // message complete
                    if read_buf.len() > PACKET_SIZE {
                        let packet: Vec<u8> = read_buf.drain(..PACKET_SIZE).collect();
                        defmt::debug!("Packet: {=[u8]:02x}", packet);
                        if let Some(wit_data) = WIT::parse_wit(&packet) {
                            defmt::info!("{}", wit_data);
                        }
                    }
                });
            }
        }
    }
}
