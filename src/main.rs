#![feature(type_alias_impl_trait)]
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

extern crate nmea_protocol;
extern crate rpi_json;
extern crate wit_protocol;

mod spi;
mod tmc2160;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
    use alloc::vec::Vec;
    use alloc::{collections::VecDeque, string::String};

    use rtic_monotonics::systick::Systick;

    // Use HAL crate for stm32f407
    use stm32f4xx_hal::{
        gpio::{gpiob, gpiod, Output, PushPull},
        pac::{SPI3, TIM4, USART1, USART2, USART3},
        prelude::*,
        serial::{config::Config, Event, Rx, Serial, Tx},
        spi::*,
        timer::{Channel, Channel2, ChannelBuilder, PwmHz},
    };

    use nmea_protocol::{GpsData, Nmea};
    use rpi_json::data_types::RegularData;
    use wit_protocol::WIT;

    // Setup heap allocator for rust collections
    use embedded_alloc::Heap;

    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    // Resources
    #[shared]
    struct Shared {
        reg_data: RegularData,
        read_buf_gps: VecDeque<u8>,
        read_buf_incl: VecDeque<u8>,
        read_buf_rpi: VecDeque<u8>,
    }

    const PACKET_SIZE: usize = 11;

    #[local]
    struct Local {
        _motor_pwm: PwmHz<TIM4, ChannelBuilder<TIM4, 1>>,
        _motor_dir: gpiob::PB6<Output<PushPull>>,
        drv_en: gpiod::PD11<Output<PushPull>>,
        cs_motor: gpiod::PD0<Output<PushPull>>,
        spi_motor: Spi<SPI3>,
        serial_gps: Serial<USART1>,
        serial_incl: Serial<USART2>,
        tx_rpi: Tx<USART3>,
        rx_rpi: Rx<USART3>,
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

        // clocls
        let clocks = rcc.cfgr.freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpiod = ctx.device.GPIOD.split();
        let gpioc = ctx.device.GPIOC.split();

        // SPI configuration (AF6)
        let sck = gpioc.pc10.into_alternate();
        let miso = gpioc.pc11.into_alternate();
        let mosi = gpioc.pc12.into_alternate();

        // create CS instance
        let cs_motor = gpiod.pd0.into_push_pull_output();

        let spi_motor = Spi::new(
            ctx.device.SPI3,
            (sck, miso, mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            1.MHz().into(),
            &clocks,
        );

        // add diagnostic leds
        // let led = gpiod.pd13.into_push_pull_output();

        let drv_en = gpiod.pd11.into_push_pull_output();
        // add pwm for motor
        let channels = Channel2::new(gpiob.pb7);
        let mut _motor_pwm = ctx.device.TIM4.pwm_hz(channels, 2.kHz(), &clocks);
        _motor_pwm.enable(Channel::C2);

        // dir for motor
        let _motor_dir = gpiob.pb6.into_push_pull_output();

        // TODO enable on board v2
        // spi_write_drv_registers::spawn().expect("Failed to spawn task spi_write_drv_registers!");

        // create usart serials
        let tx_pin = gpioa.pa9;
        let rx_pin = gpioa.pa10;

        let mut serial_gps: Serial<USART1, u8> = Serial::new(
            ctx.device.USART1,
            (tx_pin, rx_pin),
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
        .unwrap();

        let tx_pin = gpiod.pd5;
        let rx_pin = gpiod.pd6;

        let mut serial_incl: Serial<USART2, u8> = Serial::new(
            ctx.device.USART2,
            (tx_pin, rx_pin),
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
        .unwrap();

        let tx_pin = gpiob.pb10;
        let rx_pin = gpiob.pb11;

        let mut serial_rpi: Serial<USART3, u8> = Serial::new(
            ctx.device.USART3,
            (tx_pin, rx_pin),
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
        .unwrap();

        serial_gps.listen(Event::Rxne);
        serial_incl.listen(Event::Rxne);
        serial_rpi.listen(Event::Rxne);

        let (tx_rpi, rx_rpi) = serial_rpi.split();

        let read_buf_gps: VecDeque<u8> = VecDeque::new();
        let read_buf_incl: VecDeque<u8> = VecDeque::new();
        let read_buf_rpi: VecDeque<u8> = VecDeque::new();

        let reg_data = RegularData::default();

        send_regualar_data::spawn().expect("Failed to spawn task send_regualar_data!");

        (
            Shared {
                // Initialization of shared resources go here
                reg_data,
                read_buf_gps,
                read_buf_incl,
                read_buf_rpi,
            },
            Local {
                // Initialization of local resources go here
                drv_en,
                _motor_pwm,
                _motor_dir,
                spi_motor,
                cs_motor,
                serial_gps,
                serial_incl,
                tx_rpi,
                rx_rpi,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("In idle");

        loop {}
    }

    #[task(local = [spi_motor, cs_motor, drv_en], priority = 4)]
    async fn spi_write_drv_registers(ctx: spi_write_drv_registers::Context) {
        defmt::info!("Drive configuration");
        let spi = ctx.local.spi_motor;
        let cs = ctx.local.cs_motor;
        let drv_en = ctx.local.drv_en;

        use crate::spi::*;
        use crate::tmc2160::*;

        drv_en.set_high();
        cs.set_low();

        let log_error = |err| {
            defmt::error!("SPI Failed to write value on addres, {}", err);
            return 0;
        };

        spi_transfer(spi, registers::CHOPCONF, 0x110140c3).unwrap_or_else(log_error);
        let _chop = spi_transfer(spi, registers::CHOPCONF, 0x110140c3).unwrap_or_else(log_error);
        defmt::info!(
            "CHOPCONF: write value = {=u32:#x} ; read value {=u32:#x}",
            0x110140c3,
            _chop
        );
        spi_transfer(spi, registers::GLOBAL_SCALER, 0).unwrap_or_else(log_error);
        spi_transfer(spi, registers::IHOLD_IRUN, 0x000f0909).unwrap_or_else(log_error);
        let _irun = spi_transfer(spi, registers::IHOLD_IRUN, 0x000f0909).unwrap_or_else(log_error);
        defmt::info!(
            "IHOLD_IRUN: write value = {=u32:#x} ; read value {=u32:#x}",
            0x000f0909,
            _irun
        );
        spi_transfer(spi, registers::TPOWERDOWN, 10).unwrap_or_else(log_error);
        spi_transfer(spi, registers::TPWMTHRS, 0).unwrap_or_else(log_error);
        spi_transfer(spi, registers::PWMCONF, 0xc4000160).unwrap_or_else(log_error);
        spi_transfer(spi, registers::GCONF, 0x00000005).unwrap_or_else(log_error);

        cs.set_high();
        drv_en.set_low();
        defmt::info!("Drive configuration finished");
    }

    #[task(binds = USART1, local = [serial_gps], shared = [read_buf_gps, reg_data])]
    fn usart1(mut ctx: usart1::Context) {
        let serial = ctx.local.serial_gps;

        let mut received_data = GpsData::default();

        if serial.is_rx_not_empty() {
            if let Ok(byte) = serial.read() {
                defmt::debug!("RX Byte value: {:02x}", byte);
                ctx.shared.read_buf_gps.lock(|read_buf| {
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
                        // message complete
                        // call process function
                        let data: Vec<u8> = read_buf.drain(..).collect();
                        defmt::debug!("Packet: {=[u8]:02x}", data);
                        if let Some(gps_data) = Nmea::parse_nmea(&data) {
                            defmt::info!("{}", gps_data);
                            received_data = gps_data;
                        }
                    }
                });
            }
        }

        // update current gps data
        ctx.shared.reg_data.lock(|rdata| {
            rdata.gps_data = received_data;
        });
    }

    #[task(binds = USART2, local = [serial_incl], shared = [read_buf_incl, reg_data])]
    fn usart2(mut ctx: usart2::Context) {
        let serial = ctx.local.serial_incl;

        let mut quat = wit_protocol::SQuat::default();

        if serial.is_rx_not_empty() {
            if let Ok(byte) = serial.read() {
                defmt::debug!("RX Byte value: {:02x}", byte);
                ctx.shared.read_buf_incl.lock(|read_buf| {
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
                            if let wit_protocol::WITData::Quat(q) = wit_data {
                                quat = q;
                            }
                        }
                    }
                });
            }
        }

        // update current inclinometer data
        ctx.shared.reg_data.lock(|rdata| {
            rdata.quat = quat;
        });
    }

    #[task(binds = USART3, local = [rx_rpi], shared = [read_buf_rpi], priority = 4)]
    fn usart3(ctx: usart3::Context) {
        let rx = ctx.local.rx_rpi;

        if rx.is_rx_not_empty() {
            if let Ok(byte) = rx.read() {
                defmt::debug!("RX Byte value: {=u8:a}", byte);
                // TODO: add commands parsing
                // ctx.shared.read_buf_rpi.lock(|read_buf| {
                //     read_buf.push_back(byte);
                // });
            }
        }
    }

    #[task(local = [tx_rpi], shared = [reg_data], priority = 3)]
    async fn send_regualar_data(mut ctx: send_regualar_data::Context) {
        let tx = ctx.local.tx_rpi;

        loop {
            if tx.is_tx_empty() {
                let mut json_str = String::new();

                ctx.shared.reg_data.lock(|rdata| {
                    json_str = rdata.to_json().unwrap_or_default();
                    defmt::debug!("JSON data: {=str}", json_str.as_str());
                });

                for byte in json_str.as_bytes() {
                    while let Err(stm32f4xx_hal::nb::Error::WouldBlock) = tx.write(*byte) {
                        defmt::trace!("TX failed to write, would block");
                    }
                }
            }

            Systick::delay(2.secs()).await;
        }
    }
}
