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
        gpio::{gpiod, gpiof, Output, PushPull},
        pac::{SPI3, TIM12, TIM2, USART1, USART2, USART3},
        prelude::*,
        qei::Qei,
        serial::{config::Config, Event, Rx, Serial, Tx},
        spi::*,
        timer::{Channel, Channel2, ChannelBuilder, PwmHz},
    };

    use nmea_protocol::{GpsData, Nmea};
    use rpi_json::commands::*;
    use rpi_json::data_types::RegularData;
    use wit_protocol::WIT;

    // Setup heap allocator for rust collections
    use embedded_alloc::Heap;

    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    // Resources
    #[shared]
    struct Shared {
        // TODO split to separate structures
        send_data: bool,
        stop_flag: bool,
        send_delay: u32,
        reg_data: RegularData,
        read_buf_gps: VecDeque<u8>,
        read_buf_incl: VecDeque<u8>,
        read_buf_rpi: VecDeque<u8>,
    }

    const PACKET_SIZE: usize = 11;

    #[local]
    struct Local {
        // TODO split to separate structures
        motor_pwm: PwmHz<TIM12, ChannelBuilder<TIM12, 1>>,
        motor_dir: gpiof::PF5<Output<PushPull>>,
        encoder: Qei<TIM2>,
        drv_en: gpiof::PF2<Output<PushPull>>,
        cs_motor: gpiod::PD0<Output<PushPull>>,
        spi_motor: Spi<SPI3>,
        serial_gps: Serial<USART1>,
        serial_incl: Serial<USART2>,
        tx_rpi: Tx<USART3>,
        rx_rpi: Rx<USART3>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("Init started");

        // Initialize the allocator
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 2048;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        // Setup clocks
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Initialize the systick interrupt & obtain the token to prove that we did
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, clocks.sysclk().raw(), systick_mono_token);

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpiod = ctx.device.GPIOD.split();
        let gpioc = ctx.device.GPIOC.split();
        let gpiof = ctx.device.GPIOF.split();

        // SPI configuration (AF6)
        let sck = gpioc.pc10.into_alternate();
        let miso = gpioc.pc11.into_alternate();
        let mosi = gpioc.pc12.into_alternate();

        // create CS instance
        let cs_motor = gpiod.pd0.into_push_pull_output();

        let mut spi_motor = Spi::new(
            ctx.device.SPI3,
            (sck, miso, mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            1.MHz().into(),
            &clocks,
        );

        spi_motor.enable(true);

        // add diagnostic leds
        // let led = gpiod.pd13.into_push_pull_output();

        let drv_en = gpiof.pf2.into_push_pull_output();
        // add pwm for motor
        let channels = Channel2::new(gpiob.pb15);
        let mut motor_pwm = ctx.device.TIM12.pwm_hz(channels, 5.kHz(), &clocks);
        let max_duty = motor_pwm.get_max_duty();
        motor_pwm.set_duty(Channel::C2, max_duty / 2);
        // motor_pwm.enable(Channel::C2);

        // dir for motor
        let motor_dir = gpiof.pf5.into_push_pull_output();

        // add encoder interface
        let encoder_pin1 = gpioa.pa0.into_alternate();
        let encoder_pin2 = gpioa.pa1.into_alternate();

        let encoder = ctx.device.TIM2.qei((encoder_pin1, encoder_pin2));

        // TODO enable on board v2
        spi_write_drv_registers::spawn().expect("Failed to spawn task spi_write_drv_registers!");

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

        // rpi communication
        let send_delay = 2000;
        let stop_flag = false;
        let send_data = false;

        (
            Shared {
                // Initialization of shared resources go here
                send_delay,
                stop_flag,
                send_data,
                reg_data,
                read_buf_gps,
                read_buf_incl,
                read_buf_rpi,
            },
            Local {
                // Initialization of local resources go here
                drv_en,
                motor_pwm,
                motor_dir,
                encoder,
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

    #[task(local = [spi_motor, cs_motor, drv_en], priority = 3)]
    async fn spi_write_drv_registers(ctx: spi_write_drv_registers::Context) {
        defmt::info!("Drive configuration");
        let spi = ctx.local.spi_motor;
        let cs = ctx.local.cs_motor;
        let drv_en = ctx.local.drv_en;

        use crate::spi::*;
        use crate::tmc2160::*;

        let log_error = |err| {
            defmt::error!("SPI Failed to write value on addres, {}", err);
            return 0;
        };

        // spi_transfer(spi, registers::IHOLD_IRUN, 0x00001805).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::TPOWERDOWN, 0x00000000).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::TPWMTHRS, 0x00000000).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::TCOOLTHRS, 0x000FFFFF).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::THIGH, 0x00000000).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::CHOPCONF, 0x16010401).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::COOLCONF, 0x01108000).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::PWMCONF, 0x001C0F40).unwrap_or_else(log_error);
        // spi_transfer(spi, registers::GCONF, 0x00002186).unwrap_or_else(log_error);

        let a: u32 =
            spi_transfer(spi, registers::CHOPCONF, 0x110140c3, cs).unwrap_or_else(log_error);
        defmt::info!(
            "CHOPCONF: write value = {=u32:#x} ; read value {=u32:#x}",
            0x110140c3,
            a
        );
        let _chop =
            spi_transfer(spi, registers::CHOPCONF, 0x110140c3, cs).unwrap_or_else(log_error);
        defmt::info!(
            "CHOPCONF: write value = {=u32:#x} ; read value {=u32:#x}",
            0x110140c3,
            _chop
        );
        spi_transfer(spi, registers::GLOBAL_SCALER, 0, cs).unwrap_or_else(log_error);
        let b = spi_transfer(spi, registers::IHOLD_IRUN, 0x000f1f1f, cs).unwrap_or_else(log_error);
        defmt::info!(
            "CHOPCONF: write value = {=u32:#x} ; read value {=u32:#x}",
            0x110140c3,
            b
        );
        let _irun =
            spi_transfer(spi, registers::IHOLD_IRUN, 0x000f1f1f, cs).unwrap_or_else(log_error);
        defmt::info!(
            "IHOLD_IRUN: write value = {=u32:#x} ; read value {=u32:#x}",
            0x000f1f1f,
            _irun
        );
        spi_transfer(spi, registers::TPOWERDOWN, 10, cs).unwrap_or_else(log_error);
        spi_transfer(spi, registers::TPWMTHRS, 0, cs).unwrap_or_else(log_error);
        spi_transfer(spi, registers::PWMCONF, 0xc4000160, cs).unwrap_or_else(log_error);
        spi_transfer(spi, registers::GCONF, 0x00000005, cs).unwrap_or_else(log_error);

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

    #[task(binds = USART3, local = [rx_rpi], shared = [read_buf_rpi, stop_flag, send_data, send_delay])]
    fn usart3(mut ctx: usart3::Context) {
        defmt::trace!("USART3 interrupt");
        let serial = ctx.local.rx_rpi;

        let mut command: Option<Command> = None;

        if serial.is_rx_not_empty() {
            defmt::trace!("RX not empty");

            if let Ok(byte) = serial.read() {
                defmt::debug!("RX Byte value: {=u8:a}", byte);
                ctx.shared.read_buf_rpi.lock(|read_buf| {
                    if !read_buf.is_empty() && byte == b'\0' {
                        let command_json: Vec<u8> = read_buf.drain(..).collect();
                        defmt::info!("Command received: {=[u8]:a}", command_json);
                        command = Command::from_json(command_json.as_slice());
                    }
                    read_buf.push_back(byte);
                });
            } else {
                defmt::error!("RX failed to read");
            }
        }

        let mut send_data = false;
        ctx.shared.send_data.lock(|flag| send_data = *flag);

        if let Some(command) = command {
            match command.cmd {
                // stop command
                0 => {
                    if send_data {
                        ctx.shared.stop_flag.lock(|flag| *flag = true)
                    }
                }
                // start command
                1 => {
                    // update delay
                    if let Some(CommandData::Delay { delay_time }) = command.cmd_data {
                        ctx.shared.send_delay.lock(|delay| *delay = delay_time)
                    } else {
                        defmt::warn!("Recieved incorrect data for command 'start', expected delay");
                    }
                    // start task if stopped
                    if !send_data {
                        update_regualar_data::spawn()
                            .expect("Failed to spawn task send_regualar_data!");
                    }
                }
                // TODO motor command
                2 => todo!(),
                _ => defmt::warn!("Received unknown command: {=u8}", command.cmd),
            }
        }
    }

    #[task(local = [tx_rpi], shared = [reg_data, send_delay, stop_flag, send_data], priority = 1)]
    async fn update_regualar_data(mut ctx: update_regualar_data::Context) {
        ctx.shared.send_data.lock(|flag| *flag = true);

        loop {
            let mut stop_flag = false;
            ctx.shared.stop_flag.lock(|flag| stop_flag = *flag);

            if stop_flag {
                ctx.shared.send_data.lock(|flag| *flag = false);
                ctx.shared.stop_flag.lock(|flag| *flag = false);
                return;
            }

            let mut json_str = String::new();

            // get delay
            let mut send_delay = 0;
            ctx.shared.send_delay.lock(|delay| send_delay = *delay);

            // convert to json
            ctx.shared.reg_data.lock(|rdata| {
                json_str = rdata.to_json().unwrap_or_default();
                json_str.push('\0');
            });

            defmt::info!("Sending JSON data: {=str}", json_str);
            if ctx.local.tx_rpi.is_tx_empty() {
                defmt::trace!("TX empty");
                let _ = ctx.local.tx_rpi.bwrite_all(json_str.as_bytes()).unwrap();
            }

            Systick::delay(send_delay.millis()).await;
        }
    }
}
