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
use alloc::vec::Vec;

// quaternion
struct _Quat(Vec<u8>);

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI1])]
mod app {
    // Use HAL crate for stm32f407
    // use cortex_m::singleton;
    // use heapless::spsc::{Consumer, Producer, Queue};
    use alloc::vec;
    use alloc::vec::Vec;
    use stm32f4xx_hal::{
        pac::USART2,
        prelude::*,
        serial::{config::Config, Event, Rx, Serial, Tx},
    };

    // Setup heap allocator for rust collections
    use embedded_alloc::Heap;

    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    #[shared]
    struct Shared {
        buf: Vec<u8>,
    }

    // const QUEUE_LEN: usize = 8;

    #[local]
    struct Local {
        // tx_prod: Producer<'static, u8, QUEUE_LEN>,
        // tx_con: Consumer<'static, u8, QUEUE_LEN>,
        // rx_prod: Producer<'static, u8, QUEUE_LEN>,
        // rx_con: Consumer<'static, u8, QUEUE_LEN>,
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
        let clocks = rcc.cfgr.sysclk(25.MHz()).freeze();
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
        // let rx_queue = singleton!(:Queue<u8, QUEUE_LEN> = Queue::new()).unwrap();
        // let tx_queue = singleton!(:Queue<u8, QUEUE_LEN> = Queue::new()).unwrap();

        let buf: Vec<u8> = vec![];
        (
            Shared {
                // Initialization of shared resources go here
                buf,
            },
            Local {
                // Initialization of local resources go here
                // tx_con,
                // tx_prod,
                // rx_con,
                // rx_prod,
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
    fn print_quat(mut ctx: print_quat::Context) {
        // todo!();
        ctx.shared.buf.lock(|buf| {
            if buf[0] == 0x55 {
                let checksum = 0x55
                    + buf[1]
                    + buf[2]
                    + buf[3]
                    + buf[4]
                    + buf[5]
                    + buf[6]
                    + buf[7]
                    + buf[8]
                    + buf[9];

                if checksum == buf[10] {
                    if buf[1] == 0x59 {
                        // Quaternion construction
                    }
                }
            }
        });
    }

    #[task(binds = USART2, local = [tx, rx], shared = [buf])]
    fn usart2(mut ctx: usart2::Context) {
        defmt::debug!("USART2 interrupt");
        // let tx = ctx.local.tx;
        let rx = ctx.local.rx;

        rx.unlisten();

        // if tx.is_tx_empty() {
        //     defmt::println!("TX interrupt");
        //     let _ = tx.write(0xFF);
        //     tx.unlisten();
        // }

        if rx.is_rx_not_empty() {
            if let Ok(byte) = rx.read() {
                defmt::info!("Byte value: {:x}", byte);

                // wait for header byte
                ctx.shared.buf.lock(|buf| {
                    if buf.is_empty() {
                        if byte == 0x55 {
                            buf.push(byte);
                        }
                    } else {
                        buf.push(byte);
                    }

                    // message complete, send to quat print
                    if buf.len() == 11 {
                        print_quat::spawn().unwrap();
                    }
                });
            }
        }

        rx.listen();
    }
}

#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
