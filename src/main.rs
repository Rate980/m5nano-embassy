#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::{debug, info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::{digital::Wait, i2c::I2c};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    embassy,
    gpio::{AnyPin, Input, Output, PullDown, PushPull, IO},
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals, I2C0},
    prelude::*,
    systimer::SystemTimer,
    Async,
};

type LedType = Mutex<CriticalSectionRawMutex, Option<AnyPin<Output<PushPull>>>>;
static LED: LedType = Mutex::new(None);
type I2cType = Mutex<CriticalSectionRawMutex, Option<I2C<'static, I2C0, Async>>>;
static I2C0: I2cType = Mutex::new(None);

#[embassy_executor::task(pool_size = 2)]
async fn toggle_led(led: &'static LedType, delay: Duration) {
    let mut ticker = Ticker::every(delay);
    loop {
        {
            let mut led_unlocked = led.lock().await;
            if let Some(pin_ref) = led_unlocked.as_mut() {
                debug!("toggle");
                pin_ref.toggle();
            }
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn input(mut button: AnyPin<Input<PullDown>>) {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let mut count: u8 = 0;
    loop {
        unwrap!(Wait::wait_for_falling_edge(&mut button).await);
        count = count.wrapping_add(1);
        info!("ping! {}", count);
        ticker.next().await;
    }
}
#[main]
async fn main(spawner: Spawner) {
    info!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let systimer = SystemTimer::new_async(peripherals.SYSTIMER);
    embassy::init(&clocks, systimer);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = io.pins.gpio7.into_push_pull_output();
    let button = io.pins.gpio9.into_pull_down_input();
    io.pins.gpio19.into_push_pull_output().set_low();
    let scl = io.pins.gpio1;
    let sda = io.pins.gpio2;

    let i2c0 = I2C::new_async(peripherals.I2C0, sda, scl, 400.kHz(), &clocks);

    unwrap!(interrupt::enable(
        Interrupt::GPIO,
        interrupt::Priority::Priority1
    ));
    {
        *(I2C0.lock().await) = Some(i2c0);
    }

    {
        *(LED.lock().await) = Some(AnyPin::from(led))
    }

    let dt = 100 * 1_000_000;
    let k = 1.003;
    let dt2 = (dt as f32 * k) as u64;
    info!("dt: {}", dt);
    info!("dt2: {}", dt2);
    unwrap!(spawner.spawn(toggle_led(&LED, Duration::from_nanos(dt))));
    unwrap!(spawner.spawn(toggle_led(&LED, Duration::from_nanos(dt2))));
    unwrap!(spawner.spawn(input(AnyPin::from(button))));

    // loop {
    //     let mut buf = [0x00; 100];
    //     {
    //         let mut i2c0 = I2C0.lock().await;
    //         if let Some(i2c0) = i2c0.as_mut() {
    //             unwrap!(I2c::read(i2c0, 0x40, &mut buf).await);
    //             unwrap!(I2c::write(i2c0, 0x40, &buf).await);
    //         }
    //     }
    //     Timer::after(Duration::from_millis(1)).await;
    // }
}
