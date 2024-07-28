#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem::MaybeUninit;

use defmt as _;
use embassy_executor::{task, Spawner};
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    dma::*,
    gpio::{AnyOutput, GpioPin, Io, Level, Output},
    interrupt,
    peripherals::{Peripherals, SPI2},
    prelude::*,
    rng::Rng,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    system::SystemControl,
    timer::{timg::TimerGroup, OneShotTimer, PeriodicTimer},
};
use esp_hal_embassy::InterruptExecutor;
use esp_println as _;
use esp_wifi::{self, wifi::WifiApDevice, EspWifiInitFor};
use static_cell::{make_static, StaticCell};

#[embassy_executor::task]
pub async fn imu_task(
    mut cs_output: AnyOutput<'static>,
    dma_channel: esp_hal::dma::ChannelCreator<0>,
    spi: Spi<'static, SPI2, FullDuplexMode>,
) {
    defmt::info!("Starting IMU task");

    Timer::after_secs(1).await;

    let mut ticker = Ticker::every(Duration::from_hz(1000));
    loop {
        ticker.next().await;
    }
}
#[embassy_executor::task]
pub async fn control_led(mut led: esp_hal::gpio::AnyOutput<'static>) {
    defmt::info!(
        "Starting control_led() on core {}",
        esp_hal::get_core() as usize
    );

    let duration = Duration::from_hz(10);
    let mut led_state = Level::Low;
    loop {
        // if let Some(duration_new) = control.try_take() {
        //     defmt::info!("Control signal received: {:?}", duration_new);
        //     duration = duration_new;
        // }

        led_state = match led_state {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };

        // Flash the led
        led.set_level(led_state);

        // Wait for the duration
        embassy_time::Timer::after(duration).await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    let timers = make_static!([OneShotTimer::new(systimer.alarm0.into())]);

    // let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None);
    // let timers = make_static!([OneShotTimer::new(timg1.timer0.into())]);

    // let timers = esp_hal::timer::systimer::SystemTimer::new_async(peripherals.SYSTIMER);
    esp_hal_embassy::init(&clocks, timers);

    // init_heap();

    // Spawners

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio33;
    let mosi = io.pins.gpio40;
    let miso = io.pins.gpio47;
    let cs = io.pins.gpio34;
    let baro_cs = io.pins.gpio11;

    // Prevent the barometer from operating in I2C mode
    let mut baro_cs = Output::new(baro_cs, Level::High);
    baro_cs.set_low();
    Timer::after_millis(1).await;
    baro_cs.set_high();

    Timer::after_secs(2).await;

    // --- WIFI ---
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let wifi_timer = PeriodicTimer::new(timg0.timer0.into());

    let wifi = peripherals.WIFI;
    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    // spawner.spawn(network::wifi_test_task(init, wifi)).unwrap();

    Timer::after_secs(1).await;

    // --- IMU ---
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let spi = Spi::new(peripherals.SPI2, 24.MHz(), SpiMode::Mode0, &clocks)
        .with_sck(sclk)
        .with_miso(miso)
        .with_mosi(mosi);

    spawner
        .spawn(imu_task(AnyOutput::new(cs, Level::High), dma_channel, spi))
        .unwrap();

    let led = io.pins.gpio5;
    spawner
        .spawn(control_led(AnyOutput::new(led, Level::Low)))
        .ok();

    // A never-ending heartbeat
    loop {
        Timer::after_secs(5).await;
        defmt::info!("Still alive!");
    }
}
