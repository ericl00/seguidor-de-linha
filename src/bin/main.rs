#![no_std]
#![no_main]

use esp_hal::{
    main,
    clock::CpuClock,
    delay::Delay,
    ledc::{
        channel::{self, ChannelIFace},
        timer,
        LSGlobalClkSource,
        Ledc,
        LowSpeed,
        timer::TimerIFace,
    },
    time::Rate,
    timer::timg::TimerGroup,
    analog::adc::{self, Attenuation},
};


const PWM_FREQ_KHZ: u32 = 20;
const DELAY_LOOP_PRINCIPAL: u32 = 30;


#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

fn find_deviation(esquerda: u16, centro: u16, direita: u16) -> f32 {
    let valor_sensores = esquerda + centro + direita;

    let desvio = if valor_sensores != 0 { (esquerda as i16 * -1 + direita as i16 * 1) as f32 / valor_sensores as f32 } else { 0.0 };

    desvio
}

fn calc_pwm(desvio: f32) -> (u8, u8) {
    if desvio == 0.0 { return (100, 100) }
    else if desvio < 0.0 {
        let pct = (desvio * 100f32) as u8;
        return (100 - pct, 100);
    } else if desvio > 0.0 {
        let pct = (desvio * 100f32) as u8;
        return (100, 100 - pct);
    }
    (0, 0)
}

#[main]
fn main() -> ! {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut delay = Delay::new();

    let out_esq = peripherals.GPIO13;
    let out_dir = peripherals.GPIO12;

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(PWM_FREQ_KHZ),
        }).unwrap();

    let mut mot_esq: channel::Channel<'_, LowSpeed> = ledc.channel(channel::Number::Channel0, out_esq);
    let mut mot_dir: channel::Channel<'_, LowSpeed> = ledc.channel(channel::Number::Channel1, out_dir);

    let mut adc_config = adc::AdcConfig::new();

    let analog_pin_esq = peripherals.GPIO33;
    let analog_pin_ctr = peripherals.GPIO34;
    let analog_pin_dir = peripherals.GPIO35;

    let mut pin_esq = adc_config.enable_pin(
        analog_pin_esq,
        Attenuation::_11dB,
    );
    let mut pin_ctr = adc_config.enable_pin(
        analog_pin_ctr,
        Attenuation::_11dB,
    );
    let mut pin_dir = adc_config.enable_pin(
        analog_pin_dir,
        Attenuation::_11dB,
    );

    let mut adc_sensors = adc::Adc::new(peripherals.ADC1, adc_config);

    loop {
        let desvio = find_deviation(
            adc_sensors.read_oneshot(&mut pin_esq).unwrap(),
            adc_sensors.read_oneshot(&mut pin_ctr).unwrap(),
            adc_sensors.read_oneshot(&mut pin_dir).unwrap()
        );

        let (pwm_esq, pwm_dir) = calc_pwm(desvio);

        mot_esq.set_duty(pwm_esq).unwrap();
        mot_dir.set_duty(pwm_dir).unwrap();

        delay.delay_millis(DELAY_LOOP_PRINCIPAL);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}
