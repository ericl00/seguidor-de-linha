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
use esp_wifi::{
    self,
    wifi::AuthMethod::WPA2Personal,
};
use smoltcp::{
    self,
    iface::{
        Interface,
        SocketSet,
        SocketStorage
    },
    socket::{
            tcp::{
            Socket as TcpSocket,
            SocketBuffer
        },
    },
    time::Instant,
    wire::{
        EthernetAddress,
        HardwareAddress,
        IpCidr,
        Ipv4Address,
        Ipv4Cidr,
    }
};
use esp_println::println;

extern crate nb;

const PWM_FREQ_KHZ: u32 = 20;
const DELAY_LOOP_PRINCIPAL: u32 = 30;

const RX_BUFFER_SIZE: usize = 1024;
const TX_BUFFER_SIZE: usize = 1024;
const NUM_SOCKETS: usize = 1;


#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("panic!: {:?}", info);
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
        let pct = (desvio.abs() * 100f32) as u8;
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
    println!("inicio");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_controller = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    ).unwrap();

    let (mut wifi_controller, mut wifi_interfaces) = esp_wifi::wifi::new(&esp_wifi_controller, peripherals.WIFI).unwrap();

    let wifi_config = esp_wifi::wifi::Configuration::AccessPoint(esp_wifi::wifi::AccessPointConfiguration {
        ssid: { let mut string =  heapless::String::new(); string.push_str("robo").unwrap(); string },
        password: { let mut string =  heapless::String::new(); string.push_str("seguidor").unwrap(); string },
        ssid_hidden: false,
        max_connections: 1,
        auth_method: WPA2Personal,
        ..Default::default()
    });

    wifi_controller.set_configuration(&wifi_config).unwrap();
    wifi_controller.start().unwrap();

    // configuração smoltcp

    let mac_address = HardwareAddress::Ethernet(EthernetAddress::from_bytes(&wifi_interfaces.ap.mac_address()));
    let ip_address = Ipv4Cidr::new(Ipv4Address::new(192, 168, 4, 1), 24);
    let ip_addr = IpCidr::Ipv4(ip_address);
    let mut ip_addrs = [ip_address];

    let mut rx_buffer = [0u8; RX_BUFFER_SIZE];
    let mut tx_buffer = [0u8; TX_BUFFER_SIZE];

    let mut tcp_socket = TcpSocket::new(
        SocketBuffer::new(&mut rx_buffer[..]),
        SocketBuffer::new(&mut tx_buffer[..]),
    );

    let mut socket_storage = [SocketStorage::EMPTY; NUM_SOCKETS];
    let mut socket_set = SocketSet::new(&mut socket_storage[..]);
    let tcp_handle = socket_set.add(tcp_socket);

    let smoltcp_config = smoltcp::iface::Config::new(mac_address);

    let mut interface = Interface::new(
        smoltcp_config,
        &mut wifi_interfaces.ap,
        Instant::from_millis(esp_hal::time::Instant::duration_since_epoch(&esp_hal::time::Instant::now()).as_millis() as i64)
    );

    interface.update_ip_addrs(|f| {
       f.push(ip_addr).unwrap();
    });

    println!("smoltcp Server: Inicializado, Ip: {:?}", interface.ipv4_addr().unwrap());
    println!("smoltcp Server: Escutando na porta 8080...");

    // fim configuração smoltcp

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

    mot_esq.configure(
        channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull
        }
    ).unwrap();

    mot_dir.configure(
        channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull
        }
    ).unwrap();

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
        // smoltcp

        let timestamp = Instant::from_millis(esp_hal::time::Instant::duration_since_epoch(&esp_hal::time::Instant::now()).as_millis() as i64);
        interface.poll(timestamp, &mut wifi_interfaces.ap, &mut socket_set);

        let mut socket = socket_set.get_mut::<TcpSocket>(tcp_handle);

        if !socket.is_open() {
            socket.listen(8080).unwrap();
            println!("socket listening");
        }

        if socket.may_recv() {
            let data = socket.recv(|recv| {
                println!("tcp:8080 recv data: {:?}", recv);
                let mut data = recv.to_vec();
                (data.len(), data)
            }).unwrap();
            let text = core::str::from_utf8(data.as_slice()).unwrap();
            println!("tcp:8080 text: {:?}", text);
        }

        if socket.can_send() {
            socket.abort()
        }

        // smoltcp

        let esq = nb::block!(adc_sensors.read_oneshot(&mut pin_esq)).unwrap();
        let ctr = nb::block!(adc_sensors.read_oneshot(&mut pin_ctr)).unwrap();
        let dir = nb::block!(adc_sensors.read_oneshot(&mut pin_dir)).unwrap();

        //let desvio = find_deviation(
        // adc_sensors.read_oneshot(&mut pin_esq).unwrap(),
        // adc_sensors.read_oneshot(&mut pin_ctr).unwrap(),
        // adc_sensors.read_oneshot(&mut pin_dir).unwrap()
        //);

        let desvio = find_deviation(esq, ctr, dir);

        let (pwm_esq, pwm_dir) = calc_pwm(desvio);

        mot_esq.set_duty(pwm_esq).unwrap();
        mot_dir.set_duty(pwm_dir).unwrap();

        delay.delay_millis(DELAY_LOOP_PRINCIPAL);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}