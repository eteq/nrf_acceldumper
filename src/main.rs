#![no_std]
#![no_main]

use panic_persist;

mod mpu6050;

use cortex_m_rt::entry;


use nrf52840_hal as hal;

#[allow(unused_imports)]
use hal::prelude::*;
use hal::clocks::Clocks;
use hal::usbd::{UsbPeripheral, Usbd};
use hal::rtc::Rtc;
use hal::gpio::Level;
use hal::spim;
use hal::twim;

use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use heapless;
use core::fmt::Write;


#[allow(unused_imports)]
use smart_leds::{
    SmartLedsWrite,
    RGB,
    hsv::{hsv2rgb, Hsv},
};
use apa102_spi::Apa102;

// prescalar -> 64, freq = 512 hz -> 1.953125 ms period, ~8 hr overflow
const RTC_PRESCALAR : u32 = 64;
const RTC_FREQ : u32 = 512;

const MPU6050_ADDR1 : u8 = 0x68;
const MPU6050_ADDR2 : u8 = 0x69;
const BNO055_ADDR : u8 = 0x28;

#[entry]
fn main() -> ! {
    let panic_msg = panic_persist::get_panic_message_bytes();  // None means no panic last round
    
    let cperiph = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = hal::delay::Delay::new(cperiph.SYST);

    let periph = hal::pac::Peripherals::take().unwrap();
    let clocks = Clocks::new(periph.CLOCK).enable_ext_hfosc().set_lfclk_src_synth().start_lfclk();


    let rtc = Rtc::new(periph.RTC0, RTC_PRESCALAR-1).unwrap();
    rtc.enable_counter();

    let port0 = hal::gpio::p0::Parts::new(periph.P0);
    let port1 = hal::gpio::p1::Parts::new(periph.P1);
    //let mut red_led = port0.p0_06.into_push_pull_output(Level::Low);
    
    let dotstar_pins = spim::Pins {
        sck: Some(port1.p1_09.into_push_pull_output(Level::Low).degrade()),
        miso: None,
        mosi: Some(port0.p0_08.into_push_pull_output(Level::Low).degrade()),
    };
    let dotstar_spi = spim::Spim::new(
        periph.SPIM2,
        dotstar_pins,
        spim::Frequency::K250,
        spim::MODE_0,
        0,
    );
    let mut dotstar = Apa102::new(dotstar_spi);
    // light up blue to indicate startup phase
    dotstar.write([RGB{r:0, g:0, b:25}].into_iter()).expect("dotstar write failed");

    let scl = port0.p0_14.into_floating_input().degrade();
    let sda = port0.p0_16.into_floating_input().degrade();
    let mut i2c = twim::Twim::new(periph.TWIM0, 
                              twim::Pins { scl, sda }, 
                              twim::Frequency::K100);
    

    // set up usb serial
    let usb_bus = UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(periph.USBD, &clocks)));
    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();

    let mut scratch_string: heapless::String<256> = heapless::String::new();

    // For some reason this is long enough that it sits and waits for input on the computer side, which is the desired behavior...
    let rtc_end = rtc.get_counter() + RTC_FREQ*3; // 3 sec
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        if rtc.get_counter() >= rtc_end {
            break;
        }
    }
    
    // Now print out the panic message if there is one
    if panic_msg.is_some() {
        serial.write(b"\r\n\r\n").expect("Failed to write to serial usb");
        serial.write( b"Panic! Message below, pausing for reset:\r\n").expect("Failed to write to serial usb");
        serial.write(panic_msg.unwrap()).expect("Failed to write to serial usb");
        serial.write(b"\r\n\r\n").expect("Failed to write to serial usb");

        // wait a few secs to print out over the usb fully, then pause and await a reset
        let rtc_end = rtc.get_counter() + RTC_FREQ;
        loop {
            if !usb_dev.poll(&mut [&mut serial]) {
                continue;
            }
            if rtc.get_counter() >= rtc_end {
                cortex_m::asm::wfi();  // this pauses for a reset
            }
        }
    }
       
    // set up the mpu6050 accelerometers.  Note theres a few hundred ms delays for various things here
    mpu6050::mpu6050_setup(&mut i2c, &mut delay, MPU6050_ADDR1).expect("mpu6050_setup failed for addr 0x68");
    mpu6050::mpu6050_setup(&mut i2c, &mut delay, MPU6050_ADDR2).expect("mpu6050_setup failed for addr 0x68");

    // startup completed
    dotstar.write([RGB{r:0, g:0, b:0}].into_iter()).expect("dotstar write failed");

    let mut counts = 0;
    let mut next_rtc = rtc.get_counter() + RTC_FREQ;
    
    mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR1).expect("mpu6050_reset_fifo failed for addr 0x68");
    mpu6050::mpu6050_reset_fifo(&mut i2c, MPU6050_ADDR2).expect("mpu6050_reset_fifo failed for addr 0x69");
    // main loop
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
    
        // all is well, proceed as normal
        if rtc.get_counter() >= next_rtc {

            let q1 = mpu6050::mpu6050_read_fifo(&mut i2c, MPU6050_ADDR1).unwrap().q_to_float();
            let q2 = mpu6050::mpu6050_read_fifo(&mut i2c, MPU6050_ADDR2).unwrap().q_to_float();

            scratch_string.clear();
            write!(scratch_string, "Hello {counts}! {q1:?} {q2:?}\r\n").expect("write! failed");
            serial.write(scratch_string.as_bytes()).expect("Failed to write to serial usb");

            match counts % 3 {
                0 => { dotstar.write([RGB{r:25, g:0, b:0}].into_iter()).expect("dotstar write failed"); },
                1 => { dotstar.write([RGB{r:0, g:25, b:0}].into_iter()).expect("dotstar write failed"); },
                2 => { dotstar.write([RGB{r:0, g:0, b:25}].into_iter()).expect("dotstar write failed"); },
                _ => { panic!("impossible"); }
            }

            counts += 1;
            if counts > 10 {
                panic!("too many counts");
            }
            next_rtc += RTC_FREQ;
        }
    }
}