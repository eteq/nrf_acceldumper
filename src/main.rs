#![no_std]
#![no_main]

use hal::prelude::OutputPin;
use panic_persist;

//mod mpu6050;

use cortex_m_rt::entry;


use nrf52840_hal as hal;

#[allow(unused_imports)]
use hal::prelude::*;
use hal::clocks::Clocks;
use hal::usbd::{UsbPeripheral, Usbd};
use hal::rtc::Rtc;
use hal::gpio::Level;
use hal::spim;

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

#[entry]
fn main() -> ! {
    let panic_msg = panic_persist::get_panic_message_bytes();  // None means no panic last round
    
    let periph = hal::pac::Peripherals::take().unwrap();
    let clocks = Clocks::new(periph.CLOCK).enable_ext_hfosc().set_lfclk_src_synth().start_lfclk();


    let rtc = Rtc::new(periph.RTC0, RTC_PRESCALAR-1).unwrap();
    rtc.enable_counter();

    let port0 = hal::gpio::p0::Parts::new(periph.P0);
    let port1 = hal::gpio::p1::Parts::new(periph.P1);
    let mut red_led = port0.p0_06.into_push_pull_output(Level::Low);
    
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
    red_led.set_high().expect("LED set failed");
    let rtc_end = rtc.get_counter() + RTC_FREQ*3; // 3 sec
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        if rtc.get_counter() >= rtc_end {
            break;
        }
    }
    red_led.set_low().expect("LED set failed");

    let mut panic_msg_written = false;
    let mut counts = 0;
    let mut next_rtc = rtc.get_counter() + RTC_FREQ;
    // main loop
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        
        match panic_msg {
            Some(msg) => {
                red_led.set_high().expect("LED set_high failed");
                if !panic_msg_written {
                    serial.write(b"\r\n\r\n").expect("Failed to write to serial usb");
                    serial.write( b"Previous panic message:\r\n").expect("Failed to write to serial usb");
                    serial.write(msg).expect("Failed to write to serial usb");
                    serial.write(b"\r\n\r\n").expect("Failed to write to serial usb");
                    panic_msg_written = true;
                    next_rtc = rtc.get_counter() + RTC_FREQ*3; // 3 sec wait and then reset
                } else if rtc.get_counter() >= next_rtc {
                    cortex_m::peripheral::SCB::sys_reset();
                }
            },
            None => {
                // all is well, proceed as normal
                if rtc.get_counter() >= next_rtc {

                    scratch_string.clear();
                    write!(scratch_string, "Hello world! {} {}\r\n", counts, rtc.get_counter()).expect("write! failed");
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
    }
}