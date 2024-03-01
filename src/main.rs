#![no_std]
#![no_main]

use panic_persist;

mod mpu6050;
mod bno055;

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

const DSBRIGHTNESS : u8 = 25;

#[entry]
fn main() -> ! {
    let panic_msg = panic_persist::get_panic_message_bytes();  // None means no panic last round
    
    let cperiph = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = hal::delay::Delay::new(cperiph.SYST);

    let periph = hal::pac::Peripherals::take().unwrap();
    let clocks = Clocks::new(periph.CLOCK).enable_ext_hfosc().set_lfclk_src_synth().start_lfclk();


    
    let rtc = Rtc::new(periph.RTC0, RTC_PRESCALAR-1).unwrap();
    rtc.enable_counter();

    // the current Timer in the HAL doesn't work the way we want it to so we are stuck with this approach.
    setup_timer(&periph.TIMER0);

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
    dotstar.write([RGB{r:0, g:0, b:DSBRIGHTNESS}].into_iter()).expect("dotstar write failed");

    let scl = port0.p0_14.into_floating_input().degrade();
    let sda = port0.p0_16.into_floating_input().degrade();
    let mut i2c = twim::Twim::new(periph.TWIM0, 
                              twim::Pins { scl, sda }, 
                              twim::Frequency::K400);
    

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

    let mut scratch_string: heapless::String<2560> = heapless::String::new();

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
    mpu6050::setup(&mut i2c, &mut delay, MPU6050_ADDR1).expect("mpu6050 setup failed for addr 0x68");
    mpu6050::setup(&mut i2c, &mut delay, MPU6050_ADDR2).expect("mpu6050 setup failed for addr 0x68");

    bno055::setup(&mut i2c, &mut delay, BNO055_ADDR).expect("bno055 setup failed");

    // startup completed
    dotstar.write([RGB{r:0, g:0, b:0}].into_iter()).expect("dotstar write failed");

    let mut next_time = read_timer(&periph.TIMER0) + TIMER_CYCLE_10MS;

    let mut bno_calibrated = false;
    
    mpu6050::reset_fifo(&mut i2c, MPU6050_ADDR1).expect("mpu6050 reset_fifo failed for addr 0x68");
    mpu6050::reset_fifo(&mut i2c, MPU6050_ADDR2).expect("mpu6050 reset_fifo failed for addr 0x69");

    // main loop. Here we look for both FIFOs >= 30 and interrupt set for BNo055 - read all 3 in that case, store time.  Print it out.  Watch for FIFO overflow ()>60) and reset if it occurred
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let tcounts = read_timer(&periph.TIMER0);

        if tcounts > 4000000000 {
            // timer is approaching overflow.  Reset everything and start over 
            setup_timer(&periph.TIMER0);
            next_time = TIMER_CYCLE_10MS;
            continue;
        }

        if !bno_calibrated {
            let calib = bno055::read_calib_status(&mut i2c, BNO055_ADDR).unwrap();
            
            // scratch_string.clear();
            // write!(scratch_string, "BNO055 calib: {:08b}\r\n", calib).expect("write to string failed");
            // serial.write(scratch_string.as_bytes()).expect("Failed to write to serial usb");

            let mut rgb = RGB{r:0, g:0, b:0};
            // fusion calibration
            if calib == 0b11111111 {
                rgb.r = DSBRIGHTNESS;
                rgb.g = DSBRIGHTNESS;
                rgb.b = DSBRIGHTNESS;
                bno_calibrated = true;
            } else {
                bno_calibrated = false;
                // system/fusion calibration
                if calib & 0b1100_0000 == 0b1100_0000 {
                    rgb.b = DSBRIGHTNESS;
                }
                // /// gyroscope skipped because it's almost always good
                // if calib & 0b0011_0000 == 0b0011_0000 {
                //     rgb.b = 0;
                // }
                //accelerometer
                if calib & 0b0000_1100 == 0b0000_1100 {
                    rgb.r = DSBRIGHTNESS;
                }
                //magnetometer
                if calib & 0b0000_0011 == 0b0000_0011 {
                    rgb.g = DSBRIGHTNESS;
                }
            }

            dotstar.write([rgb].into_iter()).expect("dotstar write failed");

            // keep waiting if calibration isn't done
            continue;

        }

        if tcounts >= next_time {
            let rtc_time = (rtc.get_counter() as f32) / (RTC_FREQ as f32);

            //check that the fifos both have a full dataset before we go any further
            let mut c1 = mpu6050::get_fifo_count(&mut i2c, MPU6050_ADDR1).unwrap() as usize;
            let mut c2 = mpu6050::get_fifo_count(&mut i2c, MPU6050_ADDR2).unwrap() as usize;
            if (c1 >= mpu6050::FIFO_SIZE) && (c2 >= mpu6050::FIFO_SIZE) {


                let by3 = bno055::read_data(&mut i2c, BNO055_ADDR).unwrap().to_bytes();

                // first read out whatever's in the MPU fifos until we get to the last one
                let mut by1 = mpu6050::read_fifo(&mut i2c, MPU6050_ADDR1).unwrap().to_bytes();
                c1 = mpu6050::get_fifo_count(&mut i2c, MPU6050_ADDR1).unwrap() as usize;
                while c1 >= mpu6050::FIFO_SIZE {
                    by1 = mpu6050::read_fifo(&mut i2c, MPU6050_ADDR1).unwrap().to_bytes();
                    c1 = mpu6050::get_fifo_count(&mut i2c, MPU6050_ADDR1).unwrap()  as usize;
                }

                let mut by2 = mpu6050::read_fifo(&mut i2c, MPU6050_ADDR1).unwrap().to_bytes();
                c2 = mpu6050::get_fifo_count(&mut i2c, MPU6050_ADDR1).unwrap()  as usize;
                while c2 >= mpu6050::FIFO_SIZE {
                    by2 = mpu6050::read_fifo(&mut i2c, MPU6050_ADDR1).unwrap().to_bytes();
                    c2 = mpu6050::get_fifo_count(&mut i2c, MPU6050_ADDR1).unwrap()  as usize;
                }


                scratch_string.clear();
                write!(scratch_string, "{rtc_time},").expect("write to string failed");

                serial.write(b"Q1,").expect("Failed to write to serial usb");
                serial.write(scratch_string.as_bytes()).expect("Failed to write to serial usb");
                serial.write(&by1[..8]).expect("Failed to write to serial usb");
                serial.write(b"\r\n").expect("Failed to write to serial usb");

                serial.write(b"Q2,").expect("Failed to write to serial usb");
                serial.write(scratch_string.as_bytes()).expect("Failed to write to serial usb");
                serial.write(&by2[..8]).expect("Failed to write to serial usb");
                serial.write(b"\r\n").expect("Failed to write to serial usb");

                serial.write(b"Q3,").expect("Failed to write to serial usb");
                serial.write(scratch_string.as_bytes()).expect("Failed to write to serial usb");
                serial.write(&by3[..8]).expect("Failed to write to serial usb");
                serial.write(b"\r\n").expect("Failed to write to serial usb");

                serial.write(b"A3,").expect("Failed to write to serial usb");
                serial.write(scratch_string.as_bytes()).expect("Failed to write to serial usb");
                serial.write(&by3[8..14]).expect("Failed to write to serial usb");
                serial.write(b"\r\n").expect("Failed to write to serial usb");

                next_time += TIMER_CYCLE_10MS;

                // reset the fifos since we might be slipping a smidge

                if c1 > 28 {
                    mpu6050::reset_fifo(&mut i2c, MPU6050_ADDR1).expect("mpu6050 reset_fifo failed for addr 0x68");
                }

                if c2 > 28 {
                    mpu6050::reset_fifo(&mut i2c, MPU6050_ADDR2).expect("mpu6050 reset_fifo failed for addr 0x68");
                }
            }
        }

    }
}



const TIMER_CYCLE_10MS : u32 = 625;

fn setup_timer(timer: &hal::pac::timer0::RegisterBlock) {
    // stop and clear
    timer.tasks_stop.write(|w| unsafe { w.bits(1) });
    timer.tasks_clear.write(|w| unsafe { w.bits(1) });

    // set it up as a 62.5 kHz 32 bit timer - 625 steps is then 10 ms = 100 Hz, overflow takes 19 hr
    timer.bitmode.write(|w| w.bitmode()._32bit());
    timer.mode.write(|w| w.mode().timer());
    timer.prescaler.write(|w| unsafe { w.prescaler().bits(8) });  // 16 MHz / 256 = 62.5 kHz
    // also turn off the interrupt
    timer.intenclr.write(|w| w.compare0().clear());

    // start
    timer.tasks_start.write(|w| unsafe { w.bits(1) });
}

fn read_timer(timer: &hal::pac::timer0::RegisterBlock) -> u32 {
    timer.tasks_capture[0].write(|w| unsafe { w.bits(1) });
    timer.cc[0].read().bits()
}