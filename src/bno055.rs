#![allow(dead_code)]


use nrf52840_hal as hal;

use hal::prelude::*;
use hal::delay::Delay;
use hal::twim::Error;
use embedded_hal::blocking::i2c::{Write, WriteRead};

const CALIB_STAT_REG: u8 = 0x35;
const SYS_TRIGGER_REG: u8 = 0x3F;
const CHIP_ID_REG: u8 = 0x00;
const SYS_STATUS_REG: u8 = 0x39;
const OPR_MODE_REG: u8 = 0x3D;
const INT_EN_REG: u8 = 0x10;
const INT_MSK_REG: u8 = 0xF;
const INT_STA_REG: u8 = 0x37;
const UNIT_SEL_REG: u8 = 0x3B;
const SWREV_REG: u8 = 0x4;

pub(crate) fn setup<T: Write + WriteRead>(i2c: &mut T, delay: &mut Delay, address: u8) -> Result<(), Error>  
    where hal::twim::Error: From<<T as Write>::Error>,
          hal::twim::Error: From<<T as WriteRead>::Error>, 
          <T as Write>::Error : core::fmt::Debug , 
          <T as WriteRead>::Error : core::fmt::Debug {
//hal::twim::Error: From<<T as WriteRead>::Error>, <T as WriteRead>::Error: Debug
    let mut buf1 = [0u8; 1];

    // First confirm the chip ID
    i2c.write_read(address, &[CHIP_ID_REG], &mut buf1)?;
    if buf1[0] != 0xA0 {
        panic!("BNO055 chip ID incorrect");
    }


    // let mut swrev = [0u8; 2];
    // i2c.write_read(address, &[SWREV_REG], &mut swrev).expect("swread failed");
    // if swrev[1] < 3 || swrev[0] < 14 {
    //     //panic!("BNO055 firmware < 3.14, cannot proceed");
    // } 

    // Do a reset before configuring
    i2c.write(address, &[SYS_TRIGGER_REG, 0b0010_0000])?;

    // wait for the reset to complete - datasheet says 400ms
    delay.delay_ms(400u32);

    for _ in 0..100 {
        buf1[0] = 255;
        let res = i2c.write_read(address, &[SYS_STATUS_REG], &mut buf1);
        if res.is_err() {
            // reset is not done, keep waiting
        } else {
            if buf1[0] == 0 {
                break;
            } else if buf1[0] == 1 {
                panic!("BNO055 startup error");
            }
        }
        delay.delay_ms(5u32);
    }
    if buf1[0] != 0 {
        panic!("BNO055 never reached idle state");
    }

    // Use external crystal

    i2c.write(address, &[SYS_TRIGGER_REG, 0b1000_0000])?;

    // Enable the DRDY interrupt REQUIRES SW 3.14!
    // i2c.write(address, &[INT_EN_REG, 0b0000_0001])?;
    // i2c.write(address, &[INT_MSK_REG, 0b0000_0001])?;

    // Switch to acceleration in mg units but all others default
    i2c.write(address, &[UNIT_SEL_REG, 0b1000_0001])?;


    // Now configure for NDOF mode
    i2c.write_read(address, &[OPR_MODE_REG], &mut buf1)?;
    buf1[0] &= 0b1111_0000;
    buf1[0] |= 0b1100;
    i2c.write(address, &[OPR_MODE_REG, buf1[0]])?;

    // datasheet says 7 ms to switch modes, but there's some ambiguity that it might need an extra ~250 ms from startup... so just check for 500ms
    delay.delay_ms(7u32);
    for _ in 0..100 {
        buf1[0] = 255;
        let res = i2c.write_read(address, &[SYS_STATUS_REG], &mut buf1);
        if res.is_err() {
            // reset is not done, keep waiting
        } else {
            if buf1[0] == 5 {
                break;
            } else if buf1[0] == 1 {
                panic!("BNO055 startup error");
            }
        }
        delay.delay_ms(5u32);
    }
    if buf1[0] != 5 {
        panic!("BNO055 never reached fusion data state");
    }
    Ok(())
}


pub(crate) fn read_calib_status<T: WriteRead>(i2c: &mut T, address: u8) -> Result<u8, Error>  
    where hal::twim::Error: From<<T as WriteRead>::Error> {

    let mut buf = [0u8; 1];
    i2c.write_read(address, &[CALIB_STAT_REG], &mut buf)?;

    Ok(buf[0])
        
}

// REQUIRES SW 3.14
// pub(crate) fn is_new_data<T: WriteRead>(i2c: &mut T, address: u8) -> Result<bool, Error>  
// where hal::twim::Error: From<<T as WriteRead>::Error> {
//     let mut buf = [0u8; 1];
//     i2c.write_read(address, &[INT_STA_REG], &mut buf)?;

//     Ok(buf[0] & 0b0000_0001 != 0)
// }


pub(crate) fn read_data<T: WriteRead>(i2c: &mut T, address: u8) -> Result<BNO055Data, Error>  
    where hal::twim::Error: From<<T as WriteRead>::Error> {

    // the datasheet is rather confusing about the order of the registers, but this seems to get the quats and linear acceleration in the right order?
    let mut buf = [0u8; 6+8];
    i2c.write_read(address, &[0x20], &mut buf)?;
    
    let qw = ((buf[1] as i16) << 8) | buf[0] as i16;
    let qx = ((buf[3] as i16) << 8) | buf[2] as i16;
    let qy = ((buf[5] as i16) << 8) | buf[4] as i16;
    let qz = ((buf[7] as i16) << 8) | buf[6] as i16;
    let liax = ((buf[9] as i16) << 8) | buf[8] as i16;
    let liay = ((buf[11] as i16) << 8) | buf[10] as i16;
    let liaz = ((buf[13] as i16) << 8) | buf[12] as i16;

    let data = BNO055Data {
        liax: liax,
        liay: liay,
        liaz: liaz,
        qw: qw,
        qx: qx,
        qy: qy,
        qz: qz,
    };
    Ok(data)
}

// in raw units - accel is mg, quaternions are 2**14 signed
#[derive(Debug)]
pub(crate) struct BNO055Data {
    pub liax: i16,
    pub liay: i16,
    pub liaz: i16,
    pub qw: i16,
    pub qx: i16,
    pub qy: i16,
    pub qz: i16,
}

impl BNO055Data {
    pub(crate) fn to_float(&self) -> BNO055DataF {
        BNO055DataF {
            liax: self.liax as f32 / 1000.,
            liay: self.liay as f32 / 1000.,
            liaz: self.liaz as f32 / 1000.,
            qw: self.qw as f32 / 16384.,
            qx: self.qx as f32 / 16384.,
            qy: self.qy as f32 / 16384.,
            qz: self.qz as f32 / 16384.,
        }
    }

    pub(crate) fn to_bytes(&self) -> [u8; 14] {
        let mut buf = [0u8; 14];

        buf[0..2].clone_from_slice(&self.qw.to_le_bytes());
        buf[2..4].clone_from_slice(&self.qx.to_le_bytes());
        buf[4..6].clone_from_slice(&self.qy.to_le_bytes());
        buf[6..8].clone_from_slice(&self.qz.to_le_bytes());
        buf[8..10].clone_from_slice(&self.liax.to_le_bytes());
        buf[10..12].clone_from_slice(&self.liay.to_le_bytes());
        buf[12..14].clone_from_slice(&self.liaz.to_le_bytes());

        buf
    }
}


// accel in g, quaternions is normalized quaternion units
#[derive(Debug)]
pub(crate) struct BNO055DataF {
    pub liax: f32,
    pub liay: f32,
    pub liaz: f32,
    pub qw: f32,
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
}
