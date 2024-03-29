#![allow(dead_code)]


mod dmp_firmware;

use micromath::F32Ext;

use nrf52840_hal as hal;

use hal::prelude::*;
use hal::delay::Delay;
use hal::twim::Error;
use embedded_hal::blocking::i2c::{Write, WriteRead};

#[allow(unused_imports)]
pub(crate) use dmp_firmware::DMP_PACKET_SIZE;

const ACCEL_SCALE : f32 = 2.0;


pub(crate) fn setup<T: Write + WriteRead>(i2c: &mut T, delay: &mut Delay, address: u8) -> Result<(), Error>  
    where hal::twim::Error: From<<T as Write>::Error>,
          hal::twim::Error: From<<T as WriteRead>::Error> {
    let mut reg_buffer = [0; 1];
    i2c.write_read(address, &[0x75], &mut reg_buffer)?;
    if reg_buffer[0] != 0x68 { panic!("no/invalid response from MPU6050! {}", reg_buffer[0]); }

    // device reset
    i2c.write(address, &[107, 0b10000000])?;
    delay.delay_ms(100u32);
    // singal path reset - not clear if this is necessary but maybe recommended in register map doc?
    i2c.write(address, &[104, 0b00000111])?;
    delay.delay_ms(100u32);

    i2c.write(address, &[56,  0b00000000])?; // no interrupts
    i2c.write(address, &[35,  0b00000000])?; // Turn off regular FIFO to use DMP FIFO instead
    i2c.write(address, &[28,  0b00000000])?; // 2g full scale accelerometer
    assert!(ACCEL_SCALE == 2.0);
    i2c.write(address, &[55,  0b10010000])?; //logic level for int pin low, and clear int status on any read MAY NOT BE NEEDED?
    i2c.write(address, &[107, 0b00000001])?; // Clock to PLL with X axis gyroscope reference, dont sleep
    i2c.write(address, &[26,  0b00000001])?; // no external synchronization, set the DLPF to 184/188 Hz bandwidth
    i2c.write(address, &[25,  0b00000100])?; // divides the sample rate - yields 200hz I think combined with above?
	
    write_firmware(i2c, address, true)?;
    // set DMP Program Start Address 0x0400
    i2c.write(address, &[dmp_firmware::MPU6050_REGADDR_DMP_PROG_START, 0x04, 0x00])?; 
    
   
    i2c.write(address, &[27,  0b00011000])?; // set gyro to +2000 Deg/sec full range
    i2c.write(address, &[106, 0b11001100])?; // turn on the fifo and reset it, and also turn on and reset the DMP (bits 7,3, respectively)
    i2c.write(address, &[56,  0b00000010])?; // now turn on the RAW_DMP_INT_EN since the DMP is now set up
  
    i2c.write_read(address, &[0x75], &mut reg_buffer)?;
    if reg_buffer[0] != 0x68 { panic!("no/invalid response from MPU6050 after reset/setup! {}", reg_buffer[0]); }
    Ok(())
}

pub(crate) fn write_firmware<T: Write + WriteRead>(i2c: &mut T, address: u8, verify: bool) -> Result<(), Error>  
where hal::twim::Error: From<<T as Write>::Error>,
      hal::twim::Error: From<<T as WriteRead>::Error> {
    let nbanks = dmp_firmware::DMP_FIRMWARE.len() / dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE + 
        if dmp_firmware::DMP_FIRMWARE.len() % dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE == 0 {0} else {1};

    for b in 0..nbanks {
        let bank_size = if b == (nbanks - 1) {
            // last block
            dmp_firmware::DMP_FIRMWARE.len() - (nbanks-1) * dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE
        } else {
            dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE
        };

        // set the bank
        i2c.write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, b as u8])?;

        // now write the bank one chunk at a time
        for i in (0..bank_size).step_by(dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE) {
            let chunk_size = if (i + dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE) > bank_size {
                    bank_size - i
                } else {
                    dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE
                };
            let mut write_buffer = [dmp_firmware::MPU6050_REGADDR_MEM_R_W; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE+1];
            for j in 0..chunk_size {
                write_buffer[j + 1] = dmp_firmware::DMP_FIRMWARE[i + j + b*dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE];
            }
            // set the start address then write
            i2c.write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, i as u8])?;
            i2c.write(address, &write_buffer[0..(chunk_size+1)])?;
        }

    }

    if verify {
        let mut verify_buffer = [0; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE];
        let mut i = 0;
        let mut last_bank = 255;
        while i < dmp_firmware::DMP_FIRMWARE.len() {
            let bank = (i / dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE) as u8;
            if bank != last_bank {
                //set to new bank 
                i2c.write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, bank])?;
            }

            i2c.write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, (i % dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE) as u8])?;
            i2c.write_read(address, &[dmp_firmware::MPU6050_REGADDR_MEM_R_W],  
                                    &mut verify_buffer)?;

            for j in 0..verify_buffer.len() {
                if (i+j) < dmp_firmware::DMP_FIRMWARE.len() {
                    if dmp_firmware::DMP_FIRMWARE[i+j] != verify_buffer[j] {
                        panic!("verification of firmware failed at {}! {} vs {}", i + j, dmp_firmware::DMP_FIRMWARE[i+j], verify_buffer[j]);
                    }
                }
            }
            i += verify_buffer.len();
            last_bank = bank;
        }
    }
    Ok(())
}

pub(crate) fn reset_fifo<T: Write + WriteRead>(i2c: &mut T, address: u8) -> Result<(), Error>  
where hal::twim::Error: From<<T as Write>::Error>,
      hal::twim::Error: From<<T as WriteRead>::Error> {
    let mut reg_buffer = [0; 1];
    // first read the USER_CTRL register
    i2c.write_read(address, &[106], &mut reg_buffer)?;
    i2c.write(address, &[106, reg_buffer[0] | 0b00000100])?;  // write back the same thing but with the fifo reset bit set
    Ok(())
}

pub(crate) fn get_fifo_count<T: WriteRead>(i2c: &mut T, address: u8) -> Result<u16, Error>  
where hal::twim::Error: From<<T as WriteRead>::Error> {
    let mut rd_buffer = [0; 2];
    // first read the USER_CTRL register
    i2c.write_read(address, &[114], &mut rd_buffer)?;
    Ok(((rd_buffer[0] as u16) << 8) | (rd_buffer[1] as u16))
}

// For unclear reasons there are an additional 2 bytes that need reading even though the packet is supposet to be opnly 28 bytes
pub(crate) const FIFO_SIZE: usize = dmp_firmware::DMP_PACKET_SIZE + 2;

pub(crate) fn read_fifo_raw<T: WriteRead>(i2c: &mut T, address: u8) -> Result<[u8; FIFO_SIZE], Error>  
where hal::twim::Error: From<<T as WriteRead>::Error> {
    let mut count = get_fifo_count(i2c, address)?;
    while count < 28 {
        count = get_fifo_count(i2c, address)?;
    }
    let mut fifo = [0; FIFO_SIZE];
    i2c.write_read(address, &[116], &mut fifo)?;
    Ok(fifo)
}


pub(crate) fn read_fifo<T: Write + WriteRead>(i2c: &mut T, address: u8) -> Result<MAPPData, Error>  
where hal::twim::Error: From<<T as WriteRead>::Error> {
    let fifo_raw = read_fifo_raw(i2c, address)?;

    let qw = ((fifo_raw[0] as i16) << 8) | fifo_raw[1] as i16;
    let qx = ((fifo_raw[4] as i16) << 8) | fifo_raw[5] as i16;
    let qy = ((fifo_raw[8] as i16) << 8) | fifo_raw[9] as i16;
    let qz = ((fifo_raw[12] as i16) << 8) | fifo_raw[13] as i16;

    let gx = ((fifo_raw[16] as i16) << 8) | fifo_raw[17] as i16;
    let gy = ((fifo_raw[18] as i16) << 8) | fifo_raw[19] as i16;
    let gz = ((fifo_raw[20] as i16) << 8) | fifo_raw[21] as i16;

    let ax = ((fifo_raw[22] as i16) << 8) | fifo_raw[23] as i16;
    let ay = ((fifo_raw[24] as i16) << 8) | fifo_raw[25] as i16;
    let az = ((fifo_raw[26] as i16) << 8) | fifo_raw[27] as i16;

    Ok(MAPPData {
        qw: qw,
        qx: qx,
        qy: qy,
        qz: qz,
        gyro_x: gx,
        gyro_y: gy,
        gyro_z: gz,
        accel_x: ax,
        accel_y: ay,
        accel_z: az,
    })
}

pub(crate) struct MAPPData {
    pub qw: i16,
    pub qx: i16,
    pub qy: i16,
    pub qz: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
}

impl MAPPData {
    pub(crate) fn to_byte_array(&self, out: &mut [u8], offset: usize){
        let mut i = offset;
        for elem in [self.qw, self.qx, self.qy, self.qz,
                     self.gyro_x, self.gyro_y, self.gyro_z,
                     self.accel_x, self.accel_y, self.accel_z] {
            let elemb = elem.to_le_bytes();
            out[i] = elemb[0];
            out[i+1] = elemb[1];
            i+= 2;
        }
    }

    pub(crate) fn to_bytes(&self) -> [u8; MPU6050_DATA_SIZE] {
        let mut buf = [0u8; MPU6050_DATA_SIZE];
        self.to_byte_array(&mut buf, 0);
        buf
    }

    pub(crate) fn q_to_float(&self) -> (f32, f32, f32, f32) {
        let wf = (self.qw as f32) / (i16::MAX as f32);
        let xf = (self.qx as f32) / (i16::MAX as f32);
        let yf = (self.qy as f32) / (i16::MAX as f32);
        let zf = (self.qz as f32) / (i16::MAX as f32);
        (wf, xf, yf, zf)
    }

    pub(crate) fn accel_to_float(&self) -> (f32, f32, f32) {
        let xf = (self.accel_x as f32) / (i16::MAX as f32);
        let yf = (self.accel_y as f32) / (i16::MAX as f32);
        let zf = (self.accel_z as f32) / (i16::MAX as f32);
        (xf, yf, zf)
    }

    // Where the quaternion maps the x-axis to
    pub(crate) fn to_alignment_vector_x(&self) -> (f32, f32, f32){
        let (qw, qx, qy, qz)  = self.q_to_float();
        let x = qw*qw - qx*qx - qy*qy - qz*qz;
        let y = 2.*(qw*qz + qx*qy);
        let z = 2.*(qx*qz - qw*qy);
        let n = f32::sqrt(x*x + y*y + z*z);
        (x/n, y/n, z/n)
    }

    // Where the quaternion maps the y-axis to
    pub(crate) fn to_alignment_vector_y(&self) -> (f32, f32, f32){
        let (qw, qx, qy, qz)  = self.q_to_float();
        let x = 2.*(qx*qy - qw*qz);
        let y =  qw*qw - qx*qx + qy*qy - qz*qz;
        let z = 2.*(qy*qz + qw*qx);
        let n = f32::sqrt(x*x + y*y + z*z);
        (x/n, y/n, z/n)
    }

    // Where the quaternion maps the z-axis to
    pub(crate) fn to_alignment_vector_z(&self) -> (f32, f32, f32){
        let (qw, qx, qy, qz)  = self.q_to_float();
        let x = 2.*(qw*qy + qx*qz);
        let y = 2.*(qy*qz - qw*qx);
        let z = qw*qw - qx*qx - qy*qy + qz*qz;
        let n = f32::sqrt(x*x + y*y + z*z);
        (x/n, y/n, z/n)
    }

    // sign might be wrong in the quaternion application!
    pub(crate) fn to_z_accel(&self) -> f32{
        let (qw, qx, qy, qz)  = self.q_to_float();
        let (ax, ay, az) = self.accel_to_float();
        let normed = qw*(-ax*qy + ay*qx + az*qw) + qx*(ax*qz + ay*qw - az*qx) - qy*(ax*qw - ay*qz + az*qy) + qz*(ax*qx + ay*qy + az*qz);
        normed * ACCEL_SCALE
    }

    pub(crate) fn to_xy_rotdeg(&self) -> f32 {
        let (qw, qx, qy, qz)  = self.q_to_float();
        let numerator = 2. * (-qw*qx + qy*qz);
        let denominator = 2. * (-qw*qy + qx*qz);
        f32::to_degrees(f32::atan2(numerator, denominator))

    }

    pub(crate) fn to_float(&self) -> QADataFloat {
        let (qw, qx, qy, qz)  = self.q_to_float();
        let ax = (self.accel_x as f32) / (i16::MAX as f32);
        let ay = (self.accel_y as f32) / (i16::MAX as f32);
        let az = (self.accel_z as f32) / (i16::MAX as f32);
        QADataFloat {
            qw: qw,
            qx: qx,
            qy: qy,
            qz: qz,
            accel_x: ax,
            accel_y: ay,
            accel_z: az,
        }
    }
}
pub(crate) struct QADataFloat {
    pub qw: f32,
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
}

pub(crate) const MPU6050_DATA_SIZE: usize = core::mem::size_of::<MAPPData>();