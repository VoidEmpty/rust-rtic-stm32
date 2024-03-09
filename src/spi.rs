use alloc::vec;
use alloc::vec::Vec;
use stm32f4xx_hal::{pac::SPI3, spi::*};

pub fn spi_send(spi: &mut Spi<SPI3>, mut address: u8, value: u32) -> Result<(), Error> {
    // set write bit
    address |= 0x80;
    let mut buffer = [0; 5];
    let mut package: Vec<u8> = vec![address];
    package.append(&mut value.to_le_bytes().to_vec());
    spi.transfer(&mut buffer, package.as_mut_slice())?;
    return Ok(());
}

pub fn spi_read(spi: &mut Spi<SPI3>, address: u8) -> Result<u32, Error> {
    let val;
    let mut buffer = [1; 5];
    let mut package = vec![address, 0x10, 0x20, 0x30, 0x40];
    spi.transfer(&mut buffer, package.as_mut_slice())?;
    let mut res = [0; 4];
    res.copy_from_slice(&buffer[1..]);
    val = u32::from_le_bytes(res);
    return Ok(val);
}
