use alloc::vec;
use alloc::vec::Vec;
use stm32f4xx_hal::{
    gpio::{Output, Pin},
    pac::SPI3,
    spi::*,
};

pub fn spi_transfer(
    spi: &mut Spi<SPI3>,
    mut address: u8,
    value: u32,
    cs: &mut Pin<'D', 0, Output>,
) -> Result<u32, Error> {
    // set write bit
    address |= 0x80;
    let mut buffer = [0; 5];
    let mut package: Vec<u8> = vec![address];
    package.append(&mut value.to_le_bytes().to_vec());
    cs.set_low();
    spi.transfer(&mut buffer, package.as_mut_slice())?;
    cs.set_high();
    // got value
    let mut res = [0; 4];
    res.copy_from_slice(&buffer[1..]);
    let val = u32::from_le_bytes(res);
    Ok(val)
}