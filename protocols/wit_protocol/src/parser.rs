use core::{convert::TryInto, mem::size_of};

use nom::bytes::complete::take;
pub use nom::IResult;

pub fn take_i16_le(s: &[u8]) -> IResult<&[u8], i16> {
    let (s, u) = take(size_of::<i16>())(s)?;
    let res = i16::from_le_bytes(u.try_into().unwrap());
    Ok((s, res))
}

pub fn take_i32_le(s: &[u8]) -> IResult<&[u8], i32> {
    let (s, u) = take(size_of::<i32>())(s)?;
    let res = i32::from_le_bytes(u.try_into().unwrap());
    Ok((s, res))
}
