#![warn(clippy::std_instead_of_alloc, clippy::std_instead_of_core)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use defmt::Format;
use nom::branch::alt;

// extern crate lexical_core;
// extern crate nom;

use nom::bytes::complete::tag;
use nom::character::complete::alpha1;
use nom::number::complete::float;
use nom::sequence::tuple;
use nom::IResult;

#[derive(Format)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

#[derive(Format)]
pub enum DataType {
    GLL,
    GGA,
    Invalid,
}

pub struct GpsData {
    data_type: DataType,
    pub latitude: f32,
    pub lat_dir: Direction,
    pub longitude: f32,
    pub lon_dir: Direction,
}

impl Format for GpsData {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "GpsData: data_type: {}, latitude: {} {}, longitude: {} {}",
            self.data_type,
            self.latitude,
            self.lat_dir,
            self.longitude,
            self.lon_dir,
        );
    }
}

pub struct Nmea;

impl Nmea {
    fn parse_satellite_type(input: &[u8]) -> IResult<&[u8], &[u8]> {
        alt((
            tag("$GP"),
            tag("$GL"),
            tag("$GA"),
            tag("$BD"),
            tag("$GQ"),
            tag("$GN"),
        ))(input)
    }

    fn parse_nmea_data_type(input: &[u8]) -> IResult<&[u8], DataType> {
        let data_type;
        let (input, dt) = alt((tag("GGA"), tag("GLL")))(input)?;
        let (input, _) = tag(",")(input)?;
        if dt == "GGA".as_bytes() {
            data_type = DataType::GGA;
        } else if dt == "GLL".as_bytes() {
            data_type = DataType::GLL;
        } else {
            data_type = DataType::Invalid;
        }
        Ok((input, data_type))
    }

    fn parse_nmea_coord(input: &[u8]) -> IResult<&[u8], (f32, Direction)> {
        let mut direction = Direction::North;
        let (input, (coord, _, dir, _)) = tuple((float, tag(","), alpha1, tag(",")))(input)?;
        if dir == "S".as_bytes() {
            direction = Direction::South;
        } else if dir == "N".as_bytes() {
            direction = Direction::North;
        } else if dir == "E".as_bytes() {
            direction = Direction::East;
        } else if dir == "W".as_bytes() {
            direction = Direction::West;
        }
        Ok((input, (coord, direction)))
    }

    fn parse_nmea_gga(input: &[u8]) -> IResult<&[u8], GpsData> {
        let data_type = DataType::GGA;
        // get time
        let (input, (_time, _)) = tuple((float, tag(",")))(input)?;
        // get location
        let (input, (latitude, lat_dir)) = Self::parse_nmea_coord(input)?;
        let (input, (longitude, lon_dir)) = Self::parse_nmea_coord(input)?;

        Ok((
            input,
            GpsData {
                data_type,
                latitude,
                lat_dir,
                longitude,
                lon_dir,
            },
        ))
    }

    fn parse_nmea_gll(input: &[u8]) -> IResult<&[u8], GpsData> {
        let data_type = DataType::GLL;
        // get location
        let (input, (latitude, lat_dir)) = Self::parse_nmea_coord(input)?;
        let (input, (longitude, lon_dir)) = Self::parse_nmea_coord(input)?;
        // get time
        let (input, (_time, _)) = tuple((float, tag(",")))(input)?;

        Ok((
            input,
            GpsData {
                data_type,
                latitude,
                lat_dir,
                longitude,
                lon_dir,
            },
        ))
    }

    pub fn parse_nmea(input: &[u8]) -> Option<GpsData> {
        //? don't need this info
        let (input, _) = Self::parse_satellite_type(input).unwrap();

        let (input, data_type) = Self::parse_nmea_data_type(input).unwrap();

        let (_, data) = match data_type {
            DataType::GGA => Self::parse_nmea_gga(input).unwrap(),
            DataType::GLL => Self::parse_nmea_gll(input).unwrap(),
            DataType::Invalid => {
                return None;
            }
        };

        Some(data)
    }
}

// GGA - Данные о последнем зафиксированном местоположении.
// GLL - Географические координаты.
// GSA - Информация об активных спутниках (участвующих в позиционировании).
// GSV - Информация о всех наблюдаемых спутниках.
// RMC - Рекомендуемый минимум навигационных данных.
// VTG - Скорость и курс относительно земли.
// ZDA - Дата и время.
// DHV - Информация о скорости движения GNSS приемника.
// GST - Статистика ошибок позиционирования.
// TXT - Текстовое сообщение. GGA
