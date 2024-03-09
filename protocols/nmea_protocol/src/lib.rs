#![no_std]

use defmt::Format;
use nom::branch::alt;
use nom::bytes::complete::tag;
use nom::character::complete::alpha1;
use nom::number::complete::float;
use nom::sequence::tuple;
use nom::IResult;

use serde::Serialize;

#[derive(Format, Debug, Default, PartialEq, Serialize)]
pub enum Direction {
    #[default]
    North,
    South,
    East,
    West,
}

#[derive(Format, Serialize)]
pub enum DataType {
    GLL,
    GGA,
    Invalid,
}

#[derive(Serialize, Default)]
pub struct GpsData {
    pub latitude: f32,
    pub lat_dir: Direction,
    pub longitude: f32,
    pub lon_dir: Direction,
    pub time: f32,
}

impl Format for GpsData {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "GpsData: latitude: {} {}, longitude: {} {}, time: {}",
            self.latitude,
            self.lat_dir,
            self.longitude,
            self.lon_dir,
            self.time,
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
        // get time
        let (input, (time, _)) = tuple((float, tag(",")))(input)?;
        // get location
        let (input, (latitude, lat_dir)) = Self::parse_nmea_coord(input)?;
        let (input, (longitude, lon_dir)) = Self::parse_nmea_coord(input)?;

        Ok((
            input,
            GpsData {
                latitude,
                lat_dir,
                longitude,
                lon_dir,
                time,
            },
        ))
    }

    fn parse_nmea_gll(input: &[u8]) -> IResult<&[u8], GpsData> {
        // get location
        let (input, (latitude, lat_dir)) = Self::parse_nmea_coord(input)?;
        let (input, (longitude, lon_dir)) = Self::parse_nmea_coord(input)?;
        // get time
        let (input, (time, _)) = tuple((float, tag(",")))(input)?;

        Ok((
            input,
            GpsData {
                latitude,
                lat_dir,
                longitude,
                lon_dir,
                time,
            },
        ))
    }

    pub fn parse_nmea(input: &[u8]) -> Option<GpsData> {
        if let Ok((input, _)) = Self::parse_satellite_type(input) {
            if let Ok((input, data_type)) = Self::parse_nmea_data_type(input) {
                let result = match data_type {
                    DataType::GGA => Self::parse_nmea_gga(input).ok(),
                    DataType::GLL => Self::parse_nmea_gll(input).ok(),
                    DataType::Invalid => {
                        return None;
                    }
                };

                if let Some((_, gps_data)) = result {
                    return Some(gps_data);
                }
            }
        }

        None
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
