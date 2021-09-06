use adsb_deku::{cpr, Altitude, CPRFormat, Frame, DF, ICAO, ME};
use std::collections::HashMap;
use std::fmt;
use std::time::SystemTime;

#[derive(Debug)]
pub struct AirplaneCoor {
    /// [odd, even]
    pub altitudes: [Option<Altitude>; 2],
    /// last time of frame rx
    pub last_time: SystemTime,
}

impl Default for AirplaneCoor {
    fn default() -> Self {
        Self {
            altitudes: [None, None],
            last_time: SystemTime::now(),
        }
    }
}

#[derive(Debug)]
pub struct Airplanes(pub HashMap<ICAO, AirplaneCoor>);

impl fmt::Display for Airplanes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (key, _) in &self.0 {
            let value = self.lat_long_altitude(*key);
            if let Some(value) = value {
                writeln!(f, "{}: {:?}", key, value);
            }
        }
        Ok(())
    }
}

impl Airplanes {
    pub fn new() -> Self {
        Self(HashMap::new())
    }

    /// Add `Altitude` from adsb frame
    pub fn add_extended_quitter_ap(&mut self, icao: ICAO, frame: Frame) {
        let airplane_coor = self.0.entry(icao).or_insert(AirplaneCoor::default());
        match frame.df {
            DF::ADSB(adsb) => match adsb.me {
                ME::AirbornePositionBaroAltitude(altitude) => match altitude.odd_flag {
                    CPRFormat::Odd => {
                        *airplane_coor = AirplaneCoor {
                            altitudes: [airplane_coor.altitudes[0].clone(), Some(altitude)],
                            last_time: SystemTime::now(),
                        };
                    }
                    CPRFormat::Even => {
                        *airplane_coor = AirplaneCoor {
                            altitudes: [Some(altitude), airplane_coor.altitudes[1].clone()],
                            last_time: SystemTime::now(),
                        };
                    }
                },
                _ => (),
            },
            _ => (),
        }
    }

    /// Calculate latitude, longitude and altitude
    pub fn lat_long_altitude(&self, icao: ICAO) -> Option<(cpr::Position, u32)> {
        match self.0.get(&icao) {
            Some(altitudes) => {
                if let (Some(first_altitude), Some(second_altitude)) =
                    (altitudes.altitudes[0], altitudes.altitudes[1])
                {
                    cpr::get_position((&first_altitude, &second_altitude))
                        .map(|position| (position, first_altitude.alt))
                } else {
                    None
                }
            }
            None => None,
        }
    }

    /// Remove airplain after not active for a time
    pub fn prune(&mut self) {
        self.0
            .retain(|_, v| v.last_time.elapsed().unwrap() < std::time::Duration::from_secs(60));
    }
}
