#![doc = include_str!("../README.md")]
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]
#![doc(html_logo_url = "https://raw.githubusercontent.com/rsadsb/adsb_deku/master/media/logo.png")]

extern crate alloc;

#[cfg(feature = "alloc")]
use alloc::{collections::BTreeMap, fmt, string::String, vec, vec::Vec};
#[cfg(feature = "alloc")]
use core::{
    clone::Clone, default::Default, fmt::Debug, marker::Copy, prelude::rust_2021::derive,
    result::Result::Ok, writeln,
};
#[cfg(feature = "std")]
use std::time::SystemTime;

use adsb_deku::adsb::{
    AirborneVelocity, Identification, StatusForGroundTrack, SurfacePosition, ME,
};
use adsb_deku::{cpr, Altitude, CPRFormat, Frame, DF, ICAO};
use tracing::{debug, info, warn};

// Max absurd distance an aircraft travelled between messages
const MAX_AIRCRAFT_DISTANCE: f64 = 100.0;

#[derive(Debug, PartialEq, Eq)]
pub enum Added {
    /// Airplane was not added
    No,
    /// Airplane was added
    Yes,
}

impl From<bool> for Added {
    fn from(other: bool) -> Self {
        match other {
            true => Self::Yes,
            false => Self::No,
        }
    }
}

/// `BTreeMap` of of all currently tracked `ICAO` and `AirplaneState`.
///
/// Currently tracked means that within calling [`Self::action`], an aircraft is added to this data
/// structure.
#[cfg_attr(feature = "serde", serde_with::serde_as)]
#[derive(Debug, Default, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Airplanes(
    #[cfg_attr(
        feature = "serde",
        serde(with = "serde_with::As::<Vec<(serde_with::DisplayFromStr, serde_with::Same)>>")
    )]
    BTreeMap<ICAO, AirplaneState>,
);

impl fmt::Display for Airplanes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for key in self.0.keys() {
            let value = self.aircraft_details(*key);
            if let Some(value) = value {
                writeln!(f, "{key}: {value:?}")?;
            }
        }
        Ok(())
    }
}

// public
impl Airplanes {
    #[must_use]
    pub fn new() -> Self {
        Self(BTreeMap::new())
    }

    /// Tuple `iter()` of all `(ICAO, AirplanesState)`
    ///
    /// equivalent [`BTreeMap::iter`]
    pub fn iter(&self) -> alloc::collections::btree_map::Iter<'_, ICAO, AirplaneState> {
        self.0.iter()
    }

    /// Get all `ICAO` keys
    ///
    /// equivalent [`BTreeMap::keys`]
    pub fn keys(&self) -> alloc::collections::btree_map::Keys<'_, ICAO, AirplaneState> {
        self.0.keys()
    }

    /// From `ICAO`, get `AirplaneState`
    ///
    /// equivalent [`BTreeMap::get`]
    #[must_use]
    pub fn get(&self, key: ICAO) -> Option<&AirplaneState> {
        self.0.get(&key)
    }

    /// Amount of currently tracked airplanes
    ///
    /// equivalent [`BTreeMap::len`]
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// equivalent [`BTreeMap::is_empty`]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Update `Airplanes` with new `Frame`
    ///
    /// Take parsed `Frame` and read the `DF::ADSB` type and act upon the parsed message. This
    /// updates the field that the `ME` value equates to within [`Self`]. This also adds
    /// airplanes (`ICAO` and `AirplaneState`) when a new aircraft is detected.
    ///
    /// `lat_long`: (latitude, longitude) of current receiver location
    ///
    /// `max_range`: max range of the receiver
    ///
    /// Return true if entry was added into `Airplanes`
    pub fn action(&mut self, frame: Frame, lat_long: (f64, f64), max_rang: f64) -> Added {
        let mut airplane_added = Added::No;
        match frame.df {
            DF::ADSB(ref adsb) => {
                airplane_added = match &adsb.me {
                    ME::AircraftIdentification(identification) => {
                        self.add_identification(adsb.icao, identification)
                    }
                    ME::AirborneVelocity(vel) => self.add_airborne_velocity(adsb.icao, vel),
                    ME::AirbornePositionGNSSAltitude(altitude)
                    | ME::AirbornePositionBaroAltitude(altitude) => {
                        self.update_position(adsb.icao, altitude, lat_long, max_rang)
                    }
                    ME::SurfacePosition(surface_position) => {
                        self.update_surface_position(adsb.icao, surface_position, lat_long)
                    }
                    _ => Added::No,
                };
                let incr_airplane_added = self.incr_messages(adsb.icao);
                airplane_added =
                    if incr_airplane_added == Added::Yes || airplane_added == Added::Yes {
                        Added::Yes
                    } else {
                        Added::No
                    };
            }
            DF::TisB { cf, pi } => {
                info!("TISB: {cf:?}, {pi:?}");
                airplane_added = match cf.me {
                    ME::AircraftIdentification(identification) => {
                        self.add_identification(pi, &identification)
                    }
                    ME::AirborneVelocity(vel) => self.add_airborne_velocity(pi, &vel),
                    ME::AirbornePositionGNSSAltitude(altitude)
                    | ME::AirbornePositionBaroAltitude(altitude) => {
                        self.update_position(pi, &altitude, lat_long, max_rang)
                    }
                    _ => Added::No,
                };
                let incr_airplane_added = self.incr_messages(pi);
                airplane_added =
                    if incr_airplane_added == Added::Yes || airplane_added == Added::Yes {
                        Added::Yes
                    } else {
                        Added::No
                    };
            }
            _ => (),
        }

        airplane_added
    }

    /// from `ICAO` return details on that airplane
    ///
    /// position, altitude, and `kilo_distance` are required to be set to Some(value) in order for
    /// this function to return any values from that `ICAO`. Other values from that `ICAO` are
    /// optional and can be None. See [`AirplaneDetails`] for all the values this function returns.
    #[must_use]
    pub fn aircraft_details(&self, icao: ICAO) -> Option<AirplaneDetails> {
        match self.get(icao) {
            Some(airplane_state) => {
                let track = &airplane_state.track;
                let coor = &airplane_state.coords;
                if let (Some(position), Some(altitude), Some(kilo_distance)) =
                    (&coor.position, coor.altitude(), coor.kilo_distance)
                {
                    Some(AirplaneDetails {
                        position: *position,
                        altitude,
                        kilo_distance,
                        heading: airplane_state.heading,
                        track: track.clone(),
                    })
                } else {
                    None
                }
            }
            None => None,
        }
    }

    /// Return all aircraft that currently have a [`cpr::Position`]
    #[must_use]
    pub fn all_position(&self) -> Vec<(ICAO, cpr::Position)> {
        let mut all_lat_long = vec![];
        for (key, airplane_state) in self.iter() {
            let coor = &airplane_state.coords;
            if let Some(position) = &coor.position {
                all_lat_long.push((*key, *position));
            }
        }

        all_lat_long
    }

    /// Remove airplanes that have not been seen since `filter_time` seconds
    #[cfg(feature = "std")]
    pub fn prune(&mut self, filter_time: u64) {
        self.0.retain(|k, v| {
            if let Ok(time) = v.last_time.elapsed() {
                if time < std::time::Duration::from_secs(filter_time) {
                    true
                } else {
                    info!("[{k}] non-active, removing");
                    false
                }
            } else {
                info!("[{k}] non-active(time error), removing");
                false
            }
        });
    }
}

// private
impl Airplanes {
    // Return (matching state from icao, true if airplane added)
    fn entry_or_insert(&mut self, icao: ICAO) -> (&mut AirplaneState, Added) {
        let entry = self.0.entry(icao);
        let airplane_added =
            Added::from(matches!(entry, alloc::collections::btree_map::Entry::Vacant(_)));
        if Added::Yes == airplane_added {
            info!("[{icao}] now tracking");
        }
        (entry.or_default(), airplane_added)
    }

    /// Increment message count of `ICAO`. If feature: `std`, set `last_time` to current time.
    ///
    /// Return true if entry was added into `Airplanes`
    pub fn incr_messages(&mut self, icao: ICAO) -> Added {
        let (state, airplane_added) = self.entry_or_insert(icao);
        state.num_messages += 1;
        #[cfg(feature = "std")]
        {
            state.last_time = std::time::SystemTime::now();
        }

        airplane_added
    }

    /// update from `ME::AircraftIdentification`
    ///
    /// Return true if entry was added into `Airplanes`
    fn add_identification(&mut self, icao: ICAO, identification: &Identification) -> Added {
        let (state, airplane_added) = self.entry_or_insert(icao);
        state.callsign = Some(identification.cn.clone());
        info!("[{icao}] with identification: {}", identification.cn);

        airplane_added
    }

    /// update from `ME::AirborneVelocity`
    ///
    /// Return true if entry was added into `Airplanes`
    fn add_airborne_velocity(&mut self, icao: ICAO, vel: &AirborneVelocity) -> Added {
        let (state, airplane_added) = self.entry_or_insert(icao);
        if let Some((heading, ground_speed, vert_speed)) = vel.calculate() {
            info!("[{icao}] with airborne velocity: heading: {heading}, speed: {ground_speed}, vertical speed: {vert_speed}");
            state.heading = Some(heading);
            state.speed = Some(ground_speed as f32);
            state.vert_speed = Some(vert_speed);
        }

        airplane_added
    }

    /// update from `ME::AirbornePosition{GNSSAltitude, BaroAltitude}`
    ///
    /// Return true if entry was added into `Airplanes`
    fn update_position(
        &mut self,
        icao: ICAO,
        altitude: &Altitude,
        lat_long: (f64, f64),
        max_range: f64,
    ) -> Added {
        let (state, airplane_added) = self.entry_or_insert(icao);
        info!(
            "[{icao}] with: {:?}, cpr lat: {}, cpr long: {}",
            altitude.alt, altitude.lat_cpr, altitude.lon_cpr
        );
        let mut temp_coords = match altitude.odd_flag {
            CPRFormat::Odd => AirplaneCoor {
                altitudes: [state.coords.altitudes[0], Some(*altitude)],
                ..state.coords
            },
            CPRFormat::Even => AirplaneCoor {
                altitudes: [Some(*altitude), state.coords.altitudes[1]],
                ..state.coords
            },
        };
        // update the position from the new even/odd message if it's a good new position
        if temp_coords.update_position(lat_long, max_range) {
            // don't bother updating if it's the same coords
            if state.coords != temp_coords {
                // update track
                if let Some(track) = &mut state.track {
                    track.push(state.coords);
                } else {
                    state.track = Some(vec![state.coords]);
                }
                // update new position
                state.coords = temp_coords;
            }
        } else {
            // clear record
            state.coords = AirplaneCoor::default();
        }

        airplane_added
    }

    /// update from `ME::SurfacePosition`
    ///
    /// Return true if entry was added into `Airplanes`
    ///
    fn update_surface_position(
        &mut self,
        icao: ICAO,
        surface_position: &SurfacePosition,
        lat_long: (f64, f64),
    ) -> Added {
        let (state, airplane_added) = self.entry_or_insert(icao);
        state.coords.position = cpr::surface_position_with_reference(surface_position, lat_long);
        let speed = get_surface_speed(surface_position);
        state.speed = speed;
        state.heading = if surface_position.s == StatusForGroundTrack::Valid {
            Some(surface_position.trk as f32 * 360.0 / 128.0)
        } else {
            None
        };
        airplane_added
    }
}

fn get_surface_speed(surface_position: &SurfacePosition) -> Option<f32> {
    let speed = match surface_position.mov {
        0 => None,
        1 => Some(0.0),
        2..=8 => Some(0.125 + (surface_position.mov + 1 - 2) as f32 * 0.125),
        9..=12 => Some(1.0 + (surface_position.mov as f32 + 1.0 - 9.0) * 0.25),
        13..=38 => Some(2.0 + (surface_position.mov as f32 + 1.0 - 13.0) * 0.5),
        39..=93 => Some(15.0 + (surface_position.mov as f32 + 1.0 - 39.0) * 1.0),
        94..=108 => Some(70.0 + (surface_position.mov as f32 + 1.0 - 94.0) * 2.0),
        109..=123 => Some(100.0 + (surface_position.mov as f32 + 1.0 - 109.0) * 5.0),
        124 => Some(175.0),
        _ => Some(200.0),
    };
    speed
}

/// Generated by `Airplanes::aircraft_details()`
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AirplaneDetails {
    pub position: cpr::Position,
    pub altitude: u16,
    pub kilo_distance: f64,
    pub heading: Option<f32>,
    pub track: Option<Vec<AirplaneCoor>>,
}

/// Value in `BTreeMap` of `Airplanes`
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AirplaneState {
    // TODO: rename to coor
    pub coords: AirplaneCoor,
    pub squawk: Option<u32>,
    pub callsign: Option<String>,
    /// heading from `adsb::AirborneVelocity::calculate()`
    ///
    /// 0 = Straight up
    /// 90 = Right, and so on
    pub heading: Option<f32>,
    /// ground_speed from `adsb::AirborneVelocity::calculate()`
    ///
    /// Stored as a f64 in that library but we store as f32 for size reasons in this library
    pub speed: Option<f32>,
    /// vert_speed from `adsb::AirborneVelocity::calculate()`
    pub vert_speed: Option<i16>,
    pub on_ground: Option<bool>,
    pub num_messages: u32,
    #[cfg(feature = "std")]
    pub last_time: SystemTime,
    pub track: Option<Vec<AirplaneCoor>>,
}

impl Default for AirplaneState {
    fn default() -> Self {
        Self {
            coords: AirplaneCoor::default(),
            squawk: None,
            callsign: None,
            heading: None,
            speed: None,
            vert_speed: None,
            on_ground: None,
            num_messages: 0,
            #[cfg(feature = "std")]
            last_time: SystemTime::now(),
            track: None,
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AirplaneCoor {
    /// [odd, even]
    pub altitudes: [Option<Altitude>; 2],
    /// lat/long
    pub position: Option<cpr::Position>,
    /// last good time
    #[cfg(feature = "std")]
    pub last_time: Option<SystemTime>,
    /// distance from receiver lat/long
    pub kilo_distance: Option<f64>,
}

impl AirplaneCoor {
    /// After checking the range of the new lat / long, new position from last position, update the
    /// position of an aircraft
    fn update_position(&mut self, lat_long: (f64, f64), max_range: f64) -> bool {
        if let [Some(odd), Some(even)] = self.altitudes {
            let test_position = cpr::get_position((&odd, &even));

            // Check kilometer range from receiver
            if let Some(test_position) = test_position {
                let kilo_distance = Self::haversine_distance(
                    lat_long,
                    (test_position.latitude, test_position.longitude),
                );
                if kilo_distance > max_range {
                    warn!("range: {kilo_distance} -  old: {lat_long:?} new: {test_position:?}");
                    return false;
                }
                self.kilo_distance = Some(kilo_distance);
                debug!("range: {kilo_distance}");
            }

            // if previous position, check against for range. This is a non-great way of doing
            // this, but maybe in the future we can check against the speed of the aircraft
            if let (Some(current_position), Some(test_position)) = (self.position, test_position) {
                let distance = Self::haversine_distance_position(current_position, test_position);
                if distance > MAX_AIRCRAFT_DISTANCE {
                    warn!("distance: {distance} old: {current_position:?}, invalid: {test_position:?}");
                    return false;
                }
                debug!("distance: {distance}");
            }

            // Good new position!
            self.position = test_position;
            debug!("update_position: odd: (lat: {}, long: {}), even: (lat: {}, long: {}), position: {:?}",
                odd.lat_cpr,
                odd.lon_cpr,
                even.lat_cpr,
                even.lat_cpr,
                self.position);
            #[cfg(feature = "std")]
            {
                self.last_time = Some(SystemTime::now());
            }
        }
        true
    }

    /// Return altitude from Odd Altitude
    fn altitude(&self) -> Option<u16> {
        if let Some(odd) = self.altitudes[0] {
            if let Some(alt) = odd.alt {
                return Some(alt);
            }
        }
        None
    }

    /// Calculate the kilometers between two lat/long points
    fn haversine_distance_position(position: cpr::Position, other: cpr::Position) -> f64 {
        let lat1 = position.latitude;
        let lat2 = other.latitude;
        let long1 = position.longitude;
        let long2 = other.longitude;
        Self::haversine_distance((lat1, long1), (lat2, long2))
    }

    // https://en.wikipedia.org/wiki/Haversine_formula
    fn haversine_distance(s: (f64, f64), other: (f64, f64)) -> f64 {
        // kilometers
        let lat1_rad = s.0.to_radians();
        let lat2_rad = other.0.to_radians();
        let long1_rad = s.1.to_radians();
        let long2_rad = other.1.to_radians();

        let x_lat = libm::sin((lat2_rad - lat1_rad) / 2.00);
        let x_long = libm::sin((long2_rad - long1_rad) / 2.00);

        // this clippy lint will dis-allow mul_add, this isn't available for `no_std`
        #[allow(clippy::suboptimal_flops)]
        let a = x_lat * x_lat
            + libm::cos(lat1_rad)
                * libm::cos(lat2_rad)
                * f64::from(libm::powf(libm::sin(x_long) as f32, 2.0));

        let c = 2.0 * libm::atan2(libm::sqrt(a), libm::sqrt(1.0 - a));

        let r = 6371.00;
        r * c
    }
}
