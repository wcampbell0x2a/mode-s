//! All data structures needed for parsing [`DF::ADSB`] or [`DF::TisB`] messages
//!
//! [`DF::ADSB`]: crate::DF::ADSB
//! [`DF::TisB`]: crate::DF::TisB

#[cfg(feature = "alloc")]
use alloc::{fmt, format, string::String};
#[cfg(feature = "alloc")]
use core::{
    clone::Clone,
    cmp::PartialEq,
    convert::From,
    f64,
    fmt::Write,
    fmt::{Debug, Error},
    marker::Copy,
    option::{Option::None, Option::Some},
    prelude::rust_2021::derive,
    result,
    result::Result::Ok,
    stringify, write, writeln,
};
#[cfg(not(feature = "alloc"))]
use std::{fmt, i64};

use deku::no_std_io::{Read, Seek};
use deku::prelude::*;

use crate::mode_ac::decode_id13_field;
use crate::{aircraft_identification_read, Altitude, CPRFormat, Capability, Sign, ICAO};

/// [`crate::DF::ADSB`] || [`crate::DF::TisB`]
#[derive(Debug, PartialEq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ADSB {
    /// Transponder Capability
    pub capability: Capability,
    /// ICAO aircraft address
    pub icao: ICAO,
    /// Message, extended Squitter
    pub me: ME,
    /// Parity/Interrogator ID
    pub pi: ICAO,
}

impl ADSB {
    /// `to_string` with DF.id() input
    pub(crate) fn to_string(&self, address_type: &str) -> result::Result<String, Error> {
        let mut f = String::new();
        write!(f, "{}", self.me.to_string(self.icao, address_type, self.capability, true)?)?;
        Ok(f)
    }
}

/// ADS-B Message, 5 first bits are known as Type Code (TC)
///
/// reference: ICAO 9871 (A.2.3.1)
#[derive(Debug, PartialEq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "5")]
pub enum ME {
    #[deku(id_pat = "9..=18")]
    AirbornePositionBaroAltitude(Altitude),

    #[deku(id = "19")]
    AirborneVelocity(AirborneVelocity),

    #[deku(id = "0")]
    NoPosition([u8; 6]),

    #[deku(id_pat = "1..=4")]
    AircraftIdentification(Identification),

    #[deku(id_pat = "5..=8")]
    SurfacePosition(SurfacePosition),

    #[deku(id_pat = "20..=22")]
    AirbornePositionGNSSAltitude(Altitude),

    #[deku(id = "23")]
    Reserved0([u8; 6]),

    #[deku(id_pat = "24")]
    SurfaceSystemStatus([u8; 6]),

    #[deku(id_pat = "25..=27")]
    Reserved1([u8; 6]),

    #[deku(id = "28")]
    AircraftStatus(AircraftStatus),

    #[deku(id = "29")]
    TargetStateAndStatusInformation(TargetStateAndStatusInformation),

    #[deku(id = "30")]
    AircraftOperationalCoordination([u8; 6]),

    #[deku(id = "31")]
    AircraftOperationStatus(OperationStatus),
}

impl ME {
    /// `to_string` with DF.id() input
    pub(crate) fn to_string(
        &self,
        icao: ICAO,
        address_type: &str,
        capability: Capability,
        is_transponder: bool,
    ) -> result::Result<String, Error> {
        let transponder = match is_transponder {
            true => " ",
            false => " (Non-Transponder) ",
        };
        let mut f = String::new();
        match self {
            ME::NoPosition(_) => {
                writeln!(f, " Extended Squitter{transponder}No position information")?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
            }
            ME::AircraftIdentification(Identification { tc, ca, cn }) => {
                writeln!(f, " Extended Squitter{transponder}Aircraft identification and category")?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
                writeln!(f, "  Ident:         {cn}")?;
                writeln!(f, "  Category:      {tc}{ca}")?;
            }
            ME::SurfacePosition(..) => {
                writeln!(f, " Extended Squitter{transponder}Surface position")?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
            }
            ME::AirbornePositionBaroAltitude(altitude) => {
                writeln!(
                    f,
                    " Extended Squitter{transponder}Airborne position (barometric altitude)"
                )?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
                write!(f, "{altitude}")?;
            }
            ME::AirborneVelocity(airborne_velocity) => match &airborne_velocity.sub_type {
                AirborneVelocitySubType::GroundSpeedDecoding(_) => {
                    writeln!(
                        f,
                        " Extended Squitter{transponder}Airborne velocity over ground, subsonic"
                    )?;
                    writeln!(f, "  Address:       {icao} {address_type}")?;
                    writeln!(f, "  Air/Ground:    {capability}")?;
                    writeln!(
                        f,
                        "  GNSS delta:    {}{} ft",
                        airborne_velocity.gnss_sign, airborne_velocity.gnss_baro_diff
                    )?;
                    if let Some((heading, ground_speed, vertical_rate)) =
                        airborne_velocity.calculate()
                    {
                        writeln!(f, "  Heading:       {}", libm::ceil(heading as f64))?;
                        writeln!(
                            f,
                            "  Speed:         {} kt groundspeed",
                            libm::floor(ground_speed)
                        )?;
                        writeln!(
                            f,
                            "  Vertical rate: {} ft/min {}",
                            vertical_rate, airborne_velocity.vrate_src
                        )?;
                    } else {
                        writeln!(f, "  Invalid packet")?;
                    }
                }
                AirborneVelocitySubType::AirspeedDecoding(airspeed_decoding) => {
                    writeln!(f, " Extended Squitter{transponder}Airspeed and heading, subsonic",)?;
                    writeln!(f, "  Address:       {icao} {address_type}")?;
                    writeln!(f, "  Air/Ground:    {capability}")?;
                    writeln!(f, "  IAS:           {} kt", airspeed_decoding.airspeed)?;
                    if airborne_velocity.vrate_value > 0 {
                        writeln!(
                            f,
                            "  Baro rate:     {}{} ft/min",
                            airborne_velocity.vrate_sign,
                            (airborne_velocity.vrate_value - 1) * 64
                        )?;
                    }
                    writeln!(f, "  NACv:          {}", airborne_velocity.nac_v)?;
                }
                AirborneVelocitySubType::Reserved0(_) | AirborneVelocitySubType::Reserved1(_) => {
                    writeln!(
                        f,
                        " Extended Squitter{transponder}Airborne Velocity status (reserved)",
                    )?;
                    writeln!(f, "  Address:       {icao} {address_type}")?;
                }
            },
            ME::AirbornePositionGNSSAltitude(altitude) => {
                writeln!(f, " Extended Squitter{transponder}Airborne position (GNSS altitude)",)?;
                writeln!(f, "  Address:      {icao} {address_type}")?;
                write!(f, "{altitude}")?;
            }
            ME::Reserved0(_) | ME::Reserved1(_) => {
                writeln!(f, " Extended Squitter{transponder}Unknown")?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
            }
            ME::SurfaceSystemStatus(_) => {
                writeln!(f, " Extended Squitter{transponder}Reserved for surface system status",)?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
            }
            ME::AircraftStatus(AircraftStatus { emergency_state, squawk, .. }) => {
                writeln!(f, " Extended Squitter{transponder}Emergency/priority status",)?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
                writeln!(f, "  Squawk:        {squawk:x?}")?;
                writeln!(f, "  Emergency/priority:    {emergency_state}")?;
            }
            ME::TargetStateAndStatusInformation(target_info) => {
                writeln!(f, " Extended Squitter{transponder}Target state and status (V2)",)?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
                writeln!(f, "  Target State and Status:")?;
                writeln!(f, "    Target altitude:   MCP, {} ft", target_info.altitude)?;
                writeln!(f, "    Altimeter setting: {} millibars", target_info.qnh)?;
                if target_info.is_heading {
                    writeln!(f, "    Target heading:    {}", target_info.heading)?;
                }
                if target_info.tcas {
                    write!(f, "    ACAS:              operational ")?;
                    if target_info.autopilot {
                        write!(f, "autopilot ")?;
                    }
                    if target_info.vnac {
                        write!(f, "vnav ")?;
                    }
                    if target_info.alt_hold {
                        write!(f, "altitude-hold ")?;
                    }
                    if target_info.approach {
                        write!(f, " approach")?;
                    }
                    writeln!(f)?;
                } else {
                    writeln!(f, "    ACAS:              NOT operational")?;
                }
                writeln!(f, "    NACp:              {}", target_info.nacp)?;
                writeln!(f, "    NICbaro:           {}", target_info.nicbaro)?;
                writeln!(f, "    SIL:               {} (per sample)", target_info.sil)?;
                writeln!(f, "    QNH:               {} millibars", target_info.qnh)?;
            }
            ME::AircraftOperationalCoordination(_) => {
                writeln!(f, " Extended Squitter{transponder}Aircraft Operational Coordination",)?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
            }
            ME::AircraftOperationStatus(OperationStatus::Airborne(opstatus_airborne)) => {
                writeln!(
                    f,
                    " Extended Squitter{transponder}Aircraft operational status (airborne)",
                )?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
                write!(f, "  Aircraft Operational Status:\n{opstatus_airborne}")?;
            }
            ME::AircraftOperationStatus(OperationStatus::Surface(opstatus_surface)) => {
                writeln!(
                    f,
                    " Extended Squitter{transponder}Aircraft operational status (surface)",
                )?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
                writeln!(f, "  Air/Ground:    {capability}")?;
                write!(f, "  Aircraft Operational Status:\n {opstatus_surface}")?;
            }
            ME::AircraftOperationStatus(OperationStatus::Reserved(..)) => {
                writeln!(
                    f,
                    " Extended Squitter{transponder}Aircraft operational status (reserved)",
                )?;
                writeln!(f, "  Address:       {icao} {address_type}")?;
            }
        }
        Ok(f)
    }
}

/// [`ME::AirborneVelocity`] && [`AirborneVelocitySubType::GroundSpeedDecoding`]
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroundSpeedDecoding {
    pub ew_sign: Sign,
    #[deku(endian = "big", bits = "10")]
    pub ew_vel: u16,
    pub ns_sign: Sign,
    #[deku(endian = "big", bits = "10")]
    pub ns_vel: u16,
}

/// [`ME::AirborneVelocity`] && [`AirborneVelocitySubType::AirspeedDecoding`]
#[derive(Debug, PartialEq, Eq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AirspeedDecoding {
    #[deku(bits = "1")]
    pub status_heading: u8,
    #[deku(endian = "big", bits = "10")]
    pub mag_heading: u16,
    #[deku(bits = "1")]
    pub airspeed_type: u8,
    #[deku(
        endian = "big",
        bits = "10",
        map = "|airspeed: u16| -> result::Result<_, DekuError> {Ok(if airspeed > 0 { airspeed - 1 } else { 0 })}"
    )]
    pub airspeed: u16,
}

/// Aircraft Operational Status Subtype
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "3")]
pub enum OperationStatus {
    #[deku(id = "0")]
    Airborne(OperationStatusAirborne),

    #[deku(id = "1")]
    Surface(OperationStatusSurface),

    #[deku(id_pat = "2..=7")]
    Reserved(#[deku(bits = "5")] u8, [u8; 5]),
}

/// [`ME::AircraftOperationStatus`] && [`OperationStatus`] == 0
///
/// Version 2 support only
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OperationStatusAirborne {
    /// CC (16 bits)
    pub capability_class: CapabilityClassAirborne,

    /// OM
    pub operational_mode: OperationalMode,

    #[deku(pad_bytes_before = "1")] // reserved: OM last 8 bits (diff for airborne/surface)
    pub version_number: ADSBVersion,

    #[deku(bits = "1")]
    pub nic_supplement_a: u8,

    #[deku(bits = "4")]
    pub navigational_accuracy_category: u8,

    #[deku(bits = "2")]
    pub geometric_vertical_accuracy: u8,

    #[deku(bits = "2")]
    pub source_integrity_level: u8,

    #[deku(bits = "1")]
    pub barometric_altitude_integrity: u8,

    #[deku(bits = "1")]
    pub horizontal_reference_direction: u8,

    #[deku(bits = "1")]
    #[deku(pad_bits_after = "1")] // reserved
    pub sil_supplement: u8,
}

impl fmt::Display for OperationStatusAirborne {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "   Version:            {}", self.version_number)?;
        writeln!(f, "   Capability classes:{}", self.capability_class)?;
        writeln!(f, "   Operational modes: {}", self.operational_mode)?;
        writeln!(f, "   NIC-A:              {}", self.nic_supplement_a)?;
        writeln!(f, "   NACp:               {}", self.navigational_accuracy_category)?;
        writeln!(f, "   GVA:                {}", self.geometric_vertical_accuracy)?;
        writeln!(f, "   SIL:                {} (per hour)", self.source_integrity_level)?;
        writeln!(f, "   NICbaro:            {}", self.barometric_altitude_integrity)?;
        if self.horizontal_reference_direction == 1 {
            writeln!(f, "   Heading reference:  magnetic north")?;
        } else {
            writeln!(f, "   Heading reference:  true north")?;
        }
        Ok(())
    }
}

/// [`ME::AircraftOperationStatus`]
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CapabilityClassAirborne {
    #[deku(bits = "2", assert_eq = "0")]
    pub reserved0: u8,

    /// TCAS Operational
    #[deku(bits = "1")]
    pub acas: u8,

    /// 1090ES IN
    ///
    /// bit 12
    #[deku(bits = "1")]
    pub cdti: u8,

    #[deku(bits = "2", assert_eq = "0")]
    pub reserved1: u8,

    #[deku(bits = "1")]
    pub arv: u8,
    #[deku(bits = "1")]
    pub ts: u8,
    #[deku(bits = "2")]
    #[deku(pad_bits_after = "6")] //reserved
    pub tc: u8,
}

impl fmt::Display for CapabilityClassAirborne {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.acas == 1 {
            write!(f, " ACAS")?;
        }
        if self.cdti == 1 {
            write!(f, " CDTI")?;
        }
        if self.arv == 1 {
            write!(f, " ARV")?;
        }
        if self.ts == 1 {
            write!(f, " TS")?;
        }
        if self.tc == 1 {
            write!(f, " TC")?;
        }
        Ok(())
    }
}

/// [`ME::AircraftOperationStatus`] && [`OperationStatus`] == 1
///
/// Version 2 support only
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OperationStatusSurface {
    /// CC (14 bits)
    pub capability_class: CapabilityClassSurface,

    /// CC L/W codes
    #[deku(bits = "4")]
    pub lw_codes: u8,

    /// OM
    pub operational_mode: OperationalMode,

    /// OM last 8 bits (diff for airborne/surface)
    // TODO: parse:
    // http://www.anteni.net/adsb/Doc/1090-WP30-18-DRAFT_DO-260B-V42.pdf
    // 2.2.3.2.7.2.4.7 “GPS Antenna Offset” OM Code Subfield in Aircraft Operational Status Messages
    pub gps_antenna_offset: u8,

    pub version_number: ADSBVersion,

    #[deku(bits = "1")]
    pub nic_supplement_a: u8,

    #[deku(bits = "4")]
    #[deku(pad_bits_after = "2")] // reserved
    pub navigational_accuracy_category: u8,

    #[deku(bits = "2")]
    pub source_integrity_level: u8,

    #[deku(bits = "1")]
    pub barometric_altitude_integrity: u8,

    #[deku(bits = "1")]
    pub horizontal_reference_direction: u8,

    #[deku(bits = "1")]
    #[deku(pad_bits_after = "1")] // reserved
    pub sil_supplement: u8,
}

impl fmt::Display for OperationStatusSurface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "  Version:            {}", self.version_number)?;
        writeln!(f, "   NIC-A:              {}", self.nic_supplement_a)?;
        write!(f, "{}", self.capability_class)?;
        write!(f, "   Capability classes:")?;
        if self.lw_codes != 0 {
            writeln!(f, " L/W={}", self.lw_codes)?;
        } else {
            writeln!(f)?;
        }
        write!(f, "   Operational modes: {}", self.operational_mode)?;
        writeln!(f)?;
        writeln!(f, "   NACp:               {}", self.navigational_accuracy_category)?;
        writeln!(f, "   SIL:                {} (per hour)", self.source_integrity_level)?;
        writeln!(f, "   NICbaro:            {}", self.barometric_altitude_integrity)?;
        if self.horizontal_reference_direction == 1 {
            writeln!(f, "   Heading reference:  magnetic north")?;
        } else {
            writeln!(f, "   Heading reference:  true north")?;
        }
        Ok(())
    }
}

/// [`ME::AircraftOperationStatus`]
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CapabilityClassSurface {
    /// 0, 0 in current version, reserved as id for later versions
    #[deku(bits = "2", assert_eq = "0")]
    pub reserved0: u8,

    /// Position Offset Applied
    #[deku(bits = "1")]
    pub poe: u8,

    /// Aircraft has ADS-B 1090ES Receive Capability
    #[deku(bits = "1")]
    #[deku(pad_bits_after = "2")] // reserved
    pub es1090: u8,

    /// Class B2 Ground Vehicle transmitting with less than 70 watts
    #[deku(bits = "1")]
    pub b2_low: u8,

    /// Aircraft has ADS-B UAT Receive Capability
    #[deku(bits = "1")]
    pub uat_in: u8,

    /// Nagivation Accuracy Category for Velocity
    #[deku(bits = "3")]
    pub nac_v: u8,

    /// NIC Supplement used on the Surface
    #[deku(bits = "1")]
    pub nic_supplement_c: u8,
}

impl fmt::Display for CapabilityClassSurface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "   NIC-C:              {}", self.nic_supplement_c)?;
        writeln!(f, "   NACv:               {}", self.nac_v)?;
        Ok(())
    }
}

/// `OperationMode` field not including the last 8 bits that are different for Surface/Airborne
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OperationalMode {
    /// (0, 0) in Version 2, reserved for other values
    #[deku(bits = "2", assert_eq = "0")]
    reserved: u8,

    #[deku(bits = "1")]
    tcas_ra_active: bool,

    #[deku(bits = "1")]
    ident_switch_active: bool,

    #[deku(bits = "1")]
    reserved_recv_atc_service: bool,

    #[deku(bits = "1")]
    single_antenna_flag: bool,

    #[deku(bits = "2")]
    system_design_assurance: u8,
}

impl fmt::Display for OperationalMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.tcas_ra_active {
            write!(f, " TCAS")?;
        }
        if self.ident_switch_active {
            write!(f, " IDENT_SWITCH_ACTIVE")?;
        }
        if self.reserved_recv_atc_service {
            write!(f, " ATC")?;
        }
        if self.single_antenna_flag {
            write!(f, " SAF")?;
        }
        if self.system_design_assurance != 0 {
            write!(f, " SDA={}", self.system_design_assurance)?;
        }
        Ok(())
    }
}

/// ADS-B Defined from different ICAO documents
///
/// reference: ICAO 9871 (5.3.2.3)
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "3")]
pub enum ADSBVersion {
    #[deku(id = "0")]
    DOC9871AppendixA,
    #[deku(id = "1")]
    DOC9871AppendixB,
    #[deku(id = "2")]
    DOC9871AppendixC,
}

impl fmt::Display for ADSBVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.deku_id().unwrap())
    }
}

/// Control Field (B.3) for [`crate::DF::TisB`]
///
/// reference: ICAO 9871
#[derive(Debug, PartialEq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ControlField {
    t: ControlFieldType,
    /// AA: Address, Announced
    pub aa: ICAO,
    /// ME: message, extended quitter
    pub me: ME,
}

impl fmt::Display for ControlField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            self.me.to_string(self.aa, &format!("{}", self.t), Capability::AG_UNCERTAIN3, false,)?
        )
    }
}

#[derive(Debug, PartialEq, Eq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "3")]
#[allow(non_camel_case_types)]
pub enum ControlFieldType {
    /// ADS-B Message from a non-transponder device
    #[deku(id = "0")]
    ADSB_ES_NT,

    /// Reserved for ADS-B for ES/NT devices for alternate address space
    #[deku(id = "1")]
    ADSB_ES_NT_ALT,

    /// Code 2, Fine Format TIS-B Message
    #[deku(id = "2")]
    TISB_FINE,

    /// Code 3, Coarse Format TIS-B Message
    #[deku(id = "3")]
    TISB_COARSE,

    /// Code 4, Coarse Format TIS-B Message
    #[deku(id = "4")]
    TISB_MANAGE,

    /// Code 5, TIS-B Message for replay ADS-B Message
    ///
    /// Anonymous 24-bit addresses
    #[deku(id = "5")]
    TISB_ADSB_RELAY,

    /// Code 6, TIS-B Message, Same as DF=17
    #[deku(id = "6")]
    TISB_ADSB,

    /// Code 7, Reserved
    #[deku(id = "7")]
    Reserved,
}

impl fmt::Display for ControlFieldType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s_type = match self {
            Self::ADSB_ES_NT | Self::ADSB_ES_NT_ALT => "(ADS-B)",
            Self::TISB_COARSE | Self::TISB_ADSB_RELAY | Self::TISB_FINE => "(TIS-B)",
            Self::TISB_MANAGE | Self::TISB_ADSB => "(ADS-R)",
            Self::Reserved => "(unknown addressing scheme)",
        };
        write!(f, "{s_type}")
    }
}

/// Table: A-2-97
#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AircraftStatus {
    pub sub_type: AircraftStatusType,
    pub emergency_state: EmergencyState,
    #[deku(
        bits = "13",
        endian = "big",
        map = "|squawk: u32| -> Result<_, DekuError> {Ok(decode_id13_field(squawk))}"
    )]
    pub squawk: u32,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "3")]
pub enum AircraftStatusType {
    #[deku(id = "0")]
    NoInformation,
    #[deku(id = "1")]
    EmergencyPriorityStatus,
    #[deku(id = "2")]
    ACASRaBroadcast,
    #[deku(id_pat = "_")]
    Reserved,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "3")]
pub enum EmergencyState {
    None = 0,
    General = 1,
    Lifeguard = 2,
    MinimumFuel = 3,
    NoCommunication = 4,
    UnlawfulInterference = 5,
    DownedAircraft = 6,
    Reserved2 = 7,
}

impl fmt::Display for EmergencyState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Self::None => "no emergency",
            Self::General => "general",
            Self::Lifeguard => "lifeguard",
            Self::MinimumFuel => "minimum fuel",
            Self::NoCommunication => "no communication",
            Self::UnlawfulInterference => "unflawful interference",
            Self::DownedAircraft => "downed aircraft",
            Self::Reserved2 => "reserved2",
        };
        write!(f, "{s}")?;
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OperationCodeSurface {
    #[deku(bits = "1")]
    pub poe: u8,
    #[deku(bits = "1")]
    pub cdti: u8,
    #[deku(bits = "1")]
    pub b2_low: u8,
    #[deku(bits = "3")]
    #[deku(pad_bits_before = "6")]
    pub lw: u8,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Identification {
    pub tc: TypeCoding,

    #[deku(bits = "3")]
    pub ca: u8,

    /// N-Number / Tail Number
    #[deku(reader = "aircraft_identification_read(deku::reader)")]
    pub cn: String,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "5")]
pub enum TypeCoding {
    D = 1,
    C = 2,
    B = 3,
    A = 4,
}

impl fmt::Display for TypeCoding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::D => "D",
                Self::C => "C",
                Self::B => "B",
                Self::A => "A",
            }
        )
    }
}

/// Target State and Status (§2.2.3.2.7.1)
#[derive(Copy, Clone, Debug, PartialEq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetStateAndStatusInformation {
    // TODO Support Target State and Status defined in DO-260A, ADS-B Version=1
    // TODO Support reserved 2..=3
    #[deku(bits = "2")]
    pub subtype: u8,
    #[deku(bits = "1")]
    pub is_fms: bool,
    #[deku(
        bits = "12",
        endian = "big",
        map = "|altitude: u32| -> Result<_, DekuError> {Ok(if altitude > 1 {(altitude - 1) * 32} else {0} )}"
    )]
    pub altitude: u32,
    #[deku(
        bits = "9",
        endian = "big",
        map = "|qnh: u32| -> Result<_, DekuError> {if qnh == 0 { Ok(0.0) } else { Ok(800.0 + ((qnh - 1) as f32) * 0.8)}}"
    )]
    pub qnh: f32,
    #[deku(bits = "1")]
    pub is_heading: bool,
    #[deku(
        bits = "9",
        endian = "big",
        map = "|heading: u16| -> Result<_, DekuError> {Ok(heading as f32 * 180.0 / 256.0)}"
    )]
    pub heading: f32,
    #[deku(bits = "4")]
    pub nacp: u8,
    #[deku(bits = "1")]
    pub nicbaro: u8,
    #[deku(bits = "2")]
    pub sil: u8,
    #[deku(bits = "1")]
    pub mode_validity: bool,
    #[deku(bits = "1")]
    pub autopilot: bool,
    #[deku(bits = "1")]
    pub vnac: bool,
    #[deku(bits = "1")]
    pub alt_hold: bool,
    #[deku(bits = "1")]
    pub imf: bool,
    #[deku(bits = "1")]
    pub approach: bool,
    #[deku(bits = "1")]
    pub tcas: bool,
    #[deku(bits = "1")]
    #[deku(pad_bits_after = "2")] // reserved
    pub lnav: bool,
}

/// [`ME::AirborneVelocity`]
#[derive(Debug, PartialEq, Eq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AirborneVelocity {
    #[deku(bits = "3")]
    pub st: u8,
    #[deku(bits = "5")]
    pub nac_v: u8,
    #[deku(ctx = "*st")]
    pub sub_type: AirborneVelocitySubType,
    pub vrate_src: VerticalRateSource,
    pub vrate_sign: Sign,
    #[deku(endian = "big", bits = "9")]
    pub vrate_value: u16,
    #[deku(bits = "2")]
    pub reverved: u8,
    pub gnss_sign: Sign,
    #[deku(
        bits = "7",
        map = "|gnss_baro_diff: u16| -> Result<_, DekuError> {Ok(if gnss_baro_diff > 1 {(gnss_baro_diff - 1)* 25} else { 0 })}"
    )]
    pub gnss_baro_diff: u16,
}

impl AirborneVelocity {
    /// Return effective (`heading`, `ground_speed`, `vertical_rate`) for groundspeed
    #[must_use]
    pub fn calculate(&self) -> Option<(f32, f64, i16)> {
        if let AirborneVelocitySubType::GroundSpeedDecoding(ground_speed) = &self.sub_type {
            let v_ew = f64::from((ground_speed.ew_vel as i16 - 1) * ground_speed.ew_sign.value());
            let v_ns = f64::from((ground_speed.ns_vel as i16 - 1) * ground_speed.ns_sign.value());
            let h = libm::atan2(v_ew, v_ns) * (360.0 / (2.0 * f64::consts::PI));
            let heading = if h < 0.0 { h + 360.0 } else { h };

            let vrate = self
                .vrate_value
                .checked_sub(1)
                .and_then(|v| v.checked_mul(64))
                .map(|v| (v as i16) * self.vrate_sign.value());

            if let Some(vrate) = vrate {
                return Some((heading as f32, libm::hypot(v_ew, v_ns), vrate));
            }
        }
        None
    }
}

/// Airborne Velocity Message “Subtype” Code Field Encoding
#[derive(Debug, PartialEq, Eq, DekuRead, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(ctx = "st: u8", id = "st")]
pub enum AirborneVelocitySubType {
    #[deku(id = "0")]
    Reserved0(#[deku(bits = "22")] u32),

    #[deku(id_pat = "1..=2")]
    GroundSpeedDecoding(GroundSpeedDecoding),

    #[deku(id_pat = "3..=4")]
    AirspeedDecoding(AirspeedDecoding),

    #[deku(id_pat = "5..=7")]
    Reserved1(#[deku(bits = "22")] u32),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "3")]
pub enum AirborneVelocityType {
    Subsonic = 1,
    Supersonic = 3,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[deku(ctx = "t: AirborneVelocityType")]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AirborneVelocitySubFields {
    pub dew: DirectionEW,
    #[deku(reader = "Self::read_v(deku::reader, t)")]
    pub vew: u16,
    pub dns: DirectionNS,
    #[deku(reader = "Self::read_v(deku::reader, t)")]
    pub vns: u16,
}

impl AirborneVelocitySubFields {
    fn read_v<R: Read + Seek>(
        reader: &mut Reader<R>,
        t: AirborneVelocityType,
    ) -> result::Result<u16, DekuError> {
        match t {
            AirborneVelocityType::Subsonic => {
                u16::from_reader_with_ctx(reader, (deku::ctx::Endian::Big, deku::ctx::BitSize(10)))
                    .map(|value| value - 1)
            }
            AirborneVelocityType::Supersonic => {
                u16::from_reader_with_ctx(reader, (deku::ctx::Endian::Big, deku::ctx::BitSize(10)))
                    .map(|value| 4 * (value - 1))
            }
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum DirectionEW {
    WestToEast = 0,
    EastToWest = 1,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum DirectionNS {
    SouthToNorth = 0,
    NorthToSouth = 1,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum SourceBitVerticalRate {
    GNSS = 0,
    Barometer = 1,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum SignBitVerticalRate {
    Up = 0,
    Down = 1,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, DekuRead)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum SignBitGNSSBaroAltitudesDiff {
    Above = 0,
    Below = 1,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum VerticalRateSource {
    BarometricPressureAltitude = 0,
    GeometricAltitude = 1,
}

impl fmt::Display for VerticalRateSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::BarometricPressureAltitude => "barometric",
                Self::GeometricAltitude => "GNSS",
            }
        )
    }
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SurfacePosition {
    #[deku(bits = "5")]
    pub tc: u8,
    #[deku(bits = "7")]
    pub mov: u8,
    pub s: StatusForGroundTrack,
    #[deku(bits = "7")]
    pub trk: u8,
    #[deku(bits = "1")]
    pub t: bool,
    pub f: CPRFormat,
    #[deku(bits = "17", endian = "big")]
    pub lat_cpr: u32,
    #[deku(bits = "17", endian = "big")]
    pub lon_cpr: u32,
}

#[derive(Debug, PartialEq, Eq, DekuRead, Copy, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[deku(id_type = "u8", bits = "1")]
pub enum StatusForGroundTrack {
    Invalid = 0,
    Valid = 1,
}
