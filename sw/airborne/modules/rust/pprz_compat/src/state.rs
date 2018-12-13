pub const INT32_POS_FRAC: u32 = 8;
pub const INT32_POS_OF_CM: f64 = 2.56;
pub const INT32_POS_OF_CM_NUM: u32 = 64;
pub const INT32_POS_OF_CM_DEN: u32 = 25;
pub const INT32_SPEED_FRAC: u32 = 19;
pub const INT32_SPEED_OF_CM_S: f64 = 5242.88;
pub const INT32_SPEED_OF_CM_S_NUM: u32 = 41943;
pub const INT32_SPEED_OF_CM_S_DEN: u32 = 8;
pub const INT32_ACCEL_FRAC: u32 = 10;
pub const INT32_MAG_FRAC: u32 = 11;
pub const INT32_PERCENTAGE_FRAC: u32 = 10;
pub const INT32_QUAT_FRAC: u32 = 15;
pub const INT32_ANGLE_FRAC: u32 = 12;
pub const INT32_RATE_FRAC: u32 = 12;
pub const INT32_TRIG_FRAC: u32 = 14;
pub const HIGH_RES_TRIG_FRAC: u32 = 20;
pub const ORREP_QUAT_I: u32 = 0;
pub const ORREP_EULER_I: u32 = 1;
pub const ORREP_RMAT_I: u32 = 2;
pub const ORREP_QUAT_F: u32 = 3;
pub const ORREP_EULER_F: u32 = 4;
pub const ORREP_RMAT_F: u32 = 5;
pub const POS_ECEF_I: u32 = 0;
pub const POS_NED_I: u32 = 1;
pub const POS_ENU_I: u32 = 2;
pub const POS_LLA_I: u32 = 3;
pub const POS_UTM_I: u32 = 4;
pub const POS_ECEF_F: u32 = 5;
pub const POS_NED_F: u32 = 6;
pub const POS_ENU_F: u32 = 7;
pub const POS_LLA_F: u32 = 8;
pub const POS_UTM_F: u32 = 9;
pub const POS_LOCAL_COORD: u32 = 198;
pub const POS_GLOBAL_COORD: u32 = 825;
pub const SPEED_ECEF_I: u32 = 0;
pub const SPEED_NED_I: u32 = 1;
pub const SPEED_ENU_I: u32 = 2;
pub const SPEED_HNORM_I: u32 = 3;
pub const SPEED_HDIR_I: u32 = 4;
pub const SPEED_ECEF_F: u32 = 5;
pub const SPEED_NED_F: u32 = 6;
pub const SPEED_ENU_F: u32 = 7;
pub const SPEED_HNORM_F: u32 = 8;
pub const SPEED_HDIR_F: u32 = 9;
pub const SPEED_LOCAL_COORD: u32 = 198;
pub const ACCEL_ECEF_I: u32 = 0;
pub const ACCEL_NED_I: u32 = 1;
pub const ACCEL_ECEF_F: u32 = 2;
pub const ACCEL_NED_F: u32 = 3;
pub const RATE_I: u32 = 0;
pub const RATE_F: u32 = 1;
pub const WINDSPEED_I: u32 = 0;
pub const DOWNWIND_I: u32 = 1;
pub const AIRSPEED_I: u32 = 2;
pub const WINDSPEED_F: u32 = 3;
pub const DOWNWIND_F: u32 = 4;
pub const AIRSPEED_F: u32 = 5;
pub const AOA_F: u32 = 6;
pub const SIDESLIP_F: u32 = 7;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Uint8Vect3 {
    pub x: u8,
    pub y: u8,
    pub z: u8,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int8Vect3 {
    pub x: i8,
    pub y: i8,
    pub z: i8,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Uint16Vect3 {
    pub x: u16,
    pub y: u16,
    pub z: u16,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int16Vect3 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32Vect2 {
    pub x: i32,
    pub y: i32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32Vect3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}
/// @brief Rotation quaternion
/// @details Units: BFP with #INT32_QUAT_FRAC
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32Quat {
    pub qi: i32,
    pub qx: i32,
    pub qy: i32,
    pub qz: i32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int64Quat {
    pub qi: i64,
    pub qx: i64,
    pub qy: i64,
    pub qz: i64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int16Eulers {
    pub phi: i16,
    pub theta: i16,
    pub psi: i16,
}
/// @brief euler angles
/// @details Units: rad in BFP with #INT32_ANGLE_FRAC
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32Eulers {
    ///< in rad with #INT32_ANGLE_FRAC
    pub phi: i32,
    ///< in rad with #INT32_ANGLE_FRAC
    pub theta: i32,
    ///< in rad with #INT32_ANGLE_FRAC
    pub psi: i32,
}
/// @brief rotation matrix
/// @details Units: rad in BFP with #INT32_TRIG_FRAC
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32RMat {
    pub m: [i32; 9usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32Mat33 {
    pub m: [i32; 9usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int16Rates {
    pub p: i16,
    pub q: i16,
    pub r: i16,
}
/// @brief angular rates
/// @details Units: rad/s in BFP with #INT32_RATE_FRAC
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int32Rates {
    ///< in rad/s with #INT32_RATE_FRAC
    pub p: i32,
    ///< in rad/s with #INT32_RATE_FRAC
    pub q: i32,
    ///< in rad/s with #INT32_RATE_FRAC
    pub r: i32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int64Rates {
    pub p: i64,
    pub q: i64,
    pub r: i64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int64Vect2 {
    pub x: i64,
    pub y: i64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Int64Vect3 {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatVect2 {
    pub x: f32,
    pub y: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatVect3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
/// @brief Roation quaternion
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatQuat {
    pub qi: f32,
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatMat33 {
    pub m: [f32; 9usize],
}
/// @brief rotation matrix
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatRMat {
    pub m: [f32; 9usize],
}
/// @brief euler angles
/// @details Units: radians
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatEulers {
    ///< in radians
    pub phi: f32,
    ///< in radians
    pub theta: f32,
    ///< in radians
    pub psi: f32,
}
/// @brief angular rates
/// @details Units: rad/s
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct FloatRates {
    ///< in rad/s
    pub p: f32,
    ///< in rad/s
    pub q: f32,
    ///< in rad/s
    pub r: f32,
}
/// @brief vector in EarthCenteredEarthFixed coordinates
/// @details Origin at center of mass of the Earth. Z-axis is pointing north,
/// the x-axis intersects the sphere of the earth at 0° latitude (Equator)
/// and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
/// Units: centimeters
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct EcefCoor_i {
    ///< in centimeters
    pub x: i32,
    ///< in centimeters
    pub y: i32,
    ///< in centimeters
    pub z: i32,
}
/// @brief vector in Latitude, Longitude and Altitude
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LlaCoor_i {
    ///< in degrees*1e7
    pub lat: i32,
    ///< in degrees*1e7
    pub lon: i32,
    ///< in millimeters above WGS84 reference ellipsoid
    pub alt: i32,
}
/// @brief vector in North East Down coordinates
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct NedCoor_i {
    ///< North
    pub x: i32,
    ///< East
    pub y: i32,
    ///< Down
    pub z: i32,
}
/// @brief vector in East North Up coordinates
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct EnuCoor_i {
    ///< East
    pub x: i32,
    ///< North
    pub y: i32,
    ///< Up
    pub z: i32,
}
/// @brief position in UTM coordinates
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct UtmCoor_i {
    ///< in centimeters
    pub north: i32,
    ///< in centimeters
    pub east: i32,
    ///< in millimeters (above WGS84 reference ellipsoid or above MSL)
    pub alt: i32,
    ///< UTM zone number
    pub zone: u8,
}
/// @brief definition of the local (flat earth) coordinate system
/// @details Defines the origin of the local coordinate system
/// in ECEF and LLA coordinates and the roation matrix from
/// ECEF to local frame
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LtpDef_i {
    ///< Reference point in ecef
    pub ecef: EcefCoor_i,
    ///< Reference point in lla
    pub lla: LlaCoor_i,
    ///< Rotation matrix
    pub ltp_of_ecef: Int32RMat,
    ///< Height above mean sea level in mm
    pub hmsl: i32,
}
/// @brief vector in EarthCenteredEarthFixed coordinates
/// @details Origin at center of mass of the Earth. Z-axis is pointing north,
/// the x-axis intersects the sphere of the earth at 0° latitude (Equator)
/// and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
/// Units: meters
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct EcefCoor_f {
    ///< in meters
    pub x: f32,
    ///< in meters
    pub y: f32,
    ///< in meters
    pub z: f32,
}
/// @brief vector in Latitude, Longitude and Altitude
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LlaCoor_f {
    ///< in radians
    pub lat: f32,
    ///< in radians
    pub lon: f32,
    ///< in meters (normally above WGS84 reference ellipsoid)
    pub alt: f32,
}
/// @brief vector in North East Down coordinates
/// Units: meters
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct NedCoor_f {
    ///< in meters
    pub x: f32,
    ///< in meters
    pub y: f32,
    ///< in meters
    pub z: f32,
}
/// @brief vector in East North Up coordinates
/// Units: meters
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct EnuCoor_f {
    ///< in meters
    pub x: f32,
    ///< in meters
    pub y: f32,
    ///< in meters
    pub z: f32,
}
/// @brief position in UTM coordinates
/// Units: meters
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct UtmCoor_f {
    ///< in meters
    pub north: f32,
    ///< in meters
    pub east: f32,
    ///< in meters (above WGS84 reference ellipsoid or above MSL)
    pub alt: f32,
    ///< UTM zone number
    pub zone: u8,
}
/// @brief definition of the local (flat earth) coordinate system
/// @details Defines the origin of the local coordinate system
/// in ECEF and LLA coordinates and the roation matrix from
/// ECEF to local frame
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LtpDef_f {
    ///< origin of local frame in ECEF
    pub ecef: EcefCoor_f,
    ///< origin of local frame in LLA
    pub lla: LlaCoor_f,
    ///< rotation from ECEF to local frame
    pub ltp_of_ecef: FloatRMat,
    ///< Height above mean sea level in meters
    pub hmsl: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct OrientationReps {
    /// Holds the status bits for all orientation representations.
    /// When the corresponding bit is set, the representation
    /// is already computed.
    pub status: u8,
    /// Orientation quaternion.
    /// Units: #INT32_QUAT_FRAC
    pub quat_i: Int32Quat,
    /// Orientation in zyx euler angles.
    /// Units: rad in BFP with #INT32_ANGLE_FRAC
    pub eulers_i: Int32Eulers,
    /// Orientation rotation matrix.
    /// Units: rad in BFP with #INT32_TRIG_FRAC
    pub rmat_i: Int32RMat,
    /// Orientation as quaternion.
    /// Units: unit length quaternion
    pub quat_f: FloatQuat,
    /// Orienation in zyx euler angles.
    /// Units: rad
    pub eulers_f: FloatEulers,
    /// Orientation rotation matrix.
    /// Units: rad
    pub rmat_f: FloatRMat,
}
/// Structure holding vehicle state data.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct State {
    /// Holds the status bits for all position representations.
    /// When the corresponding bit is set the representation
    /// is already computed.
    pub pos_status: u16,
    /// Position in EarthCenteredEarthFixed coordinates.
    /// Units: centimeters
    pub ecef_pos_i: EcefCoor_i,
    /// Position in Latitude, Longitude and Altitude.
    /// Units lat,lon: degrees*1e7
    /// Units alt: milimeters above reference ellipsoid
    pub lla_pos_i: LlaCoor_i,
    /// Definition of the local (flat earth) coordinate system.
    /// Defines the origin of the local NorthEastDown coordinate system
    /// in ECEF (EarthCenteredEarthFixed) and LLA (LatitudeLongitudeAlt)
    /// coordinates and the roation matrix from ECEF to local frame.
    /// (int version)
    pub ned_origin_i: LtpDef_i,
    /// true if local int coordinate frame is initialsed
    pub ned_initialized_i: bool,
    /// Position in North East Down coordinates.
    /// with respect to ned_origin_i (flat earth)
    /// Units: m in BFP with INT32_POS_FRAC
    pub ned_pos_i: NedCoor_i,
    /// Position in East North Up coordinates.
    /// with respect to ned_origin_i (flat earth)
    /// Units: m in BFP with INT32_POS_FRAC
    pub enu_pos_i: EnuCoor_i,
    /// Position in UTM coordinates.
    /// Units x,y: meters.
    /// Units z: meters above MSL
    pub utm_pos_f: UtmCoor_f,
    /// Altitude above ground level.
    /// Unit: meters
    pub alt_agl_f: f32,
    /// Position in Latitude, Longitude and Altitude.
    /// Units lat,lon: radians
    /// Units alt: meters above reference ellipsoid
    pub lla_pos_f: LlaCoor_f,
    /// Position in EarthCenteredEarthFixed coordinates.
    /// Units: meters
    pub ecef_pos_f: EcefCoor_f,
    /// Definition of the local (flat earth) coordinate system.
    /// Defines the origin of the local NorthEastDown coordinate system
    /// in ECEF (EarthCenteredEarthFixed) and LLA (LatitudeLongitudeAlt)
    /// coordinates and the roation matrix from ECEF to local frame.
    /// (float version)
    pub ned_origin_f: LtpDef_f,
    /// True if local float coordinate frame is initialsed
    pub ned_initialized_f: bool,
    /// Definition of the origin of Utm coordinate system.
    /// Defines the origin of the local NorthEastDown coordinate system
    /// in UTM coordinates, used as a reference when ned_origin is not
    /// initialized.
    /// Altitude is height above MSL.
    /// (float version)
    pub utm_origin_f: UtmCoor_f,
    /// True if utm origin (float) coordinate frame is initialsed
    pub utm_initialized_f: bool,
    /// Position in North East Down coordinates.
    /// with respect to ned_origin_i (flat earth)
    /// Units: meters
    pub ned_pos_f: NedCoor_f,
    /// Position in East North Up coordinates.
    /// with respect to ned_origin_i (flat earth)
    /// Units: meters
    pub enu_pos_f: EnuCoor_f,
    /// @addtogroup state_velocity
    ///  @{ */
    ////**
    /// Holds the status bits for all ground speed representations.
    /// When the corresponding bit is one the representation
    /// is already computed.
    pub speed_status: u16,
    /// Velocity in EarthCenteredEarthFixed coordinates.
    /// Units: m/s in BFP with #INT32_SPEED_FRAC
    pub ecef_speed_i: EcefCoor_i,
    /// Velocity in North East Down coordinates.
    /// Units: m/s in BFP with #INT32_SPEED_FRAC
    pub ned_speed_i: NedCoor_i,
    /// Velocity in East North Up coordinates.
    /// Units: m/s in BFP with #INT32_SPEED_FRAC
    pub enu_speed_i: EnuCoor_i,
    /// Norm of horizontal ground speed.
    /// Unit: m/s in BFP with #INT32_SPEED_FRAC
    pub h_speed_norm_i: u32,
    /// Direction of horizontal ground speed.
    /// Unit: rad in BFP with #INT32_ANGLE_FRAC
    /// (clockwise, zero=north)
    pub h_speed_dir_i: i32,
    /// Velocity in EarthCenteredEarthFixed coordinates.
    /// Units: m/s
    pub ecef_speed_f: EcefCoor_f,
    /// @brief speed in North East Down coordinates
    /// @details Units: m/s
    pub ned_speed_f: NedCoor_f,
    /// Velocity in East North Up coordinates.
    /// Units: m/s
    pub enu_speed_f: EnuCoor_f,
    /// Norm of horizontal ground speed.
    /// Unit: m/s
    pub h_speed_norm_f: f32,
    /// Direction of horizontal ground speed.
    /// Unit: rad (clockwise, zero=north)
    pub h_speed_dir_f: f32,
    /// @addtogroup state_acceleration
    ///  @{ */
    ////**
    /// Holds the status bits for all acceleration representations.
    /// When the corresponding bit is one the representation
    /// is already computed.
    pub accel_status: u8,
    /// Acceleration in North East Down coordinates.
    /// Units: m/s^2 in BFP with #INT32_ACCEL_FRAC
    pub body_accel_i: Int32Vect3,
    /// Acceleration in North East Down coordinates.
    /// Units: m/s^2 in BFP with #INT32_ACCEL_FRAC
    pub ned_accel_i: NedCoor_i,
    /// Acceleration in EarthCenteredEarthFixed coordinates.
    /// Units: m/s^2 in BFP with INT32_ACCEL_FRAC
    pub ecef_accel_i: EcefCoor_i,
    /// Acceleration in North East Down coordinates.
    /// Units: m/s^2
    pub ned_accel_f: NedCoor_f,
    /// Acceleration in EarthCenteredEarthFixed coordinates.
    /// Units: m/s^2
    pub ecef_accel_f: EcefCoor_f,
    /// @defgroup state_attitude Attitude representations
    pub ned_to_body_orientation: OrientationReps,
    /// @addtogroup state_rate
    ///  @{ */
    ////**
    /// Holds the status bits for all angular rate representations.
    /// When the corresponding bit is one the representation
    /// is already computed.
    pub rate_status: u8,
    /// Angular rates in body frame.
    /// Units: rad/s in BFP with #INT32_RATE_FRAC
    pub body_rates_i: Int32Rates,
    /// Angular rates in body frame.
    /// Units: rad/s
    pub body_rates_f: FloatRates,
    /// @addtogroup state_wind_airspeed
    ///  @{ */
    ////**
    /// Holds the status bits for all wind- and airspeed representations.
    /// When the corresponding bit is one the representation
    /// is already computed.
    pub wind_air_status: u8,
    /// Norm of relative wind speed.
    /// Unit: m/s in BFP with #INT32_SPEED_FRAC
    pub airspeed_i: i32,
    /// Norm of relative air speed.
    /// Unit: m/s
    pub airspeed_f: f32,
    /// Angle of attack
    /// Unit: rad
    pub angle_of_attack_f: f32,
    /// Sideslip angle
    /// Unit: rad
    pub sideslip_f: f32,
}

extern "C" {
    #[link_name = "\u{1}state"]
    pub static mut state: State;
}

