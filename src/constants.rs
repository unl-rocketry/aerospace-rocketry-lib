//! Generally applicable constant values.

use core::f64::consts::PI;

/// Multiply radians by this to get degrees
pub const RAD_TO_DEG: f64 = 180.0 / PI;
/// Multiply degrees by this to get radians
pub const DEG_TO_RAD: f64 = PI / 180.0;

/// The radius of the Earth in meters, roughly
pub const EARTH_RADIUS_METERS: u32 = 6_378_137;
