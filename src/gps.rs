use std::f64::consts::PI;

use thiserror::Error;

/// The radius of the Earth in meters, roughly
const EARTH_RADIUS_METERS: u32 = 6_378_137;
/// Multiply radians by this to get degrees
const RAD_TO_DEG: f64 = 180.0 / PI;
/// Multiply degrees by this to get radians
const DEG_TO_RAD: f64 = PI / 180.0;

/// A single point on the Earth, including altitude.
#[derive(Debug, Clone, Copy)]
pub struct GPSPoint {
    /// Latitude in decimal degrees
    latitude: f64,
    /// Longitude in decimal degrees
    longitude: f64,
    /// Altitude in decimal meters
    altitude: Option<f64>,
}

#[derive(Debug, Error)]
pub enum GPSError {
    #[error("The operation could not be completed because the altitude field is None")]
    NoAltitude,
    #[error("The value ({0}) is not in the range {1} to {2}")]
    OutOfRange(f64, f64, f64),
}

impl GPSPoint {
    pub fn new(latitude: f64, longitude: f64, altitude: Option<f64>) -> Self {
        Self { latitude, longitude, altitude }
    }

    /// Returns the latitude component in radians.
    pub const fn lat_rad(&self) -> f64 {
        self.latitude * DEG_TO_RAD
    }

    /// Returns the longitude component in radians.
    pub const fn lon_rad(&self) -> f64 {
        self.longitude * DEG_TO_RAD
    }

    /// Great-circle ground-only distance in meters between two GPS Points.
    pub fn distance_to(self, other: &Self) -> f64 {
        let delta_lat_rad = other.lat_rad() - self.lat_rad();
        let delta_lon_rad = other.lon_rad() - self.lon_rad();

        let a =
        f64::sin(delta_lat_rad / 2.0).powi(2) + f64::cos(self.lat_rad())
        * f64::cos(other.lat_rad()) * f64::sin(delta_lon_rad / 2.0).powi(2);

        let c = 2.0 * f64::atan2(f64::sqrt(a), f64::sqrt(1.0 - a));

        EARTH_RADIUS_METERS as f64 * c
    }

    pub fn altitude_to(self, other: &Self) -> Option<f64> {
        if self.altitude.is_some() && other.altitude.is_some() {
            Some(other.altitude.unwrap() - self.altitude.unwrap())
        } else {
            None
        }
    }

    /// Find the absolute bearing (azimuth) to another point.
    pub fn bearing_to(self, other: &Self, positive: bool) -> f64 {
        // Calculate the bearing
        let bearing = f64::atan2(
            f64::sin(other.lon_rad() - self.lon_rad()) * f64::cos(other.lat_rad()),
                                 f64::cos(self.lat_rad()) * f64::sin(other.lat_rad()) - f64::sin(self.lat_rad())
                                 * f64::cos(other.lat_rad()) * f64::cos(other.lon_rad() - self.lon_rad())
        );

        // Convert the bearing to degrees
        let mut bearing = bearing * RAD_TO_DEG;

        // Ensure the value is from 0→360 instead of -180→180
        if positive {
            bearing = (bearing + 360.0) % 360.0
        }

        bearing
    }

    /// Find the elevation above the horizon (aka altitude, zenith) to another point.
    ///
    /// The result will never be greater than 90° or less than -90°.
    pub fn elevation_to(self, other: &Self) -> Result<f64, GPSError> {
        // Distance in meters, and horizontal angle (azimuth)
        let horizontal_distance = self.distance_to(other);

        // Altitude difference in meters
        let altitude_delta = self.altitude_to(other)
            .ok_or(GPSError::NoAltitude)?;

        // In this case things would divide by zero, so it must be directly
        // above or below, so bail out here and return
        if horizontal_distance == 0.0 {
            match altitude_delta {
                a if a > 0.0 => return Ok(90.0),
                a if a < 0.0 => return Ok(-90.0),
                _ => return Ok(0.0),
            }
        }

        let final_angle = f64::atan(altitude_delta / horizontal_distance) * RAD_TO_DEG;

        if (-90.0..90.0).contains(&final_angle) {
            return Err(GPSError::OutOfRange(final_angle, -90.0, 90.0))
        }

        // Vertical angle (altitude)
        Ok(final_angle)
    }
}
