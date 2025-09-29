use core::ops::{Add, Deref};

use thiserror::Error;

use crate::constants::{DEG_TO_RAD, RAD_TO_DEG};

/// The radius of the Earth in meters, roughly
pub const EARTH_RADIUS_METERS: u32 = 6_378_137;

#[derive(Debug, Error)]
pub enum GPSError {
    #[error("The operation could not be completed because the altitude field is None")]
    NoAltitude,
    #[error("The value ({0}) is not in the range {1} to {2}")]
    OutOfRange(f64, f64, f64),
}

#[derive(Debug, Clone, Copy)]
pub struct Bearing(f64);

impl Bearing {
    pub fn new(degrees: f64) -> Result<Self, GPSError> {
        if !(0.0..360.0).contains(&degrees) {
            return Err(GPSError::OutOfRange(degrees, 0.0, 360.0))
        }

        Ok(Self(degrees))
    }

    pub fn new_rad(radians: f64) -> Result<Self, GPSError> {
        let degrees = radians * RAD_TO_DEG;

        Self::new(degrees)
    }
}

impl Deref for Bearing {
    type Target = f64;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// A single point on the Earth, including altitude.
#[derive(Debug, Clone, Copy)]
pub struct Point {
    /// Latitude in decimal degrees
    latitude: f64,
    /// Longitude in decimal degrees
    longitude: f64,
    /// Altitude in meters
    altitude: Option<f64>,
}

impl Point {
    pub fn new_3d(latitude: f64, longitude: f64, altitude: f64) -> Result<Self, GPSError> {
        if !(-90.0..90.0).contains(&latitude) {
            return Err(GPSError::OutOfRange(latitude, -90.0, 90.0))
        }

        if !(-180.0..180.0).contains(&longitude) {
            return Err(GPSError::OutOfRange(longitude, -180.0, 180.0))
        }

        Ok(Self {
            latitude,
            longitude,
            altitude: Some(altitude),
        })
    }

    pub fn new_2d(latitude: f64, longitude: f64) -> Result<Self, GPSError> {
        if !(-90.0..90.0).contains(&latitude) {
            return Err(GPSError::OutOfRange(latitude, -90.0, 90.0))
        }

        if !(-180.0..180.0).contains(&longitude) {
            return Err(GPSError::OutOfRange(longitude, -180.0, 180.0))
        }

        Ok(Self {
            latitude,
            longitude,
            altitude: None,
        })
    }

    pub const fn latitude(&self) -> f64 {
        self.latitude
    }

    pub const fn longitude(&self) -> f64 {
        self.longitude
    }

    pub const fn altitude(&self) -> Option<f64> {
        self.altitude
    }

    /// Returns the latitude component in radians.
    pub const fn latitude_rad(&self) -> f64 {
        self.latitude * DEG_TO_RAD
    }

    /// Returns the longitude component in radians.
    pub const fn longitude_rad(&self) -> f64 {
        self.longitude * DEG_TO_RAD
    }

    /// Great-circle ground-only distance in meters between two GPS Points.
    pub fn distance_to(self, other: Self) -> f64 {
        let delta_lat_rad = other.latitude_rad() - self.latitude_rad();
        let delta_lon_rad = other.longitude_rad() - self.longitude_rad();

        let a = f64::sin(delta_lat_rad / 2.0).powi(2)
            + f64::cos(self.latitude_rad())
                * f64::cos(other.latitude_rad())
                * f64::sin(delta_lon_rad / 2.0).powi(2);

        let c = 2.0 * f64::atan2(f64::sqrt(a), f64::sqrt(1.0 - a));

        EARTH_RADIUS_METERS as f64 * c
    }

    pub fn altitude_to(self, other: Self) -> Option<f64> {
        if self.altitude.is_some() && other.altitude.is_some() {
            Some(other.altitude.unwrap() - self.altitude.unwrap())
        } else {
            None
        }
    }

    /// Find the absolute bearing (azimuth) to another point.
    pub fn bearing_to(self, other: Self, positive: bool) -> Bearing {
        // Calculate the bearing
        let bearing = f64::atan2(
            f64::sin(other.longitude_rad() - self.longitude_rad()) * f64::cos(other.latitude_rad()),
            f64::cos(self.latitude_rad()) * f64::sin(other.latitude_rad())
                - f64::sin(self.latitude_rad())
                    * f64::cos(other.latitude_rad())
                    * f64::cos(other.longitude_rad() - self.longitude_rad()),
        );

        // Convert the bearing to degrees
        let mut bearing = bearing * RAD_TO_DEG;

        // Ensure the value is from 0→360 instead of -180→180
        if positive {
            bearing = (bearing + 360.0) % 360.0
        }

        Bearing(bearing)
    }

    /// Find the elevation above the horizon (aka altitude, zenith) to another point.
    ///
    /// The result will never be greater than 90° or less than -90°.
    pub fn elevation_to(self, other: Self) -> Result<f64, GPSError> {
        // Distance in meters, and horizontal angle (azimuth)
        let horizontal_distance = self.distance_to(other);

        // Altitude difference in meters
        let altitude_delta = self.altitude_to(other).ok_or(GPSError::NoAltitude)?;

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
            return Err(GPSError::OutOfRange(final_angle, -90.0, 90.0));
        }

        // Vertical angle (altitude)
        Ok(final_angle)
    }
}

impl Add<Self> for Point {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut new_self = self;

        new_self.latitude += rhs.latitude;
        new_self.longitude += rhs.longitude;
        new_self.altitude = new_self.altitude.map(|a| a + rhs.altitude.unwrap_or(0.0));

        new_self
    }
}

impl Add<(f64, f64)> for Point {
    type Output = Self;

    fn add(self, rhs: (f64, f64)) -> Self::Output {
        let mut new_self = self;

        new_self.latitude += rhs.0;
        new_self.longitude += rhs.1;

        new_self
    }
}

impl Add<(f64, f64, f64)> for Point {
    type Output = Self;

    fn add(self, rhs: (f64, f64, f64)) -> Self::Output {
        let mut new_self = self;

        new_self.latitude += rhs.0;
        new_self.longitude += rhs.1;
        new_self.altitude = new_self.altitude.map(|a| a + rhs.2);

        new_self
    }
}

/// Find the Latitude-Longitude intersection point between two rays from points with bearings.
pub fn bearing_intersection(point_1: (Point, Bearing), point_2: (Point, Bearing)) -> Result<Point, GPSError> {
    let point_1a = point_1.0;
    let point_2a = point_2.0;

    let point_1b = offset_point_bearing(point_1a, point_1.1, 10.0)?;
    let point_2b = offset_point_bearing(point_2a, point_2.1, 10.0)?;

    // These equations are based on data from this article:
    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    let intersection_x =
        (
            ((point_1a.longitude() * point_1b.latitude() - point_1a.latitude() * point_1b.longitude()) * (point_2a.longitude() - point_2b.longitude()))
            -
            ((point_2a.longitude() * point_2b.latitude() - point_2a.latitude() * point_2b.longitude()) * (point_1a.longitude() - point_1b.longitude()))
        )
        /
        (
            ((point_1a.longitude() - point_1b.longitude()) * (point_2a.latitude() - point_2b.latitude()))
            -
            ((point_1a.latitude() - point_1b.latitude()) * (point_2a.longitude() - point_2b.longitude()))
        );

    let intersection_y =
        (
            ((point_1a.longitude() * point_1b.latitude() - point_1a.latitude() * point_1b.longitude()) * (point_2a.latitude() - point_2b.latitude()))
            -
            ((point_2a.longitude() * point_2b.latitude() - point_2a.latitude() * point_2b.longitude()) * (point_1a.latitude() - point_1b.latitude()))
        )
        /
        (
            ((point_1a.longitude() - point_1b.longitude()) * (point_2a.latitude() - point_2b.latitude()))
            -
            ((point_1a.latitude() - point_1b.latitude()) * (point_2a.longitude() - point_2b.longitude()))
        );

    Point::new_2d(intersection_y, intersection_x)
}

pub fn offset_point_bearing(point: Point, bearing: Bearing, distance: f64) -> Result<Point, GPSError> {
    let bearing_rad = bearing.0 * DEG_TO_RAD;

    // Starting lat/lon in radians
    let lat1 = point.latitude_rad();
    let lon1 = point.longitude_rad();

    // Forward geodesic on a sphere
    let sin_lat1 = lat1.sin();
    let cos_lat1 = lat1.cos();
    let ang_dist = distance / EARTH_RADIUS_METERS as f64;

    let sin_ad = ang_dist.sin();
    let cos_ad = ang_dist.cos();

    let lat2 = (sin_lat1 * cos_ad + cos_lat1 * sin_ad * bearing_rad.cos()).asin();

    let y = bearing_rad.sin() * sin_ad * cos_lat1;
    let x = cos_ad - sin_lat1 * lat2.sin();
    let lon2 = lon1 + y.atan2(x);

    // Convert back to degrees
    let lat2_deg = lat2 * RAD_TO_DEG;
    let lon2_deg = lon2 * RAD_TO_DEG;

    Point::new_2d(lat2_deg, lon2_deg)
}
