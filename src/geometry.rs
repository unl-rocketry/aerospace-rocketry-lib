//! Utilities for working with geometric constructions.
//!
//! Generally the things in this module apply to the [`crate::gps`] module as
//! well, but this one is not designed to be geospatial-specific.

/// Find the centroid of a polygon using its verticies.
pub fn polygon_centroid(pts: &[(f64, f64)]) -> (f64, f64) {
    let x = pts.iter().fold(0.0, |acc, x| acc + x.0) / pts.len() as f64;
    let y = pts.iter().fold(0.0, |acc, y| acc + y.1) / pts.len() as f64;

    (x, y)
}

