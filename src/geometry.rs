//! Utilities for working with geometric constructions.
//!
//! Generally the things in this module apply to the [`crate::geospatial`] module as
//! well, but this one is not designed to be geospatial-specific.

/// Find the centroid of a polygon using its verticies.
///
/// Returns an ordered pair corresponding to the position of the centroid in
/// the plane. There is no guarantee that the centroid is contained within the
/// polygon.
#[must_use]
pub fn polygon_centroid(pts: &[(f64, f64)]) -> (f64, f64) {
    let x = pts.iter().fold(0.0, |acc, x| acc + x.0) / pts.len() as f64;
    let y = pts.iter().fold(0.0, |acc, y| acc + y.1) / pts.len() as f64;

    (x, y)
}

