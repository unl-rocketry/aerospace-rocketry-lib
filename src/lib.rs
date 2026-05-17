//! # UNL Aerospace Rocketry Catch-all Library
//!
//! This is a library for UNL Aerospace Rocketry which contains many useful
//! things we happen to use all the time. Each module is well documented, so
//! check out the top-level module information to learn how to use the types
//! within.
//!
//! ## Units
//! Unless stated otherwise, all the functions in this library utilize the
//! following units for their input and output. For example,
//! [`geospatial::Point::latitude`] returns **decimal degrees**, while
//! [`geospatial::Point::latitude_rad`] returns **radians**. The internal types of
//! the [`geospatial::Point`] type are expressed in the units below.
//!
//! | Measurement | Unit            |
//! |-------------|-----------------|
//! | Coordinates | Decimal Degrees |
//! | Angles      | Decimal Degrees |
//! | Distance    | Meters          |
//! | Temperature | Celsius         |
//! | Time        | Seconds         |
//!
//! ## `no_std`
//! This crate is `no_std` compatible to allow for running on microcontrollers
//! and other devices which don't support `std`. Functions may be added in the
//! future which require `std` support, and a feature flag will be added to
//! allow for disabling these functions if not required.
//!
//! ## Tests
//! This crate contains tests for many of the functions. The tests require the
//! `std` library, and it is automatically enabled for testing.

#![no_std]

// Enable `std` for tests
#[cfg(test)]
#[macro_use]
extern crate std;

pub mod constants;
pub mod geospatial;
pub mod geometry;
pub mod utils;
