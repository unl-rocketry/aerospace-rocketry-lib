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
//! [`gps::Point::latitude`] returns **decimal degrees**, while
//! [`gps::Point::latitude_rad`] returns **radians**. The internal types of
//! the [`gps::Point`] type are expressed in the units below.
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
//! This library is `no_std` compatible to allow for running on microcontrollers
//! and other devices which don't support `std`. Functions may be added in the
//! future which require `std` support, and a feature flag will be added to
//! allow for disabling these functions if not required.
//!

#![no_std]

pub mod constants;
pub mod gps;
pub mod geometry;
