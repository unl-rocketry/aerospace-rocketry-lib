# UNL Aerospace Rocketry Catch-all Library
![Test Status](https://img.shields.io/github/actions/workflow/status/unl-rocketry/aerospace-rocketry-lib/test.yml?style=for-the-badge&label=Tests)
![Test Status](https://img.shields.io/github/actions/workflow/status/unl-rocketry/aerospace-rocketry-lib/clippy.yml?style=for-the-badge&label=Clippy)
![Static Badge](https://img.shields.io/badge/no__std-579af7?style=for-the-badge&logo=Rust&logoColor=000000)



This is a library for UNL Aerospace Rocketry which contains many useful
things we happen to use all the time. Each module is well documented, so
check out the top-level module information to learn how to use the types
within.

## Units
Unless stated otherwise, all the functions in this library utilize the
following units for their input and output.

| Measurement | Unit            |
|-------------|-----------------|
| Coordinates | Decimal Degrees |
| Angles      | Decimal Degrees |
| Distance    | Meters          |
| Temperature | Celsius         |
| Time        | Seconds         |

Examples of a type which has default values is `geospatial::Point::latitude` which returns **decimal degrees**, while
`geospatial::Point::latitude_rad` has nonstandard values and returns **radians**.

## `no_std`
This crate is `no_std` compatible to allow for running on microcontrollers
and other devices which don't support `std`. Functions may be added in the
future which require `std` support, and a feature flag will be added to
allow for disabling these functions if not required.

## Tests
This crate contains tests for many of the functions. The tests require the
`std` library, and it is automatically enabled for testing.
