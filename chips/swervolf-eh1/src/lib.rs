// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Drivers and chip support for SweRVolf.

#![feature(naked_functions)]
#![no_std]
#![crate_name = "swervolf_eh1"]
#![crate_type = "rlib"]

pub mod chip;
pub mod syscon;
pub mod uart;
