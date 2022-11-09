// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Drivers and chip support for the E21 soft core.

#![no_std]
#![crate_name = "arty_e21_chip"]
#![crate_type = "rlib"]

mod interrupts;

pub mod chip;
pub mod clint;
pub mod deferred_call_tasks;
pub mod gpio;
pub mod uart;
