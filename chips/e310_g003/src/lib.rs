// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Chip support for the E310-G003 from SiFive.

#![no_std]
#![crate_name = "e310_g003"]
#![crate_type = "rlib"]

pub use e310x::{chip, clint, deferred_call_tasks, gpio, plic, prci, pwm, rtc, uart, watchdog};

pub mod interrupt_service;
mod interrupts;
