// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! RISC-V CSR Library
//!
//! Uses the Tock Register Interface to control RISC-V CSRs.

#![feature(asm_const)]
#![no_std]

pub mod csr;
