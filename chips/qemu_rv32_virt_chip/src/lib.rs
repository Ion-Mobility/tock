// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Chip support for the qemu-system-riscv32 virt machine

#![no_std]
#![crate_name = "qemu_rv32_virt_chip"]
#![crate_type = "rlib"]

mod interrupts;

pub mod chip;
pub mod clint;
pub mod plic;
pub mod uart;
