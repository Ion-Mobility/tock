//! Peripheral implementations for the s32k14x MCU.
//!
//! STM32F446RE: <https://www.st.com/en/microcontrollers/stm32f4.html>

#![crate_name = "s32k14x"]
#![crate_type = "rlib"]
#![no_std]

pub mod chip;
pub mod nvic;
pub mod pcc;
pub mod dma;
pub mod gpio;
pub mod wdt;

// Peripherals
// pub mod deferred_calls;
//pub mod gpio;

use cortexm4::{initialize_ram_jump_to_main, unhandled_interrupt, CortexM4, CortexMVariant};

extern "C" {
    // _estack is not really a function, but it makes the types work
    // You should never actually invoke it!!
    fn _estack();
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static BASE_VECTORS: [unsafe extern "C" fn(); 16] = [
    _estack,
    initialize_ram_jump_to_main,
    unhandled_interrupt,          // NMI
    CortexM4::HARD_FAULT_HANDLER, // Hard Fault
    unhandled_interrupt,          // MemManage
    unhandled_interrupt,          // BusFault
    unhandled_interrupt,          // UsageFault
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    CortexM4::SVC_HANDLER, // SVC
    unhandled_interrupt,   // DebugMon
    unhandled_interrupt,
    unhandled_interrupt,       // PendSV
    CortexM4::SYSTICK_HANDLER, // SysTick
];

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static IRQS: [unsafe extern "C" fn(); 89] = [
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
    CortexM4::GENERIC_ISR,
];

pub unsafe fn init() {
    cortexm4::nvic::disable_all();
    cortexm4::nvic::clear_all_pending();
    cortexm4::nvic::enable_all();
}
