//! Peripheral implementations for the s32k14x MCU.
//!
//! STM32F446RE: <https://www.st.com/en/microcontrollers/stm32f4.html>

#![crate_name = "s32k14x"]
#![crate_type = "rlib"]
#![no_std]

pub mod chip;
pub mod nvic;

// Peripherals
pub mod deferred_calls;
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
pub static IRQS: [unsafe extern "C" fn(); 122] = [
    CortexM4::GENERIC_ISR, /**< DMA channel 0 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 1 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 2 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 3 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 4 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 5 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 6 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 7 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 8 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 9 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 10 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 11 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 12 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 13 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 14 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA channel 15 transfer complete */
    CortexM4::GENERIC_ISR, /**< DMA error interrupt channels 0-15 */
    CortexM4::GENERIC_ISR, /**< FPU sources */
    CortexM4::GENERIC_ISR, /**< FTFC Command complete */
    CortexM4::GENERIC_ISR, /**< FTFC Read collision */
    CortexM4::GENERIC_ISR, /**< PMC Low voltage detect interrupt */
    CortexM4::GENERIC_ISR, /**< FTFC Double bit fault detect */
    CortexM4::GENERIC_ISR, /**< Single interrupt vector for WDOG and EWM */
    CortexM4::GENERIC_ISR, /**< RCM Asynchronous Interrupt */
    CortexM4::GENERIC_ISR, /**< LPI2C0 Master Interrupt */
    CortexM4::GENERIC_ISR, /**< LPI2C0 Slave Interrupt */
    CortexM4::GENERIC_ISR, /**< LPSPI0 Interrupt */
    CortexM4::GENERIC_ISR, /**< LPSPI1 Interrupt */
    CortexM4::GENERIC_ISR, /**< LPSPI2 Interrupt */
    CortexM4::GENERIC_ISR, /**< LPUART0 Transmit / Receive Interrupt */
    CortexM4::GENERIC_ISR, /**< LPUART1 Transmit / Receive  Interrupt */
    CortexM4::GENERIC_ISR, /**< LPUART2 Transmit / Receive  Interrupt */
    CortexM4::GENERIC_ISR, /**< ADC0 interrupt request. */
    CortexM4::GENERIC_ISR, /**< ADC1 interrupt request. */
    CortexM4::GENERIC_ISR, /**< CMP0 interrupt request */
    CortexM4::GENERIC_ISR, /**< ERM single bit error correction */
    CortexM4::GENERIC_ISR, /**< ERM double bit error non-correctable */
    CortexM4::GENERIC_ISR, /**< RTC alarm interrupt */
    CortexM4::GENERIC_ISR, /**< RTC seconds interrupt */
    CortexM4::GENERIC_ISR, /**< LPIT0 channel 0 overflow interrupt */
    CortexM4::GENERIC_ISR, /**< LPIT0 channel 1 overflow interrupt */
    CortexM4::GENERIC_ISR, /**< LPIT0 channel 2 overflow interrupt */
    CortexM4::GENERIC_ISR, /**< LPIT0 channel 3 overflow interrupt */
    CortexM4::GENERIC_ISR, /**< PDB0 interrupt */
    CortexM4::GENERIC_ISR, /**< SCG bus interrupt request */
    CortexM4::GENERIC_ISR, /**< LPTIMER interrupt request */
    CortexM4::GENERIC_ISR, /**< Port A pin detect interrupt */
    CortexM4::GENERIC_ISR, /**< Port B pin detect interrupt */
    CortexM4::GENERIC_ISR, /**< Port C pin detect interrupt */
    CortexM4::GENERIC_ISR, /**< Port D pin detect interrupt */
    CortexM4::GENERIC_ISR, /**< Port E pin detect interrupt */
    CortexM4::GENERIC_ISR, /**< Software interrupt */
    CortexM4::GENERIC_ISR, /**< PDB1 interrupt */
    CortexM4::GENERIC_ISR, /**< FlexIO Interrupt */
    CortexM4::GENERIC_ISR, /**< CAN0 OR'ed [Bus Off OR Transmit Warning OR Receive Warning] */
    CortexM4::GENERIC_ISR, /**< CAN0 Interrupt indicating that errors were detected on the CAN bus */
    CortexM4::GENERIC_ISR, /**< CAN0 Interrupt asserted when Pretended Networking operation is enabled, and a valid message matches the selected filter criteria during Low Power mode */
    CortexM4::GENERIC_ISR, /**< CAN0 OR'ed Message buffer (0-15) */
    CortexM4::GENERIC_ISR, /**< CAN0 OR'ed Message buffer (16-31) */
    CortexM4::GENERIC_ISR, /**< CAN1 OR'ed [Bus Off OR Transmit Warning OR Receive Warning] */
    CortexM4::GENERIC_ISR, /**< CAN1 Interrupt indicating that errors were detected on the CAN bus */
    CortexM4::GENERIC_ISR, /**< CAN1 OR'ed Interrupt for Message buffer (0-15) */
    CortexM4::GENERIC_ISR, /**< CAN2 OR'ed [Bus Off OR Transmit Warning OR Receive Warning] */
    CortexM4::GENERIC_ISR, /**< CAN2 Interrupt indicating that errors were detected on the CAN bus */
    CortexM4::GENERIC_ISR, /**< CAN2 OR'ed Message buffer (0-15) */
    CortexM4::GENERIC_ISR, /**< FTM0 Channel 0 and 1 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM0 Channel 2 and 3 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM0 Channel 4 and 5 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM0 Channel 6 and 7 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM0 Fault interrupt */
    CortexM4::GENERIC_ISR, /**< FTM0 Counter overflow and Reload interrupt */
    CortexM4::GENERIC_ISR, /**< FTM1 Channel 0 and 1 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM1 Channel 2 and 3 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM1 Channel 4 and 5 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM1 Channel 6 and 7 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM1 Fault interrupt */
    CortexM4::GENERIC_ISR, /**< FTM1 Counter overflow and Reload interrupt */
    CortexM4::GENERIC_ISR, /**< FTM2 Channel 0 and 1 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM2 Channel 2 and 3 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM2 Channel 4 and 5 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM2 Channel 6 and 7 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM2 Fault interrupt */
    CortexM4::GENERIC_ISR, /**< FTM2 Counter overflow and Reload interrupt */
    CortexM4::GENERIC_ISR, /**< FTM3 Channel 0 and 1 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM3 Channel 2 and 3 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM3 Channel 4 and 5 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM3 Channel 6 and 7 interrupt */
    CortexM4::GENERIC_ISR, /**< FTM3 Fault interrupt */
    CortexM4::GENERIC_ISR, /**< FTM3 Counter overflow and Reload interrupt */
];

pub unsafe fn init() {
    cortexm4::nvic::disable_all();
    cortexm4::nvic::clear_all_pending();
    cortexm4::nvic::enable_all();
}
