//! Chip trait setup.
use core::fmt::Write;
use cortexm4::{self, CortexM4, CortexMVariant};
use kernel::deferred_call;
use kernel::platform::chip::Chip;
use kernel::platform::chip::InterruptService;

use crate::nvic;

pub struct S32k14x<I: InterruptService<()> + 'static> {
    mpu: cortexm4::mpu::MPU,
    userspace_kernel_boundary: cortexm4::syscall::SysCall,
    interrupt_service: &'static I,
}

impl<I: InterruptService<()> + 'static> S32k14x<I> {
    pub unsafe fn new(interrupt_service: &'static I) -> Self {
        S32k14x {
            mpu: cortexm4::mpu::MPU::new(),
            userspace_kernel_boundary: cortexm4::syscall::SysCall::new(),
            interrupt_service,
        }
    }
}

pub struct S32k14xDefaultPeripherals {
    pub pcc: &'static crate::pcc::Pcc,
    pub dma: crate::dma::Dma<'static>,
}

impl S32k14xDefaultPeripherals {
    pub fn new(pcc: &'static crate::pcc::Pcc) -> Self {
        Self {
            pcc,
            dma: crate::dma::Dma::new(pcc),
        }
    }
}

impl InterruptService<()> for S32k14xDefaultPeripherals {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            // nvic::LPUART1 => self.lpuart1.handle_interrupt(),
            // nvic::LPUART2 => self.lpuart2.handle_interrupt(),
            // nvic::LPI2C1 => self.lpi2c1.handle_event(),
            // nvic::GPT1 => self.gpt1.handle_interrupt(),
            // nvic::GPT2 => self.gpt2.handle_interrupt(),
            // nvic::GPIO1_1 => self.ports.gpio1.handle_interrupt(),
            // nvic::GPIO1_2 => self.ports.gpio1.handle_interrupt(),
            // nvic::GPIO2_1 => self.ports.gpio2.handle_interrupt(),
            // nvic::GPIO2_2 => self.ports.gpio2.handle_interrupt(),
            // nvic::GPIO3_1 => self.ports.gpio3.handle_interrupt(),
            // nvic::GPIO3_2 => self.ports.gpio3.handle_interrupt(),
            // nvic::GPIO4_1 => self.ports.gpio4.handle_interrupt(),
            // nvic::GPIO4_2 => self.ports.gpio4.handle_interrupt(),
            // nvic::GPIO5_1 => self.ports.gpio5.handle_interrupt(),
            // nvic::GPIO5_2 => self.ports.gpio5.handle_interrupt(),
            // nvic::SNVS_LP_WRAPPER => debug!("Interrupt: SNVS_LP_WRAPPER"),
            // nvic::DMA0_16..=nvic::DMA15_31 => {
            //     let low = (interrupt - nvic::DMA0_16) as usize;
            //     let high = low + 16;
            //     for channel in [&self.dma.channels[low], &self.dma.channels[high]] {
            //         if channel.is_interrupt() | channel.is_error() {
            //             channel.handle_interrupt();
            //         }
            //     }
            // }
            nvic::DMA_ERROR_IRQN => {
                while let Some(channel) = self.dma.error_channel() {
                    channel.handle_interrupt();
                }
            }
            _ => {
                return false;
            }
        }
        true
    }

    unsafe fn service_deferred_call(&self, _: ()) -> bool {
        false
    }
}

impl<I: InterruptService<()> + 'static> Chip for S32k14x<I> {
    type MPU = cortexm4::mpu::MPU;
    type UserspaceKernelBoundary = cortexm4::syscall::SysCall;

    fn service_pending_interrupts(&self) {
        unsafe {
            loop {
                if let Some(interrupt) = cortexm4::nvic::next_pending() {
                    let handled = self.interrupt_service.service_interrupt(interrupt);
                    assert!(handled, "Unhandled interrupt number {}", interrupt);
                    let n = cortexm4::nvic::Nvic::new(interrupt);
                    n.clear_pending();
                    n.enable();
                } else {
                    break;
                }
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { cortexm4::nvic::has_pending() }
    }

    fn mpu(&self) -> &cortexm4::mpu::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &cortexm4::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn sleep(&self) {
        unsafe {
            cortexm4::support::wfi();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm4::support::atomic(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        CortexM4::print_cortexm_state(write);
    }
}
