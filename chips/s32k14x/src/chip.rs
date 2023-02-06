//! Chip trait setup.
use core::fmt::Write;
use cortexm4::{self, CortexM4, CortexMVariant};
use kernel::deferred_call;
use kernel::platform::chip::Chip;
use kernel::platform::chip::InterruptService;

use crate::deferred_calls::DeferredCallTask;

pub struct S32k14x<'a, I: InterruptService<DeferredCallTask> + 'a> {
    mpu: cortexm4::mpu::MPU,
    userspace_kernel_boundary: cortexm4::syscall::SysCall,
    interrupt_service: &'a I,
}

// pub struct S32k14xDefaultPeripherals<'a> {
//     x: *const T,
//     phantom: PhantomData<&'a T>
// }

// impl<'a> S32k14xDefaultPeripherals<'a> {
//     pub fn new(
//         // fixme: 
//     ) -> Self {
//         Self {
//             // Fixme: 
//         }
//     }

//     pub fn setup_circular_deps(&'a self) {
//         // self.gpio_ports.setup_circular_deps();
//     }
// }

// impl<'a> InterruptService<DeferredCallTask> for S32k14xDefaultPeripherals<'a> {
//     unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
//         true
//     }

//     unsafe fn service_deferred_call(&self, task: DeferredCallTask) -> bool {
//         true
//     }
// }

impl<'a, I: InterruptService<DeferredCallTask> + 'a> S32k14x<'a, I> {
    pub unsafe fn new(interrupt_service: &'a I) -> Self {
        Self {
            mpu: cortexm4::mpu::MPU::new(),
            userspace_kernel_boundary: cortexm4::syscall::SysCall::new(),
            interrupt_service,
        }
    }
}

impl<'a, I: InterruptService<DeferredCallTask> + 'a> Chip for S32k14x<'a, I> {
    type MPU = cortexm4::mpu::MPU;
    type UserspaceKernelBoundary = cortexm4::syscall::SysCall;

    fn service_pending_interrupts(&self) {
        unsafe {
            loop {
                if let Some(task) = deferred_call::DeferredCall::next_pending() {
                    if !self.interrupt_service.service_deferred_call(task) {
                        panic!("Unhandled deferred call");
                    }
                } else if let Some(interrupt) = cortexm4::nvic::next_pending() {
                    if !self.interrupt_service.service_interrupt(interrupt) {
                        panic!("unhandled interrupt {}", interrupt);
                    }

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
        unsafe { cortexm4::nvic::has_pending() || deferred_call::has_tasks() }
    }

    fn mpu(&self) -> &cortexm4::mpu::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &cortexm4::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn sleep(&self) {
        unsafe {
            cortexm4::scb::unset_sleepdeep();
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
