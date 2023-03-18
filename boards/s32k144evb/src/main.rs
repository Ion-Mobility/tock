//! Board file for STM32F429I Discovery development board
//!
//! - <https://www.st.com/en/evaluation-tools/32f429idiscovery.html>

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use capsules::virtual_alarm::VirtualMuxAlarm;
use components::gpio::GpioComponent;
use kernel::capabilities;
use kernel::component::Component;
use kernel::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::hil;
use kernel::hil::led::LedHigh;
use kernel::hil::led::LedLow;
use kernel::hil::Controller;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{create_capability, debug, static_init};
use s32k14x::chip::S32k14xDefaultPeripherals;
use kernel::platform::watchdog::WatchDog;
/// Support routines for debugging I/O.
pub mod io;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

// Actual memory for holding the active process structures.
static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None, None, None, None];

static mut CHIP: Option<&'static s32k14x::chip::S32k14x<S32k14xDefaultPeripherals>> = None;
static mut PROCESS_PRINTER: Option<&'static kernel::process::ProcessPrinterText> = None;

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

/// Supported drivers by the platform
pub struct Platform {
    gpio: &'static capsules::gpio::GPIO<'static, s32k14x::gpio::GPIOPin<'static>>,
    ipc: kernel::ipc::IPC<{ NUM_PROCS as u8 }>,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}
impl KernelResources<s32k14x::chip::S32k14x<S32k14xDefaultPeripherals>> for Platform {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type CredentialsCheckingPolicy = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm4::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        &self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn credentials_checking_policy(&self) -> &'static Self::CredentialsCheckingPolicy {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// Helper function called during bring-up that configures multiplexed I/O.
unsafe fn set_pin_primary_functions(peripherals: &S32k14xDefaultPeripherals) {
    use s32k14x::gpio::PeripheralFunction::{A, B};

    peripherals.pa[00].configure(Some(A)); // A0 - ADC0
    peripherals.pa[01].configure(Some(A)); // A1 - ADC1
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
unsafe fn get_peripherals(pcc: &'static s32k14x::pcc::Pcc) -> &'static S32k14xDefaultPeripherals {
    static_init!(
        S32k14xDefaultPeripherals,
        S32k14xDefaultPeripherals::new(pcc)
    )
}

// impl KernelResources<nrf52832::chip::NRF52<'static, Nrf52832DefaultPeripherals<'static>>>
//     for Platform
// {
//     type SyscallDriverLookup = Self;
//     type SyscallFilter = ();
//     type ProcessFault = ();
//     type CredentialsCheckingPolicy = ();
//     type Scheduler = RoundRobinSched<'static>;
//     type SchedulerTimer = cortexm4::systick::SysTick;
//     type WatchDog = ();
//     type ContextSwitchCallback = ();

//     fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
//         &self
//     }
//     fn syscall_filter(&self) -> &Self::SyscallFilter {
//         &()
//     }
//     fn process_fault(&self) -> &Self::ProcessFault {
//         &()
//     }
//     fn credentials_checking_policy(&self) -> &'static Self::CredentialsCheckingPolicy {
//         &()
//     }
//     fn scheduler(&self) -> &Self::Scheduler {
//         self.scheduler
//     }
//     fn scheduler_timer(&self) -> &Self::SchedulerTimer {
//         &self.systick
//     }
//     fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
//         &()
//     }
// }
/// A structure representing this platform that holds references to all
/// capsules for this platform.
struct S32K144EvalueationKit {
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
}

/// Mapping of integer syscalls to objects that implement syscalls.
// impl SyscallDriverLookup for S32K144EvalueationKit {
//     fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
//     where
//         F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
//     {
//         // match driver_num {
//         //     capsules::console::DRIVER_NUM => f(Some(self.console)),
//         //     capsules::led::DRIVER_NUM => f(Some(self.led)),
//         //     capsules::button::DRIVER_NUM => f(Some(self.button)),
//         //     capsules::adc::DRIVER_NUM => f(Some(self.adc)),
//         //     capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
//         //     capsules::temperature::DRIVER_NUM => f(Some(self.temperature)),
//         //     kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
//         //     capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
//         //     _ => f(None),
//         // }
//     }
// }

// impl
//     KernelResources<
//         s32k14x::chip::S32k14x<
//             'static,
//             s32k14x::interrupt_service::S32K14xDefaultPeripherals<'static>,
//         >,
//     > for S32K144EvalueationKit
// {
//     type SyscallDriverLookup = Self;
//     type SyscallFilter = ();
//     type ProcessFault = ();
//     type CredentialsCheckingPolicy = ();
//     type Scheduler = RoundRobinSched<'static>;
//     type SchedulerTimer = cortexm4::systick::SysTick;
//     type WatchDog = ();
//     type ContextSwitchCallback = ();

//     fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
//         &self
//     }
//     fn syscall_filter(&self) -> &Self::SyscallFilter {
//         &()
//     }
//     fn process_fault(&self) -> &Self::ProcessFault {
//         &()
//     }
//     fn credentials_checking_policy(&self) -> &'static Self::CredentialsCheckingPolicy {
//         &()
//     }
//     fn scheduler(&self) -> &Self::Scheduler {
//         self.scheduler
//     }
//     fn scheduler_timer(&self) -> &Self::SchedulerTimer {
//         &self.systick
//     }
//     fn watchdog(&self) -> &Self::WatchDog {
//         &()
//     }
//     fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
//         &()
//     }
// }

/// Main function
///
/// This is called after RAM initialization is complete.
#[no_mangle]
pub unsafe fn main() {
    s32k14x::init();
    let wdt = s32k14x::wdt::Wdt::new();

    WatchDog::suspend(&wdt);
    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));
    let pcc = static_init!(s32k14x::pcc::Pcc, s32k14x::pcc::Pcc::new());
    let peripherals = get_peripherals(pcc);


    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 2], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    set_pin_primary_functions(peripherals);

    // let chip = static_init!(
    //     stm32f429zi::chip::Stm32f4xx<Stm32f429ziDefaultPeripherals>,
    //     stm32f429zi::chip::Stm32f4xx::new(peripherals)
    // );
    // CHIP = Some(chip);

    io::WRITER.set_initialized();

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    // letnone = S32K144EvalueationKit {
    //     scheduler,
    //     systick: cortexm4::systick::SysTick::new(),
    // };

    // These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    // kernel::process::load_processes(
    //     board_kernel,
    //     chip,
    //     core::slice::from_raw_parts(
    //         &_sapps as *const u8,
    //         &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
    //     ),
    //     core::slice::from_raw_parts_mut(
    //         &mut _sappmem as *mut u8,
    //         &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
    //     ),
    //     &mut PROCESSES,
    //     &FAULT_RESPONSE,
    //     &process_management_capability,
    // )
    // .unwrap_or_else(|err| {
    //     debug!("Error loading processes!");
    //     debug!("{:?}", err);
    // });

    //Uncomment to run multi alarm test
    /*components::test::multi_alarm_test::MultiAlarmTestComponent::new(mux_alarm)
    .finalize(components::multi_alarm_test_component_buf!(stm32f429zi::tim2::Tim2))
    .run();*/

    // board_kernel.kernel_loop(
    //     NULL_PTR,
    //     None,
    //     None,
    //     &main_loop_capability,
    // );
    loop {}
}
