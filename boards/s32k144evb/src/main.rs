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
use kernel::hil::gpio::Configure;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{create_capability, debug, static_init};
use s32k14x::chip::S32k14xDefaultPeripherals;
use kernel::platform::watchdog::WatchDog;
/// Support routines for debugging I/O.
pub mod io;
/// Defines a vector which contains the boot section
pub mod flashcfg;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

// Actual memory for holding the active process structures.
static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None, None, None, None];

static mut CHIP: Option<&'static s32k14x::chip::S32k14x<S32k14xDefaultPeripherals>> = None;
static mut PROCESS_PRINTER: Option<&'static kernel::process::ProcessPrinterText> = None;

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

// Manually setting the boot header section that contains the FCB header
#[used]
#[link_section = ".flashconfig"]
static BOOT_HDR: [u32; 4] = flashcfg::FLASH_CFG;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x4000] = [0; 0x4000];

/// Supported drivers by the platform
pub struct Platform {
    gpio: &'static capsules::gpio::GPIO<'static, s32k14x::gpio::Pin<'static>>,
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

/// Helper function called during bring-up that configures multiplexed I/O.
unsafe fn set_pin_primary_functions(peripherals: &S32k14xDefaultPeripherals) {
    peripherals.ports.gpio1.enable_clock();
    peripherals.ports.gpio2.enable_clock();
    peripherals.ports.gpio3.enable_clock();
    peripherals.ports.gpio4.enable_clock();
    peripherals.ports.gpio5.enable_clock();

}

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
    
    // Configuring the GPIO_AD_B0_09 as output
    let RedLed = peripherals.ports.pin(s32k14x::gpio::PinId::PTD_00);
    RedLed.make_output();
    loop {}
}
