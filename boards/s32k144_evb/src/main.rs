//! Board file for WeAct STM32F401CCU6 Core Board
//!
//! - <https://github.com/WeActTC/MiniF4-STM32F4x1>

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
use kernel::hil::led::LedLow;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{create_capability, debug, static_init};

use s32k14x::chip::S32k14xDefaultPeripherals;

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

/// A structure representing this platform that holds references to all
/// capsules for this platform.

/// Mapping of integer syscalls to objects that implement syscalls.

impl KernelResources<stm32f401cc::chip::Stm32f4xx<'static, Stm32f401ccDefaultPeripherals<'static>>>
    for WeactF401CC
{
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

/// Helper function called during bring-up that configures DMA.

/// Helper function called during bring-up that configures multiplexed I/O.

/// Helper function for miscellaneous peripheral functions
// unsafe fn setup_peripherals(tim2: &stm32f401cc::tim2::Tim2) {
//     // USART2 IRQn is 37
//     cortexm4::nvic::Nvic::new(stm32f401cc::nvic::USART2).enable();

//     // TIM2 IRQn is 28
//     tim2.enable_clock();
//     tim2.start();
//     cortexm4::nvic::Nvic::new(stm32f401cc::nvic::TIM2).enable();
// }

/// Statically initialize the core peripherals for the chip.
///
/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
// #[inline(never)]
// unsafe fn get_peripherals() -> (
//     &'static mut Stm32f401ccDefaultPeripherals<'static>,
//     &'static stm32f401cc::syscfg::Syscfg<'static>,
//     &'static stm32f401cc::dma::Dma1<'static>,
// ) {
//     // We use the default HSI 16Mhz clock
//     let rcc = static_init!(stm32f401cc::rcc::Rcc, stm32f401cc::rcc::Rcc::new());
//     let syscfg = static_init!(
//         stm32f401cc::syscfg::Syscfg,
//         stm32f401cc::syscfg::Syscfg::new(rcc)
//     );
//     let exti = static_init!(
//         stm32f401cc::exti::Exti,
//         stm32f401cc::exti::Exti::new(syscfg)
//     );
//     let dma1 = static_init!(stm32f401cc::dma::Dma1, stm32f401cc::dma::Dma1::new(rcc));
//     let dma2 = static_init!(stm32f401cc::dma::Dma2, stm32f401cc::dma::Dma2::new(rcc));

//     let peripherals = static_init!(
//         Stm32f401ccDefaultPeripherals,
//         Stm32f401ccDefaultPeripherals::new(rcc, exti, dma1, dma2)
//     );
//     (peripherals, syscfg, dma1)
// }

/// Main function.
///
/// This is called after RAM initialization is complete.
#[no_mangle]
pub unsafe fn main() {
    s32k14x::init();

    // let (peripherals, syscfg, dma1) = get_peripherals();
    // peripherals.init();
    // let base_peripherals = &peripherals.stm32f4;

    // setup_peripherals(&base_peripherals.tim2);

    // set_pin_primary_functions(syscfg, &base_peripherals.gpio_ports);

    // setup_dma(
    //     dma1,
    //     &base_peripherals.dma1_streams,
    //     &base_peripherals.usart2,
    // );

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 2], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    let chip = static_init!(
        s32k14x::chip::S32k14x<S32k14xDefaultPeripherals>,
        s32k14x::chip::S32k14x::new()
    );
    // CHIP = Some(chip);

    // UART

    // Create a shared UART channel for kernel debug.
    // base_peripherals.usart2.enable_clock();
    // let uart_mux = components::console::UartMuxComponent::new(
    //     &base_peripherals.usart2,
    //     115200,
    //     dynamic_deferred_caller,
    // )
    // .finalize(components::uart_mux_component_static!());

    // io::WRITER.set_initialized();

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    // // Setup the console.
    // let console = components::console::ConsoleComponent::new(
    //     board_kernel,
    //     capsules::console::DRIVER_NUM,
    //     uart_mux,
    // )
    // .finalize(components::console_component_static!());
    // // Create the debugger object that handles calls to `debug!()`.
    // components::debug_writer::DebugWriterComponent::new(uart_mux)
    //     .finalize(components::debug_writer_component_static!());

    // // LEDs
    // // Clock to Port A, B, C are enabled in `set_pin_primary_functions()`
    // let gpio_ports = &base_peripherals.gpio_ports;

    // let led = components::led::LedsComponent::new().finalize(components::led_component_static!(
    //     LedLow<'static, stm32f401cc::gpio::Pin>,
    //     LedLow::new(gpio_ports.get_pin(stm32f401cc::gpio::PinId::PC13).unwrap()),
    // ));

    // // BUTTONs
    // let button = components::button::ButtonComponent::new(
    //     board_kernel,
    //     capsules::button::DRIVER_NUM,
    //     components::button_component_helper!(
    //         stm32f401cc::gpio::Pin,
    //         (
    //             gpio_ports.get_pin(stm32f401cc::gpio::PinId::PA00).unwrap(),
    //             kernel::hil::gpio::ActivationMode::ActiveLow,
    //             kernel::hil::gpio::FloatingState::PullUp
    //         )
    //     ),
    // )
    // .finalize(components::button_component_static!(stm32f401cc::gpio::Pin));

    // ALARM

    // let tim2 = &base_peripherals.tim2;
    // let mux_alarm = components::alarm::AlarmMuxComponent::new(tim2).finalize(
    //     components::alarm_mux_component_static!(stm32f401cc::tim2::Tim2),
    // );

    // let alarm = components::alarm::AlarmDriverComponent::new(
    //     board_kernel,
    //     capsules::alarm::DRIVER_NUM,
    //     mux_alarm,
    // )
    // .finalize(components::alarm_component_static!(stm32f401cc::tim2::Tim2));

    // ADC

    let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
        .finalize(components::process_printer_text_component_static!());
    PROCESS_PRINTER = Some(process_printer);

    // PROCESS CONSOLE
    let process_console = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
    )
    .finalize(components::process_console_component_static!(
        stm32f401cc::tim2::Tim2
    ));
    let _ = process_console.start();

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    let weact_f401cc = WeactF401CC {
        console: console,
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        adc: adc_syscall,
        led: led,
        button: button,
        alarm: alarm,
        gpio: gpio,
        scheduler,
        systick: cortexm4::systick::SysTick::new(),
    };

    debug!("Initialization complete. Entering main loop");

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

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            &_sapps as *const u8,
            &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
        ),
        core::slice::from_raw_parts_mut(
            &mut _sappmem as *mut u8,
            &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
        ),
        &mut PROCESSES,
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    //Uncomment to run multi alarm test
    /*components::test::multi_alarm_test::MultiAlarmTestComponent::new(mux_alarm)
    .finalize(components::multi_alarm_test_component_buf!(stm32f401cc::tim2::Tim2))
    .run();*/

    board_kernel.kernel_loop(
        &weact_f401cc,
        chip,
        Some(&weact_f401cc.ipc),
        &main_loop_capability,
    );
}
