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
use cortexm4;
use cortexm4::support;
use cortexm4::support::atomic;
use kernel::capabilities;
use kernel::component::Component;
use kernel::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::hil;
use kernel::hil::gpio::Configure;
use kernel::hil::led::LedHigh;
use kernel::hil::led::LedLow;
use kernel::hil::Controller;
use kernel::platform::watchdog::WatchDog;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{create_capability, debug, static_init};
use s32k14x::chip::S32k14xDefaultPeripherals;
/// Defines a vector which contains the boot section
pub mod flashcfg;
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
unsafe fn clk_initialize(peripherals: &S32k14xDefaultPeripherals) {
    use s32k14x::spc;
    let sircConfig: spc::SIRCConfig = spc::SIRCConfig {
        initialize: true,
        /// Enable SIRC in stop mode
        enableInStop: true,
        /// Enable SIRC in low power mode
        enableInLowPower: true,
        /// unlocked
        locked: false,
        /// Slow IRC high range clock (8 MHz)
        range: 1,
        /// Slow IRC Clock Divider 1: divided by 1
        div1: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_1,
        /// Slow IRC Clock Divider 3: divided by 1
        div2: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_1,
    };
    let fircConfig: spc::FIRCConfig = spc::FIRCConfig {
        initialize: true,
        /// Enable FIRC in stop mode
        enableInStop: true,
        /// Enable FIRC in low power mode
        enableInLowPower: true,
        /// FIRC regulator is enabled
        regulator: true,
        /// unlocked
        locked: false,
        /// RANGE      
        range: 0,
        /// Fast IRC Clock Divider 1: divided by 1
        div1: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_1,
        /// Fast IRC Clock Divider 3: divided by 1
        div2: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_1,
    };
    let rtc_config: spc::RTCConfig = spc::RTCConfig {
        rtcClkInFreq: 8000000,
        initialize: true,
    };
    let soscConfig: spc::SOSCConfig = spc::SOSCConfig {
        initialize: true,
        /// Enable SOSC in stop mode
        enableInStop: true,
        /// Enable SOSC in low power mode
        enableInLowPower: true,
        /// System Oscillator frequency: 16000000Hz
        req: 16000000,
        /// Monitor disabled
        monitorMode: spc::SOSCMonitorMode::SCG_SOSC_MONITOR_DISABLE,
        /// SOSC disabled
        locked: false,
        /// Internal oscillator of OSC requested
        extRef: spc::SOSCRefSelect::SCG_SOSC_REF_OSC,
        /// Configure crystal oscillator for low-gain operation
        gain: spc::SOSCGainMode::SCG_SOSC_GAIN_LOW,
        /// High frequency range selected for the crystal oscillator of 8 MHz to 40 MHz
        range: spc::SOSCRange::SCG_SOSC_RANGE_HIGH,
        /// System OSC Clock Divider 1: divided by 1
        div1: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_1,
        /// System OSC Clock Divider 3: divided by 1
        div2: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_1,
    };

    let spllConfig: spc::SPLLConfig = spc::SPLLConfig {
        initialize: true,
        /// Enable SPLL in stop mode
        enableInStop: true,
        /// Monitor disabled
        monitorMode: spc::SPLLMonitorMode::SCG_SPLL_MONITOR_DISABLE,
        /// unlocked
        locked: false,
        /// Divided by 1
        prediv: spc::SPLLClockDiv::SCG_SPLL_CLOCK_PREDIV_BY_2,
        /// Multiply Factor is 28
        mult: spc::SPLLClockMul::SCG_SPLL_CLOCK_MULTIPLY_BY_28,
        src: 0,
        /// System PLL Clock Divider 1: divided by 2
        div1: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_2,
        /// System PLL Clock Divider 3: divided by 4
        div2: spc::AsyncClockDiv::SCG_ASYNC_CLOCK_DIV_BY_4,
    };

    let clockOutConfig: spc::ClockOutConfig = spc::ClockOutConfig {
        initialize: true,
        /// Fast IRC
        source: spc::ClockOutSource::SCG_CLOCKOUT_SRC_FIRC,
    };

    let clockModeConfig: spc::ClockModeConfig = spc::ClockModeConfig {
        initialize: true,
        rccrConfig: spc::SystemClockConfig {
            /// Fast FIRC
            sysclksrc: spc::SysClockSource::SCG_SYSTEM_CLOCK_SRC_SYS_PLL,
            /// Core Clock Divider: divided by 1
            divCore: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_3,
            /// Bus Clock Divider: divided by 1
            divBus: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_1,
            /// Slow Clock Divider: divided by 2
            divSlow: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_2,
        },
        vccrConfig: spc::SystemClockConfig {
            /// Slow SIRC
            sysclksrc: spc::SysClockSource::SCG_SYSTEM_CLOCK_SRC_SIRC,
            /// Core Clock Divider divided by 2
            divCore: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_2,
            /// Bus Clock Divider divided by 1
            divBus: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_1,
            /// Slow Clock Divider: divided by 4
            divSlow: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_4,
        },
        hccrConfig: spc::SystemClockConfig {
            /// System PLL
            sysclksrc: spc::SysClockSource::SCG_SYSTEM_CLOCK_SRC_SYS_PLL,
            /// Core Clock Divider: divided by 1
            divCore: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_1,
            /// Bus Clock Divider: divided by 2
            divBus: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_2,
            /// Slow Clock Divider: divided by 4
            divSlow: spc::SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_4,
        },
        alternateClock: spc::SysAlterClockSource::SCG_SYSTEM_CLOCK_SRC_SYS_OSC,
    };

    let pccConfig: spc::PCCConfig = spc::PCCConfig {};

    let simConfig: spc::SIMConfig = spc::SIMConfig {
        clockOutConfig: spc::SimClockOutConfig {
            /// Initialize    
            initialize: true,
            /* enabled */
            enable: true,
            /* SCG CLKOUT clock select: SCG slow clock */
            source: spc::SIMClockOutSource::SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT,
            /* Divided by 1 */
            divider: spc::SIMClockOutDiv::SIM_CLKOUT_DIV_BY_1,
        },
        lpoClockConfig: spc::LpoClockConfig {
            /// Initialize    
            initialize: true,
            /// LPO1KCLKEN    
            enableLpo1k: true,
            /// LPO32KCLKEN   
            enableLpo32k: true,
            /* 128 kHz LPO clock */
            sourceLpoClk: spc::SimLpoClockSel::SIM_LPO_CLK_SEL_LPO_128K,
            /* FIRCDIV1 clock */
            sourceRtcClk: spc::SimRtcClockSel::SIM_RTCCLK_SEL_FIRCDIV1_CLK,
        },
        platGateConfig: spc::SimPlatGateConfig {
            /// Initialize    
            initialize: true,
            /// CGCEIM        
            enableEim: true,
            /// CGCERM        
            enableErm: true,
            /// CGCDMA        
            enableDma: true,
            /// CGCMPU        
            enableMpu: true,
            /// CGCMSCM       
            enableMscm: true,
        },
        qspiRefClkGating: spc::SimQSPIRefClockGating {
            /// QSPI internal reference clock gate       
            enableQspiRefClk: true,
        },
        tclkConfig: spc::SimTCLKClockConfig {
            /// Initialize    
            initialize: false,
            /// TCK Input
            tclkFreq: [0, 0, 0],
            /// FTM Ext Pin
            extPinSrc: [0, 0, 0, 0, 0, 0, 0, 0],
        },
        traceClockConfig: spc::SimTraceClockConfig {
            /// Initialize    
            initialize: true,
            /// TRACEDIVEN    
            divEnable: true,
            /// TRACECLK_SEL  
            source: spc::TraceClockSource::CLOCK_TRACE_SRC_CORE_CLK,
            /// TRACEDIV      
            divider: 0,
            /// TRACEFRAC     
            divFraction: false,
        },
    };

    let pmcConfig: spc::PMCConfig = spc::PMCConfig {
        lpoClockConfig: spc::PmcLpoClockConfig {
            /// Initialize    
            initialize: true,  
            /// Enable/disable LPO     
            enable: true, 
            /// Trimming value for LPO 
            trimValue: 0, 
        },
    };

    let scgConfig: spc::SCGConfig = spc::SCGConfig {
        sircConfig: sircConfig,
        fircConfig: fircConfig,
        soscConfig: soscConfig,
        spllConfig: spllConfig,
        rtcConfig: rtc_config,
        clockOutConfig: clockOutConfig,
        clockModeConfig: clockModeConfig,
    };

    let clkuserconfig: spc::CLKUserConfig = spc::CLKUserConfig {
        scgconfig: scgConfig,
        simconfig: simConfig,
        pccconfig: pccConfig,
        pmcconfig: pmcConfig,
    };

    // peripherals.spc.ConfigureModulesFromScg();
    peripherals.spc.init(clkuserconfig);
}

/// Helper function called during bring-up that configures multiplexed I/O.
unsafe fn set_pin_primary_functions(peripherals: &S32k14xDefaultPeripherals) {
    use s32k14x::pcc::PerclkClockSel;
    peripherals.ports.gpio1.enable_clock();
    peripherals.ports.gpio2.enable_clock();
    peripherals.ports.gpio3.enable_clock();
    peripherals.ports.gpio4.enable_clock();
    peripherals.ports.gpio5.enable_clock();
    peripherals.lpuart0.enable_clock();
    peripherals.lpuart1.enable_clock();
    peripherals.lpuart2.enable_clock();
    peripherals.lpuart0.set_baud();
    peripherals.lpuart1.set_baud();
    peripherals.lpuart2.set_baud();
    peripherals.lpit1.enable_clock();
    peripherals.pcc.set_uart_clock_sel(PerclkClockSel::SIRC);
    peripherals.lpit1.start(PerclkClockSel::SIRC, 0);
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
    clk_initialize(peripherals);

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 2], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    set_pin_primary_functions(peripherals);

    // Configuring the GPIO_AD_B0_09 as output
    // let RedLed = peripherals.ports.pin(s32k14x::gpio::PinId::PTD_00);
    // RedLed.make_output();
    loop {}
}
