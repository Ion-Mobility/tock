//! Board file for GridaniaTelematic Discovery development board
//!
//! - <https://www.nxp.com/document/guide/get-started-with-the-GridaniaTelematic:NGS-GridaniaTelematic>
#![no_std]
#![no_main]
#[deny(missing_docs)]
use capsules::virtual_alarm::VirtualMuxAlarm;
use components::gpio::GpioComponent;
use kernel::capabilities;
use kernel::component::Component;
use kernel::debug;
use kernel::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::hil::gpio::Configure;
use kernel::hil::led::LedLow;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{create_capability, static_init};
use s32k14x as gridaniatel;
use s32k14x::flexcan::FLEXCAN;
use s32k14x::lpspi::LPSPI;

// Unit Tests for drivers.
// #[allow(dead_code)]
// mod virtual_uart_rx_test;

/// Defines a vector which contains the boot section
pub mod flashcfg;
/// Support routines for debugging I/O.
pub mod io;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

// Actual memory for holding the active process structures.
static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

type Chip = gridaniatel::chip::S32k14x<gridaniatel::chip::S32k14xDefaultPeripherals>;
static mut CHIP: Option<&'static Chip> = None;
static mut PROCESS_PRINTER: Option<&'static kernel::process::ProcessPrinterText> = None;

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

extern "C" {
    pub fn CanTransmitPacket(MailboxID: u8, MsgID: u32, data: *const u8, len: u8) -> u8;
    pub fn CanInit();
    pub fn CanReceivePacket(
        MailboxID: u8,
        MsgID: *const u32,
        data: *const u8,
        len: *const u8,
    ) -> u8;
}

// Manually setting the boot header section that contains the FCB header
#[used]
#[link_section = ".flashconfig"]
static BOOT_HDR: [u32; 4] = flashcfg::FLASH_CFG;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x4000] = [0; 0x4000];

/// A structure representing this platform that holds references to all
/// capsules for this platform.
struct GridaniaTelematic {
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        VirtualMuxAlarm<'static, gridaniatel::lpit::Lpit1<'static>>,
    >,
    button: &'static capsules::button::Button<'static, gridaniatel::gpio::Pin<'static>>,
    console: &'static capsules::console::Console<'static>,
    gpio: &'static capsules::gpio::GPIO<'static, gridaniatel::gpio::Pin<'static>>,
    ipc: kernel::ipc::IPC<{ NUM_PROCS as u8 }>,
    led: &'static capsules::led::LedDriver<
        'static,
        LedLow<'static, gridaniatel::gpio::Pin<'static>>,
        1,
    >,
    // ninedof: &'static capsules::ninedof::NineDof<'static>,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
}

/// Mapping of integer syscalls to objects that implement syscalls.
impl SyscallDriverLookup for GridaniaTelematic {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            // capsules::ninedof::DRIVER_NUM => f(Some(self.ninedof)),
            _ => f(None),
        }
    }
}

impl KernelResources<gridaniatel::chip::S32k14x<gridaniatel::chip::S32k14xDefaultPeripherals>>
    for GridaniaTelematic
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
/// DMA for imxrt1050-evkb is not implemented yet.
// unsafe fn setup_dma() {
// }

/// Helper function called during bring-up that configures multiplexed I/O.
unsafe fn set_pin_primary_functions(
    peripherals: &'static gridaniatel::chip::S32k14xDefaultPeripherals,
) {
    use gridaniatel::gpio::PinId;
    use gridaniatel::gpio::PinMuxFunction;

    peripherals.ports.gpio1.enable_clock();
    peripherals.ports.gpio2.enable_clock();
    peripherals.ports.gpio3.enable_clock();
    peripherals.ports.gpio4.enable_clock();
    peripherals.ports.gpio5.enable_clock();

    // // User_LED is connected to GPIO_AD_B0_09.
    // // Values set accordingly to the evkbimxrt1050_iled_blinky SDK example
    // Configuring the GPIO_AD_B0_09 as output
    let pin = peripherals.ports.pin(PinId::Ptd24);
    pin.pin_make_function(PinMuxFunction::PORT_MUX_AS_GPIO);
    pin.make_output();
    kernel::debug::assign_gpios(Some(pin), None, None);

    // We configure the pin in GPIO mode and disable the Software Input
    // on Field, so that the Input Path is determined by functionality.
    // peripherals.iomuxc_snvs.enable_sw_mux_ctl_pad_gpio(
    //     MuxMode::ALT5, // ALT5 for AdB0_09: GPIO5_IO00 of instance: gpio5
    //     Sion::Disabled,
    //     0,
    // );
    let uarttxpin = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb00);
    let uartrxpin = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb01);
    uarttxpin.pin_make_function(PinMuxFunction::PORT_MUX_ALT2);
    uartrxpin.pin_make_function(PinMuxFunction::PORT_MUX_ALT2);
    let cantxpin = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte24);
    let canrxpin = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte25);
    cantxpin.pin_make_function(PinMuxFunction::PORT_MUX_ALT3);
    canrxpin.pin_make_function(PinMuxFunction::PORT_MUX_ALT3);

    // Configuring the IOMUXC_SNVS_WAKEUP pin as input
    peripherals
        .ports
        .pin(PinId::Ptc12)
        .pin_make_function(PinMuxFunction::PORT_MUX_AS_GPIO);
    peripherals.ports.pin(PinId::Ptc12).make_input();
}

/// Helper function for miscellaneous peripheral functions
unsafe fn setup_peripherals(peripherals: &gridaniatel::chip::S32k14xDefaultPeripherals) {
    use gridaniatel::gpio::PinId;
    use gridaniatel::gpio::PinMuxFunction;

    let PinThrottleAdc = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta00);
    let PinLcdTempAdc = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta01);
    let PinPinSysI2cSda = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta02);
    let PinPinSysI2cScl = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta03);
    // let PinPinSwdioTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta04);
    // let PinPinResetTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta05);
    let PinSpiTmLizardCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta06);
    let PinRtcClkIn = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta07);
    let PinUartLteToTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta08);
    let PinUartTmToLte = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta09);
    let PinSwoTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta10);
    let PinSpkI2sLrclk = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta11);
    let PinSpkI2sBclk = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta12);
    let PinSpkI2sData = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta13);
    let PinSpiAccelCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta15);
    let PinSpiTmWifiCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta16);
    let PinPinSpiTmWifiInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta17);
    let PinSpiAudFlCsel = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta26);
    let PinSpiTmCopi = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta27);
    let PinSpiTmClk = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta28);
    let PinSpiTmCipo = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta29);
    let PinWifiRstn = gridaniatel::gpio::Pin::from_pin_id(PinId::Pta31);

    let PinUartRXTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb00);
    let PinUartTXTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb01);
    let PinSpiAudClk = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb02);
    let PinSpiAudCipo = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb03);
    let PinSpiAudCopi = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb04);
    let PinSpiAudFlashRst = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb05);
    let PinPkeExtClk = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb08);
    let PinTmHmiGpio0 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb09);
    let PinTmHmiGpio1 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb10);
    let PinTmHmiGpio2 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb11);
    let PinHudLampPkeR = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb12);
    let PinHudLampPkeG = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb13);
    let PinAmbientTemp2 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb14);
    let PinAmbientTemp3 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb15);
    let PinSpkI2sMute = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb16);
    let PinTmBleCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb17);
    let PinSpkI2sShutdow = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb18);
    let PinSpiTmRtSout = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb27);
    let PinSpiTmRtSin = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb28);
    let PinSpiTmRtSck = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptb29);

    let PinVbattAdc = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc00);
    let PinQspiTmIo3 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc02);
    let PinQspiTmCsel = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc03);
    // let PinSwclkTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc04);
    let PinBoostShutdown = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc05);
    let PinWifiBootMode0 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc08);
    let PinWifiBootMode1 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc09);
    let PinUsbOtgEn = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc11);
    let PinSpiRtTmInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc13);
    let PinLteSimHotplug = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc14);
    let PinLteSim1Presence = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc15);
    let PinLteSimSwitch = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc17);
    let PinSpiTmRtCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc19);
    let PinSpiTmFlReset = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc23);
    let PinPkeBusy = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc27);
    let PinSpkCdDiag148 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc29);
    let PinSpiPkeClk = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc30);
    let PinVHmi5vEn = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptc31);

    let PinLightInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd00);
    let PinSpiPkeCipo = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd01);
    let PinSpiPkeCopi = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd02);
    let PinSpiTmPkeCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd03);
    let PinAmbientTemp1 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd04);
    let PinLteResetN = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd05);
    let PinLtePwrKey = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd06);
    let PinQspiTmIo1 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd07);
    let PinTmHmiCopi = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd08);
    let PinTmHmiClk = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd09);
    let PinQspiTmSck = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd10);
    let PinQspiTmIo0 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd11);
    let PinQspiTmIo2 = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd12);
    let PinUartRtToTmRx = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd13);
    let PinUartRtToTmTx = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd14);
    let PinLteLsEn = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd17);
    let PinVlteEn = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd22);
    let PinV5VEn = gridaniatel::gpio::Pin::from_pin_id(PinId::Ptd24);

    let PinI2sHmiEn = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte00);
    let PinTmHmiInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte01);
    let PinDiagCanStbTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte02);
    let PinUartLteRts = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte03);
    let PinDiagCanRxTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte04);
    let PinDiagCanTxTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte05);
    let PinLizardLna = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte07);
    let PinLizardRst = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte08);
    let PinUartLteCts = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte09);
    let PinSpiTmLizardRdy = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte10);
    let PinSpiTmLizardInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte11);
    let PinLizardAntSel = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte12);
    let PinPkeRst = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte13);
    let PinPkeInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte14);
    let PinTmHmiCipo = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte15);
    let PinTmHmiCs = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte16);
    let PinRtcInt1 = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte19);
    let PinTmBleInt = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte20);
    let PinTmBleNReset = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte21);
    let PinTmBleWakeup = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte22);
    let PinSysCanStbTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte23);
    let PinSysCanTxTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte24);
    let PinSysCanRxTm = gridaniatel::gpio::Pin::from_pin_id(PinId::Pte25);

    PinThrottleAdc.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLcdTempAdc.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPinSysI2cSda.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPinSysI2cScl.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmLizardCs.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinRtcClkIn.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartLteToTm.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartTmToLte.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpkI2sLrclk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpkI2sBclk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpkI2sData.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiAccelCs.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmWifiCs.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPinSpiTmWifiInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiAudFlCsel.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmCopi.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmClk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmCipo.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinWifiRstn.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartRXTm.pin_make_function(PinMuxFunction::PORT_MUX_ALT2);
    PinUartTXTm.pin_make_function(PinMuxFunction::PORT_MUX_ALT2);
    PinSpiAudClk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiAudCipo.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiAudCopi.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiAudFlashRst.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPkeExtClk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiGpio0.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiGpio1.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiGpio2.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinHudLampPkeR.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinHudLampPkeG.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinAmbientTemp2.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinAmbientTemp3.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpkI2sMute.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmBleCs.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpkI2sShutdow.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinVbattAdc.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinQspiTmIo3.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinQspiTmCsel.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinBoostShutdown.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinWifiBootMode0.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinWifiBootMode1.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUsbOtgEn.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiRtTmInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLteSimHotplug.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLteSim1Presence.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLteSimSwitch.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmRtCs.pin_make_function(PinMuxFunction::PORT_MUX_ALT5);
    PinSpiTmFlReset.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPkeBusy.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpkCdDiag148.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiPkeClk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinVHmi5vEn.pin_make_function(PinMuxFunction::PORT_MUX_AS_GPIO);
    PinLightInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiPkeCipo.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiPkeCopi.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmPkeCs.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinAmbientTemp1.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLteResetN.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLtePwrKey.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinQspiTmIo1.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiCopi.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiClk.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinQspiTmSck.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinQspiTmIo0.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinQspiTmIo2.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartRtToTmRx.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartRtToTmTx.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLteLsEn.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinVlteEn.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinV5VEn.pin_make_function(PinMuxFunction::PORT_MUX_AS_GPIO);
    PinI2sHmiEn.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinDiagCanStbTm.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartLteRts.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinDiagCanRxTm.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinDiagCanTxTm.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLizardLna.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLizardRst.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinUartLteCts.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmLizardRdy.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSpiTmLizardInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinLizardAntSel.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPkeRst.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinPkeInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiCipo.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmHmiCs.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinRtcInt1.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmBleInt.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmBleNReset.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinTmBleWakeup.pin_make_function(PinMuxFunction::PORT_PIN_DISABLED);
    PinSysCanStbTm.pin_make_function(PinMuxFunction::PORT_MUX_AS_GPIO);
    PinSysCanTxTm.pin_make_function(PinMuxFunction::PORT_MUX_ALT3);
    PinSysCanRxTm.pin_make_function(PinMuxFunction::PORT_MUX_ALT3);
    PinSpiTmRtSout.pin_make_function(PinMuxFunction::PORT_MUX_ALT5);
    PinSpiTmRtSin.pin_make_function(PinMuxFunction::PORT_MUX_ALT5);
    PinSpiTmRtSck.pin_make_function(PinMuxFunction::PORT_MUX_ALT5);

    PinV5VEn.make_output();
    PinSysCanStbTm.make_output();
    PinVHmi5vEn.make_output();
    PinV5VEn.set_output_high();
    PinSysCanStbTm.set_output_low();
    PinVHmi5vEn.set_output_high();
    // LPUART1 IRQn is 20
    // cortexm4::nvic::Nvic::new(gridaniatel::nvic::LPUART0_RXTX_IRQN).enable();
    cortexm4::nvic::Nvic::new(gridaniatel::nvic::RTC_IRQN).disable();

    // TIM2 IRQn is 28
    // peripherals.lpit1.enable_clock();
    // peripherals.lpit1.start(
    //     peripherals.ccm.perclk_sel(),
    //     peripherals.ccm.perclk_divider(),
    // );
    // cortexm4::nvic::Nvic::new(gridaniatel::nvic::LPIT0_CH0_IRQN).enable();
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
unsafe fn get_peripherals() -> &'static mut gridaniatel::chip::S32k14xDefaultPeripherals {
    let pcc = static_init!(gridaniatel::pcc::Pcc, gridaniatel::pcc::Pcc::new());
    let peripherals = static_init!(
        gridaniatel::chip::S32k14xDefaultPeripherals,
        gridaniatel::chip::S32k14xDefaultPeripherals::new(pcc)
    );

    peripherals
}

/// Helper function called during bring-up that configures multiplexed I/O.
unsafe fn clk_initialize(peripherals: &gridaniatel::chip::S32k14xDefaultPeripherals) {
    use s32k14x::gpio;
    use s32k14x::pcc;
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

    let peripheralclockconfig: [spc::PeripheralClockConfig; 32] = [
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::ADC0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::ADC1_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPSPI0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPSPI1_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPSPI2_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC_DIV2,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPUART0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPUART1_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPUART2_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPI2C0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_TWO,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPIT0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::LPTMR0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FTM0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC_DIV1,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FTM1_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC_DIV1,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FTM2_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC_DIV1,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FTM3_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC_DIV1,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FLEXIO0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_SIRC,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::CMP0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::CRC0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::DMAMUX0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::EWM0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FTFC0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PDB0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PDB1_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::RTC0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FlexCAN0_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FlexCAN1_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::FlexCAN2_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PORTA_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PORTB_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PORTC_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PORTD_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
        spc::PeripheralClockConfig {
            clockName: spc::PeripheralClockName::PORTE_CLK,
            clkGate: true,
            clkSrc: spc::PeripheralClockSource::CLK_SRC_OFF,
            frac: spc::PeripheralClockFrac::MULTIPLY_BY_ONE,
            divider: spc::PeripheralClockDiv::DIVIDE_BY_ONE,
        },
    ];

    let pccConfig: spc::PCCConfig = spc::PCCConfig {
        perclockconfig: peripheralclockconfig,
        numberofper: 32,
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

/// Main function.
///
/// This is called after RAM initialization is complete.
#[no_mangle]
pub unsafe fn main() {
    gridaniatel::init();
    let peripherals = get_peripherals();
    clk_initialize(peripherals);
    // peripherals.pcc.set_low_power_mode();
    // peripherals.lpuart0.enable_clock();
    // peripherals.lpuart1.enable_clock();
    // peripherals.lpuart2.enable_clock();
    // peripherals
    //     .pcc
    //     .set_uart_clock_sel(gridaniatel::pcc::PerclkClockSel::SIRC);
    // peripherals.pcc.set_uart_clock_podf(1);
    // peripherals.lpuart0.set_baud();

    set_pin_primary_functions(peripherals);

    setup_peripherals(peripherals);

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 2], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    let chip = static_init!(Chip, Chip::new(peripherals));
    CHIP = Some(chip);

    // LPUART1

    // // Enable tx and rx from iomuxc
    // // TX is on pad GPIO_AD_B0_12
    // // RX is on pad GPIO_AD_B0_13
    // // Values set accordingly to the evkbimxrt1050_hello_world SDK example

    // // First we configure the pin in LPUART mode and disable the Software Input
    // // on Field, so that the Input Path is determined by functionality.
    // peripherals.iomuxc.enable_sw_mux_ctl_pad_gpio(
    //     PadId::AdB0,
    //     MuxMode::ALT2, // ALT2: LPUART1_TXD of instance: lpuart1
    //     Sion::Disabled,
    //     13,
    // );
    // peripherals.iomuxc.enable_sw_mux_ctl_pad_gpio(
    //     PadId::AdB0,
    //     MuxMode::ALT2, // ALT2: LPUART1_RXD of instance: lpuart1
    //     Sion::Disabled,
    //     14,
    // );

    // Configure the pin resistance value, pull up or pull down and other
    // physical aspects.
    // peripherals.iomuxc.configure_sw_pad_ctl_pad_gpio(
    //     PadId::AdB0,
    //     13,
    //     PullUpDown::Pus0_100kOhmPullDown,   // 100K Ohm Pull Down
    //     PullKeepEn::Pke1PullKeeperEnabled,  // Pull-down resistor or keep the previous value
    //     OpenDrainEn::Ode0OpenDrainDisabled, // Output is CMOS, either 0 logic or 1 logic
    //     Speed::Medium2,                     // Operating frequency: 100MHz - 150MHz
    //     DriveStrength::DSE6, // Dual/Single voltage: 43/43 Ohm @ 1.8V, 40/26 Ohm @ 3.3V
    // );
    // peripherals.iomuxc.configure_sw_pad_ctl_pad_gpio(
    //     PadId::AdB0,
    //     14,
    //     PullUpDown::Pus0_100kOhmPullDown,   // 100K Ohm Pull Down
    //     PullKeepEn::Pke1PullKeeperEnabled,  // Pull-down resistor or keep the previous value
    //     OpenDrainEn::Ode0OpenDrainDisabled, // Output is CMOS, either 0 logic or 1 logic
    //     Speed::Medium2,                     // Operating frequency: 100MHz - 150MHz
    //     DriveStrength::DSE6, // Dual/Single voltage: 43/43 Ohm @ 1.8V, 40/26 Ohm @ 3.3V
    // );

    // Enable clock
    peripherals.lpuart0.enable_clock();

    let lpuart_mux = components::console::UartMuxComponent::new(
        &peripherals.lpuart0,
        115200,
        dynamic_deferred_caller,
    )
    .finalize(components::uart_mux_component_static!());
    io::WRITER.set_initialized();

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    // Setup the console.
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules::console::DRIVER_NUM,
        lpuart_mux,
    )
    .finalize(components::console_component_static!());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(lpuart_mux)
        .finalize(components::debug_writer_component_static!());

    // Setup SPI2
    // const SPI_BAUD_RATE_FREQUENCY: u32 = 1_000_000;
    let pcc = crate::gridaniatel::pcc::Pcc::new();
    // let mut lpspi2 = LPSPI::new_lpspi2(&pcc);
    // lpspi2.init();
    // lpspi2.set_spi_clock(8_000_000 as u32, SPI_BAUD_RATE_FREQUENCY);
    // let mut spi_tx_buf: [u8; 17] = [
    //     0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    //     0x10, 0x11,
    // ];
    // let mut spi_rx_buf: [u8; 17] = [0; 17];
    // lpspi2.exchange(1, spi_tx_buf.as_ptr(), 17, spi_rx_buf.as_mut_ptr(), 17);
    // // lpspi2.write_no_read(1, spi_tx_buf.as_ptr(), 17);

    //setup flex CAN
    let mut flexcan2 = FLEXCAN::new_flexcan2(&pcc);
    flexcan2.init();
    let mut can_tx_buf: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    flexcan2.can_transmit(1, 0x8000_FFFF, can_tx_buf.as_ptr(), 8);
    flexcan2.can_transmit(1, 0x8000_FFFF, can_tx_buf.as_ptr(), 8);
    flexcan2.can_transmit(1, 0x8000_FFFF, can_tx_buf.as_ptr(), 8);

    // LEDs

    // Clock to Port A is enabled in `set_pin_primary_functions()
    let led = components::led::LedsComponent::new().finalize(components::led_component_static!(
        LedLow<'static, gridaniatel::gpio::Pin<'static>>,
        LedLow::new(peripherals.ports.pin(gridaniatel::gpio::PinId::Ptd16)),
    ));

    // BUTTONs
    let button = components::button::ButtonComponent::new(
        board_kernel,
        capsules::button::DRIVER_NUM,
        components::button_component_helper!(
            gridaniatel::gpio::Pin,
            (
                peripherals.ports.pin(gridaniatel::gpio::PinId::Ptc12),
                kernel::hil::gpio::ActivationMode::ActiveHigh,
                kernel::hil::gpio::FloatingState::PullDown
            )
        ),
    )
    .finalize(components::button_component_static!(gridaniatel::gpio::Pin));

    // ALARM
    let lpit1 = &peripherals.lpit1;
    let mux_alarm = components::alarm::AlarmMuxComponent::new(lpit1).finalize(
        components::alarm_mux_component_static!(gridaniatel::lpit::Lpit1),
    );

    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_static!(
        gridaniatel::lpit::Lpit1
    ));

    // GPIO
    // For now we expose only two pins
    let gpio = GpioComponent::new(
        board_kernel,
        capsules::gpio::DRIVER_NUM,
        components::gpio_component_helper!(
            gridaniatel::gpio::Pin<'static>,
            // The User Led
            0 => peripherals.ports.pin(gridaniatel::gpio::PinId::Ptd16)
        ),
    )
    .finalize(components::gpio_component_static!(
        gridaniatel::gpio::Pin<'static>
    ));

    // LPI2C
    // AD_B1_00 is LPI2C1_SCL
    // AD_B1_01 is LPI2C1_SDA
    // Values set accordingly to the evkbimxrt1050_bubble_peripheral SDK example

    // // First we configure the pin in LPUART mode and enable the Software Input
    // // on Field, so that we force input path of the pad.
    // peripherals.iomuxc.enable_sw_mux_ctl_pad_gpio(
    //     PadId::AdB1,
    //     MuxMode::ALT3, // ALT3:  LPI2C1_SCL of instance: lpi2c1
    //     Sion::Enabled,
    //     0,
    // );
    // // Selecting AD_B1_00 for LPI2C1_SCL in the Daisy Chain.
    // peripherals.iomuxc.enable_lpi2c_scl_select_input();

    // peripherals.iomuxc.enable_sw_mux_ctl_pad_gpio(
    //     PadId::AdB1,
    //     MuxMode::ALT3, // ALT3:  LPI2C1_SDA of instance: lpi2c1
    //     Sion::Enabled,
    //     1,
    // );
    // // Selecting AD_B1_01 for LPI2C1_SDA in the Daisy Chain.
    // peripherals.iomuxc.enable_lpi2c_sda_select_input();

    // // Configure the pin resistance value, pull up or pull down and other
    // // physical aspects.
    // peripherals.iomuxc.configure_sw_pad_ctl_pad_gpio(
    //     PadId::AdB1,
    //     0,
    //     PullUpDown::Pus3_22kOhmPullUp,     // 22K Ohm Pull Up
    //     PullKeepEn::Pke1PullKeeperEnabled, // Pull-down resistor or keep the previous value
    //     OpenDrainEn::Ode1OpenDrainEnabled, // Open Drain Enabled (Output is Open Drain)
    //     Speed::Medium2,                    // Operating frequency: 100MHz - 150MHz
    //     DriveStrength::DSE6, // Dual/Single voltage: 43/43 Ohm @ 1.8V, 40/26 Ohm @ 3.3V
    // );

    // peripherals.iomuxc.configure_sw_pad_ctl_pad_gpio(
    //     PadId::AdB1,
    //     1,
    //     PullUpDown::Pus3_22kOhmPullUp,     // 22K Ohm Pull Up
    //     PullKeepEn::Pke1PullKeeperEnabled, // Pull-down resistor or keep the previous value
    //     OpenDrainEn::Ode1OpenDrainEnabled, // Open Drain Enabled (Output is Open Drain)
    //     Speed::Medium2,                    // Operating frequency: 100MHz - 150MHz
    //     DriveStrength::DSE6, // Dual/Single voltage: 43/43 Ohm @ 1.8V, 40/26 Ohm @ 3.3V
    // );

    // Enabling the lpi2c1 clock and setting the speed.
    // peripherals.lpi2c1.enable_clock();
    // peripherals
    //     .lpi2c1
    //     .set_speed(imxrt1050::lpi2c::Lpi2cSpeed::Speed100k, 8);

    // use imxrt1050::gpio::PinId;
    // let mux_i2c =
    //     components::i2c::I2CMuxComponent::new(&peripherals.lpi2c1, None, dynamic_deferred_caller)
    //         .finalize(components::i2c_mux_component_static!());

    // Fxos8700 sensor
    // let fxos8700 = components::fxos8700::Fxos8700Component::new(
    //     mux_i2c,
    //     0x1f,
    //     peripherals.ports.pin(PinId::AdB1_00),
    // )
    // .finalize(components::fxos8700_component_static!());

    // Ninedof
    // let ninedof =
    //     components::ninedof::NineDofComponent::new(board_kernel, capsules::ninedof::DRIVER_NUM)
    //         .finalize(components::ninedof_component_static!(fxos8700));

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    let s32k144 = GridaniaTelematic {
        console: console,
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        led: led,
        button: button,
        // ninedof: ninedof,
        alarm: alarm,
        gpio: gpio,

        scheduler,
        systick: cortexm4::systick::SysTick::new_with_calibration(792_000_000),
    };

    // Optional kernel tests
    //
    // See comment in `boards/imix/src/main.rs`
    // virtual_uart_rx_test::run_virtual_uart_receive(mux_uart);

    //--------------------------------------------------------------------------
    // Process Console
    //---------------------------------------------------------------------------
    let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
        .finalize(components::process_printer_text_component_static!());
    PROCESS_PRINTER = Some(process_printer);

    let process_console = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        lpuart_mux,
        mux_alarm,
        process_printer,
    )
    .finalize(components::process_console_component_static!(
        gridaniatel::lpit::Lpit1
    ));
    let _ = process_console.start();

    debug!("Tock OS initialization complete. Entering main loop");

    debug!("Init CAN\n");
    // CanInit();
    // debug!("Transmit CAN packet:\n");
    // let mut _tx_buf: [u8; 8] = [0x12; 8];
    // let mut _rx_buf: [u8; 8] = [0x12; 8];
    // let mut _value: u8 = 0;
    // let mut _rx_value: u8 = 0;
    // let MsgID: u32 = 0;
    // let len: u8 = 0;
    // _value = CanTransmitPacket(8, 0x80001234, _tx_buf.as_ptr(), 8);
    // _value = CanTransmitPacket(8, 0x80001234, _tx_buf.as_ptr(), 8);
    // _value = CanTransmitPacket(8, 0x80001234, _tx_buf.as_ptr(), 8);

    // while _rx_value == 0 {
    //     _rx_value = CanReceivePacket(0, &MsgID, _rx_buf.as_ptr(), &len);
    //     debug!("rx value: {:?} \n", _rx_buf);
    // }

    extern "C" {
        /// Beginning of the ROM region containing app images.
        ///
        /// This symbol is defined in the linker script.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        ///
        /// This symbol is defined in the linker script.
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

    board_kernel.kernel_loop(&s32k144, chip, Some(&s32k144.ipc), &main_loop_capability);
}
