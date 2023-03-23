use cortexm4;
use cortexm4::support;
use cortexm4::support::atomic;
use kernel::platform::chip::ClockInterface;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;

#[repr(C)]
/// Clock Controller Module
struct ScgRegisters {
    /// 0x00 - Version ID Register
    pub scgverid: ReadWrite<u32, SCGVERID::Register>,
    /// 0x04 - Parameter Register
    pub scgparam: ReadWrite<u32, SCGPARAM::Register>,
    _reserved0: [u8; 8usize],
    /// 0x10 - Clock Status Register
    pub csr: ReadWrite<u32, CSR::Register>,
    /// 0x14 - Run Clock Control Register
    pub rccr: ReadWrite<u32, RCCR::Register>,
    /// 0x18 - VLPR Clock Control Register
    pub vccr: ReadWrite<u32, VCCR::Register>,
    /// 0x1c - HSRUN Clock Control Register
    pub hccr: ReadWrite<u32, HCCR::Register>,
    /// 0x20 - SCG CLKOUT Configuration Register
    pub clkoutcnfg: ReadWrite<u32, CLKOUTCNFG::Register>,
    _reserved1: [u8; 220usize],
    /// 0x100 - System OSC Control Status Register
    pub sosccsr: ReadWrite<u32, SOSCCSR::Register>,
    /// 0x104 - System OSC Divide Register
    pub soscdiv: ReadWrite<u32, SOSCDIV::Register>,
    /// 0x108 - System Oscillator Configuration Register
    pub sosccfg: ReadWrite<u32, SOSCFG::Register>,
    _reserved2: [u8; 244usize],
    /// 0x200 - Slow IRC Control Status Register
    pub sirccsr: ReadWrite<u32, SIRCCSR::Register>,
    /// 0x204 - Slow IRC Divide Register
    pub sircdiv: ReadWrite<u32, SIRCDIV::Register>,
    /// 0x208 - Slow IRC Configuration Register
    pub sirccfg: ReadWrite<u32, SIRCCFG::Register>,
    _reserved3: [u8; 244usize],
    /// 0x300 - Fast IRC Control Status Register
    pub firccsr: ReadWrite<u32, FIRCCSR::Register>,
    /// 0x304 - Fast IRC Divide Register
    pub fircdiv: ReadWrite<u32, FIRCDIV::Register>,
    /// 0x308 - Fast IRC Configuration Register
    pub firccfg: ReadWrite<u32, FIRCCFG::Register>,
    _reserved4: [u8; 756usize],
    /// 0x600 - System PLL Control Status Register
    pub spllcsr: ReadWrite<u32, SPLLCSR::Register>,
    /// 0x604 - System PLL Divide Register
    pub splldiv: ReadWrite<u32, SPLLDIV::Register>,
    /// 0x608 - System PLL Configuration Register
    pub spllcfg: ReadWrite<u32, SPLLCFG::Register>,
}

const SCG_BASE: StaticRef<ScgRegisters> =
    unsafe { StaticRef::new(0x4006_4000 as *const ScgRegisters) };

pub struct Scg {
    registers: StaticRef<ScgRegisters>,
}

#[repr(C)]
/// Clock Controller Module
struct SmcRegisters {
    /// 0x00 - SMC Version ID Register
    pub smcverid: ReadWrite<u32, SMCVERID::Register>,
    /// 0x04 - SMC Parameter Register
    pub smcparam: ReadWrite<u32, SMCPARAM::Register>,
    /// 0x08 - Power Mode Protection register
    pub pmprot: ReadWrite<u32, PMPROT::Register>,
    /// 0x0c - Power Mode Control register
    pub pmctrl: ReadWrite<u32, PMCTRL::Register>,
    /// 0x10 - Stop Control Register
    pub stopctrl: ReadWrite<u32, STOPCTRL::Register>,
    /// 0x14 - Power Mode Status register
    pub pmstat: ReadWrite<u32, PMSTAT::Register>,
}

const SMC_BASE: StaticRef<SmcRegisters> =
    unsafe { StaticRef::new(0x4007_E000 as *const SmcRegisters) };

pub struct Smc {
    registers: StaticRef<SmcRegisters>,
}

#[repr(C)]
/// Clock Controller Module
struct PmcRegisters {
    /// 0x00 - Low Voltage Detect Status and Control 1 Register
    pub lvdsc1: ReadWrite<u8, LVDSC1::Register>,
    /// 0x01 - Low Voltage Detect Status and Control 2 Register
    pub lvdsc2: ReadWrite<u8, LVDSC2::Register>,
    /// 0x02 - Regulator Status and Control Register
    pub regsc: ReadWrite<u8, REGSC::Register>,
    _reserved0: [u8; 1usize],
    /// 0x04 - Low Power Oscillator Trim Register
    pub lpotrim: ReadWrite<u8, LPOTRIM::Register>,
}

const PMC_BASE: StaticRef<PmcRegisters> =
    unsafe { StaticRef::new(0x4007_D000 as *const PmcRegisters) };

pub struct Pmc {
    registers: StaticRef<PmcRegisters>,
}

register_bitfields![u32,
    SCGVERID [
        VERID OFFSET(0) NUMBITS(32) [],
    ],
    SCGPARAM [
        DIVPRES OFFSET(27) NUMBITS(5) [],
        CLKPRES OFFSET(0) NUMBITS(8) []
    ],
    CSR [
        SCS     OFFSET(24) NUMBITS(4) [],
        DIVCORE OFFSET(16) NUMBITS(4) [],
        DIVBUS  OFFSET(4) NUMBITS(4) [],
        DIVSLOW OFFSET(0) NUMBITS(4) []
    ],
    RCCR [
        SCS     OFFSET(24) NUMBITS(4) [],
        DIVCORE OFFSET(16) NUMBITS(4) [],
        DIVBUS  OFFSET(4) NUMBITS(4) [],
        DIVSLOW OFFSET(0) NUMBITS(4) []

    ],
    VCCR [
        SCS     OFFSET(24) NUMBITS(4) [],
        DIVCORE OFFSET(16) NUMBITS(4) [],
        DIVBUS  OFFSET(4) NUMBITS(4) [],
        DIVSLOW OFFSET(0) NUMBITS(4) []
    ],
    HCCR [
        SCS     OFFSET(24) NUMBITS(4) [],
        DIVCORE OFFSET(16) NUMBITS(4) [],
        DIVBUS  OFFSET(4) NUMBITS(4) [],
        DIVSLOW OFFSET(0) NUMBITS(4) []
    ],
    CLKOUTCNFG [
        CLKOUTSEL OFFSET(24) NUMBITS(4) [],
    ],
    SOSCCSR [
        SOSCERR OFFSET(26) NUMBITS(1) [],
        SOSCSEL OFFSET(25) NUMBITS(1) [],
        SOSCVLD OFFSET(24) NUMBITS(1) [],
        LK OFFSET(23) NUMBITS(1) [],
        SOSCCMRE OFFSET(17) NUMBITS(1) [],
        SOSCCM OFFSET(16) NUMBITS(1) [],
        SOSCEN OFFSET(0) NUMBITS(1) []
    ],
    SOSCDIV [
        SOSCDIV2 OFFSET(8) NUMBITS(3) [],
        SOSCDIV1 OFFSET(0) NUMBITS(3) []
    ],
    SOSCFG [
        RANGE  OFFSET(8) NUMBITS(2) [],
        HGO    OFFSET(3) NUMBITS(1) [],
        EREFS  OFFSET(2) NUMBITS(1) []
    ],
    SIRCCSR [
        SIRCSEL     OFFSET(25) NUMBITS(1) [],
        SIRCVLD     OFFSET(24) NUMBITS(1) [],
        LK          OFFSET(23) NUMBITS(1) [],
        SIRCLPEN    OFFSET(2)  NUMBITS(1) [],
        SIRCSTEN    OFFSET(1)  NUMBITS(1) [],
        SIRCEN      OFFSET(0)  NUMBITS(1) []
    ],
    SIRCDIV [
        SIRCDIV2 OFFSET(8) NUMBITS(3) [],
        SIRCDIV1 OFFSET(0) NUMBITS(3) [],
    ],
    SIRCCFG [
        RANGE   OFFSET(0)   NUMBITS(1) []
    ],
    FIRCCSR [
        FIRCERR     OFFSET(26)   NUMBITS(1) [],
        FIRCSEL     OFFSET(25)   NUMBITS(1) [],
        FIRCVLD     OFFSET(24)   NUMBITS(1) [],
        LK          OFFSET(23)   NUMBITS(1) [],
        FIRCREGOFF  OFFSET(3)    NUMBITS(1) [],
        FIRCEN      OFFSET(0)    NUMBITS(1) [],
    ],
    FIRCDIV [
        FIRCDIV2 OFFSET(8)   NUMBITS(3) [],
        FIRCDIV1 OFFSET(0)   NUMBITS(3) [],
    ],
    FIRCCFG [
        RANGE OFFSET(0)   NUMBITS(2) [],
    ],
    SPLLCSR [
        SPLLERR OFFSET(26)   NUMBITS(1) [],
        SPLLSEL OFFSET(25)   NUMBITS(1) [],
        SPLLVLD OFFSET(24)   NUMBITS(1) [],
        LK OFFSET(23)   NUMBITS(1) [],
        SPLLCMRE OFFSET(17)   NUMBITS(1) [],
        SPLLCM OFFSET(16)   NUMBITS(1) [],
        SPLLEN OFFSET(0)   NUMBITS(1) [],
    ],
    SPLLDIV [
        SPLLDIV2 OFFSET(8)   NUMBITS(3) [],
        SPLLDIV1 OFFSET(0)   NUMBITS(3) [],
    ],
    SPLLCFG [
        MULT    OFFSET(16)   NUMBITS(5) [],
        PREDIV  OFFSET(8)   NUMBITS(3) [],
        SOURCE  OFFSET(0)   NUMBITS(1) [],
    ],
];

register_bitfields![u32,
    SMCVERID [
        MAJOR   OFFSET(24)   NUMBITS(8) [],
        MINOR   OFFSET(16)   NUMBITS(8) [],
        FEATURE OFFSET(0)   NUMBITS(16) [],
    ],
    SMCPARAM [
        EVLLS0 OFFSET(6)   NUMBITS(1) [],
        ELLS2  OFFSET(5)   NUMBITS(1) [],
        ELLS   OFFSET(3)   NUMBITS(1) [],
        EHSRUN OFFSET(0)   NUMBITS(1) [],
    ],
    PMPROT [
        AHSRUN OFFSET(7)   NUMBITS(1) [],
        AVLP   OFFSET(5)   NUMBITS(1) [],
    ],
    PMCTRL [
        RUNM  OFFSET(5)   NUMBITS(2) [],
        VLPSA OFFSET(3)   NUMBITS(1) [],
        STOPM OFFSET(0)   NUMBITS(3) [],
    ],
    STOPCTRL [
        STOPO OFFSET(6)   NUMBITS(2) [],
    ],
    PMSTAT [
        PMSTAT OFFSET(0)   NUMBITS(8) [],
    ],
];

register_bitfields![u8,
    LVDSC1 [
        LVDF    OFFSET(7)   NUMBITS(1) [],
        LVDACK  OFFSET(6)   NUMBITS(1) [],
        LVDIE   OFFSET(5)   NUMBITS(1) [],
        LVDRE   OFFSET(4)   NUMBITS(1) []
    ],
    LVDSC2 [
        LVWF    OFFSET(7)   NUMBITS(1) [],
        LVWACK  OFFSET(6)   NUMBITS(1) [],
        LVWIE   OFFSET(5)   NUMBITS(1) []
    ],
    REGSC [
        LPODIS      OFFSET(7)   NUMBITS(1) [],
        LPOSTAT     OFFSET(6)   NUMBITS(1) [],
        REGFPM      OFFSET(2)   NUMBITS(1) [],
        CLKBIASDIS  OFFSET(1)   NUMBITS(1) [],
        BIASEN      OFFSET(0)   NUMBITS(1) []
    ],
    LPOTRIM [
        LPOTRIM OFFSET(0)   NUMBITS(4) []
    ],
];

/// System Run Mode
pub enum Mode {
    /// Run mode
    ///
    /// - `CORE_CLK` and `SYS_CLK` clock freuqency must be 80M Hz or less (but not configured to be less than `BUS_CLK`).
    /// - `BUS_CLK` clock frequency must be 48 Mhz or less (when using PLL as system clock source maximum bus clock frequency is 40 MHz).
    /// - `FLASH_CLK` clock frequency must be 26.67 MHz or less.
    /// - The core clock to flash clock ratio is limited to a max value of 8.
    Run(RunMode),

    /// High Speed Run mode
    ///
    /// - `CORE_CLK` and `SYS_CLK` clock freuqency must be 112M Hz or less.
    /// - `BUS_CLK` clock frequency must be 56 Mhz or less.
    /// - `FLASH_CLK` clock frequency must be 28 MHz or less.
    /// - The core clock to flash clock ratio is limited to a max value of 8.
    HighSpeed(HighSpeedMode),

    /// Very low power mode
    VeryLowPower(VeryLowPowerMode),
}

/// Clock selection modes available in `Mode::Run(_)`
pub enum RunMode {
    /// System Oscillator Clock
    SOSC,

    /// Slow Internal Reference Clock
    SIRC,

    /// Fast internal Reference Clock
    FIRC,

    /// Sys PLL
    SPLL,
}

/// Clock selection modes available in `Mode::HighSpeed(_)`
pub enum HighSpeedMode {
    /// Fast internal Reference Clock
    FIRC,

    /// Sys PLL
    SPLL,
}

/// Clock selection modes available in `Mode::VeryLowPower(_)`
pub enum VeryLowPowerMode {
    /// Slow Internal Reference Clock
    SIRC,
}

/// Clock divider for `CORE_CLK` and `SYS_CLK`.
pub enum DivCore {
    /// Divide by 1
    Div1 = 1,
    /// Divide by 2
    Div2 = 2,
    /// Divide by 3
    Div3 = 3,
    /// Divide by 4
    Div4 = 4,
    /// Divide by 5
    Div5 = 5,
    /// Divide by 6
    Div6 = 6,
    /// Divide by 7
    Div7 = 7,
    /// Divide by 8
    Div8 = 8,
    /// Divide by 9
    Div9 = 9,
    /// Divide by 10
    Div10 = 10,
    /// Divide by 11
    Div11 = 11,
    /// Divide by 12
    Div12 = 12,
    /// Divide by 13
    Div13 = 13,
    /// Divide by 14
    Div14 = 14,
    /// Divide by 15
    Div15 = 15,
    /// Divide by 16
    Div16 = 16,
}

impl Default for DivCore {
    fn default() -> Self {
        DivCore::Div1
    }
}

impl From<DivCore> for u8 {
    fn from(d: DivCore) -> u8 {
        d as u8
    }
}

impl From<DivCore> for u32 {
    fn from(d: DivCore) -> u32 {
        d as u32
    }
}

/// Clock divider options for system oscillator.
pub enum SystemOscillatorOutput {
    /// Output disabled
    Disable = 0,

    /// Divide by 1
    Div1 = 1,

    /// Divide by 2
    Div2 = 2,

    /// Divide by 4
    Div4 = 3,

    /// Divide by 8
    Div8 = 4,

    /// Divide by 16
    Div16 = 5,

    /// Divide by 32
    Div32 = 6,

    /// Divide by 64
    Div64 = 7,
}

/// Set the configuration of XTAL and EXTAL pins.
pub enum SystemOscillatorInput {
    /// Neither a crystal oscillator nor an external clock is connected.
    None,

    /// A crystal oscillator is connected between XTAL and EXTAL pins.
    ///
    /// The `u32` value specifies the oscillator frequency in Hz
    Crystal(u32),

    /// An external clock reference is connected to the EXTAL pins.
    ///
    /// The `u32` value specifies the reference frequency in Hz
    Reference(u32),
}

impl SystemOscillatorInput {
    pub(crate) fn clock_frequency(&self) -> Option<u32> {
        match *self {
            SystemOscillatorInput::Crystal(f) | SystemOscillatorInput::Reference(f) => Some(f),
            SystemOscillatorInput::None => None,
        }
    }
}

impl Default for SystemOscillatorInput {
    fn default() -> Self {
        SystemOscillatorInput::None
    }
}

impl From<SystemOscillatorOutput> for u8 {
    fn from(div: SystemOscillatorOutput) -> u8 {
        div as u8
    }
}

impl From<SystemOscillatorOutput> for usize {
    fn from(div: SystemOscillatorOutput) -> usize {
        div as usize
    }
}

impl From<SystemOscillatorOutput> for isize {
    fn from(div: SystemOscillatorOutput) -> isize {
        div as isize
    }
}

impl Default for SystemOscillatorOutput {
    fn default() -> Self {
        SystemOscillatorOutput::Disable
    }
}

pub struct Config {
    /// Set the power mode and system clock source
    pub mode: Mode,

    /// Clock divider for `CORE_CLK` and `SYS_CLK`.
    pub div_core: DivCore,

    /// Set the configuration of XTAL and EXTAL pins.
    pub system_oscillator: SystemOscillatorInput,

    /// Set the divider for the soscdiv1_clk
    ///
    /// This should be configured to 40MHz or less in RUN/HSRUN mode.
    pub soscdiv1: SystemOscillatorOutput,

    /// Set the divider for the soscdiv1_clk
    ///
    /// This should be configured to 40MHz or less in RUN/HSRUN mode.
    pub soscdiv2: SystemOscillatorOutput,
}

pub struct Spc {
    ScgBase: StaticRef<ScgRegisters>,
    SmcBase: StaticRef<SmcRegisters>,
    PmcBase: StaticRef<PmcRegisters>,
}

// impl Spc {
//     pub const fn new() -> Spc {
//         Spc {
//             ScgBase: SCG_BASE,
//             SmcBase: SMC_BASE,
//             PmcBase: PMC_BASE,
//         }
//     }
//     pub fn test() {
//         support::nop();
//     }
// }
impl Spc {
    /// Creates a new `CcmAnalog` peripheral
    pub const fn new() -> Self {
        Self {
            ScgBase: SCG_BASE,
            SmcBase: SMC_BASE,
            PmcBase: PMC_BASE,
        }
    }
}