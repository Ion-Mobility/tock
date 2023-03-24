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
    pub sosccfg: ReadWrite<u32, SOSCCFG::Register>,
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

#[repr(C)]
/// SIM Register
struct SimRegisters {
    _reserved0: [u8; 4],
    /**< Chip Control register, offset: 0x4 */
    pub chipctl: ReadWrite<u32, CHIPCTL::Register>,
    _reserved1: [u8; 4],
    /**< FTM Option Register 0, offset: 0xC */
    pub ftmopt0: ReadWrite<u32, FTMOPT0::Register>,
    /**< LPO Clock Select Register, offset: 0x10 */
    pub lpoclks: ReadWrite<u32, LPOCLKS::Register>,
    _reserved2: [u8; 4],
    /**< ADC Options Register, offset: 0x18 */
    pub adcopt: ReadWrite<u32, ADCOPT::Register>,
    /**< FTM Option Register 1, offset: 0x1C */
    pub ftmopt1: ReadWrite<u32, FTMOPT1::Register>,
    /**< Miscellaneous control register 0, offset: 0x20 */
    pub misctrl0: ReadWrite<u32, MISCTRL0::Register>,
    /**< System Device Identification Register, offset: 0x24 */
    pub sdid: ReadOnly<u32, SDID::Register>,
    _reserved3: [u8; 24],
    /**< Platform Clock Gating Control Register, offset: 0x40 */
    pub platcgc: ReadWrite<u32, PLATCGC::Register>,
    _reserved4: [u8; 8],
    /**< Flash Configuration Register 1, offset: 0x4C */
    pub fcfg1: ReadOnly<u32, FCFG1::Register>,
    _reserved5: [u8; 4],
    /**< Unique Identification Register High, offset: 0x54 */
    pub uidh: ReadOnly<u32, UIDH::Register>,
    /**< Unique Identification Register Mid-High, offset: 0x58 */
    pub uidmh: ReadOnly<u32, UIDMH::Register>,
    /**< Unique Identification Register Mid Low, offset: 0x5C */
    pub uidml: ReadOnly<u32, UIDML::Register>,
    /**< Unique Identification Register Low, offset: 0x60 */
    pub uidl: ReadOnly<u32, UIDL::Register>,
    _reserved6: [u8; 4],
    /**< System Clock Divider Register 4, offset: 0x68 */
    pub clkdiv4: ReadWrite<u32, CLKDIV4::Register>,
    /**< Miscellaneous Control register 1, offset: 0x6C */
    pub misctrl1: ReadWrite<u32, MISCTRL1::Register>,
}

const SIM_BASE: StaticRef<SimRegisters> =
    unsafe { StaticRef::new(0x4004_8000 as *const SimRegisters) };

pub struct Sim {
    registers: StaticRef<SimRegisters>,
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
        SCS     OFFSET(24) NUMBITS(4) [
            SYS_OSC = 1,
            SIRC = 2,
            FIRC = 3,
            SYS_PLL = 6,
            SRC_NONE = 255
        ],
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
    SOSCCFG [
        RANGE  OFFSET(4) NUMBITS(2) [],
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
        PMSTAT OFFSET(0)   NUMBITS(8) [
            RUN = 0x01,
            VLPR = 0x02,
            VLPS = 0x10,
            HSRUN = 0x80
        ],
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

register_bitfields![u32,
    CHIPCTL [
        PDB_BB_SEL_2        OFFSET(23) NUMBITS(1) [],
        PDB_BB_SEL_1        OFFSET(22) NUMBITS(1) [],
        SRAML_RETEN         OFFSET(21) NUMBITS(1) [],
        SRAMU_RETEN         OFFSET(20) NUMBITS(1) [],
        ADC_SUPPLYEN        OFFSET(19) NUMBITS(1) [],
        ADC_SUPPLY          OFFSET(16) NUMBITS(3) [],
        PDB_BB_SEL          OFFSET(13) NUMBITS(1) [],
        TRACECLK_SEL        OFFSET(12) NUMBITS(1) [],
        CLKOUTEN            OFFSET(11) NUMBITS(1) [],
        CLKOUTDIV           OFFSET(8) NUMBITS(3) [],
        CLKOUTSEL           OFFSET(4) NUMBITS(4) [],
        ADC_INTERLEAVE_EN   OFFSET(0) NUMBITS(3) [],
    ],
    FTMOPT0 [
        FTM3CLKSEL  OFFSET(30) NUMBITS(2) [],
        FTM2CLKSEL  OFFSET(28) NUMBITS(2) [],
        FTM1CLKSEL  OFFSET(26) NUMBITS(2) [],
        FTM0CLKSEL  OFFSET(24) NUMBITS(2) [],
        FTM7CLKSEL  OFFSET(22) NUMBITS(2) [],
        FTM6CLKSEL  OFFSET(20) NUMBITS(2) [],
        FTM5CLKSEL  OFFSET(18) NUMBITS(2) [],
        FTM4CLKSEL  OFFSET(16) NUMBITS(2) [],
        FTM3FLTxSEL OFFSET(12) NUMBITS(3) [],
        FTM2FLTxSEL OFFSET(8) NUMBITS(3) [],
        FTM1FLTxSEL OFFSET(4) NUMBITS(3) [],
        FTM0FLTxSEL OFFSET(0) NUMBITS(3) [],
    ],
    LPOCLKS [
        RTCCLKSEL   OFFSET(4) NUMBITS(2) [],
        LPOCLKSEL   OFFSET(2) NUMBITS(2) [],
        LPO32KCLKEN OFFSET(1) NUMBITS(1) [],
        LPO1KCLKEN  OFFSET(0) NUMBITS(1) [],
    ],
    ADCOPT [
        ADC1PRETRGSEL OFFSET(12) NUMBITS(2) [],
        ADC1SWPRETRG  OFFSET(9) NUMBITS(3) [],
        ADC1TRGSEL    OFFSET(8) NUMBITS(1) [],
        ADC0PRETRGSEL OFFSET(4) NUMBITS(2) [],
        ADC0SWPRETRG  OFFSET(1) NUMBITS(3) [],
        ADC0TRGSEL    OFFSET(0) NUMBITS(1) [],
    ],
    FTMOPT1 [
        FTM3_OUTSEL OFFSET(24) NUMBITS(8) [],
        FTM0_OUTSEL OFFSET(16) NUMBITS(8) [],
        FTMGLDOK    OFFSET(15) NUMBITS(1) [],
        FTM7SYNCBIT OFFSET(14) NUMBITS(1) [],
        FTM6SYNCBIT OFFSET(13) NUMBITS(1) [],
        FTM5SYNCBIT OFFSET(12) NUMBITS(1) [],
        FTM4SYNCBIT OFFSET(11) NUMBITS(1) [],
        FTM2CH1SEL  OFFSET(8) NUMBITS(1) [],
        FTM2CH0SEL  OFFSET(6) NUMBITS(2) [],
        FTM1CH0SEL  OFFSET(4) NUMBITS(2) [],
        FTM3SYNCBIT OFFSET(3) NUMBITS(1) [],
        FTM2SYNCBIT OFFSET(2) NUMBITS(1) [],
        FTM1SYNCBIT OFFSET(1) NUMBITS(1) [],
        FTM0SYNCBIT OFFSET(0) NUMBITS(1) [],
    ],
    MISCTRL0 [
        QSPI_CLK_SEL        OFFSET(26) NUMBITS(1) [],
        RMII_CLK_SEL        OFFSET(25) NUMBITS(1) [],
        RMII_CLK_OBE        OFFSET(24) NUMBITS(1) [],
        FTM7_OBE_CTRL       OFFSET(23) NUMBITS(1) [],
        FTM6_OBE_CTRL       OFFSET(22) NUMBITS(1) [],
        FTM5_OBE_CTRL       OFFSET(21) NUMBITS(1) [],
        FTM4_OBE_CTRL       OFFSET(20) NUMBITS(1) [],
        FTM3_OBE_CTRL       OFFSET(19) NUMBITS(1) [],
        FTM2_OBE_CTRL       OFFSET(18) NUMBITS(1) [],
        FTM1_OBE_CTRL       OFFSET(17) NUMBITS(1) [],
        FTM0_OBE_CTRL       OFFSET(16) NUMBITS(1) [],
        FTM_GTB_SPLIT_EN    OFFSET(14) NUMBITS(1) [],
        ECC_MGRAM_STAT      OFFSET(12) NUMBITS(1) [],
        ECC_EEERAM_STAT     OFFSET(11) NUMBITS(1) [],
        STOP2_MONITOR       OFFSET(10) NUMBITS(1) [],
        STOP1_MONITOR       OFFSET(9) NUMBITS(1) [],
    ],
    SDID [
        GENERATION OFFSET(28) NUMBITS(4) [],
        SUBSERIES  OFFSET(24) NUMBITS(4) [],
        DERIVATE   OFFSET(20) NUMBITS(4) [],
        RAMSIZE    OFFSET(16) NUMBITS(4) [],
        REVID      OFFSET(12) NUMBITS(4) [],
        PACKAGE    OFFSET(8) NUMBITS(4) [],
        FEATURES   OFFSET(0) NUMBITS(8) [],
    ],
    PLATCGC [
        CGCGPIO  OFFSET(5) NUMBITS(1) [],
        CGCEIM   OFFSET(4) NUMBITS(1) [],
        CGCERM   OFFSET(3) NUMBITS(1) [],
        CGCDMA   OFFSET(2) NUMBITS(1) [],
        CGCMPU   OFFSET(1) NUMBITS(1) [],
        CGCMSCM  OFFSET(0) NUMBITS(1) [],
    ],
    FCFG1 [
        EEERAMSIZE   OFFSET(16) NUMBITS(4) [],
        DEPART       OFFSET(12) NUMBITS(4) [],
    ],
    UIDH [
        UID127_96    OFFSET(0) NUMBITS(32) [],
    ],
    UIDMH [
        UID95_64    OFFSET(0) NUMBITS(32) [],
    ],
    UIDML [
        UID63_32    OFFSET(0) NUMBITS(32) [],
    ],
    UIDL [
        UID31_0    OFFSET(0) NUMBITS(32) [],
    ],
    CLKDIV4 [
        TRACEDIVEN OFFSET(28) NUMBITS(1) [],
        TRACEDIV   OFFSET(1) NUMBITS(3) [],
        TRACEFRAC  OFFSET(0) NUMBITS(1) [],
    ],
    MISCTRL1 [
        SW_TRG OFFSET(0) NUMBITS(1) [],
    ],
];

pub const NUMBER_OF_TCLK_INPUTS: usize = 3;
pub const FTM_INSTANCE_COUNT: usize = 8;

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

/// Clock divider for `CORE_CLK` and `SYS_CLK`.
pub enum ClockDiv {
    /// Clock disabled
    ClockDis = 0,
    /// Divide by 1
    ClockDivBy1 = 1,
    /// Divide by 2
    ClockDivBy2 = 2,
    /// Divide by 4
    ClockDivBy4 = 3,
    /// Divide by 8
    ClockDivBy8 = 4,
    /// Divide by 16
    ClockDivBy16 = 5,
    /// Divide by 32
    ClockDivBy32 = 6,
    /// Divide by 64
    ClockDivBy64 = 7,
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

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SIRCConfig {
    /// Slow IRC frequency range.               
    pub range: u8,

    /// Asynchronous peripheral source.         
    pub div1: AsyncClockDiv,

    /// Asynchronous peripheral source.         
    pub div2: AsyncClockDiv,

    /// Initialize or not the SIRC module.      
    pub initialize: bool,

    /// SIRC is enable or not in stop mode.     
    pub enableInStop: bool,

    /// SIRC is enable or not in low power mode.
    pub enableInLowPower: bool,

    /// SIRC Control Register can be written.   
    pub locked: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct FIRCConfig {
    /// Fast IRC frequency range.                 */
    pub range: u8,
    /// Asynchronous peripheral source.         
    pub div1: AsyncClockDiv,
    /// Asynchronous peripheral source.         
    pub div2: AsyncClockDiv,
    /// FIRC is enable or not in stop mode.       */
    pub enableInStop: bool,
    /// FIRC is enable or not in lowpower mode.   */
    pub enableInLowPower: bool,
    /// FIRC regulator is enable or not.          */
    pub regulator: bool,
    /// FIRC Control Register can be written.     */
    pub locked: bool,
    /// Initialize or not the FIRC module.        */
    pub initialize: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SOSCMonitorMode {
    /// Monitor disable.                          
    SCG_SOSC_MONITOR_DISABLE = 0,
    /// Interrupt when system OSC error detected.
    SCG_SOSC_MONITOR_INT = 1,
    /// Reset when system OSC error detected.     
    SCG_SOSC_MONITOR_RESET = 2,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SOSCRefSelect {
    /// External reference clock requested  
    SCG_SOSC_REF_EXT = 0x0,
    /// Internal oscillator of OSC requested
    SCG_SOSC_REF_OSC = 0x1,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SOSCGainMode {
    /// Configure crystal oscillator for low-power operation
    SCG_SOSC_GAIN_LOW = 0x0,
    /// Configure crystal oscillator for high-gain operation
    SCG_SOSC_GAIN_HIGH = 0x1,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SOSCRange {
    /// Medium frequency range selected for the crystal OSC (4 Mhz to 8 Mhz).
    SCG_SOSC_RANGE_MID = 2,
    /// High frequency range selected for the crystal OSC (8 Mhz to 40 Mhz).
    SCG_SOSC_RANGE_HIGH = 3,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SOSCConfig {
    /// System OSC frequency.                          
    pub req: u32,

    /// System OSC Clock monitor mode.                
    pub monitorMode: SOSCMonitorMode,

    /// System OSC External Reference Select.
    pub extRef: SOSCRefSelect,

    /// System OSC high-gain operation.               
    pub gain: SOSCGainMode,

    /// System OSC frequency range.                   
    pub range: SOSCRange,

    /// Asynchronous peripheral source.               
    pub div1: AsyncClockDiv,
    /// Asynchronous peripheral source.               
    pub div2: AsyncClockDiv,

    /// System OSC is enable or not in stop mode.     
    pub enableInStop: bool,
    /// System OSC is enable or not in low power mode.
    pub enableInLowPower: bool,

    /// System OSC Control Register can be written.   
    pub locked: bool,

    /// Initialize or not the System OSC module.      
    pub initialize: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SPLLMonitorMode {
    /// Monitor disable.                         
    SCG_SPLL_MONITOR_DISABLE = 0,
    /// Interrupt when system PLL error detected.
    SCG_SPLL_MONITOR_INT = 1,
    /// Reset when system PLL error detected.    
    SCG_SPLL_MONITOR_RESET = 2,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SPLLClockDiv {
    SCG_SPLL_CLOCK_PREDIV_BY_1 = 0,
    SCG_SPLL_CLOCK_PREDIV_BY_2 = 1,
    SCG_SPLL_CLOCK_PREDIV_BY_3 = 2,
    SCG_SPLL_CLOCK_PREDIV_BY_4 = 3,
    SCG_SPLL_CLOCK_PREDIV_BY_5 = 4,
    SCG_SPLL_CLOCK_PREDIV_BY_6 = 5,
    SCG_SPLL_CLOCK_PREDIV_BY_7 = 6,
    SCG_SPLL_CLOCK_PREDIV_BY_8 = 7,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SPLLClockMul {
    SCG_SPLL_CLOCK_MULTIPLY_BY_16 = 0,
    SCG_SPLL_CLOCK_MULTIPLY_BY_17 = 1,
    SCG_SPLL_CLOCK_MULTIPLY_BY_18 = 2,
    SCG_SPLL_CLOCK_MULTIPLY_BY_19 = 3,
    SCG_SPLL_CLOCK_MULTIPLY_BY_20 = 4,
    SCG_SPLL_CLOCK_MULTIPLY_BY_21 = 5,
    SCG_SPLL_CLOCK_MULTIPLY_BY_22 = 6,
    SCG_SPLL_CLOCK_MULTIPLY_BY_23 = 7,
    SCG_SPLL_CLOCK_MULTIPLY_BY_24 = 8,
    SCG_SPLL_CLOCK_MULTIPLY_BY_25 = 9,
    SCG_SPLL_CLOCK_MULTIPLY_BY_26 = 10,
    SCG_SPLL_CLOCK_MULTIPLY_BY_27 = 11,
    SCG_SPLL_CLOCK_MULTIPLY_BY_28 = 12,
    SCG_SPLL_CLOCK_MULTIPLY_BY_29 = 13,
    SCG_SPLL_CLOCK_MULTIPLY_BY_30 = 14,
    SCG_SPLL_CLOCK_MULTIPLY_BY_31 = 15,
    SCG_SPLL_CLOCK_MULTIPLY_BY_32 = 16,
    SCG_SPLL_CLOCK_MULTIPLY_BY_33 = 17,
    SCG_SPLL_CLOCK_MULTIPLY_BY_34 = 18,
    SCG_SPLL_CLOCK_MULTIPLY_BY_35 = 19,
    SCG_SPLL_CLOCK_MULTIPLY_BY_36 = 20,
    SCG_SPLL_CLOCK_MULTIPLY_BY_37 = 21,
    SCG_SPLL_CLOCK_MULTIPLY_BY_38 = 22,
    SCG_SPLL_CLOCK_MULTIPLY_BY_39 = 23,
    SCG_SPLL_CLOCK_MULTIPLY_BY_40 = 24,
    SCG_SPLL_CLOCK_MULTIPLY_BY_41 = 25,
    SCG_SPLL_CLOCK_MULTIPLY_BY_42 = 26,
    SCG_SPLL_CLOCK_MULTIPLY_BY_43 = 27,
    SCG_SPLL_CLOCK_MULTIPLY_BY_44 = 28,
    SCG_SPLL_CLOCK_MULTIPLY_BY_45 = 29,
    SCG_SPLL_CLOCK_MULTIPLY_BY_46 = 30,
    SCG_SPLL_CLOCK_MULTIPLY_BY_47 = 31,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum AsyncClockDiv {
    /// Clock output is disabled.
    SCG_ASYNC_CLOCK_DISABLE = 0,
    /// Divided by 1.             
    SCG_ASYNC_CLOCK_DIV_BY_1 = 1,
    /// Divided by 2.             
    SCG_ASYNC_CLOCK_DIV_BY_2 = 2,
    /// Divided by 4.             
    SCG_ASYNC_CLOCK_DIV_BY_4 = 3,
    /// Divided by 8.             
    SCG_ASYNC_CLOCK_DIV_BY_8 = 4,
    /// Divided by 16.            
    SCG_ASYNC_CLOCK_DIV_BY_16 = 5,
    /// Divided by 32.            
    SCG_ASYNC_CLOCK_DIV_BY_32 = 6,
    /// Divided by 64.            
    SCG_ASYNC_CLOCK_DIV_BY_64 = 7,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SPLLConfig {
    /// Clock monitor mode selected.                    
    pub monitorMode: SPLLMonitorMode,
    /// PLL reference clock divider.                    
    pub prediv: SPLLClockDiv,
    /// System PLL multiplier.                          
    pub mult: SPLLClockMul,
    /// System PLL source.                              
    pub src: u8,
    /// Asynchronous peripheral source.                 
    pub div1: AsyncClockDiv,
    /// Asynchronous peripheral source.                 
    pub div2: AsyncClockDiv,
    /// System PLL clock is enable or not in stop mode.
    pub enableInStop: bool,
    /// System PLL Control Register can be written.     
    pub locked: bool,
    /// Initialize or not the System PLL module.        
    pub initialize: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct RTCConfig {
    /// RTC_CLKIN frequency                   
    pub rtcClkInFreq: u32,
    /// Initialize or not the System PLL module.        
    pub initialize: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ClockOutSource {
    /// SCG SLOW.   
    SCG_CLOCKOUT_SRC_SCG_SLOW = 0,
    /// System OSC.
    SCG_CLOCKOUT_SRC_SOSC = 1,
    /// Slow IRC.   
    SCG_CLOCKOUT_SRC_SIRC = 2,
    /// Fast IRC.   
    SCG_CLOCKOUT_SRC_FIRC = 3,
    /// System PLL.
    SCG_CLOCKOUT_SRC_SPLL = 6,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct ClockOutConfig {
    /// ClockOut source select.
    pub source: ClockOutSource,
    /// Initialize or not the ClockOut     
    pub initialize: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SysClockDiv {
    /// Divided by 1
    SCG_SYSTEM_CLOCK_DIV_BY_1 = 0,
    /// Divided by 2
    SCG_SYSTEM_CLOCK_DIV_BY_2 = 1,
    /// Divided by 3
    SCG_SYSTEM_CLOCK_DIV_BY_3 = 2,
    /// Divided by 4
    SCG_SYSTEM_CLOCK_DIV_BY_4 = 3,
    /// Divided by 5
    SCG_SYSTEM_CLOCK_DIV_BY_5 = 4,
    /// Divided by 6
    SCG_SYSTEM_CLOCK_DIV_BY_6 = 5,
    /// Divided by 7
    SCG_SYSTEM_CLOCK_DIV_BY_7 = 6,
    /// Divided by 8
    SCG_SYSTEM_CLOCK_DIV_BY_8 = 7,
    /// Divided by 9
    SCG_SYSTEM_CLOCK_DIV_BY_9 = 8,
    /// Divided by 10
    SCG_SYSTEM_CLOCK_DIV_BY_10 = 9,
    /// Divided by 11
    SCG_SYSTEM_CLOCK_DIV_BY_11 = 10,
    /// Divided by 12
    SCG_SYSTEM_CLOCK_DIV_BY_12 = 11,
    /// Divided by 13
    SCG_SYSTEM_CLOCK_DIV_BY_13 = 12,
    /// Divided by 14
    SCG_SYSTEM_CLOCK_DIV_BY_14 = 13,
    /// Divided by 15
    SCG_SYSTEM_CLOCK_DIV_BY_15 = 14,
    /// Divided by 16
    SCG_SYSTEM_CLOCK_DIV_BY_16 = 15,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SysClockSource {
    /// System OSC.
    SCG_SYSTEM_CLOCK_SRC_SYS_OSC = 1,
    /// Slow IRC.   
    SCG_SYSTEM_CLOCK_SRC_SIRC = 2,
    /// Fast IRC.   
    SCG_SYSTEM_CLOCK_SRC_FIRC = 3,
    /// System PLL.
    SCG_SYSTEM_CLOCK_SRC_SYS_PLL = 6,
    /// MAX value.  
    SCG_SYSTEM_CLOCK_SRC_NONE = 255,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SystemClockConfig {
    /// Slow clock divider.      
    pub divSlow: SysClockDiv,
    /// BUS clock divider.       
    pub divBus: SysClockDiv,
    /// Core clock divider.      
    pub divCore: SysClockDiv,
    /// System clock source.     
    pub sysclksrc: SysClockSource,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SysAlterClockSource {
    /// System OSC
    SCG_SYSTEM_CLOCK_SRC_SYS_OSC = 1,
    /// Slow IRC.
    SCG_SYSTEM_CLOCK_SRC_SIRC = 2,
    /// Fast IRC.
    SCG_SYSTEM_CLOCK_SRC_FIRC = 3,
    /// System PLL
    SCG_SYSTEM_CLOCK_SRC_SYS_PLL = 6,
    /// MAX value.
    SCG_SYSTEM_CLOCK_SRC_NONE = 255,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct ClockModeConfig {
    /// Run Clock Control configuration.                 
    pub rccrConfig: SystemClockConfig,
    /// VLPR Clock Control configuration.                
    pub vccrConfig: SystemClockConfig,
    /// HSRUN Clock Control configuration.               
    pub hccrConfig: SystemClockConfig,
    /// Alternate clock used during initialization       
    pub alternateClock: SysAlterClockSource,
    /// Initialize or not the Clock Mode Configuration.  
    pub initialize: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SCGConfig {
    pub sircConfig: SIRCConfig,
    pub fircConfig: FIRCConfig,
    pub soscConfig: SOSCConfig,
    pub spllConfig: SPLLConfig,
    pub rtcConfig: RTCConfig,
    pub clockOutConfig: ClockOutConfig,
    pub clockModeConfig: ClockModeConfig,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SIMClockOutSource {
    /// SCG CLKOUT                                   
    SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT = 0,
    /// SOSC DIV2 CLK                                
    SIM_CLKOUT_SEL_SYSTEM_SOSC_DIV2_CLK = 2,
    /// SIRC DIV2 CLK                                
    SIM_CLKOUT_SEL_SYSTEM_SIRC_DIV2_CLK = 4,
    /// FIRC DIV2 CLK                                
    SIM_CLKOUT_SEL_SYSTEM_FIRC_DIV2_CLK = 6,
    /// HCLK                                         
    SIM_CLKOUT_SEL_SYSTEM_HCLK = 7,
    /// SPLL DIV2 CLK                                
    SIM_CLKOUT_SEL_SYSTEM_SPLL_DIV2_CLK = 8,
    /// BUS_CLK                                      
    SIM_CLKOUT_SEL_SYSTEM_BUS_CLK = 9,
    /// LPO_CLK 128 Khz                              
    SIM_CLKOUT_SEL_SYSTEM_LPO_128K_CLK = 10,
    /// LPO_CLK as selected by SIM LPO CLK Select    
    SIM_CLKOUT_SEL_SYSTEM_LPO_CLK = 12,
    /// RTC CLK as selected by SIM CLK 32 KHz Select
    SIM_CLKOUT_SEL_SYSTEM_RTC_CLK = 14,
    /// SFIF_CLK_HYP                                 
    SIM_CLKOUT_SEL_SYSTEM_SFIF_CLK_HYP = 5,
    /// IPG_CLK                                      
    SIM_CLKOUT_SEL_SYSTEM_IPG_CLK = 11,
    /// IPG_CLK_SFIF                                 
    SIM_CLKOUT_SEL_SYSTEM_IPG_CLK_SFIF = 13,
    /// IP_CLK_2XSFIF                                
    SIM_CLKOUT_SEL_SYSTEM_IPG_CLK_2XSFIF = 15,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SIMClockOutDiv {
    /// Divided by 1
    SIM_CLKOUT_DIV_BY_1 = 0x0,
    /// Divided by 2
    SIM_CLKOUT_DIV_BY_2 = 0x1,
    /// Divided by 3
    SIM_CLKOUT_DIV_BY_3 = 0x2,
    /// Divided by 4
    SIM_CLKOUT_DIV_BY_4 = 0x3,
    /// Divided by 5
    SIM_CLKOUT_DIV_BY_5 = 0x4,
    /// Divided by 6
    SIM_CLKOUT_DIV_BY_6 = 0x5,
    /// Divided by 7
    SIM_CLKOUT_DIV_BY_7 = 0x6,
    /// Divided by 8
    SIM_CLKOUT_DIV_BY_8 = 0x7,
}
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SimClockOutConfig {
    /// Initialize or not the ClockOut clock.  
    pub initialize: bool,
    /// SIM ClockOut enable.                   
    pub enable: bool,
    /// SIM ClockOut source select.            
    pub source: SIMClockOutSource,
    /// SIM ClockOut divide ratio.             
    pub divider: SIMClockOutDiv,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SimRtcClockSel {
    /// SOSCDIV1 clock          
    SIM_RTCCLK_SEL_SOSCDIV1_CLK = 0x0,
    /// 32 kHz LPO clock        
    SIM_RTCCLK_SEL_LPO_32K = 0x1,
    /// RTC_CLKIN clock         
    SIM_RTCCLK_SEL_RTC_CLKIN = 0x2,
    /// FIRCDIV1 clock          
    SIM_RTCCLK_SEL_FIRCDIV1_CLK = 0x3,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SimLpoClockSel {
    /// 128 kHz LPO clock
    SIM_LPO_CLK_SEL_LPO_128K = 0x0,
    /// No clock
    SIM_LPO_CLK_SEL_NO_CLOCK = 0x1,
    /// 32 kHz LPO clock which is divided by the 128 kHz LPO clock
    SIM_LPO_CLK_SEL_LPO_32K = 0x2,
    /// 1 kHz LPO clock which is divided by the 128 kHz LPO clock
    SIM_LPO_CLK_SEL_LPO_1K = 0x3,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct LpoClockConfig {
    /// Initialize or not the LPO clock.     
    pub initialize: bool,
    /// RTC_CLK source select.               
    pub sourceRtcClk: SimRtcClockSel,
    /// LPO clock source select.             
    pub sourceLpoClk: SimLpoClockSel,
    /// MSCM Clock Gating Control enable.    
    pub enableLpo32k: bool,
    /// MSCM Clock Gating Control enable.    
    pub enableLpo1k: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SimTCLKClockConfig {
    /// Initialize or not the TCLK clock.  
    pub initialize: bool,
    /// TCLKx frequency.                    
    pub tclkFreq: [u32; NUMBER_OF_TCLK_INPUTS],
    /// FTMx frequency.                    
    pub extPinSrc: [u32; FTM_INSTANCE_COUNT],
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SimPlatGateConfig {
    /// Initialize or not the Trace clock.  
    pub initialize: bool,
    /// MSCM Clock Gating Control enable.   
    pub enableMscm: bool,
    /// MPU Clock Gating Control enable.    
    pub enableMpu: bool,
    /// DMA Clock Gating Control enable.    
    pub enableDma: bool,
    /// ERM Clock Gating Control enable.    
    pub enableErm: bool,
    /// EIM Clock Gating Control enable.    
    pub enableEim: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum TraceClockSource {
    /// core clock     
    CLOCK_TRACE_SRC_CORE_CLK = 0x0,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SimTraceClockConfig {
    /// Initialize or not the Trace clock.  
    pub initialize: bool,
    /// Trace clock divider enable.         
    pub divEnable: bool,
    /// Trace clock select.                 
    pub source: TraceClockSource,
    /// Trace clock divider divisor.        
    pub divider: u8,
    /// Trace clock divider fraction.       
    pub divFraction: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SimQSPIRefClockGating {
    /// qspi internal reference clock gating control enable.
    pub enableQspiRefClk: bool,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SIMConfig {
    /// Clock Out configuration.           
    pub clockOutConfig: SimClockOutConfig,
    /// Low Power Clock configuration.     
    pub lpoClockConfig: LpoClockConfig,
    /// TCLK, FTM option Clock configuration.
    pub tclkConfig: SimTCLKClockConfig,
    /// Platform Gate Clock configuration.
    pub platGateConfig: SimPlatGateConfig,
    /// Trace clock configuration.         
    pub traceClockConfig: SimTraceClockConfig,
    /// Qspi Reference Clock Gating.       
    pub qspiRefClkGating: SimQSPIRefClockGating,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct PmcLpoClockConfig {
    /// Initialize or not the PMC LPO settings.
    pub initialize: bool,
    /// Enable/disable LPO     
    pub enable: bool,
    /// LPO trimming value     
    pub trimValue: u8,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct PMCConfig {
    pub lpoClockConfig: PmcLpoClockConfig,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct PCCConfig {}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct CLKUserConfig {
    pub scgconfig: SCGConfig,
    pub simconfig: SIMConfig,
    pub pccconfig: PCCConfig,
    pub pmcconfig: PMCConfig,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SysRunMode {
    /// Current mode.          
    SCG_SYSTEM_CLOCK_MODE_CURRENT = 0,
    /// Run mode.              
    SCG_SYSTEM_CLOCK_MODE_RUN = 1,
    /// Very Low Power Run mode
    SCG_SYSTEM_CLOCK_MODE_VLPR = 2,
    /// High Speed Run mode.   
    SCG_SYSTEM_CLOCK_MODE_HSRUN = 3,
    /// MAX value.             
    SCG_SYSTEM_CLOCK_MODE_NONE,
}

static mut g_xtal0ClkFreq: u32 = 0;

fn set_g_xtal0ClkFreq(inc: u32) {
    unsafe {
        g_xtal0ClkFreq = inc;
    }
}

fn get_g_xtal0ClkFreq() -> u32 {
    unsafe { g_xtal0ClkFreq }
}

static mut g_RtcClkInFreq: u32 = 0;

fn set_g_RtcClkInFreq(inc: u32) {
    unsafe {
        g_RtcClkInFreq = inc;
    } // Potential data race!
}
fn get_g_RtcClkInFreq() -> u32 {
    unsafe { g_RtcClkInFreq } // Potential data race!
}

/// System Power and Clock Controlling
pub struct Spc {
    ScgBase: StaticRef<ScgRegisters>,
    SmcBase: StaticRef<SmcRegisters>,
    PmcBase: StaticRef<PmcRegisters>,
    SimBase: StaticRef<SimRegisters>,
}

// pub const FEATURE_SCG_SIRC_HIGH_RANGE_FREQ (8000000U)
impl Spc {
    /// Creates a new `Spc` peripheral
    pub const fn new() -> Self {
        Self {
            ScgBase: SCG_BASE,
            SmcBase: SMC_BASE,
            PmcBase: PMC_BASE,
            SimBase: SIM_BASE,
        }
    }

    fn CLOCK_SYS_GetFircFreq(&self) -> u32 {
        if self.ScgBase.firccsr.is_set(FIRCCSR::FIRCVLD) {
            48000000
        } else {
            0
        }
    }
    fn CLOCK_SYS_GetSysOscFreq(&self) -> u32 {
        if self.SCG_GetSoscStatus() == true {
            get_g_xtal0ClkFreq()
        } else {
            0
        }
    }
    fn SCG_GetSircStatus(&self) -> bool {
        if self.ScgBase.sirccsr.is_set(SIRCCSR::SIRCVLD) {
            true
        } else {
            false
        }
    }
    fn SCG_GetSircRange(&self) -> bool {
        self.ScgBase.sirccfg.is_set(SIRCCFG::RANGE)
    }
    fn CLOCK_SYS_GetSircFreq(&self) -> u32 {
        if self.SCG_GetSircStatus() == true {
            8000000
        } else {
            0
        }
    }

    fn SCG_GetSoscStatus(&self) -> bool {
        if self.ScgBase.sosccsr.is_set(SOSCCSR::SOSCVLD) {
            true
        } else {
            false
        }
    }
    fn ConfigureFIRC(&self) {
        if self.GetFircSystemClockBusy() == true {
            unreachable!("Implemented all UART clock selections");
        } else {
            unreachable!("Implemented all UART clock selections");
        }
    }
    fn GetFircSystemClockBusy(&self) -> bool {
        if self.ScgBase.firccsr.is_set(FIRCCSR::FIRCSEL) {
            false
        } else {
            true
        }
    }

    fn SCG_SetFircConfiguration(&self, range: u8) {
        self.ScgBase
            .firccfg
            .modify(FIRCCFG::RANGE.val(range as u32));
    }

    fn SCG_SetSircControl(&self, enableInStop: bool, enableInLowPower: bool, lockMode: bool) {
        self.ScgBase.sirccsr.write(
            SIRCCSR::SIRCEN::SET
                + SIRCCSR::SIRCSTEN.val(enableInStop as u32)
                + SIRCCSR::SIRCLPEN.val(enableInLowPower as u32)
                + SIRCCSR::LK.val(lockMode as u32),
        );
    }

    fn SCG_ClearSircLock(&self) {
        self.ScgBase.sirccsr.modify(SIRCCSR::LK::CLEAR);
    }

    fn SCG_ClearSircControl(&self) {
        self.ScgBase.sirccsr.set(0);
    }

    fn SCG_SetSircAsyncConfig(&self, div1: AsyncClockDiv, div2: AsyncClockDiv) {
        self.ScgBase
            .sircdiv
            .write(SIRCDIV::SIRCDIV1.val(div1 as u32) + SIRCDIV::SIRCDIV2.val(div2 as u32));
    }

    fn SCG_SetSircConfiguration(&self, range: u32) {
        self.ScgBase
            .sirccfg
            .modify(SIRCCFG::RANGE.val(range as u32));
    }

    fn SCG_ClearSpllLock(&self) {
        self.ScgBase.spllcsr.modify(SPLLCSR::LK::CLEAR);
    }

    fn SCG_ClearSpllControl(&self) {
        self.ScgBase.spllcsr.set(0);
    }

    fn SCG_SetSpllAsyncConfig(&self, div1: AsyncClockDiv, div2: AsyncClockDiv) {
        self.ScgBase
            .splldiv
            .write(SPLLDIV::SPLLDIV1.val(div1 as u32) + SPLLDIV::SPLLDIV2.val(div2 as u32));
    }

    fn SCG_SetSpllConfiguration(&self, prediv: SPLLClockDiv, mul: SPLLClockMul) {
        self.ScgBase
            .spllcfg
            .write(SPLLCFG::PREDIV.val(prediv as u32) + SPLLCFG::MULT.val(mul as u32));
    }
    fn SCG_SetSpllControl(&self, monitorMode: SPLLMonitorMode, resetMode: bool, locked: bool) {
        self.ScgBase.spllcsr.write(
            SPLLCSR::SPLLEN::SET
                + SPLLCSR::SPLLCM.val(monitorMode as u32)
                + SPLLCSR::SPLLCMRE.val(resetMode as u32)
                + SPLLCSR::LK.val(locked as u32),
        );
    }
    fn SCG_GetSoscSystemClockMode(&self) -> bool {
        false
    }
    fn SCG_ClearSoscLock(&self) {
        self.ScgBase.sosccsr.modify(SOSCCSR::LK::CLEAR);
    }
    fn SCG_ClearSoscControl(&self) {
        self.ScgBase.sosccsr.set(0);
    }
    fn SCG_SetSoscAsyncConfig(&self, div1: AsyncClockDiv, div2: AsyncClockDiv) {
        self.ScgBase
            .soscdiv
            .write(SOSCDIV::SOSCDIV1.val(div1 as u32) + SOSCDIV::SOSCDIV2.val(div2 as u32));
    }

    fn SCG_SetSoscConfiguration(
        &self,
        range: SOSCRange,
        gain: SOSCGainMode,
        extref: SOSCRefSelect,
    ) {
        self.ScgBase.sosccfg.write(
            SOSCCFG::RANGE.val(range as u32)
                + SOSCCFG::HGO.val(gain as u32)
                + SOSCCFG::EREFS.val(extref as u32),
        );
    }

    fn SCG_SetSoscControl(&self, monitorMode: SOSCMonitorMode, resetMode: bool, lockMode: bool) {
        self.ScgBase.sosccsr.write(
            SOSCCSR::SOSCEN::SET
                + SOSCCSR::SOSCCM.val(monitorMode as u32)
                + SOSCCSR::SOSCCMRE.val(resetMode as u32)
                + SOSCCSR::LK.val(lockMode as u32),
        );
    }

    fn SMC_GetCurrentRunMode(&self) -> SysRunMode {
        use PMSTAT::PMSTAT::Value;
        match self.SmcBase.pmstat.read_as_enum(PMSTAT::PMSTAT) {
            Some(Value::RUN) => SysRunMode::SCG_SYSTEM_CLOCK_MODE_RUN,
            Some(Value::VLPR) => SysRunMode::SCG_SYSTEM_CLOCK_MODE_VLPR,
            Some(Value::VLPS) => SysRunMode::SCG_SYSTEM_CLOCK_MODE_NONE,
            Some(Value::HSRUN) => SysRunMode::SCG_SYSTEM_CLOCK_MODE_HSRUN,
            None => unreachable!("Unknow run mode!!!"),
        }
    }
    fn CLOCK_SYS_GetCurrentRunMode(&self) -> SysRunMode {
        let Result: SysRunMode = self.SMC_GetCurrentRunMode();
        Result
    }
    // freq /= (SCG_GetSpllPredivider(SCG) + SCG_SPLL_PREDIV_BASE);    /* Pre-divider. */
    // freq *= (SCG_GetSpllMultiplier(SCG) + SCG_SPLL_MULT_BASE);      /* Multiplier. */
    fn SCG_GetSpllPredivider(&self) -> u32 {
        let prediv: u32 = self.ScgBase.spllcfg.read(SPLLCFG::PREDIV);
        prediv
    }

    fn SCG_GetSpllMultiplier(&self) -> u32 {
        let mult: u32 = self.ScgBase.spllcfg.read(SPLLCFG::MULT);
        mult
    }
    fn SCG_GetSpllStatus(&self) -> bool {
        if self.ScgBase.spllcsr.is_set(SPLLCSR::SPLLVLD) {
            true
        } else {
            false
        }
    }
    fn CLOCK_SYS_GetSrcFreq(&self, source: SysClockSource) {
        unreachable!("T.B.D");
    }
    fn CLOCK_SYS_GetSysPllFreq(&self) -> u32 {
        let mut freq: u32 = self.CLOCK_SYS_GetSysOscFreq();
        if freq != 0 {
            let spllPreDiv: u32 = self.SCG_GetSpllPredivider();
            let spllMult: u32 = self.SCG_GetSpllMultiplier();
            freq /= spllPreDiv + 1;
            freq *= spllMult + 1;
            freq >> 1;
        }
        freq
    }
    fn SCG_SetRunClockControl(&self, config: SystemClockConfig) {
        self.ScgBase.rccr.write(
            RCCR::SCS.val(config.sysclksrc as u32)
                + RCCR::DIVCORE.val(config.divCore as u32)
                + RCCR::DIVBUS.val(config.divBus as u32)
                + RCCR::DIVSLOW.val(config.divSlow as u32),
        );
    }

    fn SCG_SetVlprClockControl(&self, config: SystemClockConfig) {
        self.ScgBase.vccr.write(
            VCCR::SCS.val(config.sysclksrc as u32)
                + VCCR::DIVCORE.val(config.divCore as u32)
                + VCCR::DIVBUS.val(config.divBus as u32)
                + VCCR::DIVSLOW.val(config.divSlow as u32),
        );
    }

    fn SCG_SetHsrunClockControl(&self, config: SystemClockConfig) {
        self.ScgBase.hccr.write(
            HCCR::SCS.val(config.sysclksrc as u32)
                + HCCR::DIVCORE.val(config.divCore as u32)
                + HCCR::DIVBUS.val(config.divBus as u32)
                + HCCR::DIVSLOW.val(config.divSlow as u32),
        );
    }
    fn SCG_GetCurrentSystemClockSource(&self) -> SysClockSource {
        use CSR::SCS::Value;
        match self.ScgBase.csr.read_as_enum(CSR::SCS) {
            Some(Value::SYS_OSC) => SysClockSource::SCG_SYSTEM_CLOCK_SRC_SYS_OSC,
            Some(Value::SIRC) => SysClockSource::SCG_SYSTEM_CLOCK_SRC_SIRC,
            Some(Value::FIRC) => SysClockSource::SCG_SYSTEM_CLOCK_SRC_FIRC,
            Some(Value::SYS_PLL) => SysClockSource::SCG_SYSTEM_CLOCK_SRC_SYS_PLL,
            Some(Value::SRC_NONE) => SysClockSource::SCG_SYSTEM_CLOCK_SRC_NONE,
            None => unreachable!("Implemented all UART clock selections"),
        }
    }

    fn SCG_SetClockoutSourceSel(&self, source: ClockOutSource) {
        self.ScgBase
            .clkoutcnfg
            .write(CLKOUTCNFG::CLKOUTSEL.val(source as u32));
    }

    fn CLOCK_SYS_SetSystemClockConfig(&self, mode: SysRunMode, config: SystemClockConfig) {
        let srcFreq: u32 = self.CLOCK_SYS_GetSysOscFreq();
        let sysFreqMul: u32 = config.divCore as u32 + 1;
        let busFreqMul: u32 = (config.divCore as u32 + 1) * (config.divBus as u32 + 1);
        let slowFreqMul: u32 = (config.divCore as u32 + 1) * (config.divSlow as u32 + 1);
        match mode {
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_RUN => {
                self.SCG_SetRunClockControl(config);
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_VLPR => {
                self.SCG_SetVlprClockControl(config);
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_HSRUN => {
                self.SCG_SetHsrunClockControl(config);
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_NONE => unreachable!("Unknow run mode!!!"),
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_CURRENT => unreachable!("Unknow run mode!!!"),
        }
    }

    fn CLOCK_SYS_TransitionSystemClock(&self, config: SystemClockConfig) {
        let cursysrunmode: SysRunMode = self.SMC_GetCurrentRunMode();
        self.CLOCK_SYS_SetSystemClockConfig(cursysrunmode, config);

        /// Wait for system clock to transition.
        loop {
            if self.SCG_GetCurrentSystemClockSource() == config.sysclksrc {
                break;
            }
        }
    }

    fn CLOCK_SYS_ConfigureSIRC(&self, config: SIRCConfig) {
        /* Fixme: Need to check if system current running from SIRRC */
        // SCG_GetSircSystemClockMode
        /* Clear LK bit field */
        // SCG_ClearSircLock(SCG);
        self.SCG_ClearSircLock();

        /* Disable monitor, disable clock and clear error. */
        // SCG_ClearSircControl(SCG);
        self.SCG_ClearSircControl();

        /* Now start to set up SIRC clock. */
        /* Step 1. Setup dividers. */
        // SCG_SetSircAsyncConfig(SCG, sircCfg->div1, sircCfg->div2);
        self.SCG_SetSircAsyncConfig(config.div1, config.div2);

        /* Step 2. Set SIRC configuration: frequency range. */
        // SCG_SetSircConfiguration(SCG, sircCfg->range);
        self.SCG_SetSircConfiguration(config.range as u32);

        /* Step 3. Set SIRC control: enable clock, configure source in STOP and VLP modes, configure lock feature. */
        // SCG_SetSircControl(SCG, sircCfg->enableInStop, sircCfg->enableInLowPower, sircCfg->locked);
        self.SCG_SetSircControl(config.enableInStop, config.enableInLowPower, config.locked);

        /* Wait for SIRC to initialize */
        // timeout = SIRC_STABILIZATION_TIMEOUT;
        loop {
            if self.CLOCK_SYS_GetSircFreq() != 0 {
                break;
            }
        }
    }

    fn CLOCK_SYS_ConfigureSPLL(&self, initialze: bool, spllcfg: SPLLConfig) {
        // /* Clear LK bit field */
        self.SCG_ClearSpllLock();

        // /* Disable monitor, disable clock and clear error. */
        self.SCG_ClearSpllControl();

        /// Get clock source frequency.
        let srcFreq: u32 = self.CLOCK_SYS_GetSysOscFreq();
        // DEV_ASSERT(srcFreq != 0U);

        // /* Pre-divider checking. */
        // let srcFreq /= (spllCfg.prediv + SCG_SPLL_PREDIV_BASE);
        // DEV_ASSERT((srcFreq >= SCG_SPLL_REF_MIN) && (srcFreq <= SCG_SPLL_REF_MAX));

        // /* Now start to set up PLL clock. */
        self.SCG_SetSpllAsyncConfig(spllcfg.div1, spllcfg.div2);

        // /* Step 2. Set PLL configuration. */
        self.SCG_SetSpllConfiguration(spllcfg.prediv, spllcfg.mult);

        // /* Step 3. Enable clock, configure monitor, lock register. */
        match spllcfg.monitorMode {
            SPLLMonitorMode::SCG_SPLL_MONITOR_DISABLE => {
                self.SCG_SetSpllControl(
                    SPLLMonitorMode::SCG_SPLL_MONITOR_DISABLE,
                    false,
                    spllcfg.locked,
                );
            }
            SPLLMonitorMode::SCG_SPLL_MONITOR_INT => {
                self.SCG_SetSpllControl(
                    SPLLMonitorMode::SCG_SPLL_MONITOR_INT,
                    false,
                    spllcfg.locked,
                );
            }
            SPLLMonitorMode::SCG_SPLL_MONITOR_RESET => {
                self.SCG_SetSpllControl(
                    SPLLMonitorMode::SCG_SPLL_MONITOR_RESET,
                    true,
                    spllcfg.locked,
                );
            }
        }

        // /* Wait for System PLL to initialize */
        loop {
            if self.CLOCK_SYS_GetSysPllFreq() != 0 {
                break;
            }
        }
    }

    fn CLOCK_SYS_ConfigureSOSC(&self, initialize: bool, soscCfg: SOSCConfig) {
        if self.SCG_GetSoscSystemClockMode() == true {
            unimplemented!("is is not supported yet");
        }

        // /* Clear LK bit field */
        // SCG_ClearSoscLock(SCG);
        self.SCG_ClearSoscLock();

        // /* Disable monitor, disable clock and clear error. */
        // SCG_ClearSoscControl(SCG);
        self.SCG_ClearSoscControl();

        // /* Now start to set up OSC clock. */
        // /* Step 1. Setup dividers. */
        // SCG_SetSoscAsyncConfig(SCG, soscCfg.div1, soscCfg.div2);
        self.SCG_SetSoscAsyncConfig(soscCfg.div1, soscCfg.div2);

        // /* Step 2. Set OSC configuration. */
        // SCG_SetSoscConfiguration(SCG, soscCfg.range, soscCfg.gain, soscCfg.extRef);
        self.SCG_SetSoscConfiguration(soscCfg.range, soscCfg.gain, soscCfg.extRef);

        // /* Step 3. Enable clock, configure monitor, lock register. */
        match soscCfg.monitorMode {
            SOSCMonitorMode::SCG_SOSC_MONITOR_DISABLE => {
                self.SCG_SetSoscControl(
                    SOSCMonitorMode::SCG_SOSC_MONITOR_DISABLE,
                    false,
                    soscCfg.locked,
                );
            }
            SOSCMonitorMode::SCG_SOSC_MONITOR_INT => {
                self.SCG_SetSoscControl(
                    SOSCMonitorMode::SCG_SOSC_MONITOR_INT,
                    false,
                    soscCfg.locked,
                );
            }
            SOSCMonitorMode::SCG_SOSC_MONITOR_RESET => {
                self.SCG_SetSoscControl(
                    SOSCMonitorMode::SCG_SOSC_MONITOR_RESET,
                    true,
                    soscCfg.locked,
                );
            }
        }

        // g_xtal0ClkFreq = soscCfg.req;
        set_g_xtal0ClkFreq(soscCfg.req as u32);
        // /* Wait for System OSC to initialize */
        loop {
            if self.CLOCK_SYS_GetSysOscFreq() != 0 {
                break;
            }
        }
    }

    fn CLOCK_SYS_ConfigureTemporarySystemClock(&self) {
        use CSR::SCS::Value;
        match self.ScgBase.csr.read_as_enum(CSR::SCS) {
            Some(Value::SYS_OSC)
            | Some(Value::SIRC)
            | Some(Value::SYS_PLL)
            | Some(Value::SRC_NONE) => {
                if self.CLOCK_SYS_GetFircFreq() == 0 {
                    /// If FIRC is not on, then FIRC is configured with the default configuration
                    self.ConfigureFIRC();
                }
                // FIRC is enabled, transition the system clock source to FIRC.
                let sysclockconfig: SystemClockConfig = SystemClockConfig {
                    sysclksrc: SysClockSource::SCG_SYSTEM_CLOCK_SRC_FIRC,
                    divCore: SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_1,
                    divBus: SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_2,
                    divSlow: SysClockDiv::SCG_SYSTEM_CLOCK_DIV_BY_4,
                };
                self.CLOCK_SYS_TransitionSystemClock(sysclockconfig);
                /* Wait for SIRC to initialize */
                // while ((CLOCK_SYS_GetSircFreq() == 0U) && (timeout > 0U))
                loop {
                    if self.CLOCK_SYS_GetFircFreq() != 0 {
                        break;
                    }
                }
            }
            None | Some(Value::FIRC) => {
                // unimplemented!("Mode::Run(RunMode::SIRC) is is not supported yet");
            }
        }
    }
    /* If the current system clock source is not FIRC:
     * 1. Enable FIRC (if it's not enabled)
     * 2. Switch to FIRC.
     */
    fn CLOCK_SYS_ConfigureModulesFromScg(&self, config: SCGConfig) {
        /// Configure all clock sources that are different from the, current system clock source FIRC (SIRC, SOSC, SPLL).
        /// SIRC
        self.CLOCK_SYS_ConfigureSIRC(config.sircConfig);
        /// SOSC
        self.CLOCK_SYS_ConfigureSOSC(config.soscConfig.initialize, config.soscConfig);
        /// SPLL
        self.CLOCK_SYS_ConfigureSPLL(config.spllConfig.initialize, config.spllConfig);
    }

    fn CLOCK_SYS_SetScgConfiguration(&self, config: SCGConfig) {
        /// Configure a temporary system clock source: FIRC
        self.CLOCK_SYS_ConfigureTemporarySystemClock();
        /// reset the value RTC_clk frequency.
        set_g_RtcClkInFreq(0);
        /// RTC Clock settings.
        if config.rtcConfig.initialize == true {
            set_g_RtcClkInFreq(config.rtcConfig.rtcClkInFreq);
        }
        /// ClockOut settings.
        if config.clockOutConfig.initialize == true {
            self.SCG_SetClockoutSourceSel(config.clockOutConfig.source);
        }

        /// Configure SCG ClockOut.
        self.CLOCK_SYS_ConfigureModulesFromScg(config);

        /* Configure SCG clock modes. */
        if config.clockModeConfig.initialize == true {
            self.CLOCK_SYS_SetSystemClockConfig(
                SysRunMode::SCG_SYSTEM_CLOCK_MODE_RUN,
                config.clockModeConfig.rccrConfig,
            );
            self.CLOCK_SYS_SetSystemClockConfig(
                SysRunMode::SCG_SYSTEM_CLOCK_MODE_VLPR,
                config.clockModeConfig.vccrConfig,
            );
            self.CLOCK_SYS_SetSystemClockConfig(
                SysRunMode::SCG_SYSTEM_CLOCK_MODE_HSRUN,
                config.clockModeConfig.hccrConfig,
            );
        }
    }

    fn SIM_SetClockout(&self, enable: bool, source: SIMClockOutSource, div: SIMClockOutDiv) {
        self.SimBase.chipctl.modify(CHIPCTL::CLKOUTEN::CLEAR);
        self.SimBase.chipctl.modify(
            CHIPCTL::CLKOUTEN.val(enable as u32)
                + CHIPCTL::CLKOUTSEL.val(source as u32)
                + CHIPCTL::CLKOUTDIV.val(div as u32),
        );
    }

    fn SIM_SetLpoClocks(
        &self,
        enableLpo1k: bool,
        enableLpo32k: bool,
        sourceRtc: SimLpoClockSel,
        sourceLpo: SimRtcClockSel,
    ) {
        self.SimBase.lpoclks.modify(
            LPOCLKS::LPO1KCLKEN.val(enableLpo1k as u32)
                + LPOCLKS::LPO32KCLKEN.val(enableLpo32k as u32)
                + LPOCLKS::LPOCLKSEL.val(sourceLpo as u32)
                + LPOCLKS::RTCCLKSEL.val(sourceRtc as u32),
        );
    }

    fn SIM_SetMscmClockGate(&self, enableMscm: bool) {
        self.SimBase
            .platcgc
            .modify(PLATCGC::CGCMSCM.val(enableMscm as u32));
    }

    fn SIM_SetMpuClockGate(&self, enableMpu: bool) {
        self.SimBase
            .platcgc
            .modify(PLATCGC::CGCMPU.val(enableMpu as u32));
    }

    fn SIM_SetDmaClockGate(&self, enableDma: bool) {
        self.SimBase
            .platcgc
            .modify(PLATCGC::CGCDMA.val(enableDma as u32));
    }

    fn SIM_SetErmClockGate(&self, enableErm: bool) {
        self.SimBase
            .platcgc
            .modify(PLATCGC::CGCERM.val(enableErm as u32));
    }

    fn SIM_SetEimClockGate(&self, enableEim: bool) {
        self.SimBase
            .platcgc
            .modify(PLATCGC::CGCEIM.val(enableEim as u32));
    }

    fn SIM_SetQspiIntRefClockGate(&self, enableQspiRefClk: bool) {
        self.SimBase
            .misctrl0
            .modify(MISCTRL0::QSPI_CLK_SEL.val(enableQspiRefClk as u32));
    }
    fn SIM_ClearTraceClockConfig(&self) {
        self.SimBase.clkdiv4.set(0);
    }
    fn SIM_SetTraceClockSource(&self, source: TraceClockSource) {
        self.SimBase
            .chipctl
            .modify(CHIPCTL::TRACECLK_SEL.val(source as u32));
    }
    fn SIM_SetTraceClockConfig(&self, enable: bool, divider: u8, mult: bool) {
        self.SimBase.clkdiv4.modify(
            CLKDIV4::TRACEDIVEN.val(enable as u32)
                + CLKDIV4::TRACEDIV.val(divider as u32)
                + CLKDIV4::TRACEFRAC.val(mult as u32),
        );
    }
    fn CLOCK_SYS_SetSimConfiguration(&self, simconfig: SIMConfig) {
        /* ClockOut settings. */
        if simconfig.clockOutConfig.initialize == true {
            self.SIM_SetClockout(
                simconfig.clockOutConfig.enable,
                simconfig.clockOutConfig.source,
                simconfig.clockOutConfig.divider,
            );
        }

        // /* Low Power Clock settings from SIM. */
        if simconfig.lpoClockConfig.initialize == true {
            self.SIM_SetLpoClocks(
                simconfig.lpoClockConfig.enableLpo1k,
                simconfig.lpoClockConfig.enableLpo32k,
                simconfig.lpoClockConfig.sourceLpoClk,
                simconfig.lpoClockConfig.sourceRtcClk,
            );
        }

        // /* Platform Gate Clock settings. */
        if simconfig.platGateConfig.initialize == true {
            self.SIM_SetMscmClockGate(simconfig.platGateConfig.enableMscm);
            self.SIM_SetMpuClockGate(simconfig.platGateConfig.enableMpu);
            self.SIM_SetDmaClockGate(simconfig.platGateConfig.enableDma);
            self.SIM_SetErmClockGate(simconfig.platGateConfig.enableErm);
            self.SIM_SetEimClockGate(simconfig.platGateConfig.enableEim);
            self.SIM_SetQspiIntRefClockGate(simconfig.qspiRefClkGating.enableQspiRefClk);
        }

        // /* TCLK Clock settings. */
        if simconfig.tclkConfig.initialize == true {
            //     for (i = 0; i < NUMBER_OF_TCLK_INPUTS; i++)
            //     {
            //         g_TClkFreq[i] = simClockConfig->tclkConfig.tclkFreq[i];
            //     }

            //     /* FTMOPT0 clock settings */
            //     for (i = 0; i < FTM_INSTANCE_COUNT; i++)
            //     {
            //         SIM_SetExtPinSourceFtm(SIM, i, simClockConfig->tclkConfig.extPinSrc[i]);
            //     }
        }

        // /* Debug trace Clock settings. */
        if simconfig.traceClockConfig.initialize == true {
            self.SIM_ClearTraceClockConfig();
            self.SIM_SetTraceClockSource(simconfig.traceClockConfig.source);

            self.SIM_SetTraceClockConfig(false, 0, false);

            if simconfig.traceClockConfig.divEnable == true {
                self.SIM_SetTraceClockConfig(
                    simconfig.traceClockConfig.divEnable,
                    simconfig.traceClockConfig.divider,
                    simconfig.traceClockConfig.divFraction,
                );
            }
        }
    }
    fn CLOCK_SYS_SetPccConfiguration(&self, pccconfig: PCCConfig) {}

    fn PMC_SetLpoMode(&self, enable: bool) {
        self.PmcBase.regsc.modify(REGSC::LPODIS.val(enable as u8));
    }

    fn PMC_SetLpoTrimValue(&self, decimalValue: u8) {
        let mut decVale: i8 = decimalValue as i8;
        let mut lpoTrim: i8 = 0;
        let mut trimValue: u8 = 0;
        if decVale < 0 {
            lpoTrim = 1 << 5;
            decVale = decVale + lpoTrim as i8;
        }
        trimValue = decVale as u8;
        self.PmcBase
            .lpotrim
            .modify(LPOTRIM::LPOTRIM.val(decVale as u8));
    }

    fn CLOCK_SYS_SetPmcConfiguration(&self, pmcconfig: PMCConfig) {
        /* Low Power Clock settings from PMC. */
        if pmcconfig.lpoClockConfig.initialize == true {
            /* Enable/disable the low power oscillator. */
            self.PMC_SetLpoMode(pmcconfig.lpoClockConfig.enable);
            /* Write trimming value. */
            self.PMC_SetLpoTrimValue(pmcconfig.lpoClockConfig.trimValue);
        }
    }
    pub fn init(&self, config: CLKUserConfig) {
        let CurRunningMode = self.CLOCK_SYS_GetCurrentRunMode();
        match CurRunningMode {
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_RUN => {
                /* Set SCG settings. */
                self.CLOCK_SYS_SetScgConfiguration(config.scgconfig);

                /* Set SIM settings. */
                self.CLOCK_SYS_SetSimConfiguration(config.simconfig);

                // /* Set PCC settings. */
                // self.CLOCK_SYS_SetPccConfiguration(config.pccconfig);

                // /* Set PMC settings. */
                self.CLOCK_SYS_SetPmcConfiguration(config.pmcconfig);
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_VLPR => {
                unreachable!("Unimplemented!!!");
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_NONE => {
                unreachable!("Unimplemented!!!");
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_HSRUN => {
                unreachable!("Unimplemented!!!");
            }
            SysRunMode::SCG_SYSTEM_CLOCK_MODE_CURRENT => {
                unreachable!("Unimplemented!!!");
            }
        }
    }

    pub fn scgconfig(&self, config: SCGConfig) {}

    pub fn config_mode(&self, config: Config) {
        match config.mode {
            Mode::Run(mode) => match mode {
                RunMode::SOSC => {
                    unimplemented!("Mode::Run(RunMode::SIRC) is is not supported yet");
                }
                RunMode::SIRC => {
                    unimplemented!("Mode::Run(RunMode::SIRC) is is not supported yet");
                }
                RunMode::FIRC => {
                    unimplemented!("Mode::Run(RunMode::SPLL) is is not supported yet");
                }
                RunMode::SPLL => {
                    unimplemented!("Mode::Run(RunMode::SPLL) is is not supported yet");
                }
            },
            Mode::HighSpeed(_mode) => {
                unimplemented!("High speed more is not supported yet");
            }
            Mode::VeryLowPower(_mode) => {
                unimplemented!("Very low power mode is not supported yet");
            }
        }
    }
}
