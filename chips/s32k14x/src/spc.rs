use kernel::platform::chip::ClockInterface;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;

register_structs! {
    /// Clock Controller Module
    ScgRegisters {
        /**< Version ID Register, offset: 0x0 */
        (0x000 => verid: ReadOnly<u32, VERID::Register>),

        /**< Parameter Register, offset: 0x4 */
        (0x004 => param: ReadOnly<u32, PARAM::Register>),

        (0x008 => _reserved0),

        /**< Clock Status Register, offset: 0x10 */
        (0x010 => csr: ReadOnly<u32, CSR::Register>),

        /**< Run Clock Control Register, offset: 0x14 */
        (0x014 => rccr: ReadWrite<u32, RCCR::Register>),

        /**< VLPR Clock Control Register, offset: 0x18 */
        (0x018 => vccr: ReadWrite<u32, VCCR::Register>),

        /**< HSRUN Clock Control Register, offset: 0x1C */
        (0x01c => hccr: ReadWrite<u32, HCCR::Register>),

        /**< SCG CLKOUT Configuration Register, offset: 0x20 */
        (0x020 => clkoutcnfg: ReadWrite<u32, CLKOUTCNFG::Register>),

        (0x024 => _reserved1),

        /**< System OSC Control Status Register, offset: 0x100 */
        (0x100 => sosccsr: ReadWrite<u32, SOSCCSR::Register>),

        /**< System OSC Divide Register, offset: 0x104 */
        (0x104 => soscdiv: ReadWrite<u32, SOSCDIV::Register>),

        /**< System Oscillator Configuration Register, offset: 0x108 */
        (0x108 => sosccfg: ReadWrite<u32, SOSCCFG::Register>),

        (0x10C => _reserved2),

        /**< Slow IRC Control Status Register, offset: 0x200 */
        (0x200 => sirccsr: ReadWrite<u32, SIRCCSR::Register>),

        /**< Slow IRC Divide Register, offset: 0x204 */
        (0x204 => sircdiv: ReadWrite<u32, SIRCDIV::Register>),

        /**< Slow IRC Configuration Register, offset: 0x208 */
        (0x208 => sirccfg: ReadWrite<u32, SIRCCFG::Register>),

        (0x20C => _reserved3),

        /**< Fast IRC Control Status Register, offset: 0x300 */
        (0x300 => firccsr: ReadWrite<u32, FIRCCSR::Register>),

        /**< Fast IRC Divide Register, offset: 0x304 */
        (0x304 => fircdiv: ReadWrite<u32, FIRCDIV::Register>),

        /**< Fast IRC Configuration Register, offset: 0x308 */
        (0x308 => firccfg: ReadWrite<u32, FIRCCFG::Register>),

        (0x30C => _reserved4),

        /**< System PLL Control Status Register, offset: 0x600 */
        (0x600 => spllcsr: ReadWrite<u32, SPLLCSR::Register>),

        /**< System PLL Divide Register, offset: 0x604 */
        (0x604 => splldiv: ReadWrite<u32, SPLLDIV::Register>),

        /**< System PLL Configuration Register, offset: 0x608 */
        (0x608 => spllcfg: ReadWrite<u32, SPLLCFG::Register>),

        (0x60C => @END),
    }
}

const SCG_BASE: StaticRef<ScgRegisters> =
    unsafe { StaticRef::new(0x4006_4000 as *const ScgRegisters) };

pub struct Scg {
    registers: StaticRef<ScgRegisters>,
}

register_structs! {
    /// Clock Controller Module
    SmcRegisters {
        /**< SMC Version ID Register, offset: 0x0 */
        (0x000 => verid: ReadOnly<u32, VERID::Register>),

        /**< SMC Parameter Register, offset: 0x4 */
        (0x004 => param: ReadOnly<u32, PARAM::Register>),

        /**< Power Mode Protection register, offset: 0x8 */
        (0x008 => pmprot: ReadWrite<u32, PMPROT::Register>),

        /**< Power Mode Control register, offset: 0xC */
        (0x00C => pmctrl: ReadWrite<u32, PMCTRL::Register>),

        /**< Stop Control Register, offset: 0x10 */
        (0x010 => stopctrl: ReadWrite<u32, STOPCTRL::Register>),

        /**< Power Mode Status register, offset: 0x14 */
        (0x014 => pmstat: ReadOnly<u32, PMSTAT::Register>),

        (0x01C => @END),
    }
}

const SMC_BASE: StaticRef<SmcRegisters> =
    unsafe { StaticRef::new(0x4007_E000 as *const SmcRegisters) };

pub struct Smc {
    registers: StaticRef<SmcRegisters>,
}

register_structs! {
    /// Clock Controller Module
    PmcRegisters {
        /**< Low Voltage Detect Status and Control 1 Register, offset: 0x0 */
        (0x000 => lvdsc1: ReadWrite<u8, LVDSC1::Register>),

        /**< Low Voltage Detect Status and Control 2 Register, offset: 0x1 */
        (0x001 => lvdsc2: ReadWrite<u8, LVDSC2::Register>),

        /**< Regulator Status and Control Register, offset: 0x2 */
        (0x002 => regsc: ReadWrite<u8, REGSC::Register>),

        (0x003 => _reserved4),

        /**< Low Power Oscillator Trim Register, offset: 0x4 */
        (0x004 => lpotrim: ReadWrite<u8, LPOTRIM::Register>),

        (0x0005 => @END),
    }
}

const PMC_BASE: StaticRef<PmcRegisters> =
    unsafe { StaticRef::new(0x4007_D000 as *const PmcRegisters) };

pub struct Pmc {
    registers: StaticRef<PmcRegisters>,
}

pub const PCC_PCCn_COUNT: usize = 116;

#[repr(C)]
struct PccRegisters {
    /// Channel configuration registers, one per channel.
    PCCn: [ReadWrite<u32, ChannelConfiguration::Register>; PCC_PCCn_COUNT],
}

const PCC_BASE: StaticRef<PccRegisters> =
    unsafe { StaticRef::new(0x4006_5000 as *const PccRegisters) };

registers::register_bitfields![u32,
    ChannelConfiguration [
        PR OFFSET(31) NUMBITS(1) [],
        CGC OFFSET(30) NUMBITS(1) [],
    ]
];

impl Scg {
    pub const fn new() -> Scg {
        Scg {
            registers: SCG_BASE,
        }
    }

    pub fn set_low_power_mode(&self) {
        // self.registers.clpcr.modify(CLPCR::LPM.val(0b00 as u32));
    }

    /// Enable the DMA clock gate
    pub fn enable_dma_clock(&self) {
        // self.registers.ccgr[5].modify(CCGR::CG3.val(0b11));
    }

    /// Disable the DMA clock gate
    pub fn disable_dma_clock(&self) {
        // self.registers.ccgr[5].modify(CCGR::CG3.val(0b00));
    }

    /// Indicates if the DMA clock gate is enabled
    pub fn is_enabled_dma_clock(&self) -> bool {
        // self.registers.ccgr[5].read(CCGR::CG3) != 0
    }
}

impl Pcc {
    pub const fn new() -> Scg {
        Pcc {
            registers: PCC_BASE,
        }
    }
}

/// Clock selections for the main peripheral
#[derive(PartialEq, Eq)]
#[repr(u32)]
pub enum PeripheralClockSelection {
    /// Pre peripheral clock
    PrePeripheralClock,
    /// Peripheral clock 2, with some division
    PeripheralClock2Divided,
}

/// Pre-peripheral clock selections
#[derive(PartialEq, Eq)]
#[repr(u32)]
pub enum PrePeripheralClockSelection {
    Pll2,
    Pll2Pfd2,
    Pll2Pfd0,
    Pll1,
}

/// Peripheral clock 2 selection
#[derive(PartialEq, Eq)]
#[repr(u32)]
pub enum PeripheralClock2Selection {
    Pll3,
    Oscillator,
    Pll2Bypass,
}

enum ClockGate {
    CCGR0(HCLK0),
    CCGR1(HCLK1),
    CCGR2(HCLK2),
    CCGR3(HCLK3),
    CCGR4(HCLK4),
    CCGR5(HCLK5),
    CCGR6(HCLK6),
}

/// A peripheral clock gate
///
/// `PeripheralClock` provides a LPCG API for controlling peripheral
/// clock gates.
pub struct PeripheralClock<'a> {
    ccm: &'a Ccm,
    clock_gate: ClockGate,
}

impl<'a> PeripheralClock<'a> {
    pub const fn ccgr0(ccm: &'a Ccm, gate: HCLK0) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR0(gate),
        }
    }
    pub const fn ccgr1(ccm: &'a Ccm, gate: HCLK1) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR1(gate),
        }
    }
    pub const fn ccgr2(ccm: &'a Ccm, gate: HCLK2) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR2(gate),
        }
    }
    pub const fn ccgr3(ccm: &'a Ccm, gate: HCLK3) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR3(gate),
        }
    }
    pub const fn ccgr4(ccm: &'a Ccm, gate: HCLK4) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR4(gate),
        }
    }
    pub const fn ccgr5(ccm: &'a Ccm, gate: HCLK5) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR5(gate),
        }
    }
    pub const fn ccgr6(ccm: &'a Ccm, gate: HCLK6) -> Self {
        Self {
            ccm,
            clock_gate: ClockGate::CCGR6(gate),
        }
    }
}

pub enum HCLK0 {
    GPIO2,
    LPUART2,
    GPT2,
}

pub enum HCLK1 {
    GPIO1,
    GPIO5,
    GPT1, // and others ...
}
pub enum HCLK2 {
    LPI2C1,
    GPIO3,
    IOMUXCSNVS, // and others ...
}

pub enum HCLK3 {
    GPIO4,
    // and others ...
}

pub enum HCLK4 {
    IOMUXC,
    // and others ...
}

pub enum HCLK5 {
    LPUART1,
    DMA,
    // and others ...
}

pub enum HCLK6 {
    DCDC,
}

/// Periodic clock selection for GPTs and PITs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PerclkClockSel {
    /// IPG clock selection (default)
    IPG,
    /// Crystal oscillator
    Oscillator,
}

impl ClockInterface for PeripheralClock<'_> {
    fn is_enabled(&self) -> bool {
        true
    }

    fn enable(&self) {}

    fn disable(&self) {}
}
