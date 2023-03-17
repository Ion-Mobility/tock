use cortexm4;
use cortexm4::support::atomic;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::hil;
use kernel::platform::chip::ClockInterface;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;

use crate::pcc;

/// General-purpose I/Os
#[repr(C)]
struct GpioRegisters {
    /// Pin Control Register n, array offset: 0x0, array step: 0x4
    pcr: [ReadWrite<u32, PCR::Register>; 32],
    /// GPIO port output type register
    gpclr: ReadWrite<u32, GPCLR::Register>,
    /// GPIO port output speed register
    gpchr: ReadWrite<u32, GPCHR::Register>,
    /// GPIO port pull-up/pull-down register
    giclr: ReadWrite<u32, GICLR::Register>,
    /// GPIO port input data register
    gichr: ReadOnly<u32, GICHR::Register>,

    _reserved0: [u8; 16],

    /// GPIO port output data register
    isfr: ReadWrite<u32, ISFR::Register>,

    _reserved1: [u8; 28],

    /// GPIO port bit set/reset register
    dfer: WriteOnly<u32, DFER::Register>,
    /// GPIO port configuration lock register
    dfcr: ReadWrite<u32, DFCR::Register>,
    /// GPIO alternate function low register
    dfwr: ReadWrite<u32, DFWR::Register>,
}

register_bitfields![u32,
    PCR [
        // Software Input On Field
        SION OFFSET(4) NUMBITS(1) [],
        // MUX Mode Select Field
        MUX_MODE OFFSET(0) NUMBITS(3) []
    ],

    GPCLR [
        // Hyst. Enable Field
        HYS OFFSET(16) NUMBITS(1) [],
        // Pull Up / Down Config Field
        PUS OFFSET(14) NUMBITS(2) [],
        // Pull / Keep Select Field
        PUE OFFSET(13) NUMBITS(1) [],
        // Pull / Keep enable field
        PKE OFFSET(12) NUMBITS(1) [],
        // Open drain enable field
        ODE OFFSET(11) NUMBITS(1) [],
        // Speed
        SPEED OFFSET(6) NUMBITS(2) [],
        // Drive Strength Field
        DSE OFFSET(3) NUMBITS(3) [],
        // Slew Rate Field
        SRE OFFSET(0) NUMBITS(1) []
    ],

    GPCHR [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(1) []
    ],

    GICLR [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(2) []
    ],

    GICHR [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(3) []
    ],
    ISFR [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(3) []
    ],
    DFER [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(3) []
    ],
    DFCR [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(3) []
    ],
    DFWR [
        // Selecting Pads Involved in Daisy Chain.
        DAISY OFFSET(0) NUMBITS(3) []
    ]
];

const GPIOE_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_D000 as *const GpioRegisters) };

const GPIOD_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_C000 as *const GpioRegisters) };

const GPIOC_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_B000 as *const GpioRegisters) };

const GPIOB_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_A000 as *const GpioRegisters) };

const GPIOA_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_9000 as *const GpioRegisters) };

/// STM32F446RE has eight GPIO ports labeled from A-H [^1]. This is represented
/// by three bits.
///
/// [^1]: Figure 3. STM32F446xC/E block diagram, page 16 of the datasheet
#[repr(u32)]
pub enum PortId {
    A = 0b000,
    B = 0b001,
    C = 0b010,
    D = 0b011,
    E = 0b100,
}
