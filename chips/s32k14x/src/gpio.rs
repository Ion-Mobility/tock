use cortexm4;
use cortexm4::support;
use cortexm4::support::atomic;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::hil;
use kernel::hil::gpio;
use kernel::platform::chip::ClockInterface;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;

use core::ops::{Index, IndexMut};
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::pcc;

/// General-purpose I/Os
#[repr(C)]
pub struct GpioRegisters {
    /// Pin Control Register n, array offset: 0x0, array step: 0x4
    pcr: [ReadWrite<u32, PCR::Register>; 32],
    /// Global Pin Control Low Register, offset: 0x80
    gpclr: ReadWrite<u32, GPCLR::Register>,
    /// Global Pin Control High Register, offset: 0x84
    gpchr: ReadWrite<u32, GPCHR::Register>,
    /// Global Interrupt Control Low Register, offset: 0x88
    giclr: ReadWrite<u32, GICLR::Register>,
    /// Global Interrupt Control High Register, offset: 0x8C
    gichr: ReadOnly<u32, GICHR::Register>,

    _reserved0: [u8; 16],

    /// Interrupt Status Flag Register, offset: 0xA0
    isfr: ReadWrite<u32, ISFR::Register>,

    _reserved1: [u8; 28],

    /// Digital Filter Enable Register, offset: 0xC0
    dfer: WriteOnly<u32, DFER::Register>,
    /// Digital Filter Clock Register, offset: 0xC4
    dfcr: ReadWrite<u32, DFCR::Register>,
    /// Digital Filter Width Register, offset: 0xC8
    dfwr: ReadWrite<u32, DFWR::Register>,
}

register_bitfields![u32,
    PCR [
        ISF OFFSET(24) NUMBITS(1) [],
        IRQC OFFSET(16) NUMBITS(4) [],
        LK OFFSET(15) NUMBITS(1) [],
        MUX OFFSET(8) NUMBITS(3) [],
        DSE OFFSET(6) NUMBITS(1) [],
        PFE OFFSET(4) NUMBITS(1) [],
        PE OFFSET(1) NUMBITS(1) [],
        PS OFFSET(0) NUMBITS(1) []
    ],

    GPCLR [
        GPWE OFFSET(16) NUMBITS(16) [],
        GPWD OFFSET(0) NUMBITS(16) []
    ],

    GPCHR [
        GPWE OFFSET(16) NUMBITS(16) [],
        GPWD OFFSET(0) NUMBITS(16) []
    ],

    GICLR [
        GIWD OFFSET(16) NUMBITS(16) [],
        GIWE OFFSET(0) NUMBITS(16) []
    ],

    GICHR [
        GIWD OFFSET(16) NUMBITS(16) [],
        GIWE OFFSET(0) NUMBITS(16) []
    ],

    ISFR [
        ISF OFFSET(0) NUMBITS(32) []
    ],

    DFER [
        DFE OFFSET(0) NUMBITS(32) []
    ],

    DFCR [
        CS OFFSET(0) NUMBITS(32) []
    ],
    DFWR [
        FILT OFFSET(0) NUMBITS(32) []
    ]
];

/// General-purpose I/Os
#[repr(C)]
pub struct IoRegisters {
    #[doc = "0x00 - Port Data Output Register"]
    pdor: ReadWrite<u32>,

    #[doc = "0x04 - Port Set Output Register"]
    psor: WriteOnly<u32>,

    #[doc = "0x08 - Port Clear Output Register"]
    pcor: WriteOnly<u32>,

    #[doc = "0x0c - Port Toggle Output Register"]
    ptor: WriteOnly<u32>,

    #[doc = "0x10 - Port Data Input Register"]
    pdir: ReadOnly<u32>,

    #[doc = "0x14 - Port Data Direction Register"]
    pddr: ReadWrite<u32>,

    #[doc = "0x18 - Port Input Disable Register"]
    pidr: ReadWrite<u32>,
}

const GPIO1_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_9000 as *const GpioRegisters) };
const GPIO2_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_A000 as *const GpioRegisters) };
const GPIO3_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_B000 as *const GpioRegisters) };
const GPIO4_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_C000 as *const GpioRegisters) };
const GPIO5_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_D000 as *const GpioRegisters) };

const IOA_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F000 as *const IoRegisters) };
const IOB_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F040 as *const IoRegisters) };
const IOC_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F080 as *const IoRegisters) };
const IOD_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F0C0 as *const IoRegisters) };
const IOE_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F100 as *const IoRegisters) };

enum_from_primitive! {
    /// Imxrt1050-evkb has 5 GPIO ports labeled from 1-5 [^1]. This is represented
    /// by three bits.
    ///
    /// [^1]: 12.5.1 GPIO memory map, page 1009 of the Reference Manual.
    #[repr(u16)]
    enum GpioPort {
        GPIO1 = 0b000,
        GPIO2 = 0b001,
        GPIO3 = 0b010,
        GPIO4 = 0b011,
        GPIO5 = 0b100,
    }
}

/// Creates a GPIO ID
///
/// Low 6 bits are the GPIO offset; the '17' in GPIO2[17]
/// Next 3 bits are the GPIO port; the '2' in GPIO2[17] (base 0 index, 2 -> 1)
const fn gpio_id(port: GpioPort, offset: u16) -> u16 {
    ((port as u16) << 6) | offset & 0x3F
}

/// GPIO Pin Identifiers
#[repr(u16)]
#[derive(Copy, Clone)]
pub enum PinId {
    // GPIO1
    PTA_00 = gpio_id(GpioPort::GPIO1, 0),
    PTA_01 = gpio_id(GpioPort::GPIO1, 1),
    PTA_02 = gpio_id(GpioPort::GPIO1, 2),
    PTA_03 = gpio_id(GpioPort::GPIO1, 3),
    PTA_04 = gpio_id(GpioPort::GPIO1, 4),
    PTA_05 = gpio_id(GpioPort::GPIO1, 5),
    PTA_06 = gpio_id(GpioPort::GPIO1, 6),
    PTA_07 = gpio_id(GpioPort::GPIO1, 7),
    PTA_08 = gpio_id(GpioPort::GPIO1, 8),
    PTA_09 = gpio_id(GpioPort::GPIO1, 9),
    PTA_10 = gpio_id(GpioPort::GPIO1, 10),
    PTA_11 = gpio_id(GpioPort::GPIO1, 11),
    PTA_12 = gpio_id(GpioPort::GPIO1, 12),
    PTA_13 = gpio_id(GpioPort::GPIO1, 13),
    PTA_14 = gpio_id(GpioPort::GPIO1, 14),
    PTA_15 = gpio_id(GpioPort::GPIO1, 15),
    PTA_16 = gpio_id(GpioPort::GPIO1, 16),
    PTA_17 = gpio_id(GpioPort::GPIO1, 17),
    PTA_18 = gpio_id(GpioPort::GPIO1, 18),
    PTA_19 = gpio_id(GpioPort::GPIO1, 19),
    PTA_20 = gpio_id(GpioPort::GPIO1, 20),
    PTA_21 = gpio_id(GpioPort::GPIO1, 21),
    PTA_22 = gpio_id(GpioPort::GPIO1, 22),
    PTA_23 = gpio_id(GpioPort::GPIO1, 23),
    PTA_24 = gpio_id(GpioPort::GPIO1, 24),
    PTA_25 = gpio_id(GpioPort::GPIO1, 25),
    PTA_26 = gpio_id(GpioPort::GPIO1, 26),
    PTA_27 = gpio_id(GpioPort::GPIO1, 27),
    PTA_28 = gpio_id(GpioPort::GPIO1, 28),
    PTA_29 = gpio_id(GpioPort::GPIO1, 29),
    PTA_30 = gpio_id(GpioPort::GPIO1, 30),
    PTA_31 = gpio_id(GpioPort::GPIO1, 31),

    // GPIO2
    PTB_00 = gpio_id(GpioPort::GPIO2, 0),
    PTB_01 = gpio_id(GpioPort::GPIO2, 1),
    PTB_02 = gpio_id(GpioPort::GPIO2, 2),
    PTB_03 = gpio_id(GpioPort::GPIO2, 3),
    PTB_04 = gpio_id(GpioPort::GPIO2, 4),
    PTB_05 = gpio_id(GpioPort::GPIO2, 5),
    PTB_06 = gpio_id(GpioPort::GPIO2, 6),
    PTB_07 = gpio_id(GpioPort::GPIO2, 7),
    PTB_08 = gpio_id(GpioPort::GPIO2, 8),
    PTB_09 = gpio_id(GpioPort::GPIO2, 9),
    PTB_10 = gpio_id(GpioPort::GPIO2, 10),
    PTB_11 = gpio_id(GpioPort::GPIO2, 11),
    PTB_12 = gpio_id(GpioPort::GPIO2, 12),
    PTB_13 = gpio_id(GpioPort::GPIO2, 13),
    PTB_14 = gpio_id(GpioPort::GPIO2, 14),
    PTB_15 = gpio_id(GpioPort::GPIO2, 15),
    PTB_16 = gpio_id(GpioPort::GPIO2, 16),
    PTB_17 = gpio_id(GpioPort::GPIO2, 17),
    PTB_18 = gpio_id(GpioPort::GPIO2, 18),
    PTB_19 = gpio_id(GpioPort::GPIO2, 19),
    PTB_20 = gpio_id(GpioPort::GPIO2, 20),
    PTB_21 = gpio_id(GpioPort::GPIO2, 21),
    PTB_22 = gpio_id(GpioPort::GPIO2, 22),
    PTB_23 = gpio_id(GpioPort::GPIO2, 23),
    PTB_24 = gpio_id(GpioPort::GPIO2, 24),
    PTB_25 = gpio_id(GpioPort::GPIO2, 25),
    PTB_26 = gpio_id(GpioPort::GPIO2, 26),
    PTB_27 = gpio_id(GpioPort::GPIO2, 27),
    PTB_28 = gpio_id(GpioPort::GPIO2, 28),
    PTB_29 = gpio_id(GpioPort::GPIO2, 29),
    PTB_30 = gpio_id(GpioPort::GPIO2, 30),
    PTB_31 = gpio_id(GpioPort::GPIO2, 31),

    // GPIO3
    PTC_00 = gpio_id(GpioPort::GPIO3, 0),
    PTC_01 = gpio_id(GpioPort::GPIO3, 1),
    PTC_02 = gpio_id(GpioPort::GPIO3, 2),
    PTC_03 = gpio_id(GpioPort::GPIO3, 3),
    PTC_04 = gpio_id(GpioPort::GPIO3, 4),
    PTC_05 = gpio_id(GpioPort::GPIO3, 5),
    PTC_06 = gpio_id(GpioPort::GPIO3, 6),
    PTC_07 = gpio_id(GpioPort::GPIO3, 7),
    PTC_08 = gpio_id(GpioPort::GPIO3, 8),
    PTC_09 = gpio_id(GpioPort::GPIO3, 9),
    PTC_10 = gpio_id(GpioPort::GPIO3, 10),
    PTC_11 = gpio_id(GpioPort::GPIO3, 11),
    PTC_12 = gpio_id(GpioPort::GPIO3, 12),
    PTC_13 = gpio_id(GpioPort::GPIO3, 13),
    PTC_14 = gpio_id(GpioPort::GPIO3, 14),
    PTC_15 = gpio_id(GpioPort::GPIO3, 15),
    PTC_16 = gpio_id(GpioPort::GPIO3, 16),
    PTC_17 = gpio_id(GpioPort::GPIO3, 17),
    PTC_18 = gpio_id(GpioPort::GPIO3, 18),
    PTC_19 = gpio_id(GpioPort::GPIO3, 19),
    PTC_20 = gpio_id(GpioPort::GPIO3, 20),
    PTC_21 = gpio_id(GpioPort::GPIO3, 21),
    PTC_22 = gpio_id(GpioPort::GPIO3, 22),
    PTC_23 = gpio_id(GpioPort::GPIO3, 23),
    PTC_24 = gpio_id(GpioPort::GPIO3, 24),
    PTC_25 = gpio_id(GpioPort::GPIO3, 25),
    PTC_26 = gpio_id(GpioPort::GPIO3, 26),
    PTC_27 = gpio_id(GpioPort::GPIO3, 27),
    PTC_28 = gpio_id(GpioPort::GPIO3, 28),
    PTC_29 = gpio_id(GpioPort::GPIO3, 29),
    PTC_30 = gpio_id(GpioPort::GPIO3, 30),
    PTC_31 = gpio_id(GpioPort::GPIO3, 31),

    // GPIO4
    PTD_00 = gpio_id(GpioPort::GPIO4, 0),
    PTD_01 = gpio_id(GpioPort::GPIO4, 1),
    PTD_02 = gpio_id(GpioPort::GPIO4, 2),
    PTD_03 = gpio_id(GpioPort::GPIO4, 3),
    PTD_04 = gpio_id(GpioPort::GPIO4, 4),
    PTD_05 = gpio_id(GpioPort::GPIO4, 5),
    PTD_06 = gpio_id(GpioPort::GPIO4, 6),
    PTD_07 = gpio_id(GpioPort::GPIO4, 7),
    PTD_08 = gpio_id(GpioPort::GPIO4, 8),
    PTD_09 = gpio_id(GpioPort::GPIO4, 9),
    PTD_10 = gpio_id(GpioPort::GPIO4, 10),
    PTD_11 = gpio_id(GpioPort::GPIO4, 11),
    PTD_12 = gpio_id(GpioPort::GPIO4, 12),
    PTD_13 = gpio_id(GpioPort::GPIO4, 13),
    PTD_14 = gpio_id(GpioPort::GPIO4, 14),
    PTD_15 = gpio_id(GpioPort::GPIO4, 15),
    PTD_16 = gpio_id(GpioPort::GPIO4, 16),
    PTD_17 = gpio_id(GpioPort::GPIO4, 17),
    PTD_18 = gpio_id(GpioPort::GPIO4, 18),
    PTD_19 = gpio_id(GpioPort::GPIO4, 19),
    PTD_20 = gpio_id(GpioPort::GPIO4, 20),
    PTD_21 = gpio_id(GpioPort::GPIO4, 21),
    PTD_22 = gpio_id(GpioPort::GPIO4, 22),
    PTD_23 = gpio_id(GpioPort::GPIO4, 23),
    PTD_24 = gpio_id(GpioPort::GPIO4, 24),
    PTD_25 = gpio_id(GpioPort::GPIO4, 25),
    PTD_26 = gpio_id(GpioPort::GPIO4, 26),
    PTD_27 = gpio_id(GpioPort::GPIO4, 27),
    PTD_28 = gpio_id(GpioPort::GPIO4, 28),
    PTD_29 = gpio_id(GpioPort::GPIO4, 29),
    PTD_30 = gpio_id(GpioPort::GPIO4, 30),
    PTD_31 = gpio_id(GpioPort::GPIO4, 31),

    // GPIO5
    PTE_00 = gpio_id(GpioPort::GPIO5, 0),
    PTE_01 = gpio_id(GpioPort::GPIO5, 1),
    PTE_02 = gpio_id(GpioPort::GPIO5, 2),
    PTE_03 = gpio_id(GpioPort::GPIO5, 3),
    PTE_04 = gpio_id(GpioPort::GPIO5, 4),
    PTE_05 = gpio_id(GpioPort::GPIO5, 5),
    PTE_06 = gpio_id(GpioPort::GPIO5, 6),
    PTE_07 = gpio_id(GpioPort::GPIO5, 7),
    PTE_08 = gpio_id(GpioPort::GPIO5, 8),
    PTE_09 = gpio_id(GpioPort::GPIO5, 9),
    PTE_10 = gpio_id(GpioPort::GPIO5, 10),
    PTE_11 = gpio_id(GpioPort::GPIO5, 11),
    PTE_12 = gpio_id(GpioPort::GPIO5, 12),
    PTE_13 = gpio_id(GpioPort::GPIO5, 13),
    PTE_14 = gpio_id(GpioPort::GPIO5, 14),
    PTE_15 = gpio_id(GpioPort::GPIO5, 15),
    PTE_16 = gpio_id(GpioPort::GPIO5, 16),
    PTE_17 = gpio_id(GpioPort::GPIO5, 17),
    PTE_18 = gpio_id(GpioPort::GPIO5, 18),
    PTE_19 = gpio_id(GpioPort::GPIO5, 19),
    PTE_20 = gpio_id(GpioPort::GPIO5, 20),
    PTE_21 = gpio_id(GpioPort::GPIO5, 21),
    PTE_22 = gpio_id(GpioPort::GPIO5, 22),
    PTE_23 = gpio_id(GpioPort::GPIO5, 23),
    PTE_24 = gpio_id(GpioPort::GPIO5, 24),
    PTE_25 = gpio_id(GpioPort::GPIO5, 25),
    PTE_26 = gpio_id(GpioPort::GPIO5, 26),
    PTE_27 = gpio_id(GpioPort::GPIO5, 27),
    PTE_28 = gpio_id(GpioPort::GPIO5, 28),
    PTE_29 = gpio_id(GpioPort::GPIO5, 29),
    PTE_30 = gpio_id(GpioPort::GPIO5, 30),
    PTE_31 = gpio_id(GpioPort::GPIO5, 31),
}

impl PinId {
    /// Returns the port
    fn port(self) -> GpioPort {
        GpioPort::from_u16((self as u16) >> 6).unwrap()
    }
    /// Returns the pin offset, half-closed range [0, 32)
    const fn offset(self) -> usize {
        (self as usize) & 0x3F
    }
}

/// GPIO pin mode
///
/// This describes the pin direction when it's a _GPIO pin_.
/// It does not describe the direction for other I/O, like LPI2C
/// or LPUART.
///
/// In order to set alternate functions such as LPI2C or LPUART,
/// you will need to use iomuxc enable_sw_mux_ctl_pad_gpio with
/// the specific MUX_MODE according to the reference manual (Chapter 11).
/// For the gpio mode, input or output we set the GDIR pin accordingly [^1]
///
/// [^1]: 12.4.3. GPIO Programming, page 1008 of the Reference Manual
pub enum Mode {
    Input = 0b00,
    Output = 0b01,
}

/// A GPIO port, like `GPIO3`
///
/// `Port`s contain collections of pins. Use `Port`s to access pin by their
/// GPIO offset. See the module-level docs for an example.
pub struct Port<'a, const N: usize> {
    registers: StaticRef<GpioRegisters>,
    io: StaticRef<IoRegisters>,
    clock: PortClock<'a>,
    pins: [Pin<'a>; N],
}

/// Implementation of a port, generic over the number of
/// pins
impl<'a, const N: usize> Port<'a, N> {
    const fn new(
        registers: StaticRef<GpioRegisters>,
        io: StaticRef<IoRegisters>,
        clock: PortClock<'a>,
        pins: [Pin<'a>; N],
    ) -> Self {
        Self {
            registers,
            io,
            clock,
            pins,
        }
    }

    pub fn is_enabled_clock(&self) -> bool {
        self.clock.is_enabled()
    }

    pub fn enable_clock(&self) {
        self.clock.enable();
    }

    pub fn disable_clock(&self) {
        self.clock.disable();
    }

    /// Returns the GPIO pin in this port
    ///
    /// This is an alterative API to [`Ports::pin`] that maps more closely
    /// to the GPIO offset.
    pub const fn pin(&self, offset: usize) -> &Pin<'a> {
        &self.pins[offset]
    }

    pub fn handle_interrupt(&self) {
        let imr_val: u32 = self.registers.isfr.get();

        // Read the `ISR` register and toggle the appropriate bits in
        // `isr`. Once that is done, write the value of `isr` back. We
        // can have a situation where memory value of `ISR` could have
        // changed due to an external interrupt. `ISR` is a read/clear write
        // 1 register (`rc_w1`). So, we only clear bits whose value has been
        // transferred to `isr`.
        let isr_val = unsafe {
            atomic(|| {
                let isr_val = self.registers.isfr.get();
                self.registers.isfr.set(isr_val);
                isr_val
            })
        };

        BitOffsets(isr_val)
            // Did we enable this interrupt?
            .filter(|offset| imr_val & (1 << offset) != 0)
            // Do we have a pin for that interrupt? (Likely)
            .filter_map(|offset| self.pins.get(offset as usize))
            // Call client
            .for_each(|pin| {
                pin.client.map(|client| client.fired());
            });
    }
}

type GPIO1<'a> = Port<'a, 32>;
type GPIO2<'a> = Port<'a, 32>;
type GPIO3<'a> = Port<'a, 32>;
type GPIO4<'a> = Port<'a, 32>;
type GPIO5<'a> = Port<'a, 32>;

impl<'a> Port<'a, 32> {
    const fn new_32(
        registers: StaticRef<GpioRegisters>,
        io: StaticRef<IoRegisters>,
        clock: PortClock<'a>,
    ) -> Self {
        Self::new(
            registers,
            io,
            clock,
            [
                Pin::new(registers, io, 00),
                Pin::new(registers, io, 01),
                Pin::new(registers, io, 02),
                Pin::new(registers, io, 03),
                Pin::new(registers, io, 04),
                Pin::new(registers, io, 05),
                Pin::new(registers, io, 06),
                Pin::new(registers, io, 07),
                Pin::new(registers, io, 08),
                Pin::new(registers, io, 09),
                Pin::new(registers, io, 10),
                Pin::new(registers, io, 11),
                Pin::new(registers, io, 12),
                Pin::new(registers, io, 13),
                Pin::new(registers, io, 14),
                Pin::new(registers, io, 15),
                Pin::new(registers, io, 16),
                Pin::new(registers, io, 17),
                Pin::new(registers, io, 18),
                Pin::new(registers, io, 19),
                Pin::new(registers, io, 20),
                Pin::new(registers, io, 21),
                Pin::new(registers, io, 22),
                Pin::new(registers, io, 23),
                Pin::new(registers, io, 24),
                Pin::new(registers, io, 25),
                Pin::new(registers, io, 26),
                Pin::new(registers, io, 27),
                Pin::new(registers, io, 28),
                Pin::new(registers, io, 29),
                Pin::new(registers, io, 30),
                Pin::new(registers, io, 31),
            ],
        )
    }
    const fn gpio1(pcc: &'a pcc::Pcc) -> GPIO1<'a> {
        Self::new_32(
            GPIO1_BASE,
            IOA_BASE,
            PortClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PCC_PORTA_INDEX,
            )),
        )
    }
    const fn gpio2(pcc: &'a pcc::Pcc) -> GPIO2<'a> {
        Self::new_32(
            GPIO2_BASE,
            IOB_BASE,
            PortClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PCC_PORTB_INDEX,
            )),
        )
    }
    const fn gpio3(pcc: &'a pcc::Pcc) -> GPIO3<'a> {
        Self::new_32(
            GPIO3_BASE,
            IOC_BASE,
            PortClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PCC_PORTC_INDEX,
            )),
        )
    }
    const fn gpio4(pcc: &'a pcc::Pcc) -> GPIO4<'a> {
        Self::new_32(
            GPIO4_BASE,
            IOD_BASE,
            PortClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PCC_PORTD_INDEX,
            )),
        )
    }
    const fn gpio5(pcc: &'a pcc::Pcc) -> GPIO5<'a> {
        Self::new_32(
            GPIO5_BASE,
            IOE_BASE,
            PortClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PCC_PORTE_INDEX,
            )),
        )
    }
}

/// All GPIO ports
///
/// Use [`new`](Ports::new) to create all GPIO ports, then use it to access GPIO
/// pins and individual ports. See the public members for the GPIO ports
#[non_exhaustive] // Fast GPIOs 6 through 9 not implemented
pub struct Ports<'a> {
    pub gpio1: GPIO1<'a>,
    pub gpio2: GPIO2<'a>,
    pub gpio3: GPIO3<'a>,
    pub gpio4: GPIO4<'a>,
    pub gpio5: GPIO5<'a>,
}

impl<'a> Ports<'a> {
    pub const fn new(pcc: &'a pcc::Pcc) -> Self {
        Self {
            gpio1: GPIO1::gpio1(pcc),
            gpio2: GPIO2::gpio2(pcc),
            gpio3: GPIO3::gpio3(pcc),
            gpio4: GPIO4::gpio4(pcc),
            gpio5: GPIO5::gpio5(pcc),
        }
    }

    /// Returns a GPIO pin
    ///
    /// For an interface that maps more closely to the numbers in
    /// `GPIO3[17]`, use a combination of the [`Ports`] members, and [`Port::pin()`].
    /// See the module-level docs for an example.
    pub fn pin(&self, pin: PinId) -> &Pin<'a> {
        match pin.port() {
            GpioPort::GPIO1 => &self.gpio1.pins[pin.offset()],
            GpioPort::GPIO2 => &self.gpio2.pins[pin.offset()],
            GpioPort::GPIO3 => &self.gpio3.pins[pin.offset()],
            GpioPort::GPIO4 => &self.gpio4.pins[pin.offset()],
            GpioPort::GPIO5 => &self.gpio5.pins[pin.offset()],
        }
    }
}

struct PortClock<'a>(pcc::PeripheralClock<'a>);

impl ClockInterface for PortClock<'_> {
    fn is_enabled(&self) -> bool {
        self.0.is_enabled()
    }

    fn enable(&self) {
        self.0.enable();
    }

    fn disable(&self) {
        self.0.disable();
    }
}

/// A GPIO pin, like `GPIO3[17]`
///
/// `Pin` implements the `hil::gpio` traits. To acquire a `Pin`,
///
/// - use [`Ports::pin`] to reference a `Pin` by a [`PinId`], or
/// - use a combination of the ports on [`Ports`], and [`Port::pin`]
pub struct Pin<'a> {
    registers: StaticRef<GpioRegisters>,
    io: StaticRef<IoRegisters>,
    offset: usize,
    client: OptionalCell<&'a dyn hil::gpio::Client>,
}

trait U32Ext {
    fn set_bit(self, offset: usize) -> Self;
    fn clear_bit(self, offset: usize) -> Self;
    fn is_bit_set(self, offset: usize) -> bool;
}

impl U32Ext for u32 {
    #[inline(always)]
    fn set_bit(self, offset: usize) -> u32 {
        self | (1 << offset)
    }
    #[inline(always)]
    fn clear_bit(self, offset: usize) -> u32 {
        self & !(1 << offset)
    }
    #[inline(always)]
    fn is_bit_set(self, offset: usize) -> bool {
        (self & (1 << offset)) != 0
    }
}

impl<'a> Pin<'a> {
    /// Fabricate a new `Pin` from a `PinId`
    // pub fn from_pin_id(pin_id: PinId) -> Self {
    //     Self::new(
    //         match pin_id.port() {
    //             GpioPort::GPIO1 => {
    //                 GPIO1_BASE,
    //                 IOA_BASE
    //             }
    //             GpioPort::GPIO2 => {
    //                 GPIO2_BASE,
    //                 IOB_BASE
    //             }
    //             GpioPort::GPIO3 => {
    //                 GPIO3_BASE,
    //                 IOC_BASE
    //             }
    //             GpioPort::GPIO4 => {
    //                 GPIO4_BASE,
    //                 IOD_BASE
    //             }
    //             GpioPort::GPIO5 => {
    //                 GPIO5_BASE,
    //                 IOE_BASE
    //             }
    //         },
    //         pin_id.offset(),
    //     )
    // }
    const fn new(
        registers: StaticRef<GpioRegisters>,
        io: StaticRef<IoRegisters>,
        offset: usize,
    ) -> Self {
        Pin {
            registers,
            io,
            offset,
            client: OptionalCell::empty(),
        }
    }

    fn get_mode(&self) -> Mode {
        if self.io.pidr.get().is_bit_set(self.offset) {
            Mode::Output
        } else {
            Mode::Input
        }
    }

    fn set_mode(&self, mode: Mode) {
        let pddr = self.io.pddr.get();
        let pddr = match mode {
            Mode::Input => pddr.clear_bit(self.offset),
            Mode::Output => pddr.set_bit(self.offset),
        };
        self.io.pddr.set(pddr);
    }

    fn set_output_high(&self) {
        self.io.psor.set(1 << self.offset);
    }

    fn set_output_low(&self) {
        self.io.pcor.set(1 << self.offset);
    }

    fn is_output_high(&self) -> bool {
        self.io.pdor.get().is_bit_set(self.offset)
    }

    fn toggle_output(&self) -> bool {
        self.io.ptor.set(1 << self.offset);
        self.is_output_high()
    }

    fn read_input(&self) -> bool {
        self.io.pdir.get().is_bit_set(self.offset)
    }

    fn mask_interrupt(&self) {
        // let imr = self.registers.imr.get();
        // let imr = imr.clear_bit(self.offset);
        // self.registers.pcr[self.offset].modify(PCR::IRQC::CLEAR);
    }

    fn unmask_interrupt(&self) {
        // let imr = self.registers.imr.get();
        // let imr = imr.set_bit(self.offset);
        // self.registers.imr.set(imr);
    }

    fn clear_pending(&self) {
        // self.registers.isr.set(1 << self.offset); // W1C
    }

    fn set_edge_sensitive(&self, sensitive: hil::gpio::InterruptEdge) {
        // use hil::gpio::InterruptEdge::*;
        // const RISING_EDGE_SENSITIVE: u32 = 0b10;
        // const FALLING_EDGE_SENSITIVE: u32 = 0b11;

        // let edge_sel = self.registers.edge_sel.get();
        // let icr_offset = (self.offset % 16) * 2;

        // let sensitive = match sensitive {
        //     EitherEdge => {
        //         let edge_sel = edge_sel.set_bit(self.offset);
        //         self.registers.edge_sel.set(edge_sel);
        //         // A high EDGE_SEL disregards the corresponding ICR[1|2] setting
        //         return;
        //     }
        //     RisingEdge => RISING_EDGE_SENSITIVE << icr_offset,
        //     FallingEdge => FALLING_EDGE_SENSITIVE << icr_offset,
        // };

        // let edge_sel = edge_sel.clear_bit(self.offset);
        // self.registers.edge_sel.set(edge_sel);

        // let icr_mask = 0b11 << icr_offset;
        // if self.offset < 16 {
        //     let icr1 = self.registers.icr1.get();
        //     let icr1 = (icr1 & !icr_mask) | sensitive;
        //     self.registers.icr1.set(icr1);
        // } else {
        //     let icr2 = self.registers.icr2.get();
        //     let icr2 = (icr2 & !icr_mask) | sensitive;
        //     self.registers.icr2.set(icr2);
        // }
    }
}

impl hil::gpio::Configure for Pin<'_> {
    fn make_output(&self) -> hil::gpio::Configuration {
        self.set_mode(Mode::Output);
        hil::gpio::Configuration::Output
    }

    fn make_input(&self) -> hil::gpio::Configuration {
        self.set_mode(Mode::Input);
        hil::gpio::Configuration::Input
    }

    fn deactivate_to_low_power(&self) {
        // Not implemented yet
    }

    fn disable_output(&self) -> hil::gpio::Configuration {
        // Not implemented yet
        hil::gpio::Configuration::LowPower
    }

    fn disable_input(&self) -> hil::gpio::Configuration {
        // Not implemented yet
        hil::gpio::Configuration::LowPower
    }

    // PullUp or PullDown mode are set through the Iomux module
    fn set_floating_state(&self, _mode: hil::gpio::FloatingState) {}

    fn floating_state(&self) -> hil::gpio::FloatingState {
        hil::gpio::FloatingState::PullNone
    }

    fn configuration(&self) -> hil::gpio::Configuration {
        match self.get_mode() {
            Mode::Input => hil::gpio::Configuration::Input,
            Mode::Output => hil::gpio::Configuration::Output,
        }
    }
}

impl hil::gpio::Output for Pin<'_> {
    fn set(&self) {
        self.set_output_high();
    }

    fn clear(&self) {
        self.set_output_low();
    }

    fn toggle(&self) -> bool {
        self.toggle_output()
    }
}

impl hil::gpio::Input for Pin<'_> {
    fn read(&self) -> bool {
        self.read_input()
    }
}

impl<'a> hil::gpio::Interrupt<'a> for Pin<'a> {
    fn enable_interrupts(&self, mode: hil::gpio::InterruptEdge) {
        unsafe {
            atomic(|| {
                // disable the interrupt
                self.mask_interrupt();
                self.clear_pending();
                self.set_edge_sensitive(mode);

                self.unmask_interrupt();
            });
        }
    }

    fn disable_interrupts(&self) {
        unsafe {
            atomic(|| {
                self.mask_interrupt();
                self.clear_pending();
            });
        }
    }

    fn set_client(&self, client: &'a dyn hil::gpio::Client) {
        self.client.set(client);
    }

    fn is_pending(&self) -> bool {
        self.registers.isfr.get().is_bit_set(self.offset)
    }
}

/// An iterator that returns the offsets of each high bit
///
/// Each offset is returned only once. There is no guarantee
/// for iteration order.
struct BitOffsets(u32);

impl Iterator for BitOffsets {
    type Item = u32;
    fn next(&mut self) -> Option<Self::Item> {
        if self.0 != 0 {
            let offset = self.0.trailing_zeros();
            self.0 &= self.0 - 1;
            Some(offset)
        } else {
            None
        }
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        let popcnt = self.0.count_ones() as usize;
        (popcnt, Some(popcnt))
    }
}

impl ExactSizeIterator for BitOffsets {}

#[cfg(test)]
mod tests {
    use super::BitOffsets;

    #[test]
    fn bit_offsets() {
        fn check(word: u32, expected: impl ExactSizeIterator<Item = u32>) {
            let offsets = BitOffsets(word);
            assert_eq!(offsets.len(), expected.len());
            assert!(
                offsets.eq(expected),
                "Incorrect bit offsets for word {:#b}",
                word
            );
        }

        check(0, core::iter::empty());
        check(u32::max_value(), 0..32);
        check(u32::max_value() >> 1, 0..31);
        check(u32::max_value() << 1, 1..32);
        check(0x5555_5555, (0..32).step_by(2));
        check(0xAAAA_AAAA, (0..32).skip(1).step_by(2));
    }
}
