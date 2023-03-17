use cortexm4;
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
struct GpioRegisters {
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

// const GPIOE_BASE: StaticRef<GpioRegisters> =
//     unsafe { StaticRef::new(0x4004_D000 as *const GpioRegisters) };

// const GPIOD_BASE: StaticRef<GpioRegisters> =
//     unsafe { StaticRef::new(0x4004_C000 as *const GpioRegisters) };

// const GPIOC_BASE: StaticRef<GpioRegisters> =
//     unsafe { StaticRef::new(0x4004_B000 as *const GpioRegisters) };

// const GPIOB_BASE: StaticRef<GpioRegisters> =
//     unsafe { StaticRef::new(0x4004_A000 as *const GpioRegisters) };

// const GPIOA_BASE: StaticRef<GpioRegisters> =
//     unsafe { StaticRef::new(0x4004_9000 as *const GpioRegisters) };

/// General-purpose I/Os
#[repr(C)]
struct PinRegisters {
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

// const PINSE_BASE: StaticRef<PinRegisters> =
//     unsafe { StaticRef::new(0x400F_F100 as *const PinRegisters) };

// const PINSD_BASE: StaticRef<PinRegisters> =
//     unsafe { StaticRef::new(0x400F_F0C0 as *const PinRegisters) };

// const PINSC_BASE: StaticRef<PinRegisters> =
//     unsafe { StaticRef::new(0x400F_F080 as *const PinRegisters) };

// const PINSB_BASE: StaticRef<PinRegisters> =
//     unsafe { StaticRef::new(0x400F_F040 as *const PinRegisters) };

// const PINSA_BASE: StaticRef<PinRegisters> =
//     unsafe { StaticRef::new(0x400F_F000 as *const PinRegisters) };

/// Peripheral functions that may be assigned to a `GPIOPin`.
///
/// GPIO pins on the S32K144 may serve multiple functions. In addition to the
/// default functionality, each pin can be assigned up to eight different
/// peripheral functions. The various functions for each pin are described in
/// "Peripheral Multiplexing I/O Lines" section of the S32K144 datasheet[^1].
///
/// [^1]: Section 3.2, pages 19-29
#[derive(Copy, Clone)]
pub enum PeripheralFunction {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

/// Name of the GPIO pin on the SAM4L.
///
/// The "Package and Pinout" section[^1] of the SAM4L datasheet shows the
/// mapping between these names and hardware pins on different chip packages.
///
/// [^1]: Section 3.1, pages 10-18
#[derive(Copy,Clone)]
#[rustfmt::skip]
pub enum Pin {
    PA00, PA01, PA02, PA03, PA04, PA05, PA06, PA07,
    PA08, PA09, PA10, PA11, PA12, PA13, PA14, PA15,
    PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23,
    PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31,

    PB00, PB01, PB02, PB03, PB04, PB05, PB06, PB07,
    PB08, PB09, PB10, PB11, PB12, PB13, PB14, PB15,
    PB16, PB17, PB18, PB19, PB20, PB21, PB22, PB23,
    PB24, PB25, PB26, PB27, PB28, PB29, PB30, PB31,
}

/// GPIO port that manages 32 pins.
///
/// The SAM4L divides GPIOs into _ports_ that each manage a group of 32
/// individual pins. There are up to three ports, depending particular chip
/// (see[^1]).
///
/// In general, the kernel and applications should care about individual
/// [GPIOPin](struct.GPIOPin.html)s. However, mirroring the hardware grouping in
/// Rust is useful, internally, for correctly handling and dispatching
/// interrupts.
///
/// The port itself is a set of 32-bit memory-mapped I/O registers. Each
/// register has a bit for each pin in the port. Pins are, thus, named by their
/// port and offset bit in each register that controls is. For example, the
/// first port has pins called "PA00" thru "PA31".
///
/// [^1]: SAM4L datasheet section 23.8 (page 573): "Module Configuration" for
///       GPIO
pub struct Port<'a> {
    port: StaticRef<GpioRegisters>,
    pin: StaticRef<PinRegisters>,
    pins: [GPIOPin<'a>; 32],
}

impl<'a> Port<'a> {
    pub const fn new_port_a() -> Self {
        Self {
            port: unsafe { StaticRef::new(0x4004_9000 as *const GpioRegisters) },
            pin: unsafe { StaticRef::new(0x400F_F000 as *const PinRegisters) },
            pins: [
                GPIOPin::new(Pin::PA00),
                GPIOPin::new(Pin::PA01),
                GPIOPin::new(Pin::PA02),
                GPIOPin::new(Pin::PA03),
                GPIOPin::new(Pin::PA04),
                GPIOPin::new(Pin::PA05),
                GPIOPin::new(Pin::PA06),
                GPIOPin::new(Pin::PA07),
                GPIOPin::new(Pin::PA08),
                GPIOPin::new(Pin::PA09),
                GPIOPin::new(Pin::PA10),
                GPIOPin::new(Pin::PA11),
                GPIOPin::new(Pin::PA12),
                GPIOPin::new(Pin::PA13),
                GPIOPin::new(Pin::PA14),
                GPIOPin::new(Pin::PA15),
                GPIOPin::new(Pin::PA16),
                GPIOPin::new(Pin::PA17),
                GPIOPin::new(Pin::PA18),
                GPIOPin::new(Pin::PA19),
                GPIOPin::new(Pin::PA20),
                GPIOPin::new(Pin::PA21),
                GPIOPin::new(Pin::PA22),
                GPIOPin::new(Pin::PA23),
                GPIOPin::new(Pin::PA24),
                GPIOPin::new(Pin::PA25),
                GPIOPin::new(Pin::PA26),
                GPIOPin::new(Pin::PA27),
                GPIOPin::new(Pin::PA28),
                GPIOPin::new(Pin::PA29),
                GPIOPin::new(Pin::PA30),
                GPIOPin::new(Pin::PA31),
            ],
        }
    }

    pub const fn new_port_b() -> Self {
        Self {
            port: unsafe { StaticRef::new(0x4004_A000 as *const GpioRegisters) },
            pin: unsafe { StaticRef::new(0x400F_F040 as *const PinRegisters) },
            pins: [
                GPIOPin::new(Pin::PB00),
                GPIOPin::new(Pin::PB01),
                GPIOPin::new(Pin::PB02),
                GPIOPin::new(Pin::PB03),
                GPIOPin::new(Pin::PB04),
                GPIOPin::new(Pin::PB05),
                GPIOPin::new(Pin::PB06),
                GPIOPin::new(Pin::PB07),
                GPIOPin::new(Pin::PB08),
                GPIOPin::new(Pin::PB09),
                GPIOPin::new(Pin::PB10),
                GPIOPin::new(Pin::PB11),
                GPIOPin::new(Pin::PB12),
                GPIOPin::new(Pin::PB13),
                GPIOPin::new(Pin::PB14),
                GPIOPin::new(Pin::PB15),
                GPIOPin::new(Pin::PB16),
                GPIOPin::new(Pin::PB17),
                GPIOPin::new(Pin::PB18),
                GPIOPin::new(Pin::PB19),
                GPIOPin::new(Pin::PB20),
                GPIOPin::new(Pin::PB21),
                GPIOPin::new(Pin::PB22),
                GPIOPin::new(Pin::PB23),
                GPIOPin::new(Pin::PB24),
                GPIOPin::new(Pin::PB25),
                GPIOPin::new(Pin::PB26),
                GPIOPin::new(Pin::PB27),
                GPIOPin::new(Pin::PB28),
                GPIOPin::new(Pin::PB29),
                GPIOPin::new(Pin::PB30),
                GPIOPin::new(Pin::PB31),
            ],
        }
    }

    pub fn handle_interrupt(&self) {
        let port: &GpioRegisters = &*self.port;
        let pin: &PinRegisters = &*self.pin;

        // Interrupt Flag Register (IFR) bits are only valid if the same bits
        // are enabled in Interrupt Enabled Register (IER).
        let mut fired = port.isfr.get() & port.gpchr.get();
        loop {
            let pin = fired.trailing_zeros() as usize;
            if pin > 32 {
                fired &= !(1 << pin);
                self.pins[pin].handle_interrupt();
                port.isfr.set(1 << pin);
            } else {
                break;
            }
        }
    }
}

pub struct GPIOPin<'a> {
    port: StaticRef<GpioRegisters>,
    pin_mask: u32,
    client: OptionalCell<&'a dyn hil::gpio::Client>,
}

impl<'a> GPIOPin<'a> {
    pub const fn new(pin: Pin) -> GPIOPin<'a> {
        GPIOPin {
            port: unsafe { StaticRef::new(0x4004_9000 as *const GpioRegisters) },
            pin_mask: 1 << ((pin as u32) % 32),
            client: OptionalCell::empty(),
        }
    }
    pub fn set_client(&self, client: &'a dyn gpio::Client) {
        self.client.set(client);
    }

    pub fn select_peripheral(&self, function: PeripheralFunction) {
        let f = function as u32;
        let (bit0, bit1, bit2) = (f & 0b1, (f & 0b10) >> 1, (f & 0b100) >> 2);
        let port: &GpioRegisters = &*self.port;

        // clear GPIO enable for pin
        // port.gper.clear.set(self.pin_mask);

        // Set PMR0-2 according to passed in peripheral
        if bit0 == 0 {
            // port.pmr0.clear.set(self.pin_mask);
        } else {
            // port.pmr0.set.set(self.pin_mask);
        }
        if bit1 == 0 {
            // port.pmr1.clear.set(self.pin_mask);
        } else {
            // port.pmr1.set.set(self.pin_mask);
        }
        if bit2 == 0 {
            // port.pmr2.clear.set(self.pin_mask);
        } else {
            // port.pmr2.set.set(self.pin_mask);
        }
    }

    pub fn enable(&self) {}

    pub fn disable(&self) {}

    pub fn is_pending(&self) -> bool {
        true
    }

    pub fn enable_output(&self) {}

    pub fn disable_output(&self) {}

    pub fn enable_pull_down(&self) {}

    pub fn disable_pull_down(&self) {}

    pub fn enable_pull_up(&self) {}

    pub fn disable_pull_up(&self) {}
    /// Sets the interrupt mode registers. Interrupts may fire on the rising or
    /// falling edge of the pin or on both.
    ///
    /// The mode is a two-bit value based on the mapping from section 23.7.13 of
    /// the SAM4L datasheet (page 563):
    ///
    /// | `mode` value | Interrupt Mode |
    /// | ------------ | -------------- |
    /// | 0b00         | Pin change     |
    /// | 0b01         | Rising edge    |
    /// | 0b10         | Falling edge   |
    ///
    pub fn set_interrupt_mode(&self, mode: u8) {
        let port: &GpioRegisters = &*self.port;
        if mode & 0b01 != 0 {
            // port.imr0.set.set(self.pin_mask);
        } else {
            // port.imr0.clear.set(self.pin_mask);
        }

        if mode & 0b10 != 0 {
            // port.imr1.set.set(self.pin_mask);
        } else {
            // port.imr1.clear.set(self.pin_mask);
        }
    }

    pub fn enable_interrupt(&self) {}

    pub fn disable_interrupt(&self) {}

    pub fn handle_interrupt(&self) {}

    pub fn disable_schmidtt_trigger(&self) {}

    pub fn enable_schmidtt_trigger(&self) {}

    pub fn read(&self) -> bool {
        true
    }

    pub fn toggle(&self) -> bool {
        true
    }

    pub fn set(&self) {}

    pub fn clear(&self) {}
}

impl<'a> hil::Controller for GPIOPin<'a> {
    type Config = Option<PeripheralFunction>;

    fn configure(&self, config: Self::Config) {
        match config {
            Some(c) => self.select_peripheral(c),
            None => self.enable(),
        }
    }
}

impl<'a> gpio::Configure for GPIOPin<'a> {
    fn set_floating_state(&self, mode: gpio::FloatingState) {
        match mode {
            gpio::FloatingState::PullUp => {
                self.disable_pull_down();
                self.enable_pull_up();
            }
            gpio::FloatingState::PullDown => {
                self.disable_pull_up();
                self.enable_pull_down();
            }
            gpio::FloatingState::PullNone => {
                self.disable_pull_up();
                self.disable_pull_down();
            }
        }
    }

    fn deactivate_to_low_power(&self) {
        GPIOPin::disable(self);
    }

    fn make_output(&self) -> gpio::Configuration {
        self.enable();
        GPIOPin::enable_output(self);
        self.disable_schmidtt_trigger();
        gpio::Configuration::Output
    }

    fn make_input(&self) -> gpio::Configuration {
        self.enable();
        GPIOPin::disable_output(self);
        self.enable_schmidtt_trigger();
        gpio::Configuration::Input
    }

    fn disable_output(&self) -> gpio::Configuration {
        let port: &GpioRegisters = &*self.port;
        // port.oder.clear.set(self.pin_mask);
        self.configuration()
    }

    fn disable_input(&self) -> gpio::Configuration {
        self.configuration()
    }

    fn is_input(&self) -> bool {
        let port: &GpioRegisters = &*self.port;
        // port.gper.get() & self.pin_mask != 0
        true
    }

    fn is_output(&self) -> bool {
        let port: &GpioRegisters = &*self.port;
        // port.oder.get() & self.pin_mask != 0
        true
    }

    fn floating_state(&self) -> gpio::FloatingState {
        let port: &GpioRegisters = &*self.port;
        let down = (port.gpchr.get() & self.pin_mask) != 0;
        let up = (port.gpchr.get() & self.pin_mask) != 0;
        if down {
            gpio::FloatingState::PullDown
        } else if up {
            gpio::FloatingState::PullUp
        } else {
            gpio::FloatingState::PullNone
        }
    }

    fn configuration(&self) -> gpio::Configuration {
        let port: &GpioRegisters = &*self.port;
        let input = self.is_input();
        let output = self.is_output();
        let gpio = (port.gpchr.get() & self.pin_mask) == 1;
        let config = (gpio, input, output);
        match config {
            (false, _, _) => gpio::Configuration::Function,
            (true, false, false) => gpio::Configuration::Other,
            (true, false, true) => gpio::Configuration::Output,
            (true, true, false) => gpio::Configuration::Input,
            (true, true, true) => gpio::Configuration::InputOutput,
        }
    }
}

impl<'a> gpio::Input for GPIOPin<'a> {
    fn read(&self) -> bool {
        GPIOPin::read(self)
    }
}

impl<'a> gpio::Output for GPIOPin<'a> {
    fn toggle(&self) -> bool {
        GPIOPin::toggle(self)
    }

    fn set(&self) {
        GPIOPin::set(self);
    }

    fn clear(&self) {
        GPIOPin::clear(self);
    }
}

impl<'a> gpio::Interrupt<'a> for GPIOPin<'a> {
    fn enable_interrupts(&self, mode: gpio::InterruptEdge) {
        let mode_bits = match mode {
            hil::gpio::InterruptEdge::EitherEdge => 0b00,
            hil::gpio::InterruptEdge::RisingEdge => 0b01,
            hil::gpio::InterruptEdge::FallingEdge => 0b10,
        };
        GPIOPin::set_interrupt_mode(self, mode_bits);
        GPIOPin::enable_interrupt(self);
    }

    fn disable_interrupts(&self) {
        GPIOPin::disable_interrupt(self);
    }

    fn set_client(&self, client: &'a dyn gpio::Client) {
        GPIOPin::set_client(self, client);
    }

    fn is_pending(&self) -> bool {
        GPIOPin::is_pending(self)
    }
}
