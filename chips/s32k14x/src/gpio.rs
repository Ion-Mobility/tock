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

const GPIOA_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_9000 as *const GpioRegisters) };
const GPIOB_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_A000 as *const GpioRegisters) };
const GPIOC_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_B000 as *const GpioRegisters) };
const GPIOD_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_C000 as *const GpioRegisters) };
const GPIOE_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x4004_D000 as *const GpioRegisters) };

const PINSA_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F000 as *const IoRegisters) };
const PINSB_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F040 as *const IoRegisters) };
const PINSC_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F080 as *const IoRegisters) };
const PINSD_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F0C0 as *const IoRegisters) };
const PINSE_BASE: StaticRef<IoRegisters> =
    unsafe { StaticRef::new(0x400F_F100 as *const IoRegisters) };

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

    PC00, PC01, PC02, PC03, PC04, PC05, PC06, PC07,
    PC08, PC09, PC10, PC11, PC12, PC13, PC14, PC15,
    PC16, PC17, PC18, PC19, PC20, PC21, PC22, PC23,
    PC24, PC25, PC26, PC27, PC28, PC29, PC30, PC31,

    PD00, PD01, PD02, PD03, PD04, PD05, PD06, PD07,
    PD08, PD09, PD10, PD11, PD12, PD13, PD14, PD15,
    PD16, PD17, PD18, PD19, PD20, PD21, PD22, PD23,
    PD24, PD25, PD26, PD27, PD28, PD29, PD30, PD31,

    PE00, PE01, PE02, PE03, PE04, PE05, PE06, PE07,
    PE08, PE09, PE10, PE11, PE12, PE13, PE14, PE15,
    PE16, PE17, PE18, PE19, PE20, PE21, PE22, PE23,
    PE24, PE25, PE26, PE27, PE28, PE29, PE30, PE31,

}
pub struct GpioPin<'a> {
    gpio_registers: StaticRef<GpioRegisters>,
    io_registers: StaticRef<IoRegisters>,
    pin: Pin,
    client: OptionalCell<&'a dyn gpio::Client>,
}

impl<'a> GpioPin<'a> {
    pub const fn new(
        gpio_base: StaticRef<GpioRegisters>,
        io_base: StaticRef<IoRegisters>,
        pin: Pin,
    ) -> GpioPin<'a> {
        GpioPin {
            gpio_registers: gpio_base,
            io_registers: io_base,
            pin,
            client: OptionalCell::empty(),
        }
    }

    fn handle_interrupt(&self) {
        support::nop();
    }
}

impl gpio::Configure for GpioPin<'_> {
    fn configuration(&self) -> gpio::Configuration {
        // if self.io_registers.pidr.read() {
        //     gpio::Configuration::Input
        // } else {
        //     gpio::Configuration::InputOutput
        // }
        gpio::Configuration::InputOutput
    }

    fn set_floating_state(&self, mode: gpio::FloatingState) {
        match mode {
            gpio::FloatingState::PullUp => {
                // self.iomux_registers.gpio[self.pin.shift]
                //     .modify(IO_MUX_GPIO::FUN_WPU::SET + IO_MUX_GPIO::MCU_WPU::SET);
                // self.iomux_registers.gpio[self.pin.shift]
                //     .modify(IO_MUX_GPIO::FUN_WPD::CLEAR + IO_MUX_GPIO::MCU_WPD::CLEAR);
            }
            gpio::FloatingState::PullDown => {
                // self.iomux_registers.gpio[self.pin.shift]
                //     .modify(IO_MUX_GPIO::FUN_WPU::CLEAR + IO_MUX_GPIO::MCU_WPU::CLEAR);
                // self.iomux_registers.gpio[self.pin.shift]
                //     .modify(IO_MUX_GPIO::FUN_WPD::SET + IO_MUX_GPIO::MCU_WPD::SET);
            }
            gpio::FloatingState::PullNone => {
                // self.iomux_registers.gpio[self.pin.shift]
                //     .modify(IO_MUX_GPIO::FUN_WPU::CLEAR + IO_MUX_GPIO::MCU_WPU::CLEAR);
                // self.iomux_registers.gpio[self.pin.shift]
                //     .modify(IO_MUX_GPIO::FUN_WPD::CLEAR + IO_MUX_GPIO::MCU_WPD::CLEAR);
            }
        }
    }

    fn floating_state(&self) -> gpio::FloatingState {
        // if self.iomux_registers.gpio[self.pin.shift].is_set(IO_MUX_GPIO::FUN_WPU) {
        //     gpio::FloatingState::PullUp
        // } else if self.iomux_registers.gpio[self.pin.shift].is_set(IO_MUX_GPIO::FUN_WPD) {
        //     gpio::FloatingState::PullDown
        if false {
            gpio::FloatingState::PullUp
        } else {
            gpio::FloatingState::PullNone
        }
    }

    fn deactivate_to_low_power(&self) {
        self.disable_input();
        self.disable_output();
    }

    fn make_output(&self) -> gpio::Configuration {
        // Setting peripheral index 128 causes GPIO_OUT_REG and GPIO_ENABLE_REG to enable for the given pin
        // self.registers.func_out_sel_cfg[self.pin.shift].set(0x80);

        // IO Mux function 1 on all pins is GPIO, no alternate peripherals (see table 5-2 in ESP32 datasheet)
        // self.iomux_registers.gpio[self.pin.shift].modify(IO_MUX_GPIO::MCU_SEL::FUN_1);

        // self.registers
        //     .enable_w1ts
        //     .set(self.pin.mask << self.pin.shift);
        gpio::Configuration::Output
    }

    fn disable_output(&self) -> gpio::Configuration {
        // self.registers
        //     .enable_w1tc
        //     .set(self.pin.mask << self.pin.shift);
        gpio::Configuration::Input
    }

    fn make_input(&self) -> gpio::Configuration {
        self.configuration()
    }

    fn disable_input(&self) -> gpio::Configuration {
        /* We can't do this from the GPIO controller.
         * It does look like the IO Mux is capable of this
         * though.
         */
        gpio::Configuration::Input
    }
}

impl gpio::Input for GpioPin<'_> {
    fn read(&self) -> bool {
        // self.io_registers.pdir.is_set(self.pin as u32)
        true
    }
}

impl gpio::Output for GpioPin<'_> {
    fn toggle(&self) -> bool {
        true
    }

    fn set(&self) {}

    fn clear(&self) {}
}

impl<'a> gpio::Interrupt<'a> for GpioPin<'a> {
    fn set_client(&self, client: &'a dyn gpio::Client) {
        self.client.set(client);
    }

    fn enable_interrupts(&self, mode: gpio::InterruptEdge) {
        // self.registers.pin[self.pin.shift].modify(PIN::INT_ENA::Enable);

        match mode {
            gpio::InterruptEdge::RisingEdge => {
                // self.registers.pin[self.pin.shift].modify(PIN::INT_TYPE::POSEDGE);
            }
            gpio::InterruptEdge::FallingEdge => {
                // self.registers.pin[self.pin.shift].modify(PIN::INT_TYPE::NEGEDGE);
            }
            gpio::InterruptEdge::EitherEdge => {
                // self.registers.pin[self.pin.shift].modify(PIN::INT_TYPE::ANYEDGE);
            }
        }

        // self.iomux_registers.gpio[self.pin.shift]
        //     .write(IO_MUX_GPIO::FUN_IE::SET + IO_MUX_GPIO::MCU_IE::SET);

        // self.registers.pin[self.pin.shift].modify(PIN::SYNC2_BYPASS::SET);
        // self.registers.pin[self.pin.shift].modify(PIN::SYNC1_BYPASS::SET);

        // self.registers
        //     .status_next
        //     .set(1 << self.pin.shift | self.registers.status_next.get());

        // self.registers.pin[self.pin.shift].modify(PIN::WAKEUP_ENABLE::SET);
    }

    fn disable_interrupts(&self) {
        // self.registers.pin[self.pin.shift].modify(PIN::INT_ENA::Disabled);
    }

    fn is_pending(&self) -> bool {
        // self.registers.status.is_set(self.pin)
        true
    }
}
struct PortClock<'a>(pcc::PeripheralClock<'a>);

pub struct Port<'a> {
    clock: PortClock<'a>,
    pins: [GpioPin<'a>; 4],
}

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

impl<'a> Port<'a> {
    pub const fn new(pcc: &'a pcc::Pcc) -> Self {
        Self {
            pins: [
                GpioPin::new(GPIOA_BASE, PINSA_BASE, Pin::PA00),
                GpioPin::new(GPIOA_BASE, PINSA_BASE, Pin::PA01),
                GpioPin::new(GPIOA_BASE, PINSA_BASE, Pin::PA02),
                GpioPin::new(GPIOA_BASE, PINSA_BASE, Pin::PA03),
            ],
            clock: PortClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PCC_PORTA_INDEX,
            )),
        }
    }

    pub fn handle_interrupt(&self) {
        // Determine the GPIO pin that triggered.
        // let pin = self.pins[0].registers.status.get().trailing_zeros() as usize;

        // self.pins[pin].handle_interrupt();
    }
}

impl<'a> Index<usize> for Port<'a> {
    type Output = GpioPin<'a>;

    fn index(&self, index: usize) -> &GpioPin<'a> {
        &self.pins[index]
    }
}

impl<'a> IndexMut<usize> for Port<'a> {
    fn index_mut(&mut self, index: usize) -> &mut GpioPin<'a> {
        &mut self.pins[index]
    }
}
