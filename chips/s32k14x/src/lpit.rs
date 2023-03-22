use core::sync::atomic::{AtomicU32, Ordering};
use cortexm4;
use cortexm4::support;
use cortexm4::support::atomic;
use kernel::hil;
use kernel::hil::time::{Ticks, Ticks32, Time};
use kernel::platform::chip::ClockInterface;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use crate::nvic;
use crate::pcc;

pub const LPIT_TMR_COUNT: usize = 4;

/// General purpose timers
#[repr(C)]
struct TimerRegisters {
    /// Timer Value Register, array offset: 0x20, array step: 0x10
    tval: ReadWrite<u32, TVAL::Register>,
    /// Current Timer Value, array offset: 0x24, array step: 0x10
    cval: ReadWrite<u32, CVAL::Register>,
    /// Timer Control Register, array offset: 0x28, array step: 0x10
    tctrl: ReadWrite<u32, TCTRL::Register>,

    /// Reserved 0
    _reserved0: [u8; 4],
}

register_bitfields![u32,
    TVAL [
        TMR_VAL OFFSET(0)   NUMBITS(32)
    ],
    CVAL [
        TMR_CUR_VAL OFFSET(0)   NUMBITS(32)
    ],
    TCTRL [
        TRG_SEL OFFSET(24)  NUMBITS(4),
        TRG_SRC OFFSET(23)  NUMBITS(1),
        TROT    OFFSET(18)  NUMBITS(1),
        TSOI    OFFSET(17)  NUMBITS(1),
        TSOT    OFFSET(16)  NUMBITS(1),
        MODE    OFFSET(2)   NUMBITS(2),
        CHAIN   OFFSET(1)   NUMBITS(1),
        T_EN    OFFSET(0)   NUMBITS(1)
    ]
];

/// General Low Power Interrupt Timer (LPIT)
#[repr(C)]
struct LpitRegisters {
    /// GPT Control Register
    verid: ReadOnly<u32, VERID::Register>,
    /// GPT Prescaler Register
    param: ReadOnly<u32, PARAM::Register>,
    /// GPT Status Register
    mcr: ReadWrite<u32, MCR::Register>,
    /// GPT Interrupt Register
    msr: ReadWrite<u32, MSR::Register>,
    /// GPT Output Compare Register 1
    mier: ReadWrite<u32, MIER::Register>,
    /// GPT Output Compare Register 2
    setten: ReadWrite<u32, SETTEN::Register>,
    /// GPT Output Compare Register 3
    clrten: ReadWrite<u32, CLRTEN::Register>,
    // Reverse 0
    _reserved0: [u8; 4],
    // Timer n Channel Control
    tmr: [TimerRegisters; LPIT_TMR_COUNT],
}

register_bitfields![u32,
    VERID [
        MAJOR   OFFSET(24)  NUMBITS(8),
        MINOR   OFFSET(16)  NUMBITS(8),
        FEATURE OFFSET(0)   NUMBITS(16)
    ],
    PARAM [
        EXT_TRIG    OFFSET(8)   NUMBITS(8),
        CHANNEL     OFFSET(0)   NUMBITS(8)
    ],
    MCR [
        DBG_EN      OFFSET(3)   NUMBITS(1),
        DOZE_EN     OFFSET(2)   NUMBITS(1),
        SW_RST      OFFSET(1)   NUMBITS(1),
        M_CEN       OFFSET(0)   NUMBITS(1)
    ],
    MSR [
        TIF3        OFFSET(3)   NUMBITS(1),
        TIF2        OFFSET(2)   NUMBITS(1),
        TIF1        OFFSET(1)   NUMBITS(1),
        TIF0        OFFSET(0)   NUMBITS(1)
    ],
    MIER [
        TIE3        OFFSET(3)   NUMBITS(1),
        TIE2        OFFSET(2)   NUMBITS(1),
        TIE1        OFFSET(1)   NUMBITS(1),
        TIE0        OFFSET(0)   NUMBITS(1)
    ],
    SETTEN [
        SET_T_EN3        OFFSET(3)   NUMBITS(1),
        SET_T_EN2        OFFSET(2)   NUMBITS(1),
        SET_T_EN1        OFFSET(1)   NUMBITS(1),
        SET_T_EN0        OFFSET(0)   NUMBITS(1)
    ],
    CLRTEN [
        CLR_T_EN3        OFFSET(3)   NUMBITS(1),
        CLR_T_EN2        OFFSET(2)   NUMBITS(1),
        CLR_T_EN1        OFFSET(1)   NUMBITS(1),
        CLR_T_EN0        OFFSET(0)   NUMBITS(1)
    ],
];

const LPIT0_BASE: StaticRef<LpitRegisters> =
    unsafe { StaticRef::new(0x4003_7000 as *const LpitRegisters) };

pub struct Lpit<'a, S> {
    registers: StaticRef<LpitRegisters>,
    clock: LpitClock<'a>,
    client: OptionalCell<&'a dyn hil::time::AlarmClient>,
    irqn: u32,
    _selection: core::marker::PhantomData<S>,
}

pub type Lpit1<'a> = Lpit<'a, _1>;

impl<'a> Lpit1<'a> {
    pub const fn new_lpit1(pcc: &'a crate::pcc::Pcc) -> Self {
        Lpit::new(
            LPIT0_BASE,
            nvic::LPIT0_CH0_IRQN,
            LpitClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccLPITGate,
            )),
        )
    }
}

impl<'a, S> Lpit<'a, S> {
    const fn new(base_addr: StaticRef<LpitRegisters>, irqn: u32, clock: LpitClock<'a>) -> Self {
        Lpit {
            registers: base_addr,
            clock: clock,
            client: OptionalCell::empty(),
            irqn,
            _selection: core::marker::PhantomData,
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

    pub fn handle_interrupt(&self) {
        // self.registers.sr.modify(SR::OF1::SET);
        // self.registers.ir.modify(IR::OF1IE::CLEAR);

        self.client.map(|client| client.alarm());
    }

    /// Start the GPT, specifying the peripheral clock selection and the peripheral clock divider
    ///
    /// If you select the crystal oscillator as the periodic clock root, the GPT will divide the
    /// input clock by 3.
    ///
    /// `divider` must be non-zero.
    pub fn start(&self, selection: pcc::PerclkClockSel, divider: u8) {
        // // Disable GPT and the GPT interrupt register first
        // self.registers.cr.modify(CR::EN::CLEAR);

        // self.registers.ir.modify(IR::ROVIE::CLEAR);
        // self.registers.ir.modify(IR::IF1IE::CLEAR);
        // self.registers.ir.modify(IR::IF2IE::CLEAR);
        // self.registers.ir.modify(IR::OF1IE::CLEAR);
        // self.registers.ir.modify(IR::OF2IE::CLEAR);
        // self.registers.ir.modify(IR::OF3IE::CLEAR);

        // // Clear Output mode to disconnected
        // self.registers.cr.modify(CR::OM1::CLEAR);
        // self.registers.cr.modify(CR::OM2::CLEAR);
        // self.registers.cr.modify(CR::OM3::CLEAR);

        // // Disable Input Capture Mode
        // self.registers.cr.modify(CR::IM1::CLEAR);
        // self.registers.cr.modify(CR::IM2::CLEAR);

        // // Reset all the registers to the their default values, except EN,
        // // ENMOD, STOPEN, DOZEEN, WAITEN, and DBGEN bits in the CR
        // self.registers.cr.modify(CR::SWR::SET);

        // // wait until registers are cleared
        // while self.registers.cr.is_set(CR::SWR) {}

        // // Clear the GPT status register
        // self.registers.sr.set(31 as u32);

        // // Enable free run mode
        // self.registers.cr.modify(CR::FRR::SET);

        // // Enable run in wait mode
        // self.registers.cr.modify(CR::WAITEN::SET);

        // // Enable run in stop mode
        // self.registers.cr.modify(CR::STOPEN::SET);

        // // Bring GPT counter to 0x00000000
        // self.registers.cr.modify(CR::ENMOD::SET);

        // // Set the value of the Output Compare Register
        // self.registers.ocr1.set(0xFFFF_FFFF - 1);

        // match selection {
        //     ccm::PerclkClockSel::IPG => {
        //         // Disable 24Mhz clock input from crystal
        //         self.registers.cr.modify(CR::EN_24M::CLEAR);

        //         // We will use the ipg_clk_highfreq provided by perclk_clk_root,
        //         // which runs at 24.75 MHz. Before calling set_alarm, we assume clock
        //         // to GPT1 has been enabled.
        //         self.registers.cr.modify(CR::CLKSRC.val(0x2 as u32));

        //         // We do not prescale the value for the moment. We will do so
        //         // after we will set the ARM_PLL1 CLK accordingly.
        //         self.registers.pr.modify(PR::PRESCALER.val(0 as u32));

        //         self.set_frequency(IMXRT1050_IPG_CLOCK_HZ / divider as u32);
        //     }
        //     ccm::PerclkClockSel::Oscillator => {
        //         // Enable 24MHz clock input
        //         self.registers
        //             .cr
        //             .modify(CR::EN_24M::SET + CR::CLKSRC::CrystalOscillator);

        //         // Funknown reasons, the 24HMz prescaler must be non-zero, even
        //         // though zero is a valid value according to the reference manual.
        //         // If it's not set, the counter doesn't count! Thanks to the se4L
        //         // project for adding a comment to their code.
        //         //
        //         // I'm also finding that it can't be too large; a prescaler of 8
        //         // for the 24MHz clock doesn't work!
        //         const DEFAULT_PRESCALER: u32 = 3;
        //         self.registers
        //             .pr
        //             .write(PR::PRESCALER24M.val(DEFAULT_PRESCALER - 1));
        //         self.set_frequency(OSCILLATOR_HZ / DEFAULT_PRESCALER / divider as u32);
        //     }
        // }

        // // Enable the GPT
        // self.registers.cr.modify(CR::EN::SET);

        // // Enable the Output Compare 1 Interrupt Enable
        // self.registers.ir.modify(IR::OF1IE::SET);
    }

    fn set_frequency(&self, hz: u32) {
        let idx = match self.irqn {
            nvic::LPIT0_CH0_IRQN => 0,
            nvic::LPIT0_CH1_IRQN => 1,
            nvic::LPIT0_CH2_IRQN => 2,
            nvic::LPIT0_CH3_IRQN => 3,
            _ => unreachable!(),
        };
        GPT_FREQUENCIES[idx].store(hz, Ordering::Release);
    }
}

/// Assumed IPG clock frequency for the iMXRT1050 processor family.
///
/// TODO this is not a constant value; it changes when setting the ARM clock
/// frequency. Change this after correctly configuring ARM frequency.
const IMXRT1050_IPG_CLOCK_HZ: u32 = 24_750_000;
/// Crystal oscillator frequency
const OSCILLATOR_HZ: u32 = 24_000_000;

/// GPT selection tags
pub enum _1 {}
pub enum _2 {}

static GPT_FREQUENCIES: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];

impl hil::time::Frequency for _1 {
    fn frequency() -> u32 {
        GPT_FREQUENCIES[0].load(Ordering::Acquire)
    }
}

impl hil::time::Frequency for _2 {
    fn frequency() -> u32 {
        GPT_FREQUENCIES[1].load(Ordering::Acquire)
    }
}

impl<F: hil::time::Frequency> hil::time::Time for Lpit<'_, F> {
    type Frequency = F;
    type Ticks = Ticks32;

    fn now(&self) -> Ticks32 {
        Ticks32::from(self.registers.tmr[0].cval.get())
    }
}

impl<'a, F: hil::time::Frequency> hil::time::Alarm<'a> for Lpit<'a, F> {
    fn set_alarm_client(&self, client: &'a dyn hil::time::AlarmClient) {
        self.client.set(client);
    }

    fn set_alarm(&self, reference: Self::Ticks, dt: Self::Ticks) {
        let mut expire = reference.wrapping_add(dt);
        let now = self.now();
        if !now.within_range(reference, expire) {
            expire = now;
        }

        if expire.wrapping_sub(now) < self.minimum_dt() {
            expire = now.wrapping_add(self.minimum_dt());
        }

        let _ = self.disarm();
        self.registers.tmr[0].tval.set(expire.into_u32());
        // self.registers.ir.modify(IR::OF1IE::SET);
    }

    fn get_alarm(&self) -> Self::Ticks {
        Self::Ticks::from(self.registers.tmr[0].tval.get())
    }

    fn disarm(&self) -> Result<(), ErrorCode> {
        unsafe {
            atomic(|| {
                // Disable counter
                // self.registers.ir.modify(IR::OF1IE::CLEAR);
                cortexm4::nvic::Nvic::new(self.irqn).clear_pending();
            });
        }
        Ok(())
    }

    fn is_armed(&self) -> bool {
        // If alarm is enabled, then OF1IE is set
        self.registers.mier.is_set(MIER::TIE0)
    }

    fn minimum_dt(&self) -> Self::Ticks {
        Self::Ticks::from(1)
    }
}

struct LpitClock<'a>(pcc::PeripheralClock<'a>);

impl ClockInterface for LpitClock<'_> {
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
