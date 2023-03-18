//! Implementation of the S32K14x hardware watchdog timer.

use core::cell::Cell;
use cortexm4::support;

use kernel::utilities::math::log_base_two_u64;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, FieldValue, ReadOnly, ReadWrite, WriteOnly,
};
use kernel::utilities::StaticRef;

#[repr(C)]
pub struct WdtRegisters {
    cs: ReadWrite<u32, Control::Register>,
    cnt: ReadWrite<u32, Counter::Register>,
    toval: ReadWrite<u32, Timout::Register>,
    win: ReadWrite<u32, Window::Register>,
}

register_bitfields![u32,
    Control [
        /// Watchdog Clear
        WIN     OFFSET(15)  NUMBITS(1) [],
        FLG     OFFSET(14)  NUMBITS(1) [],
        CMD32EN OFFSET(13)  NUMBITS(1) [],
        PRES    OFFSET(12)  NUMBITS(1) [],
        ULK     OFFSET(11)  NUMBITS(1) [],
        RCS     OFFSET(10)  NUMBITS(1) [],
        CLK     OFFSET(8)   NUMBITS(2) [],
        EN      OFFSET(7)   NUMBITS(1) [],
        INT     OFFSET(6)   NUMBITS(1) [],
        UPDATE  OFFSET(5)   NUMBITS(1) [],
        TST     OFFSET(3)   NUMBITS(2) [],
        DBG     OFFSET(2)   NUMBITS(1) [],
        WAIT    OFFSET(0)   NUMBITS(1) [],
        STOP    OFFSET(0)   NUMBITS(1) []
    ],

    Counter [
        COUNT  OFFSET(0)   NUMBITS(32) [
            UNLOCK = 0xD928_C520
        ]
    ],

    Timout [
        TIMEOUT OFFSET(0)  NUMBITS(16) [
            UNLOCK = 256
        ],
    ],
    Window [
        WINHIGH OFFSET(8)  NUMBITS(8) [],
        WINLOW  OFFSET(0)  NUMBITS(8) []
    ]
];
// Page 59 of SAM4L data sheet
const WDT_BASE: *mut WdtRegisters = 0x4005_2000 as *mut WdtRegisters;
const WDT_REGS: StaticRef<WdtRegisters> =
    unsafe { StaticRef::new(WDT_BASE as *const WdtRegisters) };

pub struct Wdt {
    enabled: Cell<bool>,
}

impl Wdt {
    pub const fn new() -> Wdt {
        Wdt {
            enabled: Cell::new(false),
        }
    }
    /// WDT Errata: §45.1.3
    ///
    /// When writing any of the PSEL, TBAN, EN, or MODE fields, must insert a
    /// delay for synchronization to complete.
    ///
    /// Also handle the KEY for the caller since we're special casing this.
    fn write_cr(&self, counter: FieldValue<u32, Counter::Register>) {
        WDT_REGS.cnt.write(Counter::COUNT::UNLOCK);
        WDT_REGS.toval.write(Timout::TIMEOUT::UNLOCK);
        WDT_REGS.cs.write(
            Control::EN::CLEAR
                + Control::CLK::SET
                + Control::INT::SET
                + Control::WIN::CLEAR
                + Control::UPDATE::SET,
        );
        // When writing to the affected fields, the user must ensure a wait
        // corresponding to 2 clock cycles of both the WDT peripheral bus clock
        // and the selected WDT clock source.
        //
        // TODO: Actual math based on chosen clock, ASF does:
        //       delay = div_ceil(sysclk_hz(), OSC_[chosen]_NOMINAL_HZ)
        for _ in 0..10 {
            support::nop();
        }
        support::nop();
        support::nop();
    }
    fn start(&self, period: usize) {
        self.enabled.set(true);

        // pm::enable_clock(Clock::PBD(PBDClock::WDT));

        // Note: Must use this clock to allow deep sleep. If you leave the
        // default RCSYS, then the watchdog simply will not fire if you enter
        // deep sleep (despite §20.4.1's protestations to the contrary).
        // This is lower power anyway, so take the win.
        // self.select_clock(WdtClockSource::ClockOsc32);

        // Choose the best period setting based on what was passed to `start()`
        //
        // §20.5.1.3 Configuring the WDT
        //
        // T_timeout = T_psel = 2^(PSEL + 1) / f_wdt_clk
        //
        // Period is in ms so use freq in khz for easy integer math
        // let f_clk_khz: u64 = if WDT_REGS.cr.matches_all(Control::CSSEL::RCSYS) {
        //     115
        // } else {
        //     // OSC32K
        //     32
        // };
        // let mult: u64 = f_clk_khz * (period as u64);
        // let scaler = log_base_two_u64(mult); // prefer rounding for longer WD (thus no -1)

        // let control = Control::CEN::ClockEnable
        //     + Control::PSEL.val(scaler)
        //     + Control::FCD::DoNotRedoCalibration
        //     + Control::DAR::DisableAfterReset
        //     + Control::EN::Enable;
        // self.write_cr(control);
    }

    fn stop(&self) {
        self.write_cr(Counter::COUNT::SET);

        // pm::disable_clock(Clock::PBD(PBDClock::WDT));

        // self.enabled.set(false);
    }

    fn tickle(&self) {
        // Need to write the WDTCLR bit twice for it to work
        // WDT_REGS.clr.write(Clear::KEY::KEY1 + Clear::WDTCLR::SET);
        // WDT_REGS.clr.write(Clear::KEY::KEY2 + Clear::WDTCLR::SET);
    }
}

impl kernel::platform::watchdog::WatchDog for Wdt {
    fn setup(&self) {
        // Setup the WatchDog with a 100ms period.
        self.start(100);
    }

    fn tickle(&self) {
        self.tickle();
    }

    fn suspend(&self) {
        self.stop();
    }
}
