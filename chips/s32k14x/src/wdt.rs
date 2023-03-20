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
        for _ in 0..10000 {
            support::nop();
        }
    }
    fn start(&self, period: usize) {
        self.enabled.set(true);
    }

    fn stop(&self) {
        self.write_cr(Counter::COUNT::SET);
        self.enabled.set(false);
    }

    fn tickle(&self) {
        support::nop();
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
