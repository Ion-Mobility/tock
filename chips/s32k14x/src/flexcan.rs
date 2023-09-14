use core::cell::Cell;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite};

use kernel::hil;
use kernel::platform::chip::ClockInterface;
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use nb;

use crate::{dma, pcc};

/// LP Universal asynchronous receiver transmitter
#[repr(C)]
struct FlexcanRegisters {
    ///  Module Configuration Register (MCR)
    mcr: ReadWrite<u32, MCR::Register>,
    /// Control 1 Register (CTRL1)
    ctrl1: ReadWrite<u32, CTRL1::Register>,
    /// Free Running Timer Register (TIMER)
    timer: ReadWrite<u32, TIMER::Register>,
    _reserved0: [u8; 4],
    /// Rx Mailboxes Global Mask register (RXMGMASK)
    rxmgmask: ReadWrite<u32, RXMGMASK::Register>,
    /// Rx 14 Mask register (RX14MASK)
    rx14mask: ReadWrite<u32, RX14MASK::Register>,
    /// Rx 15 Mask register (RX15MASK)
    rx15mask: ReadWrite<u32, RX15MASK::Register>,
    /// Error Counter Register (ECR)
    ecr: ReadWrite<u32, ECR::Register>,
    /// Error and Status 1 register (ESR1)
    esr1: ReadWrite<u32, ESR1::Register>,
    _reserved1: [u8; 4],
    /// Interrupt Masks 1 register (IMASK1)
    imask1: ReadWrite<u32, IMASK1::Register>,
    _reserved2: [u8; 4],
    /// Interrupt Flags 1 register (IFLAG1)
    iflag1: ReadWrite<u32, IFLAG1::Register>,
    /// Control 2 register (CTRL2)
    ctrl2: ReadWrite<u32, CTRL2::Register>,
    /// Error and Status 2 register (ESR2)
    esr2: ReadWrite<u32, ESR2::Register>,
    _reserved3: [u8; 8],
    /// CRC register (CRCR)
    crcr: ReadWrite<u32, CRCR::Register>,
    /// Rx FIFO Global Mask register (RXFGMASK)
    rxfgmask: ReadWrite<u32, RXFGMASK::Register>,
    /// Rx FIFO Information register (RXFIR)
    rxfir: ReadWrite<u32, RXFIR::Register>,
    /// CAN Bit Timing register (CBT)
    cbt: ReadWrite<u32, CBT::Register>,
    _reserved4: [u8; 2092],
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr0: ReadWrite<u32, RXIMR0::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr1: ReadWrite<u32, RXIMR1::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr2: ReadWrite<u32, RXIMR2::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr3: ReadWrite<u32, RXIMR3::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr4: ReadWrite<u32, RXIMR4::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr5: ReadWrite<u32, RXIMR5::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr6: ReadWrite<u32, RXIMR6::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr7: ReadWrite<u32, RXIMR7::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr8: ReadWrite<u32, RXIMR8::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr9: ReadWrite<u32, RXIMR9::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr10: ReadWrite<u32, RXIMR10::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr11: ReadWrite<u32, RXIMR11::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr12: ReadWrite<u32, RXIMR12::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr13: ReadWrite<u32, RXIMR13::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr14: ReadWrite<u32, RXIMR14::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr15: ReadWrite<u32, RXIMR15::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr16: ReadWrite<u32, RXIMR16::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr17: ReadWrite<u32, RXIMR17::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr18: ReadWrite<u32, RXIMR18::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr19: ReadWrite<u32, RXIMR19::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr20: ReadWrite<u32, RXIMR20::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr21: ReadWrite<u32, RXIMR21::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr22: ReadWrite<u32, RXIMR22::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr23: ReadWrite<u32, RXIMR23::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr24: ReadWrite<u32, RXIMR24::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr25: ReadWrite<u32, RXIMR25::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr26: ReadWrite<u32, RXIMR26::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr27: ReadWrite<u32, RXIMR27::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr28: ReadWrite<u32, RXIMR28::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr29: ReadWrite<u32, RXIMR29::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr30: ReadWrite<u32, RXIMR30::Register>,
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximr31: ReadWrite<u32, RXIMR31::Register>,
    _reserved5: [u8; 2296],
    // /// Pretended Networking Control 1 register (CTRL1_PN)
    // ctrl1_pn: ReadWrite<u32, CTRL1_PN::Register>,
    // /// Pretended Networking Control 2 register (CTRL2_PN)
    // ctrl2_pn: ReadWrite<u32, CTRL2_PN::Register>,
    // /// Pretended Networking Wake Up Match register (WU_MTC)
    // wu_mtc: ReadWrite<u32, WU_MTC::Register>,
    // /// Pretended Networking ID Filter 1 register (FLT_ID1)
    // flt_id1: ReadWrite<u32, FLT_ID1::Register>,
    // /// Pretended Networking DLC Filter register (FLT_DLC)
    // flt_dlc: ReadWrite<u32, FLT_DLC::Register>,
    // /// Pretended Networking Payload Low Filter 1 register (PL1_LO)
    // pl1_lo: ReadWrite<u32, PL1_LO::Register>,
    // /// Pretended Networking Payload High Filter 1 register (PL1_HI)
    // pl1_hi: ReadWrite<u32, PL1_HI::Register>,
    // /// Pretended Networking ID Filter 2 Register / ID Mask register (FLT_ID2_IDMASK)
    // flt_id2_idmask: ReadWrite<u32, FLT_ID2_IDMASK::Register>,
    // /// Pretended Networking Payload Low Filter 2 Register / Payload Low Mask register (PL2_PLMASK_LO)
    // pl2_plmask_lo: ReadWrite<u32, PL2_PLMASK_LO::Register>,
    // ///  Pretended Networking Payload High Filter 2 low order bits / Payload High Mask register (PL2_PLMASK_HI)
    // pl2_plmask_hi: ReadWrite<u32, PL2_PLMASK_HI::Register>,
    // _reserved6: [u8; 2848],
    // ///  Wake Up Message Buffer register for C/S (WMB0_CS - WMB3_CS)
    // wmb0_cs: ReadWrite<u32, WMB0_CS::Register>,
    // /// Wake Up Message Buffer Register for ID (WMB0_ID - WMB3_ID)
    // wmb0_id: ReadWrite<u32, WMB0_ID::Register>,
    // /// Wake Up Message Buffer Register for Data 0-3 (WMB0_D03 - WMB3_D03)
    // wmb0_d03: ReadWrite<u32, WMB0_D03::Register>,
    // ///  Wake Up Message Buffer Register Data 4-7 (WMB0_D47 - WMB3_D47)
    // wmb0_d47: ReadWrite<u32, WMB0_D47::Register>,
    // ///  Wake Up Message Buffer register for C/S (WMB0_CS - WMB3_CS)
    // wmb1_cs: ReadWrite<u32, WMB1_CS::Register>,
    // /// Wake Up Message Buffer Register for ID (WMB0_ID - WMB3_ID)
    // wmb1_id: ReadWrite<u32, WMB1_ID::Register>,
    // /// Wake Up Message Buffer Register for Data 0-3 (WMB0_D03 - WMB3_D03)
    // wmb1_d03: ReadWrite<u32, WMB1_D03::Register>,
    // ///  Wake Up Message Buffer Register Data 4-7 (WMB0_D47 - WMB3_D47)
    // wmb1_d47: ReadWrite<u32, WMB1_D47::Register>,
    // ///  Wake Up Message Buffer register for C/S (WMB0_CS - WMB3_CS)
    // wmb2_cs: ReadWrite<u32, WMB2_CS::Register>,
    // /// Wake Up Message Buffer Register for ID (WMB0_ID - WMB3_ID)
    // wmb2_id: ReadWrite<u32, WMB2_ID::Register>,
    // /// Wake Up Message Buffer Register for Data 0-3 (WMB0_D03 - WMB3_D03)
    // wmb2_d03: ReadWrite<u32, WMB2_D03::Register>,
    // ///  Wake Up Message Buffer Register Data 4-7 (WMB0_D47 - WMB3_D47)
    // wmb2_d47: ReadWrite<u32, WMB2_D47::Register>,
    // ///  Wake Up Message Buffer register for C/S (WMB0_CS - WMB3_CS)
    // wmb3_cs: ReadWrite<u32, WMB3_CS::Register>,
    // /// Wake Up Message Buffer Register for ID (WMB0_ID - WMB3_ID)
    // wmb3_id: ReadWrite<u32, WMB3_ID::Register>,
    // /// Wake Up Message Buffer Register for Data 0-3 (WMB0_D03 - WMB3_D03)
    // wmb3_d03: ReadWrite<u32, WMB3_D03::Register>,
    // ///  Wake Up Message Buffer Register Data 4-7 (WMB0_D47 - WMB3_D47)
    // wmb3_d47: ReadWrite<u32, WMB3_D47::Register>,
    // /// CAN FD Control register (FDCTRL)
    // fdctrl: ReadWrite<u32, FDCTRL::Register>,
    // /// CAN FD Bit Timing register (FDCBT)
    // fdcbt: ReadWrite<u32, FDCBT::Register>,
    // /// CAN FD CRC register (FDCRC)
    // fdcrc: ReadWrite<u32, FDCRC::Register>,
}

register_bitfields![u32,
    MCR [
        /// Major Version Number
        MDIS OFFSET(31) NUMBITS(1) [],
        /// Major Version Number
        FRZ OFFSET(30) NUMBITS(1) [],
        /// Major Version Number
        RFEN OFFSET(29) NUMBITS(1) [],
        /// Major Version Number
        HALT OFFSET(28) NUMBITS(1) [],
        /// Major Version Number
        NOTRDY OFFSET(27) NUMBITS(1) [],
        /// Major Version Number
        SOFTRST OFFSET(25) NUMBITS(1) [],
        /// Major Version Number
        FRZACK OFFSET(24) NUMBITS(1) [],
        /// Major Version Number
        SUPV OFFSET(23) NUMBITS(1) [],
        /// Major Version Number
        WRNEN OFFSET(21) NUMBITS(1) [],
        /// Major Version Number
        LPMACK OFFSET(20) NUMBITS(1) [],
        /// Major Version Number
        SRXDIS OFFSET(17) NUMBITS(1) [],
        /// Major Version Number
        IRMQ OFFSET(16) NUMBITS(1) [],
        /// Major Version Number
        DMA OFFSET(15) NUMBITS(1) [],
        /// Major Version Number
        PNET_EN OFFSET(14) NUMBITS(1) [],
        /// Major Version Number
        LPRIOEN OFFSET(13) NUMBITS(1) [],
        /// Major Version Number
        AEN OFFSET(12) NUMBITS(1) [],
        /// Major Version Number
        FDEN OFFSET(11) NUMBITS(1) [],
        /// Minor Version Number
        IDAM OFFSET(8) NUMBITS(2) [],
        /// Feature Identification Number
        MAXMB OFFSET(0) NUMBITS(6) []
    ],

    CTRL1 [
        /// Receive FIFO Size
        PRESDIV OFFSET(24) NUMBITS(8) [],
        /// Receive FIFO Size
        RJW OFFSET(22) NUMBITS(2) [],
        /// Receive FIFO Size
        PSEG1 OFFSET(19) NUMBITS(3) [],
        /// Receive FIFO Size
        PSEG2 OFFSET(16) NUMBITS(3) [],
        /// Receive FIFO Size
        BOFFMSK OFFSET(15) NUMBITS(1) [],
        /// Receive FIFO Size
        ERRMSK OFFSET(14) NUMBITS(1) [],
        /// Receive FIFO Size
        CLKSRC OFFSET(13) NUMBITS(1) [],
        /// Receive FIFO Size
        LPB OFFSET(12) NUMBITS(1) [],
        /// Receive FIFO Size
        TWRNMSK OFFSET(11) NUMBITS(1) [],
        /// Receive FIFO Size
        RWRNMSK OFFSET(10) NUMBITS(1) [],
        /// Receive FIFO Size
        SMP OFFSET(7) NUMBITS(1) [],
        /// Receive FIFO Size
        BOFFREC OFFSET(6) NUMBITS(1) [],
        /// Receive FIFO Size
        TSYN OFFSET(5) NUMBITS(1) [],
        /// Receive FIFO Size
        LBUF OFFSET(4) NUMBITS(1) [],
        /// Receive FIFO Size
        LOM OFFSET(3) NUMBITS(1) [],
        /// Transmit FIFO Size
        PROPSEG OFFSET(0) NUMBITS(2) []
    ],

    TIMER [
        /// Reset Receive FIFO
        TIMER OFFSET(0) NUMBITS(16) []
    ],

    RXMGMASK [
        /// Transmit Data Flag
        MG OFFSET(0) NUMBITS(32) []
    ],

    RX14MASK [
        /// Transmit Data Flag
        RX14M OFFSET(0) NUMBITS(32) []
    ],

    RX15MASK [
        /// Transmit Data Flag
        RX15M OFFSET(0) NUMBITS(32) []
    ],

    ECR [
        /// Receive Data Match Only
        RXERRCNT_FAST OFFSET(24) NUMBITS(8) [],
        /// Circular FIFO Enable
        TXERRCNT_FAST OFFSET(16) NUMBITS(8) [],
        /// Host Request Select
        RXERRCNT OFFSET(8) NUMBITS(8) [],
        /// Host Request Polarity
        TXERRCNT OFFSET(0) NUMBITS(8) []
    ],

    ESR1 [
        /// Peripheral Chip Select Configuration
        BIT1ERR_FAST OFFSET(31) NUMBITS(1) [],
        /// Output Configuration
        BIT0ERR_FAST OFFSET(30) NUMBITS(1) [],
        /// Pin Configuration
        CRCERR_FAST OFFSET(28) NUMBITS(1) [],
        /// Match Configuration
        FRMERR_FAST OFFSET(27) NUMBITS(1) [],
        /// Peripheral Chip Select Polarity
        STFERR_FAST OFFSET(26) NUMBITS(1) [],
        /// No Stall
        ERROVR OFFSET(21) NUMBITS(1) [],
        /// Automatic PCS
        ERRINT_FAST OFFSET(20) NUMBITS(1) [],
        /// Sample Point
        BOFFDONEINT OFFSET(19) NUMBITS(1) [],
        /// Master Mode
        SYNCH OFFSET(18) NUMBITS(1) [],
        /// Master Mode
        TWRNINT OFFSET(17) NUMBITS(1) [],
        /// Master Mode
        RWRNINT OFFSET(16) NUMBITS(1) [],
        /// Master Mode
        BIT1ERR OFFSET(15) NUMBITS(1) [],
        /// Master Mode
        BIT0ERR OFFSET(14) NUMBITS(1) [],
        /// Master Mode
        ACKERR OFFSET(13) NUMBITS(1) [],
        /// Master Mode
        CRCERR OFFSET(12) NUMBITS(1) [],
        /// Master Mode
        FRMERR OFFSET(11) NUMBITS(1) [],
        /// Master Mode
        STFERR OFFSET(10) NUMBITS(1) [],
        /// Master Mode
        TXWRN OFFSET(9) NUMBITS(1) [],
        /// Master Mode
        RXWRN OFFSET(8) NUMBITS(1) [],
        /// Master Mode
        IDLE OFFSET(7) NUMBITS(1) [],
        /// Master Mode
        TX OFFSET(6) NUMBITS(1) [],
        /// Master Mode
        FLTCONF OFFSET(4) NUMBITS(2) [],
        /// Master Mode
        RX OFFSET(3) NUMBITS(1) [],
        /// Master Mode
        BOFFINT OFFSET(2) NUMBITS(1) [],
        /// Master Mode
        ERRINT OFFSET(1) NUMBITS(1) []
    ],

    IMASK1 [
        /// Match 0 Value
        BU31TO0M OFFSET(0) NUMBITS(32) []
    ],

    IFLAG1 [
        /// Match 0 Value
        BUF31TO8I OFFSET(8) NUMBITS(24) [],
        /// Match 0 Value
        BUF7I OFFSET(7) NUMBITS(1) [],
        /// Match 0 Value
        BUF6I OFFSET(6) NUMBITS(1) [],
        /// Match 0 Value
        BUF5I OFFSET(5) NUMBITS(1) [],
        /// Match 0 Value
        BUF4TO1I OFFSET(1) NUMBITS(4) [],
        /// Match 0 Value
        BUF0I OFFSET(0) NUMBITS(1) []
    ],

    CTRL2 [
        /// Match Address 2
        ERRMSK_FAST OFFSET(31) NUMBITS(1) [],
        /// Match Address 2
        BOFFDONEMSK OFFSET(30) NUMBITS(1) [],
        /// Match Address 2
        RFFN OFFSET(24) NUMBITS(4) [],
        /// Match Address 2
        TASD OFFSET(19) NUMBITS(5) [],
        /// Match Address 2
        MRP OFFSET(18) NUMBITS(1) [],
        /// Match Address 2
        RRS OFFSET(17) NUMBITS(1) [],
        /// Match Address 2
        EACEN OFFSET(16) NUMBITS(1) [],
        /// Match Address 2
        TIMER_SRC OFFSET(15) NUMBITS(1) [],
        /// Match Address 2
        PREXCEN OFFSET(14) NUMBITS(1) [],
        /// Match Address 2
        ISOCANFDEN OFFSET(12) NUMBITS(1) [],
        /// Match Address 2
        EDFLTDIS OFFSET(11) NUMBITS(1) []
    ],

    ESR2 [
        /// SCK-to-PCS Delay
        LPTM OFFSET(16) NUMBITS(7) [],
        /// PCS-to-SCK Delay
        VPS OFFSET(14) NUMBITS(1) [],
        /// Delay Between Transfers
        IMB OFFSET(13) NUMBITS(1) []
    ],

    CRCR [
        /// Receive FIFO Watermark
        MBCRC OFFSET(16) NUMBITS(7) [],
        /// Transmit FIFO Watermark
        TXCRC OFFSET(0) NUMBITS(15) []
    ],

    RXFGMASK [
        /// Receive FIFO Count
        FGM OFFSET(0) NUMBITS(32) []
    ],

    RXFIR [
        /// Clock Polarity
        IDHIT OFFSET(0) NUMBITS(9) []
    ],

    CBT [
        /// Transmit Data
        BTF OFFSET(31) NUMBITS(1) [],
        /// Transmit Data
        EPRESDIV OFFSET(21) NUMBITS(10) [],
        /// Transmit Data
        ERJW OFFSET(16) NUMBITS(5) [],
        /// Transmit Data
        EPROPSEG OFFSET(10) NUMBITS(6) [],
        /// Transmit Data
        EPSEG1 OFFSET(5) NUMBITS(5) [],
        /// Transmit Data
        EPSEG2 OFFSET(0) NUMBITS(5) []
    ],

    RXIMR0 [
        /// RX FIFO Empty
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR1 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR2 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR3 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR4 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR5 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR6 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR7 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR8 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR9 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR10 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR11 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR12 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR13 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR14 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR15 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR16 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR17 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR18 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR19 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR20 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR21 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR22 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR23 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR24 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR25 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR26 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR27 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR28 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR29 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR30 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],

    RXIMR31 [
        /// Transmit Data
        MI OFFSET(0) NUMBITS(32) []
    ],
];

struct FlexcanClock<'a>(pcc::PeripheralClock<'a>);

impl ClockInterface for FlexcanClock<'_> {
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

const FLEXCAN_BASE: [StaticRef<FlexcanRegisters>; 3] = [
    unsafe { StaticRef::new(0x4002_4000 as *const FlexcanRegisters) },
    unsafe { StaticRef::new(0x4002_5000 as *const FlexcanRegisters) },
    unsafe { StaticRef::new(0x4002_B000 as *const FlexcanRegisters) },
];

pub struct FLEXCAN<'a> {
    registers: StaticRef<FlexcanRegisters>,
    clock: FlexcanClock<'a>,
}

impl<'a> FLEXCAN<'a> {
    pub fn new_flexcan0(pcc: &'a pcc::Pcc) -> Self {
        FLEXCAN::new(
            FLEXCAN_BASE[0],
            FlexcanClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccFLEXCAN0Gate,
            )),
        )
    }

    pub fn new_flexcan1(pcc: &'a pcc::Pcc) -> Self {
        FLEXCAN::new(
            FLEXCAN_BASE[1],
            FlexcanClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccFLEXCAN1Gate,
            )),
        )
    }

    pub fn new_flexcan2(pcc: &'a pcc::Pcc) -> Self {
        FLEXCAN::new(
            FLEXCAN_BASE[2],
            FlexcanClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccFLEXCAN2Gate,
            )),
        )
    }

    fn new(base_addr: StaticRef<FlexcanRegisters>, clock: FlexcanClock<'a>) -> FLEXCAN<'a> {
        FLEXCAN {
            registers: base_addr,
            clock: clock,
        }
    }

    pub fn init(&self) {
        // CanDisabledModeExit
        self.registers.mcr.is_set(MCR::MDIS);
    }
}
