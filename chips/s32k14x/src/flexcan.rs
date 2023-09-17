use core::cell::Cell;
use core::iter::once_with;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite};

use kernel::hil;
use kernel::platform::chip::ClockInterface;
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use nb;

use crate::{dma, pcc};

use self::RAMn::RAM;

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
    _reserved4: [u8; 44],
    ramn: [ReadWrite<u32, RAMn::Register>; 128],
    _reserved5: [u8; 1536],
    /// Rx Individual Mask registers (RXIMR0 - RXIMR31)
    rximrn: [ReadWrite<u32, RXIMRn::Register>; 32],
    _reserved6: [u8; 2296],
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

    RAMn [
        RAM OFFSET(0) NUMBITS(32) [],
    ],

    RXIMRn [
        /// RX FIFO Empty
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
        self.can_disabled_mode_exit();
        self.can_freeze_mode_enter();
        self.can_disabled_mode_enter();

        self.registers.ctrl1.modify(CTRL1::CLKSRC::CLEAR);
        self.can_disabled_mode_exit();
        self.can_freeze_mode_enter();

        // Reset current bit timming configuration
        self.registers.ctrl1.modify(
            CTRL1::PRESDIV::CLEAR
                + CTRL1::PROPSEG::CLEAR
                + CTRL1::PSEG1::CLEAR
                + CTRL1::PSEG2::CLEAR
                + CTRL1::RJW::CLEAR
                + CTRL1::SMP::CLEAR,
        );

        self.registers.ctrl1.modify(
            CTRL1::PRESDIV.val(0x03)
                + CTRL1::PROPSEG.val(0x07)
                + CTRL1::PSEG1.val(0x04)
                + CTRL1::PSEG2.val(0x01)
                + CTRL1::RJW.val(0x01)
                + CTRL1::SMP.val(0),
        );

        // Clear the message box RAM. Each message max cover 4 words (4 x 32bit)
        for n in 0..128 {
            self.registers.ramn[n].modify(RAMn::RAM::CLEAR);
        }

        // Clear the reception mask register for each message box
        for n in 0..32 {
            self.registers.rximrn[n].modify(RXIMRn::MI::CLEAR);
        }

        // Configure the maximum number of message boxes
        self.registers
            .mcr
            .modify(MCR::MAXMB.val(31) + MCR::SRXDIS.val(1) + MCR::IRMQ.val(1) + MCR::RFEN::CLEAR);

        // Configure filter ID for MB
        for n in 0..32 {
            if n == 0 {
                self.registers.rximrn[n].modify(RXIMRn::MI.val(0x4000_0000 | 0x1F00_FFFF));
                self.registers.ramn[(n * 4) + 0].modify(RAMn::RAM.val(0x0420_0000));
                // filter ID 0xFFFF extended ID
                self.registers.ramn[(n * 4) + 1].modify(RAMn::RAM.val(0x0000_FFFF));
            }
        }

        // Disable all message box interrupts
        self.registers.imask1.modify(IMASK1::BU31TO0M::CLEAR);
        // Clear all message box interrupt flags
        self.registers.iflag1.set(0xFFFF_FFFF);
        // Clear all error interrupt flags
        self.registers.esr1.modify(
            ESR1::ERRINT::SET
                + ESR1::BOFFINT::SET
                + ESR1::RWRNINT::SET
                + ESR1::TWRNINT::SET
                + ESR1::BOFFDONEINT::SET
                + ESR1::ERRINT_FAST::SET
                + ESR1::ERROVR::SET,
        );
        // Switch to normal user mode
        self.registers.mcr.modify(MCR::SUPV::CLEAR);
        self.registers
            .ctrl1
            .modify(CTRL1::LOM::CLEAR + CTRL1::LPB::CLEAR);

        self.can_freeze_mode_exit();
        while self.registers.mcr.is_set(MCR::NOTRDY) {}
    }

    pub fn can_transmit(&self, mb: usize, mut mes_id: u32, tx_buffer: *const u8, tx_len: u8) {
        let mut is_ext_id: u8 = 0;
        if (mes_id & 0x8000_0000) != 0 {
            mes_id = mes_id & !0x8000_0000;
            is_ext_id = 1;
        }

        self.registers.iflag1.set(1 << mb);

        let mut ram_value = self.registers.ramn[(mb * 4) + 0].get();
        self.registers.ramn[(mb * 4) + 0].set(
            (ram_value & (!0xE0000000 & !(0x200000 | 0x100000) & !0xF0000))
                | (0x400000 | (((tx_len as u32) << 16) & 0xF0000)),
        );

        let mut ram2: u32 = 0;
        let mut ram3: u32 = 0;

        for n in 0..4 as usize {
            // 0 1 2 3 -> 3 2 1 0 -> mb 2
            // 4 5 6 7 -> 7 6 5 4 -> mb 3
            let mut word = tx_buffer.wrapping_offset(n as isize);
            // self.registers.ramn[(mb * 4) + 1 + (n / 4)].set(unsafe { *tx_buffer } as u32);
            ram2 = ram2 | ((unsafe { *word } as u32) << (8 * (3 - n)));
        }

        for n in 4..8 as usize {
            // 0 1 2 3 -> 3 2 1 0 -> mb 2
            // 4 5 6 7 -> 7 6 5 4 -> mb 3
            let mut word = tx_buffer.wrapping_offset(n as isize);
            // self.registers.ramn[(mb * 4) + 1 + (n / 4)].set(unsafe { *tx_buffer } as u32);
            ram3 = ram3 | ((unsafe { *word } as u32) << (8 * (7 - n)));
        }
        self.registers.ramn[(mb * 4) + 2].set(ram2);
        self.registers.ramn[(mb * 4) + 3].set(ram3);

        if is_ext_id == 0 {
        } else {
            ram_value = self.registers.ramn[(mb * 4) + 0].get();
            self.registers.ramn[(mb * 4) + 0].set(ram_value | 0x20_0000);
            self.registers.ramn[(mb * 4) + 1].set(mes_id & 0x1FFF_FFFF);
        }

        ram_value = self.registers.ramn[(mb * 4) + 0].get();
        self.registers.ramn[(mb * 4) + 0].set(ram_value | (0x0C << 24) & 0x0F00_0000);
    }

    fn can_disabled_mode_exit(&self) {
        if self.registers.mcr.is_set(MCR::MDIS) {
            self.registers.mcr.modify(MCR::MDIS::CLEAR);
            while self.registers.mcr.is_set(MCR::LPMACK) {}
        }
    }

    fn can_freeze_mode_enter(&self) {
        self.registers.mcr.modify(MCR::FRZ::SET + MCR::HALT::SET);
        while self.registers.mcr.is_set(MCR::MDIS) {}
    }

    fn can_freeze_mode_exit(&self) {
        self.registers.mcr.modify(MCR::FRZ::CLEAR);
        self.registers.mcr.modify(MCR::HALT::CLEAR);
        while self.registers.mcr.is_set(MCR::FRZACK) {}
    }

    fn can_disabled_mode_enter(&self) {
        if !self.registers.mcr.is_set(MCR::MDIS) {
            self.registers.mcr.modify(MCR::MDIS::SET);
            while !self.registers.mcr.is_set(MCR::LPMACK) {}
        }
    }
}
