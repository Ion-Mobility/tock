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

/// Full duplex (master mode)
///
/// # Notes
///
/// - It's the task of the user of this interface to manage the slave select lines
///
/// - Due to how full duplex SPI works each `read` call must be preceded by a `send` call.
///
/// - Some SPIs can work with 8-bit *and* 16-bit words. You can overload this trait with different
/// `Word` types to allow operation in both modes.
pub trait FullDuplex<Word> {
    /// An enumeration of SPI errors
    type Error;

    /// Reads the word stored in the shift register
    ///
    /// **NOTE** A word must be sent to the slave before attempting to call this
    /// method.
    fn read(&mut self) -> nb::Result<Word, Self::Error>;

    /// Sends a word to the slave
    fn send(&mut self, word: Word) -> nb::Result<(), Self::Error>;
}

/// Clock polarity
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    /// Clock signal low when idle
    IdleLow,
    /// Clock signal high when idle
    IdleHigh,
}

/// Clock phase
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Data in "captured" on the first clock transition
    CaptureOnFirstTransition,
    /// Data in "captured" on the second clock transition
    CaptureOnSecondTransition,
}

/// SPI mode
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Mode {
    /// Clock polarity
    pub polarity: Polarity,
    /// Clock phase
    pub phase: Phase,
}

/// Helper for CPOL = 0, CPHA = 0
pub const MODE_0: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// Helper for CPOL = 0, CPHA = 1
pub const MODE_1: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnSecondTransition,
};

/// Helper for CPOL = 1, CPHA = 0
pub const MODE_2: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnFirstTransition,
};

/// Helper for CPOL = 1, CPHA = 1
pub const MODE_3: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnSecondTransition,
};

/// Data direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Transmit direction (leaving the peripheral).
    Tx,
    /// Receive direction (entering the peripheral).
    Rx,
}

/// Bit order.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum BitOrder {
    /// Data is transferred most significant bit first (default).
    #[default]
    Msb,
    /// Data is transferred least significant bit first.
    Lsb,
}

/// Receive sample point behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplePoint {
    /// Input data is sampled on SCK edge.
    Edge,
    /// Input data is sampled on delayed SCK edge.
    DelayedEdge,
}

/// Possible errors when interfacing the LPSPI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LpspiError {
    /// The transaction frame size is incorrect.
    ///
    /// The frame size, in bits, must be between 8 bits and
    /// 4095 bits.
    FrameSize,
    /// FIFO error in the given direction.
    Fifo(Direction),
    /// Bus is busy at the start of a transfer.
    Busy,
    /// Caller provided no data.
    NoData,
}

/// The number of words in each FIFO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FifoStatus {
    /// Number of words in the receive FIFO.
    pub rxcount: u16,
    /// Number of words in the transmit FIFO.
    pub txcount: u16,
}

impl FifoStatus {
    /// Indicates if the FIFO is full for the given direction.
    #[inline]
    pub const fn is_full(self, direction: Direction) -> bool {
        /// See PARAM register docs.
        const MAX_FIFO_SIZE: u16 = 16;
        let count = match direction {
            Direction::Tx => self.txcount,
            Direction::Rx => self.rxcount,
        };
        count >= MAX_FIFO_SIZE
    }
}

pub struct Transaction {
    /// Enable byte swap.
    ///
    /// When enabled (`true`), swap bytes with the `u32` word. This allows
    /// you to change the endianness of the 32-bit word transfer. The
    /// default is `false`.
    pub byte_swap: bool,
    /// Bit order.
    ///
    /// See [`BitOrder`] for details. The default is [`BitOrder::Msb`].
    pub bit_order: BitOrder,
    /// Mask the received data.
    ///
    /// If `true`, the peripheral discards received data. Use this
    /// when you only care about sending data. The default is `false`;
    /// the peripheral puts received data in the receive FIFO.
    pub receive_data_mask: bool,
    /// Mask the transmit data.
    ///
    /// If `true`, the peripheral doesn't send any data. Use this when
    /// you only care about receiving data. The default is `false`;
    /// the peripheral expects to send data using the transmit FIFO.
    pub transmit_data_mask: bool,
    /// Indicates (`true`) the start of a continuous transfer.
    ///
    /// If set, the peripherals chip select will remain asserted after
    /// exchanging the frame. This allows you to enqueue new commands
    /// and data words within the same transaction. Those new commands
    /// should have [`continuing`](Self::continuing) set to `true`.
    ///
    /// The default is `false`; chip select de-asserts after exchanging
    /// the frame. To stop a continuous transfer, enqueue a new `Transaction`
    /// in which this flag, and `continuing`, is false.
    pub continuous: bool,
    /// Indicates (`true`) that this command belongs to a previous transaction.
    ///
    /// Set this to indicate that this new `Transaction` belongs to a previous
    /// `Transaction`, one that had [`continuous`](Self::continuous) set.
    /// The default value is `false`.
    pub continuing: bool,

    frame_size: u16,
}

impl Transaction {
    /// Defines a transaction for a `u32` buffer.
    ///
    /// After successfully defining a transaction of this buffer,
    /// supply it to the LPSPI driver, then start sending the
    /// data.
    ///
    /// Returns an error if any are true:
    ///
    /// - the buffer is empty.
    /// - there's more than 128 elements in the buffer.
    pub fn new_u32s(data: &[u32]) -> Result<Self, LpspiError> {
        Transaction::new_words(data)
    }

    fn new_words<W>(data: &[W]) -> Result<Self, LpspiError> {
        Transaction::new(8 * core::mem::size_of_val(data) as u16)
    }

    /// Define a transaction by specifying the frame size, in bits.
    ///
    /// The frame size describes the number of bits that will be transferred and
    /// received during the next transaction. Specifically, it describes the number
    /// of bits for which the PCS pin signals a transaction.
    ///
    /// # Requirements
    ///
    /// - `frame_size` fits within 12 bits; the implementation enforces this maximum value.
    /// - The minimum value for `frame_size` is 8; the implementation enforces this minimum
    ///   value.
    pub fn new(frame_size: u16) -> Result<Self, LpspiError> {
        const MIN_FRAME_SIZE: u16 = 8;
        const MAX_FRAME_SIZE: u16 = 1 << 12;
        if (MIN_FRAME_SIZE..MAX_FRAME_SIZE).contains(&frame_size) {
            Ok(Self {
                byte_swap: false,
                bit_order: Default::default(),
                receive_data_mask: false,
                transmit_data_mask: false,
                frame_size: frame_size - 1,
                continuing: false,
                continuous: false,
            })
        } else {
            Err(LpspiError::FrameSize)
        }
    }
}

/// LP Universal asynchronous receiver transmitter
#[repr(C)]
struct LpspiRegisters {
    ///  Version ID Register
    verid: ReadOnly<u32, VERID::Register>,
    /// Parameter Register
    param: ReadOnly<u32, PARAM::Register>,
    /// LPSPI Control Register (CR)
    _reserved0: [u8; 8],
    cr: ReadWrite<u32, CR::Register>,
    /// LPSPI Status Register (SR)
    sr: ReadWrite<u32, SR::Register>,
    /// LPSPI Interrupt Enable Register (IER)
    ier: ReadWrite<u32, IER::Register>,
    /// LPSPI DMA Enable Register (DER)
    der: ReadWrite<u32, DER::Register>,
    /// LPSPI Configuration Register 0 (CFGR0)
    cfgr0: ReadWrite<u32, CFGR0::Register>,
    /// LPSPI Configuration Register 1 (CFGR1)
    cfgr1: ReadWrite<u32, CFGR1::Register>,
    _reserved1: [u8; 8],
    /// LPSPI Data Match Register 0 (DMR0)
    dmr0: ReadWrite<u32, DMR0::Register>,
    /// LPSPI Data Match Register 1 (DMR1)
    dmr1: ReadWrite<u32, DMR1::Register>,
    _reserved2: [u8; 8],
    /// LPSPI Clock Configuration Register (CCR)
    ccr: ReadWrite<u32, CCR::Register>,
    _reserved3: [u8; 20],
    /// LPSPI FIFO Control Register (FCR)
    fcr: ReadWrite<u32, FCR::Register>,
    /// LPSPI FIFO Status Register (FSR)
    fsr: ReadWrite<u32, FSR::Register>,
    /// LPSPI Transmit Command Register (TCR)
    tcr: ReadWrite<u32, TCR::Register>,
    /// LPSPI Transmit Data Register (TDR)
    tdr: ReadWrite<u32, TDR::Register>,
    _reserved4: [u8; 8],
    /// LPSPI Receive Status Register (RSR)
    rsr: ReadWrite<u32, RSR::Register>,
    /// LPSPI Receive Data Register (RDR)
    rdr: ReadWrite<u32, RDR::Register>,
}

register_bitfields![u32,
    VERID [
        /// Major Version Number
        MAJOR OFFSET(24) NUMBITS(8) [],
        /// Minor Version Number
        MINOR OFFSET(16) NUMBITS(8) [],
        /// Feature Identification Number
        FEATURE OFFSET(0) NUMBITS(16) []
    ],

    PARAM [
        /// Receive FIFO Size
        RXFIFO OFFSET(8) NUMBITS(8) [],
        /// Transmit FIFO Size
        TXFIFO OFFSET(0) NUMBITS(8) []
    ],

    CR [
        /// Reset Receive FIFO
        RRF OFFSET(9) NUMBITS(1) [],
        /// Reset Transmit FIFO
        RTF OFFSET(8) NUMBITS(1) [],
        /// Debug Enable
        DBGEN OFFSET(3) NUMBITS(1) [],
        /// Doze Mode Enable
        DOZEN OFFSET(2) NUMBITS(1) [],
        /// Software Reset
        RST OFFSET(1) NUMBITS(1) [],
        /// Module Enable
        MEN OFFSET(0) NUMBITS(1) []
    ],

    SR [
        /// Module Busy Flag
        MBF OFFSET(24) NUMBITS(1) [],
        /// Data Match Flag
        DMF OFFSET(13) NUMBITS(1) [],
        /// Receive Error Flag
        REF OFFSET(12) NUMBITS(1) [],
        /// Transmit Error Flag
        TEF OFFSET(11) NUMBITS(1) [],
        /// Transfer Complete Flag
        TCF OFFSET(10) NUMBITS(1) [],
        /// Frame Complete Flag
        FCF OFFSET(9) NUMBITS(1) [],
        /// Word Complete Flag
        WCF OFFSET(8) NUMBITS(1) [],
        /// Receive Data Flag
        RDF OFFSET(1) NUMBITS(1) [],
        /// Transmit Data Flag
        TDF OFFSET(0) NUMBITS(1) []
    ],

    IER [
        /// Data Match Interrupt Enable
        DMIE OFFSET(13) NUMBITS(1) [],
        /// Receive Error Interrupt Enable
        REIE OFFSET(12) NUMBITS(1) [],
        /// Transmit Error Interrupt Enable
        TEIE OFFSET(11) NUMBITS(1) [],
        /// Transfer Complete Interrupt Enable
        TCIE OFFSET(10) NUMBITS(1) [],
        /// Frame Complete Interrupt Enable
        FCIE OFFSET(9) NUMBITS(1) [],
        /// Word Complete Interrupt Enable
        WCIE OFFSET(8) NUMBITS(1) [],
        /// Receive Data Interrupt Enable
        RDIE OFFSET(1) NUMBITS(1) [],
        /// Transmit Data Interrupt Enable
        TDIE OFFSET(0) NUMBITS(1) []
    ],

    DER [
        /// Receive Data DMA Enable
        RDDE OFFSET(1) NUMBITS(1) [],
        /// Transmit Data DMA Enable
        TDDE OFFSET(0) NUMBITS(1) []
    ],

    CFGR0 [
        /// Receive Data Match Only
        RDMO OFFSET(9) NUMBITS(1) [],
        /// Circular FIFO Enable
        CIRFIFO OFFSET(8) NUMBITS(1) [],
        /// Host Request Select
        HRSEL OFFSET(2) NUMBITS(1) [],
        /// Host Request Polarity
        HRPOL OFFSET(1) NUMBITS(1) [],
        /// Host Request Enable
        HREN OFFSET(0) NUMBITS(1) []
    ],

    CFGR1 [
        /// Peripheral Chip Select Configuration
        PCSCFG OFFSET(27) NUMBITS(1) [],
        /// Output Configuration
        OUTCFG OFFSET(26) NUMBITS(1) [],
        /// Pin Configuration
        PINCFG OFFSET(24) NUMBITS(2) [],
        /// Match Configuration
        MATCFG OFFSET(16) NUMBITS(3) [],
        /// Peripheral Chip Select Polarity
        PCSPOL OFFSET(8) NUMBITS(4) [],
        /// No Stall
        NOSTALL OFFSET(3) NUMBITS(1) [],
        /// Automatic PCS
        AUTOPCS OFFSET(2) NUMBITS(1) [],
        /// Sample Point
        SAMPLE OFFSET(1) NUMBITS(1) [],
        /// Master Mode
        MASTER OFFSET(0) NUMBITS(1) []
    ],

    DMR0 [
        /// Match 0 Value
        MATCH0 OFFSET(0) NUMBITS(32) []
    ],

    DMR1 [
        /// Match 0 Value
        MATCH1 OFFSET(0) NUMBITS(32) []
    ],

    MATCH [
        /// Match Address 2
        MA2 OFFSET(16) NUMBITS(10) [],
        /// Match Address 1
        MA1 OFFSET(0) NUMBITS(10) []
    ],

    CCR [
        /// SCK-to-PCS Delay
        SCKPCS OFFSET(24) NUMBITS(8) [],
        /// PCS-to-SCK Delay
        PCSSCK OFFSET(16) NUMBITS(8) [],
        /// Delay Between Transfers
        DBT OFFSET(8) NUMBITS(8) [],
        /// SCK Divider
        SCKDIV OFFSET(0) NUMBITS(8) []
    ],

    FCR [
        /// Receive FIFO Watermark
        RXWATER OFFSET(16) NUMBITS(2) [],
        /// Transmit FIFO Watermark
        TXWATER OFFSET(0) NUMBITS(2) []
    ],

    FSR [
        /// Receive FIFO Count
        RXCOUNT OFFSET(16) NUMBITS(3) [],
        /// Transmit FIFO Count
        TXCOUNT OFFSET(0) NUMBITS(3) []
    ],

    TCR [
        /// Clock Polarity
        CPOL OFFSET(31) NUMBITS(1) [],
        /// Clock Phase
        CPHA OFFSET(30) NUMBITS(1) [],
        /// Prescaler Value
        PRESCALE OFFSET(27) NUMBITS(3) [],
        /// Peripheral Chip Select
        PCS OFFSET(24) NUMBITS(2) [],
        /// LSB First
        LSBF OFFSET(23) NUMBITS(1) [],
        /// Byte Swap
        BYSW OFFSET(22) NUMBITS(1) [],
        /// Continuous Transfer
        CONT OFFSET(21) NUMBITS(1) [],
        /// Continuing Command
        CONTC OFFSET(20) NUMBITS(1) [],
        /// Receive Data Mask
        RXMSK OFFSET(19) NUMBITS(1) [],
        /// Transmit Data Mask
        TXMSK OFFSET(18) NUMBITS(1) [],
        /// Transfer Width
        WIDTH OFFSET(16) NUMBITS(2) [],
        /// Frame Size
        FRAMESZ OFFSET(0) NUMBITS(12) []
    ],

    TDR [
        /// Transmit Data
        DATA OFFSET(0) NUMBITS(32) []
    ],

    RSR [
        /// RX FIFO Empty
        RXEMPTY OFFSET(1) NUMBITS(1) [],
        /// Start Of Frame
        SOF OFFSET(0) NUMBITS(1) []
    ],

    RDR [
        /// Receive Data
        DATA OFFSET(0) NUMBITS(32) []
    ]
];

struct LpspiClock<'a>(pcc::PeripheralClock<'a>);

impl ClockInterface for LpspiClock<'_> {
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

const LPSPI_BASE: [StaticRef<LpspiRegisters>; 3] = [
    unsafe { StaticRef::new(0x4002_C000 as *const LpspiRegisters) },
    unsafe { StaticRef::new(0x4002_D000 as *const LpspiRegisters) },
    unsafe { StaticRef::new(0x4002_E000 as *const LpspiRegisters) },
];
#[derive(Copy, Clone, PartialEq)]
enum LPSPIStateTX {
    Idle,
    Transmitting,
    AbortRequested,
}

pub struct LPSPI<'a> {
    registers: StaticRef<LpspiRegisters>,
    clock: LpspiClock<'a>,
    tx_dma_source: dma::DmaHardwareSource,
    rx_dma_source: dma::DmaHardwareSource,
}

impl<'a> LPSPI<'a> {
    pub fn new_lpspi0(pcc: &'a pcc::Pcc) -> Self {
        LPSPI::new(
            LPSPI_BASE[0],
            LpspiClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccLPSPI0Gate,
            )),
            dma::DmaHardwareSource::Lpuart1Transfer,
            dma::DmaHardwareSource::Lpuart1Receive,
        )
    }

    pub fn new_lpspi1(pcc: &'a pcc::Pcc) -> Self {
        LPSPI::new(
            LPSPI_BASE[1],
            LpspiClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccLPSPI1Gate,
            )),
            dma::DmaHardwareSource::Lpuart1Transfer,
            dma::DmaHardwareSource::Lpuart1Receive,
        )
    }

    pub fn new_lpspi2(pcc: &'a pcc::Pcc) -> Self {
        LPSPI::new(
            LPSPI_BASE[2],
            LpspiClock(pcc::PeripheralClock::new(
                pcc,
                pcc::ClockGate::PccLPSPI2Gate,
            )),
            dma::DmaHardwareSource::Lpuart1Transfer,
            dma::DmaHardwareSource::Lpuart1Receive,
        )
    }

    fn new(
        base_addr: StaticRef<LpspiRegisters>,
        clock: LpspiClock<'a>,
        tx_dma_source: dma::DmaHardwareSource,
        rx_dma_source: dma::DmaHardwareSource,
    ) -> LPSPI<'a> {
        LPSPI {
            registers: base_addr,
            clock: clock,
            tx_dma_source,
            rx_dma_source,
        }
    }

    pub fn init(&self) {
        //ral::write_reg!(ral::lpspi, spi.lpspi, CR, RST: RST_1);
        self.registers.cr.modify(CR::RST::SET);
        // ral::write_reg!(ral::lpspi, spi.lpspi, CR, RST: RST_0);
        self.registers.cr.modify(CR::RST::CLEAR);
        // ral::write_reg!(
        //     ral::lpspi,
        //     spi.lpspi,
        //     CFGR1,
        //     MASTER: MASTER_1,
        //     SAMPLE: SAMPLE_1
        // );
        self.registers
            .cfgr1
            .modify(CFGR1::MASTER::SET + CFGR1::SAMPLE::SET);
        // Disabled::set_mode(&self, MODE_0);
        self.registers
            .tcr
            .modify(TCR::CPOL::CLEAR + TCR::CPHA::CLEAR);
        // ral::write_reg!(ral::lpspi, spi.lpspi, FCR, RXWATER: 0xF, TXWATER: 0xF);
        // ral::write_reg!(ral::lpspi, spi.lpspi, CR, MEN: MEN_1);
        self.registers
            .fcr
            .modify(FCR::RXWATER.val(0x0F) + FCR::TXWATER.val(0xF));
        self.registers.cr.modify(CR::MEN::SET);
    }

    /// Indicates if the driver is (`true`) or is not (`false`) enabled.
    pub fn is_enabled(&self) -> bool {
        // ral::read_reg!(ral::lpspi, self.lpspi, CR, MEN == MEN_1)
        self.registers.cr.is_set(CR::MEN) as bool
    }

    /// Enable (`true`) or disable (`false`) the peripheral.
    pub fn set_enable(&mut self, enable: bool) {
        // ral::modify_reg!(ral::lpspi, self.lpspi, CR, MEN: enable as u32)
        self.registers.cr.modify(CR::MEN::SET);
    }

    /// Reset the driver.
    ///
    /// Note that this may not not reset all peripheral state, like the
    /// enabled state.
    pub fn reset(&mut self) {
        // ral::modify_reg!(ral::lpspi, self.lpspi, CR, RST: RST_1);
        self.registers.cr.modify(CR::RST::SET);
        while self.registers.cr.is_set(CR::RST) {
            self.registers.cr.modify(CR::RST::CLEAR);
        }
        // while ral::read_reg!(ral::lpspi, self.lpspi, CR, RST == RST_1) {
        //     ral::modify_reg!(ral::lpspi, self.lpspi, CR, RST: RST_0);
        // }
    }

    /// Returns the bit order configuration.
    ///
    /// See notes in [`set_bit_order`](Lpspi::set_bit_order) to
    /// understand when this configuration takes effect.
    pub fn bit_order(&self) -> BitOrder {
        self.bit_order()
    }

    // /// Set the bit order configuration.
    // ///
    // /// This applies to all higher-level write and transfer operations.
    // /// If you're using the [`Transaction`] API with manual word reads
    // /// and writes, set the configuration as part of the transaction.
    // pub fn set_bit_order(&mut self, bit_order: BitOrder) {
    //     self.bit_order = bit_order;
    // }

    // /// Temporarily disable the LPSPI peripheral.
    // ///
    // /// The handle to a [`Disabled`](crate::lpspi::Disabled) driver lets you modify
    // /// LPSPI settings that require a fully disabled peripheral. This will clear the transmit
    // /// and receive FIFOs.
    // pub fn disabled<R>(&mut self, func: impl FnOnce(&mut Disabled<N>) -> R) -> R {
    //     self.clear_fifos();
    //     let mut disabled = Disabled::new(&mut self.lpspi);
    //     func(&mut disabled)
    // }

    /// Read the status register.
    pub fn status(&self) -> u32 {
        // Status::from_bits_truncate(ral::read_reg!(ral::lpspi, self.lpspi, SR))
        self.registers.sr.get()
    }

    /// Clear the status flags.
    ///
    /// To clear status flags, set them high, then call `clear_status()`.
    ///
    /// The implementation will ensure that only the W1C bits are written, so it's
    /// OK to supply `Status::all()` to clear all bits.
    pub fn clear_status(&self) {
        self.registers.sr.modify(
            SR::WCF::CLEAR
                + SR::FCF::CLEAR
                + SR::TCF::CLEAR
                + SR::TEF::CLEAR
                + SR::REF::CLEAR
                + SR::DMF::CLEAR,
        );
    }

    /// Read the interrupt enable bits.
    pub fn interrupts(&self) -> u32 {
        // Interrupts::from_bits_truncate(ral::read_reg!(ral::lpspi, self.lpspi, IER));
        self.registers.ier.get()
    }

    /// Set the interrupt enable bits.
    ///
    /// This writes the bits described by `interrupts` as is to the register.
    /// To modify the existing interrupts flags, you should first call [`interrupts`](Lpspi::interrupts)
    /// to get the current state, then modify that state.
    pub fn set_interrupts(&self, interrupts: u32) {
        self.registers.ier.set(interrupts)
    }

    /// Clear any existing data in the SPI receive or transfer FIFOs.
    #[inline]
    pub fn clear_fifo(&mut self, direction: Direction) {
        match direction {
            Direction::Tx => self.registers.cr.modify(CR::RTF::SET),
            Direction::Rx => self.registers.cr.modify(CR::RRF::SET),
        }
    }

    /// Clear both FIFOs.
    pub fn clear_fifos(&mut self) {
        self.registers.cr.modify(CR::RTF::SET + CR::RRF::SET);
    }

    /// Returns the watermark level for the given direction.
    #[inline]
    pub fn watermark(&self, direction: Direction) -> u8 {
        (match direction {
            Direction::Rx => self.registers.fcr.read(FCR::RXWATER),
            Direction::Tx => self.registers.fcr.read(FCR::TXWATER),
        }) as u8
    }

    /// Returns the FIFO status.
    #[inline]
    pub fn fifo_status(&self) -> FifoStatus {
        let (rxcount, txcount) = (
            self.registers.fsr.read(FSR::RXCOUNT) as u16,
            self.registers.fsr.read(FSR::TXCOUNT),
        );
        FifoStatus {
            rxcount: rxcount as u16,
            txcount: txcount as u16,
        }
    }

    /// Simply read whatever is in the receiver data register.
    fn read_data_unchecked(&self) -> u32 {
        // ral::read_reg!(ral::lpspi, self.lpspi, RDR)
        self.registers.rdr.get()
    }

    /// Read the data register.
    ///
    /// Returns `None` if the receive FIFO is empty. Otherwise, returns the complete
    /// read of the register. You're reponsible for interpreting the raw value as
    /// a data word, depending on the frame size.
    pub fn read_data(&mut self) -> Option<u32> {
        if !self.registers.rsr.is_set(RSR::RXEMPTY) {
            Some(self.read_data_unchecked())
        } else {
            None
        }
    }

    /// Check for any receiver errors.
    fn recv_ok(&self) -> Result<(), LpspiError> {
        let status = self.status();
        if ((status >> 12) & 0x01) != 0 {
            Err(LpspiError::Fifo(Direction::Rx))
        } else {
            Ok(())
        }
    }

    /// Place `word` into the transmit FIFO.
    ///
    /// This will result in the value being sent from the LPSPI.
    /// You're responsible for making sure that the transmit FIFO can
    /// fit this word.
    pub fn enqueue_data(&self, word: u8) {
        self.registers.tdr.set(word as u32)
        // ral::write_reg!(ral::lpspi, self.lpspi, TDR, word);
    }

    pub(crate) fn wait_for_transmit_fifo_space(&mut self) -> Result<(), LpspiError> {
        loop {
            let status = self.status();
            if ((status >> 11) & 0x01) != 0 {
                return Err(LpspiError::Fifo(Direction::Tx));
            }
            let fifo_status = self.fifo_status();
            if !fifo_status.is_full(Direction::Tx) {
                return Ok(());
            }
        }
    }

    /// Place a transaction definition into the transmit FIFO.
    ///
    /// Once this definition is popped from the transmit FIFO, this may
    /// affect, or abort, any ongoing transactions.
    ///
    /// You're responsible for making sure there's space in the transmit
    /// FIFO for this transaction command.
    pub fn enqueue_transaction(&mut self, transaction: &Transaction) {
        self.registers.tcr.modify(
            TCR::LSBF.val(transaction.bit_order as u32)
                + TCR::BYSW.val(transaction.byte_swap as u32)
                + TCR::RXMSK.val(transaction.receive_data_mask as u32)
                + TCR::TXMSK.val(transaction.transmit_data_mask as u32)
                + TCR::FRAMESZ.val(transaction.frame_size as u32)
                + TCR::CONT.val(transaction.continuous as u32)
                + TCR::CONTC.val(transaction.continuing as u32),
        );
        // ral::modify_reg!(ral::lpspi, self.lpspi, TCR,
        //     LSBF: transaction.bit_order as u32,
        //     BYSW: transaction.byte_swap as u32,
        //     RXMSK: transaction.receive_data_mask as u32,
        //     TXMSK: transaction.transmit_data_mask as u32,
        //     FRAMESZ: transaction.frame_size as u32,
        //     CONT: transaction.continuous as u32,
        //     CONTC: transaction.continuing as u32
        // );
    }

    /// Exchanges data with the SPI device.
    ///
    /// This routine uses continuous transfers to perform the transaction, no matter the
    /// primitive type. There's an optimization for &[u32] that we're missing; in this case,
    /// we don't necessarily need to use continuous transfers. The frame size could be set to
    /// 8 * buffer.len() * sizeof(u32), and we copy user words into the transmit queue as-is.
    /// But handling the packing of u8s and u16s into the u32 transmit queue in software is
    /// extra work, work that's effectively achieved when we use continuous transfers.
    /// We're guessing that the time to pop a transmit command from the queue is much faster
    /// than the time taken to pop from the data queue, so the extra queue utilization shouldn't
    /// matter.
    pub fn exchange(
        &mut self,
        pcs: u32,
        tx_buffer: *const u8,
        tx_len: u8,
        rx_buffer: *mut u8,
        rx_len: u8,
    ) -> Result<(), LpspiError> {
        if (self.status() | (1 << 24)) == (1 << 24) {
            return Err(LpspiError::Busy);
        }
        self.registers.tcr.modify(TCR::PCS.val(pcs));

        self.clear_fifos();

        let mut transaction = Transaction::new(8 as u16)?;
        // transaction.bit_order = self.bit_order();
        transaction.continuous = true;
        // transaction.receive_data_mask = true;

        let mut tx_idx = 0u8;
        let mut rx_idx = 0u8;

        // Continue looping while there is either tx OR rx remaining
        while tx_idx < tx_len || rx_idx < rx_len {
            if tx_idx < tx_len {
                let mut word = tx_buffer.wrapping_offset(tx_idx as isize);

                // Turn off TCR CONT on last tx as a workaround so that the final
                // falling edge comes through:
                // https://community.nxp.com/t5/i-MX-RT/RT1050-LPSPI-last-bit-not-completing-in-continuous-mode/m-p/898460
                if tx_idx + 1 == tx_len {
                    transaction.continuous = false;
                }

                self.wait_for_transmit_fifo_space()?;
                self.enqueue_transaction(&transaction);

                self.wait_for_transmit_fifo_space()?;
                self.enqueue_data(unsafe { *word });
                transaction.continuing = true;
                tx_idx += 1;
            }

            if rx_idx < rx_len {
                self.recv_ok()?;
                if let Some(word) = self.read_data() {
                    unsafe {
                        *(rx_buffer.wrapping_offset(rx_idx as isize)) = word as u8;
                    }
                    rx_idx += 1;
                }
            }
        }

        self.registers.tcr.modify(TCR::PCS::CLEAR);

        Ok(())
    }

    /// Sets the clock speed parameters.
    ///
    /// This should only happen when the LPSPI peripheral is disabled.
    pub fn set_spi_clock(&self, source_clock_hz: u32, spi_clock_hz: u32) {
        let mut div = source_clock_hz / spi_clock_hz;

        if source_clock_hz / div > spi_clock_hz {
            div += 1;
        }

        // 0 <= div <= 255, and the true coefficient is really div + 2
        let div = div.saturating_sub(2).clamp(0, 255);

        self.registers.ccr.modify(
            CCR::SCKDIV.val(div)
                + CCR::DBT.val(div / 2)
                + CCR::SCKPCS.val(0x1f)
                + CCR::PCSSCK.val(0x1f),
        );
    }

    /// Write data to the transmit queue without subsequently reading
    /// the receive queue.
    ///
    /// Use this method when you know that the receiver queue is disabled
    /// (RXMASK high in TCR).
    ///
    /// Similar to `exchange`, this is using continuous transfers for all supported primitives.
    pub fn write_no_read(
        &mut self,
        pcs: u32,
        buffer: *const u8,
        len: u8,
    ) -> Result<(), LpspiError> {
        let status = self.status();

        if (status >> 24) & 0x01 == 0x01 {
            return Err(LpspiError::Busy);
        }

        self.registers.tcr.modify(TCR::PCS.val(pcs));
        self.clear_fifos();

        let mut transaction = Transaction::new(8 as u16)?;
        // transaction.bit_order = self.bit_order();
        transaction.continuous = true;
        transaction.receive_data_mask = true;
        let mut tx_idx = 0;

        while tx_idx < len {
            let mut word = buffer.wrapping_offset(tx_idx as isize);
            self.wait_for_transmit_fifo_space()?;
            self.enqueue_transaction(&transaction);

            self.wait_for_transmit_fifo_space()?;
            self.enqueue_data((unsafe { *word }));
            transaction.continuing = true;
            tx_idx += 1;
        }

        transaction.continuing = false;
        transaction.continuous = false;

        self.wait_for_transmit_fifo_space()?;
        self.enqueue_transaction(&transaction);

        self.registers.tcr.modify(TCR::PCS::CLEAR);

        Ok(())
    }

    /// Let the peripheral act as a DMA source.
    ///
    /// After this call, the peripheral will signal to the DMA engine whenever
    /// it has data available to read.
    pub fn enable_dma_receive(&mut self) {
        self.registers.fcr.modify(FCR::RXWATER::CLEAR);
        self.registers.der.modify(DER::RDDE::SET);
        // ral::modify_reg!(ral::lpspi, self.lpspi, FCR, RXWATER: 0); // No watermarks; affects DMA signaling
        // ral::modify_reg!(ral::lpspi, self.lpspi, DER, RDDE: 1);
    }

    /// Stop the peripheral from acting as a DMA source.
    ///
    /// See the DMA chapter in the reference manual to understand when this
    /// should be called in the DMA transfer lifecycle.
    pub fn disable_dma_receive(&mut self) {
        while self.registers.der.is_set(DER::RDDE) {
            self.registers.der.modify(DER::RDDE::CLEAR);
        }

        // while ral::read_reg!(ral::lpspi, self.lpspi, DER, RDDE == 1) {
        //     ral::modify_reg!(ral::lpspi, self.lpspi, DER, RDDE: 0);
        // }
    }

    /// Let the peripheral act as a DMA destination.
    ///
    /// After this call, the peripheral will signal to the DMA engine whenever
    /// it has free space in its transfer buffer.
    pub fn enable_dma_transmit(&mut self) {
        self.registers.fcr.modify(FCR::TXWATER::CLEAR);
        self.registers.der.modify(DER::TDDE::SET);
        // ral::modify_reg!(ral::lpspi, self.lpspi, FCR, TXWATER: 0); // No watermarks; affects DMA signaling
        // ral::modify_reg!(ral::lpspi, self.lpspi, DER, TDDE: 1);
    }

    /// Stop the peripheral from acting as a DMA destination.
    ///
    /// See the DMA chapter in the reference manual to understand when this
    /// should be called in the DMA transfer lifecycle.
    pub fn disable_dma_transmit(&mut self) {
        while self.registers.der.is_set(DER::TDDE) {
            self.registers.der.modify(DER::TDDE::CLEAR);
        }

        // while ral::read_reg!(ral::lpspi, self.lpspi, DER, TDDE == 1) {
        //     ral::modify_reg!(ral::lpspi, self.lpspi, DER, TDDE: 0);
        // }
    }

    // /// Produces a pointer to the receiver data register.
    // ///
    // /// You should use this pointer when coordinating a DMA transfer.
    // /// You're not expected to read from this pointer in software.
    // pub fn rdr(&self) -> *const ral::RORegister<u32> {
    //     core::ptr::addr_of!(self.lpspi.RDR)
    // }

    // /// Produces a pointer to the transfer data register.
    // ///
    // /// You should use this pointer when coordinating a DMA transfer.
    // /// You're not expected to read from this pointer in software.
    // pub fn tdr(&self) -> *const ral::WORegister<u32> {
    //     core::ptr::addr_of!(self.lpspi.TDR)
    // }
}

// bitflags::bitflags! {
//     /// Status flags for the LPSPI interface.
//     pub struct Status : u32 {
//         /// Module busy flag.
//         ///
//         /// This flag is read only.
//         const BUSY = 1 << 24;

//         //
//         // Start W1C bits.
//         //

//         /// Data match flag.
//         ///
//         /// Indicates that received data has matched one or both of the match
//         /// fields. To clear this flag, write this bit to the status register
//         /// (W1C).
//         const DATA_MATCH = 1 << 13;
//         /// Receive error flag.
//         ///
//         /// Set when the receive FIFO has overflowed. Before clearing this bit,
//         /// empty the receive FIFO. Then, write this bit to clear the flag (W1C).
//         const RECEIVE_ERROR = 1 << 12;
//         /// Transmit error flag.
//         ///
//         /// Set when the transmit FIFO has underruns. Before clearing this bit,
//         /// end the transfer. Then, write this bit to clear the flag (W1C).
//         const TRANSMIT_ERROR = 1 << 11;
//         /// Transfer complete flag.
//         ///
//         /// Set when the LPSPI returns to an idle state, and the transmit FIFO
//         /// is empty. To clear this flag, write this bit (W1C).
//         const TRANSFER_COMPLETE = 1 << 10;
//         /// Frame complete flag.
//         ///
//         /// Set at the end of each frame transfer, when PCS negates. To clear this
//         /// flag, write this bit (W1C).
//         const FRAME_COMPLETE = 1 << 9;
//         /// Word complete flag.
//         ///
//         /// Set when the last bit of a received word is sampled. To clear this flag, write
//         /// this bit (W1C).
//         const WORD_COMPLETE = 1 << 8;

//         //
//         // End W1C bits.
//         //

//         /// Receive data flag.
//         ///
//         /// Set when the number of words in the receive FIFO is greater than the watermark.
//         /// This flag is read only. To clear the flag, exhaust the receive FIFO.
//         const RECEIVE_DATA = 1 << 1;
//         /// Transmit data flag.
//         ///
//         /// Set when the number of words in the transmit FIFO is less than or equal to the
//         /// watermark. This flag is read only. TO clear the flag, fill the transmit FIFO.
//         const TRANSMIT_DATA = 1 << 0;
//     }
// }

// impl Status {
//     const W1C: Self = Self::from_bits_truncate(
//         Self::DATA_MATCH.bits()
//             | Self::RECEIVE_ERROR.bits()
//             | Self::TRANSMIT_ERROR.bits()
//             | Self::TRANSFER_COMPLETE.bits()
//             | Self::FRAME_COMPLETE.bits()
//             | Self::WORD_COMPLETE.bits(),
//     );
// }

// /// The number of words in each FIFO.
// #[derive(Debug, Clone, Copy, PartialEq, Eq)]
// pub struct FifoStatus {
//     /// Number of words in the receive FIFO.
//     pub rxcount: u16,
//     /// Number of words in the transmit FIFO.
//     pub txcount: u16,
// }

// impl FifoStatus {
//     /// Indicates if the FIFO is full for the given direction.
//     #[inline]
//     pub const fn is_full(self, direction: Direction) -> bool {
//         /// See PARAM register docs.
//         const MAX_FIFO_SIZE: u16 = 16;
//         let count = match direction {
//             Direction::Tx => self.txcount,
//             Direction::Rx => self.rxcount,
//         };
//         count >= MAX_FIFO_SIZE
//     }
// }

// bitflags::bitflags! {
//     /// Interrupt flags.
//     ///
//     /// A high bit indicates that the condition generates an interrupt.
//     /// See the status bits for more information.
//     pub struct Interrupts : u32 {
//         /// Data match interrupt enable.
//         const DATA_MATCH = 1 << 13;
//         /// Receive error interrupt enable.
//         const RECEIVE_ERROR = 1 << 12;
//         /// Transmit error interrupt enable.
//         const TRANSMIT_ERROR = 1 << 11;
//         /// Transmit complete interrupt enable.
//         const TRANSMIT_COMPLETE = 1 << 10;
//         /// Frame complete interrupt enable.
//         const FRAME_COMPLETE = 1 << 9;
//         /// Word complete interrupt enable.
//         const WORD_COMPLETE = 1 << 8;

//         /// Receive data interrupt enable.
//         const RECEIVE_DATA = 1 << 1;
//         /// Transmit data interrupt enable.
//         const TRANSMIT_DATA = 1 << 0;
//     }
// }

/// An LPSPI peripheral which is temporarily disabled.
pub struct Disabled;

impl Disabled {
    // /// Set the SPI mode for the peripheral
    // pub fn set_mode(&self, mode: Mode) {
    //     // This could probably be changed when we're not disabled.
    //     // However, there's rules about when you can read TCR.
    //     // Specifically, reading TCR while it's being loaded from
    //     // the transmit FIFO could result in an incorrect reading.
    //     // Only permitting this when we're disabled might help
    //     // us avoid something troublesome.
    //     // ral::modify_reg!(
    //     //     ral::lpspi,
    //     //     self.lpspi,
    //     //     TCR,
    //     //     CPOL: ((mode.polarity == Polarity::IdleHigh) as u32),
    //     //     CPHA: ((mode.phase == Phase::CaptureOnSecondTransition) as u32)
    //     // );
    //     LPSPI_BASE[instance]
    //         .tcr
    //         .modify(TCR::CPOL.val(mode.polarity as u32));
    //     LPSPI_BASE[instance]
    //         .tcr
    //         .modify(TCR::CPHA.val(mode.phase as u32));
    // }

    // /// Set the LPSPI clock speed (Hz).
    // ///
    // /// `source_clock_hz` is the LPSPI peripheral clock speed. To specify the
    // /// peripheral clock, see the [`ccm::lpspi_clk`](crate::ccm::lpspi_clk) documentation.
    // pub fn set_clock_hz(&mut self, source_clock_hz: u32, clock_hz: u32) {
    //     set_spi_clock(source_clock_hz, clock_hz, self.lpspi);
    // }

    // /// Set the watermark level for a given direction.
    // ///
    // /// Returns the watermark level committed to the hardware. This may be different
    // /// than the supplied `watermark`, since it's limited by the hardware.
    // ///
    // /// When `direction == Direction::Rx`, the receive data flag is set whenever the
    // /// number of words in the receive FIFO is greater than `watermark`.
    // ///
    // /// When `direction == Direction::Tx`, the transmit data flag is set whenever the
    // /// the number of words in the transmit FIFO is less than, or equal, to `watermark`.
    // #[inline]
    // pub fn set_watermark(&mut self, direction: Direction, watermark: u8) -> u8 {
    //     let max_watermark = match direction {
    //         Direction::Rx => 1 << ral::read_reg!(ral::lpspi, self.lpspi, PARAM, RXFIFO),
    //         Direction::Tx => 1 << ral::read_reg!(ral::lpspi, self.lpspi, PARAM, TXFIFO),
    //     };

    //     let watermark = watermark.min(max_watermark - 1);

    //     match direction {
    //         Direction::Rx => {
    //             ral::modify_reg!(ral::lpspi, self.lpspi, FCR, RXWATER: watermark as u32)
    //         }
    //         Direction::Tx => {
    //             ral::modify_reg!(ral::lpspi, self.lpspi, FCR, TXWATER: watermark as u32)
    //         }
    //     }

    //     watermark
    // }

    // /// Set the sampling point of the LPSPI peripheral.
    // ///
    // /// When set to `SamplePoint::DelayedEdge`, the LPSPI will sample the input data
    // /// on a delayed LPSPI_SCK edge, which improves the setup time when sampling data.
    // #[inline]
    // pub fn set_sample_point(&mut self, sample_point: SamplePoint) {
    //     match sample_point {
    //         SamplePoint::Edge => ral::modify_reg!(ral::lpspi, self.lpspi, CFGR1, SAMPLE: SAMPLE_0),
    //         SamplePoint::DelayedEdge => {
    //             ral::modify_reg!(ral::lpspi, self.lpspi, CFGR1, SAMPLE: SAMPLE_1)
    //         }
    //     }
    // }
}

// impl<const N: u8> Drop for Disabled<'_, N> {
//     fn drop(&mut self) {
//         ral::modify_reg!(ral::lpspi, self.lpspi, CR, MEN: self.men as u32);
//     }
// }

// impl<P, const N: u8> eh02::blocking::spi::Transfer<u8> for Lpspi<P, N> {
//     type Error = LpspiError;

//     fn transfer<'a>(&mut self, words: &'a mut [u8]) -> Result<&'a [u8], Self::Error> {
//         self.exchange(words)?;
//         Ok(words)
//     }
// }

// impl<P, const N: u8> eh02::blocking::spi::Transfer<u16> for Lpspi<P, N> {
//     type Error = LpspiError;

//     fn transfer<'a>(&mut self, words: &'a mut [u16]) -> Result<&'a [u16], Self::Error> {
//         self.exchange(words)?;
//         Ok(words)
//     }
// }

// impl<P, const N: u8> eh02::blocking::spi::Transfer<u32> for Lpspi<P, N> {
//     type Error = LpspiError;

//     fn transfer<'a>(&mut self, words: &'a mut [u32]) -> Result<&'a [u32], Self::Error> {
//         self.exchange(words)?;
//         Ok(words)
//     }
// }

// impl<P, const N: u8> eh02::blocking::spi::Write<u8> for Lpspi<P, N> {
//     type Error = LpspiError;

//     fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//         self.write_no_read(words)
//     }
// }

// impl<P, const N: u8> eh02::blocking::spi::Write<u16> for Lpspi<P, N> {
//     type Error = LpspiError;

//     fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
//         self.write_no_read(words)
//     }
// }

// impl<P, const N: u8> eh02::blocking::spi::Write<u32> for Lpspi<P, N> {
//     type Error = LpspiError;

//     fn write(&mut self, words: &[u32]) -> Result<(), Self::Error> {
//         self.write_no_read(words)
//     }
// }

// // Not supporting WriteIter right now. Since we don't know how many bytes we're
// // going to write, we can't specify the frame size. There might be ways around
// // this by playing with CONTC and CONT bits, but we can evaluate that later.

// /// Describes SPI words that can participate in transactions.
// trait Word: Copy + Into<u32> + TryFrom<u32> {
//     const MAX: Self;
// }

// impl Word for u8 {
//     const MAX: u8 = u8::MAX;
// }

// impl Word for u16 {
//     const MAX: u16 = u16::MAX;
// }

// impl Word for u32 {
//     const MAX: u32 = u32::MAX;
// }
