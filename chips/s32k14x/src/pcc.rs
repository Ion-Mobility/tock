use kernel::platform::chip::ClockInterface;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;

/// General-purpose I/Os
#[repr(C)]
struct PccRegisters {
    _reserved0: [u8; 128],
    pcc_ftfc: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_dmamux: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved1: [u8; 8],
    pcc_flex_can0: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_flex_can1: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_ftm3: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_adc1: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved2: [u8; 12],
    pcc_flex_can2: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_lpspi0: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_lpspi1: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_lpspi2: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved3: [u8; 8],
    pcc_pdb1: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_crc: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved4: [u8; 12],
    pcc_pdb0: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_lpit: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_ftm0: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_ftm1: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_ftm2: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_adc0: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved5: [u8; 4],
    pcc_rtc: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved6: [u8; 8],
    pcc_lptmr0: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved7: [u8; 32],
    pcc_porta: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_portb: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_portc: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_portd: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_porte: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved8: [u8; 48],
    pcc_flexio: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved9: [u8; 24],
    pcc_ewm: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved10: [u8; 16],
    pcc_lpi2c0: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved11: [u8; 12],
    pcc_lpuart0: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_lpuart1: ReadWrite<u32, PCC_FIELDS::Register>,
    pcc_lpuart2: ReadWrite<u32, PCC_FIELDS::Register>,
    _reserved12: [u8; 24],
    pcc_cmp0: ReadWrite<u32, PCC_FIELDS::Register>,
}

register_bitfields![u32,
    PCC_FIELDS [
        PR OFFSET(31) NUMBITS(1) [],
        SGC OFFSET(30) NUMBITS(1) [],
    ]
];

const PCC_BASE: StaticRef<PccRegisters> =
    unsafe { StaticRef::new(0x4006_5000 as *const PccRegisters) };

pub struct Pcc {
    registers: StaticRef<PccRegisters>,
}

impl Pcc {
    pub const fn new() -> Pcc {
        Pcc {
            registers: PCC_BASE,
        }
    }

    fn is_enable_ftfc(&self) -> bool {
        self.registers.pcc_ftfc.is_set(PCC_FIELDS::PR)
    }
    fn enable_ftfc(&self) {
        self.registers.pcc_ftfc.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_ftfc(&self) {
        self.registers.pcc_ftfc.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_dmamux(&self) -> bool {
        self.registers.pcc_dmamux.is_set(PCC_FIELDS::PR)
    }
    fn enable_dmamux(&self) {
        self.registers.pcc_dmamux.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_dmamux(&self) {
        self.registers.pcc_dmamux.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_flex_can0(&self) -> bool {
        self.registers.pcc_flex_can0.is_set(PCC_FIELDS::PR)
    }
    fn enable_flex_can0(&self) {
        self.registers.pcc_flex_can0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_flex_can0(&self) {
        self.registers.pcc_flex_can0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_flex_can1(&self) -> bool {
        self.registers.pcc_flex_can1.is_set(PCC_FIELDS::PR)
    }
    fn enable_flex_can1(&self) {
        self.registers.pcc_flex_can1.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_flex_can1(&self) {
        self.registers.pcc_flex_can1.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_ftm3(&self) -> bool {
        self.registers.pcc_ftm3.is_set(PCC_FIELDS::PR)
    }
    fn enable_ftm3(&self) {
        self.registers.pcc_ftm3.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_ftm3(&self) {
        self.registers.pcc_ftm3.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_adc1(&self) -> bool {
        self.registers.pcc_adc1.is_set(PCC_FIELDS::PR)
    }
    fn enable_adc1(&self) {
        self.registers.pcc_adc1.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_adc1(&self) {
        self.registers.pcc_adc1.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_flex_can2(&self) -> bool {
        self.registers.pcc_flex_can2.is_set(PCC_FIELDS::PR)
    }
    fn enable_flex_can2(&self) {
        self.registers.pcc_flex_can2.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_flex_can2(&self) {
        self.registers.pcc_flex_can2.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpspi0(&self) -> bool {
        self.registers.pcc_lpspi0.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpspi0(&self) {
        self.registers.pcc_lpspi0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpspi0(&self) {
        self.registers.pcc_lpspi0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpspi1(&self) -> bool {
        self.registers.pcc_lpspi1.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpspi1(&self) {
        self.registers.pcc_lpspi1.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpspi1(&self) {
        self.registers.pcc_lpspi1.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpspi2(&self) -> bool {
        self.registers.pcc_lpspi2.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpspi2(&self) {
        self.registers.pcc_lpspi2.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpspi2(&self) {
        self.registers.pcc_lpspi2.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_pdb1(&self) -> bool {
        self.registers.pcc_pdb1.is_set(PCC_FIELDS::PR)
    }
    fn enable_pdb1(&self) {
        self.registers.pcc_pdb1.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_pdb1(&self) {
        self.registers.pcc_pdb1.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_crc(&self) -> bool {
        self.registers.pcc_crc.is_set(PCC_FIELDS::PR)
    }
    fn enable_crc(&self) {
        self.registers.pcc_crc.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_crc(&self) {
        self.registers.pcc_crc.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_pdb0(&self) -> bool {
        self.registers.pcc_pdb0.is_set(PCC_FIELDS::PR)
    }
    fn enable_pdb0(&self) {
        self.registers.pcc_pdb0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_pdb0(&self) {
        self.registers.pcc_pdb0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpit(&self) -> bool {
        self.registers.pcc_lpit.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpit(&self) {
        self.registers.pcc_lpit.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpit(&self) {
        self.registers.pcc_lpit.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_ftm0(&self) -> bool {
        self.registers.pcc_ftm0.is_set(PCC_FIELDS::PR)
    }
    fn enable_ftm0(&self) {
        self.registers.pcc_ftm0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_ftm0(&self) {
        self.registers.pcc_ftm0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_ftm1(&self) -> bool {
        self.registers.pcc_ftm1.is_set(PCC_FIELDS::PR)
    }
    fn enable_ftm1(&self) {
        self.registers.pcc_ftm1.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_ftm1(&self) {
        self.registers.pcc_ftm1.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_ftm2(&self) -> bool {
        self.registers.pcc_ftm2.is_set(PCC_FIELDS::PR)
    }
    fn enable_ftm2(&self) {
        self.registers.pcc_ftm2.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_ftm2(&self) {
        self.registers.pcc_ftm2.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_adc0(&self) -> bool {
        self.registers.pcc_adc0.is_set(PCC_FIELDS::PR)
    }
    fn enable_adc0(&self) {
        self.registers.pcc_adc0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_adc0(&self) {
        self.registers.pcc_adc0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_rtc(&self) -> bool {
        self.registers.pcc_rtc.is_set(PCC_FIELDS::PR)
    }
    fn enable_rtc(&self) {
        self.registers.pcc_rtc.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_rtc(&self) {
        self.registers.pcc_rtc.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lptmr0(&self) -> bool {
        self.registers.pcc_lptmr0.is_set(PCC_FIELDS::PR)
    }
    fn enable_lptmr0(&self) {
        self.registers.pcc_lptmr0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lptmr0(&self) {
        self.registers.pcc_lptmr0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_porta(&self) -> bool {
        self.registers.pcc_porta.is_set(PCC_FIELDS::PR)
    }
    fn enable_porta(&self) {
        self.registers.pcc_porta.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_porta(&self) {
        self.registers.pcc_porta.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_portb(&self) -> bool {
        self.registers.pcc_portb.is_set(PCC_FIELDS::PR)
    }
    fn enable_portb(&self) {
        self.registers.pcc_portb.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_portb(&self) {
        self.registers.pcc_portb.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_portc(&self) -> bool {
        self.registers.pcc_portc.is_set(PCC_FIELDS::PR)
    }
    fn enable_portc(&self) {
        self.registers.pcc_portc.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_portc(&self) {
        self.registers.pcc_portc.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_portd(&self) -> bool {
        self.registers.pcc_portd.is_set(PCC_FIELDS::PR)
    }
    fn enable_portd(&self) {
        self.registers.pcc_portd.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_portd(&self) {
        self.registers.pcc_portd.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_porte(&self) -> bool {
        self.registers.pcc_porte.is_set(PCC_FIELDS::PR)
    }
    fn enable_porte(&self) {
        self.registers.pcc_porte.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_porte(&self) {
        self.registers.pcc_porte.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_flexio(&self) -> bool {
        self.registers.pcc_flexio.is_set(PCC_FIELDS::PR)
    }
    fn enable_flexio(&self) {
        self.registers.pcc_flexio.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_flexio(&self) {
        self.registers.pcc_flexio.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_ewm(&self) -> bool {
        self.registers.pcc_ewm.is_set(PCC_FIELDS::PR)
    }
    fn enable_ewm(&self) {
        self.registers.pcc_ewm.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_ewm(&self) {
        self.registers.pcc_ewm.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpi2c0(&self) -> bool {
        self.registers.pcc_lpi2c0.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpi2c0(&self) {
        self.registers.pcc_lpi2c0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpi2c0(&self) {
        self.registers.pcc_lpi2c0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpuart0(&self) -> bool {
        self.registers.pcc_lpuart0.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpuart0(&self) {
        self.registers.pcc_lpuart0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpuart0(&self) {
        self.registers.pcc_lpuart0.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpuart1(&self) -> bool {
        self.registers.pcc_lpuart1.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpuart1(&self) {
        self.registers.pcc_lpuart1.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpuart1(&self) {
        self.registers.pcc_lpuart1.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_lpuart2(&self) -> bool {
        self.registers.pcc_lpuart2.is_set(PCC_FIELDS::PR)
    }
    fn enable_lpuart2(&self) {
        self.registers.pcc_lpuart2.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_lpuart2(&self) {
        self.registers.pcc_lpuart2.modify(PCC_FIELDS::SGC::CLEAR)
    }
    fn is_enable_cmp0(&self) -> bool {
        self.registers.pcc_cmp0.is_set(PCC_FIELDS::PR)
    }
    fn enable_cmp0(&self) {
        self.registers.pcc_cmp0.modify(PCC_FIELDS::SGC::SET)
    }
    fn disable_cmp0(&self) {
        self.registers.pcc_cmp0.modify(PCC_FIELDS::SGC::CLEAR)
    }
}

/// Clock name for the peripherals
pub enum ClockGate {
    PCC_FTFC_INDEX,
    PCC_DMAMUX_INDEX,
    PCC_FLEXCAN0_INDEX,
    PCC_FLEXCAN1_INDEX,
    PCC_FTM3_INDEX,
    PCC_ADC1_INDEX,
    PCC_FLEXCAN2_INDEX,
    PCC_LPSPI0_INDEX,
    PCC_LPSPI1_INDEX,
    PCC_LPSPI2_INDEX,
    PCC_PDB1_INDEX,
    PCC_CRC_INDEX,
    PCC_PDB0_INDEX,
    PCC_LPIT_INDEX,
    PCC_FTM0_INDEX,
    PCC_FTM1_INDEX,
    PCC_FTM2_INDEX,
    PCC_ADC0_INDEX,
    PCC_RTC_INDEX,
    PCC_LPTMR0_INDEX,
    PCC_PORTA_INDEX,
    PCC_PORTB_INDEX,
    PCC_PORTC_INDEX,
    PCC_PORTD_INDEX,
    PCC_PORTE_INDEX,
    PCC_FLEXIO_INDEX,
    PCC_EWM_INDEX,
    PCC_LPI2C0_INDEX,
    PCC_LPUART0_INDEX,
    PCC_LPUART1_INDEX,
    PCC_LPUART2_INDEX,
    PCC_CMP0_INDEX,
}

pub struct PeripheralClock<'a> {
    pcc: &'a Pcc,
    clock_gate: ClockGate,
}

impl<'a> PeripheralClock<'a> {
    pub const fn new(pcc: &'a Pcc, gate: ClockGate) -> Self {
        Self {
            pcc,
            clock_gate: gate,
        }
    }
}

impl<'a> ClockInterface for PeripheralClock<'a> {
    fn is_enabled(&self) -> bool {
        match self.clock_gate {
            ClockGate::PCC_FTFC_INDEX => self.pcc.is_enable_ftfc(),
            ClockGate::PCC_DMAMUX_INDEX => self.pcc.is_enable_dmamux(),
            ClockGate::PCC_FLEXCAN0_INDEX => self.pcc.is_enable_flex_can0(),
            ClockGate::PCC_FLEXCAN1_INDEX => self.pcc.is_enable_flex_can1(),
            ClockGate::PCC_FTM3_INDEX => self.pcc.is_enable_ftm3(),
            ClockGate::PCC_ADC1_INDEX => self.pcc.is_enable_adc1(),
            ClockGate::PCC_FLEXCAN2_INDEX => self.pcc.is_enable_flex_can2(),
            ClockGate::PCC_LPSPI0_INDEX => self.pcc.is_enable_lpspi0(),
            ClockGate::PCC_LPSPI1_INDEX => self.pcc.is_enable_lpspi1(),
            ClockGate::PCC_LPSPI2_INDEX => self.pcc.is_enable_lpspi2(),
            ClockGate::PCC_PDB1_INDEX => self.pcc.is_enable_pdb1(),
            ClockGate::PCC_CRC_INDEX => self.pcc.is_enable_crc(),
            ClockGate::PCC_PDB0_INDEX => self.pcc.is_enable_pdb0(),
            ClockGate::PCC_LPIT_INDEX => self.pcc.is_enable_lpit(),
            ClockGate::PCC_FTM0_INDEX => self.pcc.is_enable_ftm0(),
            ClockGate::PCC_FTM1_INDEX => self.pcc.is_enable_ftm1(),
            ClockGate::PCC_FTM2_INDEX => self.pcc.is_enable_ftm2(),
            ClockGate::PCC_ADC0_INDEX => self.pcc.is_enable_adc0(),
            ClockGate::PCC_RTC_INDEX => self.pcc.is_enable_rtc(),
            ClockGate::PCC_LPTMR0_INDEX => self.pcc.is_enable_lptmr0(),
            ClockGate::PCC_PORTA_INDEX => self.pcc.is_enable_porta(),
            ClockGate::PCC_PORTB_INDEX => self.pcc.is_enable_portb(),
            ClockGate::PCC_PORTC_INDEX => self.pcc.is_enable_portc(),
            ClockGate::PCC_PORTD_INDEX => self.pcc.is_enable_portd(),
            ClockGate::PCC_PORTE_INDEX => self.pcc.is_enable_porte(),
            ClockGate::PCC_FLEXIO_INDEX => self.pcc.is_enable_flexio(),
            ClockGate::PCC_EWM_INDEX => self.pcc.is_enable_ewm(),
            ClockGate::PCC_LPI2C0_INDEX => self.pcc.is_enable_lpi2c0(),
            ClockGate::PCC_LPUART0_INDEX => self.pcc.is_enable_lpuart0(),
            ClockGate::PCC_LPUART1_INDEX => self.pcc.is_enable_lpuart1(),
            ClockGate::PCC_LPUART2_INDEX => self.pcc.is_enable_lpuart2(),
            ClockGate::PCC_CMP0_INDEX => self.pcc.is_enable_cmp0(),
        }
    }

    fn enable(&self) {
        match self.clock_gate {
            ClockGate::PCC_FTFC_INDEX => self.pcc.enable_ftfc(),
            ClockGate::PCC_DMAMUX_INDEX => self.pcc.enable_dmamux(),
            ClockGate::PCC_FLEXCAN0_INDEX => self.pcc.enable_flex_can0(),
            ClockGate::PCC_FLEXCAN1_INDEX => self.pcc.enable_flex_can1(),
            ClockGate::PCC_FTM3_INDEX => self.pcc.enable_ftm3(),
            ClockGate::PCC_ADC1_INDEX => self.pcc.enable_adc1(),
            ClockGate::PCC_FLEXCAN2_INDEX => self.pcc.enable_flex_can2(),
            ClockGate::PCC_LPSPI0_INDEX => self.pcc.enable_lpspi0(),
            ClockGate::PCC_LPSPI1_INDEX => self.pcc.enable_lpspi1(),
            ClockGate::PCC_LPSPI2_INDEX => self.pcc.enable_lpspi2(),
            ClockGate::PCC_PDB1_INDEX => self.pcc.enable_pdb1(),
            ClockGate::PCC_CRC_INDEX => self.pcc.enable_crc(),
            ClockGate::PCC_PDB0_INDEX => self.pcc.enable_pdb0(),
            ClockGate::PCC_LPIT_INDEX => self.pcc.enable_lpit(),
            ClockGate::PCC_FTM0_INDEX => self.pcc.enable_ftm0(),
            ClockGate::PCC_FTM1_INDEX => self.pcc.enable_ftm1(),
            ClockGate::PCC_FTM2_INDEX => self.pcc.enable_ftm2(),
            ClockGate::PCC_ADC0_INDEX => self.pcc.enable_adc0(),
            ClockGate::PCC_RTC_INDEX => self.pcc.enable_rtc(),
            ClockGate::PCC_LPTMR0_INDEX => self.pcc.enable_lptmr0(),
            ClockGate::PCC_PORTA_INDEX => self.pcc.enable_porta(),
            ClockGate::PCC_PORTB_INDEX => self.pcc.enable_portb(),
            ClockGate::PCC_PORTC_INDEX => self.pcc.enable_portc(),
            ClockGate::PCC_PORTD_INDEX => self.pcc.enable_portd(),
            ClockGate::PCC_PORTE_INDEX => self.pcc.enable_porte(),
            ClockGate::PCC_FLEXIO_INDEX => self.pcc.enable_flexio(),
            ClockGate::PCC_EWM_INDEX => self.pcc.enable_ewm(),
            ClockGate::PCC_LPI2C0_INDEX => self.pcc.enable_lpi2c0(),
            ClockGate::PCC_LPUART0_INDEX => self.pcc.enable_lpuart0(),
            ClockGate::PCC_LPUART1_INDEX => self.pcc.enable_lpuart1(),
            ClockGate::PCC_LPUART2_INDEX => self.pcc.enable_lpuart2(),
            ClockGate::PCC_CMP0_INDEX => self.pcc.enable_cmp0(),
        }
    }

    fn disable(&self) {
        match self.clock_gate {
            ClockGate::PCC_FTFC_INDEX => self.pcc.disable_ftfc(),
            ClockGate::PCC_DMAMUX_INDEX => self.pcc.disable_dmamux(),
            ClockGate::PCC_FLEXCAN0_INDEX => self.pcc.disable_flex_can0(),
            ClockGate::PCC_FLEXCAN1_INDEX => self.pcc.disable_flex_can1(),
            ClockGate::PCC_FTM3_INDEX => self.pcc.disable_ftm3(),
            ClockGate::PCC_ADC1_INDEX => self.pcc.disable_adc1(),
            ClockGate::PCC_FLEXCAN2_INDEX => self.pcc.disable_flex_can2(),
            ClockGate::PCC_LPSPI0_INDEX => self.pcc.disable_lpspi0(),
            ClockGate::PCC_LPSPI1_INDEX => self.pcc.disable_lpspi1(),
            ClockGate::PCC_LPSPI2_INDEX => self.pcc.disable_lpspi2(),
            ClockGate::PCC_PDB1_INDEX => self.pcc.disable_pdb1(),
            ClockGate::PCC_CRC_INDEX => self.pcc.disable_crc(),
            ClockGate::PCC_PDB0_INDEX => self.pcc.disable_pdb0(),
            ClockGate::PCC_LPIT_INDEX => self.pcc.disable_lpit(),
            ClockGate::PCC_FTM0_INDEX => self.pcc.disable_ftm0(),
            ClockGate::PCC_FTM1_INDEX => self.pcc.disable_ftm1(),
            ClockGate::PCC_FTM2_INDEX => self.pcc.disable_ftm2(),
            ClockGate::PCC_ADC0_INDEX => self.pcc.disable_adc0(),
            ClockGate::PCC_RTC_INDEX => self.pcc.disable_rtc(),
            ClockGate::PCC_LPTMR0_INDEX => self.pcc.disable_lptmr0(),
            ClockGate::PCC_PORTA_INDEX => self.pcc.disable_porta(),
            ClockGate::PCC_PORTB_INDEX => self.pcc.disable_portb(),
            ClockGate::PCC_PORTC_INDEX => self.pcc.disable_portc(),
            ClockGate::PCC_PORTD_INDEX => self.pcc.disable_portd(),
            ClockGate::PCC_PORTE_INDEX => self.pcc.disable_porte(),
            ClockGate::PCC_FLEXIO_INDEX => self.pcc.disable_flexio(),
            ClockGate::PCC_EWM_INDEX => self.pcc.disable_ewm(),
            ClockGate::PCC_LPI2C0_INDEX => self.pcc.disable_lpi2c0(),
            ClockGate::PCC_LPUART0_INDEX => self.pcc.disable_lpuart0(),
            ClockGate::PCC_LPUART1_INDEX => self.pcc.disable_lpuart1(),
            ClockGate::PCC_LPUART2_INDEX => self.pcc.disable_lpuart2(),
            ClockGate::PCC_CMP0_INDEX => self.pcc.disable_cmp0(),
        }
    }
}
