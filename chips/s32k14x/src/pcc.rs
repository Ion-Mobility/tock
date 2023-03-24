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
        PCS OFFSET(24) NUMBITS(3) [
            OFF = 0,
            SOSC = 1,
            SIRC = 2,
            FIRC = 3,
            SPLL = 6,
        ],
        FRAC OFFSET(4) NUMBITS(1) [],
        PCD  OFFSET(0) NUMBITS(4) []
    ]
];

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PeripheralClockName {
    /* Main clocks */
    /// Core clock                     
    CORE_CLK = 0,
    /// Bus clock                      
    BUS_CLK = 1,
    /// Slow clock                     
    SLOW_CLK = 2,
    /// CLKOUT clock                   
    CLKOUT_CLK = 3,

    /* Other internal clocks used by peripherals. */
    /// SIRC clock                     
    SIRC_CLK = 4,
    /// FIRC clock                     
    FIRC_CLK = 5,
    /// SOSC clock                     
    SOSC_CLK = 6,
    /// SPLL clock                     
    SPLL_CLK = 7,
    /// RTC_CLKIN clock                
    RTC_CLKIN_CLK = 8,
    /// SCG CLK_OUT clock              
    SCG_CLKOUT_CLK = 9,

    /// SIRCDIV1 functional clock      
    SIRCDIV1_CLK = 10,
    /// SIRCDIV2 functional clock      
    SIRCDIV2_CLK = 11,
    /// FIRCDIV1 functional clock      
    FIRCDIV1_CLK = 12,
    /// FIRCDIV2 functional clock      
    FIRCDIV2_CLK = 13,
    /// SOSCDIV1 functional clock      
    SOSCDIV1_CLK = 14,
    /// SOSCDIV2 functional clock      
    SOSCDIV2_CLK = 15,
    /// SPLLDIV1 functional clock      
    SPLLDIV1_CLK = 16,
    /// SPLLDIV2 functional clock      
    SPLLDIV2_CLK = 17,

    /// End of SCG clocks              
    SCG_END_OF_CLOCKS = 18,

    /* SIM clocks */
    /// FTM0 External Clock Pin Select
    SIM_FTM0_CLOCKSEL = 21,
    /// FTM1 External Clock Pin Select
    SIM_FTM1_CLOCKSEL = 22,
    /// FTM2 External Clock Pin Select
    SIM_FTM2_CLOCKSEL = 23,
    /// FTM3 External Clock Pin Select
    SIM_FTM3_CLOCKSEL = 24,
    /// CLKOUT Select                  
    SIM_CLKOUTSELL = 25,
    /// RTCCLK clock                   
    SIM_RTCCLK_CLK = 26,
    /// LPO clock                      
    SIM_LPO_CLK = 27,
    /// LPO 1KHz clock                 
    SIM_LPO_1K_CLK = 28,
    /// LPO 32KHz clock                
    SIM_LPO_32K_CLK = 29,
    /// LPO 128KHz clock               
    SIM_LPO_128K_CLK = 30,
    /// EIM clock source               
    SIM_EIM_CLK = 31,
    /// ERM clock source               
    SIM_ERM_CLK = 32,
    /// DMA clock source               
    SIM_DMA_CLK = 33,
    /// MPU clock source               
    SIM_MPU_CLK = 34,
    /// MSCM clock source              
    SIM_MSCM_CLK = 35,
    /// End of SIM clocks              
    SIM_END_OF_CLOCKS = 36,

    /* PCC clocks */
    /// CMP0 clock source              
    CMP0_CLK = 41,
    /// CRC0 clock source              
    CRC0_CLK = 42,
    /// DMAMUX0 clock source           
    DMAMUX0_CLK = 43,
    /// EWM0 clock source              
    EWM0_CLK = 44,
    /// PORTA clock source             
    PORTA_CLK = 45,
    /// PORTB clock source             
    PORTB_CLK = 46,
    /// PORTC clock source             
    PORTC_CLK = 47,
    /// PORTD clock source             
    PORTD_CLK = 48,
    /// PORTE clock source             
    PORTE_CLK = 49,
    /// RTC0 clock source              
    RTC0_CLK = 50,
    /// End of BUS clocks              
    PCC_END_OF_BUS_CLOCKS = 51,
    /// FlexCAN0 clock source          
    FlexCAN0_CLK = 52,
    /// FlexCAN1 clock source          
    FlexCAN1_CLK = 53,
    /// FlexCAN2 clock source          
    FlexCAN2_CLK = 54,
    /// PDB0 clock source              
    PDB0_CLK = 55,
    /// PDB1 clock source              
    PDB1_CLK = 56,
    /// End of SYS clocks              
    PCC_END_OF_SYS_CLOCKS = 57,
    /// FTFC0 clock source             
    FTFC0_CLK = 58,
    /// End of SLOW clocks             
    PCC_END_OF_SLOW_CLOCKS = 59,
    /// FTM0 clock source              
    FTM0_CLK = 60,
    /// FTM1 clock source              
    FTM1_CLK = 61,
    /// FTM2 clock source              
    FTM2_CLK = 62,
    /// FTM3 clock source              
    FTM3_CLK = 63,
    /// End of ASYNCH DIV1 clocks      
    PCC_END_OF_ASYNCH_DIV1_CLOCKS = 64,
    /// ADC0 clock source              
    ADC0_CLK = 65,
    /// ADC1 clock source              
    ADC1_CLK = 66,
    /// FLEXIO0 clock source           
    FLEXIO0_CLK = 67,
    /// LPI2C0 clock source            
    LPI2C0_CLK = 68,
    /// LPIT0 clock source             
    LPIT0_CLK = 69,
    /// LPSPI0 clock source            
    LPSPI0_CLK = 70,
    /// LPSPI1 clock source            
    LPSPI1_CLK = 71,
    /// LPSPI2 clock source            
    LPSPI2_CLK = 72,
    /// LPTMR0 clock source            
    LPTMR0_CLK = 73,
    /// LPUART0 clock source           
    LPUART0_CLK = 74,
    /// LPUART1 clock source           
    LPUART1_CLK = 75,
    /// LPUART2 clock source           
    LPUART2_CLK = 76,
    /// End of ASYNCH DIV2 clocks      
    PCC_END_OF_ASYNCH_DIV2_CLOCKS = 77,
    /// End of PCC clocks              
    PCC_END_OF_CLOCKS = 78,
    /// The total number of entries    
    CLOCK_NAME_COUNT = 79,
}
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PeripheralClockSource {
    /// Clock is off
    CLK_SRC_OFF = 0,
    /// OSCCLK - System Oscillator Bus Clock
    CLK_SRC_SOSC = 1,
    /// SCGIRCLK - Slow IRC Clock
    CLK_SRC_SIRC = 2,
    /// SCGFIRCLK - Fast IRC Clock
    CLK_SRC_FIRC = 3,
    /// SCGPCLK System PLL clock
    CLK_SRC_SPLL = 6,
    /// OSCCLK - System Oscillator Bus Clock
    CLK_SRC_SOSC_DIV1 = 7,
    /// SCGIRCLK - Slow IRC Clock
    CLK_SRC_SIRC_DIV1 = 8,
    /// SCGFIRCLK - Fast IRC Clock
    CLK_SRC_FIRC_DIV1 = 9,
    /// SCGPCLK System PLL clock
    CLK_SRC_SPLL_DIV1 = 10,
    /// OSCCLK - System Oscillator Bus Clock
    CLK_SRC_SOSC_DIV2 = 11,
    /// SCGIRCLK - Slow IRC Clock
    CLK_SRC_SIRC_DIV2 = 12,
    /// SCGFIRCLK - Fast IRC Clock
    CLK_SRC_FIRC_DIV2 = 13,
    /// SCGPCLK System PLL clock
    CLK_SRC_SPLL_DIV2 = 14,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PeripheralClockFrac {
    /// Fractional value is zero
    MULTIPLY_BY_ONE = 0x00,
    /// Fractional value is one            
    MULTIPLY_BY_TWO = 0x01,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PeripheralClockDiv {
    /// Divide by 1 (pass-through, no clock divide)
    DIVIDE_BY_ONE = 0x00,
    /// Divide by 2
    DIVIDE_BY_TWO = 0x01,
    /// Divide by 3
    DIVIDE_BY_THREE = 0x02,
    /// Divide by 4
    DIVIDE_BY_FOR = 0x03,
    /// Divide by 5
    DIVIDE_BY_FIVE = 0x04,
    /// Divide by 6
    DIVIDE_BY_SIX = 0x05,
    /// Divide by 7
    DIVIDE_BY_SEVEN = 0x06,
    /// Divide by 8 */      
    DIVIDE_BY_EIGTH = 0x07,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct PeripheralClockConfig {
    pub clockName: PeripheralClockName,
    /// Peripheral clock gate.                     
    pub clkGate: bool,
    /// Peripheral clock source.                   
    pub clkSrc: PeripheralClockSource,
    /// Peripheral clock fractional value.         
    pub frac: PeripheralClockFrac,
    /// Peripheral clock divider value.            
    pub divider: PeripheralClockDiv,
}

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
        self.registers
            .pcc_lpit
            .modify(PCC_FIELDS::SGC::SET + PCC_FIELDS::PCS::SOSC)
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

    /// Set the UART clock selection
    ///
    /// Should only be called when *all* UART clock gates are disabled
    // pub fn set_uart_clock_sel(&self, selection: UartClockSelection) {
    //     self.registers
    //         .cscdr1
    //         .modify(CSCDR1::UART_CLK_SEL.val(selection as u32));
    // }
    // self.registers.pcc_lpit.is_set(PCC_FIELDS::PR)

    /// Set the periodic clock selection
    pub fn set_uart_clock_sel(&self, sel: PerclkClockSel) {
        let sel = match sel {
            PerclkClockSel::OFF => PCC_FIELDS::PCS::OFF,
            PerclkClockSel::SOSC => PCC_FIELDS::PCS::SOSC,
            PerclkClockSel::SIRC => PCC_FIELDS::PCS::SIRC,
            PerclkClockSel::FIRC => PCC_FIELDS::PCS::FIRC,
            PerclkClockSel::SPLL => PCC_FIELDS::PCS::SPLL,
            PerclkClockSel::SOSC_DIV1 => PCC_FIELDS::PCS::SOSC,
            PerclkClockSel::SIRC_DIV1 => PCC_FIELDS::PCS::SIRC,
            PerclkClockSel::FIRC_DIV1 => PCC_FIELDS::PCS::FIRC,
            PerclkClockSel::SPLL_DIV1 => PCC_FIELDS::PCS::SPLL,
            PerclkClockSel::SOSC_DIV2 => PCC_FIELDS::PCS::SOSC,
            PerclkClockSel::SIRC_DIV2 => PCC_FIELDS::PCS::SIRC,
            PerclkClockSel::FIRC_DIV2 => PCC_FIELDS::PCS::FIRC,
            PerclkClockSel::SPLL_DIV2 => PCC_FIELDS::PCS::SPLL,
        };
        // self.registers.pcc_lpit.modify(PCC_FIELDS::PCS::SIRC);
    }
    pub fn PCC_SetClockMode(&self, clockname: PeripheralClockName, enable: bool) {}
}

/// Periodic clock selection for GPTs and PITs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PerclkClockSel {
    /// Clock is off
    OFF,
    /// OSCCLK - System Oscillator Bus Clock
    SOSC,
    /// SCGIRCLK - Slow IRC Clock
    SIRC,
    /// SCGFIRCLK - Fast IRC Clock
    FIRC,
    /// SCGPCLK System PLL clock
    SPLL,
    /// OSCCLK - System Oscillator Bus Clock
    SOSC_DIV1,
    /// SCGIRCLK - Slow IRC Clock
    SIRC_DIV1,
    /// SCGFIRCLK - Fast IRC Clock
    FIRC_DIV1,
    /// SCGPCLK System PLL clock
    SPLL_DIV1,
    /// OSCCLK - System Oscillator Bus Clock
    SOSC_DIV2,
    /// SCGIRCLK - Slow IRC Clock
    SIRC_DIV2,
    /// SCGFIRCLK - Fast IRC Clock
    FIRC_DIV2,
    /// SCGPCLK System PLL clock
    SPLL_DIV2,
}

/// Clock name for the peripherals
pub enum ClockGate {
    PccFTFCGate,
    PccDMAMUXGate,
    PccFLEXCAN0Gate,
    PccFLEXCAN1Gate,
    PccFTM3Gate,
    PccADC1Gate,
    PccFLEXCAN2Gate,
    PccLPSPI0Gate,
    PccLPSPI1Gate,
    PccLPSPI2Gate,
    PccPDB1Gate,
    PccCRCGate,
    PccPDB0Gate,
    PccLPITGate,
    PccFTM0Gate,
    PccFTM1Gate,
    PccFTM2Gate,
    PccADC0Gate,
    PccRTCGate,
    PccLPTMR0Gate,
    PccPORTAGate,
    PccPORTBGate,
    PccPORTCGate,
    PccPORTDGate,
    PccPORTEGate,
    PccFLEXIOGate,
    PccEWMGate,
    PccLPI2C0Gate,
    PccLPUART0Gate,
    PccLPUART1Gate,
    PccLPUART2Gate,
    PccCMP0Gate,
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
            ClockGate::PccFTFCGate => self.pcc.is_enable_ftfc(),
            ClockGate::PccDMAMUXGate => self.pcc.is_enable_dmamux(),
            ClockGate::PccFLEXCAN0Gate => self.pcc.is_enable_flex_can0(),
            ClockGate::PccFLEXCAN1Gate => self.pcc.is_enable_flex_can1(),
            ClockGate::PccFTM3Gate => self.pcc.is_enable_ftm3(),
            ClockGate::PccADC1Gate => self.pcc.is_enable_adc1(),
            ClockGate::PccFLEXCAN2Gate => self.pcc.is_enable_flex_can2(),
            ClockGate::PccLPSPI0Gate => self.pcc.is_enable_lpspi0(),
            ClockGate::PccLPSPI1Gate => self.pcc.is_enable_lpspi1(),
            ClockGate::PccLPSPI2Gate => self.pcc.is_enable_lpspi2(),
            ClockGate::PccPDB1Gate => self.pcc.is_enable_pdb1(),
            ClockGate::PccCRCGate => self.pcc.is_enable_crc(),
            ClockGate::PccPDB0Gate => self.pcc.is_enable_pdb0(),
            ClockGate::PccLPITGate => self.pcc.is_enable_lpit(),
            ClockGate::PccFTM0Gate => self.pcc.is_enable_ftm0(),
            ClockGate::PccFTM1Gate => self.pcc.is_enable_ftm1(),
            ClockGate::PccFTM2Gate => self.pcc.is_enable_ftm2(),
            ClockGate::PccADC0Gate => self.pcc.is_enable_adc0(),
            ClockGate::PccRTCGate => self.pcc.is_enable_rtc(),
            ClockGate::PccLPTMR0Gate => self.pcc.is_enable_lptmr0(),
            ClockGate::PccPORTAGate => self.pcc.is_enable_porta(),
            ClockGate::PccPORTBGate => self.pcc.is_enable_portb(),
            ClockGate::PccPORTCGate => self.pcc.is_enable_portc(),
            ClockGate::PccPORTDGate => self.pcc.is_enable_portd(),
            ClockGate::PccPORTEGate => self.pcc.is_enable_porte(),
            ClockGate::PccFLEXIOGate => self.pcc.is_enable_flexio(),
            ClockGate::PccEWMGate => self.pcc.is_enable_ewm(),
            ClockGate::PccLPI2C0Gate => self.pcc.is_enable_lpi2c0(),
            ClockGate::PccLPUART0Gate => self.pcc.is_enable_lpuart0(),
            ClockGate::PccLPUART1Gate => self.pcc.is_enable_lpuart1(),
            ClockGate::PccLPUART2Gate => self.pcc.is_enable_lpuart2(),
            ClockGate::PccCMP0Gate => self.pcc.is_enable_cmp0(),
        }
    }

    fn enable(&self) {
        match self.clock_gate {
            ClockGate::PccFTFCGate => self.pcc.enable_ftfc(),
            ClockGate::PccDMAMUXGate => self.pcc.enable_dmamux(),
            ClockGate::PccFLEXCAN0Gate => self.pcc.enable_flex_can0(),
            ClockGate::PccFLEXCAN1Gate => self.pcc.enable_flex_can1(),
            ClockGate::PccFTM3Gate => self.pcc.enable_ftm3(),
            ClockGate::PccADC1Gate => self.pcc.enable_adc1(),
            ClockGate::PccFLEXCAN2Gate => self.pcc.enable_flex_can2(),
            ClockGate::PccLPSPI0Gate => self.pcc.enable_lpspi0(),
            ClockGate::PccLPSPI1Gate => self.pcc.enable_lpspi1(),
            ClockGate::PccLPSPI2Gate => self.pcc.enable_lpspi2(),
            ClockGate::PccPDB1Gate => self.pcc.enable_pdb1(),
            ClockGate::PccCRCGate => self.pcc.enable_crc(),
            ClockGate::PccPDB0Gate => self.pcc.enable_pdb0(),
            ClockGate::PccLPITGate => self.pcc.enable_lpit(),
            ClockGate::PccFTM0Gate => self.pcc.enable_ftm0(),
            ClockGate::PccFTM1Gate => self.pcc.enable_ftm1(),
            ClockGate::PccFTM2Gate => self.pcc.enable_ftm2(),
            ClockGate::PccADC0Gate => self.pcc.enable_adc0(),
            ClockGate::PccRTCGate => self.pcc.enable_rtc(),
            ClockGate::PccLPTMR0Gate => self.pcc.enable_lptmr0(),
            ClockGate::PccPORTAGate => self.pcc.enable_porta(),
            ClockGate::PccPORTBGate => self.pcc.enable_portb(),
            ClockGate::PccPORTCGate => self.pcc.enable_portc(),
            ClockGate::PccPORTDGate => self.pcc.enable_portd(),
            ClockGate::PccPORTEGate => self.pcc.enable_porte(),
            ClockGate::PccFLEXIOGate => self.pcc.enable_flexio(),
            ClockGate::PccEWMGate => self.pcc.enable_ewm(),
            ClockGate::PccLPI2C0Gate => self.pcc.enable_lpi2c0(),
            ClockGate::PccLPUART0Gate => self.pcc.enable_lpuart0(),
            ClockGate::PccLPUART1Gate => self.pcc.enable_lpuart1(),
            ClockGate::PccLPUART2Gate => self.pcc.enable_lpuart2(),
            ClockGate::PccCMP0Gate => self.pcc.enable_cmp0(),
        }
    }

    fn disable(&self) {
        match self.clock_gate {
            ClockGate::PccFTFCGate => self.pcc.disable_ftfc(),
            ClockGate::PccDMAMUXGate => self.pcc.disable_dmamux(),
            ClockGate::PccFLEXCAN0Gate => self.pcc.disable_flex_can0(),
            ClockGate::PccFLEXCAN1Gate => self.pcc.disable_flex_can1(),
            ClockGate::PccFTM3Gate => self.pcc.disable_ftm3(),
            ClockGate::PccADC1Gate => self.pcc.disable_adc1(),
            ClockGate::PccFLEXCAN2Gate => self.pcc.disable_flex_can2(),
            ClockGate::PccLPSPI0Gate => self.pcc.disable_lpspi0(),
            ClockGate::PccLPSPI1Gate => self.pcc.disable_lpspi1(),
            ClockGate::PccLPSPI2Gate => self.pcc.disable_lpspi2(),
            ClockGate::PccPDB1Gate => self.pcc.disable_pdb1(),
            ClockGate::PccCRCGate => self.pcc.disable_crc(),
            ClockGate::PccPDB0Gate => self.pcc.disable_pdb0(),
            ClockGate::PccLPITGate => self.pcc.disable_lpit(),
            ClockGate::PccFTM0Gate => self.pcc.disable_ftm0(),
            ClockGate::PccFTM1Gate => self.pcc.disable_ftm1(),
            ClockGate::PccFTM2Gate => self.pcc.disable_ftm2(),
            ClockGate::PccADC0Gate => self.pcc.disable_adc0(),
            ClockGate::PccRTCGate => self.pcc.disable_rtc(),
            ClockGate::PccLPTMR0Gate => self.pcc.disable_lptmr0(),
            ClockGate::PccPORTAGate => self.pcc.disable_porta(),
            ClockGate::PccPORTBGate => self.pcc.disable_portb(),
            ClockGate::PccPORTCGate => self.pcc.disable_portc(),
            ClockGate::PccPORTDGate => self.pcc.disable_portd(),
            ClockGate::PccPORTEGate => self.pcc.disable_porte(),
            ClockGate::PccFLEXIOGate => self.pcc.disable_flexio(),
            ClockGate::PccEWMGate => self.pcc.disable_ewm(),
            ClockGate::PccLPI2C0Gate => self.pcc.disable_lpi2c0(),
            ClockGate::PccLPUART0Gate => self.pcc.disable_lpuart0(),
            ClockGate::PccLPUART1Gate => self.pcc.disable_lpuart1(),
            ClockGate::PccLPUART2Gate => self.pcc.disable_lpuart2(),
            ClockGate::PccCMP0Gate => self.pcc.disable_cmp0(),
        }
    }
}
