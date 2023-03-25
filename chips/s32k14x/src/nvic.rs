/// DMA channel 0 transfer complete */
pub const DMA0_IRQN                    : u32 = 0;               
/// DMA channel 1 transfer complete */
pub const DMA1_IRQN                    : u32 = 1;               
/// DMA channel 2 transfer complete */
pub const DMA2_IRQN                    : u32 = 2;               
/// DMA channel 3 transfer complete */
pub const DMA3_IRQN                    : u32 = 3;               
/// DMA channel 4 transfer complete */
pub const DMA4_IRQN                    : u32 = 4;               
/// DMA channel 5 transfer complete */
pub const DMA5_IRQN                    : u32 = 5;               
/// DMA channel 6 transfer complete */
pub const DMA6_IRQN                    : u32 = 6;               
/// DMA channel 7 transfer complete */
pub const DMA7_IRQN                    : u32 = 7;               
/// DMA channel 8 transfer complete */
pub const DMA8_IRQN                    : u32 = 8;               
/// DMA channel 9 transfer complete */
pub const DMA9_IRQN                    : u32 = 9;               
/// DMA channel 10 transfer complete */
pub const DMA10_IRQN                   : u32 = 10;              
/// DMA channel 11 transfer complete */
pub const DMA11_IRQN                   : u32 = 11;              
/// DMA channel 12 transfer complete */
pub const DMA12_IRQN                   : u32 = 12;              
/// DMA channel 13 transfer complete */
pub const DMA13_IRQN                   : u32 = 13;              
/// DMA channel 14 transfer complete */
pub const DMA14_IRQN                   : u32 = 14;              
/// DMA channel 15 transfer complete */
pub const DMA15_IRQN                   : u32 = 15;              
/// DMA error interrupt channels 0-15 */
pub const DMA_ERROR_IRQN               : u32 = 16;              
/// FPU sources */
pub const MCM_IRQN                     : u32 = 17;              
/// FTFC Command complete */
pub const FTFC_IRQN                    : u32 = 18;              
/// FTFC Read collision */
pub const READ_COLLISION_IRQN          : u32 = 19;              
/// PMC Low voltage detect interrupt */
pub const LVD_LVW_IRQN                 : u32 = 20;              
/// FTFC Double bit fault detect */
pub const FTFC_FAULT_IRQN              : u32 = 21;              
/// Single interrupt vector for WDOG and EWM */
pub const WDOG_EWM_IRQN                : u32 = 22;              
/// RCM Asynchronous Interrupt */
pub const RCM_IRQN                     : u32 = 23;              
/// LPI2C0 Master Interrupt */
pub const LPI2C0_MASTER_IRQN           : u32 = 24;              
/// LPI2C0 Slave Interrupt */
pub const LPI2C0_SLAVE_IRQN            : u32 = 25;              
/// LPSPI0 Interrupt */
pub const LPSPI0_IRQN                  : u32 = 26;              
/// LPSPI1 Interrupt */
pub const LPSPI1_IRQN                  : u32 = 27;              
/// LPSPI2 Interrupt */
pub const LPSPI2_IRQN                  : u32 = 28;              
/// LPI2C1 Master Interrupt */
pub const LPI2C1_MASTER_IRQN           : u32 = 29;              
/// LPI2C1 Slave Interrupt */
pub const LPI2C1_SLAVE_IRQN            : u32 = 30;              
/// LPUART0 Transmit / Receive Interrupt */
pub const LPUART0_RXTX_IRQN            : u32 = 31;              
/// LPUART1 Transmit / Receive  Interrupt */
pub const LPUART1_RXTX_IRQN            : u32 = 33;              
/// LPUART2 Transmit / Receive  Interrupt */
pub const LPUART2_RXTX_IRQN            : u32 = 35;              
/// ADC0 interrupt request. */
pub const ADC0_IRQN                    : u32 = 39;              
/// ADC1 interrupt request. */
pub const ADC1_IRQN                    : u32 = 40;              
/// CMP0 interrupt request */
pub const CMP0_IRQN                    : u32 = 41;              
/// ERM single bit error correction */
pub const ERM_SINGLE_FAULT_IRQN        : u32 = 44;              
/// ERM double bit error non-correctable */
pub const ERM_DOUBLE_FAULT_IRQN        : u32 = 45;              
/// RTC alarm interrupt */
pub const RTC_IRQN                     : u32 = 46;              
/// RTC seconds interrupt */
pub const RTC_SECONDS_IRQN             : u32 = 47;              
/// LPIT0 channel 0 overflow interrupt */
pub const LPIT0_CH0_IRQN               : u32 = 48;              
/// LPIT0 channel 1 overflow interrupt */
pub const LPIT0_CH1_IRQN               : u32 = 49;              
/// LPIT0 channel 2 overflow interrupt */
pub const LPIT0_CH2_IRQN               : u32 = 50;              
/// LPIT0 channel 3 overflow interrupt */
pub const LPIT0_CH3_IRQN               : u32 = 51;              
/// PDB0 interrupt */
pub const PDB0_IRQN                    : u32 = 52;              
/// SAI1 Transmit Synchronous interrupt (for interrupt controller) */
pub const SAI1_TX_IRQN                 : u32 = 55;              
/// SAI1 Receive Synchronous interrupt (for interrupt controller) */
pub const SAI1_RX_IRQN                 : u32 = 56;              
/// SCG bus interrupt request */
pub const SCG_IRQN                     : u32 = 57;              
/// LPTIMER interrupt request */
pub const LPTMR0_IRQN                  : u32 = 58;              
/// Port A pin detect interrupt */
pub const PORTA_IRQN                   : u32 = 59;              
/// Port B pin detect interrupt */
pub const PORTB_IRQN                   : u32 = 60;              
/// Port C pin detect interrupt */
pub const PORTC_IRQN                   : u32 = 61;              
/// Port D pin detect interrupt */
pub const PORTD_IRQN                   : u32 = 62;              
/// Port E pin detect interrupt */
pub const PORTE_IRQN                   : u32 = 63;              
/// Software interrupt */
pub const SWI_IRQN                     : u32 = 64;              
/// QSPI All interrupts ORed output */
pub const QSPI_IRQN                    : u32 = 65;              
/// PDB1 interrupt */
pub const PDB1_IRQN                    : u32 = 68;              
/// FlexIO Interrupt */
pub const FLEXIO_IRQN                  : u32 = 69;              
/// SAI0 Transmit Synchronous interrupt (for interrupt controller) */
pub const SAI0_TX_IRQN                 : u32 = 70;              
/// SAI0 Receive Synchronous interrupt (for interrupt controller) */
pub const SAI0_RX_IRQN                 : u32 = 71;              
/// ENET 1588 Timer Interrupt - synchronous */
pub const ENET_TIMER_IRQN              : u32 = 72;              
/// ENET Data transfer done */
pub const ENET_TX_IRQN                 : u32 = 73;              
/// ENET Receive Buffer Done for Ring/Queue 0 */
pub const ENET_RX_IRQN                 : u32 = 74;              
/// ENET Payload receive error. */
pub const ENET_ERR_IRQN                : u32 = 75;              
/// ENET Graceful stop */
pub const ENET_STOP_IRQN               : u32 = 76;              
/// ENET Wake from sleep. */
pub const ENET_WAKE_IRQN               : u32 = 77;              
/// CAN0 OR'ed [Bus Off OR Transmit Warning OR Receive Warning] */
pub const CAN0_ORED_IRQN               : u32 = 78;              
/// CAN0 Interrupt indicating that errors were detected on the CAN bus */
pub const CAN0_ERROR_IRQN              : u32 = 79;              
/// CAN0 Interrupt asserted when Pretended Networking operation is enabled; and a valid message matches the selected filter criteria during Low Power mode */
pub const CAN0_WAKE_UP_IRQN            : u32 = 80;              
/// CAN0 OR'ed Message buffer (0-15) */
pub const CAN0_ORED_0_15_MB_IRQN       : u32 = 81;              
/// CAN0 OR'ed Message buffer (16-31) */
pub const CAN0_ORED_16_31_MB_IRQN      : u32 = 82;              
/// CAN1 OR'ed [Bus Off OR Transmit Warning OR Receive Warning] */
pub const CAN1_ORED_IRQN               : u32 = 85;              
/// CAN1 Interrupt indicating that errors were detected on the CAN bus */
pub const CAN1_ERROR_IRQN              : u32 = 86;              
/// CAN1 OR'ed Interrupt for Message buffer (0-15) */
pub const CAN1_ORED_0_15_MB_IRQN       : u32 = 88;              
/// CAN1 OR'ed Interrupt for Message buffer (16-31) */
pub const CAN1_ORED_16_31_MB_IRQN      : u32 = 89;              
/// CAN2 OR'ed [Bus Off OR Transmit Warning OR Receive Warning] */
pub const CAN2_ORED_IRQN               : u32 = 92;              
/// CAN2 Interrupt indicating that errors were detected on the CAN bus */
pub const CAN2_ERROR_IRQN              : u32 = 93;              
/// CAN2 OR'ed Message buffer (0-15) */
pub const CAN2_ORED_0_15_MB_IRQN       : u32 = 95;              
/// CAN2 OR'ed Message buffer (16-31) */
pub const CAN2_ORED_16_31_MB_IRQN      : u32 = 96;              
/// FTM0 Channel 0 and 1 interrupt */
pub const FTM0_CH0_CH1_IRQN            : u32 = 99;              
/// FTM0 Channel 2 and 3 interrupt */
pub const FTM0_CH2_CH3_IRQN            : u32 = 100;             
/// FTM0 Channel 4 and 5 interrupt */
pub const FTM0_CH4_CH5_IRQN            : u32 = 101;             
/// FTM0 Channel 6 and 7 interrupt */
pub const FTM0_CH6_CH7_IRQN            : u32 = 102;             
/// FTM0 Fault interrupt */
pub const FTM0_FAULT_IRQN              : u32 = 103;             
/// FTM0 Counter overflow and Reload interrupt */
pub const FTM0_OVF_RELOAD_IRQN         : u32 = 104;             
/// FTM1 Channel 0 and 1 interrupt */
pub const FTM1_CH0_CH1_IRQN            : u32 = 105;             
/// FTM1 Channel 2 and 3 interrupt */
pub const FTM1_CH2_CH3_IRQN            : u32 = 106;             
/// FTM1 Channel 4 and 5 interrupt */
pub const FTM1_CH4_CH5_IRQN            : u32 = 107;             
/// FTM1 Channel 6 and 7 interrupt */
pub const FTM1_CH6_CH7_IRQN            : u32 = 108;             
/// FTM1 Fault interrupt */
pub const FTM1_FAULT_IRQN              : u32 = 109;             
/// FTM1 Counter overflow and Reload interrupt */
pub const FTM1_OVF_RELOAD_IRQN         : u32 = 110;             
/// FTM2 Channel 0 and 1 interrupt */
pub const FTM2_CH0_CH1_IRQN            : u32 = 111;             
/// FTM2 Channel 2 and 3 interrupt */
pub const FTM2_CH2_CH3_IRQN            : u32 = 112;             
/// FTM2 Channel 4 and 5 interrupt */
pub const FTM2_CH4_CH5_IRQN            : u32 = 113;             
/// FTM2 Channel 6 and 7 interrupt */
pub const FTM2_CH6_CH7_IRQN            : u32 = 114;             
/// FTM2 Fault interrupt */
pub const FTM2_FAULT_IRQN              : u32 = 115;             
/// FTM2 Counter overflow and Reload interrupt */
pub const FTM2_OVF_RELOAD_IRQN         : u32 = 116;             
/// FTM3 Channel 0 and 1 interrupt */
pub const FTM3_CH0_CH1_IRQN            : u32 = 117;             
/// FTM3 Channel 2 and 3 interrupt */
pub const FTM3_CH2_CH3_IRQN            : u32 = 118;             
/// FTM3 Channel 4 and 5 interrupt */
pub const FTM3_CH4_CH5_IRQN            : u32 = 119;             
/// FTM3 Channel 6 and 7 interrupt */
pub const FTM3_CH6_CH7_IRQN            : u32 = 120;             
/// FTM3 Fault interrupt */
pub const FTM3_FAULT_IRQN              : u32 = 121;             
/// FTM3 Counter overflow and Reload interrupt */
pub const FTM3_OVF_RELOAD_IRQN         : u32 = 122;             
/// FTM4 Channel 0 and 1 interrupt */
pub const FTM4_CH0_CH1_IRQN            : u32 = 123;             
/// FTM4 Channel 2 and 3 interrupt */
pub const FTM4_CH2_CH3_IRQN            : u32 = 124;             
/// FTM4 Channel 4 and 5 interrupt */
pub const FTM4_CH4_CH5_IRQN            : u32 = 125;             
/// FTM4 Channel 6 and 7 interrupt */
pub const FTM4_CH6_CH7_IRQN            : u32 = 126;             
/// FTM4 Fault interrupt */
pub const FTM4_FAULT_IRQN              : u32 = 127;             
/// FTM4 Counter overflow and Reload interrupt */
pub const FTM4_OVF_RELOAD_IRQN         : u32 = 128;             
/// FTM5 Channel 0 and 1 interrupt */
pub const FTM5_CH0_CH1_IRQN            : u32 = 129;             
/// FTM5 Channel 2 and 3 interrupt */
pub const FTM5_CH2_CH3_IRQN            : u32 = 130;             
/// FTM5 Channel 4 and 5 interrupt */
pub const FTM5_CH4_CH5_IRQN            : u32 = 131;             
/// FTM5 Channel 6 and 7 interrupt */
pub const FTM5_CH6_CH7_IRQN            : u32 = 132;             
/// FTM5 Fault interrupt */
pub const FTM5_FAULT_IRQN              : u32 = 133;             
/// FTM5 Counter overflow and Reload interrupt */
pub const FTM5_OVF_RELOAD_IRQN         : u32 = 134;             
/// FTM6 Channel 0 and 1 interrupt */
pub const FTM6_CH0_CH1_IRQN            : u32 = 135;             
/// FTM6 Channel 2 and 3 interrupt */
pub const FTM6_CH2_CH3_IRQN            : u32 = 136;             
/// FTM6 Channel 4 and 5 interrupt */
pub const FTM6_CH4_CH5_IRQN            : u32 = 137;             
/// FTM6 Channel 6 and 7 interrupt */
pub const FTM6_CH6_CH7_IRQN            : u32 = 138;             
/// FTM6 Fault interrupt */
pub const FTM6_FAULT_IRQN              : u32 = 139;             
/// FTM6 Counter overflow and Reload interrupt */
pub const FTM6_OVF_RELOAD_IRQN         : u32 = 140;             
/// FTM7 Channel 0 and 1 interrupt */
pub const FTM7_CH0_CH1_IRQN            : u32 = 141;             
/// FTM7 Channel 2 and 3 interrupt */
pub const FTM7_CH2_CH3_IRQN            : u32 = 142;             
/// FTM7 Channel 4 and 5 interrupt */
pub const FTM7_CH4_CH5_IRQN            : u32 = 143;             
/// FTM7 Channel 6 and 7 interrupt */
pub const FTM7_CH6_CH7_IRQN            : u32 = 144;             
/// FTM7 Fault interrupt */
pub const FTM7_FAULT_IRQN              : u32 = 145;             
/// FTM7 Counter overflow and Reload interrupt */
pub const FTM7_OVF_RELOAD_IRQN         : u32 = 146;