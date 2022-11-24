//! Peripheral implementations for the IMXRT1050 and IMXRT1060 MCUs.
//!
//! imxrt1050 chip: <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1050-evaluation-kit:MIMXRT1050-EVK>

#![crate_name = "s32k14x"]
#![crate_type = "rlib"]
#![no_std]

pub mod chip;
pub mod nvic;

// Peripherals
pub mod ccm;
pub mod ccm_analog;
pub mod dcdc;
pub mod dma;
pub mod gpio;
pub mod gpt;
pub mod iomuxc;
pub mod iomuxc_snvs;
pub mod lpi2c;
pub mod lpuart;

use cortexm4::{initialize_ram_jump_to_main, unhandled_interrupt, CortexM4, CortexMVariant};

extern "C" {
    // _estack is not really a function, but it makes the types work
    // You should never actually invoke it!!
    fn _estack();
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static BASE_VECTORS: [unsafe extern "C" fn(); 16] = [
    _estack,
    initialize_ram_jump_to_main,
    unhandled_interrupt,          // NMI
    CortexM4::HARD_FAULT_HANDLER, // Hard Fault
    unhandled_interrupt,          // MemManage
    unhandled_interrupt,          // BusFault
    unhandled_interrupt,          // UsageFault
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    CortexM4::SVC_HANDLER, // SVC
    unhandled_interrupt,   // DebugMon
    unhandled_interrupt,
    unhandled_interrupt,       // PendSV
    CortexM4::SYSTICK_HANDLER, // SysTick
];

// imxrt 1050 has total of 160 interrupts
#[cfg_attr(all(target_arch = "arm", target_os = "none"), link_section = ".irqs")]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static IRQS: [unsafe extern "C" fn(); 160] = [
    CortexM4::GENERIC_ISR, // eDMA (0)
    CortexM4::GENERIC_ISR, // eDMA (1)
    CortexM4::GENERIC_ISR, // eDMA (2)
    CortexM4::GENERIC_ISR, // eDMA (3)
    CortexM4::GENERIC_ISR, // eDMA (4)
    CortexM4::GENERIC_ISR, // eDMA (5)
    CortexM4::GENERIC_ISR, // eDMA (6)
    CortexM4::GENERIC_ISR, // eDMA (7)
    CortexM4::GENERIC_ISR, // eDMA (8)
    CortexM4::GENERIC_ISR, // eDMA (9)
    CortexM4::GENERIC_ISR, // eDMA (10)
    CortexM4::GENERIC_ISR, // eDMA (11)
    CortexM4::GENERIC_ISR, // eDMA (12)
    CortexM4::GENERIC_ISR, // eDMA (13)
    CortexM4::GENERIC_ISR, // eDMA (14)
    CortexM4::GENERIC_ISR, // eDMA (15)
    CortexM4::GENERIC_ISR, // Error_interrupt (16) - DMA error interrupt channels 0-15
    CortexM4::GENERIC_ISR, // MCM - PU sources(17)
    CortexM4::GENERIC_ISR, // ftfc/ftfm - Command Complete (18)
    CortexM4::GENERIC_ISR, // ftfc/ftfm - Read collision (19)
    CortexM4::GENERIC_ISR, // LPUART1 (20)
    CortexM4::GENERIC_ISR, // LPUART2 (21)
    CortexM4::GENERIC_ISR, // LPUART3 (22)
    CortexM4::GENERIC_ISR, // LPUART4 (23)
    CortexM4::GENERIC_ISR, // LPUART5 (24)
    CortexM4::GENERIC_ISR, // LPUART6 (25)
    CortexM4::GENERIC_ISR, // LPUART7 (26)
    CortexM4::GENERIC_ISR, // LPUART8 (27)
    CortexM4::GENERIC_ISR, // LPI2C1 (28)
    CortexM4::GENERIC_ISR, // LPI2C2 (29)
    CortexM4::GENERIC_ISR, // LPI2C3 (30)
    CortexM4::GENERIC_ISR, // LPI2C4 (31)
    CortexM4::GENERIC_ISR, // LPSPI1 (32)
    CortexM4::GENERIC_ISR, // LPSPI2 (33)
    CortexM4::GENERIC_ISR, // LPSPI3 (34)
    CortexM4::GENERIC_ISR, // LPSPI4 (35)
    CortexM4::GENERIC_ISR, // FLEXCAN1 (36)
    CortexM4::GENERIC_ISR, // FLEXCAN2 (37)
    CortexM4::GENERIC_ISR, // CM7 (38)
    CortexM4::GENERIC_ISR, // KPP (39)
    CortexM4::GENERIC_ISR, // TSC_DIG (40)
    CortexM4::GENERIC_ISR, // GPR_IRQ (41)
    CortexM4::GENERIC_ISR, // LCDIF (42)
    CortexM4::GENERIC_ISR, // CSI (43)
    CortexM4::GENERIC_ISR, // PXP (44)
    CortexM4::GENERIC_ISR, // WDOG2 (45)
    CortexM4::GENERIC_ISR, // SNVS_HP_WRAPPER (46)
    CortexM4::GENERIC_ISR, // SNVS_HP_WRAPPER (47)
    CortexM4::GENERIC_ISR, // SNVS_HP_WRAPPER / SNVS_LP_WRAPPER (48)
    CortexM4::GENERIC_ISR, // CSU (49)
    CortexM4::GENERIC_ISR, // DCP (50)
    CortexM4::GENERIC_ISR, // DCP (51)
    CortexM4::GENERIC_ISR, // DCP (52)
    CortexM4::GENERIC_ISR, // TRNG (53)
    CortexM4::GENERIC_ISR, // Reserved (54)
    CortexM4::GENERIC_ISR, // BEE (55)
    CortexM4::GENERIC_ISR, // SAI1 (56)
    CortexM4::GENERIC_ISR, // SAI2 (57)
    CortexM4::GENERIC_ISR, // SAI3 (58)
    CortexM4::GENERIC_ISR, // SAI3 (59)
    CortexM4::GENERIC_ISR, // SPDIF (60)
    CortexM4::GENERIC_ISR, // PMU (61)
    CortexM4::GENERIC_ISR, // Reserved (62)
    CortexM4::GENERIC_ISR, // Temperature Monitor (63)
    CortexM4::GENERIC_ISR, // Temperature Monitor (64)
    CortexM4::GENERIC_ISR, // USB PHY (65)
    CortexM4::GENERIC_ISR, // USB PHY (66)
    CortexM4::GENERIC_ISR, // ADC1 (67)
    CortexM4::GENERIC_ISR, // ADC2 (68)
    CortexM4::GENERIC_ISR, // DCDC (69)
    CortexM4::GENERIC_ISR, // Reserved (70)
    CortexM4::GENERIC_ISR, // Reserved (71)
    CortexM4::GENERIC_ISR, // GPIO1 (72)
    CortexM4::GENERIC_ISR, // GPIO1 (73)
    CortexM4::GENERIC_ISR, // GPIO1 (74)
    CortexM4::GENERIC_ISR, // GPIO1 (75)
    CortexM4::GENERIC_ISR, // GPIO1 (76)
    CortexM4::GENERIC_ISR, // GPIO1 (77)
    CortexM4::GENERIC_ISR, // GPIO1 (78)
    CortexM4::GENERIC_ISR, // GPIO1 (79)
    CortexM4::GENERIC_ISR, // GPIO1_1 (80)
    CortexM4::GENERIC_ISR, // GPIO1_2 (81)
    CortexM4::GENERIC_ISR, // GPIO2_1 (82)
    CortexM4::GENERIC_ISR, // GPIO2_2 (83)
    CortexM4::GENERIC_ISR, // GPIO3_1 (84)
    CortexM4::GENERIC_ISR, // GPIO3_2 (85)
    CortexM4::GENERIC_ISR, // GPIO4_1 (86)
    CortexM4::GENERIC_ISR, // GPIO4_2 (87)
    CortexM4::GENERIC_ISR, // GPIO5_1 (88)
    CortexM4::GENERIC_ISR, // GPIO5_2 (89)
    CortexM4::GENERIC_ISR, // FLEXIO1 (90)
    CortexM4::GENERIC_ISR, // FLEXIO2 (91)
    CortexM4::GENERIC_ISR, // WDOG1 (92)
    CortexM4::GENERIC_ISR, // RTWDOG (93)
    CortexM4::GENERIC_ISR, // EWM (94)
    CortexM4::GENERIC_ISR, // CCM (95)
    CortexM4::GENERIC_ISR, // CCM (96)
    CortexM4::GENERIC_ISR, // GPC (97)
    CortexM4::GENERIC_ISR, // SRC (98)
    CortexM4::GENERIC_ISR, // Reserved (99)
    CortexM4::GENERIC_ISR, // GPT1 (100)
    CortexM4::GENERIC_ISR, // GPT2 (101)
    CortexM4::GENERIC_ISR, // FLEXPWM1 (102)
    CortexM4::GENERIC_ISR, // FLEXPWM1 (103)
    CortexM4::GENERIC_ISR, // FLEXPWM1 (104)
    CortexM4::GENERIC_ISR, // FLEXPWM1 (105)
    CortexM4::GENERIC_ISR, // FLEXPWM1 (106)
    CortexM4::GENERIC_ISR, // Reserved (107)
    CortexM4::GENERIC_ISR, // FLEXSPI (108)
    CortexM4::GENERIC_ISR, // SEMC (109)
    CortexM4::GENERIC_ISR, // USDHC1 (110)
    CortexM4::GENERIC_ISR, // USDHC2 (111)
    CortexM4::GENERIC_ISR, // USB (112)
    CortexM4::GENERIC_ISR, // USB (113)
    CortexM4::GENERIC_ISR, // ENET (114)
    CortexM4::GENERIC_ISR, // ENET (115)
    CortexM4::GENERIC_ISR, // XBAR1 (116)
    CortexM4::GENERIC_ISR, // XBAR1 (117)
    CortexM4::GENERIC_ISR, // ADC_ETC (118)
    CortexM4::GENERIC_ISR, // ADC_ETC (119)
    CortexM4::GENERIC_ISR, // ADC_ETC (120)
    CortexM4::GENERIC_ISR, // ADC_ETC (121)
    CortexM4::GENERIC_ISR, // PIT (122)
    CortexM4::GENERIC_ISR, // ACMP (123)
    CortexM4::GENERIC_ISR, // ACMP (124)
    CortexM4::GENERIC_ISR, // ACMP (125)
    CortexM4::GENERIC_ISR, // ACMP (126)
    CortexM4::GENERIC_ISR, // Reserved (127)
    CortexM4::GENERIC_ISR, // Reserved (128)
    CortexM4::GENERIC_ISR, // ENC1 (129)
    CortexM4::GENERIC_ISR, // ENC2 (130)
    CortexM4::GENERIC_ISR, // ENC3 (131)
    CortexM4::GENERIC_ISR, // ENC4 (132)
    CortexM4::GENERIC_ISR, // QTIMER1 (133)
    CortexM4::GENERIC_ISR, // QTIMER2 (134)
    CortexM4::GENERIC_ISR, // QTIMER3 (135)
    CortexM4::GENERIC_ISR, // QTIMER4 (136)
    CortexM4::GENERIC_ISR, // FLEXPWM2 (137)
    CortexM4::GENERIC_ISR, // FLEXPWM2 (138)
    CortexM4::GENERIC_ISR, // FLEXPWM2 (139)
    CortexM4::GENERIC_ISR, // FLEXPWM2 (140)
    CortexM4::GENERIC_ISR, // FLEXPWM2 (141)
    CortexM4::GENERIC_ISR, // FLEXPWM3 (142)
    CortexM4::GENERIC_ISR, // FLEXPWM3 (143)
    CortexM4::GENERIC_ISR, // FLEXPWM3 (144)
    CortexM4::GENERIC_ISR, // FLEXPWM3 (145)
    CortexM4::GENERIC_ISR, // FLEXPWM3 (146)
    CortexM4::GENERIC_ISR, // FLEXPWM4 (147)
    CortexM4::GENERIC_ISR, // FLEXPWM4 (148)
    CortexM4::GENERIC_ISR, // FLEXPWM4 (149)
    CortexM4::GENERIC_ISR, // FLEXPWM4 (150)
    CortexM4::GENERIC_ISR, // FLEXPWM4 (151)
    CortexM4::GENERIC_ISR, // Reserved (152)
    CortexM4::GENERIC_ISR, // Reserved (153)
    CortexM4::GENERIC_ISR, // Reserved (154)
    CortexM4::GENERIC_ISR, // Reserved (155)
    CortexM4::GENERIC_ISR, // Reserved (156)
    CortexM4::GENERIC_ISR, // Reserved (157)
    CortexM4::GENERIC_ISR, // Reserved (158)
    CortexM4::GENERIC_ISR, // Reserved (159)
];

pub unsafe fn init() {
    cortexm4::nvic::disable_all();
    cortexm4::nvic::clear_all_pending();

    cortexm4::scb::set_vector_table_offset(
        &BASE_VECTORS as *const [unsafe extern "C" fn(); 16] as *const (),
    );

    cortexm4::nvic::enable_all();
}
#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".FlashConfig"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static BASE_VECTORS: [unsafe extern "C" data(); 4] = [
    0xFFFFFFFF,
    0xFFFFFFFF,
    0xFFFFFFFF,
    0xFFFF7FFE, // Unsecured
];