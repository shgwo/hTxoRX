#ifndef __IODEFINE_ENUM_H__
#define __IODEFINE_ENUM_H__

#include "iodefine.h"

#define ADDRESS_HOCOCR2 (( unsigned char *)0x00080037/*HOCOCR2*/)
#define ADDRESS_PMCTLTSTR (( unsigned char *) 0x0008c2e0/*PMCLTSTR*/)

// bit meanings of System Critical Protect register
enum enum_PRCR_PRCn {
  PRCn_LOCK,  // register lock
  PRCn_UNLOCK // register unlock
};

enum enum_PRCR_PRCR {
  PRKEY_LOCK,         // register lock
  PRKEY_UNLOCK=181 // register unlock
};


// bit meanings of Clock Gen Circuit Module
enum enum_SCKCR_XCK {
  // System Clock Control Register
  XCK_DIV_1,   // PLLCK
  XCK_DIV_2,   // PLLCK /2
  XCK_DIV_4,   // PLLCK /4
  XCK_DIV_8,   // PLLCK /8
  XCK_DIV_16,  // PLLCK /16
  XCK_DIV_32,  // PLLCK /32
  XCK_DIV_64,  // PLLCK /64
  XCK_DIV_XX   // prohibit
};

enum enum_SCKCR_PSTOP0 {
  // SDCLK Pin Output Control
  PSTOP0_EN,   // SDCLK enable
  PSTOP0_DE    // SDCLK_disable
};

enum enum_SCKCR_PSTOP1 {
  // BCLK Pin Output Control
  PSTOP1_EN,   // SDCLK enable
  PSTOP1_DE    // SDCLK_disable
};

enum enum_SCKCR2_IECK {
  // IEBUS Clock (IECLK)  Select
  IECK_DIV_2,   // PLLCK /2
  IECK_DIV_4,   // PLLCK /4
  IECK_DIV_8,   // PLLCK /8
  IECK_DIV_16,  // PLLCK /16
  IECK_DIV_32,  // PLLCK /32
  IECK_DIV_64,  // PLLCK /64
  IECK_DIV_6    // PLLCK /6
};

enum enum_SCKCR2_UCK {
  // USB Clock (UCLK) Select
  UCK_DIV_3,   // PLLCK /3
  UCK_DIV_4    // PLLCK /4
};

enum enum_SCKCR3_CKSEL {
  // PLL Input Frequency Division Ratio Select
  CKSEL_LOCO,
  CKSEL_HOCO,
  CKSEL_MOSC,
  CKSEL_SOSC,
  CKSEL_PLL
};

enum enum_PLLCR_PLIDIV {
  // Frequency Multiplication Factor Select
  PLIDIV_1,  // x1
  PLIDIV_2,  // x1/2
  PLIDIV_4   // x1/4
};

enum enum_PLLCR_STC {
  // Frequency Multiplication Factor Select
  STC_8X=7,
  STC_10X=9,
  STC_12X=11,
  STC_16X=15,
  STC_20X=19,
  STC_24X=23,
  STC_25X,
  STC_50X=49
};

enum enum_PLLCR2_PLLEN {
  // PLL Input Frequency Division Ratio Select
  PLLEN_RUN,   // PLL is operating
  PLLEN_STOP   // PLL is stopping
};

enum enum_MOSCCR_MOSTP {
  // Main Clock Oscillator Stop
  MOSTP_RUN,   // MOSC is operating
  MOSTP_STOP  // MOSC is stopping
};

// bit meanings of Module stop bit
enum enum_MSTP {
  MSTP_RUN,
  MSTP_STOP
};


// bit meanings of I/O Port
enum enum_PMR {
  PMR_GPIO,
  PMR_FUNC
};

enum enum_PDR {
  PDR_IN,
  PDR_OUT
};

enum enum_ODR {
  ODR_CMOS,
  ODR_OD
};

enum enum_PCR {
  PCR_OPEN,
  PCR_PULLUP
};

// bit meanings of MPC
enum enum_PWPR_PFSWE {
  // PFS Register Write Enable
  PFSWE_LOCK,   // inhibit
  PFSWE_UNLOCK  // permit
};

enum enum_PWPR_B0WI {
  // PFSWE Bit Write Disable
  B0WI_UNLOCK,  // permit "PFSWE" edit
  B0WI_LOCK     // inhibit "PFSWE" edit
};

enum enum_PmnPFS_ISEL {
  ISEL_OFF,
  ISEL_ON
};

enum enum_PmnPFS_ASEL {
  ASEL_OFF,
  ASEL_ON
};

enum enum_P21PFS_PSEL {
  P21PFS_HIZ,
  P21PFS_MTIOC1B,
  P21PFS_TIOCA3=3,
  P21PFS_TMCI0=5,
  P21PFS_PO1,
  P21PFS_RXD0=10,        //01010b
  P21PFS_SCL1=15,        //01111b
  P21PFS_USB0_EXICEN=19, //10011b
  P21PFS_PIXD5=28        //01111b
};

enum enum_P24PFS_PSEL {
  P24PFS_HIZ,
  P24PFS_MTIOC4A,
  P24PFS_MTCLKA,
  P24PFS_TIOCB4,
  P24PFS_TMRI1=5,
  P24PFS_PO4,
  P24PFS_SCK3=10,        //b01010
  P24PFS_USB0_VBUSEN=19, //b10011
  P24PFS_EDREQ1=24,      //b11000
  P24PFS_PIXCLK=28       //b11100
};

enum enum_P25PFS_PSEL {
  P25PFS_HIZ,
  P25PFS_MTIOC4C,
  P25PFS_MTCLKB,
  P25PFS_TIOCA4,
  P25PFS_PO5=6,          //b00110
  P25PFS_ADTRG0=9,       //b01001
  P25PFS_RXD3,           //b01010
  P25PFS_USB0_DPRP0=19,  //b10011
  P25PFS_EDACK=24,       //b11000
  P25PFS_HSYNC=28        //b11100
};

enum enum_P32PFS_PSEL {
  P32PFS_HIZ,
  P32PFS_MTIOC0C,
  P32PFS_TIOCC0,
  P32PFS_TMO3,
  P32PFS_PO10,
  P32PFS_RTCOUT,
  P32PFS_TXD6,
  P32PFS_TXD0,
  P32PFS_CTX0=16,        //b10000
  P32PFS_USB0_VBUSEN=19, //b10011
  P32PFS_VSYNC=28        //b11100
};

enum enum_PJ3PFS_PSEL {
  PJ3PFS_HIZ,
  PJ3PFS_MTIOC3C,
  PJ3PFS_CSI6=10,//1010b
  PJ3PFS_CSI0=11//1011b
};


// bit meanings of TMR
enum enum_TMR_TCR_CCLR {
  // Counter Clear
  CCLR_NOCLR,     // Clearing is disabled
  CCLR_COMPA,     // Cleared by compare match A
  CCLR_COMPB,     // Cleared by compare match B
  CCLR_EXRST      // Cleared by the external reset input
};

enum enum_TMR_TCR_OVIE {
  // Timer Overflow Interrupt Enable
  OVIE_IRDE,     // Overflow interrupt requests (OVIn) are disabled
  OVIE_IREN      // Overflow interrupt requests (OVIn) are enabled
};

enum enum_TMR_TCR_CMIEx {
  // Compare Match Interrupt Enable A/B
  CMIEx_IRDE,     // Overflow interrupt requests (OVIn) are disabled
  CMIEx_IREN      // Overflow interrupt requests (OVIn) are enabled
};

enum enum_TMR_TCCR_CSS_CKS {
  // { Clock Source Select [1:0]  Clock Select [2:0] }
  CSS_CKS_CLK_PROH,
  CSS_CKS_EXTPOS,
  CSS_CKS_EXTNEG,
  CSS_CKS_EXTEDG,
  CSS_CKS_PCLK=8, // PCLK
  CSS_CKS_PCLK_2,
  CSS_CKS_PCLK_8,
  CSS_CKS_PCLK_32,
  CSS_CKS_PCLK_64,
  CSS_CKS_PCLK_1024,
  CSS_CKS_PCLK_8192,
  CSS_CKS_CLK_PROH_=16,
  CSS_CKS_OTHER=25      // another TCNT overflow signal (TMR0,2)
                        // another TCNT compare match (TMR1,3)
};

enum enum_TMR_TCCR_TMRIS {
  // Timer Reset Detection Condition Select
  TMRIS_ERST_PEDGE,     // Cleared at rising edge of EX reset
  TMRIS_ERST_HIGH       // Cleared when the external reset is high
};

enum enum_TMR_TCSR_OSx {
  // Output Select A/B
  OSx_NOTHING,     // No change when compare match A/B occurs
  OSx_LOW,      // Low is output when compare match A/B occurs
  OSx_HIGH,     // High is output when compare match A/B occurs
  OSx_TGL       // Low is output when compare match A/B occurs
};

enum enum_TMR_TCSR_ADTE {
  // A/D Trigger Enable
  ADTE_DE,     // A/D converter start requests by compare match A are disabled
  ADTE_EN      // A/D converter start requests by compare match A are enabled
};


// bit meanings of MTU2a
enum enum_MTU34_TCR_TPSC {
  TPSC_PCLK,      // PCLK
  TPSC_PCLK_4,    // PCLK/4
  TPSC_PCLK_16,   // PCLK/16
  TPSC_PCLK_64,   // PCLK/64
  TPSC_PCLK_256,  // PCLK/256
  TPSC_PCLK_1024, // PCLK/1024
  TPSC_MTCLKA,    // MTCLKA
  TPSC_MTCLKB     // MTCLKB
};

enum enum_MTU34_TCR_CKEG {
  CKEG_PEDGE,     // Pos edge
  CKEG_NEDGE,     // Neg edge
  CKEG_EDGE       // Both edge
};

enum enum_MTU34_TCR_CCLR {
  CCLR_OFF,      // OFF
  CCLR_TGRA,     // by TGRA
  CCLR_TGRB,     // by TGRB
  CCLR_SYNC,     // by other synced ch
  CCLR_OFF_,     // OFF
  CCLR_TGRC,     // by TGRC
  CCLR_TGRD,     // by TGRD
  CCLR_SYNC_,    // by other synced ch
};

enum enum_MTU34_TMDR_MD {
  TMDR_MD_NORMAL,      // OFF
  TMDR_MD_XXX0,     // prohibit
  TMDR_MD_PWM1,     // PWM1 mode
  TMDR_MD_PWM2,     // PWM2 mode
  TMDR_MD_PHC1,     // Phase counter 1 mode
  TMDR_MD_PHC2,     // Phase counter 2 mode
  TMDR_MD_PHC3,     // Phase counter 3 mode
  TMDR_MD_PHC4,     // Phase counter 4 mode
  TMDR_MD_PWMRST,   // PWM w/ reset mode
  TMDR_MD_XXX1,     // prohibit
  TMDR_MD_XXX2,     // prohibit
  TMDR_MD_XXX3,     // prohibit
  TMDR_MD_CPWM1,     // Transfer in crest
  TMDR_MD_CPWM2,     // Transfer in trough
  TMDR_MD_CPWM3,     // Transfer in both
};

enum enum_MTU34_TMDR_BFx {
  TMDR_BFx_NORM,     // unbuffered func (TGR A&C / B&D / MTU0. E&F)
  TMDR_BFx_BUFF,     // buffered func
};


// bit meanings of TPUa
enum enum_TPU06_TCR_TPSC {
  // Timer Prescaler Select
  TPU06_TPSC_PCLK,       // PCLK
  TPU06_TPSC_PCLK_4,     // PCLK/4
  TPU06_TPSC_PCLK_16,    // PCLK/16
  TPU06_TPSC_PCLK_64,    // PCLK/64
  
  TPU06_TPSC_EXCLK_AE,   // External clock
                         // (TPU0:TCLKA pin / TPU6:TCLKE pin)
  TPU06_TPSC_EXCLK_BF,   // External clock
                         // (TPU0:TCLKB pin / TPU6:TCLKF pin)
  TPU06_TPSC_EXCLK_CG,   // External clock
                         // (TPU0:TCLKC pin / TPU6:TCLKG pin)
  TPU06_TPSC_EXCLK_DH    // External clock
                         // (TPU0:TCLKD pin / TPU6:TCLKH pin)
};

enum enum_TPU39_TCR_TPSC {
  // Timer Prescaler Select
  TPU39_TPSC_PCLK,       // PCLK
  TPU39_TPSC_PCLK_4,     // PCLK/4
  TPU39_TPSC_PCLK_16,    // PCLK/16
  TPU39_TPSC_PCLK_64,    // PCLK/64
  TPU39_TPSC_EXCLK,      // External clock
                         // (TPU3:TCLKA pin / TPU9:TCLKE pin)
  TPU39_TPSC_PCLK_1024,  // PCLK/1024
  TPU39_TPSC_PCLK_256,   // PCLK/256
  TPU39_TPSC_PCLK_4096   // PCLK/4096
};

enum enum_TPU410_TCR_TPSC {
  // Timer Prescaler Select
  TPU410_TPSC_PCLK,       // PCLK
  TPU410_TPSC_PCLK_4,     // PCLK/4
  TPU410_TPSC_PCLK_16,    // PCLK/16
  TPU410_TPSC_PCLK_64,    // PCLK/64
  TPU410_TPSC_EXCLK_AE,   // External clock
                          // (TPU0:TCLKA pin / TPU6:TCLKE pin)
  TPU410_TPSC_EXCLK_CG,   // External clock
                          // (TPU0:TCLKC pin / TPU6:TCLKG pin)
  TPU410_TPSC_PCLK_1024,  // PCLK/1024
  TPU410_TPSC_OVUV        // Over / under flow flag
                          // (TPU4:TPU5 OV/UV flag / TPU10:TPU11 OV/UV flag
};



enum enum_TPU_TCR_CKEG {
  // Input Clock Edge Select
  TPU_CKEG_EDGE_IN_EP,  // Int: falling / Ext: rising
  TPU_CKEG_EDGE_IP_EN,  // Int: rising  / Ext: falling
  TPU_CKEG_EDGE         // Both edge
};

enum enum_TPU_TCR_CCLR {
  // Counter Clear Source Select
  TPU_CCLR_OFF,      // OFF
  TPU_CCLR_TGRA,     // by TGRA
  TPU_CCLR_TGRB,     // by TGRB
  TPU_CCLR_SYNC,     // by other synced ch
  TPU_CCLR_OFF_,     // OFF
  TPU_CCLR_TGRC,     // by TGRC (only TPU0369)
  TPU_CCLR_TGRD,     // by TGRD (only TPU0369)
  TPU_CCLR_SYNC_     // by other synced ch (TPU0369)
};

enum enum_TPU_TMDR_MD {
  // Mode Select
  TPU_MD_NORM,     // Normal operation
  TPU_MD_XXX0,     // prohibit
  TPU_MD_PWM1,     // PWM1 mode
  TPU_MD_PWM2,     // PWM2 mode
  TPU_MD_PHC1,     // Phase counter 1 mode
  TPU_MD_PHC2,     // Phase counter 2 mode
  TPU_MD_PHC3,     // Phase counter 3 mode
  TPU_MD_PHC4     // Phase counter 4 mode
};

enum enum_TPU_TMDR_ICSELB {
  // TGRB Input Capture Input Select
  TPU_ICSELB_TIOCBn,    // TIOCBn
  TPU_ICSELB_TIOCAn     // TIOCAn
};

enum enum_TPU_TMDR_ICSELD {
  // TGRB Input Capture Input Select
  TPU_ICSELD_TIOCDn,    // TIOCDn pin
  TPU_ICSELD_TIOCCn     // TIOCCn pin
};

enum enum_TPU39_TIOR_IOX {
  // TGRA Control
  TPU_IOX_DE,      // disabled
  TPU_IOX_OLCL,    // Output: init Low / cmatch Low
  TPU_IOX_OLCH,    // Output: init Low / cmatch High
  TPU_IOX_OLCT,    // Output: init Low / cmatch toggle
  TPU_IOX_DE_,     // disabled
  TPU_IOX_OHCL,    // Output: init High / cmatch Low
  TPU_IOX_OHCH,    // Output: init High / cmatch High
  TPU_IOX_OHCT,    // Output: init High / cmatch toggle
  TPU_IOX_IPEDGE,  // Input(IOA): TIOCAn pin capture at pos edge
                   // Input(IOB): TIOCBn / TIOCAn (by ISELB) pin capture at pos edge
                   // Input(IOC): TIOCCn pin capture at pos edge
                   // Input(IOD): TIOCCn / TIOCDn (by ISELD) pin capture at pos edge
  TPU_IOX_INEDGE,  // Input: TIOCAn pin capture at neg edge
  TPU_IOX_IEDGE,   // Input: TIOCAn pin capture at both edge
  TPU_IOX_IEDGE_,  // same as above
  TPU_IOX_TCNT     // Input: TPU0 -> TPU1.TCNT / TPU6 -> TPU7.TCNT
                   //        TPU1 -> TPU0.TCNT / TPU7 -> TPU6.TCNT
                   //        TPU2 -> None / TPU8 -> None
                   //        TPU3 -> TPU4.TCNT / TPU9 -> TPU10.TCNT
                   //        TPU4 -> TPU3.TCNT / TPU10 -> TPU9.TCNT
                   //        TPU5 -> None / TPU11 -> None
};

enum enum_TPU_TIER_TGIEX {
  TGIEX_DE,   // disable
  TGIEX_EN    // enable
};

enum enum_TPU_TIER_TTGE {
  // A/D Conversion Start Request Enable
  TTGE_DE,    // disable
  TTGE_EN     // enable
};

enum enum_TPU_NFCR_NFnEN {
  // Noise Filter Enable m
  NFnEN_DE,    // disable
  NFnEN_EN     // enable
};

enum enum_TPU_NFCR_NFCS {
  // Noise Filter Clock Select
  NFCS_PCLK,      // NF CLK -> PCLK/1
  NFCS_PCLK_8,    // NF CLK -> PCLK/8
  NFCS_PCLK_32,   // NF CLK -> PCLK/32
  NFCS_CCLK       // Clock source that drives counting
};


// bit meanings of TPUA to B
enum enum_TPU_TSTR_CSTn {
  // Counter Start n
  CSTn_STOP,    // count stop
  CSTn_RUN      // count start
};

enum enum_TPU_TSTR_SYNCn {
  // Timer Synchronization n
  SYNCn_IND,      // operates independently
  SYNCn_SYNC      // operates w/ synchronization
};


// bit meanings of S12ADa
enum enum_S12AD_ADCSR_EXTRG {
  EXTRG_SYNC,   // synchronous (MTU, TPU, TMR)
  EXTRG_ASYN    // asynchronous
};

enum enum_S12AD_ADCSR_TRGE {
  TRGE_DE,   // inhibit
  TRGE_EN    // permit
};

enum enum_S12AD_ADCSR_CKS {
  CKS_PCLK_8,   // PCLK/8
  CKS_PCLK_4,   // PCLK/4
  CKS_PCLK_2,   // PCLK/2
  CKS_PCLK      // PCLK
};

enum enum_S12AD_ADCSR_ADIE {
  ADIE_DE,   // inhibit
  ADIE_EN    // permit
};

enum enum_S12AD_ADCSR_ADCS {
  ADCS_SINGLE,  // single-scan mode
  ADCS_CONT     // continuous scan mode
};

enum enum_S12AD_ADCSR_ADST {
  ADST_STOP,    // Stop conversion
  ADST_START    // Start conversion
};

enum enum_S12AD_ADANS {
  ADANS_DE,    // disselect to conversion
  ADANS_EN     // select to conversion
};

enum enum_S12AD_ADADS {
  ADADS_DE,    // disselect to operate addition mode
  ADADS_EN     // select to operate addition mode
};

enum enum_S12AD_ADADC_ADC { // ADdition Count select
  ADADC_X1,    // 1 time conversion
  ADADC_X2,    // 2 time conversion
  ADADC_X3,    // 3 time conversion
  ADADC_X4     // 4 time conversion
};

enum enum_S12AD_ADCER_ACE { // Automatic Clearing Enable
  ACE_DE,      // disable automatic clearing
  ACE_EN       // enable automatic clearing
};

enum enum_S12AD_ADCER_ADRFMT { // AD Register ForMaT
  ADRFMT_FLRIGHT,      // flush-right format
  ADRFMT_FLLEFT        // flush-left format
};

enum enum_S12AD_ADSTRGR_ADSTRS {
  // AD Start TRiGer Register
  ADSTRS_ASYN,       // flush-right format
  ADSTRS_TRG0AN_0,      // MTU0.TGRA & MTU0.TCNT
  ADSTRS_TRG0BN_0,      // MTU0.TGRB & MTU0.TCNT
  ADSTRS_TRGAN_0,       // MTUn.TGRA & MTUn.TCNT
  ADSTRS_TRGAN_1,       // TPUn.TGRA
  ADSTRS_TRG0EN_0,      // MTU0.TGRE & MTU0.TCNT
  ADSTRS_TRG0FN_0,      // MTU0.TGRF & MTU0.TCNT
  ADSTRS_TRG04ABN_0,    // MTU4.TADCORA & MTU4.TCNT
                        // MTU4.TADCORB & MTU4.TCNT
  ADSTRS_TRG04ABN_1,    // TPU0.TGRA
  ADSTRS_TMTRG0AN_0,    // TMR0.TCORA & TMR0.TCNT
  ADSTRS_TMTRG0AN_1     // TMR2.TCORA & TMR2.TCNT
};

enum enum_S12AD_ADEXICR_TSSAD {
  // Temperature Sensor Output A/D Converted Value Addition Mode Select
  TSSAD_DE,        // addition mode
  TSSAD_ADD        // disable
};

enum enum_S12AD_ADEXICR_OCSAD {
  // A/D Internal Reference Voltage A/D Converted Value Addition Mode Select
  OCSAD_DE,        // addition mode
  OCSAD_ADD        // disable
};

enum enum_S12AD_ADEXICR_TSS {
  // Temperature Sensor Output A/D Conversion Select
  TSS_DE,         // addition mode
  TSS_EN          // disable
};

enum enum_S12AD_ADEXICR_OCS {
  // A/D Internal Reference Voltage A/D Conversion Select
  OCS_DE,         // addition mode
  OCS_EN          // disable
};

#endif
