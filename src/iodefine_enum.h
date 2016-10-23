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

enum enum_P12PFS_PSEL {
  P12PFS_HIZ,
  P12PFS_MTIC5U,
  P12PFS_TMCI1=5,        //00101b
  P12PFS_RXD2=10,        //01010b
  P12PFS_SCL0=15         //01111b
};

enum enum_P13PFS_PSEL {
  P13PFS_HIZ,
  P13PFS_MTIOC0B,
  P13PFS_TIOCA5=3,       //00011b
  P13PFS_TMO3=5,         //00101b
  P13PFS_PO13,
  P13PFS_ADTRG=9,        //01001b
  P13PFS_TXD2,           //01010b
  P13PFS_SDA0=15         //01111b
};

enum enum_P14PFS_PSEL {
  P14PFS_HIZ,
  P14PFS_MTIOC3A,
  P14PFS_MTCLKA,         //00010b
  P14PFS_TIOCB5,         //00011b
  P14PFS_TCLKA,          //00100b
  P14PFS_TMRI2,          //00101b
  P14PFS_PO15,           //00110b
  P14PFS_CTS=11,         //01011b
  P14PFS_CTX1=16,        //10000b
  P14PFS_USB0_DPUPE,     //10001b
  P14PFS_USB0_OVRCURA    //10010b
};

enum enum_P15PFS_PSEL {
  P15PFS_HIZ,
  P15PFS_MTIOC0B,        //00001b
  P15PFS_MTCLKB,         //00010b
  P15PFS_TIOCB2,         //00011b
  P15PFS_TCLKB,          //00100b
  P15PFS_TMCI2,          //00101b
  P15PFS_PO13,           //00110b
  P15PFS_RXD1=10,        //01010b
  P15PFS_SCK3,           //01011b
  P14PFS_CRX1DS=16,      //10000b
  P15PFS_USB1_DPUPE,     //10001b
  P15PFS_PIXD0=28        //11100b
};

enum enum_P16PFS_PSEL {
  P16PFS_HIZ,            //00000b
  P16PFS_MTIOC3C,        //00001b
  P16PFS_MTIOC3D,        //00010b
  P16PFS_TIOCB1,         //00011b
  P16PFS_TCLKC,          //00100b
  P16PFS_TMO2,           //00101b
  P16PFS_PO14,           //00110b
  P16PFS_RTCOUT,         //00111b
  P16PFS_ADTRG0=9,       //01001b
  P16PFS_TXD1,           //01010b
  P16PFS_RXD3,           //01011b
  P16PFS_MOSIA=13,       //01101b
  P16PFS_SCL2_DS=15,     //01111b
  P16PFS_IERXD,          //10000b
  P16PFS_USB0_VBUS,      //10001b
  P16PFS_USB0_VBUSEN,    //10010b
  P16PFS_USB0_OVRCURB    //10011b
};

enum enum_P17PFS_PSEL {
  P17PFS_HIZ,            //00000b
  P17PFS_MTIOC3A,        //00001b
  P17PFS_MTIOC3B,        //00010b
  P17PFS_TIOB0,          //00011b
  P17PFS_TCLKD,          //00100b
  P17PFS_TMO1,           //00101b
  P17PFS_PO15,           //00110b
  P17PFS_POE8,           //00111b
  P17PFS_ADTRG=9,        //01001b
  P17PFS_SCK1,           //01010b
  P17PFS_TXD3,           //01011b
  P17PFS_MISOA=13,       //01101b
  P17PFS_SDA2_DS=15,     //01111b
  P17PFS_IETXD,          //10000b
  P17PFS_USB1_VBUS,      //10001b
  P17PFS_PIXD3=28        //11100b
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

enum enum_P26PFS_PSEL {
  P26PFS_HIZ,
  P26PFS_MTIOC2A,
  P26PFS_TMO1=5,         //b00101
  P26PFS_PO6=6,          //b00110
  P26PFS_TXD1=10,        //b01010
  P26PFS_CTS31,          //b01011
  P26PFS_MOSIB=13        //b01101
};

enum enum_P30PFS_PSEL {
  P30PFS_HIZ,
  P30PFS_MTIOC4B,
  P30PFS_TMRI3=5,        //b00101
  P30PFS_PO8,            //b00110
  P30PFS_POE8,           //b00111
  P30PFS_RXD1=10,        //b01010
  P30PFS_MISOB=13,       //b01101
  P30PFS_USB0_DRPD=19,   //b10011
};

enum enum_P32PFS_PSEL {
  P32PFS_HIZ,
  P32PFS_MTIOC0C,
  P32PFS_TIOCC0=3,
  P32PFS_TMO3=5,
  P32PFS_PO10,
  P32PFS_RTCOUT,
  P32PFS_TXD6=10,
  P32PFS_TXD0,
  P32PFS_CTX0=16,        //b10000
  P32PFS_USB0_VBUSEN=19, //b10011
  P32PFS_VSYNC=28        //b11100
};

enum enum_PC2PFS_PSEL {
  PC2PFS_HIZ,
  PC2PFS_MTIOC4B,
  PC2PFS_TCLKA=3,        //b00011
  PC2PFS_PO21=6,         //b00110
  PC2PFS_RXD5=10,        //b01010
  PC2PFS_SSLA3=13,       //b01101
  PC2PFS_IERXD=16,       //b10000
  PC2PFS_ET_RX_DV        //b10001
};

enum enum_PC3PFS_PSEL {
  PC3PFS_HIZ,
  PC3PFS_MTIOC4D,
  PC3PFS_TCLKB=3,        //b00011
  PC3PFS_PO24=6,         //b00110
  PC3PFS_TXD5=10,        //b01010
  PC3PFS_IETXD=16,       //b10000
  PC3PFS_ET_TX_ER        //b10001
};

enum enum_PJ3PFS_PSEL {
  PJ3PFS_HIZ,
  PJ3PFS_MTIOC3C,
  PJ3PFS_CSI6=10,//1010b
  PJ3PFS_CSI0=11//1011b
};

enum enum_PE1PFS_PSEL {
  PE1PFS_HIZ,
  PE1PFS_MTIOC4C,      //0001b
  PE1PFS_TIOCD9=3,     //0011b
  PE1PFS_PO18=6,       //0110b
  PE1PFS_TXD12=12,     //1100b
  PE1PFS_SSLB2,        //1101b
  PE1PFS_RSPCKB        //1110b
};

enum enum_PE2PFS_PSEL {
  PE2PFS_HIZ,
  PE2PFS_MTIOC4A,      //0001b
  PE2PFS_TIOCA9=3,     //0011b
  PE2PFS_PO23=6,       //0110b
  PE2PFS_RXD12=12,     //1100b
  PE2PFS_SSLB3,        //1101b
  PE2PFS_MOSIB         //1110b
};


// bit meanings of TMR
enum enum_CMT_CMCR_CKS {
  // Clock Select
  CMT_CKS_PCLK_8,      // PCLK/8
  CMT_CKS_PCLK_32,     // PCLK/32
  CMT_CKS_PCLK_128,    // PCLK/128
  CMT_CKS_PCLK_512     // PCLK/512
};

enum enum_CMT_CMCR_CMIE {
  // Compare Match Interrupt Enable
  CMIE_DE,      // Interruput disable
  CMIE_EN       // Interruput disable
};

enum enum_CMT_CMSTRn {
  // Counter Start n
  CMSTRn_STOP,    // count stop
  CMSTRn_RUN      // count start
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
enum enum_MTU0_TCR_MTU0_TPSC {
  MTU0_TPSC_PCLK,      // PCLK
  MTU0_TPSC_PCLK_4,    // PCLK/4
  MTU0_TPSC_PCLK_16,   // PCLK/16
  MTU0_TPSC_PCLK_64,   // PCLK/64
  MTU0_TPSC_MTCLKA,    // MTCLKA
  MTU0_TPSC_MTCLKB,    // MTCLKB
  MTU0_TPSC_MTCLKC,    // MTCLKC
  MTU0_TPSC_MTCLKD     // MTCLKD
};

enum enum_MTU34_TCR_MTU34_TPSC {
  MTU34_TPSC_PCLK,      // PCLK
  MTU34_TPSC_PCLK_4,    // PCLK/4
  MTU34_TPSC_PCLK_16,   // PCLK/16
  MTU34_TPSC_PCLK_64,   // PCLK/64
  MTU34_TPSC_PCLK_256,  // PCLK/256
  MTU34_TPSC_PCLK_1024, // PCLK/1024
  MTU34_TPSC_MTCLKA,    // MTCLKA
  MTU34_TPSC_MTCLKB     // MTCLKB
};

enum enum_MTU_TCR_CKEG {
  CKEG_PEDGE,     // Pos edge
  CKEG_NEDGE,     // Neg edge
  CKEG_EDGE       // Both edge
};

enum enum_MTU_TCR_CCLR {
  CCLR_OFF,      // OFF
  CCLR_TGRA,     // by TGRA
  CCLR_TGRB,     // by TGRB
  CCLR_SYNC,     // by other synced ch
  CCLR_OFF_,     // OFF      (for MTU0,3,4)
  CCLR_TGRC,     // by TGRC  (for MTU0,3,4)
  CCLR_TGRD,     // by TGRD  (for MTU0,3,4)
  CCLR_SYNC_,    // by other synced ch  (for MTU0,3,4)
};

enum enum_MTU_TMDR_MD {
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

enum enum_MTU_TMDR_BFx {
  TMDR_BFx_NORM,     // unbuffered func (TGR A&C / B&D / MTU0. E&F)
  TMDR_BFx_BUFF,     // buffered func
};

enum enum_MTU_TIOR_IOX {
  // TGR[A:D] Control
  MTU_IOX_DE,      // disabled
  MTU_IOX_OLCL,    // Output: init Low / cmatch Low
  MTU_IOX_OLCH,    // Output: init Low / cmatch High
  MTU_IOX_OLCT,    // Output: init Low / cmatch toggle
  MTU_IOX_DE_,     // disabled
  MTU_IOX_OHCL,    // Output: init High / cmatch Low
  MTU_IOX_OHCH,    // Output: init High / cmatch High
  MTU_IOX_OHCT,    // Output: init High / cmatch toggle
  MTU_IOX_IPEDGE,  // Input(IOA): TIOCAn pin capture at pos edge
                   // Input(IOB): TIOCBn / TIOCAn (by ISELB) pin capture at pos edge
                   // Input(IOC): TIOCCn pin capture at pos edge
                   // Input(IOD): TIOCCn / TIOCDn (by ISELD) pin capture at pos edge
  MTU_IOX_INEDGE,  // Input: TIOCAn pin capture at neg edge
  MTU_IOX_IEDGE,   // Input: TIOCAn pin capture at both edge
  MTU_IOX_IEDGE_,  // same as above
  MTU_IOX_TCNT     // Input: MTU0 -> MTU1.TCNT
                   //        MTU1 -> MTU0.TGRx
                   //        MTU2 -> None
                   //        MTU3 -> None
                   //        MTU4 -> None
                   //        MTU5 -> None
};

enum enum_MTU_TRWER_RWE {
  // Read/Write Enable
  TRWER_RWE_DE,   // disabled
  TRWER_RWE_EN    // enabled
};

enum enum_MTU_TOER_OEny {
  // Master Enable MTIOC[3,4][B,C]
  OEny_DE,        // disabled
  OEny_EN         // enabled
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

enum enum_TPU511_TCR_TPSC {
  // Timer Prescaler Select
  TPU511_TPSC_PCLK,       // PCLK
  TPU511_TPSC_PCLK_4,     // PCLK/4
  TPU511_TPSC_PCLK_16,    // PCLK/16
  TPU511_TPSC_PCLK_64,    // PCLK/64
  TPU511_TPSC_EXCLK_AE,   // External clock
                          // (TPU5:TCLKA pin / TPU11:TCLKE pin)
  TPU511_TPSC_EXCLK_CG,   // External clock
                          // (TPU0:TCLKC pin / TPU6:TCLKG pin)
  TPU511_TPSC_PCLK_256,   // PCLK/256
  TPU511_TPSC_EXCLK_DH    // External clock
                          // (TPU0:TCLKD pin / TPU6:TCLKH pin)
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

enum enum_TIOR_IOX {
  // TGRA Control
  IOX_DE,      // disabled
  IOX_OLCL,    // Output: init Low / cmatch Low
  IOX_OLCH,    // Output: init Low / cmatch High
  IOX_OLCT,    // Output: init Low / cmatch toggle
  IOX_DE_,     // disabled
  IOX_OHCL,    // Output: init High / cmatch Low
  IOX_OHCH,    // Output: init High / cmatch High
  IOX_OHCT,    // Output: init High / cmatch toggle
  IOX_IPEDGE,  // Input(IOA): TIOCAn pin capture at pos edge
                   // Input(IOB): TIOCBn / TIOCAn (by ISELB) pin capture at pos edge
                   // Input(IOC): TIOCCn pin capture at pos edge
                   // Input(IOD): TIOCCn / TIOCDn (by ISELD) pin capture at pos edge
  IOX_INEDGE,  // Input: TIOCAn pin capture at neg edge
  IOX_IEDGE,   // Input: TIOCAn pin capture at both edge
  IOX_IEDGE_,  // same as above
  IOX_TCNT     // Input: TPU0 -> TPU1.TCNT / TPU6 -> TPU7.TCNT
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

enum enum_SCI_SMR_CKS {
  // Clock Select
  SCI_CKS_PCLK,     // PCLK
  SCI_CKS_PCLK_4,   // PCLK/4
  SCI_CKS_PCLK_16,  // PCLK/16
  SCI_CKS_PCLK_64   // PCLK/64
};

enum enum_SCI_SMR_MP {
  // Multi-Processor Mode
  SCI_MP_DE,    // disabled
  SCI_MP_EN     // enabled
};

enum enum_SCI_SMR_STOP {
  // Stop Bit Length
  SCI_STOP_1,  // 1 stop bit
  SCI_STOP_2   // 2 stop bit
};

enum enum_SCI_SMR_PM {
  // Parity Mode
  SCI_PM_EVEN,  // Select even parity
  SCI_PM_ODD    // Select odd parity
};

enum enum_SCI_SMR_PE {
  // Parity Enable
  SCI_PE_DE,  // Parity bit addition is not performed
  SCI_PE_EN   // The parity bit is added(tx)/checked(rx)
};

enum enum_SCI_SMR_CHR {
  // Character Length
  SCI_CHR_8,  // Selects 8 bits as the data length
  SCI_CHR_7   // Selects 7 bits as the data length
};

enum enum_SCI_SMR_CM {
  // Communications Mode
  SCI_CM_ASYNC,  // Asynchronous mode
  SCI_CM_SYNC    // Clock synchronous mode
};

enum enum_SCI_SCR_CKE {
  // Clock Enable
  SCI_CKE_OCHPIO,  // On-chip baud rate generator (SCK as I/O)
  SCI_CKE_OCHPCK,  // On-chip baud rate generator (SCK as SCK)
  SCI_CKE_EXTCLK   // External clock or TMR
};

enum enum_SCI_SCR_TEIE {
  // Transmit End Interrupt Enable
  SCI_TEIE_DE,  // A TEI interrupt request is disabled
  SCI_TEIE_EN  // A TEI interrupt request is enabled
};

enum enum_SCI_SCR_MPIE {
  // Multi-Processor Interrupt Enable
  // (Valid in asynchronous mode when SMR.MP = 1)
  SCI_MPIE_NORM,  // Normal reception
  SCI_MPIE_MP     // 
};

enum enum_SCI_SCR_RE {
  // Receive Enable
  SCI_RE_DE, // Serial reception is disabled
  SCI_RE_EN  // Serial reception is enabled
};

enum enum_SCI_SCR_TE {
  // Transmit Enable
  SCI_TE_DE, // Serial transmission is disabled
  SCI_TE_EN  // Serial transmission is enabled
};

enum enum_SCI_SCR_RIE {
  // Receive Interrupt Enable
  SCI_RIE_DE, // RXI and ERI interrupt requests are disabled
  SCI_RIE_EN  // RXI and ERI interrupt requests are enabled
};

enum enum_SCI_SCR_TIE {
  // Receive Interrupt Enable
  SCI_TIE_DE, // A TXI interrupt request is disabled
  SCI_TIE_EN  // A TXI interrupt request is enabled
};

enum enum_SCI_SSR_MPBT {
  // Multi-Processor Bit Transfer
  // Sets the multi-processor bit for adding to the transmission frame
  SCI_MPBT_DAT, // Data transmission cycles
  SCI_MPBT_ID   // ID transmission cycles
};

enum enum_SCI_SSR_MPB {
  // Multi-Processor
  // Value of the multi-processor bit in the reception frame
  SCI_MPB_DAT, // Data transmission cycles
  SCI_MPB_ID   // ID transmission cycles
};

enum enum_SCI_SSR_TEND {
  // Transmit End Flag
  SCI_TEND_BUSY, // A character is being transmitted.
  SCI_TEND_DONE   // Character transfer has been completed.
};

enum enum_SCI_SSR_PER {
  // Parity Error Flag
  SCI_PER_OK,   // No parity error occurred
  SCI_PER_ERR   // A parity error has occurred
};

enum enum_SCI_SSR_FER {
  // Framing Error Flag
  SCI_FER_OK,   // No framing error occurred
  SCI_FER_ERR   // A framing error has occurred
};

enum enum_SCI_SSR_ORER {
  // Overrun Error Flag
  SCI_ORER_OK,   // No overrun error occurred
  SCI_ORER_ERR   // A overrun error has occurred
};

enum enum_SCI_SCMR_SMIF {
  // Smart Card Interface Mode Select
  SCI_SMIF_SCIF,    // Serial communications interface mode
  SCI_SMIF_SMIF     // Smart card interface mode
};

enum enum_SCI_SCMR_SINV {
  // Transmitted/Received Data Invert
  SCI_SINV_NORM,    // TDR contents are transmitted as they are.
  SCI_SINV_INV      // TDR contents are inverted before being transmitted.
                     // Receive data is stored in inverted form in RDR.
};

enum enum_SCI_SCMR_SDIR {
  // Transmitted/Received Data Transfer Direction
  SCI_SDIR_LSB,    // Transfer with LSB-first
  SCI_SMIF_MSB     // Transfer with MSB-first
};

enum enum_SCI_SEMR_ACS0 {
  // Asynchronous Mode Clock Source Select
  SCI_ACS0_EXCLK,   // External clock input
  SCI_ACS0_TMRCLK   // TMR clock input (valid only for SCI5, SCI6, and SCI12)
                     // SCI5 Unit 0 TMO0, TMO1
                     // SCI6 Unit 1 TMO2, TMO3
                     // SCI12 Unit 0 TMO0, TMO1
};

enum enum_SCI_SEMR_ABCS {
  // Asynchronous Mode Base Clock Select
  SCI_ABCS_16CLK,   // Selects 16 base clock cycles for 1-bit period
  SCI_ABCS_8CLK     // Selects 8 base clock cycles for 1-bit period
};

enum enum_SCI_SEMR_NFEN {
  // Digital Noise Filter Function Enable
  SCI_NFEN_DE,   // Noise cancellation function for the RXDn input signal is disabled.
  SCI_NFEN_EN    // Noise cancellation function for the RXDn input signal is enabled.
};

enum enum_SCI_SIMR1_IICM {
  // Simple I2C Mode Select
  SCI_IICM_SCIF,   // Serial interface mode
  SCI_IICM_I2C,    // Simple I2C mode
  SCI_IICM_SMIF    // Smart card interface mode
};

enum enum_SCI_SPMR_CKPOL {
  // Clock Polarity Select
  SCI_CKPOL_NORM,   // Clock polarity is not inverted.
  SCI_CKPOL_INV     // Clock polarity is inverted.
};

enum enum_SCI_SPMR_CKPH {
  // Clock Phase Select
  SCI_CKPH_NORM,   // Clock is not delayed.
  SCI_CKPH_DLY     // Clock is delayed.
};

enum enum_SCI_SPMR_CTSE {
  // CTS Enable
  SCI_CTSE_DE,    // CTS pin function is disabled
  SCI_CTSE_EN     // CTS pin function is enabled
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
