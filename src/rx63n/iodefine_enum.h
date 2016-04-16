#ifndef __IODEFINE_ENUM_H__
#define __IODEFINE_ENUM_H__

#include "iodefine.h"

#define ADDRESS_HOCOCR2 (( unsigned char *)0x00080037/*HOCOCR2*/)
#define ADDRESS_PMCTLTSTR (( unsigned char *) 0x0008c2e0/*PMCLTSTR*/)

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
  PFSWE_LOCK,   // inhibit
  PFSWE_UNLOCK  // permit
};

enum enum_PWPR_B0WI {
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
  PJ3PFS_HIZ,
  PJ3PFS_MTIOC1B,
  PJ3PFS_TIOCA3=3,
  PJ3PFS_TMCI0,
  PJ3PFS_P01,
  PJ3PFS_RXD0=10,        //01010b
  PJ3PFS_SCL1=15,        //01111b
  PJ3PFS_USB0_EXICEN=19, //10011b
  PJ3PFS_PIXD5=28        //01111b
};

enum enum_PJ3PFS_PSEL {
  PJ3PFS_HIZ,
  PJ3PFS_MTIOC3C,
  PJ3PFS_CSI6=10,//1010b
  PJ3PFS_CSI0=11//1011b
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
  TMDR_MD_CPWM1,     // Transfer in mountain
  TMDR_MD_CPWM2,     // Transfer in valley
  TMDR_MD_CPWM3,     // Transfer in both
};

enum enum_MTU34_TMDR_BFx {
  TMDR_BFx_NORM,     // unbuffered func (TGR A&C / B&D / MTU0. E&F)
  TMDR_BFx_BUFF,     // buffered func
};

enum enum_CKSEL {
  CKSEL_LOCO,
  CKSEL_HOCO,
  CKSEL_MOSC,
  CKSEL_SOSC,
  CKSEL_PLL
};

#endif
