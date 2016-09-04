
#include "sysutil_RX63N.h"
#include "typedefine.h"
#include "sysutil_RX63N.h"

// -------------------------------------------------------
// ------------------------ Functions ( system Utilities )
//
//  disp data on LED ( mounted on Eval Brd. )
//
//
//  lock / unlock Sytem core reg edit
//
void SysCoreLock( void ){
  SYSTEM.PRCR.BIT.PRC0  = PRCn_LOCK;   // clk gen: SCKCR / SCKCR2 / SCKCR3 / PLLCR /
                                       //          PLLCR2 / BCKCR / MOSCCR / SOSCCR /
                                       //          LOCOCR / ILOCOCR / HOCOCR / OSTDC(S)R
  SYSTEM.PRCR.BIT.PRC1  = PRCn_LOCK;   // operating modes: SYSCR0 / SYSCR1
                                       // low pow modes:   SBYCR / MSTPCRA / MSTPCRB /
                                       //                  MSTPCRC / MSTPCRD / OPCCR /
                                       //                  RSTCKCR / MOSCWTCR / SOSCWTCR /
                                       //                  PLLWTCR / DPSBYCR / DPSIER0-3 /
                                       //                  DPSIFR0-3 / DPSIEGR0-3
                                       // clk gen: MOFCR / HOCOPCR
                                       // reset:   SWRR
  SYSTEM.PRCR.BIT.PRC3  = PRCn_LOCK;   // lvd:     LVCMPCR / LVDLVLR / LVD1CR0 / LVD1CR1 /
                                       //          LVD1SR / LVD2CR0 / LVD2CR1 / LVD2SR
  SYSTEM.PRCR.BIT.PRKEY = PRKEY_LOCK;
  SYSTEM.PRCR.WORD = 0x0000;
}

void SysCoreUnlock( void ){
  //SYSTEM.PRCR.BIT.PRKEY = PRKEY_UNLOCK;
  //SYSTEM.PRCR.BIT.PRC0  = PRCn_UNLOCK;
  //SYSTEM.PRCR.BIT.PRC1  = PRCn_UNLOCK;
  //SYSTEM.PRCR.BIT.PRC3  = PRCn_UNLOCK;
  SYSTEM.PRCR.WORD = 0xA503;
}

//
//  lock / unlock MPC reg edit
//
void SysMPCUnlock( void ){
  MPC.PWPR.BIT.B0WI  = B0WI_UNLOCK;
  MPC.PWPR.BIT.PFSWE = PFSWE_UNLOCK;
}

void SysMPCLock( void ){
  MPC.PWPR.BIT.PFSWE = PFSWE_LOCK;
  MPC.PWPR.BIT.B0WI  = B0WI_LOCK;
}

void SysMTU34Unlock( void ){
  if( MTU.TRWER.BIT.RWE == TRWER_RWE_DE ){
    MTU.TRWER.BIT.RWE = TRWER_RWE_EN;
  }
}

// Clock source & divider settings
void SysClkInit( void ){
  // select CLK src & dividing
  SYSTEM.SCKCR.BIT.PCKB   = XCK_DIV_4;    // XTAL:12MHz 16x/4 -> 48MHz
  SYSTEM.SCKCR.BIT.PCKA   = XCK_DIV_2;    // XTAL:12MHz 16x/2 -> 96MHz
  SYSTEM.SCKCR.BIT.BCK    = XCK_DIV_8;    // XTAL:12MHz 16x/8 -> 24MHz
  SYSTEM.SCKCR.BIT.ICK    = XCK_DIV_2;    // XTAL:12MHz 16x/2 -> 96MHz
  SYSTEM.SCKCR.BIT.FCK    = XCK_DIV_4;    // XTAL:12MHz 16x/4 -> 48MHz
  SYSTEM.SCKCR.BIT.PSTOP0 = PSTOP0_EN;    // PSTOP0_[EN / DE] SDCLK  
  SYSTEM.SCKCR.BIT.PSTOP1 = PSTOP0_DE;    // PSTOP1_[EN / DE] BusCLK
  SYSTEM.SCKCR2.BIT.IEBCK = IECK_DIV_8;   // XTAL:12MHz 16x/8 -> 24MHz
  SYSTEM.SCKCR2.BIT.UCK   = UCK_DIV_4;    // XTAL:12MHz 16x/4 -> 48MHz
  
  SYSTEM.MOSCCR.BIT.MOSTP = MOSTP_RUN;   // MOSC Running
  SYSTEM.SCKCR3.BIT.CKSEL = CKSEL_MOSC;  // CKSEL_[LOCO / HOCO / MOSC / SOSC / PLL]
  steps(10);                             // ensure that PLL is stable
  SYSTEM.PLLCR2.BIT.PLLEN = PLLEN_STOP;  // PLLEN
  SYSTEM.PLLCR.BIT.PLIDIV = PLIDIV_1;    // PLLDIV_[1 / 2 / 4]
                                         // XTAL:12MHz, 16x/1 -> 192MHz
  SYSTEM.PLLCR.BIT.STC    = STC_16X;     // STC_[8 / 10 / 12 / 16 / 20 / 24 / 25 / 50]X
                                         // XTAL:12MHz, 16x -> 192MHz (104M - 200MHz)
  SYSTEM.PLLCR2.BIT.PLLEN = PLLEN_RUN;   // PLLEN
  steps(10);                             // ensure > 5 cycles run to be stable a PLL
  SYSTEM.SCKCR3.BIT.CKSEL = CKSEL_PLL;   // CKSEL_[LOCO / HOCO / MOSC / SOSC / PLL]

  //steps(10);                             // ensure > 5 cycles run to be stable a PLL
  //SYSTEM.SCKCR3.BIT.CKSEL = CKSEL_MOSC;  // CKSEL_[LOCO / HOCO / MOSC / SOSC / PLL]

}


// select using module
void SysMdlStopInit( void ){
  //  MSTP(TMR0) = MSTP_STOP;  // MSTP_RUN / MSTP_STOP
  //  MSTP(TMR2) = MSTP_STOP;
  // for Human - Machine interface clock
  MSTP(CMT0)  = MSTP_RUN;
  MSTP(CMT1)  = MSTP_RUN;
  // for Rotary encoder
  MSTP(MTU1)  = MSTP_RUN;
  // for gradational LED
  //MSTP(TPU0)  = MSTP_RUN;
  //MSTP(TPU4)  = MSTP_RUN;
  MSTP(MTU0)  = MSTP_RUN;
  MSTP(MTU4)  = MSTP_RUN;
  // for Sounder
  MSTP(TPU5)  = MSTP_RUN;
  // for PPM
  MSTP(TPU3)  = MSTP_RUN;
  // for stick gimbal input
  MSTP(S12AD) = MSTP_RUN;
  // for MSP via UART
  MSTP(SCI5)  = MSTP_RUN;
  // for telemetry via UART 
  MSTP(SCI12) = MSTP_RUN;
}



/* end of sysutil_RX63N.c */
