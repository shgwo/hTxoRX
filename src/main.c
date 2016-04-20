// -------------------------------------------------------
// ------------------------------------------------ Notice
//  This program is distributed to the world under
//  LICENSE of the GPL.
//
// -------------------------------------------------------
// ------------------------------------------------- Info.
//
//  Description:   Main routine of DiyTx system for my FPV Quad
//                 (w/ non-disclosure Reg maps)
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2016.04.18
//
//  history:
//    2016.04.08  create for RX63N( GR-SAKURA ) base-system
//    2016.04.16  add codes to operate TPU & ADC w/ synccronization
//                (interrupt codes are not included yet..) 
//

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "rx63n/iodefine.h"
#include "rx63n/iodefine_enum.h"
#include "rx63n/interrupt_handlers.h"

//#include "rx63n/PortUtils.h"
#include "ppm_gen.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define sleep(X) for(j = 0; j < X*1000; j++) {}
#define steps(X) for(j = 0; j < X; j++) { __asm("nop"); }

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
void DataDisp( unsigned char );
void SysCoreUnlock( void );
void SysCoreLock( void );
void MPCUnlock( void );
void MPCLock( void );
      
// -------------------------------------------------------
// ------------------------------------------ Main routine  
int main( void )
{
  char i=0;
  int j, state_disp=0;
  unsigned int start_cnt = 0;
  int val_aux1 = 6;
  int val_an[4];

  // release a reg. protect for all Sytem core regon
  SysCoreUnlock();

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
  
  // Module stop setting
  //  MSTP(TMR0) = MSTP_STOP;  // MSTP_RUN / MSTP_STOP
  //  MSTP(TMR2) = MSTP_STOP;
  MSTP(TPU3)  = MSTP_RUN;
  MSTP(S12AD) = MSTP_RUN;
  
  // distribute CLK to all module
  //SYSTEM.MSTPCRA.LONG = 0x00000000;
  //SYSTEM.MSTPCRB.LONG = 0x00000000;
  //SYSTEM.MSTPCRC.LONG = 0x00000000;

  // (to do:) LVD setting
  // **********
  
  // Enable NMI irq
  //ICU.NMIER.BIT.NMIEN = 0;
  
  // Close mission critical register
  SysCoreLock();
  
  //
  // %%add%% init IO Port
  //
  // < Available IO Ports @ RX63N 100pin > *under construction
  //   PORT0 [P03, P05]                  (x2)
  //   PORT1 [P14 -> P17]                (x4)
  //   PORT2 [P26, P27]                  (x2)
  //   PORT3 [P30 -> P32, P35]           (x4)
  //   PORT4 [P40 -> P44, P46]           (x6)
  //   PORT5 [P54, P55]                  (x2)
  //   PORTA [PA0, PA1, PA3, PA4, PA6]   (x5)
  //   PORTB [PB0, PB1, PB3, PB5 -> PB7] (x5)
  //   PORTC [PC2 -> PC7]                (x6)
  //   PORTE [PE0 -> PE7]                (x8)
  //   PORTH [PH0 -> PH3 *PH6, *PH7]     (x4(*6))
  //   PORTJ [PJ3, PJ5]                  (x(*2))

  // IO Port setting
  MPCUnlock();
  // PA0-2,6 -> LED indicator
  PORTA.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTA.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTA.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTA.PMR.BIT.B6 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTA.PDR.BIT.B0 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORTA.PDR.BIT.B1 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORTA.PDR.BIT.B2 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORTA.PDR.BIT.B6 = PDR_OUT;        // PDR_IN / PDR_OUT
  // PA7 -> Onboard SW input
  PORTA.PMR.BIT.B7 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTA.PDR.BIT.B7 = PDR_IN;         // PDR_IN / PDR_OUT
  
  // P4 -> ADC input (Vref 3.3V)
  /* PortConfADC( MPC.P40PFS, ASEL_ON, ISEL_OFF, PORT4, 0, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P41PFS, ASEL_ON, ISEL_OFF, PORT4, 1, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P42PFS, ASEL_ON, ISEL_OFF, PORT4, 2, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P43PFS, ASEL_ON, ISEL_OFF, PORT4, 3, PMR_FUNC, PDR_IN, 0 ); */
  PORT4.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B2 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B3 = PDR_IN;         // PDR_IN / PDR_OUT
  MPC.P40PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P40PFS.BIT.ASEL  = ASEL_ON;
  MPC.P41PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P41PFS.BIT.ASEL  = ASEL_ON;
  MPC.P42PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P42PFS.BIT.ASEL  = ASEL_ON;
  MPC.P43PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P43PFS.BIT.ASEL  = ASEL_ON;
  PORT4.PMR.BIT.B0 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B3 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  
  // P21 -> PPM output (3.3V)
  /* PortConfMPC( PDR_OUT, 0, PMR_FUNC, P21PFS_TIOCA3 ); */
  PORT2.PODR.BIT.B1  = 1;
  PORT2.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P21PFS.BIT.PSEL  = P21PFS_TIOCA3;
  PORT2.PMR.BIT.B1     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  MPCLock();
  // test (primitive ppm from PJ3)
  PORTJ.PODR.BIT.B3    = 1;
  PORTJ.PDR.BIT.B3     = PDR_OUT;       // PMR_GPIO / PMR_FUNC
  PORTJ.PMR.BIT.B3     = PMR_GPIO;       // PMR_GPIO / PMR_FUNC

  
  // TPUa setting
  TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_16;  // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz
  TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE;       // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz
  TPU3.TCR.BIT.CCLR    = TPU_CCLR_TGRA;       // TCNT cleared by TGRA
  TPU3.TMDR.BIT.MD     = TPU_MD_NORM;         // normal mode
  TPU3.TMDR.BIT.BFA    = TMDR_BFx_BUFF;        // buffer operation
  TPU3.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // buffer operation
  TPU3.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;   // ch B (unused)
  TPU3.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;   // ch D (unused)
  TPU3.TIORH.BIT.IOA   = TPU_IOX_OHCT;        // TIOCAn
  TPU3.TIORH.BIT.IOB   = TPU_IOX_DE;          // disable
  TPU3.TIORL.BIT.IOC   = TPU_IOX_DE;          // disable
  TPU3.TIORL.BIT.IOD   = TPU_IOX_DE;          // disable
  TPU3.TIER.BIT.TGIEA  = TGIEX_DE;            // IRQ enable (temp de)
  TPU3.TIER.BIT.TTGE   = TTGE_EN;             // enable: ADC start
  TPUA.TSTR.BIT.CST3   = CSTn_STOP;           // stop: TPU3
  
  
  // ADC setting (to do: librarize setting process)
  S12AD.ADCSR.BIT.ADST  = ADST_STOP;   // ADST_START / ADST_STOP
  // scan ch setting
  S12AD.ADADC.BIT.ADC   = ADADC_X4;   // ADADC_X1 / ADADC_X2 / ADADC_X3 / ADADC_X4
  S12AD.ADADS0.WORD     = 0x000F;   // (0000000000001111) AN000 - 015
  S12AD.ADADS1.WORD     = 0x0000;   // (0000000000011111) AN016 - 020
  S12AD.ADANS0.WORD     = 0x000F;   // (0000000000001111) AN000 - 015
  S12AD.ADANS1.WORD     = 0x0000;   // (0000000000011111) AN016 - 020
  // config
  S12AD.ADCER.BIT.ACE      = ACE_DE;          // ACE_DE / ACE_EN
  S12AD.ADCER.BIT.ADRFMT   = ADRFMT_FLLEFT;   // ADRFMT_FLRIGHT / ADRFMT_FLLEFT
  S12AD.ADSTRGR.BIT.ADSTRS = ADSTRS_TRGAN_1;  // TPUn.TGRA
  S12AD.ADSSTR01.BIT.SST1  = 0x18;            // A/D Sampling State Register 01
                                             // (PCLK 48MHz -> 48/2 steps, 1u/2 = 500ns)
  S12AD.ADSSTR23.BIT.SST2  = 0xFF;           // A/D Sampling State Register 23 (Temp sensor)
  S12AD.ADCSR.BIT.CKS     = CKS_PCLK_2;      // CKS_PCLK_8 / CKS_PCLK_4 / CKS_PCLK_2 / CKS_PCLK
  S12AD.ADCSR.BIT.ADIE    = ADIE_DE;         // ADIE_DE / ADIE_EN
  S12AD.ADCSR.BIT.EXTRG   = EXTRG_SYNC;      // EXTRG_SYNC / EXTRG_ASYN
  S12AD.ADCSR.BIT.TRGE    = TRGE_EN;         // TRGE_EN / TRGE_DE
  S12AD.ADCSR.BIT.ADCS    = ADCS_SINGLE;     // ADCS_SINGLE / ADCS_CONT
  S12AD.ADCSR.BIT.ADST    = ADST_STOP;       // ADST_START / ADST_STOP
  // [to do: ] need to add irq setting

  
  // PPM generation
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  // 8 times { H: init(0.7m)+(0 - 1.0 m) -> stop tail(0.3m) -> H: next... } + frame tail (H: 6.2m L: 0.3m)
  // requirements: time resolution < 1ms / 11bit   = 1m / 2^11 =~ 0.5u,
  //               max time length > 1ms           
  // calc time resolution:  1/(48M / 2^3) = 1/6 u = 166...ns
  // calc max time length:  2^16 / (48M / 2^3) = 1/(3*2^4) 2^19 us = 1/3 * 2^5 *1024 us = 32/3 * 1024 us = ~10ms
  TPU3.TGRA            = 4200;     // 700u * 6.0M = 4200 ()
  TPU3.TGRC            = 1800;     // 300u * 6.0M = 1800 ()
  //IR(TPU3, TGI3A) = 1;
  //IR(S12AD, S12ADI0) = 1;
  TPUA.TSTR.BIT.CST3   = CSTn_RUN;   // start to run timer pulse
	  
  //
  while(1) {
    // ch0
    sleep(9);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch1
    sleep(9);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    
    // ch2 (throttle)
    sleep(8);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch3
    sleep(9);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch4
    sleep(val_aux1);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch5
    sleep(10);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch6
    sleep(10);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch7
    sleep(10);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // sync
    sleep(50);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    if( PORTA.PIDR.BIT.B7 == 0 ){
      /* Disp LED according to AN0 */
      unsigned char test = MPC.P21PFS.BIT.PSEL;
      test = SYSTEM.SCKCR3.BIT.CKSEL;
      test = ( S12AD.ADDR0 >> 12 );
      
      //      test = MPC.PWPR.BYTE;
      /* test = TPU3.TGRA; */
      /* test = S12AD.ADDR0; */
      /* test = TPU3.TCNT; */
      DataDisp( test );
    }
    else{
      //PORTA.PODR.BYTE = i++ & (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
      PORTA.PODR.BYTE = i++ & ( 1 << 6 );
      /* Set GPIOs according to i */
      //PORTA.PODR.BYTE = i++ & (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
    }

        if( start_cnt == 10000000 ){ val_aux1 = 10; }
        start_cnt++;
  }
  
}

void INT_Excep_TPU3_TGI3A(void){
  
  return;
}

void INT_Excep_S12AD_S12ADI0(void){
  return;
}

// -------------------------------------------------------
// -------------------------------- Functions( Utilities )
//
//  disp data on LED ( mounted on Eval Brd. )
//
void DataDisp( unsigned char data ){
  PORTA.PDR.BYTE = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
  PORTA.PODR.BYTE = 0x00;
  
  PORTA.PODR.BYTE   = ( (data >> 0) & 0x07 );
  PORTA.PODR.BIT.B6 = ( (data >> 6) & 0x01 );
}

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
void MPCUnlock( void ){
  MPC.PWPR.BIT.B0WI  = B0WI_UNLOCK;
  MPC.PWPR.BIT.PFSWE = PFSWE_UNLOCK;
}


void MPCLock( void ){
  MPC.PWPR.BIT.PFSWE = PFSWE_LOCK;
  MPC.PWPR.BIT.B0WI  = B0WI_LOCK;
}

