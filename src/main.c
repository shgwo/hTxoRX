// -------------------------------------------------------
// ------------------------------------------------ Notice
//  This program is distributed or redistributed to
//  the world under the License of MIT.
//
//  
//
// -------------------------------------------------------
// ------------------------------------------------- Info.
//
//  Description:   Main routine of DiyTx system for my FPV Quad
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
#include "iodefine.h"
#include "iodefine_enum.h"
#include "interrupt_handlers.h"
#include "typedefine.h"
//#include "rx63n/PortUtils.h"
#include "ppm_gen.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define sleep(X) for(j = 0; j < X*1000; j++) {}
#define steps(X) for(j = 0; j < X; j++) { __asm("nop"); }

#define NCH_ADC 21   // number of AD channel on the HW

enum enum_AppliMode {
  OPMD_SAFE,
  OPMD_DIAG,
  OPMD_RUN,
  OPMD_FAIL,
  OPMD_UNKNOWM
};

// -------------------------------------------------------
// -------------------------------------- global variables
struct st_PPMAdj ppm_adj[NCH_PPM];

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
  // for interface clock
  MSTP(TMR0)  = MSTP_RUN;
  // LED
  MSTP(TPU0)  = MSTP_RUN;
  MSTP(TPU4)  = MSTP_RUN;
  // for PPM
  MSTP(TPU3)  = MSTP_RUN;
  // for gimbal
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
  // PE[0:1] -> Arm SW input
  // ( PE6: SW [ ARM ] (ON),  ON-OFF-(ON) )
  //   PE7: SW [ ARM ] ON,    ON-OFF-(ON) )
  PORTE.PCR.BIT.B6 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B7 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PDR.BIT.B6 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B7 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PMR.BIT.B6 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B7 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  // PC[0:1], P5[0:1] -> reserved SW input
  //   P50: SW #1 [ Left ]      ON
  //   P51: SW #1 [ Left ]      (ON)
  // ( PC0: SW #2 [ Left mid ]  tail   
  //   PC0: SW #2 [ Left mid ]  head   
  //   PE0: SW #3 [ Right mid ] 
  //   PE3: SW #3 [ Right mid ] 
  //   PE4: SW #4 [ Right ] (ON)
  //   PE5: SW $5 [ Right ] ON       )
  PORT5.PCR.BIT.B0 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT5.PCR.BIT.B1 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B0 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B1 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B0 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B3 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B4 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B5 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT5.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT5.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B3 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B4 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B5 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT5.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT5.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B4 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B5 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  
  // P4 -> ADC input (Vref 3.3V)
  /* PortConfADC( MPC.P40PFS, ASEL_ON, ISEL_OFF, PORT4, 0, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P41PFS, ASEL_ON, ISEL_OFF, PORT4, 1, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P42PFS, ASEL_ON, ISEL_OFF, PORT4, 2, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P43PFS, ASEL_ON, ISEL_OFF, PORT4, 3, PMR_FUNC, PDR_IN, 0 ); */
  PORT4.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B5 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B2 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B3 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B5 = PDR_IN;         // PDR_IN / PDR_OUT
  MPC.P40PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P40PFS.BIT.ASEL  = ASEL_ON;
  MPC.P41PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P41PFS.BIT.ASEL  = ASEL_ON;
  MPC.P42PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P42PFS.BIT.ASEL  = ASEL_ON;
  MPC.P43PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P43PFS.BIT.ASEL  = ASEL_ON;
  MPC.P45PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P45PFS.BIT.ASEL  = ASEL_ON;
  PORT4.PMR.BIT.B0 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B3 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B5 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  
  // P21 -> PPM output (3.3V)
  /* future func => PortConfMPC( PDR_OUT, 0, PMR_FUNC, P21PFS_TIOCA3 ); */
  PORT2.PODR.BIT.B1  = 1;
  PORT2.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P21PFS.BIT.PSEL  = P21PFS_TIOCA3;
  PORT2.PMR.BIT.B1     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC

  // P22,23,P33 -> LED status indicator ( LED extention board )
  // PC[0:1], P5[0:1] -> reserved SW input
  // ( P22: status Pink
  //   P23: status Violet
  //   P33: status Blue
  //   P24: status(RGB) Red
  //   P25: status(RGB) Blue
  //   P32: status(RGB) Green )
  PORT2.PODR.BIT.B2  = 1;
  PORT2.PODR.BIT.B3  = 1;
  PORT3.PODR.BIT.B3  = 1;
  PORT2.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT2.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT3.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B2 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORT2.PDR.BIT.B3 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORT3.PDR.BIT.B3 = PDR_OUT;        // PDR_IN / PDR_OUT
  // P24, 25, P32 -> LED gradational status indicator ( LED extention board )
  PORT2.PODR.BIT.B4  = 0;            //Red
  PORT2.PODR.BIT.B5  = 1;            //Blue
  PORT3.PODR.BIT.B2  = 0;            //Green
  PORT2.PMR.BIT.B4 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT2.PMR.BIT.B5 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT3.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B4 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORT2.PDR.BIT.B5 = PDR_OUT;        // PDR_IN / PDR_OUT
  PORT3.PDR.BIT.B2 = PDR_OUT;        // PDR_IN / PDR_OUT
  
  /* // P24 -> LED gradation PWM output (Red) */
  /* PORT2.PODR.BIT.B4  = 1; */
  /* PORT2.PMR.BIT.B4   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC */
  /* PORT2.PDR.BIT.B4   = PDR_OUT;          // PDR_IN / PDR_OUT */
  /* MPC.P24PFS.BIT.PSEL  = P24PFS_MTIOC4A; */
  /* PORT2.PMR.BIT.B4     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC */
  /* // P25 -> LED gradation PWM output (Blue) */
  PORT2.PODR.BIT.B5  = 1;
  PORT2.PMR.BIT.B5   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B5   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P25PFS.BIT.PSEL  = P25PFS_TIOCA4;
  PORT2.PMR.BIT.B5     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  // P32(,33) -> LED gradation PWM output (Green)
  PORT3.PODR.BIT.B2  = 1;
  PORT3.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT3.PDR.BIT.B2   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P32PFS.BIT.PSEL  = P32PFS_TIOCC0;
  PORT3.PMR.BIT.B2     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  //relock MPC
  MPCLock();

  // PJ3 debug out  (for probing by oscilloscopes )
  PORTJ.PODR.BIT.B3    = 1;
  PORTJ.PDR.BIT.B3     = PDR_OUT;       // PMR_GPIO / PMR_FUNC
  PORTJ.PMR.BIT.B3     = PMR_GPIO;       // PMR_GPIO / PMR_FUNC

  
  // system loop time step setting
  //  TMR0.TCORA.BIT
  
 
  // TPUa setting (for PPM)
  TPUA.TSTR.BIT.CST3   = CSTn_STOP;           // stop: TPU3
  TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_16;  // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz
  TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE;  // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz
  //TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_4;   // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz (high-reso test)
  //TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz (high-reso test)
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
  TPU3.TIER.BIT.TGIEA  = TGIEX_EN;            // IRQ enable (temp DE)
  TPU3.TIER.BIT.TTGE   = TTGE_EN;             // enable: ADC start

  //TPUa settings ( for gradational LED, Blue )
  TPUA.TSTR.BIT.CST4   = CSTn_STOP;           // stop: TPU04
  TPU4.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_1024;  // (12Mhz x 4) / 2^10 -> 2*2*3M/2^8 = ~21kHz
  TPU4.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // freq is same as above.
  TPU4.TCR.BIT.CCLR    = TPU_CCLR_TGRA;       // TCNT cleared by TGRA
  TPU4.TMDR.BIT.MD     = TPU_MD_PWM1;         // PWM1 mode
  TPU4.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  TPU4.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  TPU4.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;   // ch B (unused)
  TPU4.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;   // ch D (unused)
  TPU4.TIOR.BIT.IOA   = TPU_IOX_OLCL;        // TIOCAn
  TPU4.TIOR.BIT.IOB   = TPU_IOX_OLCH;          // disable
  TPU4.TIER.BIT.TGIEA  = TGIEX_DE;            // IRQ enable (temp DE)
  TPU4.TIER.BIT.TTGE   = TTGE_DE;             // enable: ADC start
  
  //TPUa settings ( for gradational LED, Green )
  TPUA.TSTR.BIT.CST0   = CSTn_STOP;           // stop: TPU0
  TPU0.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_64;  // (12Mhz x 4) / 2^6 -> 2*2*3M/2^4 = ~1.5MHz
  TPU0.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // freq is same as above.
  TPU0.TCR.BIT.CCLR    = TPU_CCLR_TGRC;       // TCNT cleared by TGRA
  TPU0.TMDR.BIT.MD     = TPU_MD_PWM1;         // PWM1 mode
  TPU0.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  TPU0.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  TPU0.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;   // ch B (unused)
  TPU0.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;   // ch D (unused)
  TPU0.TIORH.BIT.IOA   = TPU_IOX_DE;          // disable
  TPU0.TIORH.BIT.IOB   = TPU_IOX_DE;          // disable
  TPU0.TIORL.BIT.IOC   = TPU_IOX_OLCL;        // TIOCCn
  TPU0.TIORL.BIT.IOD   = TPU_IOX_OLCH;          // disable
  TPU0.TIER.BIT.TGIEA  = TGIEX_DE;            // IRQ enable (temp DE)
  TPU0.TIER.BIT.TTGE   = TTGE_DE;             // enable: ADC start
  
  
  // ADC setting (to do: librarize setting process)
  S12AD.ADCSR.BIT.ADST  = ADST_STOP;   // ADST_START / ADST_STOP
  // scan ch setting ( ch select, addition )
  S12AD.ADANS0.WORD     = 0x202F;   // (b0010 0000 0010 1111) AN015 - AN000
  S12AD.ADANS1.WORD     = 0x0000;   // (b---- ---- ---0 0000) AN020 - AN016,
                                    //   |----> "-" is fixed value as 0 
  S12AD.ADADC.BIT.ADC   = ADADC_X4;   // ADADC_X1 / ADADC_X2 / ADADC_X3 / ADADC_X4
  S12AD.ADADS0.WORD     = 0x202F;   // (b0010 0000 0010 1111) AN015 - AN000
  S12AD.ADADS1.WORD     = 0x0000;   // (b---- ---- ---0 0000) AN020 - AN016,
                                    //   |----> "-" is fixed value as 0
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

  // LED gradation
  TPU4.TGRA = 0xFFFF;
  TPU4.TGRB = 0x8010;
  TPU0.TGRC = 0xFFFF;
  TPU0.TGRD = 0x8010;
  TPUA.TSTR.BIT.CST0   = CSTn_RUN;   // start to iluminite LED
  TPUA.TSTR.BIT.CST4   = CSTn_RUN;   // start to iluminate LED
  
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
  TPU3.TGRD            = 37200;     // 6200u * 6.0M = 1800 ()
  //IR(TPU3, TGI3A) = 1;
  //IR(S12AD, S12ADI0) = 1;
  IEN( TPU3, TGI3A ) = 0;
  IPR( TPU3, TGI3A ) = 3;
  IEN( S12AD, S12ADI0 ) = 0;
  IPR( S12AD, S12ADI0 ) = 5;
  //__builtin_rx_setpsw( 'I' );
  //sss__asm __volatile("setpsw i\n");
  //__asm __volatile("clrpsw i\n");
  TPUA.TSTR.BIT.CST3   = CSTn_RUN;   // start to run timer pulse


  // initializing PPM ch val ( * future func *)
  // Name labels for each PPM ch
  char name[NCH_PPM][10] = { "Roll", "Pitch", "Throttle", "Yaw", "Arm", "AUX2", "MODE", "AUX4" };
  // set adj ( name str, ad_ch, inv, offset, gain )
  PPMGenAdjInit( &ppm_adj[0], name[0],  2, PPMADJ_NOINV,  (182 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[1], name[1],  3, PPMADJ_INV,    (162 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[2], name[2],  0, PPMADJ_NOINV, -(114 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[3], name[3],  1, PPMADJ_INV,    (783 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[4], name[4], 22, PPMADJ_NOINV,          0, 1.0 );
  PPMGenAdjInit( &ppm_adj[5], name[5], 22, PPMADJ_NOINV,          0, 1.0 );
  PPMGenAdjInit( &ppm_adj[6], name[6], 22, PPMADJ_NOINV,          0, 1.0 );
  PPMGenAdjInit( &ppm_adj[6], name[6], 22, PPMADJ_NOINV,          0, 1.0 );

  
  // 
  // Main routine start
  //
  //
  uint8_t op_md = OPMD_SAFE;
  uint8_t  status = 0;
  uint8_t  flag   = 0;
  uint16_t ppm_val[8], adc_val[21], adc_bat = 0;
  //unsigned char test = 0;
  
  while(1) {
    // state machine
    switch( op_md ){      
    case OPMD_SAFE:
      
      break;
      
    case OPMD_RUN:
      
      break;
    }
    
    // get ADC val & arrange for PPM ch
    // future func => PPMGenInputSelector()
    adc_val[0] = (S12AD.ADDR2 >> (2 + 2) ) + (182 * 6); // Roll
    //adc_val[1] = (S12AD.ADDR3 >> (2 + 2) ) + (162 * 6); // Pitch (old NINV)
    adc_val[1] = (~S12AD.ADDR3 >> (2 + 2) ) +(833 * 6); // Pitch
    //adc_val[2] = (S12AD.ADDR0 >> (2 + 2) ) - (114 * 6); // Throttle (old)
    adc_val[2] = (uint16_t)(1.0*(S12AD.ADDR0 >> 3 ) - (224 * 6) ); // Throttle (971 - 1932)
    //adc_val[3] = (S12AD.ADDR1 >> (2 + 2) ) + (209 * 6); // Yaw (NINV)
    adc_val[3] = (~S12AD.ADDR1 >> (2 + 2) ) + (783 * 6); // Yaw (INV)
    adc_val[4] = 0;
    adc_val[5] = 0;
    adc_val[6] = (S12AD.ADDR5 >> (2 + 2) ); // AUX3
    adc_val[7] = 0;

    adc_bat = (S12AD.ADDR8 >> (2 + 0) );

    // future func => PPMGenSafeChecker()  <- safe limiter
    // future func => PPMGenOutputFilter() <- LUT-base conversion
    
    // battery check 


    // Motors disarming check
    adc_val[4] = (PORTE.PIDR.BIT.B7 ? 0 : 2049);
    if( PORTE.PIDR.BIT.B7 ){
      adc_val[0] = 0; // Roll
      adc_val[1] = 0; // Pitch
      adc_val[2] = 0; // Yaw
      adc_val[3] = 0; // Throttle
    }

    // PPM generation core
    // future func => PPMGen()
    // check tpu renew timing from interrupt occurance flag
    if( IR( TPU3, TGI3A ) == 1 ){
      // tail pulse
      if( TPU3.TGRA > 2000 ){
	TPU3.TGRC = 1800;
	status++;
	if( status > 8 ){ status = 0; }
      }
      // ppm pulse
      else{
	// end pulse
	if( status >= 8 ){
	  TPU3.TGRC = 37200;
	  // debug
	  PORTA.PODR.BIT.B0 = !PORTA.PODR.BIT.B0;
	}
	// significant pulse
	else{
	  TPU3.TGRC = 4200 + adc_val[status];
	}
      }
      IR( TPU3, TGI3A ) = 0;
      PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    }


    // HMI routine
    // LED indication (for debug, printf)
    if( PORTE.PIDR.BIT.B6 == 0 ){
      unsigned char test = ( S12AD.ADDR13 >> 12 );
      DataDisp( test );
    }
    else if( PORTE.PIDR.BIT.B7 == 0 ){
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
      unsigned char test = ( S12AD.ADDR5 >> 12 );
      DataDisp( test );
      //PORTA.PODR.BYTE = i++ & (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
      //PORTA.PODR.BYTE = i++ & ( 1 << 6 );
      /* Set GPIOs according to i */
      //PORTA.PODR.BYTE = i++ & (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
    }

    // test for circuit
    if( PORTC.PIDR.BIT.B0 == 0 ){
      PORT2.PODR.BIT.B2 = 1; // pink LED
    }
    else{
      PORT2.PODR.BIT.B2 = 0;
    }
    
    if( PORTC.PIDR.BIT.B1 == 0 ){
      PORT2.PODR.BIT.B3 = 1; // violet LED
    }
    else{
      PORT2.PODR.BIT.B3 = 0;
    }

    if( PORT5.PIDR.BIT.B0 == 0 ){
      PORT3.PODR.BIT.B3 = 1;
    }
    else{
      PORT3.PODR.BIT.B3 = 0;
    }
    // red
    if( PORT5.PIDR.BIT.B1 == 0 ){
      PORT2.PODR.BIT.B4 = 0;
    }
    else{
      PORT2.PODR.BIT.B4 = 1;
    }
    // green
    if( PORTE.PIDR.BIT.B0 == 0 ){
      PORT2.PODR.BIT.B5 = 0;
    }
    else{
      PORT2.PODR.BIT.B5 = 1;
    }
    // blue
    if( PORTE.PIDR.BIT.B3 == 0 ){
      PORT3.PODR.BIT.B2 = 1;
    }
    else{
      PORT3.PODR.BIT.B2 = 0;
    }
    // RGB on
    if( PORTE.PIDR.BIT.B4 == 0 ){
      PORT2.PODR.BIT.B4 = 1;
      PORT2.PODR.BIT.B5 = 0;
      PORT3.PODR.BIT.B2 = 0;
    }
    else{
    }
    // RGB off
    if( PORTE.PIDR.BIT.B5 == 0 ){
      PORT2.PODR.BIT.B4 = 1;
      PORT2.PODR.BIT.B5 = 0;
      PORT3.PODR.BIT.B2 = 1;
    }
    else{
    }
        
    
    //        if( start_cnt == 10000000 ){ val_aux1 = 10; }
    //        start_cnt++;
  }
  
}

// -------------------------------------------------------
// -------------------------------- Functions( Utilities )
//
//  disp data on LED ( mounted on Eval Brd. )
//
void DataDisp( unsigned char data ){
  // port setting ( I/O OUT, Low )
  PORTA.PODR.BYTE = 0x00;  
  PORTA.PDR.BYTE = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
  
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
void MPCUnlock( void ){
  MPC.PWPR.BIT.B0WI  = B0WI_UNLOCK;
  MPC.PWPR.BIT.PFSWE = PFSWE_UNLOCK;
}

void MPCLock( void ){
  MPC.PWPR.BIT.PFSWE = PFSWE_LOCK;
  MPC.PWPR.BIT.B0WI  = B0WI_LOCK;
}
