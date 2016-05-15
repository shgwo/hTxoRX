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
// state difinition for main operation
enum enum_AppMode {
  OPMD_INIT,
  OPMD_DIAG,
  OPMD_SAFE,
  OPMD_RUN_INIT,
  OPMD_RUN,
  OPMD_FAIL,
  OPMD_UNKNOWN
};
enum enum_AppModeLog {
  OPMD_LOG_OFF,
  OPMD_LOG_ON
};
enum enum_AppModeBat {
  OPMD_BAT_LOW,
  OPMD_BAT_MID,
  OPMD_BAT_FULL
};

// label definition for HMI input source
enum enum_HMI_SW {
  HMI_SW_ARM_KEY,
  HMI_SW_ARM_LOG,
  HMI_SW_ROT,
  HMI_SW_L1_KEY,
  HMI_SW_L1_LCK,
  HMI_SW_L2_F,
  HMI_SW_L2_B,
  HMI_SW_R1_KEY,
  HMI_SW_R1_LCK,
  HMI_SW_R2_F,
  HMI_SW_R2_B,
  HMI_N_SW
};
enum enum_HMI_GMBL {
  HMI_GMBL_LV,
  HMI_GMBL_LH,
  HMI_GMBL_RV,
  HMI_GMBL_RH,
  HMI_N_GMBL
};
enum enum_HMI_TRM {
  HMI_TRM_ROT,
  HMI_TRM_VOL,
  HMI_N_TRM
};
enum enum_HMI_LED {
  HMI_LED_POW,
  HMI_LED_LOG,
  HMI_LED_NORM,
  HMI_N_LED
};
enum enum_HMI_LED_RGB {
  HMI_LED_R,
  HMI_LED_G,
  HMI_LED_B,
  HMI_N_LED_RGB
};
  
// -------------------------------------------------------
// -------------------------------------- global variables
st_PPMAdj ppm_adj[NCH_PPM];
struct st_PPMAdj ppm_adj[NCH_PPM];

typedef struct st_HMI {
  // for inputs
  uint8_t  sw_state[HMI_N_SW];
  uint16_t sw_cnt [HMI_N_SW];
  //  uint16_t sw_ncnt [HMI_N_SW];
  uint16_t gmbl [HMI_N_GMBL];
  uint16_t trm [HMI_N_TRM];
  // for output
  uint8_t  LED_state[HMI_N_LED];
  uint8_t  ppm_cnt;
  uint16_t LED_RGB[HMI_N_LED_RGB];
  // for utility
  uint16_t cnt_sw;
  uint16_t cnt_led;
  uint16_t cnt_ppm;
  uint16_t cnt_bat;
} st_HMI;

st_HMI hmi;

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
uint8_t HMISWInit( struct st_HMI* );
uint8_t HMIScanSW ( struct st_HMI* );
uint8_t HMISWState ( struct st_HMI*, enum enum_HMI_SW );
uint8_t HMILongPress ( struct st_HMI*, enum enum_HMI_SW, uint16_t );
uint8_t HMILEDPPMAct( st_HMI*, enum enum_AppModeLog, uint8_t);
uint8_t HMILEDBatLow( st_HMI *, enum enum_AppModeBat, uint8_t );
uint8_t HMILEDSetRGB( st_HMI*, enum enum_AppMode );
uint8_t HMILEDFlash( st_HMI*, enum enum_AppMode, enum enum_AppModeBat, enum enum_AppModeLog );

void DataDisp( unsigned char );
void SysCoreUnlock( void );
void SysCoreLock( void );
void MPCUnlock( void );
void MPCLock( void );
void MTU34Unlock( void );
// -------------------------------------------------------
// ------------------------------------------ Main routine  
int main( void )
{
  char i=0;
  int j, state_disp=0;
  uint8_t op_md     = OPMD_INIT;
  uint8_t op_md_log = OPMD_LOG_OFF;
  uint8_t op_md_bat = OPMD_BAT_MID;

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
  MSTP(CMT0)  = MSTP_RUN;
  MSTP(CMT1)  = MSTP_RUN;
  // LED
  //MSTP(TPU0)  = MSTP_RUN;
  //MSTP(TPU4)  = MSTP_RUN;
  MSTP(MTU0)  = MSTP_RUN;
  MSTP(MTU4)  = MSTP_RUN;
  // for PPM
  MSTP(TPU3)  = MSTP_RUN;
  // for gimbal
  MSTP(S12AD) = MSTP_RUN;
  
  // distribute CLK to all module
  //SYSTEM.MSTPCRA.LONG = 0x00000000;
  //SYSTEM.MSTPCRB.LONG = 0x00000000;
  //SYSTEM.MSTPCRC.LONG = 0x00000000;

  // (nothing to do:) LVD setting
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
  // PA0-2,6 -> Onboard LED indicator
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
  // ( P52: SW #0 [ LeftLeft ]  (ON) on rotary encoder
  //   P50: SW #1 [ Left ]      ON
  //   P51: SW #1 [ Left ]      (ON)
  //   PC0: SW #2 [ Left mid ]  tail   
  //   PC0: SW #2 [ Left mid ]  head   
  //   PE0: SW #3 [ Right mid ] 
  //   PE3: SW #3 [ Right mid ] 
  //   PE4: SW #4 [ Right ] (ON)
  //   PE5: SW $5 [ Right ] ON       )
  PORT5.PCR.BIT.B0 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT5.PCR.BIT.B1 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT5.PCR.BIT.B2 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B0 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B1 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B0 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B3 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B4 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B5 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT5.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT5.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT5.PDR.BIT.B2 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B3 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B4 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B5 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT5.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT5.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT5.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B4 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B5 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC

  // P22,23,P33 -> LED status indicator ( LED extention board )
  // PC[0:1], P5[0:1] -> reserved SW input
  // ( P22: status Pink
  //   P23: status Violet
  //   P33: status Blue )
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
  /* // P24 -> LED gradation PWM output (Red) */
  PORT2.PODR.BIT.B4  = 1;
  PORT2.PMR.BIT.B4   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B4   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P24PFS.BIT.PSEL  = P24PFS_MTIOC4A;
  PORT2.PMR.BIT.B4     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  /* // P25 -> LED gradation PWM output (Blue) */
  PORT2.PODR.BIT.B5  = 1;
  PORT2.PMR.BIT.B5   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B5   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P25PFS.BIT.PSEL  = P25PFS_TIOCA4;
  MPC.P25PFS.BIT.PSEL  = P25PFS_MTIOC4C;
  PORT2.PMR.BIT.B5     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  // P32 -> LED gradation PWM output (Green)
  PORT3.PODR.BIT.B2  = 1;
  PORT3.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT3.PDR.BIT.B2   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P32PFS.BIT.PSEL  = P32PFS_TIOCC0;
  MPC.P32PFS.BIT.PSEL  = P32PFS_MTIOC0C;
  PORT3.PMR.BIT.B2     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC

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

  // P21 -> PPM output (3.3V pulse out)
  /* future func => PortConfMPC( PDR_OUT, 0, PMR_FUNC, P21PFS_TIOCA3 ); */
  PORT2.PODR.BIT.B1  = 1;
  PORT2.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT2.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT
  MPC.P21PFS.BIT.PSEL  = P21PFS_TIOCA3;
  PORT2.PMR.BIT.B1     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC

  // PJ3 debug out  (for probing by oscilloscopes )
  PORTJ.PODR.BIT.B3    = 1;
  PORTJ.PDR.BIT.B3     = PDR_OUT;       // PMR_GPIO / PMR_FUNC
  PORTJ.PMR.BIT.B3     = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  //relock MPC
  MPCLock();

  // CMT0 setting (for HMI input time loop)
  // td_in = 1ms (=1kHz); 12MHz * 4 / 2^9 / 2^(1+5)
  CMT0.CMCR.BIT.CKS = CMT_CKS_PCLK_512;  // PCLK/512;
  CMT0.CMCOR = 0x001F;                   // PCLK / 2^9 / 2^5;
  CMT0.CMCR.BIT.CMIE = CMIE_EN;          // interrupt flag enable
  IEN( CMT0, CMI0 ) = 0;
  IPR( CMT0, CMI0 ) = 7;
  // td_out = 10ms (=100fps); 12MHz * 2^2 / 2^9 / 2^(1+2+7)
  CMT1.CMCR.BIT.CKS = CMT_CKS_PCLK_512;  // PCLK/512;
  CMT1.CMCOR = 0x03FF;                   // PCLK / 2^9 / 2^10;
  CMT1.CMCR.BIT.CMIE = CMIE_EN;          // interrupt flag enable
  IEN( CMT1, CMI1 ) = 0;
  IPR( CMT1, CMI1 ) = 8;
  // start all HMI loop clock
  CMT.CMSTR0.BIT.STR0 = CMSTRn_RUN;       // CMT_[RUN / STOP]
  CMT.CMSTR0.BIT.STR1 = CMSTRn_RUN;       // CMT_[RUN / STOP]
  
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
  TPU3.TIORH.BIT.IOA   = IOX_OHCT;        // TIOCAn
  TPU3.TIORH.BIT.IOB   = IOX_DE;          // disable
  TPU3.TIORL.BIT.IOC   = IOX_DE;          // disable
  TPU3.TIORL.BIT.IOD   = IOX_DE;          // disable
  TPU3.TIER.BIT.TGIEA  = TGIEX_EN;            // IRQ enable (temp DE)
  TPU3.TIER.BIT.TTGE   = TTGE_EN;             // enable: ADC start

  //MTU2a settings ( for gradational LED, Red )
  MTU.TSTR.BIT.CST4    = CSTn_STOP;           // stop: MTU4
  MTU34Unlock();
  //  MTU.TRWER.BIT.RWE    = TRWER_RWE_EN;
  MTU4.TCR.BIT.TPSC    = MTU34_TPSC_PCLK;     // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  MTU4.TCR.BIT.CKEG    = CKEG_PEDGE;          // freq is same as above.
  MTU4.TCR.BIT.CCLR    = CCLR_TGRB;           // TCNT cleared by TGRB
  MTU4.TMDR.BIT.MD     = TMDR_MD_PWM1;         // PWM1 mode
  MTU4.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  MTU4.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  MTU4.TIORH.BIT.IOA   = IOX_OLCH;           // MTIOCnA
  MTU4.TIORH.BIT.IOB   = IOX_OLCL;           // MTIOCnA(PWM1)
  MTU4.TIORL.BIT.IOC   = IOX_OLCH;          // disable
  MTU4.TIORL.BIT.IOD   = IOX_OLCL;          // disable
  MTU.TOER.BIT.OE3B    = OEny_DE;          // OEny_[DE/EN] MTIOC3B
  MTU.TOER.BIT.OE4A    = OEny_EN;          // OEny_[DE/EN] MTIOC4A
  MTU.TOER.BIT.OE4B    = OEny_DE;          // OEny_[DE/EN] MTIOC4B
  MTU.TOER.BIT.OE3D    = OEny_DE;          // OEny_[DE/EN] MTIOC3D
  MTU.TOER.BIT.OE4C    = OEny_EN;          // OEny_[DE/EN] MTIOC4C
  MTU.TOER.BIT.OE4D    = OEny_DE;          // OEny_[DE/EN] MTIOC4D
  
  //TPUa settings ( for gradational LED, Blue )
  TPUA.TSTR.BIT.CST4   = CSTn_STOP;           // stop: TPU04
  TPU4.TCR.BIT.TPSC    = TPU410_TPSC_PCLK;      // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  TPU4.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // freq is same as above.
  TPU4.TCR.BIT.CCLR    = TPU_CCLR_TGRB;       // TCNT cleared by TGRA
  TPU4.TMDR.BIT.MD     = TPU_MD_PWM1;         // PWM1 mode
  TPU4.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  TPU4.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  TPU4.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;   // ch B (unused)
  TPU4.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;   // ch D (unused)
  TPU4.TIOR.BIT.IOA    = IOX_OLCH;        // TIOCAn
  TPU4.TIOR.BIT.IOB    = IOX_OLCL;          // disable
  TPU4.TIER.BIT.TGIEA  = TGIEX_DE;            // IRQ enable (temp DE)
  TPU4.TIER.BIT.TTGE   = TTGE_DE;             // enable: ADC start
  
  //TPUa settings ( for gradational LED, Green )
  TPUA.TSTR.BIT.CST0   = CSTn_STOP;           // stop: TPU0
  TPU0.TCR.BIT.TPSC    = TPU06_TPSC_PCLK;     // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  TPU0.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // freq is same as above.
  TPU0.TCR.BIT.CCLR    = TPU_CCLR_TGRD;       // TCNT cleared by TGRC
  TPU0.TMDR.BIT.MD     = TPU_MD_PWM1;         // PWM1 mode
  TPU0.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  TPU0.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  TPU0.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;   // ch B (unused)
  TPU0.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;   // ch D (unused)
  TPU0.TIORH.BIT.IOA   = IOX_DE;          // disable
  TPU0.TIORH.BIT.IOB   = IOX_DE;          // disable
  TPU0.TIORL.BIT.IOC   = IOX_OLCH;        // TIOCCn
  TPU0.TIORL.BIT.IOD   = IOX_OLCL;          // disable
  TPU0.TIER.BIT.TGIEA  = TGIEX_DE;            // IRQ enable (temp DE)
  TPU0.TIER.BIT.TTGE   = TTGE_DE;             // enable: ADC start

  MTU0.TCR.BIT.TPSC    = MTU0_TPSC_PCLK;     // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  MTU0.TCR.BIT.CKEG    = CKEG_PEDGE;          // freq is same as above.
  MTU0.TCR.BIT.CCLR    = CCLR_TGRD;           // TCNT cleared by TGRB
  MTU0.TMDR.BIT.MD     = TMDR_MD_PWM1;         // PWM1 mode
  MTU0.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  MTU0.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  MTU0.TIORH.BIT.IOA   = IOX_DE;           // disable
  MTU0.TIORH.BIT.IOB   = IOX_DE;           // disable
  MTU0.TIORL.BIT.IOC   = IOX_OLCH;          // MTIOCnC
  MTU0.TIORL.BIT.IOD   = IOX_OLCL;          // MTIOCnA(PWM1)


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
  S12AD.ADCSR.BIT.ADIE    = ADIE_EN;         // ADIE_DE / ADIE_EN
  S12AD.ADCSR.BIT.EXTRG   = EXTRG_SYNC;      // EXTRG_SYNC / EXTRG_ASYN
  S12AD.ADCSR.BIT.TRGE    = TRGE_EN;         // TRGE_EN / TRGE_DE
  S12AD.ADCSR.BIT.ADCS    = ADCS_SINGLE;     // ADCS_SINGLE / ADCS_CONT
  S12AD.ADCSR.BIT.ADST    = ADST_STOP;       // ADST_START / ADST_STOP
  // [to do: ] need to add irq setting

  // LED gradation (initial value)
  MTU4.TGRA = 0xFFFF;  // duty
  MTU4.TGRB = 0xFFFF;  // cycle
  MTU4.TGRC = 0xFFFF;  // duty
  TPU4.TGRA = 0x1100;  // duty
  TPU4.TGRB = 0xF000;  // cycle
  TPU0.TGRC = 0xF010;  // duty
  TPU0.TGRD = 0xFFF0;  // cycle
  MTU0.TGRC = 0xFFFF;  // duty
  MTU0.TGRD = 0xFFFF;  // cycle
  MTU.TSTR.BIT.CST4   = CSTn_RUN;   // start to iluminite LED
  MTU.TSTR.BIT.CST0   = CSTn_RUN;   // start to iluminite LED
  //TPUA.TSTR.BIT.CST0   = CSTn_RUN;   // start to iluminite LED
  //TPUA.TSTR.BIT.CST4   = CSTn_RUN;   // start to iluminate LED
  
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
  op_md = OPMD_SAFE;
  uint8_t  status = 0, stat_ppm=0, div = 3;
  uint8_t  flag   = 0;
  uint16_t ppm_val[8], adc_val[21], adc_bat = 0;
  //unsigned char test = 0;

  HMISWInit( &hmi );
  
  while(1) {
    // state machine

    switch( op_md ){
      // SAFE mode; ARM => not ARMed
      //            throttle => zero
      //            Pitch, Roll, Yaw => neutral
    case OPMD_SAFE:
      if( HMILongPress( &hmi, HMI_SW_ARM_KEY, 1200 ) ){
      	op_md = OPMD_RUN_INIT;
      }
      // test
      /* if( HMISWState( &hmi, HMI_SW_ARM_KEY ) ){ */
      /* 	op_md = OPMD_RUN; */
      /* } */

      // ARM: 
      // throttle: 
      
      break;
    case OPMD_RUN_INIT:
      if( !HMILongPress( &hmi, HMI_SW_ARM_KEY, 200 ) ){
      	op_md = OPMD_RUN;
      }
      break;
      // RUN mode; ARM => ARMed
      //           throttle => gimbal
      //           Pitch, Roll, Yaw => gimbal
    case OPMD_RUN:
      // blackbox log on / off
      if( HMISWState( &hmi, HMI_SW_ARM_LOG ) ){
	op_md_log = OPMD_LOG_ON;
      }else{
	op_md_log = OPMD_LOG_OFF;	
      }

      // routine
      
      // end condition ( go to safe mode )
      if( HMILongPress( &hmi, HMI_SW_ARM_KEY, 300 ) ){ op_md = OPMD_SAFE; }
      if( HMILongPress( &hmi, HMI_SW_L1_KEY, 50 ) ){ op_md = OPMD_SAFE; }
      break;
      
      // FAIL mode; ARM => not defined yet...
      //           throttle => zero (plan)
      //           Pitch, Roll, Yaw => neutral (plan)
    case OPMD_FAIL:
      
      break;
    default:
      break;
    }
    HMILEDFlash( &hmi, op_md, op_md_bat, op_md_log );
    /* HMILEDBatLow( &hmi, op_md_bat ); */
    /* HMILEDSetRGB( &hmi, op_md );  */


    // get ADC val & arrange for PPM ch
    // future func => PPMGenInputSelector()
    if( IR( S12AD, S12ADI0) == 1 ){
      //adc_val[0] = (S12AD.ADDR2 >> (2 + 2) ) + (182 * 6); // Roll (old)
      adc_val[0] = (uint16_t)( 1.0*(S12AD.ADDR2 >> (2 + 1) ) - (132 * 6) ); // Roll
      //adc_val[1] = (S12AD.ADDR3 >> (2 + 2) ) + (162 * 6); // Pitch (old NINV)
      //adc_val[1] = (~S12AD.ADDR3 >> (2 + 2) ) +(833 * 6); // Pitch (old)
      adc_val[1] = (uint16_t)( 1.0*(~S12AD.ADDR3 >> (2 + 1) ) + (1168 * 6) ); // Pitch
      //adc_val[2] = (S12AD.ADDR0 >> (2 + 2) ) - (114 * 6); // Throttle (old)
      adc_val[2] = (uint16_t)( 1.0*(S12AD.ADDR0 >> (2 + 1) ) - (224 * 6) ); // Throttle (971 - 1932)
      //adc_val[3] = (S12AD.ADDR1 >> (2 + 2) ) + (209 * 6); // Yaw (NINV)
      //adc_val[3] = (~S12AD.ADDR1 >> (2 + 2) ) + (783 * 6); // Yaw (INV old)
      adc_val[3] = (uint16_t)( 1.0*(~S12AD.ADDR1 >> (2 + 1) ) + (1069 * 6) ); // Yaw (INV)
      adc_val[4] = 0;
      adc_val[5] = 0;
      adc_val[6] = (uint16_t)( 1.0*(S12AD.ADDR5 >> (2 + 1)) ); // AUX3
      adc_val[7] = 0;
      
      adc_bat = (S12AD.ADDR13 >> (2 + 0) );
      IR( S12AD, S12ADI0 ) = 0;
    }

    // future func => PPMGenSafeChecker()  <- safe limiter
    // future func => PPMGenOutputFilter() <- LUT-base conversion
    
    // battery check
    // series res Vbat = Vli2S * (3kOhm / 8kOhm)
    // Vadc_max = 3.3, Vli_max = 8.8 V
    // define Li-ion Vmax:4.2*2 - Vmin: 3.2*2  = 2.0, 
    // res_Vbat = 2.0 / 8.8 = 0.227 ~ 1/4
    // Dad_min = 6.4 * 3/8 / 3.3 * 2^14 = 2.4 *k = 0.727 * 2^14 ~ 730 * 16 = 11680
    // Dad_low = 7.2 * 3/8 / 3.3 * 2^14 = 2.7 *k = 1.1  * 11680  = 12848
    // Dad_mid = 8.0 * 3/8 / 3.3 * 2^14 = 3.0 *k = 1.25 * 11680 = 14600
    // Dad_max = 8.4 * 3/8 / 3.3 * 2^14 = 3.15*k = 0.957 * 2^14 ~ 960 * 16 = 15360
    if( adc_bat < 12900 ){
      op_md_bat = OPMD_BAT_LOW;
    /* }else if( adc_bat < 14600 ){ */
    }else if( adc_bat < 14000 ){
      op_md_bat = OPMD_BAT_MID;
    }else{
      op_md_bat = OPMD_BAT_FULL;
    }

    // Motors disarming check
    //    adc_val[4] = (PORTE.PIDR.BIT.B7 ? 0 : 2049);
    adc_val[4] = ( (op_md == OPMD_RUN) ? ((op_md_log == OPMD_LOG_ON) ? (800 * 6) : (300 * 6) ) : 0 );
    if( op_md != OPMD_RUN ){
      adc_val[2] = 0; // Throttle
    }

    // PPM generation core
    // future func => PPMGen()
    // check tpu renew timing from interrupt occurance flag
    if( IR( TPU3, TGI3A ) == 1 ){
      // tail pulse
      if( TPU3.TGRA > (400 * 6) ){
	TPU3.TGRC = (300 * 6);
	status++;
	if( status > 8 ){ status = 0; }
      }
      // ppm pulse
      else{
	// end pulse
	if( status >= 8 ){
	  TPU3.TGRC = (6200 * 6);
	  hmi.ppm_cnt++;
	  // debug
	  PORTA.PODR.BIT.B0 = !PORTA.PODR.BIT.B0;
	}
	// significant pulse
	else{
	  TPU3.TGRC = (700 * 6) + adc_val[status];
	}
      }
      IR( TPU3, TGI3A ) = 0;
      PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    }


    // HMI routine
    // LED indication (for debug, printf)
    if( ( stat_ppm >> 5 ) & 0x01 ){
      /* MTU4.TGRA = adc_val[0]; // Red */
      /* //TPU0.TGRC = adc_val[1]; // Green */
      /* MTU0.TGRC = adc_val[1]; // Green */
      /* //TPU4.TGRA = adc_val[2]; // Blue */
      /* MTU4.TGRC = adc_val[2]; // Bluefg */
    }

    if( HMISWState( &hmi, HMI_SW_L1_LCK ) ){
    /* if( PORTE.PIDR.BIT.B6 == 0 ){ */
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
  }
}

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  HMI utilities 
//
//  for Inputs
uint8_t HMISWInit( st_HMI *hmi){
    // reset state & time count of pressing
    for( int i=0 ; HMI_N_SW < i ; i++ ){
      hmi->sw_state[i] = 0;
      hmi->sw_cnt[i]   = 0;
    }
    return( 0 );
}

uint8_t HMIScanSW ( st_HMI *hmi ){
  // scan all input channels
  if( IR( CMT0, CMI0 ) ){
    // for debg
    /* hmi->dbg_cnt++; */
    /* if( (hmi->dbg_cnt >> 9) & 0x0001 ){ */
    /*   PORT2.PODR.BIT.B2 = !PORT2.PODR.BIT.B2; */
    /* } */

    // get current states for each SW
    hmi->sw_state[ HMI_SW_ARM_KEY ] = !PORTE.PIDR.BIT.B6;
    hmi->sw_state[ HMI_SW_ARM_LOG ] = !PORTE.PIDR.BIT.B7;
    hmi->sw_state[ HMI_SW_ROT     ] = !PORT5.PIDR.BIT.B2;
    hmi->sw_state[ HMI_SW_L1_KEY  ] = !PORT5.PIDR.BIT.B1;
    hmi->sw_state[ HMI_SW_L1_LCK  ] = !PORT5.PIDR.BIT.B0;
    hmi->sw_state[ HMI_SW_L2_F    ] = !PORTC.PIDR.BIT.B1;
    hmi->sw_state[ HMI_SW_L2_B    ] = !PORTC.PIDR.BIT.B0;
    hmi->sw_state[ HMI_SW_R1_KEY  ] = !PORTE.PIDR.BIT.B4; // ?
    hmi->sw_state[ HMI_SW_R1_LCK  ] = !PORTE.PIDR.BIT.B5; // ?
    hmi->sw_state[ HMI_SW_R2_F    ] = !PORTE.PIDR.BIT.B0; // ?
    hmi->sw_state[ HMI_SW_R2_B    ] = !PORTE.PIDR.BIT.B3; // ?

    // renew time count of pressing
    for( int i=0 ; i < HMI_N_SW ; i++ ){
      if( hmi->sw_state[i] ){
	if( hmi->sw_cnt[i] != 0xFFFF )
	  hmi->sw_cnt[i]++;
      }else{
	if( hmi->sw_cnt[i] != 0 )
	  hmi->sw_cnt[i]-- ;
      }
    }
    
    // rescan
    IR( CMT0, CMI0 ) = 0;
  }
    return( 0 );
}

uint8_t HMISWState( st_HMI *hmi, enum enum_HMI_SW n_sw){
  HMIScanSW( hmi );
  return( hmi->sw_state[n_sw] );
  //(old) using bit operation
  //return( (hmi->sw_state[n_sw] >> n_sw) & 0x0001 );
}

uint8_t HMILongPress( st_HMI *hmi, enum enum_HMI_SW n_sw, uint16_t time ){
  // SW scan
  HMIScanSW( hmi );

  // for debug
  /* if( hmi->sw_cnt[ n_sw ] != 0 ){ */
  /*   PORT2.PODR.BIT.B2 = 1;     */
  /* }else{ */
  /*   PORT2.PODR.BIT.B2 = 0;     */
  /* } */
  /* PORT2.PODR.BIT.B3 = hmi->sw_state[ n_sw ]; */
  
  // long press check ( msec )
  if( hmi->sw_cnt[n_sw] >= time ){
    return( 1 );
  }else{
    return( 0 );
  }
}

uint8_t HMILEDPPMAct( st_HMI *hmi, enum enum_AppModeLog op_md_log, uint8_t div){
  // PPM active indicator
  if( op_md_log == OPMD_LOG_ON ){
    PORT2.PODR.BIT.B3 = ( (hmi->ppm_cnt++ >> div) & 0x01 );
    PORT3.PODR.BIT.B3 = 0;
  }else{
    PORT2.PODR.BIT.B3 = 0;
    PORT3.PODR.BIT.B3 = ( (hmi->ppm_cnt++ >> div) & 0x01 );
  }
  return( 0 );
}

uint8_t HMILEDBatLow( st_HMI *hmi, enum enum_AppModeBat op_md_bat, uint8_t div ){
  hmi->cnt_bat++;  
  // Low battery indicater
  if( op_md_bat == OPMD_BAT_LOW ){
    PORT2.PODR.BIT.B2 = 1;
  }else if( op_md_bat == OPMD_BAT_MID ){
    PORT2.PODR.BIT.B2 = ( (hmi->cnt_bat++ >> div) & 0x01 );
      /* PORT2.PODR.BIT.B2 = !PORT2.PODR.BIT.B2; */
  }else{
    PORT2.PODR.BIT.B2 = 0;
  }
  return( 0 );
}

uint8_t HMILEDSetRGB( st_HMI *hmi, enum enum_AppMode op_md ){
  // RGB mode indicator
  switch( op_md ){
  case OPMD_SAFE:
    MTU4.TGRA = 0x8000;  // Red
    MTU0.TGRC = 0x8800;  // Green
    MTU4.TGRC = 0x8800;  // Blue
    break;
  case OPMD_RUN_INIT:
    MTU4.TGRA = 0x0040;  // Red
    MTU0.TGRC = 0x2B00;  // Green
    MTU4.TGRC = 0x0400;  // Blue
    break;
  case OPMD_RUN:
    MTU4.TGRA = 0x0100;  // Red
    MTU0.TGRC = 0xF000;  // Green
    MTU4.TGRC = 0x1000;  // Blue
    break;
  case OPMD_FAIL:
    MTU4.TGRA = 0xF000;  // Red
    MTU0.TGRC = 0x0800;  // Green
    MTU4.TGRC = 0x0800;  // Blue
    break;
  default:
    MTU4.TGRA = 0xF000;  // Red
    MTU0.TGRC = 0xF000;  // Green
    MTU4.TGRC = 0xF000;  // Blue
    break;
  }
  return( 0 );
}

uint8_t HMILEDFlash( st_HMI *hmi, enum enum_AppMode op_md,
		     enum enum_AppModeBat op_md_bat,
		     enum enum_AppModeLog op_md_log ){
  /* if( IR( CMT1, CMI1 ) ){ */
  if( IR( CMT1, CMI1 ) ){
    /* PORT2.PODR.BIT.B2 = !PORT2.PODR.BIT.B2; */
    /* PORT3.PODR.BIT.B2 = !PORT3.PODR.BIT.B2; */
    HMILEDSetRGB( hmi, op_md );
    HMILEDBatLow( hmi, op_md_bat, 8 );
    HMILEDPPMAct( hmi, op_md_log, 4 );
    // next frame
    /* IR( CMT1, CMI1 ) = 0; */
    IR( CMT1, CMI1 ) = 0;
  }
}

// -------------------------------------------------------
// ------------------------------- Functions ( Utilities )
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
void MPCUnlock( void ){
  MPC.PWPR.BIT.B0WI  = B0WI_UNLOCK;
  MPC.PWPR.BIT.PFSWE = PFSWE_UNLOCK;
}

void MPCLock( void ){
  MPC.PWPR.BIT.PFSWE = PFSWE_LOCK;
  MPC.PWPR.BIT.B0WI  = B0WI_LOCK;
}

void MTU34Unlock( void ){
  if( MTU.TRWER.BIT.RWE == TRWER_RWE_DE ){
    MTU.TRWER.BIT.RWE = TRWER_RWE_EN;
  }
}
