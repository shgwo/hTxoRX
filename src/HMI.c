
#include "iodefine_enum.h"
#include "typedefine.h"
#include "sysutil_RX63N.h"
#include "HMI.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  HMI utilities 
//
//  for 
uint8_t HMISysInTmrInit( void ){
  // CMT0 setting (for HMI input time loop)
  // td_in = 1ms (=1kHz); 12MHz * 4 / 2^9 / 2^(1+5)
  CMT0.CMCR.BIT.CKS = CMT_CKS_PCLK_512;  // PCLK/512;
  CMT0.CMCOR = 0x001F;                   // PCLK / 2^9 / 2^5;
  CMT0.CMCR.BIT.CMIE = CMIE_EN;          // interrupt flag enable
  IEN( CMT0, CMI0 ) = 0;
  IPR( CMT0, CMI0 ) = 7;
}

uint8_t HMISysOutTmrInit( void ){
  // CMT0 setting (for HMI output time loop)
  // td_out = 10ms (=100fps); 12MHz * 2^2 / 2^9 / 2^(1+2+7)
  CMT1.CMCR.BIT.CKS = CMT_CKS_PCLK_512;  // PCLK/512;
  CMT1.CMCOR = 0x03FF;                   // PCLK / 2^9 / 2^10;
  CMT1.CMCR.BIT.CMIE = CMIE_EN;          // interrupt flag enable
  IEN( CMT1, CMI1 ) = 0;
  IPR( CMT1, CMI1 ) = 8;
  // start all HMI loop clock
  CMT.CMSTR0.BIT.STR0 = CMSTRn_RUN;       // CMT_[RUN / STOP]
  CMT.CMSTR0.BIT.STR1 = CMSTRn_RUN;       // CMT_[RUN / STOP]
}

uint8_t HMIPortInInit( void ){
  // PE[0:1] -> Arm SW input
  // ( PE6: SW [ ARM ] (ON),  ON-OFF-(ON) )
  //   PE7: SW [ ARM ] ON,    ON-OFF-(ON) )
  PORTE.PCR.BIT.B6 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B7 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PDR.BIT.B6 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B7 = PDR_IN;         // PDR_IN / PDR_OUT
  PORTE.PMR.BIT.B6 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B7 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  // P1[2,4:5] -> Rotary encoder input
  // ( P12: SW [ KEY ]   ,   )
  //   P14: SW [ ROT_A ] ,   )
  //   P15: SW [ ROT_B ] ,   )
  PORT1.PCR.BIT.B2 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT1.PCR.BIT.B4 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT1.PCR.BIT.B5 = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORT1.PDR.BIT.B2 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT1.PDR.BIT.B4 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT1.PDR.BIT.B5 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT1.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT1.PMR.BIT.B4 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT1.PMR.BIT.B5 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  // pulse count function
  /* MPC.P14PFS.BIT.PSEL  = P14PFS_MTCLKA; */
  /* MPC.P15PFS.BIT.PSEL  = P15PFS_MTCLKB; */
  /* PORT1.PMR.BIT.B4     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC */
  /* PORT1.PMR.BIT.B5     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC   */
  // PC[0:1], P5[0:1] -> reserved SW input
  // ( P52: SW #0 [ LeftLeft ]  (ON) on rotary encoder
  //   P50: SW #1 [ Left ]      ON
  //   P51: SW #1 [ Left ]      (ON)
  //   PC0: SW #2 [ Left mid ]  tail   
  //   PC1: SW #2 [ Left mid ]  head
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
  return( 0 );
}

uint8_t HMIPortInFuncInit( void ){
  // P14, P15 Phase counting mode
  //MTU2a settings ( for Rrotary encoder phase couni )
  MTU.TSTR.BIT.CST1    = CSTn_STOP;           // stop: MTU4
  MTU1.TCR.BIT.CCLR    = CCLR_TGRB;           // TCNT cleared by TGRB
  MTU1.TGRB    = 0x00FF;                      //  (TGRB = FF)
  TPU4.TIOR.BIT.IOA    = IOX_IEDGE;           // Phase counting CLKA
  TPU4.TIOR.BIT.IOB    = IOX_IEDGE;           // Phase counting CLKB
  MTU1.TMDR.BIT.MD     = TMDR_MD_PHC1;        // Phase counting 1 mod
  return( 0 );
}

uint8_t HMIPortOutLEDExtInit( void ){
  // P22,23,P33 -> LED status indicator ( LED extention board )
  // PC[0:1], P5[0:1] -> reserved SW input
  // ( P22: status Pink   (Power)
  //   P23: status Violet (active & blackbox)
  //   P33: status Blue   (active)             )
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

  return( 0 );
}

uint8_t HMIPortOutFuncLEDExtInit( ){
  //MTU2a settings ( for gradational LED, Red )
  MTU.TSTR.BIT.CST4    = CSTn_STOP;           // stop: MTU4
  SysMTU34Unlock();
  //  MTU.TRWER.BIT.RWE    = TRWER_RWE_EN;
  MTU4.TCR.BIT.TPSC    = MTU34_TPSC_PCLK;     // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  MTU4.TCR.BIT.CKEG    = CKEG_PEDGE;          // freq is same as above.
  MTU4.TCR.BIT.CCLR    = CCLR_TGRB;           // TCNT cleared by TGRB
  MTU4.TMDR.BIT.MD     = TMDR_MD_PWM1;        // PWM1 mode
  MTU4.TMDR.BIT.BFA    = TMDR_BFx_NORM;       // normal operation
  MTU4.TMDR.BIT.BFB    = TMDR_BFx_NORM;       // normal operation
  MTU4.TIORH.BIT.IOA   = IOX_OLCH;            // MTIOCnA
  MTU4.TIORH.BIT.IOB   = IOX_OLCL;            // MTIOCnA(PWM1)
  MTU4.TIORL.BIT.IOC   = IOX_OLCH;            // disable
  MTU4.TIORL.BIT.IOD   = IOX_OLCL;            // disable
  MTU.TOER.BIT.OE3B    = OEny_DE;             // OEny_[DE/EN] MTIOC3B
  MTU.TOER.BIT.OE4A    = OEny_EN;             // OEny_[DE/EN] MTIOC4A
  MTU.TOER.BIT.OE4B    = OEny_DE;             // OEny_[DE/EN] MTIOC4B
  MTU.TOER.BIT.OE3D    = OEny_DE;             // OEny_[DE/EN] MTIOC3D
  MTU.TOER.BIT.OE4C    = OEny_EN;             // OEny_[DE/EN] MTIOC4C
  MTU.TOER.BIT.OE4D    = OEny_DE;             // OEny_[DE/EN] MTIOC4D
  
  //TPUa settings ( for gradational LED, Blue )
  TPUA.TSTR.BIT.CST4   = CSTn_STOP;            // stop: TPU04
  TPU4.TCR.BIT.TPSC    = TPU410_TPSC_PCLK;     // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  TPU4.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // freq is same as above.
  TPU4.TCR.BIT.CCLR    = TPU_CCLR_TGRB;        // TCNT cleared by TGRA
  TPU4.TMDR.BIT.MD     = TPU_MD_PWM1;          // PWM1 mode
  TPU4.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  TPU4.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  TPU4.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;    // ch B (unused)
  TPU4.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;    // ch D (unused)
  TPU4.TIOR.BIT.IOA    = IOX_OLCH;             // TIOCAn
  TPU4.TIOR.BIT.IOB    = IOX_OLCL;             // disable
  TPU4.TIER.BIT.TGIEA  = TGIEX_DE;             // IRQ enable (temp DE)
  TPU4.TIER.BIT.TTGE   = TTGE_DE;              // disable: ADC start
  
  //TPUa settings ( for gradational LED, Green )
  TPUA.TSTR.BIT.CST0   = CSTn_STOP;            // stop: TPU0
  TPU0.TCR.BIT.TPSC    = TPU06_TPSC_PCLK;      // (12Mhz x 4) / 1 -> 2*2*3M/2^8 = ~48MHz
  TPU0.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // freq is same as above.
  TPU0.TCR.BIT.CCLR    = TPU_CCLR_TGRD;        // TCNT cleared by TGRC
  TPU0.TMDR.BIT.MD     = TPU_MD_PWM1;          // PWM1 mode
  TPU0.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  TPU0.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  TPU0.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;    // ch B (unused)
  TPU0.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;    // ch D (unused)
  TPU0.TIORH.BIT.IOA   = IOX_DE;               // disable
  TPU0.TIORH.BIT.IOB   = IOX_DE;               // disable
  TPU0.TIORL.BIT.IOC   = IOX_OLCH;             // TIOCCn
  TPU0.TIORL.BIT.IOD   = IOX_OLCL;             // disable
  TPU0.TIER.BIT.TGIEA  = TGIEX_DE;             // IRQ enable (temp DE)
  TPU0.TIER.BIT.TTGE   = TTGE_DE;              // disable: ADC start

  MTU0.TCR.BIT.TPSC    = MTU0_TPSC_PCLK;       // (12Mhz x 4) / 1 -> 2*3M * 2^2 = ~48MHz
  MTU0.TCR.BIT.CKEG    = CKEG_PEDGE;           // freq is same as above.
  MTU0.TCR.BIT.CCLR    = CCLR_TGRD;            // TCNT cleared by TGRB
  MTU0.TMDR.BIT.MD     = TMDR_MD_PWM1;         // PWM1 mode
  MTU0.TMDR.BIT.BFA    = TMDR_BFx_NORM;        // normal operation
  MTU0.TMDR.BIT.BFB    = TMDR_BFx_NORM;        // normal operation
  MTU0.TIORH.BIT.IOA   = IOX_DE;               // disable
  MTU0.TIORH.BIT.IOB   = IOX_DE;               // disable
  MTU0.TIORL.BIT.IOC   = IOX_OLCH;             // MTIOCnC
  MTU0.TIORL.BIT.IOD   = IOX_OLCL;             // MTIOCnA(PWM1)
  return( 0 );
}

uint8_t HMILEDExtInit( struct st_HMI *hmi ){
  // Timer driver initialize
  HMISysOutTmrInit();
  // Port & Func module driver initialization
  HMIPortOutLEDExtInit();
  HMIPortOutFuncLEDExtInit();

  // LED gradation (initial value)
  // Red
  MTU4.TGRA = 0x3FFF;  // duty
  MTU4.TGRB = 0xFFFF;  // cycle
  // Blue
  MTU4.TGRC = 0x0FFF;  // duty
  MTU4.TGRD = 0xFFFF;  // cycle
  /* TPU4.TGRA = 0x1100;  // duty */
  /* TPU4.TGRB = 0xF000;  // cycle */
  /* MTU0.TGRC = 0xF010;  // duty */
  /* MTU0.TGRD = 0xFFF0;  // cycle */
  // Green
  MTU0.TGRC = 0x3FFF;  // duty
  MTU0.TGRD = 0xFFFF;  // cycle
  MTU.TSTR.BIT.CST4   = CSTn_RUN;   // start to iluminite LED ()
  MTU.TSTR.BIT.CST0   = CSTn_RUN;   // start to iluminite LED
  //TPUA.TSTR.BIT.CST0   = CSTn_RUN;   // start to iluminite LED
  //TPUA.TSTR.BIT.CST4   = CSTn_RUN;   // start to iluminate LED
  return ( 0 );
}


uint8_t HMIPortOutSndInit( void ){
  // P13 -> Buzzer ( Buzzer extention board )
  PORT1.PODR.BIT.B3  = 1;
  PORT1.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT1.PDR.BIT.B3 = PDR_OUT;        // PDR_IN / PDR_OUT
  MPC.P13PFS.BIT.PSEL  = P13PFS_MTIOC0B;
  MPC.P13PFS.BIT.PSEL  = P13PFS_TIOCA5;
  PORT1.PMR.BIT.B3     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  return( 0 );
}

uint8_t HMIPortOutFuncSndInit( ){
  //TPUa settings ( for Piezoelectric Sounder )
  TPUA.TSTR.BIT.CST5   = CSTn_STOP;             // stop: TPU5
  TPU5.TCR.BIT.TPSC    = TPU511_TPSC_PCLK_256;  // (12Mhz x 4) / 256 -> 3M/2^4 = ~150kHz
  TPU5.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;   // freq is same as above.
  TPU5.TCR.BIT.CCLR    = TPU_CCLR_TGRD;         // TCNT cleared by TGRC
  TPU5.TMDR.BIT.MD     = TPU_MD_PWM1;           // PWM1 mode
  TPU5.TMDR.BIT.BFA    = TMDR_BFx_NORM;         // normal operation
  TPU5.TMDR.BIT.BFB    = TMDR_BFx_NORM;         // normal operation
  TPU5.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;     // ch B (unused)
  TPU5.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;     // ch D (unused)
  TPU5.TIOR.BIT.IOA    = IOX_OHCL;                // PWM1 duty
  TPU5.TIOR.BIT.IOB    = IOX_OHCH;                // PWM1 cycle
  TPU5.TIER.BIT.TGIEA  = TGIEX_DE;              // IRQ disable
  TPU5.TIER.BIT.TTGE   = TTGE_DE;               // enable: ADC start
  return( 0 );
}

uint8_t HMISndInit( st_HMI *hmi ){
  hmi->snd_state_seq = 0;
  hmi->snd_state_fb  = 0;
  hmi->snd_cnt_fb    = 0;
  // Timer driver initialize
  HMISysOutTmrInit();
  // Sounder driver initialize
  HMIPortOutSndInit();
  HMIPortOutFuncSndInit();

  // Sounder generation
  // Max sound freq = 40kHz
  // Min pulse width = 10% duty
  // requirements: time resolution none ( in situ )
  // calc freq:  12M * 4 / 2^8 = ~150kHz
  TPU5.TGRA            = 0x0010;     // duty 300u * 6.0M = 1800 ()
  TPU5.TGRB            = 0x0040;     // 12M * 4 / 2^8 / 2^6 = ~3kHz
  IEN( TPU5, TGI5A ) = 0;
  IPR( TPU5, TGI5A ) = 4;
  //TPUA.TSTR.BIT.CST5   = CSTn_RUN;   // start to run timer pulse
}


//  for Inputs
uint8_t HMISWInit( st_HMI *hmi){
  // Timer driver initialize
  HMISysInTmrInit();
  // Switch input driver initialize
  HMIPortInInit();
  HMIPortInFuncInit();
  
  // reset state & time count of pressing
  for( int i=0 ; HMI_N_SW < i ; i++ ){
    hmi->sw_state[i] = 0;
    hmi->sw_cnt[i]   = 0;
  }
  MTU.TSTR.BIT.CST1    = CSTn_RUN;  // run: MTU1 (phase count)    
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

    // copy old states for each SW
    for ( int i=0 ; i < HMI_N_SW ; i++ ){
      hmi->sw_state_old[ i ] = hmi->sw_state[ i ];
    }

    // get current states for each SW
    hmi->sw_state[ HMI_SW_ARM_KEY ] = !PORTE.PIDR.BIT.B6;
    hmi->sw_state[ HMI_SW_ARM_LOG ] = !PORTE.PIDR.BIT.B7;
    hmi->sw_state[ HMI_SW_ROT     ] = !PORT1.PIDR.BIT.B2;
    hmi->sw_state[ HMI_SW_ROT_A   ] = !PORT1.PIDR.BIT.B4;
    hmi->sw_state[ HMI_SW_ROT_B   ] = !PORT1.PIDR.BIT.B5;
    hmi->sw_state[ HMI_SW_ROT_CNT ] = MTU1.TCNT;
    hmi->sw_state[ HMI_SW_L1_KEY  ] = !PORT5.PIDR.BIT.B1;
    hmi->sw_state[ HMI_SW_L1_LCK  ] = !PORT5.PIDR.BIT.B0;
    hmi->sw_state[ HMI_SW_L2_F    ] = !PORTC.PIDR.BIT.B1;
    hmi->sw_state[ HMI_SW_L2_B    ] = !PORTC.PIDR.BIT.B0;
    hmi->sw_state[ HMI_SW_R1_KEY  ] = !PORTE.PIDR.BIT.B4; // ?
    hmi->sw_state[ HMI_SW_R1_LCK  ] = !PORTE.PIDR.BIT.B5; // ?
    hmi->sw_state[ HMI_SW_R2_F    ] = !PORTE.PIDR.BIT.B0; // ?
    hmi->sw_state[ HMI_SW_R2_B    ] = !PORTE.PIDR.BIT.B3; // ?

    // transition check
    for ( int i=0 ; i < HMI_N_SW ; i++ ){
      if( hmi->sw_state_old[ i ] != hmi->sw_state[ i ] ){
	hmi->snd_state_fb = 1;
	hmi->LED_RGB_state_fb = 1;
      }
    }
    
    // renew time count of pressing
    for( int i=0 ; i < HMI_N_SW ; i++ ){
      // press -> count up
      if( hmi->sw_state[i] ){
	if( hmi->sw_cnt[i] != 0xFFFF )
	  hmi->sw_cnt[i]++;

      }else{
	//unpress -> count down
	if( hmi->sw_cnt[i] != 0 )
	  hmi->sw_cnt[i]-- ;
      }
    }
    
    // set rescan
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

uint8_t HMILEDSetRGB( st_HMI *hmi ){
  MTU4.TGRA = hmi->LED_RGB[HMI_LED_R];  // Red
  MTU0.TGRC = hmi->LED_RGB[HMI_LED_G];  // Green
  MTU4.TGRC = hmi->LED_RGB[HMI_LED_B];  // Blue
  //  TPU0.TGRC = hmi->LED_RGB[HMI_LED_B];  // Blue
  return( 0 );
}

uint8_t HMILEDFBRGB( st_HMI *hmi ){
  // RGB LED feed back operation
  if( hmi->LED_RGB_state_fb ){
    hmi->LED_RGB[HMI_LED_R] = 0xAF00;  // Red
    hmi->LED_RGB[HMI_LED_G] = 0xAF00;  // Green
    hmi->LED_RGB[HMI_LED_B] = 0x00FF;  // Blue
    if( hmi->LED_RGB_cnt_fb++ > HMI_LED_FB_LEN  ){
      hmi->LED_RGB_cnt_fb = 0;
      hmi->LED_RGB_state_fb = 0;
    }
  }
  return( 0 );
}

uint8_t HMILEDSetRGBMode( st_HMI *hmi, enum enum_AppMode op_md ){
  // RGB mode indicator
  switch( op_md ){
  case OPMD_SAFE:
    hmi->LED_RGB[HMI_LED_R] = 0x7000;  // Red
    hmi->LED_RGB[HMI_LED_G] = 0x8F00;  // Green
    hmi->LED_RGB[HMI_LED_B] = 0x8F00;  // Blue
    break;
  case OPMD_RUN_INIT:
    hmi->LED_RGB[HMI_LED_R] = 0x0040;  // Red
    hmi->LED_RGB[HMI_LED_G] = 0x2B00;  // Green
    hmi->LED_RGB[HMI_LED_B] = 0x0400;  // Blue
    break;
  case OPMD_RUN:
    hmi->LED_RGB[HMI_LED_R] = 0x0100;  // Red
    hmi->LED_RGB[HMI_LED_G] = 0xF000;  // Green
    hmi->LED_RGB[HMI_LED_B] = 0x1000;  // Blue
    break;
  case OPMD_FAIL:
    hmi->LED_RGB[HMI_LED_R] = 0xF000;  // Red
    hmi->LED_RGB[HMI_LED_G] = 0x0800;  // Green
    hmi->LED_RGB[HMI_LED_B] = 0x0800;  // Blue
    break;
  default:
    hmi->LED_RGB[HMI_LED_R] = 0x0F00;  // Red
    hmi->LED_RGB[HMI_LED_G] = 0x0F00;  // Green
    hmi->LED_RGB[HMI_LED_B] = 0x0F00;  // Blue
    break;
  }
  HMILEDFBRGB( hmi );
  HMILEDSetRGB( hmi );
  return( 0 );
}

uint8_t HMISndTune( st_HMI *hmi ){
  // Sounder sound tuning
  // Max sound freq = 40kHz
  // Min pulse width = 10% duty
  // requirements: time resolution none ( in situ )
  // calc freq:  12M * 4 / 2^8 = ~150kHz
  TPU5.TGRA            = 0x0010;     // cant controll (capacitive sounder) 
  TPU5.TGRB            = 0x0040;     // 12M * 4 / 2^8 / 2^6 = ~3kHz
}

uint8_t HMISndSeqGen( st_HMI *hmi,
		      enum enum_AppModeBat op_md_bat ){  
  return( 0 );  
}

uint8_t HMISndFB( st_HMI *hmi ){
  if( hmi->snd_state_seq ){
  }else{
    // Sound feed back operation
    if( hmi->snd_state_fb ){
      TPUA.TSTR.BIT.CST5   = CSTn_RUN;   // start to beep
      if( hmi->snd_cnt_fb++ > HMI_SND_FB_LEN  ){
	TPUA.TSTR.BIT.CST5   = CSTn_STOP;   // stop to beep
	hmi->snd_cnt_fb = 0;
	hmi->snd_state_fb = 0;
      }
    }
  }
  return( 0 );
}


uint8_t HMIFlash( st_HMI *hmi, enum enum_AppMode op_md,
		     enum enum_AppModeBat op_md_bat,
		     enum enum_AppModeLog op_md_log ){
  /* if( IR( CMT1, CMI1 ) ){ */
  if( IR( CMT1, CMI1 ) ){
    // LED loop
    /* PORT2.PODR.BIT.B2 = !PORT2.PODR.BIT.B2; */
    /* PORT3.PODR.BIT.B2 = !PORT3.PODR.BIT.B2; */
    HMILEDSetRGBMode( hmi, op_md );
    HMILEDBatLow( hmi, op_md_bat, 8 );
    HMILEDPPMAct( hmi, op_md_log, 4 );

    // Sounder loop
    HMISndSeqGen( hmi, op_md_bat );
    HMISndFB( hmi );

    // next frame
    IR( CMT1, CMI1 ) = 0;
  }
}