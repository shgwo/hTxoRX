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
#include "sysutil_RX63N.h"
#include "hTxoRX.h"
#include "ppm_gen.h"
#include "HMI.h"
#include "serial.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define sleep(X) for(j = 0; j < X*1000; j++) {}


#define NCH_ADC 21   // number of AD channel on the HW

// -------------------------------------------------------
// -------------------------------------- global variables
st_PPMAdj ppm_adj[NCH_PPM];
st_HMI    hmi;
st_Serial ser[UART_N_APP];

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
void DataDisp( unsigned char );

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
  SysClkInit();
  // distribute CLK to all module
  //SYSTEM.MSTPCRA.LONG = 0x00000000;
  //SYSTEM.MSTPCRB.LONG = 0x00000000;
  //SYSTEM.MSTPCRC.LONG = 0x00000000;

  // select using module
  SysMdlStopInit();

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
  SysMPCUnlock();
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

  // LED extention board
  HMILEDExtInit( &hmi );
  // Piezo-sounder extention board
  HMISndInit( &hmi );
  // Switches
  HMISWInit( &hmi );

  // P4 -> ADC input from potentio meters in stick gimbal (Vref 3.3V)
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

  // PC2,3 UART Main IF
  //SerInit( &ser[UART_MSP], UART_BRATE_MSP, SER_HOST );
  // PE1,2 UART Telemetry IF
  //Ser12Init( &ser[UART_TELEMETRY], UART_BRATE_TELEM );

  //relock MPC
  SysMPCLock();

  
  // TPUa setting (for PPM)
  TPUA.TSTR.BIT.CST3   = CSTn_STOP;           // stop: TPU3
  TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_16;  // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz
  TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE;       // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz
  //TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_4;   // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz (high-reso test)
  //TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz (high-reso test)
  TPU3.TCR.BIT.CCLR    = TPU_CCLR_TGRA;       // TCNT cleared by TGRA
  TPU3.TMDR.BIT.MD     = TPU_MD_NORM;         // normal mode
  TPU3.TMDR.BIT.BFA    = TMDR_BFx_BUFF;       // buffer operation
  TPU3.TMDR.BIT.BFB    = TMDR_BFx_NORM;       // buffer operation
  TPU3.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;   // ch B (unused)
  TPU3.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;   // ch D (unused)
  TPU3.TIORH.BIT.IOA   = IOX_OHCT;            // TIOCAn
  TPU3.TIORH.BIT.IOB   = IOX_DE;              // disable
  TPU3.TIORL.BIT.IOC   = IOX_DE;              // disable
  TPU3.TIORL.BIT.IOD   = IOX_DE;              // disable
  TPU3.TIER.BIT.TGIEA  = TGIEX_EN;            // IRQ enable (temp DE)
  TPU3.TIER.BIT.TTGE   = TTGE_EN;             // enable: ADC start

  
  // settings for gradational LED, Red / Green / Blue
  /* HMIPortOutFuncLEDExtInit( ); */
  // setings for Piezoelectric Sounder
  /* HMIPortOutFuncSndInit( ); */

  // setting for Rotary encoder
  /* HMIPortInFuncInit( ); */
  
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
  // ADC config
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
  
  // PPM generation
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  //   -> 8 times { H: init(0.7m) + variable(0 - 1.0 m) + stop tail(0.3m)
  //   -> frame tail (H: 6.2m L: 0.3m)
  //   -> Go next frame (8 times) repetitively ...
  // requirements: time resolution < 1ms / 11bit   = 1m / 2^11 =~ 0.5u,
  //               max time length > 1ms           
  // calc time resolution:  1/(48M / 2^3) = 1/6 u = 166...ns
  // calc max time length:  2^16 / (48M / 2^3) = 1/(3*2^4) 2^19 us = 1/3 * 2^5 *1024 us = 32/3 * 1024 us = ~10ms
  TPU3.TGRA            = 4200;     //  700u * 6.0M = 4200 ()
  TPU3.TGRC            = 1800;     //  300u * 6.0M = 1800 ()
  TPU3.TGRD            = 37200;    // 6200u * 6.0M = 37200 ()
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
  char ppm_chname[NCH_PPM][10] = { "Roll", "Pitch", "Throttle", "Yaw", "Arm", "AUX2", "FiteMD", "AUX4" };
  // set adj ( name str, ad_ch, inv, offset, gain )
  PPMGenAdjInit( &ppm_adj[0], ppm_chname[0],  2, PPMADJ_NOINV,  (182 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[1], ppm_chname[1],  3, PPMADJ_INV,    (162 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[2], ppm_chname[2],  0, PPMADJ_NOINV, -(114 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[3], ppm_chname[3],  1, PPMADJ_INV,    (783 * 6), 1.0 );
  PPMGenAdjInit( &ppm_adj[4], ppm_chname[4], 22, PPMADJ_NOINV,          0, 1.0 );
  PPMGenAdjInit( &ppm_adj[5], ppm_chname[5], 22, PPMADJ_NOINV,          0, 1.0 );
  PPMGenAdjInit( &ppm_adj[6], ppm_chname[6], 22, PPMADJ_NOINV,          0, 1.0 );
  PPMGenAdjInit( &ppm_adj[7], ppm_chname[7], 22, PPMADJ_NOINV,          0, 1.0 );

  sleep( 1000 );
  
  // 
  // Main routine start
  //
  // Init
  op_md = OPMD_SAFE;
  uint8_t  status = 0, stat_ppm=0, div = 3;
  uint8_t  flag   = 0;
  uint16_t ppm_val[8], adc_val[21], adc_bat = 0;
  //unsigned char test = 0;

  HMISWInit( &hmi );

  // main loop
  while(1) {
    // state machine

    switch( op_md ){
      // SAFE mode; ARM => not ARMed
      //            throttle => zero
      //            Pitch, Roll, Yaw => neutral
    case OPMD_SAFE:
      op_md_log = OPMD_LOG_OFF;
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
    HMIFlash( &hmi, op_md, op_md_bat, op_md_log );
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
/* void SysCoreLock( void ){ */
/*   SYSTEM.PRCR.BIT.PRC0  = PRCn_LOCK;   // clk gen: SCKCR / SCKCR2 / SCKCR3 / PLLCR / */
/*                                        //          PLLCR2 / BCKCR / MOSCCR / SOSCCR / */
/*                                        //          LOCOCR / ILOCOCR / HOCOCR / OSTDC(S)R */
/*   SYSTEM.PRCR.BIT.PRC1  = PRCn_LOCK;   // operating modes: SYSCR0 / SYSCR1 */
/*                                        // low pow modes:   SBYCR / MSTPCRA / MSTPCRB / */
/*                                        //                  MSTPCRC / MSTPCRD / OPCCR / */
/*                                        //                  RSTCKCR / MOSCWTCR / SOSCWTCR / */
/*                                        //                  PLLWTCR / DPSBYCR / DPSIER0-3 / */
/*                                        //                  DPSIFR0-3 / DPSIEGR0-3 */
/*                                        // clk gen: MOFCR / HOCOPCR */
/*                                        // reset:   SWRR */
/*   SYSTEM.PRCR.BIT.PRC3  = PRCn_LOCK;   // lvd:     LVCMPCR / LVDLVLR / LVD1CR0 / LVD1CR1 / */
/*                                        //          LVD1SR / LVD2CR0 / LVD2CR1 / LVD2SR */
/*   SYSTEM.PRCR.BIT.PRKEY = PRKEY_LOCK; */
/*   SYSTEM.PRCR.WORD = 0x0000; */
/* } */

/* end of main.c */
