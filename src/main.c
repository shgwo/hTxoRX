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
#include "HMI.h"
#include "ppm_gen.h"
#include "serial.h"
#include "adc_sar12b.h"
#include "dbg_utils.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define sleep(X) for(j = 0; j < X*1000; j++) {}

//#define NCH_ADC 21   // number of AD channel on the HW

// -------------------------------------------------------
// -------------------------------------- global variables
st_ADC12  adc12;
st_PPM    ppm;
st_PPMAdj ppm_adj[PPM_N_CH];
st_HMI    hmi;
st_Serial ser[UART_N_APP];
st_hTx    htx;
// -------------------------------------------------------
// -------------------------------- Proto-type declaration
void DataDisp( unsigned char );

// -------------------------------------------------------
// ------------------------------------------ Main routine  
int main( void )
{
  char i=0;
  int j, state_disp=0;
  hTxInit( &htx );

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

  // IO & Peri Port setting
  SysMPCUnlock();
  // LED extention board  
  HMILEDExtInit( &hmi );
  // Piezo-sounder extention board
  HMISndInit( &hmi );
  // Switches
  HMISWInit( &hmi );
  // ADC input (Gimbal stick input)
  ADC12Init();
  // PPM output
  PPMGenInit();
  // PC2,3 UART Main IF
  SerInit( &ser[UART_MSP], UART_BRATE_MSP, SER_MSP );
  // PE1,2 UART Telemetry IF
  SerInit( &ser[UART_TELEM], UART_BRATE_TELEM, SER_TELEM );  
  /* Ser12Init( &ser[UART_TELEM], UART_BRATE_TELEM ); */
  
  // test setting
  // PE1, PE2 -> UART Port setting (Rx/Tx)
  /* PORTE.PODR.BIT.B1  = 0; */
  /* PORTE.PCR.BIT.B1   = PCR_OPEN;         // PCR_OPEN / PCR_PULLUP */
  /* PORTE.PCR.BIT.B2   = PCR_PULLUP;       // PCR_OPEN / PCR_PULLUP */
  /* PORTE.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC */
  /* PORTE.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC */
  /* PORTE.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT */
  /* PORTE.PDR.BIT.B2   = PDR_IN;           // PDR_IN / PDR_OUT */

  // on-board LED
  dbgDispLEDInit();
  
  //relock MPC
  SysMPCLock();

  // setting for Rotary encoder
  /* HMIPortInFuncInit( ); */  

  // initializing PPM ch val ( * future func *)
  // Name labels for each PPM ch
  char ppm_chname[PPM_N_CH][10] = { "Roll", "Pitch", "Throttle", "Yaw", "Arm", "AUX2", "FiteMD", "AUX4" };
  // set adj ( name str, ad_ch, inv, offset, gain )
  PPMGenAdjInit( &ppm_adj[0], ppm_chname[0],  2, PPMADJ_NOINV,  (182 * 6), 1.0 ); // Roll
  PPMGenAdjInit( &ppm_adj[1], ppm_chname[1],  3, PPMADJ_INV,    (162 * 6), 1.0 ); // Pitch
  PPMGenAdjInit( &ppm_adj[2], ppm_chname[2],  0, PPMADJ_NOINV, -(114 * 6), 1.0 ); // Throttle
  PPMGenAdjInit( &ppm_adj[3], ppm_chname[3],  1, PPMADJ_INV,    (783 * 6), 1.0 ); // Yaw
  PPMGenAdjInit( &ppm_adj[4], ppm_chname[4], 22, PPMADJ_NOINV,          0, 1.0 ); // Arm
  PPMGenAdjInit( &ppm_adj[5], ppm_chname[5], 22, PPMADJ_NOINV,          0, 1.0 ); // AUX2
  PPMGenAdjInit( &ppm_adj[6], ppm_chname[6], 22, PPMADJ_NOINV,          0, 1.0 ); // FliteMD
  PPMGenAdjInit( &ppm_adj[7], ppm_chname[7], 22, PPMADJ_NOINV,          0, 1.0 ); // AUX4

  PPMGenStart();
  
  // 
  // Main routine start
  //
  // Init
  uint8_t  status = 0, stat_ppm=0, div = 3;
  uint8_t  flag   = 0;
  uint16_t ppm_val[8], adc_val[21], adc_bat = 0;
  uint8_t  mode_test = 0xAA;
  uint8_t  dbg = 0x00, test = 0x00;
  //unsigned char test = 0;
  
  hTxSetMode( &htx, OPMD_SAFE);
  //SerWrite( &ser[UART_MSP], 0xA1 );  
  sleep( 1000 );

  // main loop
  while(1) {
    // state machine

    switch( htx.opmd ){
      // SAFE mode; ARM => not ARMed
      //            throttle => zero
      //            Pitch, Roll, Yaw => neutral
    case OPMD_SAFE:
      hTxSetModeLogOff( &htx );
      if( HMILongPress( &hmi, HMI_SW_ARM_KEY, 1200 ) ){
	hTxSetMode( &htx, OPMD_RUN_INIT );
	SerWrite( &ser[UART_MSP], 'r' );
      }
      // test
      /* if( HMISWState( &hmi, HMI_SW_ARM_KEY ) ){ */
      /* 	op_md = OPMD_RUN; */
      /* } */
      // ARM: 
      // throttle: 
      break;
    case OPMD_RUN_INIT:
      // RUN_INIT mode; transition to Run mode
      //            ARM => not ARMed
      //            throttle => zero
      //            Pitch, Roll, Yaw => neutral
      if( !HMILongPress( &hmi, HMI_SW_ARM_KEY, 100 ) ){
	hTxSetMode( &htx, OPMD_RUN );
	SerWrite( &ser[UART_MSP], 'T' );
	SerBytesWrite( &ser[UART_MSP], "trn to RUN.." );
      }
      break;
      // RUN mode; ARM => ARMed
      //           throttle => gimbal
      //           Pitch, Roll, Yaw => gimbal
    case OPMD_RUN:
      // blackbox log on / off
      if( HMILongPress( &hmi, HMI_SW_ARM_LOG, 100 ) ){
	hTxSetModeLogOn( &htx );
	SerBytesWrite( &ser[UART_MSP], "RUN: Log on" );
      }else{
	hTxSetModeLogOff( &htx );
	SerBytesWrite( &ser[UART_MSP], "run: Log off" );
      }

      // routine
      
      // end condition ( go to safe mode )
      if( HMILongPress( &hmi, HMI_SW_ARM_KEY, 300 ) ){ hTxSetMode( &htx, OPMD_SAFE);  SerWrite( &ser[UART_MSP], 'S' ); }
      if( HMILongPress( &hmi, HMI_SW_L1_KEY, 50 ) ){ hTxSetMode( &htx, OPMD_SAFE);  SerWrite( &ser[UART_MSP], 'S' ); }
      // future func: stick disarm()
      break;
      
      // FAIL mode; ARM => not defined yet...
      //           throttle => zero (plan)
      //           Pitch, Roll, Yaw => neutral (plan)
    case OPMD_FAIL:
      break;
    default:
      break;
    }

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
    // future func => PPMGenOutputFilter() <- LUT-base / Filt-matrix  conversion
    
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
      htx.opmd_bat = OPMD_BAT_LOW;
    /* }else if( adc_bat < 14600 ){ */
    }else if( adc_bat < 14000 ){
      htx.opmd_bat = OPMD_BAT_MID;
    }else{
      htx.opmd_bat = OPMD_BAT_FULL;
    }

    // Motors disarming check
    //    adc_val[4] = (PORTE.PIDR.BIT.B7 ? 0 : 2049);
    adc_val[4] = ( (htx.opmd == OPMD_RUN) ? ((htx.opmd_log == OPMD_LOG_ON) ? 0 : (100 * 6) ) : (200 * 6) );
    if( htx.opmd != OPMD_RUN ){
      adc_val[2] = 0; // Throttle
    }

    // PPM generation core
    // future func => PPMGen()
    // PPMGen( st_PPM );
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

    // Back gound processes
    HMIFlash( &hmi, &htx );
    SerWriteBG( &ser[UART_MSP] );
    SerReadBG( &ser[UART_MSP] );
    SerReadBG( &ser[UART_TELEM] );
    //if( Ser12ReadBG( &ser[UART_TELEM] ); ){
      // error sequence
      
    //}
    //SerWriteTest( &ser[UART_MSP], mode_test );
    /* HMILEDBatLow( &hmi, op_md_bat ); */
    /* HMILEDSetRGB( &hmi, op_md );  */
    
    // HMI routine
    // LED indication (for debug, printf)

    if( HMISWState( &hmi, HMI_SW_R2_F ) ){
      /* // GPIO telem pin check */
      /* if( PORTE.PIDR.BIT.B2 ){ */
      /* 	SerWrite( &ser[UART_MSP], '1'); */
      /* }else{ */
      /* 	SerWrite( &ser[UART_MSP], '0'); */
      /* } */
      //dbg = 0x30 + SerRead( &ser[UART_TELEM] ) % 10;
      /* dbg = SerRead( &ser[UART_TELEM] ); */
      /* SerWrite( &ser[UART_MSP], dbg ); */
      dbg = SerRead( &ser[UART_MSP]);
      test = SerReadTest( &ser[UART_TELEM] );
      SerWrite( &ser[UART_MSP], test);
      /* 	SerBytesWrite( &ser[UART_MSP], (0x30 + (ser[UART_MSP].rx_head % 10)) ); */
      /* } */
      if( dbg = SerRead( &ser[UART_TELEM] ) ){
	SerWrite( &ser[UART_MSP], (0x30 + (ser[UART_TELEM].rx_head % 10)) );
      }
      //SerWrite( &ser[UART_MSP], (0x30 + (ser[UART_TELEM].rx_tail % 10)) );
      //SerWrite( &ser[UART_TELEM], (0x30 + (ser[UART_TELEM].rx_tail % 10)) );
      /* SerWrite( &ser[UART_MSP], ser[UART_TELEM].rx_head ); */
     
	/* unsigned char test = ( S12AD.ADDR13 >> 12 ); */

      /* if( (hmi.ppm_cnt >> 4) & 0x01 ){ */
      /* 	//	dbg = Ser12ReadTest( &ser[UART_TELEM] ); */
      /* 	//	dbg = PORTE.PIDR.BYTE; */
      /* } */
      /* if( hmi.ppm_cnt == 0x00 ){ */
      /* 	( dbg = Ser12Read( &ser[UART_TELEM] ) ?  : dbg = 'x' ); */
      /* /\* 	/\\* SerWriteTest( &ser[UART_MSP], '.' ); *\\/ *\/ */
      /* /\* 	/\\* SerWrite( &ser[UART_MSP], 'T' ); *\\/ *\/ */
      /* /\* 	//SerBytesWrite( &ser[UART_MSP], ". " ); *\/ */
      /* /\* 	dbg = 0x30 + (dbg++ % 10); *\/ */
      /* 	SerWrite( &ser[UART_MSP], dbg ); */
      /* /\* 	//SerBytesWrite( &ser[UART_MSP], sprintf( "0x%x", dbg++ ); ); *\/ */
      /* } */
      /* else{ dbg = 0; } */
      //test = Ser12Read( &ser[UART_TELEM] );
      dbgDispLEDbit( IR(SCI5, TEI5), 3 );
      dbgDispLEDbit( IR(SCI5, TXI5), 2 );
      //      dbgDispLEDbit( IR(SCI12, ERI), 1 );
      /* dbgDispLEDbit( IR(SCI5, RXI5), 1 ); */
      if( (hmi.ppm_cnt >> 7) & 0x01 ){
	dbgDispLEDbit( (ser[UART_MSP].rx_tail >> 4), 1);
	dbgDispLEDbit( (ser[UART_MSP].rx_tail >> 4), 1);
      }
      /* dbgDispLEDbit( (ser[UART_TELEM].rx_tail >> 5), 1 ); */
      //dbgDispLEDbit( (hmi.ppm_cnt >> 2), 0 );
      /* dbgDispLED( hmi.ppm_cnt ); */
    }
    else if( PORTE.PIDR.BIT.B7 == 0 ){
      /* Disp LED according to AN0 */
      unsigned char test = MPC.P21PFS.BIT.PSEL;
      test = SYSTEM.SCKCR3.BIT.CKSEL;
      test = ( S12AD.ADDR0 >> 12 );
      test = (adc_bat >> 9);
      //      test = MPC.PWPR.BYTE;
      /* test = TPU3.TGRA; */
      /* test = S12AD.ADDR0; */
      /* test = TPU3.TCNT; */
      dbgDispLED( test );
    }
    else{
      unsigned char test = ( S12AD.ADDR5 >> 12 );
      dbgDispLED( test );

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


// -------------------------------------------------------
// ------------------------ Functions ( system Utilities )

/* end of main.c */
