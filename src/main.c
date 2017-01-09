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
#include "telem_frsky.h"
#include "ppm_gen.h"
#include "adc_sar12b.h"
#include "serial.h"
#include "dbg_utils.h"
#include <stdio.h>

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define sleep(X) for(j=0 ; j < X*1000 ; j++){ }  // wait for X ms

//#define NCH_ADC 21   // number of AD channel on the HW

// -------------------------------------------------------
// -------------------------------------- global variables
st_ADC12  adc12;
st_PPM    ppm;
st_PPMAdj ppm_adj[PPM_N_CH];
st_HMI    hmi;
st_Serial ser[UART_N_APP];
st_hTx    htx;
st_TelemFrskyD telem;
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
  // PC30,26 UART Infotaiment on the Air IF
  SerInit( &ser[UART_IOA], UART_BRATE_IOA, SER_IOA );
  // PE1,2 UART Telemetry IF
  SerInit( &ser[UART_TELEM], UART_BRATE_TELEM, SER_TELEM );  

  // on-board LED
  dbgDispLEDInit();
  
  //relock MPC
  SysMPCLock();

  // setting for Rotary encoder
  /* HMIPortInFuncInit( ); */  

  // initializing PPM ch val ( * beta func *)
  // Name labels for each PPM ch
  char ppm_chname[PPM_N_CH][10] = { "Roll", "Pitch", "Throttle", "Yaw", "Arm", "AUX2", "FiteMD", "AUX4" };
  // set adj ( name str, ad_ch, inv, offset, gain )
  PPMGenAdjInit( &ppm.adj[0], ppm_chname[0],  2, PPMADJ_NOINV, -( 132 * 6), 0.5 ); // Roll
  PPMGenAdjInit( &ppm.adj[0], ppm_chname[0],  2, PPMADJ_NOINV, -( 147 * 6), 0.5 ); // Roll(SBUS)
  PPMGenAdjInit( &ppm.adj[1], ppm_chname[1],  3, PPMADJ_INV,   -( 196 * 6), 0.5 ); // Pitch
  PPMGenAdjInit( &ppm.adj[1], ppm_chname[1],  3, PPMADJ_INV,   -( 211 * 6), 0.5 ); // Pitch(SBUS)
  PPMGenAdjInit( &ppm.adj[2], ppm_chname[2],  0, PPMADJ_NOINV, -( 224 * 6), 0.5 ); // Throttle
  PPMGenAdjInit( &ppm.adj[3], ppm_chname[3],  1, PPMADJ_INV,   -( 297 * 6), 0.5 ); // Yaw
  PPMGenAdjInit( &ppm.adj[3], ppm_chname[3],  1, PPMADJ_INV,   -( 311 * 6), 0.5 ); // Yaw(SBUS)
  PPMGenAdjInit( &ppm.adj[4], ppm_chname[4], 22, PPMADJ_NOINV,           0, 1.0 ); // Arm
  PPMGenAdjInit( &ppm.adj[5], ppm_chname[5],  4, PPMADJ_NOINV,           0, 1.0 ); // AUX2
  PPMGenAdjInit( &ppm.adj[6], ppm_chname[6], 22, PPMADJ_NOINV,           0, 1.0 ); // FliteMD
  PPMGenAdjInit( &ppm.adj[7], ppm_chname[7], 22, PPMADJ_NOINV,           0, 1.0 ); // AUX4

  PPMGenStart();
  
  // 
  // Main routine begin
  //
  // Init vars
  uint8_t  status = 0, stat_ppm=0, div = 3;
  uint8_t  flag   = 0;
  uint16_t ppm_val[8], adc_val[21], adc_bat = 0, adc_arm = 1000;
  uint8_t  mode_test = 0xAA;
  uint8_t  dbg = 0x00, dbg_in = 0x00, dbg_tgl = 0x00;
  uint8_t  dbg_telem, dbg_telem_tail = 0x00;
  uint8_t  dat_telem[SER_BUFF], num_telem = 0, test = 0x00, dbg_ioa[SER_BUFF];
  uint16_t dbg_cnt = 0x00;
  char     dbg_str[SER_BUFF];
  //unsigned char test = 0;
  
  hTxSetMode( &htx, OPMD_SAFE);
  //SerWrite( &ser[UART_MSP], 0xA1 );
  telem.stat = TELFRSKY_SRC;
  sleep( 1000 );

  // main loop
  while(1) {
    // mission-critical loop
    
    
    // state machine
    switch( htx.opmd ){
      // SAFE mode; ARM => not ARMed
      //            throttle => zero
      //            Pitch, Roll, Yaw => neutral
    case OPMD_SAFE:
      hTxSetModeLogOff( &htx );
      if( HMILongPress( &hmi, HMI_SW_ARM_KEY, 500 ) ){
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
      // end condition ( go to safe mode )
      if( HMILongPress( &hmi, HMI_SW_ARM_KEY, 100 ) ){ hTxSetMode( &htx, OPMD_SAFE);  SerWrite( &ser[UART_MSP], 'S' ); }
      if( HMILongPress( &hmi, HMI_SW_L2_KEY, 50 ) ){ hTxSetMode( &htx, OPMD_SAFE);  SerWrite( &ser[UART_MSP], 'S' ); }
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
    /* ADC12GetVal( &adc12 ); */
    /* PPMGenInputFilter( &ppm, &adc12 ); */
    /* /\* if( IR( S12AD, S12ADI0) == 1 ){ *\/ */
    /*   //adc_val[0] = (S12AD.ADDR2 >> (2 + 2) ) + (182 * 6); // Roll (old) */
    /*   /\* adc_val[0] = (uint16_t)( 1.0*(S12AD.ADDR2 >> (2 + 1) ) - (132 * 6) ); // Roll *\/ */
    /*   adc_val[0] = (uint16_t)( 1.0*( adc12.data[2] >> (2 + 1) ) - (132 * 6) ); // Roll */
    /*   //adc_val[1] = (S12AD.ADDR3 >> (2 + 2) ) + (162 * 6); // Pitch (old NINV) */
    /*   //adc_val[1] = (~S12AD.ADDR3 >> (2 + 2) ) +(833 * 6); // Pitch (old) */
    /*   /\* adc_val[1] = (uint16_t)( 1.0*(~S12AD.ADDR3 >> (2 + 1) ) + (1168 * 6) ); // Pitch *\/ */
    /*   adc_val[1] = (uint16_t)( 1.0*( ~adc12.data[3] >> (2 + 1) ) + (1168 * 6) ); // Pitch */
    /*   //adc_val[2] = (S12AD.ADDR0 >> (2 + 2) ) - (114 * 6); // Throttle (old) */
    /*   /\* adc_val[2] = (uint16_t)( 1.0*(S12AD.ADDR0 >> (2 + 1) ) - (224 * 6) ); // Throttle (971 - 1932) *\/ */
    /*   adc_val[2] = (uint16_t)( 1.0*( adc12.data[0] >> (2 + 1) ) - (224 * 6) ); // Throttle (971 - 1932) */
    /*   //adc_val[3] = (S12AD.ADDR1 >> (2 + 2) ) + (209 * 6); // Yaw (NINV) */
    /*   //adc_val[3] = (~S12AD.ADDR1 >> (2 + 2) ) + (783 * 6); // Yaw (INV old) */
    /*   /\* adc_val[3] = (uint16_t)( 1.0*(~S12AD.ADDR1 >> (2 + 1) ) + (1069 * 6) ); // Yaw (INV) *\/ */
    /*   adc_val[3] = (uint16_t)( 1.0*( ~adc12.data[1] >> (2 + 1) ) + (1069 * 6) ); // Yaw (INV) */
    /*   adc_val[4] = 0; */
    /*   adc_val[5] = 0; */
    /*   /\* adc_val[6] = (uint16_t)( 1.0*(S12AD.ADDR5 >> (2 + 1)) ); // AUX3 *\/ */
    /*   adc_val[6] = (uint16_t)( 1.0*( adc12.data[4] >> (2 + 1)) ); // AUX3 */
    /*   adc_val[7] = 0; */

      /* adc_bat = (S12AD.ADDR13 >> (2 + 0) ); */
      adc_bat = ( adc12.data[13] >> (2 + 0) );
    /*   IR( S12AD, S12ADI0 ) = 0; */
    /* } */
    // future func => PPMGenSafeChecker()  <- safe limiter
    // future func => PPMGenOutputFilter() <- LUT-base / Filt-matrix  conversion
    
    // battery check
    // series res Vbat = Vli2S * (3kOhm / 8kOhm)
    //  => Vadc_max = 3.3, Vli_max = 8.8 V
    // define Li-ion Vmax:4.2*2 - Vmin: 3.2*2  = 2.0, 
    // res_Vbat = 2.0 / 8.8 = 0.227 ~ 1/4 of adc resolution
    // Dad_min = 6.4 * 3/8 / 3.3 * 2^14 = 2.4 *k = 0.727 * 2^14 ~ 730 * 16 = 11915
    // Dad_low = 7.2 * 3/8 / 3.3 * 2^14 = 2.7 *k = 1.1  * 11680 = 13405
    // Dad_mid = 7.4 * 3/8 / 3.3 * 2^14 = 3.0 *k = 1.16 * 11680 = 13777
    // Dad_hgh = 8.0 * 3/8 / 3.3 * 2^14 = 3.0 *k = 1.25 * 11680 = 14894
    // Dad_max = 8.4 * 3/8 / 3.3 * 2^14 = 3.15*k = 0.957 * 2^14 ~ 960 * 16 = 15640
    if( adc_bat < 12000 ){
      htx.opmd_bat = OPMD_BAT_DEAD;
    }else if( adc_bat < 13400 ){
      htx.opmd_bat = OPMD_BAT_LOW;
    }else if( adc_bat < 14000 ){
      htx.opmd_bat = OPMD_BAT_MID;
    }else{
      htx.opmd_bat = OPMD_BAT_FULL;
    }
    
    // Motors disarming check
    adc_val[4] = ( (htx.opmd == OPMD_RUN) ? ((htx.opmd_log == OPMD_LOG_ON) ? 0 : (100 * 6) ) : (300 * 6) );
    if( htx.opmd != OPMD_RUN ){
      adc_val[2] = 0; // Throttle
    }
    adc_arm = ( (htx.opmd == OPMD_RUN) ? ((htx.opmd_log == OPMD_LOG_ON) ? 0 : (50 * 6) ) : (300 * 6) );
    PPMGenSetVal( &ppm, 4, adc_arm );
    // Throttle control ( force zero when disarmed )
    if( htx.opmd != OPMD_RUN ){ PPMGenSetVal( &ppm, 2, 0 ); }

    // PPM generation core
    PPMGen( &ppm, &adc12 );
    hmi.cnt_ppm = ppm.cnt_end;
    
    /* adc_val[0] = ppm.data[0]; // Roll */
    /* adc_val[1] = ppm.data[1]; // Pitch */
    /* adc_val[2] = ppm.data[2]; // Throttle */
    /* adc_val[3] = ppm.data[3]; // Yaw */
    /* // check tpu renew timing from interrupt occurance flag */
    /* if( IR( TPU3, TGI3A ) == 1 ){ */
    /*   // tail pulse */
    /*   if( TPU3.TGRA > (400 * 6) ){ */
    /* 	TPU3.TGRC = (300 * 6); */
    /* 	status++; */
    /* 	if( status > 8 ){ status = 0; } */
    /*   } */
    /*   // ppm pulse */
    /*   else{ */
    /* 	// end pulse */
    /* 	if( status >= 8 ){ */
    /* 	  TPU3.TGRC = (6200 * 6); */
    /* 	  hmi.ppm_cnt++; */
    /* 	  // debug */
    /* 	  PORTA.PODR.BIT.B0 = !PORTA.PODR.BIT.B0; */
    /* 	} */
    /* 	// significant pulse */
    /* 	else{ */
    /* 	  TPU3.TGRC = (700 * 6) + adc_val[status]; */
    /* 	} */
    /*   } */
    /*   IR( TPU3, TGI3A ) = 0; */
    /*   PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3; */
    /* } */

    // Back gound processes (UART)
    SerDaemon( &ser[UART_MSP] );
    SerDaemon( &ser[UART_IOA] );
    SerDaemon( &ser[UART_TELEM] );
    // Back gound processes (Telemetry) /* future func */
    /* TelemfrskyD( &telem ); */
    
    // Back gound processes (HMI)
    HMIFlash( &hmi, &htx );
    //if( Ser12ReadBG( &ser[UART_TELEM] ); ){
      // error sequence
      
    //}
    //SerWriteTest( &ser[UART_MSP], mode_test );
    /* HMILEDBatLow( &hmi, op_md_bat ); */
    /* HMILEDSetRGB( &hmi, op_md );  */
 
    // debug routine
    // LED indication (for debug, printf)
    if( !HMISWState( &hmi, HMI_SW_R2_B ) ){
      // IOA (Infotainment over the Air) data stream
      //SerBytesRead( &ser[UART_IOA], dbg_ioa);
      //SerBytesWrite( &ser[UART_IOA], dbg_ioa);
      //SerBytesWrite( &ser[UART_IOA], "hTxoRX: IOA test stream.\n\r");
      dbg_in = SerRead( &ser[UART_MSP]);
      dbg_in = dbg_in | SerRead( &ser[UART_IOA]);
      if( dbg_in == 'd' ){
	/* SerBytesWrite( &ser[UART_MSP], "<dbg toggle>"); */
	dbg_tgl = ~dbg_tgl;
      }
      // debug on each operation loop
      if( dbg_tgl ){
	while( SerReadable( &ser[UART_TELEM] ) ){
	  // state: search (begin/end separator)
	  if( telem.stat == TELFRSKY_SRC ){
	    if( (dbg_telem = SerRead( &ser[UART_TELEM] )) != 0x7e ){ continue; }
	    else{
	      SerBytesWrite( &ser[UART_IOA], "7E " );
	      telem.stat = TELFRSKY_REC;
	      break;
	    }
	  }
	  // state: recognition (begin/end separator)
	  if( telem.stat == TELFRSKY_REC ){
	    // separator: begin
	    if( (dbg_telem = SerRead( &ser[UART_TELEM] )) != 0x7e ){
	      telem.stat = TELFRSKY_RD;
	      sprintf( dbg_str, "%02X ", dbg_telem );
	      SerBytesWrite( &ser[UART_IOA], dbg_str );
	      break;
	    }
	    // separator: end
	    else{
	      SerBytesWrite( &ser[UART_IOA], "7E " );
	    }
	  }
	  if( telem.stat == TELFRSKY_RD ){
	    // data body (w/ bit stuffing)
	    if( (dbg_telem = SerRead( &ser[UART_TELEM] )) == 0x7d ){
	      /* SerBytesWrite( &ser[UART_IOA], "!7D" ); */
	      telem.stat = TELFRSKY_RD_BS;
	      break;
	    }
	    // data body
	    else if( dbg_telem != 0x7e ){
	      sprintf( dbg_str, "%02X ", dbg_telem );
	      SerBytesWrite( &ser[UART_IOA], dbg_str );
	    }
	    // data end
	    else{
	      telem.stat = TELFRSKY_REC;
	      SerBytesWrite( &ser[UART_IOA], "7E \n\r" );
	      break;
	    }
	  }
	  if( telem.stat == TELFRSKY_RD_BS ){
	    // conv 7D 5E => 7E
	    if( (dbg_telem = SerRead( &ser[UART_TELEM] )) == 0x5e ){
	      /* SerBytesWrite( &ser[UART_IOA], "!5E " ); */
	      SerBytesWrite( &ser[UART_IOA], "7E " );
	      telem.stat = TELFRSKY_REC;
	    }
	    // conv 7D 5D => 7D
	    else if( dbg_telem == 0x5d ){
	      /* SerBytesWrite( &ser[UART_IOA], "!5D " ); */
	      SerBytesWrite( &ser[UART_IOA], "7D " );
	      telem.stat = TELFRSKY_REC;
	    }
	    else{
	      SerBytesWrite( &ser[UART_IOA], "FF " );
	      telem.stat = TELFRSKY_REC;
	    }
	  }
	}
	/* SerBytesWrite( &ser[UART_IOA], "|" ); */
	
	/* sprintf( dbg_str, "head|%03d <=> %03d|tail  ", */
	/* 	 ser[UART_TELEM].rx_head, ser[UART_TELEM].rx_tail ); */
	/* SerBytesWrite( &ser[UART_IOA], dbg_str ); */
	/* for( int i=0 ; i<(SER_BUFF>>2) ; i++ ){ */
	/*   sprintf( dbg_str, "%02X ", ser[UART_TELEM].rx_buff[ser[UART_TELEM].rx_head+i] ); */
	/*   SerBytesWrite( &ser[UART_IOA], dbg_str ); */
	/* } */
	/* SerBytesWrite( &ser[UART_IOA], "\n\r" ); */
	/* SerWrite( &ser[UART_IOA], SerRead( &ser[UART_TELEM] ) ); */
      }
      if( (dbg_tgl) && (dbg_cnt++ == 0x00) ){
	//SerBytesWrite( &ser[UART_IOA], "AT" );
	//SerBytesRead( &ser[UART_TELEM], dat_telem );
	/* test = SerRead( &ser[UART_TELEM] ); */
	num_telem = SerBytesRead( &ser[UART_TELEM], dat_telem );
	for( int i=0 ; i<num_telem ; i++ ){
	  sprintf( dbg_str, "%02X ", dat_telem[i] );
	  SerBytesWrite( &ser[UART_IOA], dbg_str );
	}
	/* SerBytesWrite( &ser[UART_IOA], dat_telem); */
	if( num_telem ){
	  SerBytesWrite( &ser[UART_IOA], "\n\r" );
	}
	sprintf( dbg_str, "dbg:(0x%02X)[head %d/tail %d] ",
		 dat_telem[0], ser[UART_TELEM].rx_head, ser[UART_TELEM].rx_tail );
	SerBytesWrite( &ser[UART_MSP], dbg_str );
	SerBytesWrite( &ser[UART_MSP], "SCI12.SSR(0x" );
	uint8_t *sci_err = (struct st_sci0_ssr *)ser[UART_TELEM].addr_base_stat;
	for( int j=0 ; j<sizeof(uint8_t)*8 ; j++ ){
	  if( (*sci_err >> (7-j)) & 0x01 )
	    SerWrite( &ser[UART_MSP], '1' );
	  else
	    SerWrite( &ser[UART_MSP], '0' );
	}
	SerBytesWrite( &ser[UART_MSP], ")\n\r" );
	*sci_err = *sci_err & ~0x38;
	
	/* SerBytesWrite( &ser[UART_MSP], dbg_str ); */
	/* SerBytesWrite( &ser[UART_MSP], "dbg:" ); */
	/* /\* SerWrite( &ser[UART_MSP], test); *\/ */
	/* SerBytesWrite( &ser[UART_MSP], "[head/tail][" ); */
	/* test = 0x30 + (ser[UART_TELEM].rx_head % 10); */
	/* SerWrite( &ser[UART_MSP], test); */
	/* test = 0x30 + (ser[UART_TELEM].rx_tail % 10); */
	/* SerBytesWrite( &ser[UART_MSP], "/" ); */
	/* SerWrite( &ser[UART_MSP], test); */
       
	//SerWrite( &ser[UART_TELEM], dbg );
	/* if( dbg = SerRead( &ser[UART_TELEM] ) ){ */
	/*   sprintf( dbg_str, "0x%0X", dbg ); */
	/*   SerBytesWrite( &ser[UART_MSP], dbg_str ); */
	/*   //SerWrite( &ser[UART_MSP], (0x30 + (ser[UART_TELEM].rx_head % 10)) ); */
	/*   //SerWrite( &ser[UART_MSP], (0x30 + (dbg % 10)) ); */
	/*   SerWrite( &ser[UART_MSP], ' ' ); */
	/* }	 */
      }
      /* // GPIO telem pin check */
      /* if( PORTE.PIDR.BIT.B2 ){ */
      /* 	SerWrite( &ser[UART_MSP], '1'); */
      /* }else{ */
      /* 	SerWrite( &ser[UART_MSP], '0'); */
      /* } */
      //dbg = 0x30 + SerRead( &ser[UART_TELEM] ) % 10;
      /* dbg = SerRead( &ser[UART_TELEM] ); */
      /* SerWrite( &ser[UART_MSP], dbg ); */

      /* 	SerBytesWrite( &ser[UART_MSP], (0x30 + (ser[UART_MSP].rx_head % 10)) ); */
      /* } */

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
      if( (hmi.cnt_ppm >> 7) & 0x01 ){
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
      unsigned char test = ( S12AD.ADDR4 >> 12 );
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
