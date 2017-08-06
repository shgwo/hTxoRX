// -------------------------------------------------------
// ------------------------------------------------ Notice
//  This program is distributed or redistributed to
//  the world under the License of MIT.
//
//
// -------------------------------------------------------
// ------------------------------------------------- Info.
//
//  Description:   Relative utilities for telemetry for frsky D series
//                 (based on serial.h/serial.c)
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.2
//  last updated : 2016.01.09
//
//  history:
//    2017.01.09  create for RX63N( GR-SAKURA ) base-system
//                target system:  TX | DHT  <=> D4R-ii | RX
//    2017.06.18  add codes to parsing telemetry user packet
//
#include "typedefine.h"
#include "telem_frsky.h"
#include <stdio.h>

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  Telemetry parse utilities 
//
// utility for parse cue
uint8_t TelemFrskyDParseInit( st_TelemFrskyD_Parse *parse ){
  // clear buff
  for( uint8_t i=0 ; i<TELEM_FRSKY_PKT_LEN ; i++ ){
    parse->PKT.buff[i] = 0;
  }
  // clear index
  parse->idx = 0;
  return( 0 );
}

uint8_t TelemFrskyDParseEncue( st_TelemFrskyD_Parse *parse, uint8_t dat ){
  // encue dat (cue is full -> ignore )
  if( parse->idx < TELEM_FRSKY_PKT_LEN ){
    parse->PKT.buff[parse->idx++] = dat;
  }else{
    // filling flag
    parse->stat_fill = 1;
  }
  return( 0 );
}

uint8_t TelemFrskyDParseClrcue( st_TelemFrskyD_Parse *parse ){
  // clear index
  parse->idx       = 0;
  parse->stat_fill = 0;
  return( 0 );
}

uint8_t TelemFrskyDSwap16( uint16_t *bytes ){
  // tmp var
  uint16_t tmp;

  // swap
  tmp    = *bytes;
  *bytes = (*bytes << 8) & 0xff00;
  *bytes |= (tmp >> 8) & 0x00ff;
}

// utility for FC parse cue
uint8_t TelemFrskyDFCParseInit( st_TelemFrskyD_Parse *parse ){
  // clear buff
  for( uint8_t i=0 ; i<TELEM_FRSKY_FCR_CUE_LEN ; i++ ){
    parse->FCR.buff[i] = 0;
  }
  for( uint8_t i=0 ; i<TELEM_FRSKY_FCR_IDS_LEN ; i++ ){
    parse->IDS.buff[i] = 0xFFFF;
  }  
  // clear index
  parse->idx_fcr = 0;
  // clear chain count
  parse->fcr_cnt_old = 0xFF;
  return( 0 );
}

uint8_t TelemFrskyDFCParseEncue( st_TelemFrskyD_Parse *parse, uint8_t dat ){
  // encue dat (cue is full -> ignore )
  if( parse->idx_fcr < TELEM_FRSKY_FCR_CUE_LEN ){
    parse->FCR.buff[parse->idx_fcr++] = dat;
  }else{
    // filling flag
    parse->stat_fcr_fill = 1;
  }
  return( 0 );
}

uint8_t TelemFrskyDFCParseClrcue( st_TelemFrskyD_Parse *parse ){
  // clear index
  parse->idx_fcr       = 0;
  parse->stat_fcr_fill = 0;
  return( 0 );
}


// Initializing telemetry reading utils
uint8_t TelemFrskyDInit( st_TelemFrskyD *telem ){
  // buffer cue initialize
  TelemFrskyDParseInit( &(telem->parse) );
  TelemFrskyDFCParseInit( &(telem->parse) );

  // safe state initialize
  telem->parse.stat_safe = TELM_SAFE_NFD;
  
  // reading state initialize
  telem->parse.stat_br  = TELFRSKY_STBR_SRC;
  telem->parse.stat_br  = TELFRSKY_STPT_LNK;
  telem->parse.stat_fcr = TELFRSKY_STFCRBF_SRC;
  
  return( 0 );
}

// State observer of block reading for Frsky telemetry
uint8_t TelemFrskyDObsBR( st_TelemFrskyD *telem, uint8_t byte ){

  // state observer sync 1 step
  switch( telem->parse.stat_br ){
  case TELFRSKY_STBR_SRC:   // state: search (begin/end separator)
    if( byte != TELEM_FRSKY_HEAD ){ }
    else{
      telem->parse.stat_br = TELFRSKY_STBR_REC;
    }
    break;
  case TELFRSKY_STBR_REC:   // state: recognition (begin/end separator)
    // pre-read separator: begin > data
    if( byte != TELEM_FRSKY_TAIL )
      telem->parse.stat_br = TELFRSKY_STBR_RD;
    // pre-read separator: end   > begin
    else
      TelemFrskyDParseClrcue( &(telem->parse) );
    break;
  case TELFRSKY_STBR_RD:
    // data body (w/ bit stuffing)
    if( byte == TELEM_FRSKY_BS00 )
      telem->parse.stat_br = TELFRSKY_STBR_RD_BS;
    // data body (ordinal)
    else if( byte != TELEM_FRSKY_TAIL ){}
    // data end
    else
	telem->parse.stat_br = TELFRSKY_STBR_REC;
    break;    
  case TELFRSKY_STBR_RD_BS:
    // conv 7D 5E => 7E
    if( byte == TELEM_FRSKY_BS10 ){
      byte = TELEM_FRSKY_BS10_;
      telem->parse.stat_br = TELFRSKY_STBR_REC;
    }
    // conv 7D 5D => 7D
    else if( byte == TELEM_FRSKY_BS11 ){
      byte = TELEM_FRSKY_BS11_;
      telem->parse.stat_br = TELFRSKY_STBR_REC;
    }
    else{
      telem->parse.stat_br = TELFRSKY_STBR_SRC;
    }    
    break;
  default:
    telem->parse.stat_br = TELFRSKY_STBR_SRC;
    TelemFrskyDParseClrcue( &(telem->parse) );
    break;
  }
  // encue efficient byte
  if( telem->parse.stat_br != TELFRSKY_STBR_RD_BS )
    TelemFrskyDParseEncue( &(telem->parse), byte );

  return( 0 );
}

// state observer for Frsky packet
uint8_t TelemFrskyDPktSel( st_TelemFrskyD *telem ){
  // operation cycle check
  if( telem->parse.idx != 2 )
    return(1);
  
  // state observer sync step
  switch( telem->parse.PKT.buff[1] ){
  case LINKPKT:
    telem->parse.stat_pt  = TELFRSKY_STPT_LNK;
    break;
  case USRPKT:
    telem->parse.stat_pt  = TELFRSKY_STPT_USR;
    break;
  default:
    telem->parse.stat_pt  = TELFRSKY_STPT_UKWN;
    break;
  }
  
  return( 0 );
}

// get var of link packet from Frsky telemetry
uint8_t TelemFrskyDBRLnk( st_TelemFrskyD *telem ){
  // operation cycle check
  if( telem->parse.idx != 11 )
    return(1);
  
  // tmp var
  st_TelemFrskyD_LnkPkt pkt_lnk;
  for( uint8_t i=0 ; i<TELEM_FRSKY_PKT_LEN ; i++ ){
    /* pkt_lnk.PKTLNK.buff[i] = telem->parse.buff[i]; */
  }
}

// swap frsky packet bytes in link packet
// (ex.)
// 7E FE 59 67 AD 5E 00 00 00 00 7E
// |  |  |  |  |<>|              |
// +HD|  |  |  + RSSI            +TL
//    |  |  + AIN2
//    |  + AIN1
//    + packet type
uint8_t TelemFrskyDBRLnkRSSI( st_TelemFrskyD *telem ){
  // operation cycle check
  if( telem->parse.idx != 7 )
    return(1);

  // get correct rssi value
  TelemFrskyDSwap16( &(telem->parse.PKT.LNK.rssi) );
  return( 0 );
}


// check frsky packet to packet chain error in user packet
// (ex.)
// 7E FD 06 03 30 00 00 5E 5E 24 7E
// |  |  |  |  |  -  -  -  -  |  |
// +HD|  |  |  + <  payload  >|  +TL
//    |  |  + chain number 
//    |  + eff. byte length
//    + packet type
uint8_t TelemFrskyDBRFCChainChk( st_TelemFrskyD *telem ){
  // tmp var
  uint8_t ret;
  int8_t  diff;
  // operation cycle check
  if( telem->parse.idx != 4 )
    return(1);
  
  // packet chain error check (00 - 1F numbering)
  diff = telem->parse.PKT.USR.cnt - telem->parse.fcr_cnt_old;
  // incremented => next data chain
  // holded      => continuous data chain
  // 1F >> 00    => count carry
  if( diff == 1 || diff == 0 || diff == (uint8_t)(0x00 - 0x1F) )
    ret = 0;
  // other       => count refresh ( exception )
  else
    ret = 2;
  
  telem->parse.fcr_cnt_old = telem->parse.PKT.USR.cnt;
  
  // error code:
  //   0 / 1 / 2 = run / wait / clr cnt
  return( ret );
}

// state observer for Frsky sensor data train
uint8_t TelemFrskyDObsFC( st_TelemFrskyD *telem, uint8_t byte ){
  // tmp var
  // N/A

  // data correctivity check (using 4th byte of chain number)
  if( TelemFrskyDBRFCChainChk( telem ) == 2 ){
    telem->parse.stat_fcr + TELFRSKY_STFCRBF_CCHKERR;
    TelemFrskyDFCParseClrcue( &(telem->parse) );      // wait for next BR (to do)
    return(1);
  }
  // operation cycle check (using 5th byte of payload)
  if( telem->parse.idx < 5 || (telem->parse.idx > (4 + telem->parse.PKT.USR.len)) )
    return(1);

  // state observer sync 1 step
  switch( telem->parse.stat_fcr ){
  case TELFRSKY_STFCRBF_SRC:  // state: search (be begin/end separator)
    if( byte != TELEM_FRSKY_FC_HEAD ){ }
    else{
      telem->parse.stat_fcr = TELFRSKY_STFCRBF_REC;
    }
    break;
  case TELFRSKY_STFCRBF_REC:  // state: recognition (by begin/end separator)
    // pre-read separator: begin > data
    if( byte != TELEM_FRSKY_FC_TAIL ){
      telem->parse.stat_fcr = TELFRSKY_STFCRBF_RD;
    }
    // pre-read separator: end   > begin
    else{
      telem->parse.stat_fcr = TELFRSKY_STFCRBF_REC;
      TelemFrskyDFCParseClrcue( &(telem->parse) );
    }
    break;
  case TELFRSKY_STFCRBF_CCHKERR:  // chain count check error
    if( telem->parse.idx == 1 )
      telem->parse.stat_fcr = TELFRSKY_STFCRBF_REC;      
    break;
  case TELFRSKY_STFCRBF_RD:
    // data body (w/ byte stuffing)
    if( byte == TELEM_FRSKY_BSFC00 )
      telem->parse.stat_fcr = TELFRSKY_STFCRBF_RD_BS;
    else if( byte != TELEM_FRSKY_FC_TAIL ){ }
    else{
      TelemFrskyDFCParseClrcue( &(telem->parse) );
      telem->parse.stat_fcr = TELFRSKY_STFCRBF_REC;
    }
    break;
  case TELFRSKY_STFCRBF_RD_BS:
    // conv 5D 3E => 5E
    if( byte == TELEM_FRSKY_BSFC10 ){
      byte = TELEM_FRSKY_BSFC10_;
      telem->parse.stat_fcr = TELFRSKY_STBR_REC;
    }
    // conv 5D 3D => 5D
    else if( byte == TELEM_FRSKY_BSFC11 ){
      byte = TELEM_FRSKY_BSFC11_;
      telem->parse.stat_fcr = TELFRSKY_STBR_REC;
    }
    else{
      // exception
      telem->parse.stat_fcr = TELFRSKY_STBR_SRC;
      TelemFrskyDFCParseClrcue( &(telem->parse) );
    }    
    break;
  default:
    // to the start point
    telem->parse.stat_fcr = TELFRSKY_STBR_SRC;
    TelemFrskyDFCParseClrcue( &(telem->parse) );
    break;
  }
  // encue efficient byte
  if( telem->parse.stat_br != TELFRSKY_STFCRBF_RD_BS )
    TelemFrskyDFCParseEncue( &(telem->parse), byte );  
  
  return(0);
}

// from sendAccel of betaflight
uint16_t TelemFrskyDConvFCAccel( uint8_t *dat )
{
  // tmp var
  uint16_t tmp;
  // swap
  tmp |= (*(dat+1) << 8);
  tmp |= *(dat);
  // calc
  return( tmp * TELFRSKY_ACC_1G / 1000 );
}

// from sendVoltage of betaflight
  // == from the betaflight ==
  /*
   * Format for Voltage Data for single cells is like this:
   *
   *  F--C B--8 7--4 3--0
   *  llll llll cccc hhhh
   *  l: Low voltage bits
   *  h: High voltage bits
   *  c: Cell number (starting at 0)
   *
   * The actual value sent for cell voltage has resolution of 0.002 volts 
   */
uint16_t TelemFrskyDConvFCVoltage( uint8_t *dat ){
  // tmp var
  uint16_t tmp;
  // swapping
  /* tmp |= (*(dat) << 8); */
  /* tmp |= *(dat); */
  return( tmp = (((uint16_t)*(dat) & 0x000F) << 8) | *(dat+1) );

  // Cell number is at bit 7->4
  //payload = (currentCell << 4);
  
  // Lower voltage bits are at bit F->8
  /* payload |= ((cellVoltage & 0x0ff) << 8); */
  
  // Higher voltage bits are at bits 3->0
  /* payload |= ((cellVoltage & 0xf00) >> 8); */
  
  /* the cell volatage sent from betaflight is derived */
  /* cellVoltage = ((uint32_t)getBatteryVoltage() * 100 + cellCount) / (cellCount * 2); */
} 

// not correctly be calced yet
uint16_t TelemFrskyDConvFCFuel( uint8_t *dat ){
  // tmp var
  uint16_t tmp;
  // swap
  tmp |= (*(dat+1) << 8);
  tmp |= *(dat);
  // calc
  return( tmp );
}

// not correctly be calced yet
uint16_t TelemFrskyDConvFCVario( uint8_t *dat ){
  // tmp var
  uint16_t tmp;
  // swap
  tmp |= (*(dat+1) << 8);
  tmp |= *(dat);
  // calc
  return( tmp );
}

// not correctly be calced yet
uint16_t TelemFrskyDConvFCHourMinute( uint8_t *dat ){
  // tmp var
  uint16_t tmp = 0x0000;
  // swap
  tmp |= (*(dat+1) << 8);
  tmp |= *(dat);
  // calc
  return( tmp );
}

// not correctly be calced yet
uint16_t TelemFrskyDConvFCSecond( uint8_t *dat ){
  // tmp var
  uint16_t tmp = 0x0000;
  // swap
  tmp |= (*(dat+1) << 8);
  tmp |= *(dat);
  // calc
  return( tmp );
}

// not correctly be calced yet
uint16_t TelemFrskyDConvFCGyro( uint8_t *dat ){
  // tmp var
  uint16_t tmp = 0x0000;
  // swap
  tmp |= (*(dat+1) << 8);
  tmp |= *(dat);
  // calc
  return( tmp );
}


// state observer for Frsky sensor data train
uint8_t TelemFrskyDConvFC( st_TelemFrskyD *telem ){
  // operation cycle wait check
  if( telem->parse.idx_fcr != 4 )
    return(1);
  
  // func call for each conversion calc
  switch( telem->parse.FCR.CUE.id ){
  case ID_GPS_ALTIDUTE_BP:
    // calling (to do)
    break;
  case ID_GPS_ALTIDUTE_AP:
    // calling (to do)
    break;
  case ID_FUEL_LEVEL:   // 0x04
    telem->parse.IDS.SEN.fuel = TelemFrskyDConvFCFuel( telem->parse.FCR.CUE.dat );
    break;
  case ID_VOLT:   // 0x06
    telem->parse.IDS.SEN.volt = TelemFrskyDConvFCVoltage( telem->parse.FCR.CUE.dat );
    break;
  case ID_HOUR_MINUTE:   // 0x17
    telem->parse.IDS.SEN.hr_min = TelemFrskyDConvFCHourMinute( telem->parse.FCR.CUE.dat );
    break;
  case ID_SECOND:   // 0x18
    telem->parse.IDS.SEN.sec = TelemFrskyDConvFCSecond( telem->parse.FCR.CUE.dat );
    break;
  case ID_ACC_X:  // 0x24
    telem->parse.IDS.SEN.accx = TelemFrskyDConvFCAccel( telem->parse.FCR.CUE.dat );
    break;
  case ID_ACC_Y:  // 0x25
    telem->parse.IDS.SEN.accy = TelemFrskyDConvFCAccel( telem->parse.FCR.CUE.dat );
    break;
  case ID_ACC_Z:  // 0x26
    telem->parse.IDS.SEN.accz = TelemFrskyDConvFCAccel( telem->parse.FCR.CUE.dat );
    break;
  case ID_VERT_SPEED:  // 0x30
    telem->parse.IDS.SEN.vspd = TelemFrskyDConvFCVario( telem->parse.FCR.CUE.dat );
    break;
  case ID_GYRO_X:  // 0x40
    telem->parse.IDS.SEN.gyrox = TelemFrskyDConvFCGyro( telem->parse.FCR.CUE.dat );
    break;
  case ID_GYRO_Y:  // 0x41
    telem->parse.IDS.SEN.gyroy = TelemFrskyDConvFCGyro( telem->parse.FCR.CUE.dat );
    break;
  case ID_GYRO_Z:  // 0x42
    telem->parse.IDS.SEN.gyroz = TelemFrskyDConvFCGyro( telem->parse.FCR.CUE.dat );
    break;
  default:
    // calling (to do)
    break;
  }
  
  return(0);
}

uint8_t TelemFrskyDSaferInit( st_TelemFrskyD *telem ){
  telem->parse.stat_safe = TELM_SAFE_NFD;   // initial stat, telemetry not found
  telem->parse.cnt_link  = 0;               // zero clear of lost count
  return(0);
}

uint8_t TelemFrskyDSafer( st_TelemFrskyD *telem, uint8_t ret_ser_chk ){
  // tmp vars

  // serial continuos check (for state tran to TELM_SAFE_LOST)
  if( ret_ser_chk ){
    telem->parse.cnt_link = 0;
    telem->parse.stat_safe = TELM_SAFE_ESTB;
  }else{
    if( telem->parse.cnt_link != 0xFF ) // carry guard
      telem->parse.cnt_link++;

    // state transition
    if( telem->parse.cnt_link >= TELFRSKY_TH_LNK_CNT ){
      switch( telem->parse.stat_safe ){
      case   TELM_SAFE_ESTB:   // telemetry established
      case   TELM_SAFE_WEAK:   // established, signal is low
      case   TELM_SAFE_LOSTL:  // once, established & now lost (link packet)
      case   TELM_SAFE_LOSTU:  // once, established & now lost (user packet)
	telem->parse.stat_safe = TELM_SAFE_LOST;
	break;
      default: // state is NFD
	break;
      }
    }
  }
    
  // operation cycle check (for state tran to TELM_SAFE_WEAK/LOSTL/LOSTU)
  // at idx=6, expect to finish to get RSSI / usr pcket chain
  if( telem->parse.idx != 6 ){
    // user packet => check if LOSTU
    if( telem->parse.stat_pt == TELFRSKY_STPT_USR ){
      if( telem->parse.stat_fcr == TELFRSKY_STFCRBF_CCHKERR )
	telem->parse.stat_safe = TELM_SAFE_LOSTU;
      // link packet => check if WEAK&LOSTL
    }else if(telem->parse.stat_pt == TELFRSKY_STPT_LNK){
      if( telem->parse.PKT.LNK.rssi <= TELFRSKY_TH_LNK_RSSI_WEAK )
	telem->parse.stat_safe = TELM_SAFE_WEAK;
      else if( telem->parse.PKT.LNK.rssi <= TELFRSKY_TH_LNK_RSSI_LOST )
      	telem->parse.stat_safe = TELM_SAFE_LOSTL;
    }
  }
  return(1);  
  
}

// for frsky D series modules frame parsing
uint8_t TelemFrskyD( st_TelemFrskyD *telem, st_Serial *ser ){
  // vars
  uint8_t tmp_telem = 0x00;

  // telem safe obs
  TelemFrskyDSafer( telem, SerReadable( ser ) );
  
  // read & parse operation loop
  while( SerReadable( ser ) ){
    // read a byte of telemetry UART
    tmp_telem = SerRead( ser );

    // obs sync: block read
    TelemFrskyDObsBR( telem, tmp_telem );
    // obs sync: packet type recognition
    TelemFrskyDPktSel( telem );

    // packet data retreiving
    switch( telem->parse.stat_pt ){
    case TELFRSKY_STPT_LNK:
      // packet read: link packet
      TelemFrskyDBRLnkRSSI( telem );
      /* TelemFrskyDBRLnk( telem ); /\* how often is this called? *\/ */
      break;
    case TELFRSKY_STPT_USR:
      // obs sync: sensor (user packet) read
      TelemFrskyDObsFC( telem, tmp_telem );
      TelemFrskyDConvFC( telem );
      break;
    default:
      // exception
      break;
    }
  }
  return( 0 );
}

// for frsky D series modules frame parsing
uint8_t TelemFrskyD__backup__( st_TelemFrskyD *telem, st_Serial *ser, st_Serial *ser_dbg ){
  // vars
  uint8_t tmp_telem = 0x00;
  uint8_t dbg_str[SER_BUFF];
  // start to extract
  while( SerReadable( ser ) ){
    // state: search (begin/end separator)
    if( telem->parse.stat_br == TELFRSKY_STBR_SRC ){
      if( (tmp_telem = SerRead( ser )) != TELEM_FRSKY_HEAD ){ continue; }
      else{
	telem->parse.stat_br = TELFRSKY_STBR_REC;
      }
    }
    // state: recognition (begin/end separator)
    else if( telem->parse.stat_br == TELFRSKY_STBR_REC ){
      // separator: begin | data
      if( (tmp_telem = SerRead( ser )) != TELEM_FRSKY_TAIL ){
	telem->parse.stat_br = TELFRSKY_STBR_RD;
      }
      // separator: end | begin
      else{
	TelemFrskyDParseClrcue( &(telem->parse) );
      }
    }
    else if( telem->parse.stat_br == TELFRSKY_STBR_RD ){
      // data body (w/ bit stuffing)
      if( (tmp_telem = SerRead( ser )) == TELEM_FRSKY_BS00 ){
	telem->parse.stat_br = TELFRSKY_STBR_RD_BS;
      }
      // data body
      else if( tmp_telem != TELEM_FRSKY_TAIL ){
      }
      // data end
      else{
	telem->parse.stat_br = TELFRSKY_STBR_REC;
	/* telem->parse.stat_fill = 1; */
	// debug: block disp
	/* for( uint8_t i=0 ; i<TELFRSKY_BUFF_PARSE ; i++ ){ */
	/*   sprintf( dbg_str, "%02X ", telem->parse.buff[i] ); */
	/*   SerBytesWrite( ser_dbg, dbg_str ); */
	/* } */
	/* SerBytesWrite( ser_dbg, "\n\r" ); */
	/* break; */
      }
    }
    else if( telem->parse.stat_br == TELFRSKY_STBR_RD_BS ){
      // conv 7D 5E => 7E
      if( (tmp_telem = SerRead( ser )) == TELEM_FRSKY_BS10 ){
	tmp_telem = TELEM_FRSKY_BS10_;
	telem->parse.stat_br = TELFRSKY_STBR_REC;
      }
      // conv 7D 5D => 7D
      else if( tmp_telem == TELEM_FRSKY_BS11 ){
	tmp_telem = TELEM_FRSKY_BS11_;
	telem->parse.stat_br = TELFRSKY_STBR_REC;
      }
      else{
	telem->parse.stat_br = TELFRSKY_STBR_SRC;
      }
    }
    // encue efficient byte
    if( telem->parse.stat_br != TELFRSKY_STBR_RD_BS ){
      TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
      /* sprintf( dbg_str, "[%02d]%02X ", telem->parse.idx, tmp_telem ); */
      /* sprintf( dbg_str, "[%02d]%02X ", telem->parse.idx, telem->parse.buff[telem->parse.idx-1] ); */
      /* SerBytesWrite( ser_dbg, dbg_str ); */
    }
  }
  return ( 0 );
}

// Frsky D series telemery debug print (test function)
uint8_t TelemFrskyDdbgBRPrint( st_TelemFrskyD *telem, st_Serial *ser_dbg ){
  // temp var
  uint8_t dbg_str[SER_BUFF];
  // process
  // regulated to print one-time / PKT
  if( telem->parse.idx != TELEM_FRSKY_PKT_LEN ){
    telem->parse.stat_fill = 0;    
  }else if( telem->parse.stat_fill ){
    return(0);
  }else{
    // one-time / PKT flag
    telem->parse.stat_fill = 1;
    
    // bulk data disp
    for( uint8_t i=0 ; i<TELEM_FRSKY_PKT_LEN ; i++ ){
      sprintf( dbg_str, "%02X ", telem->parse.PKT.buff[i] );
      SerBytesWrite( ser_dbg, dbg_str );
    }
    /* //disp var [idx] */
    /* sprintf( dbg_str, " [%02d] ", telem->parse.idx ); */
    /* SerBytesWrite( ser_dbg, dbg_str ); */
    //disp var [statBlockParse]
    sprintf( dbg_str, "(%01d) ", telem->parse.stat_pt );
    SerBytesWrite( ser_dbg, dbg_str );
    // link packet disp
    if( telem->parse.stat_pt == TELFRSKY_STPT_LNK ){
      sprintf( dbg_str, "[%02X][%02X][%04X] ",
	       telem->parse.PKT.LNK.port1,
	       telem->parse.PKT.LNK.port2,
	       telem->parse.PKT.LNK.rssi );
      SerBytesWrite( ser_dbg, dbg_str );
    }else{
      SerBytesWrite( ser_dbg, "               " );
    }
    // user packet disp
    if( telem->parse.stat_pt == TELFRSKY_STPT_USR ){
      // block data disp
      for(uint8_t i=0 ; i<(telem->parse.PKT.USR.len) ; i++){
    	sprintf( dbg_str, "%02X ", telem->parse.PKT.USR.dat[i]);
	SerBytesWrite( ser_dbg, dbg_str );
      }
      for(uint8_t i=0 ; i<(6 - (telem->parse.PKT.USR.len)) ; i++){
	SerBytesWrite( ser_dbg, "   " );
      }
      // user packet parse cue disp
      SerBytesWrite( ser_dbg, "| " );
      for(uint8_t j=0 ; j<TELEM_FRSKY_FCR_CUE_LEN ; j++){
      	sprintf( dbg_str, "%02X ", telem->parse.FCR.buff[j]);
	SerBytesWrite( ser_dbg, dbg_str );
      }
      // user packet parse state disp
      SerBytesWrite( ser_dbg, " " );
      sprintf( dbg_str, "[%02d|%02X]",
	       telem->parse.idx_fcr, telem->parse.stat_fcr );
      SerBytesWrite( ser_dbg, dbg_str );
    }

    SerBytesWrite( ser_dbg, " \n\r" );

    // clear block fill state
    /* telem->parse.stat_fill = 0; */
  }
  return ( 0 );
}

uint8_t TelemFrskyDdbgStatPrint( st_TelemFrskyD *telem, st_Serial *ser_dbg ){
  // temp var
  uint8_t dbg_str[SER_BUFF];
  uint16_t tmp_gage;
  // process
  // regulated to print one-time / PKT
  if( telem->parse.idx_fcr != (TELEM_FRSKY_FCR_CUE_LEN-1) ){
    telem->parse.stat_fcr_fill = 0;    
  }else if( telem->parse.stat_fcr_fill ){
    return(0);
  }else{
    // one-time / PKT flag
    telem->parse.stat_fcr_fill = 1;

    // sensor disp
    tmp_gage = telem->parse.IDS.buff[telem->parse.idx_fcr_ids];
    sprintf( dbg_str, "[%02X]  %04X ",
	     telem->parse.idx_fcr_ids,
	     telem->parse.IDS.buff[telem->parse.idx_fcr_ids++] );
    SerBytesWrite( ser_dbg, dbg_str );
    // sensor meter disp
    tmp_gage = (tmp_gage >> 10);
    SerWrite( ser_dbg, '[' );
    for( uint8_t i=0 ; i < tmp_gage ; i++  )
      SerWrite( ser_dbg, '=' );
    SerBytesWrite( ser_dbg, "] \n\r" );

    if( telem->parse.idx_fcr_ids >= TELEM_FRSKY_FCR_IDS_LEN ){
      SerBytesWrite( ser_dbg, "\n\r==== telemetry status ====\n\r" );
      /* SerWrite( ser_dbg, 0x12 ); */
      telem->parse.idx_fcr_ids = 0;
    }
  }
    
  return ( 0 );
}

uint8_t TelemFrskyDdbgBRPrintBat( st_TelemFrskyD *telem, st_Serial *ser_dbg ){
  // packet ditinguish check
  if( telem->parse.stat_fill ){
    // user packet (bat) disp
    
  }
}



// for frsky D series modules frame debug (via UART)
uint8_t TelemFrskyDdbg( st_TelemFrskyD *telem, st_Serial *ser_telem, st_Serial *ser_dbg ){
  // vars
  uint8_t tmp_telem = 0x00;
  uint8_t dbg_str[SER_BUFF];
  // start to extract
  while( SerReadable( ser_telem ) ){
    // state: search (begin/end separator)
    if( telem->parse.stat_br == TELFRSKY_STBR_SRC ){
      if( (tmp_telem = SerRead( ser_telem )) != TELEM_FRSKY_HEAD ){ continue; }
      else{
	SerBytesWrite( ser_dbg, "7E " );
	telem->parse.stat_br = TELFRSKY_STBR_REC;
	break;
      }
    }
    // state: recognition (begin/end separator)
    if( telem->parse.stat_br == TELFRSKY_STBR_REC ){
      // separator: begin
      if( (tmp_telem = SerRead( ser_telem )) != TELEM_FRSKY_TAIL ){
	sprintf( dbg_str, "%02X ", tmp_telem );
	SerBytesWrite( ser_dbg, dbg_str );
	telem->parse.stat_br = TELFRSKY_STBR_RD;
	break;
      }
      // separator: end
      else{
	SerBytesWrite( ser_dbg, "7E " );
      }
    }
    if( telem->parse.stat_br == TELFRSKY_STBR_RD ){
      // data body (w/ bit stuffing)
      if( (tmp_telem = SerRead( ser_telem )) == TELEM_FRSKY_BS00 ){
	/* SerBytesWrite( ser_dbg, "!7D" ); */
	telem->parse.stat_br = TELFRSKY_STBR_RD_BS;
	break;
      }
      // data body
      else if( tmp_telem != TELEM_FRSKY_TAIL ){
	sprintf( dbg_str, "%02X ", tmp_telem );
	SerBytesWrite( ser_dbg, dbg_str );
      }
      // data end
      else{
	SerBytesWrite( ser_dbg, "7E \n\r" );
	telem->parse.stat_br = TELFRSKY_STBR_REC;
	break;
      }
    }
    if( telem->parse.stat_br == TELFRSKY_STBR_RD_BS ){
      // conv 7D 5E => 7E
      if( (tmp_telem = SerRead( ser_telem )) == TELEM_FRSKY_BS10 ){
	/* SerBytesWrite( ser_dbg, "!5E " ); */
	SerBytesWrite( ser_dbg, "7D " );
	telem->parse.stat_br = TELFRSKY_STBR_REC;
      }
      // conv 7D 5D => 7D
      else if( tmp_telem == TELEM_FRSKY_BS11 ){
	/* SerBytesWrite( ser_dbg, "!5D " ); */
	SerBytesWrite( ser_dbg, "7E " );
	telem->parse.stat_br = TELFRSKY_STBR_REC;
      }
      else{
	SerBytesWrite( ser_dbg, "FF " );
	telem->parse.stat_br = TELFRSKY_STBR_REC;
      }
    }
  }
  return ( 0 );
}


/* end of telem_frsky.c */
