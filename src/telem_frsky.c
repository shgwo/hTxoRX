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
  for( uint8_t i=0 ; i<TELFRSKY_BUFF_PARSE ; i++ ){
    parse->buff[i] = 0;
  }
  // clear index
  parse->idx = 0;
  return( 0 );
}

uint8_t TelemFrskyDParseEncue( st_TelemFrskyD_Parse *parse, uint8_t dat ){
  // encue dat
  parse->buff[parse->idx++] = dat;
  // block data fill check
  if( parse->idx >= TELEM_FRSKY_PCK_LEN ){
      parse->stat_fill = 1;
      parse->idx--;
  }
  return( 0 );
}

uint8_t TelemFrskyDParseClrcue( st_TelemFrskyD_Parse *parse ){
  // clear index
  parse->idx = 0;
  return( 0 );
}

//
uint8_t TelemFrskyDInit( st_TelemFrskyD *telem ){
  // buffer cue initialize
  TelemFrskyDParseInit( &(telem->parse) );

  // reading state initialize
  telem->parse.stat_br  = TELFRSKY_STBR_SRC;
  telem->parse.stat_fcr = TELFRSKY_STFCRBT_SRC;
  
  return( 0 );
}

// for frsky D series modules frame parsing
uint8_t TelemFrskyD( st_TelemFrskyD *telem, st_Serial *ser, st_Serial *ser_dbg ){
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
	/* break; */
      }
    }
    // state: recognition (begin/end separator)
    else if( telem->parse.stat_br == TELFRSKY_STBR_REC ){
      // separator: begin | data
      if( (tmp_telem = SerRead( ser )) != TELEM_FRSKY_TAIL ){
	telem->parse.stat_br = TELFRSKY_STBR_RD;
	/* break; */
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
	/* break; */
      }
      // data body
      else if( tmp_telem != TELEM_FRSKY_TAIL ){
	/* break; */
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


uint8_t TelemFrskyDdbgBRPrint( st_TelemFrskyD *telem, st_Serial *ser_dbg ){
  // temp var
  uint8_t dbg_str[SER_BUFF];
  // process
  if( telem->parse.stat_fill ){
    for( uint8_t i=0 ; i<TELFRSKY_BUFF_PARSE ; i++ ){
      sprintf( dbg_str, "%02X ", telem->parse.buff[i] );
      SerBytesWrite( ser_dbg, dbg_str );
    }
    sprintf( dbg_str, " [%02d] \n\r", telem->parse.idx );
    SerBytesWrite( ser_dbg, dbg_str );

    // clear block fill state
    telem->parse.stat_fill = 0;
  }
  return ( 0 );
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
