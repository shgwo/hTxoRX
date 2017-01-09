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
  if( parse->idx >= TELFRSKY_BUFF_PARSE )
    return( 1 );
  parse->buff[parse->idx++] = dat;
  return( 0 );
}

//
uint8_t TelemFrskyDInit( st_TelemFrskyD *telem ){
  TelemFrskyDParseInit( &(telem->parse) );
}

// for frsky D series modules frame parsing
uint8_t TelemFrskyD( st_TelemFrskyD *telem, st_Serial *ser ){
  // vars
  uint8_t tmp_telem = 0x00;
  // start to extract
  while( SerReadable( ser ) ){
    // state: search (begin/end separator)
    if( telem->stat == TELFRSKY_SRC ){
      if( (tmp_telem = SerRead( ser )) != TELEM_FRSKY_HEAD ){ continue; }
      else{
	/* SerBytesWrite( ser[UART_IOA], "7E " ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
	telem->stat = TELFRSKY_REC;
	break;
      }
    }
    // state: recognition (begin/end separator)
    if( telem->stat == TELFRSKY_REC ){
      // separator: begin
      if( (tmp_telem = SerRead( ser )) != TELEM_FRSKY_TAIL ){
	/* sprintf( dbg_str, "%02X ", tmp_telem ); */
	/* SerBytesWrite( ser[UART_IOA], dbg_str ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
	telem->stat = TELFRSKY_RD;
	break;
      }
      // separator: end
      else{
	/* SerBytesWrite( ser[UART_IOA], "7E " ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
      }
    }
    if( telem->stat == TELFRSKY_RD ){
      // data body (w/ bit stuffing)
      if( (tmp_telem = SerRead( ser )) == TELEM_FRSKY_BS00 ){
	/* SerBytesWrite( ser[UART_IOA], "!7D" ); */
	telem->stat = TELFRSKY_RD_BS;
	break;
      }
      // data body
      else if( tmp_telem != TELEM_FRSKY_TAIL ){
	/* sprintf( dbg_str, "%02X ", tmp_telem ); */
	/* SerBytesWrite( ser[UART_IOA], dbg_str ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
      }
      // data end
      else{
	/* SerBytesWrite( ser[UART_IOA], "7E \n\r" ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
	telem->stat = TELFRSKY_REC;
	break;
      }
    }
    if( telem->stat == TELFRSKY_RD_BS ){
      // conv 7D 5E => 7E
      if( (tmp_telem = SerRead( ser )) == TELEM_FRSKY_BS10 ){
	/* SerBytesWrite( ser[UART_IOA], "!5E " ); */
	/* SerBytesWrite( ser[UART_IOA], "7D " ); */
	TelemFrskyDParseEncue( &(telem->parse), TELEM_FRSKY_BS10_ );
	telem->stat = TELFRSKY_REC;
      }
      // conv 7D 5D => 7D
      else if( tmp_telem == TELEM_FRSKY_BS11 ){
	/* SerBytesWrite( ser[UART_IOA], "!5D " ); */
	/* SerBytesWrite( ser[UART_IOA], "7E " ); */
	TelemFrskyDParseEncue( &(telem->parse), TELEM_FRSKY_BS11_ );
	telem->stat = TELFRSKY_REC;
      }
      else{
	/* SerBytesWrite( ser[UART_IOA], "FF " ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );	
	telem->stat = TELFRSKY_REC;
      }
    }
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
    if( telem->stat == TELFRSKY_SRC ){
      if( (tmp_telem = SerRead( ser_telem )) != TELEM_FRSKY_HEAD ){ continue; }
      else{
	SerBytesWrite( ser_dbg, "7E " );
	telem->stat = TELFRSKY_REC;
	break;
      }
    }
    // state: recognition (begin/end separator)
    if( telem->stat == TELFRSKY_REC ){
      // separator: begin
      if( (tmp_telem = SerRead( ser_telem )) != TELEM_FRSKY_TAIL ){
	sprintf( dbg_str, "%02X ", tmp_telem );
	SerBytesWrite( ser_dbg, dbg_str );
	telem->stat = TELFRSKY_RD;
	break;
      }
      // separator: end
      else{
	/* SerBytesWrite( ser_dbg, "7E " ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
      }
    }
    if( telem->stat == TELFRSKY_RD ){
      // data body (w/ bit stuffing)
      if( (tmp_telem = SerRead( ser_telem )) == TELEM_FRSKY_BS00 ){
	/* SerBytesWrite( ser_dbg, "!7D" ); */
	telem->stat = TELFRSKY_RD_BS;
	break;
      }
      // data body
      else if( tmp_telem != TELEM_FRSKY_TAIL ){
	/* sprintf( dbg_str, "%02X ", tmp_telem ); */
	/* SerBytesWrite( ser_dbg, dbg_str ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
      }
      // data end
      else{
	/* SerBytesWrite( ser_dbg, "7E \n\r" ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );
	telem->stat = TELFRSKY_REC;
	break;
      }
    }
    if( telem->stat == TELFRSKY_RD_BS ){
      // conv 7D 5E => 7E
      if( (tmp_telem = SerRead( ser_telem )) == TELEM_FRSKY_BS10 ){
	/* SerBytesWrite( ser_dbg, "!5E " ); */
	/* SerBytesWrite( ser_dbg, "7D " ); */
	TelemFrskyDParseEncue( &(telem->parse), TELEM_FRSKY_BS10_ );
	telem->stat = TELFRSKY_REC;
      }
      // conv 7D 5D => 7D
      else if( tmp_telem == TELEM_FRSKY_BS11 ){
	/* SerBytesWrite( ser_dbg, "!5D " ); */
	/* SerBytesWrite( ser_dbg, "7E " ); */
	TelemFrskyDParseEncue( &(telem->parse), TELEM_FRSKY_BS11_ );
	telem->stat = TELFRSKY_REC;
      }
      else{
	/* SerBytesWrite( ser_dbg, "FF " ); */
	TelemFrskyDParseEncue( &(telem->parse), tmp_telem );	
	telem->stat = TELFRSKY_REC;
      }
    }
  }
  return ( 0 );
}


/* end of telem_frsky.c */
