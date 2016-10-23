
#include "iodefine_enum.h"
#include "typedefine.h"
#include "sysutil_RX63N.h"
#include "dbg_utils.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  disp data on LED ( mounted on Eval Brd. )
//
void dbgDispLEDPortInit( void ){
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
}

void dbgDispLEDInit( void ){
  dbgDispLEDPortInit();
}

void dbgDispLED( uint8_t data ){
  // port setting ( I/O OUT, Low )
  PORTA.PODR.BYTE = 0x00;  
  PORTA.PDR.BYTE = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
  
  PORTA.PODR.BYTE   = ( (data >> 0) & 0x07 );
  PORTA.PODR.BIT.B6 = ( (data >> 3) & 0x01 );
}

void dbgDispLEDbit( uint8_t bit, uint8_t port ){
  // port setting ( I/O OUT, Low )
  PORTA.PODR.BYTE = 0x00;  
  PORTA.PDR.BYTE = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);

  if( port < 2 )
    PORTA.PODR.BYTE = ( (bit & 0x01) << port );
  else if( port == 3 )
    PORTA.PODR.BYTE = ( (bit & 0x01) << 6 );
}

void dbgPulseOutInit( void ){
  // PJ3 debug out  (for probing by oscilloscopes )
  PORTJ.PODR.BIT.B3    = 1;
  PORTJ.PDR.BIT.B3     = PDR_OUT;       // PMR_GPIO / PMR_FUNC
  PORTJ.PMR.BIT.B3     = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
}

void dbgPulseOutConf( void ){
}

/* end of dgb_led.c */
