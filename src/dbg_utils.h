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
//  Description:   Relative utilities for debug IO
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2016.08.17
//
//  history:
//    2016.08.17  port from main code for RX63N( GR-SAKURA ) base-system
//                
//
#ifndef __DBG_UTILS_H__
#define __DBG_UTILS_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// label definition
//#define HMI_LED_FB_LEN 80   // cycle ( * CMT1 cycle )

// -------------------------------------------------------
// ----------------------------------------------- Structs


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern void dbgDispLEDInit( void );
extern void dbgDispLED( uint8_t );
extern void dbgDispLEDbit( uint8_t, uint8_t );

extern void dbgPulseOutInit( void );


#endif
