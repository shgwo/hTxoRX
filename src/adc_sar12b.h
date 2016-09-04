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
//  Description:   Driver libraries for 12bit SAR-ADC input
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.5
//  last updated : 2016.08.17
//
//  history:
//    2016.08.17  port from main code to compose as library
//    2016.mm.dd  
//
#ifndef __ADC_SAR12B_H__
#define __ADC_SAR12B_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
/* #define NCH_ADC 21   // number of AD channel on the HW (all ch on RX63N) */
#define NCH_ADC 21   // number of AD channel (used)
#define ADC12_N_CH 21   // number of AD channel (used)

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_ADC12 {
  uint16_t data[ADC12_N_CH];
} st_ADC12;

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t ADC12Init( void );
extern void ADC12IRQOn( void );
extern void ADC12IRQOff( void );
extern void ADC12AutoStart( void );

extern uint8_t ADC12GetVal( st_ADC12 * );


#endif
