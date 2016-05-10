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
//  Description:   Relative utilities for PPM generation
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
#ifndef __PPM_GEN_H__
#define __PPM_GEN_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "typedefine.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define NCH_PPM  8   // number of CH on PPM signal

enum enum_PPMADJ{
  PPMADJ_NOINV,
  PPMADJ_INV
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
struct st_PPMAdj {
  char     name[10];
  uint8_t  ch_adc;
  uint8_t  invert;
  uint16_t offset;
  double   gain;
};


struct st_ppm_pref {
  // fclk w/ prescaler

  // resolution of ADC

  // coefficient of convert
};


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern int PPMGenAdjInit( struct st_PPMAdj* , char*, uint8_t, enum enum_PPMADJ, uint16_t, double );
extern uint16_t ms2cnt( double, uint32_t );

#endif
