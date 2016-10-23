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
#include "iodefine_enum.h"
#include "typedefine.h"
#include "adc_sar12b.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define PPM_N_CH  8   // number of CH on PPM signal

enum enum_PPMADJ{
  PPMADJ_NOINV,
  PPMADJ_INV
};

enum enum_PPMErr{
  PPMERR_NOERR,     // No Error: normal operation
  PPMERR_INIT,   // Error:    Initialization error
  PPMERR_START,  // Error:    PPM Gen Start error
  PPMERR_OP,     // Error:    PPM Gen self check error
  PPMERR_EXPT    // Error:    unknown
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_PPMAdj {
  char     name[10];  // id
  uint8_t  ch_adc;    // ch no. of adc connected to hw
  uint8_t  invert;    // data inversion requirement
  uint16_t offset;    // offset adjustment
  double   gain;      // gain adjustment
  double   expo;      // exponential adjustment
} st_PPMAdj;

typedef struct st_PPM {
  uint8_t    ir_vec;          // interrupt vector
  st_PPMAdj  adj[PPM_N_CH];   // output adjust setting
  uint16_t   data[PPM_N_CH];  // output data
  uint8_t    ch_vec;          // ch vector in ppm generation
  uint8_t    cnt_tail;          // ch vector in ppm generation
  uint8_t    cnt_end;          // ch vector in ppm generation
} st_PPM;

struct st_ppm_pref {
  // fclk w/ prescaler

  // resolution of ADC

  // coefficient of convert
};


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t PPMGenInit( void );
extern uint8_t PPMGenStart( void );
extern uint8_t PPMGenAdjInit( struct st_PPMAdj* , char*, uint8_t, enum enum_PPMADJ, uint16_t, double );
extern uint8_t PPMGen( st_PPM *, st_ADC12 * );

#endif
