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
//  Ver.:          0.3
//  last updated : 2016.12.25
//
//  history:
//    2016.04.08  create for RX63N( GR-SAKURA ) base-system
//    2016.04.16  add codes to operate TPU & ADC w/ synccronization
//                (interrupt codes are not included yet..)
//    2016.12.25  add codes to migrate this modules
//                main-stream of PPM functions
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

enum enum_PPMADJMode{
  PPMADJ_ADJMD_LIN,   // linear adjstment
  PPMADJ_ADJMD_EXPO,  // exponential adj.
  PPMADJ_ADJMD_LUT,   // lut adj. 
  PPMADJ_ADJMD_N
};

enum enum_PPMADJModeInv{
  PPMADJ_NOINV,
  PPMADJ_INV
};

enum enum_PPMADJModeLock{
  PPMADJ_UNLOCK,
  PPMADJ_LOCK
};

enum enum_PPMErr{
  PPMERR_NOERR,     // No Error: normal operation
  PPMERR_INIT,   // Error:    Initialization error
  PPMERR_START,  // Error:    PPM Gen Start error
  PPMERR_OP,     // Error:    PPM Gen self check error
  PPMERR_EXPT    // Error:    unknown
};

enum enum_PPMMode{
  PPM_OPMD_NORM, // normal opration
  PPM_OPMD_LOCK, // locking
  PPM_OPMD_DEGR, // degrading
  PPM_OPMD_N
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_PPMAdj {
  char                     name[10];  // id
  enum enum_PPMADJModeLock lock;      // data force flag
  uint16_t                 lckdat;    // force data
  uint8_t                  ch_adc;    // ch no. of adc connected to hw
  enum enum_PPMADJModeInv  invert;    // data inversion (neg-logic) requirement
  enum enum_PPMADJMode     mode;      // adjustment mode
  double                   gain;      // gain adjustment
  uint16_t                 offset;    // offset adjustment
  double                   expo;      // exponential adjustment
} st_PPMAdj;

typedef struct st_PPM {
  enum enum_PPMMode opmd;       // operation mode
  st_PPMAdj    adj[PPM_N_CH];   // output adjust setting
  uint16_t     data[PPM_N_CH];  // output data
  uint8_t      ch_vec;          // ch vector in ppm generation
  uint8_t      cnt_tail;        // tail pulse conut in ppm generation
  uint8_t      cnt_end;         // train count in ppm generation
  uint8_t      td_loop;         // measurement time for each PPM loop
  uint8_t      ir_vec;          // interrupt vector
} st_PPM;

struct st_ppm_pref {
  // fclk w/ prescaler

  // resolution of ADC

  // coefficient of conversion
};


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t PPMGenInit( void );
extern uint8_t PPMGenStart( void );
extern uint8_t PPMGenAdjInit( struct st_PPMAdj* , char*, uint8_t, enum enum_PPMADJModeInv, uint16_t, double );
extern uint8_t PPMGen( st_PPM *, st_ADC12 * );
extern uint8_t PPMGenSetVal( st_PPM *, uint8_t, uint16_t );
extern uint8_t PPMGenInputFilter( st_PPM *, st_ADC12 * );

#endif
