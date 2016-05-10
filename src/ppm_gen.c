
#include "ppm_gen.h"
#include "rx63n/typedefine.h"

#define PPMGEN_PNUM 8

int PPMGenAdjInit( struct st_PPMAdj *ppm_adj , char *name, uint8_t ch_adc, enum enum_PPMADJ inv, uint16_t offset, double gain){
  int i;
  // copy name
  for( i=0 ; i < 10 ; i++ ){
    ppm_adj->name[i] = name[i];
  }
  // set ad_ch
  ppm_adj->ch_adc = ch_adc;
  // set invert
  ppm_adj->invert = inv;
  // set offset
  ppm_adj->offset = offset;
  ppm_adj->gain = gain;

  return(0);
}

// initialize unit convert constant
double PPMGenInit( double f_clk, uint8_t n_prs ){
  double k_us2tcnt;

  return( k_us2tcnt );
}

// unit conversion from time space to TGRx count space 
uint16_t PPMGenms2Tcnt( double ms, uint32_t f_clk ){
  uint16_t tcnt = 0;  

  tcnt = (uint16_t)( ms * f_clk);
  // calc time resolution:  1/(48M / 2^3) = 1/6 u = 166...ns
  // calc max time length:  2^16 / (48M / 2^3) = 1/(3*2^4) 2^19 us = 1/3 * 2^5 *1024 us = 32/3 * 1024 us = ~10ms
  //TPU3.TGRA            = 4200;     // 700u * 6.0M = 4200 ()
  //TPU3.TGRC            = 1800;     // 300u * 6.0M = 1800 ()

  return ( tcnt );
}

uint16_t PPMGenAD2Timer( uint16_t *bin_adc, uint8_t bit_adc, uint8_t bit_timer ){
  uint16_t tcnt = 0;
  
  return ( tcnt );
}

void PPMGen( uint16_t *bin_adc, uint8_t bit_adc, uint8_t bit_timer, uint8_t n_pulse, double td_prepls, double td_pulse, double td_tail, double td_post ){
  int i;
  uint16_t tcnt_pulses[PPMGEN_PNUM];

  // gen tcnt (td) array for each ppm pulse train
  for ( i=0 ; i < n_pulse ; i++ ){
    tcnt_pulses[i] = PPMGenms2Tcnt( 0.0007, 6000000 ) + PPMGenAD2Timer( bin_adc[i], bit_adc, bit_timer );
  }

  // PPM generation
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  // 8 times { H: init(0.7m)+(0 - 1.0 m) -> stop tail(0.3m) -> H: next... } + frame tail (H: 6.2m L: 0.3m)
  // requirements: time resolution < 1ms / 11bit   = 1m / 2^11 =~ 0.5u,
  //               max time length > 1ms           


  return;
}
