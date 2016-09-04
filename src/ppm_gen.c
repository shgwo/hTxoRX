
#include "rx63n/typedefine.h"
#include "ppm_gen.h"
#include "adc_sar12b.h"

#define PPMGEN_PNUM 8

uint8_t PPMGenAdjInit( struct st_PPMAdj *ppm_adj , char *name, uint8_t ch_adc, enum enum_PPMADJ inv, uint16_t offset, double gain){
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
  // set gain
  ppm_adj->gain = gain;

  return(0);
}

uint8_t PPMGenAdjInitAll( struct st_PPMAdj *ppm_adj , char *name, uint8_t ch_adc, enum enum_PPMADJ inv, uint16_t offset, double gain){
  return(0);
}

uint8_t PPMGenPortInit_RX63N( void ){
  // P21 -> PPM output (3.3V pulse out)
  /* future func => PortConfMPC( PDR_OUT, 0, PMR_FUNC, P21PFS_TIOCA3 ); */
  PORT2.PODR.BIT.B1  = 1;
  PORT2.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT
  PORT2.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
}

// initialize unit convert constant
uint8_t PPMGenConf_RX63N( void ){
  // PPM generation
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  //   -> 8 times { H: init(0.7m) + variable(0 - 1.0 m) + stop tail(0.3m)
  //   -> frame tail (H: 6.2m L: 0.3m)
  //   -> Go next frame (8 times) repetitively ...
  // requirements: time resolution < 1ms / 11bit   = 1m / 2^11 =~ 0.5u,
  //               max time length > 1ms           
  // calc time resolution:  1/(48M / 2^3) = 1/6 u = 166...ns
  // calc max time length:  2^16 / (48M / 2^3) = 1/(3*2^4) 2^19 us = 1/3 * 2^5 *1024 us = 32/3 * 1024 us = ~10ms
  
  /* future func => PortConfMPC( PDR_OUT, 0, PMR_FUNC, P21PFS_TIOCA3 ); */
  // TPUa setting (for PPM)
  TPUA.TSTR.BIT.CST3   = CSTn_STOP;           // stop: TPU3
  TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_16;  // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz
  TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE;       // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz
  //TPU3.TCR.BIT.TPSC    = TPU39_TPSC_PCLK_4;   // (12Mhz x 4) / 2^4 -> 12M/2^2 = 3.0 MHz (high-reso test)
  //TPU3.TCR.BIT.CKEG    = TPU_CKEG_EDGE_IP_EN;  // (12Mhz x 4) / 2^3 -> 12M/2   = 6.0 MHz (high-reso test)
  TPU3.TCR.BIT.CCLR    = TPU_CCLR_TGRA;      // TCNT cleared by TGRA
  TPU3.TMDR.BIT.MD     = TPU_MD_NORM;        // normal mode
  TPU3.TMDR.BIT.BFA    = TMDR_BFx_BUFF;      // buffer operation
  TPU3.TMDR.BIT.BFB    = TMDR_BFx_NORM;      // buffer operation
  TPU3.TMDR.BIT.ICSELB = TPU_ICSELB_TIOCBn;  // ch B (unused)
  TPU3.TMDR.BIT.ICSELD = TPU_ICSELD_TIOCDn;  // ch D (unused)
  TPU3.TIORH.BIT.IOA   = IOX_OHCT;           // TIOCAn
  TPU3.TIORH.BIT.IOB   = IOX_DE;             // disable
  TPU3.TIORL.BIT.IOC   = IOX_DE;             // disable
  TPU3.TIORL.BIT.IOD   = IOX_DE;             // disable
  TPU3.TIER.BIT.TGIEA  = TGIEX_EN;           // IRQ enable (temp DE)
  TPU3.TIER.BIT.TTGE   = TTGE_EN;            // TTGE[EN / DE] enable: ADC start

  // P21 -> PPM output (3.3V pulse out)
  /* future func => PortConfMPC( PDR_OUT, 0, PMR_FUNC, P21PFS_TIOCA3 ); */
  MPC.P21PFS.BIT.PSEL  = P21PFS_TIOCA3;
  PORT2.PMR.BIT.B1     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  
  return( 0 );
}

uint8_t PPMGenInit( void ){
  PPMGenPortInit_RX63N( );
  PPMGenConf_RX63N( );

  return( PPMERR_NOERR );
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

uint8_t PPMGenStart( void ){
  // ADC12 sync
  ADC12IRQOff();
  // PPM generation
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  //   -> 8 times { H: init(0.7m) + variable(0 - 1.0 m) + stop tail(0.3m)
  //   -> frame tail (H: 6.2m L: 0.3m)
  //   -> Go next frame (8 times) repetitively ...
  // requirements: time resolution < 1ms / 11bit   = 1m / 2^11 =~ 0.5u,
  //               max time length > 1ms           
  // calc time resolution:  1/(48M / 2^3) = 1/6 u = 166...ns
  // calc max time length:  2^16 / (48M / 2^3) = 1/(3*2^4) 2^19 us = 1/3 * 2^5 *1024 us = 32/3 * 1024 us = ~10ms
  TPU3.TGRA            = 4200;     //  700u * 6.0M = 4200 ()
  TPU3.TGRC            = 1800;     //  300u * 6.0M = 1800 ()
  TPU3.TGRD            = 37200;    // 6200u * 6.0M = 37200 ()

  IEN( TPU3, TGI3A ) = 0;
  IPR( TPU3, TGI3A ) = 3;
  IR( TPU3, TGI3A )  = 0;
  TPUA.TSTR.BIT.CST3   = CSTn_RUN;   // start to run timer pulse
  
  if( TPUA.TSTR.BIT.CST3 != CSTn_RUN ){
    return( PPMERR_START );
  };   // start to run timer pulse )
  
  return( PPMERR_NOERR );
}

uint8_t PPMGenInputFilter( st_PPM *ppm, st_ADC12 *adc12 ){
    // temp values
    uint16_t dat_tmp = 0;
    uint8_t  ch_adc = 0;
    double   gain   = 1.0;
    uint16_t off    = 0;
    uint16_t inv    = 0;

  for( uint8_t i=0 ; i<PPM_N_CH ; i++ ){
    dat_tmp = 0;
    ch_adc  = ppm->adj[i].ch_adc;
    gain    = ppm->adj[i].gain;
    off     = ppm->adj[i].offset;
    inv     = ppm->adj[i].invert;
    // invert & shifting ( inv/not inv, flush-left 14bit -> flush-right 14bit)
    dat_tmp = ( inv? (~adc12->data[ch_adc] >> 2) : (adc12->data[ch_adc] >> 2) );
    dat_tmp = 0x0000;
    // linear adjustment ( gradient & intercept )
    ppm->data[i] = (uint16_t)( gain * dat_tmp + off );
  }
  return (0);
}


uint8_t PPMGen( st_PPM *ppm, st_ADC12 *adc12 ){
  // Input data generation core
  ADC12GetVal( adc12 );
  PPMGenInputFilter( ppm, adc12 );
  // PPM generation core
  // check tpu renew timing from occurance flag of interrupt
  // each pulse is 0.7..1.7ms length with a additional 0.3ms stop tail
  //   -> 8 times { H: init(0.7m) + variable(0 - 1.0 m) + L: stop tail(0.3m)
  //   -> frame tail (H: 6.2m + L: 0.3m)
  //   -> Go next frame (8 times) repetitively ...
  if( IR( TPU3, TGI3A ) == 1 ){
    // tail pulse set
    if( TPU3.TGRA > (400 * 6) ){
      TPU3.TGRC = (300 * 6);
      ppm->vect++;
      ppm->cnt_tail++;
      if( ppm->vect > PPM_N_CH ){ ppm->vect = 0; }
    }
    // ppm pulse set
    else{
      // end pulse set
      if( ppm->vect >= PPM_N_CH ){
	TPU3.TGRC = (6200 * 6);
	ppm->cnt_end++;
	// debug
	/* PORTA.PODR.BIT.B0 = !PORTA.PODR.BIT.B0; */
      }
      // significant pulse
      else{
	TPU3.TGRC = (700 * 6) + ppm->data[ppm->vect];
      }
    }
    IR( TPU3, TGI3A ) = 0;
    /* PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3; */
  }
  
  // PPM generation
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  //   -> 8 times { H: init(0.7m) + variable(0 - 1.0 m) + stop tail(0.3m)
  //   -> frame tail (H: 6.2m L: 0.3m)
  //   -> Go next frame (8 times) repetitively ...
  // requirements: time resolution < 1ms / 11bit   = 1m / 2^11 =~ 0.5u,
  //               max time length > 1ms           
  // calc time resolution:  1/(48M / 2^3) = 1/6 u = 166...ns
  // calc max time length:  2^16 / (48M / 2^3) = 1/(3*2^4) 2^19 us = 1/3 * 2^5 *1024 us = 32/3 * 1024 us = ~10ms
  TPU3.TGRA            = 4200;     //  700u * 6.0M = 4200 ()
  TPU3.TGRC            = 1800;     //  300u * 6.0M = 1800 ()
  TPU3.TGRD            = 37200;    // 6200u * 6.0M = 37200 ()
  //IR(TPU3, TGI3A) = 1;
  //IR(S12AD, S12ADI0) = 1;

  IEN( S12AD, S12ADI0 ) = 0;
  IPR( S12AD, S12ADI0 ) = 4;  
}

void PPMGenTest( uint16_t *bin_adc, uint8_t bit_adc, uint8_t bit_timer, uint8_t n_pulse, double td_prepls, double td_pulse, double td_tail, double td_post ){
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
