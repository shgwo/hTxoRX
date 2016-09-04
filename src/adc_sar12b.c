
#include "iodefine_enum.h"
#include "typedefine.h"
#include "sysutil_RX63N.h"
#include "adc_sar12b.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  HMI utilities 
//

uint8_t ADC12PortInit( void ){
  // ADC input (Gimbal stick input)
  // P4 -> ADC input from potentio meters in stick gimbal (Vref 3.3V)
  /* PortConfADC( MPC.P40PFS, ASEL_ON, ISEL_OFF, PORT4, 0, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P41PFS, ASEL_ON, ISEL_OFF, PORT4, 1, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P42PFS, ASEL_ON, ISEL_OFF, PORT4, 2, PMR_FUNC, PDR_IN, 0 ); */
  /* PortConfADC( MPC.P43PFS, ASEL_ON, ISEL_OFF, PORT4, 3, PMR_FUNC, PDR_IN, 0 ); */
  PORT4.PDR.BIT.B0 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B1 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B2 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B3 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PDR.BIT.B5 = PDR_IN;         // PDR_IN / PDR_OUT
  PORT4.PMR.BIT.B0 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B3 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B5 = PMR_GPIO;       // PMR_GPIO / PMR_FUNC

  return( 0 );
}

uint8_t ADC12FuncInit( void ){
  // ADC setting (to do: librarize setting process)
  S12AD.ADCSR.BIT.ADST  = ADST_STOP;   // ADST_START / ADST_STOP
  // scan ch setting ( ch select, addition )
  S12AD.ADANS0.WORD     = 0x202F;   // (b0010 0000 0010 1111) AN015 - AN000
  S12AD.ADANS1.WORD     = 0x0000;   // (b---- ---- ---0 0000) AN020 - AN016,
                                    //   |----> "-" is fixed value as 0 
  S12AD.ADADC.BIT.ADC   = ADADC_X4;   // ADADC_X1 / ADADC_X2 / ADADC_X3 / ADADC_X4
  S12AD.ADADS0.WORD     = 0x202F;   // (b0010 0000 0010 1111) AN015 - AN000
  S12AD.ADADS1.WORD     = 0x0000;   // (b---- ---- ---0 0000) AN020 - AN016,
                                    //   |----> "-" is fixed value as 0
  // ADC config
  S12AD.ADCER.BIT.ACE      = ACE_DE;          // ACE_DE / ACE_EN
  S12AD.ADCER.BIT.ADRFMT   = ADRFMT_FLLEFT;   // ADRFMT_FLRIGHT / ADRFMT_FLLEFT
  S12AD.ADSTRGR.BIT.ADSTRS = ADSTRS_TRGAN_1;  // TPUn.TGRA
  S12AD.ADSSTR01.BIT.SST1  = 0x18;            // A/D Sampling State Register 01
                                             // (PCLK 48MHz -> 48/2 steps, 1u/2 = 500ns)
  S12AD.ADSSTR23.BIT.SST2  = 0xFF;           // A/D Sampling State Register 23 (Temp sensor)
  S12AD.ADCSR.BIT.CKS     = CKS_PCLK_2;      // CKS_PCLK_8 / CKS_PCLK_4 / CKS_PCLK_2 / CKS_PCLK
  S12AD.ADCSR.BIT.ADIE    = ADIE_EN;         // ADIE_DE / ADIE_EN
  S12AD.ADCSR.BIT.EXTRG   = EXTRG_SYNC;      // EXTRG_SYNC / EXTRG_ASYN
  S12AD.ADCSR.BIT.TRGE    = TRGE_EN;         // TRGE_EN / TRGE_DE
  S12AD.ADCSR.BIT.ADCS    = ADCS_SINGLE;     // ADCS_SINGLE / ADCS_CONT
  S12AD.ADCSR.BIT.ADST    = ADST_STOP;       // ADST_START / ADST_STOP

  // switch IO to peripheral func.
  MPC.P40PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P40PFS.BIT.ASEL  = ASEL_ON;
  MPC.P41PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P41PFS.BIT.ASEL  = ASEL_ON;
  MPC.P42PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P42PFS.BIT.ASEL  = ASEL_ON;
  MPC.P43PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P43PFS.BIT.ASEL  = ASEL_ON;
  MPC.P45PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P45PFS.BIT.ASEL  = ASEL_ON;
  PORT4.PMR.BIT.B0 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B3 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B5 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  
  return( 0 );
}

uint8_t ADC12Init( void ){
  ADC12PortInit();
  ADC12FuncInit();
  return( 0 );
}

void ADC12Stop( void ){
  // ADC IRQ setting (Off)
  IEN( S12AD, S12ADI0 ) = 0;
  // ADC setting (to do: librarize setting process)
  S12AD.ADCSR.BIT.ADST  = ADST_STOP;   // ADST_START / ADST_STOP
}

void ADC12IRQOn( void ){
  // ADC setting (to do: librarize setting process)
  S12AD.ADCSR.BIT.ADST  = ADST_STOP;   // ADST_START / ADST_STOP
  
  // ADC IRQ setting (On)
  IEN( S12AD, S12ADI0 ) = 1;
  IPR( S12AD, S12ADI0 ) = 4;
  // ADC IRQ flag clear
  IR( S12AD, S12ADI0 ) = 0;
}

void ADC12IRQOff( void ){
  // ADC setting (to do: librarize setting process)
  S12AD.ADCSR.BIT.ADST  = ADST_STOP;   // ADST_START / ADST_STOP
  
  // ADC IRQ setting (Off)
  IEN( S12AD, S12ADI0 ) = 0;
  IPR( S12AD, S12ADI0 ) = 4;
  // ADC IRQ flag clear
  IR( S12AD, S12ADI0 ) = 0;
}

void ADC12AutoStart( void ){
  S12AD.ADCSR.BIT.ADST = ADST_START;       // ADST_START / ADST_STOP  
}

uint8_t ADC12GetVal( st_ADC12 *adc12 ){
  // get ADC val & arrange for PPM ch
  // future func => PPMGenInputSelector()
  if( IR( S12AD, S12ADI0) == 1 ){

    /* st_adc12 *base = 0x00089020; */
    /* for( int i=0 ; i < PPM_N_CH ; i++ ){ */
    /*   adc12->adc_val[i] = base++; */
    /* } */
    // all stores
    adc12->data[ 0] = S12AD.ADDR0;
    adc12->data[ 1] = S12AD.ADDR1;
    adc12->data[ 2] = S12AD.ADDR2;
    adc12->data[ 3] = S12AD.ADDR3;
    adc12->data[ 4] = S12AD.ADDR4;
    adc12->data[ 5] = S12AD.ADDR5;
    adc12->data[ 6] = S12AD.ADDR6;
    adc12->data[ 7] = S12AD.ADDR7;
    adc12->data[ 8] = S12AD.ADDR8;
    adc12->data[ 9] = S12AD.ADDR9;
    adc12->data[10] = S12AD.ADDR10;
    adc12->data[11] = S12AD.ADDR11;
    adc12->data[12] = S12AD.ADDR12;
    adc12->data[13] = S12AD.ADDR13;
    adc12->data[14] = S12AD.ADDR14;
    adc12->data[15] = S12AD.ADDR15;
    adc12->data[16] = S12AD.ADDR16;
    adc12->data[17] = S12AD.ADDR17;
    adc12->data[18] = S12AD.ADDR18;
    adc12->data[19] = S12AD.ADDR19;
    adc12->data[20] = S12AD.ADDR20;

    IR( S12AD, S12ADI0 ) = 0;
  }
  
}


/* end of adc_sar12b */
