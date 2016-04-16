// -------------------------------------------------------
// ------------------------------------------------ Notice
//  This program is distributed to the world under
//  LICENSE of the GPL.
//
// -------------------------------------------------------
// ------------------------------------------------- Info.
//
//  Description:   Main routine of DiyTx system for my FPV Quad
//                 (w/ non-disclosure Reg maps)
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2016.04.08
//
//  history:
//    2016.04.08  create for RX63N( GR-SAKURA ) base-system
//

#include "rx63n/iodefine.h"
#include "rx63n/iodefine_enum.h"

#define sleep(X) for(j = 0; j < X*1000; j++) {}

int main( void )
{
  char i=0;
  int j, state_disp=0;
  unsigned int start_cnt = 0;
  int val_aux1 = 6;
  int val_an[4];
  
  //
  // %%add%% init IO Port
  //
  // < Available IO Ports @ RX63N 100pin > *under construction
  //   PORT0 [P03, P05]                  (x2)
  //   PORT1 [P14 -> P17]                (x4)
  //   PORT2 [P26, P27]                  (x2)
  //   PORT3 [P30 -> P32, P35]           (x4)
  //   PORT4 [P40 -> P44, P46]           (x6)
  //   PORT5 [P54, P55]                  (x2)
  //   PORTA [PA0, PA1, PA3, PA4, PA6]   (x5)
  //   PORTB [PB0, PB1, PB3, PB5 -> PB7] (x5)
  //   PORTC [PC2 -> PC7]                (x6)
  //   PORTE [PE0 -> PE7]                (x8)
  //   PORTH [PH0 -> PH3 *PH6, *PH7]     (x4(*6))
  //   PORTJ [PJ3, PJ5]                  (x(*2))

  // IO Port setting
  MPCUnlock();
  // PA0-2,6 -> LED indicator
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
  
  // P4 -> ADC input (3.3V)
  MPC.P40PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P40PFS.BIT.ASEL  = ASEL_ON;
  MPC.P41PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P41PFS.BIT.ASEL  = ASEL_ON;
  MPC.P42PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P42PFS.BIT.ASEL  = ASEL_ON;
  MPC.P43PFS.BIT.ISEL  = ISEL_OFF;
  MPC.P43PFS.BIT.ASEL  = ASEL_ON;
  PORT4.PMR.BIT.B0 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B1 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B2 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORT4.PMR.BIT.B4 = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  
  // PJ3 -> PPM output (3.3V)
  MPC.PJ3PFS.BIT.PSEL  = PJ3PFS_MTIOC3C;
  //MPC.PJ3PFS.BIT.PSEL  = PJ3PFS_CSI0;
  PORTJ.PMR.BIT.B3     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  MPCLock();

  //
  
  // test (primitive ppm)
  PORTJ.PODR.BIT.B3    = 1;
  PORTJ.PDR.BIT.B3     = PDR_OUT;       // PMR_GPIO / PMR_FUNC
  PORTJ.PMR.BIT.B3     = PMR_GPIO;       // PMR_GPIO / PMR_FUNC
  
  // MTU setting (after, to be functional to another file )
  MTU3.TCR.BIT.TPSC = TPSC_PCLK_16;  // PCLK/16
  MTU3.TCR.BIT.CKEG = CKEG_PEDGE;    // Pos edge
  MTU3.TCR.BIT.CCLR = CCLR_TGRC;     // Both edge
  MTU3.TMDR.BIT.MD =  TMDR_MD_NORMAL;     // Normal mode
  MTU3.TMDR.BIT.BFA = TMDR_BFx_BUFF;     // Buffered operation
  MTU3.TMDR.BIT.BFB = TMDR_BFx_NORM;     // unbuffered	


  
  // ADC setting
  //S12ADC.ADCSR.
	  
  // Total frame length = 22.5msec
  // each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  // The pulse ISR is 2mhz that's why everything is multiplied by 2
	  
  //
  while(1) {
    // ch0
    sleep(9);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch1
    sleep(9);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    
    // ch2 (throttle)
    sleep(8);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch3
    sleep(9);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch4
    sleep(val_aux1);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch5
    sleep(10);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch6
    sleep(10);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // ch7
    sleep(10);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    // sync
    sleep(50);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;
    sleep(3);
    PORTJ.PODR.BIT.B3 = !PORTJ.PODR.BIT.B3;

    if( PORTA.PIDR.BIT.B7 == 0 ){
      /* Disp LED according to AN0 */
      unsigned char test = MPC.PJ3PFS.BIT.PSEL;
      //      test = MPC.PWPR.BYTE;
      DataDisp( test );
    }
    else{
      PORTA.PODR.BYTE = i++ & (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
      /* Set GPIOs according to i */
      //PORTA.PODR.BYTE = i++ & (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
    }

        if( start_cnt == 10000000 ){ val_aux1 = 10; }
        start_cnt++;
  }
  
}


// -------------------------------------------------------
// -------------------------------- Functions( Utilities )

//
//  disp data on LED ( mounted on Eval Brd. )
//
void DataDisp( unsigned char data ){
  PORTA.PDR.BYTE = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 6);
  PORTA.PODR.BYTE = 0x00;
  
  PORTA.PODR.BYTE   = ( (data >> 0) & 0x07 );
  PORTA.PODR.BIT.B6 = ( (data >> 6) & 0x01 );
}

//
//  lock / unlock MPC reg edit
//
void MPCUnlock( void ){
  MPC.PWPR.BIT.B0WI  = B0WI_UNLOCK;
  MPC.PWPR.BIT.PFSWE = PFSWE_UNLOCK;
}


void MPCLock( void ){
  MPC.PWPR.BIT.PFSWE = PFSWE_LOCK;
  MPC.PWPR.BIT.B0WI  = B0WI_LOCK;
}
