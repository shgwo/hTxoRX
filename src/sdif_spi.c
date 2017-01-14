
#include "typedefine.h"
#include "sdif_spi.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  SDIF by SPI  utilities 
//
uint8_t SDIF_SPIPortInit( void ){
  // PC0, PC5, PC6, PC7, P15
  //  -> SD card access (SPI mode) Port setting (Rx/Tx)
  // ( PC0: CS1
  //   PC5: CLK
  //   PC6: MOSI
  //   PC7: MISO
  //   P15: Card detection, CD )
  // init
  PORTC.PCR.BIT.B0   = PCR_OPEN;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B5   = PCR_OPEN;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B6   = PCR_OPEN;         // PCR_OPEN / PCR_PULLUP
  PORTC.PCR.BIT.B7   = PCR_PULLUP;       // PCR_OPEN / PCR_PULLUP
  PORT1.PCR.BIT.B5   = PCR_PULLUP;       // PCR_OPEN / PCR_PULLUP
  PORTC.PODR.BIT.B0 = 1;
  PORTC.PODR.BIT.B5 = 0;
  PORTC.PODR.BIT.B6 = 0;
  PORTC.PDR.BIT.B0   = PDR_OUT;          // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B5   = PDR_OUT;          // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B6   = PDR_OUT;          // PDR_IN / PDR_OUT
  PORTC.PDR.BIT.B7   = PDR_IN;           // PDR_IN / PDR_OUT
  PORT1.PDR.BIT.B5   = PDR_IN;           // PDR_IN / PDR_OUT
  PORTC.PMR.BIT.B0   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B5   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B6   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B7   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORT1.PMR.BIT.B5   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  return( 0 );
}

uint8_t SDIF_SPIFuncInit( void ){
  // func
  MPC.PC0PFS.BIT.PSEL  = PC0PFS_SSLA1;
  MPC.PC5PFS.BIT.PSEL  = PC5PFS_RSPCKA;
  MPC.PC6PFS.BIT.PSEL  = PC6PFS_MOSIA;
  MPC.PC7PFS.BIT.PSEL  = PC7PFS_MISOA;
  PORTC.PMR.BIT.B0     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B5     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B6     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORTC.PMR.BIT.B7     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
}

// clocking to enter SPI IF mode
uint8_t SDIF_SPICardInit( st_SDSPI *sdspi ){
  SDIF_SPIPortInit();
  return( 0 );
}

// external initialize call
uint8_t SDIF_SPIInit( st_SDSPI *sdspi ){
  // Port init
  SDIF_SPIPortInit();
  SDIF_SPIFuncInit();

  // SPI mode trigger clocking

  
  // reset state & time count of pressing
  
  return( 0 );
}

/* end of sdif_spi.c */
