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
//  Description:   Relative utilities for SD-card I/O using SPI mode
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2016.06.05
//
//  history:
//    2016.06.05  create for RX63N( GR-SAKURA ) base-system
//
//
#ifndef __SDIF_SPI_H__
#define __SDIF_SPI_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"
#include "hTxoRX.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// label definition for XX


// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_SDData {
  char     name[10];
  uint8_t  ch_adc;
  uint8_t  invert;
  uint16_t offset;
  double   gain;
} st_SDData;


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t SDIF_SPIInit( struct st_SDData* );

#endif
