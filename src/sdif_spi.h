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
#define RSPI_CH 0

enum enum_SDSPIMode{
  SDSPI_MD_UNKNOWN, // normal opration
  SDSPI_MD_PRECLK,  // normal opration
  SDSPI_MD_RSTCMD,  // normal opration
  SDSPI_MD_N
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_SDSPI {
  enum enum_SDSPIMode md;
} st_SDSPI;


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t SDIF_SPIInit( struct st_SDSPI* );

#endif
