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
#ifndef __HTXORX_H__
#define __HTXORX_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "typedefine.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// state difinition for main operation
enum enum_AppMode {
  OPMD_INIT,
  OPMD_DIAG,
  OPMD_SAFE,
  OPMD_RUN_INIT,
  OPMD_RUN,
  OPMD_FAIL,
  OPMD_UNKNOWN
};
enum enum_AppModeLog {
  OPMD_LOG_OFF,
  OPMD_LOG_ON
};
enum enum_AppModeBat {
  OPMD_BAT_LOW,
  OPMD_BAT_MID,
  OPMD_BAT_FULL
};

enum enum_UARTApp {
  UART_MSP,
  UART_TELEMETRY,
  UART_N_APP
};

#define UART_BRATE_MSP   115200
#define UART_BRATE_TELEM 9600

// -------------------------------------------------------
// ----------------------------------------------- Structs



// -------------------------------------------------------
// -------------------------------- Proto-type declaration

#endif
