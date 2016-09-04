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
  OPMD_BOOT,
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
  UART_TELEM,
  UART_N_APP
};

#define UART_BRATE_MSP   115200
//#define UART_BRATE_MSP   9600
#define UART_BRATE_TELEM 9600
//sssssssssssssssssssss#define UART_BRATE_TELEM 4800

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_hTx {
  // Main modes
  enum enum_AppMode    opmd;
  enum enum_AppMode    opmd_old;
  uint8_t              opmd_tran;
  // Log modes
  enum enum_AppModeLog opmd_log;
  enum enum_AppModeLog opmd_log_old;
  uint8_t              opmd_log_tran;
  // Battery states
  enum enum_AppModeBat opmd_bat;
  enum enum_AppModeBat opmd_bat_old;
  uint8_t              opmd_bat_tran;
} st_hTx;

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern void hTxInit( st_hTx* );
extern void hTxSetMode( st_hTx*, enum enum_AppMode );
extern void hTxSetModeLogOff( st_hTx* );
extern void hTxSetModeLogOn( st_hTx* );

#endif
