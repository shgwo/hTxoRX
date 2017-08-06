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
//    2017.08.05  mod difinition for telemetry & vbat mode
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
  OPMD_BOOT,  // app
  OPMD_DIAG,
  OPMD_SAFE,
  OPMD_RUN_INIT,
  OPMD_RUN,
  OPMD_FAIL,
  OPMD_UNKNOWN,
  OPMD_N
};
enum enum_AppModeLog {
  OPMD_LOG_OFF,
  OPMD_LOG_ON,
  OPMD_LOG_N
};
enum enum_AppModeTelm {
  OPMD_TELM_NFD,   // telemetry not found
  OPMD_TELM_ESTB,  // telemetry established
  OPMD_TELM_WEAK,  // established, signal is low
  OPMD_TELM_LOST,  // once, established & now lost
  OPMD_TELM_BATL,  // battery low
  OPMD_TELM_OFF,   // telemetry manually off
  OPMD_TELM_N
};

enum enum_AppModeBat {
  OPMD_BAT_DEAD,  // deadly low
  OPMD_BAT_LOW,   // low
  OPMD_BAT_MID,   // middle
  OPMD_BAT_FULL,  // full
  OPMD_BAT_N
};

enum enum_UARTApp {
  UART_MSP,   // main
  UART_IOA,   // Infotainment over air
  UART_TELEM, // telemetry
  UART_N_APP
};

#define UART_BRATE_MSP   115200
//#define UART_BRATE_MSP   9600
#define UART_BRATE_IOA   115200
#define UART_BRATE_TELEM 9600
//#define UART_BRATE_TELEM 4800

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
  // Telemetry modes
  enum enum_AppModeTelm opmd_telm;
  enum enum_AppModeTelm opmd_telm_old;
  uint8_t               opmd_telm_tran;
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
extern void hTxSetModeBat( st_hTx*, enum enum_AppModeBat );
extern void hTxSetModeTelm( st_hTx*, enum enum_AppModeTelm );
extern void hTxSetModeTelmTglOnOff( st_hTx* );

#endif
