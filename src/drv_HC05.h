// -------------------------------------------------------
// ------------------------------------------------ Notice
//  This program is distributed or redistributed to
//  the world under the License of MIT.
//
// -------------------------------------------------------
// ------------------------------------------------- Info.
//
//  Description:   Relative utilities for HC-05 UART BT(SPP) module
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2016.10.22
//
//  history:
//    2016.10.22  create for hTxoRX system
//    2016.mm.dd  xxxx
//                
//
#ifndef __DRV_HC05_H__
#define __DRV_HC05_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "serial.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines

// label definition for HMI input source
// for BT/UART dongle command 
enum enum_SER_HC05 {
  SER_HC05_OK,    // AT
  SER_HC05_VER,   // AT+VERSION Linvor1.5
  SER_HC05_BAUD,  // AT+BAUDx (x: 1 - C )
                  //  (    1200 /   2400 /   4800 /    9600
                  //   /  19200 /  38400 /  53600 /  115200
                  //   / 230400 / 460800 / 921600 / 1382400 )
  SER_HC05_NAME,  // AT+NAME[String]
  SER_HC05_PIN,   // AT+PIN[PIN #]
  SER_N_HC05      // number of err state
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_drvHC05Err {
  struct st_sci0_ssr *addr_stat_base;
  uint8_t err[SER_N_ERR];
} st_drvHC05Err;

typedef struct st_drvHC05 {
  uint8_t proc; // indicates procedure step #
} st_drvHC05;

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern void drvHC05Init( struct st_drvHC05* );
extern void drvHC05ConfBaud( struct st_drvHC05* );

#endif
