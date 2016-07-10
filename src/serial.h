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
//  last updated : 2016.07.06
//
//  history:
//    2016.07.06  create for RX63N( GR-SAKURA ) base-system
//    2016.04.16  add codes to operate TPU & ADC w/ synccronization
//                (interrupt codes are not included yet..) 
//
#ifndef __SERIAL_H__
#define __SERIAL_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"
#include "sysutil_RX63N.h"
#include "hTxoRX.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// label definition for HMI input source
enum enum_SERI_CHID {
  SER_SRV00,    // SCId ch0
  SER_SRV01,    // SCId ch1
  SER_SRV02,    // SCId ch2
  SER_SRV03,    // SCId ch3
  SER_SRV04,    // SCId ch4
  SER_HOST,     // SCId ch5
  SER_SRV06,    // SCId ch6
  SER_SRV07,    // SCId ch7
  SER_SRV08,    // SCId ch8
  SER_SRV09,    // SCId ch9
  SER_SRV10,    // SCId ch10
  SER_SRV11,    // SCId ch11
  SER_TELEM,    // SCId ch12
  SER_N_CH      // number of 
};

enum enum_SERI_STAT {
  SER_ORD,      // SCId ch10
  SER_BUSY,   // SCId ch12
  SER_ERR_TX,   // SCId ch11
  SER_ERR_RX,   // SCId ch12
  SER_N_STAT    // number of state
};

enum enum_SERI_ERR {
  SER_ERR_P,     // parity err
  SER_ERR_F,     // frame err
  SER_ERR_OR,    // over run err
  SER_N_ERR      // number of err state
};

#define SER_BUFF 256

// calc: N = 48 * 10^6 / ( 64 * 2^(2n-1) * 9600 ) - 1
#define BRR_N( BR, n )  48 * 1000000 / 64 / 


// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_SerialErr {
  uint8_t err[SER_N_ERR];
} st_SerialErr;

typedef struct st_Serial {
  struct st_sci0 *addr_base;
  uint8_t stat[SER_N_STAT];
  uint8_t stat_err[SER_N_ERR];
  uint8_t buff_tx[SER_BUFF];
  uint8_t buff_rx[SER_BUFF];
} st_Serial;


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t SerGenBRR( st_Serial*, uint32_t, uint8_t );
  
extern void SerStop( struct st_Serial* );
extern void SerStart( struct st_Serial* );
extern void SerPortInit( enum enum_SERI_CHID );
extern uint8_t SerFuncInit( struct st_Serial*, uint32_t );
extern uint8_t SerInit( struct st_Serial*, uint32_t, enum enum_SERI_CHID );
/* extern uint8_t SerGetStat( struct st_Serial* ); */

extern void Ser12Stop( void );
extern void Ser12Start( void );
extern void Ser12PortInit( void );
extern uint8_t Ser12FuncInit( struct st_Serial*, uint32_t );
extern void Ser12Init( struct st_Serial*, uint32_t );
/* extern uint8_t Ser12GetStat( struct st_Serial* ); */

#endif
