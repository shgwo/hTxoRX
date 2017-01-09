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
//  Description:   Relative utilities for UART comm.
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.2
//  last updated : 2017.01.09
//
//  history:
//    2016.06.XX  create for RX63N( GR-SAKURA ) base-system
//                add codes to use as debug com (via FT232)
//    2016.09.XX  add codes to receive frsky telemetry data
//    2016.11.XX  add codes to comm w/ BT module
//    2017.01.09  code cleaning
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
// SFR definition
#define SCI0_SSR    (*(volatile struct st_sci0_ssr    *)0x0008A004)
#define SCI1_SSR    (*(volatile struct st_sci0_ssr    *)0x0008A024)
#define SCI2_SSR    (*(volatile struct st_sci0_ssr    *)0x8A044)
#define SCI3_SSR    (*(volatile struct st_sci0_ssr    *)0x8A064)
#define SCI4_SSR    (*(volatile struct st_sci0_ssr    *)0x8A084)
#define SCI5_SSR    (*(volatile struct st_sci0_ssr    *)0x8A0A4)
#define SCI6_SSR    (*(volatile struct st_sci0_ssr    *)0x8A0C4)
#define SCI7_SSR    (*(volatile struct st_sci0_ssr    *)0x8A0E4)
#define SCI8_SSR    (*(volatile struct st_sci0_ssr    *)0x8A104)
#define SCI9_SSR    (*(volatile struct st_sci0_ssr    *)0x8A124)
#define SCI10_SSR   (*(volatile struct st_sci0_ssr    *)0x8A144)
#define SCI11_SSR   (*(volatile struct st_sci0_ssr    *)0x8A164)
#define SCI12_SSR   (*(volatile struct st_sci0_ssr    *)0x0008B304)

// label definition for HMI input source
enum enum_SER_CHID {
  SER_SRV00,    // SCId ch0
  SER_IOA,      // SCId ch1
  SER_SRV02,    // SCId ch2
  SER_SRV03,    // SCId ch3
  SER_SRV04,    // SCId ch4
  SER_MSP,      // SCId ch5
  SER_SRV06,    // SCId ch6
  SER_SRV07,    // SCId ch7
  SER_SRV08,    // SCId ch8
  SER_SRV09,    // SCId ch9
  SER_SRV10,    // SCId ch10
  SER_SRV11,    // SCId ch11
  SER_TELEM,    // SCId ch12
  SER_N_CH      // number of 
};

enum enum_SER_STAT {
  SER_ORD,      // SCId ch10
  SER_BUSY,     // SCId ch12
  SER_ERR_TX,   // SCId ch11
  SER_ERR_RX,   // SCId ch12
  SER_N_STAT    // number of state
};

enum enum_SER_ERR {
  SER_ERR_P,     // parity err
  SER_ERR_F,     // frame err
  SER_ERR_PF,    // parity / frame err (ERI gen)
  SER_ERR_OR,    // over run err
  SER_N_ERR      // number of err state
};

#define SER_BUFF 256

// calc: N = 48 * 10^6 / ( 64 * 2^(2n-1) * 9600 ) - 1
#define BRR_N( BR, n )  48 * 1000000 / 64 / 

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_SerialErr {
  struct st_sci0_ssr *addr_base_stat;
  uint8_t err[SER_N_ERR];
} st_SerialErr;

typedef struct st_Serial {
  struct st_sci0     *addr_base;
  struct st_sci0_ssr *addr_base_stat;
  struct st_icu      *addr_base_irq;
  uint8_t             vec_base;
  /* uint8_t stat[SER_N_STAT];    // <= too big. omit */
  /* uint8_t stat_err[SER_N_ERR]; // <= too big. omit */
  // for receive
  uint8_t tx_stat;
  uint8_t tx_buff[SER_BUFF];
  uint8_t tx_head;
  uint8_t tx_tail;
  // for transmission
  uint8_t rx_stat;
  uint8_t rx_buff[SER_BUFF];
  uint8_t rx_head;
  uint8_t rx_tail;
} st_Serial;


// -------------------------------------------------------
// -------------------------------- Proto-type declaration
uint8_t SerGenBRR( st_Serial*, uint32_t, uint8_t );
  
extern void SerStop( struct st_Serial* );
extern void SerStart( struct st_Serial* );
void SerPortInit( enum enum_SER_CHID );
uint8_t SerFuncInit( struct st_Serial*, uint32_t );
extern uint8_t SerInit( struct st_Serial*, uint32_t, enum enum_SER_CHID );
/* extern uint8_t SerGetStat( struct st_Serial* ); */
/* extern uint8_t SerWriteBG( st_Serial* ); */
extern uint8_t SerWrite( st_Serial*, uint8_t );
extern uint8_t SerBytesWrite( st_Serial*, uint8_t * );
extern uint8_t SerWriteTest( st_Serial*, uint8_t );
/* extern uint8_t SerReadBG( st_Serial* ); */
extern uint8_t SerRead( st_Serial* );
extern uint8_t SerBytesRead( st_Serial*, uint8_t* );
extern uint8_t SerReadable( st_Serial* );
extern uint8_t SerReadTest( st_Serial* );
extern uint8_t SerDaemon( st_Serial* );


//* extern func. no longer be required *//
extern void Ser12Stop( void );
extern void Ser12Start( void );
void Ser12PortInit( void );
uint8_t Ser12FuncInit( struct st_Serial*, uint32_t );
extern void Ser12Init( struct st_Serial*, uint32_t );
/* extern uint8_t Ser12GetStat( struct st_Serial* ); */
extern uint8_t Ser12ReadBG( st_Serial* );
extern uint8_t Ser12Read( st_Serial* );
extern uint8_t Ser12ReadTest( st_Serial* );

#endif
