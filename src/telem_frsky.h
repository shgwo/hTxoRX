// -------------------------------------------------------
// ------------------------------------------------ Notice
/*
 * partially cited from frsky.h, opentx project
 * 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
//
// -------------------------------------------------------
// ------------------------------------------------- Info.
//
//  Description:   parser for frsky telemetry data 
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2017.01.09
//
//  history:
//    2016.01.09  create for RX63N( GR-SAKURA ) base-system
//                add codes to parse frsky D series telemetry data
//
#ifndef __TELEM_FRSKY_H__
#define __TELEM_FRSKY_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
/* #include "iodefine.h" */
/* #include "iodefine_enum.h" */
#include "typedefine.h"
#include "serial.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// separator of Frsky paket
#define TELEM_FRSKY_HEAD          0x7e
#define TELEM_FRSKY_TAIL          0x7e

// byte stuffing (carrier)
#define TELEM_FRSKY_BS00          0x7d
#define TELEM_FRSKY_BS10          0x5e
#define TELEM_FRSKY_BS11          0x5d
// byte stuffing (replaced)
#define TELEM_FRSKY_BS10_         0x7e
#define TELEM_FRSKY_BS11_         0x7d

// Frsky packet length
#define TELEM_FRSKY_PCK_LEN       11

// Enumerate FrSky packet codes  == from frsky.h ==
#define LINKPKT                   0xfe
#define USRPKT                    0xfd
#define A11PKT                    0xfc
#define A12PKT                    0xfb
#define A21PKT                    0xfa
#define A22PKT                    0xf9
#define ALRM_REQUEST              0xf8
#define RSSI1PKT                  0xf7
#define RSSI2PKT                  0xf6
#define RSSI_REQUEST              0xf1

// byte stuffing (carrier)
#define TELEM_FRSKY_BSFC00        0x5d
#define TELEM_FRSKY_BSFC10        0x3e
#define TELEM_FRSKY_BSFC11        0x3d
// byte stuffing (replaced)
#define TELEM_FRSKY_BSFC10_       0x5e
#define TELEM_FRSKY_BSRC11_       0x5d


// FrSky old DATA IDs (1 byte)  == from frsky.h ==
#define GPS_ALT_BP_ID             0x01
#define TEMP1_ID                  0x02
#define RPM_ID                    0x03
#define FUEL_ID                   0x04
#define TEMP2_ID                  0x05
#define VOLTS_ID                  0x06
#define GPS_ALT_AP_ID             0x09
#define BARO_ALT_BP_ID            0x10
#define GPS_SPEED_BP_ID           0x11
#define GPS_LONG_BP_ID            0x12
#define GPS_LAT_BP_ID             0x13
#define GPS_COURS_BP_ID           0x14
#define GPS_DAY_MONTH_ID          0x15
#define GPS_YEAR_ID               0x16
#define GPS_HOUR_MIN_ID           0x17
#define GPS_SEC_ID                0x18
#define GPS_SPEED_AP_ID           0x19
#define GPS_LONG_AP_ID            0x1A
#define GPS_LAT_AP_ID             0x1B
#define GPS_COURS_AP_ID           0x1C
#define BARO_ALT_AP_ID            0x21
#define GPS_LONG_EW_ID            0x22
#define GPS_LAT_NS_ID             0x23
#define ACCEL_X_ID                0x24
#define ACCEL_Y_ID                0x25
#define ACCEL_Z_ID                0x26
#define CURRENT_ID                0x28
#define VARIO_ID                  0x30
#define VFAS_ID                   0x39
#define VOLTS_BP_ID               0x3A
#define VOLTS_AP_ID               0x3B
#define FRSKY_LAST_ID             0x3F
#define D_RSSI_ID                 0xF0
#define D_A1_ID                   0xF1
#define D_A2_ID                   0xF2

#define TELFRSKY_BUFF_PARSE 16

// read state definition at frsky block reading
enum enum_TelemfrskyD_statBlockRead{
  TELFRSKY_STBR_SRC,     // separator search
  TELFRSKY_STBR_REC,     // recognize begin/end
  TELFRSKY_STBR_RD,      // reading
  TELFRSKY_STBR_RD_BS,   // reading (byte-stuffing collection)
  TELFRSKY_STBR_N        // number of state
};

// read state definition at frsky FC reading (for Betaflight)
enum enum_TelemfrskyD_statFCReadBeta{
  TELFRSKY_STFCRBT_SRC,     // separator search
  TELFRSKY_STFCRBT_REC,     // recognize begin/end
  TELFRSKY_STFCRBT_RD,      // reading
  TELFRSKY_STFCRBT_RD_BS,   // reading (byte-stuffing collection)
  TELFRSKY_STFCRBT_N        // number of state
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_TelemFrskyD_ParseLnkPkt {
  union {
    uint8_t  id;        // should be 0xFE
    uint8_t  port1;     // AIN1 port 3.3V : 0x00-0xFF
    uint8_t  port2;     // AIN2 port 3.3V : 0x00-0xFF
    uint16_t rssi;      // Link quality (data 1/2 byte?)
    uint8_t  dummy[4];  // filled by 0x00
  };
} st_TelemFrskyD_LnkPkt;

typedef struct st_TelemFrskyD_ParseUsrPkt {
  union {
    uint8_t  id;        // should be 0xFD
    uint8_t  len;       // payload data length
    uint8_t  cnt;       // chain counter
    uint8_t  dat[7];    // significant bytes
  };
} st_TelemFrskyD_UsrPkt;


typedef struct st_TelemFrskyD_Parse {
  enum enum_TelemfrskyD_statBlockRead  stat_br;
  enum enum_TelemfrskyD_statFCReadBeta stat_fcr;
  uint8_t buff[TELFRSKY_BUFF_PARSE];
  uint8_t idx;
  uint8_t stat_fill;
  /* uint8_t lnk_buff[TELFRSKY_BUFF_PARSE]; */
  /* uint8_t idx_lnk; */
  /* uint8_t usr_buff[TELFRSKY_BUFF_PARSE]; */
  /* uint8_t idx_usr; */
} st_TelemFrskyD_Parse;
  
typedef struct st_TelemFrskyD {
  st_TelemFrskyD_Parse parse;
  uint16_t gps_alt;
  uint16_t acc_x;
  uint16_t acc_y;
  uint16_t acc_z;
  uint16_t vario;
  
  uint8_t tail_old;
} st_TelemFrskyD;

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t TelemFrskyDInit( struct st_TelemFrskyD* );
extern uint8_t TelemFrskyD( struct st_TelemFrskyD*, struct st_Serial*, struct st_Serial* );

// for telemetry debug
extern uint8_t TelemFrskyDdbg( struct st_TelemFrskyD*, struct st_Serial*, struct st_Serial* );
extern uint8_t TelemFrskyDdbgBRPrint( struct st_TelemFrskyD*, struct st_Serial* );

#endif
