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
//    2017.06.18  add defines & structs for parsing telemetry user packet
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
// separator of Frsky telemetry paket
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
#define TELEM_FRSKY_PKT_LEN       11

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

// separator of fc packet in frsky user packet
#define TELEM_FRSKY_FC_HEAD       0x5e
#define TELEM_FRSKY_FC_TAIL       0x5e

// Frsky FC Record length
#define TELEM_FRSKY_FCR_CUE_LEN   5    // each (1+1+2+1) bytes
#define TELEM_FRSKY_FCR_IDS_LEN   50   // 35 ( * each 2 bytes )

// byte stuffing (carrier)
#define TELEM_FRSKY_BSFC00        0x5d
#define TELEM_FRSKY_BSFC10        0x3e
#define TELEM_FRSKY_BSFC11        0x3d
// byte stuffing (replaced)
#define TELEM_FRSKY_BSFC10_       0x5e
#define TELEM_FRSKY_BSFC11_       0x5d


// FrSky old DATA IDs (1 byte)  == from frsky.h of OpenTx ==
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

// Data Ids  (bp = before decimal point; ap = after decimal point)
// Official data IDs   == from frsky.c of Betaflight ==
#define ID_GPS_ALTIDUTE_BP    0x01
#define ID_GPS_ALTIDUTE_AP    0x09
#define ID_TEMPRATURE1        0x02
#define ID_RPM                0x03
#define ID_FUEL_LEVEL         0x04
#define ID_TEMPRATURE2        0x05
#define ID_VOLT               0x06
#define ID_ALTITUDE_BP        0x10
#define ID_ALTITUDE_AP        0x21
#define ID_GPS_SPEED_BP       0x11
#define ID_GPS_SPEED_AP       0x19
#define ID_LONGITUDE_BP       0x12
#define ID_LONGITUDE_AP       0x1A
#define ID_E_W                0x22
#define ID_LATITUDE_BP        0x13
#define ID_LATITUDE_AP        0x1B
#define ID_N_S                0x23
#define ID_COURSE_BP          0x14
#define ID_COURSE_AP          0x1C
#define ID_DATE_MONTH         0x15
#define ID_YEAR               0x16
#define ID_HOUR_MINUTE        0x17
#define ID_SECOND             0x18
#define ID_ACC_X              0x24
#define ID_ACC_Y              0x25
#define ID_ACC_Z              0x26
#define ID_VOLTAGE_AMP        0x39
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28
// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42

#define ID_VERT_SPEED         0x30 //opentx vario

// User defined IDs (hTxoRX original)
#define ID_ORG_ID             0x80  // 8'b 1000 0000
#define ID_ORG_RPM_M1         0x90  // 8'b 1001 0000
#define ID_ORG_RPM_M2         0x91  // 8'b 1001 0001
#define ID_ORG_RPM_M3         0x92  // 8'b 1001 0002
#define ID_ORG_RPM_M4         0x93  // 8'b 1001 0003
#define ID_ORG_PID_PIT_P      0xA0  // 8'b 1010 0000
#define ID_ORG_PID_PIT_I      0xA1  // 8'b 1010 0001
#define ID_ORG_PID_PIT_D      0xA2  // 8'b 1010 0002
#define ID_ORG_PID_ROL_P      0xA4  // 8'b 1010 0100
#define ID_ORG_PID_ROL_I      0xA5  // 8'b 1010 0101
#define ID_ORG_PID_ROL_D      0xA6  // 8'b 1010 0102
#define ID_ORG_PID_YAW_P      0xA4  // 8'b 1010 1000
#define ID_ORG_PID_YAW_I      0xA5  // 8'b 1010 1001
#define ID_ORG_PID_YAW_D      0xA6  // 8'b 1010 1002

#define TELFRSKY_BUFF_PARSE   16  // 11 byte + 1 (BS)

#define TELFRSKY_ACC_1G   256  // acc_1G of betaflight ?

#define TELFRSKY_TH_LNK_RSSI_WEAK  0x4E00  // threshold of rssi lost (temp var)
#define TELFRSKY_TH_LNK_RSSI_LOST  0x4000  // threshold of rssi weak (temp var)
#define TELFRSKY_TH_LNK_CNT        0xFF    // threshold of continuity (temp var)

// read state definition at frsky block reading
enum enum_TelemfrskyD_statBlockRead{
  TELFRSKY_STBR_SRC,     // separator search
  TELFRSKY_STBR_REC,     // recognize begin/end
  TELFRSKY_STBR_RD,      // reading
  TELFRSKY_STBR_RD_BS,   // reading (byte-stuffing collection)
  TELFRSKY_STBR_N        // number of state
};

// read state definition at frsky packet parsing
enum enum_TelemfrskyD_statPacketType{
  TELFRSKY_STPT_UKWN,    // can't recognize
  TELFRSKY_STPT_LNK,     // link packet
  TELFRSKY_STPT_USR,     // user packet
  TELFRSKY_STPT_A11,     // ?
  TELFRSKY_STPT_A12,
  TELFRSKY_STPT_A21,
  TELFRSKY_STPT_A22,
  TELFRSKY_STPT_ALM,
  TELFRSKY_STPT_RSSI1,
  TELFRSKY_STPT_RSSI2,
  TELFRSKY_STPT_REQ,
  TELFRSKY_STPT_N
};

// read state definition at frsky FC reading (for Betaflight)
enum enum_TelemfrskyD_statFCReadBeta{
  TELFRSKY_STFCRBF_SRC,      // separator search
  TELFRSKY_STFCRBF_REC,      // recognize begin/end
  TELFRSKY_STFCRBF_CCHK,     // chain count checking w/ reading
  TELFRSKY_STFCRBF_CCHKERR,  // chain count check error
  TELFRSKY_STFCRBF_RD,       // reading
  TELFRSKY_STFCRBF_RD_BS,    // reading (byte-stuffing collection)
  TELFRSKY_STFCRBF_N         // number of state
};

enum enum_TelemSafe {
  TELM_SAFE_NFD,    // telemetry not found
  TELM_SAFE_ESTB,   // telemetry established
  TELM_SAFE_WEAK,   // established, signal is low
  TELM_SAFE_LOSTL,  // once, established & now lost (link packet)
  TELM_SAFE_LOSTU,  // once, established & now lost (user packet)
  TELM_SAFE_LOST,   // once, established & all lost (continuity)
  TELM_SAFE_N
};

// -------------------------------------------------------
// ----------------------------------------------- Structs
/* unused now */
typedef struct st_TelemFrskyD_ParseLnkPkt {
  union {
    uint8_t  buff[TELEM_FRSKY_PKT_LEN];
    struct{
      uint8_t  head;      // should be 0x7E
      uint8_t  id;        // should be 0xFE
      uint8_t  port1;     // AIN1 port 3.3V : 0x00-0xFF
      uint8_t  port2;     // AIN2 port 3.3V : 0x00-0xFF
      uint16_t rssi;      // Link quality (data 1/2 byte?)
      uint8_t  dummy[4];  // filled by 0x00
      uint8_t  tail;      // should be 0x7E
    } PKTVAR;
  } PKTLNK;
} st_TelemFrskyD_LnkPkt;

/* unused now */
typedef struct st_TelemFrskyD_ParseUsrPkt {
  union {
    uint8_t  id;        // should be 0xFD
    uint8_t  len;       // payload data length
    uint8_t  cnt;       // chain counter
    uint8_t  dat[6];    // significant bytes
  };
} st_TelemFrskyD_UsrPkt;


typedef struct st_TelemFrskyD_Parse {
  // telem safer data
  enum enum_TelemSafe  stat_safe; // for safe obs
  uint8_t              cnt_link;  // link lost count
  
  // block read data
  enum enum_TelemfrskyD_statBlockRead   stat_br;
  enum enum_TelemfrskyD_statPacketType  stat_pt;
  uint8_t idx;
  uint8_t stat_fill;
  int8_t  fcr_cnt_old;
  uint8_t stat_dbg_flash;
  union {
    uint8_t  buff[TELEM_FRSKY_PKT_LEN];
    struct{
      uint8_t  head;      // should be 0x7E
      uint8_t  id;        // should be 0xFE
      uint8_t  port1;     // AIN1 port 3.3V : 0x00-0xFF
      uint8_t  port2;     // AIN2 port 3.3V : 0x00-0xFF
      uint16_t rssi;      // Link quality (data 1/2 byte?)
      uint8_t  dummy[4];  // filled by 0x00
      uint8_t  tail;      // should be 0x7E
    } LNK;
    struct{
      uint8_t  head;      // should be 0x7E
      uint8_t  id;        // should be 0xFD
      uint8_t  len;       // payload data length
      int8_t   cnt;       // chain counter
      uint8_t  dat[6];    // significant bytes
      uint8_t  tail;      // should be 0x7E
    } USR;
  } PKT;

  // receiver telem data
  
  // fc telem data
  enum enum_TelemfrskyD_statFCReadBeta stat_fcr;
  uint8_t idx_fcr;
  uint8_t stat_fcr_fill;
  /* uint8_t stat_fcr_chain; */
  union{
    uint8_t buff[TELEM_FRSKY_FCR_CUE_LEN];
    struct{
      uint8_t  head;      // should be 0x5E
      uint8_t  id;        // depend on data
      uint8_t  dat[2];    // 2 byte length data
      uint8_t  tail;      // should be 0x5E
    } CUE;
  } FCR;
  uint8_t idx_fcr_ids;
  union{
    uint16_t buff[TELEM_FRSKY_FCR_IDS_LEN];
    struct{
      uint16_t gps_alt_bp; // ID: 0x01
      uint16_t gps_alt_ap; // ID: 0x09
      uint16_t temp1;      // ID: 0x02
      uint16_t rpm;        // ID: 0x03
      uint16_t fuel;       // ID: 0x04
      uint16_t temp2;      // ID: 0x05 
      uint16_t volt;       // ID: 0x06 16'b llll llll cccc hhhh
      uint16_t alt_bp;     // ID: 0x10
      uint16_t alt_ap;     // ID: 0x21      
      uint16_t gps_sp_bp;  // ID: 0x19
      uint16_t gps_sp_ap;  // ID: 0x19
      uint16_t lng_bp;     // ID: 0x12
      uint16_t lng_ap;     // ID: 0x1A
      uint16_t e_w;        // ID: 0x22
      uint16_t lat_bp;     // ID: 0x13
      uint16_t lat_ap;     // ID: 0x1B
      uint16_t n_s;        // ID: 0x23
      uint16_t crs_bp;     // ID: 0x14
      uint16_t crs_ap;     // ID: 0x1C
      uint16_t yr;         // ID: 0x16
      uint16_t hr_min;     // ID: 0x17
      uint16_t sec;        // ID: 0x18
      uint16_t accx;       // ID: 0x24
      uint16_t accy;       // ID: 0x25
      uint16_t accz;       // ID: 0x26
      uint16_t volamp;
      uint16_t volamp_bp;
      uint16_t volamp_ap;
      uint16_t cur;
      uint16_t gyrox;      // ID: 0x40
      uint16_t gyroy;      // ID: 0x41
      uint16_t gyroz;      // ID: 0x42
      uint16_t vspd;       // ID: 0x30
      // user define for hTxoRX ( to do )
      uint16_t id;         // ID: 0x80
      uint16_t rpm_m1;     // ID: 0x90
      uint16_t rpm_m2;     // ID: 0x91
      uint16_t rpm_m3;     // ID: 0x92
      uint16_t rpm_m4;     // ID: 0x93
      uint16_t pid_pit_p;  // ID: 0xA0
      uint16_t pid_pit_i;  // ID: 0xA1
      uint16_t pid_pit_d;  // ID: 0xA2
      uint16_t pid_rol_p;  // ID: 0xA4
      uint16_t pid_rol_i;  // ID: 0xA5
      uint16_t pid_rol_d;  // ID: 0xA6
      uint16_t pid_yaw_p;  // ID: 0xA8
      uint16_t pid_yaw_i;  // ID: 0xA9
      uint16_t pid_yaw_d;  // ID: 0xAA
    } SEN;
  } IDS;
 
  /* uint8_t lnk_buff[TELFRSKY_BUFF_PARSE]; */
  /* uint8_t idx_lnk; */
  /* uint8_t usr_buff[TELFRSKY_BUFF_PARSE]; */
  /* uint8_t idx_usr; */
} st_TelemFrskyD_Parse;
  
typedef struct st_TelemFrskyD {
  st_TelemFrskyD_Parse parse;
  // to do: consider needs below vars
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
extern uint8_t TelemFrskyD( struct st_TelemFrskyD*, struct st_Serial* );

// for telemetry debug
extern uint8_t TelemFrskyDdbg( struct st_TelemFrskyD*, struct st_Serial*, struct st_Serial* );
extern uint8_t TelemFrskyDdbgBRPrint( struct st_TelemFrskyD*, struct st_Serial* );
extern uint8_t TelemFrskyDdbgStatPrint( struct st_TelemFrskyD*, struct st_Serial* );

#endif
