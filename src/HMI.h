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
#ifndef __HMI_H__
#define __HMI_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"
#include "hTxoRX.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// label definition for HMI input source
enum enum_HMI_SW {
  HMI_SW_ARM_KEY,
  HMI_SW_ARM_LOG,
  HMI_SW_ROT,
  HMI_SW_ROT_A,
  HMI_SW_ROT_B,
  HMI_SW_ROT_CNT,
  HMI_SW_L1_KEY,
  HMI_SW_L1_LCK,
  HMI_SW_L2_F,
  HMI_SW_L2_B,
  HMI_SW_R1_KEY,
  HMI_SW_R1_LCK,
  HMI_SW_R2_F,
  HMI_SW_R2_B,
  HMI_N_SW
};
enum enum_HMI_GMBL {
  HMI_GMBL_LV,
  HMI_GMBL_LH,
  HMI_GMBL_RV,
  HMI_GMBL_RH,
  HMI_N_GMBL
};
enum enum_HMI_TRM {
  HMI_TRM_ROT,
  HMI_TRM_VOL,
  HMI_N_TRM
};
enum enum_HMI_LED {
  HMI_LED_POW,
  HMI_LED_LOG,
  HMI_LED_NORM,
  HMI_N_LED
};
enum enum_HMI_LED_RGB {
  HMI_LED_R,
  HMI_LED_G,
  HMI_LED_B,
  HMI_N_LED_RGB
};

#define HMI_LED_FB_LEN 5   // cycle ( * CMT1 cycle )

#define HMI_SND_FB_LEN 5   // cycle ( * CMT1 cycle )
#define HMI_SND_BUFF   20  // num of beep

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_HMISnd {
  // for output (Sounder)
  uint16_t tune;
  uint16_t time;
} st_HMISnd;

typedef struct st_HMI {
  // for inputs
  uint8_t  sw_state[HMI_N_SW];
  uint8_t  sw_state_old[HMI_N_SW];
  uint8_t  sw_state_act;
  uint16_t sw_cnt [HMI_N_SW];
  //  uint16_t sw_ncnt [HMI_N_SW];
  uint16_t gmbl [HMI_N_GMBL];
  uint16_t trm [HMI_N_TRM];
  // for output (LED)
  uint8_t  LED_state[HMI_N_LED];
  uint8_t  ppm_cnt;
  uint8_t  LED_RGB_state_fb;
  uint8_t  LED_RGB_cnt_fb;
  uint16_t LED_RGB[HMI_N_LED_RGB];
  // for output (Sounder)
  uint8_t   snd_state_fb;
  uint16_t  snd_cnt_fb;
  uint8_t   snd_state_seq;
  st_HMISnd snd_seq[HMI_SND_BUFF];
  // for utility
  uint16_t cnt_sw;
  uint16_t cnt_sw_act;
  uint16_t cnt_led;
  uint16_t cnt_ppm;
  uint16_t cnt_bat;
} st_HMI;

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern uint8_t HMISysInTmrInit( void );
extern uint8_t HMISysOutTmrInit( void );

extern uint8_t HMIPortInInit( void );
extern uint8_t HMIPortInFuncInit( void );
extern uint8_t HMISWInit( struct st_HMI* );

extern uint8_t HMIPortOutLEDExtInit( void );
extern uint8_t HMIPortOutFuncLEDExtInit( void );
extern uint8_t HMILEDExtInit( struct st_HMI* );

extern uint8_t HMIPortOutSndInit( void );
extern uint8_t HMIPortOutFuncSndInit( void );
extern uint8_t HMISndInit( struct st_HMI* );

extern uint8_t HMIScanSW ( struct st_HMI* );
extern uint8_t HMISWState ( struct st_HMI*, enum enum_HMI_SW );
extern uint8_t HMILongPress ( struct st_HMI*, enum enum_HMI_SW, uint16_t );

extern uint8_t HMILEDPPMAct( st_HMI*, enum enum_AppModeLog, uint8_t);
extern uint8_t HMILEDBatLow( st_HMI *, enum enum_AppModeBat, uint8_t );
extern uint8_t HMILEDSetRGB( st_HMI* );
extern uint8_t HMILEDFBRGB( st_HMI* );
extern uint8_t HMILEDSetRGBMode( st_HMI*, enum enum_AppMode );

extern uint8_t HMIFlash( st_HMI*, enum enum_AppMode, enum enum_AppModeBat, enum enum_AppModeLog );

#endif