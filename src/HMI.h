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
//    2016.12.25  SW port reassignment. Porting PC0 as CD function
//                of SD card slot & PC52 newly assigned for replacement. 
//
#ifndef __HMI_H__
#define __HMI_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"
#include "hTxoRX.h"
#include "adc_sar12b.h"
#include "serial.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
#define HMI_DBG 1 // debug print (UART) OFF:0 / ON:1

// label definition for HMI input source
enum enum_HMI_SW {
  HMI_SW_GHOST,
  HMI_SW_ARM_KEY,
  HMI_SW_ARM_LOG,
  HMI_SW_ROT_KEY,
  HMI_SW_ROT_A,
  HMI_SW_ROT_B,
  HMI_SW_ROT_CNT,
  HMI_SW_MD_F,
  HMI_SW_MD_B,
  HMI_SW_L2_KEY,
  HMI_SW_L2_LCK,
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

enum enum_HMI_SND_PITCH {
  HMI_SND_MUTE, // 10,
  HMI_SND_A4,   // 440,
  HMI_SND_A4S,  // 466.16,
  HMI_SND_B4,   // 493.88,
  HMI_SND_C5,   // 523.25,
  HMI_SND_C5S,  // 554.37,
  HMI_SND_D5,   // 587.33,
  HMI_SND_D5S,  // 622.25,
  HMI_SND_E5,   // 659.26,
  HMI_SND_F5,   // 698.46,
  HMI_SND_F5S,  // 739.99,
  HMI_SND_G5,   // 783.99,
  HMI_SND_G5S,  // 830.61
  HMI_SND_A5,   // 880,
  HMI_SND_A5S,  // 932.33,
  HMI_SND_B5,   // 987.77,
  HMI_SND_A6,   // 1760,
  HMI_SND_A7,   // 3520,
  HMI_SND_A8    // 7040,
};

#define HMI_CYCLE      1    // system: loop cycle (ms)  /* unused */
#define HMI_CYCLE_IN   1    // input loop cycle (ms)    /* unused */
#define HMI_CYCLE_OUT  10   // output loop cycle (ms)   /* unused */

#define HMI_SW_ON_THR  5    // cycle ( * CMT1 cycle )

#define HMI_LED_FB_LEN 80   // cycle ( * CMT1 cycle )

#define HMISndGetPitch(freq) (uint16_t)((double)(4*12*1000000 / 256)/((double)(freq)))
#define HMI_SND_TEMPO_BbHASE   20  // cycle ( * CMT1 cycle )
#define HMI_SND_FB_LEN       5   // cycle ( * CMT1 cycle )
#define HMI_SND_MOR_S_LEN    10  // cycle ( * CMT1 cycle )
#define HMI_SND_MOR_L_LEN    50  // cycle ( * CMT1 cycle )
#define HMI_SND_BUFF         20  // num of beep

#define HMI_SELFT_DT         100  // cycle ( * CMT1 cycle )

// -------------------------------------------------------
// ----------------------------------------------- Structs
typedef struct st_HMISnd {
  // for output (Sounder)
  enum enum_HMI_SND_PITCH pitch;
  uint16_t tempo;
} st_HMISnd;

typedef struct st_HMI {
  // for inputs
  uint16_t adc_val[ADC12_N_CH];
  uint8_t  sw_state[HMI_N_SW];
  uint8_t  sw_state_old[HMI_N_SW];
  uint16_t sw_cnt [HMI_N_SW];
  uint8_t  sw_state_act;
  //  uint16_t sw_ncnt [HMI_N_SW];
  uint16_t gmbl [HMI_N_GMBL];
  uint16_t trm [HMI_N_TRM];
  // for output (general)
  uint8_t  cnt_cyc_out;    
  // for output (LED)
  uint8_t  LED_state[HMI_N_LED];
  enum enum_HMI_SW LED_RGB_state_fb;
  uint16_t LED_RGB[HMI_N_LED_RGB];
  uint8_t  LED_RGB_cnt_fb;
  // for output (Sounder)
  enum enum_HMI_SW snd_state_fb;
  uint16_t  snd_cnt_fb;
  uint8_t   snd_state_seq;
  uint8_t   snd_point_seq;
  st_HMISnd snd_seq[HMI_SND_BUFF];
  uint16_t  snd_cnt_tempo;
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

extern uint8_t HMILEDPPMAct( st_HMI*, st_hTx*, uint8_t);
extern uint8_t HMILEDBatLow( st_HMI *, st_hTx*, uint8_t );
extern uint8_t HMILEDSetRGB( st_HMI* );
extern uint8_t HMILEDFBRGB( st_HMI* );
extern uint8_t HMILEDSetRGBMode( st_HMI*, st_hTx* );

extern uint8_t HMIFlash( st_HMI*, st_hTx* );

#endif
