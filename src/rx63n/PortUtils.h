#ifndef __PORTUTILS_H__
#define __PORTUTILS_H__

#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"

extern int  PortConfADC( volatile struct st_mpc *, uint8_t, uint8_t, volatile struct st_port4 *, uint8_t, enum, enum, uint8_t );
extern int  PortConfGPIO( volatile struct st_port4 *, uint8_t, enum enum_PDR, enum enum_PMR, uint8_t );



#endif
