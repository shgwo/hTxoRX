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
//  Description:   primitime register access of RX63N
//
//  Author:        S.Nakamura (shgwo)
//
//  Ver.:          0.1
//  last updated : 2016.06.05
//
//  history:
//    2016.06.05  create for RX63N( GR-SAKURA ) base-system
//
//
#ifndef __SYSUTIL_RX63N_H__
#define __SYSUTIL_RX63N_H__

// -------------------------------------------------------
// ---------------------------------------------- Includes
#include "iodefine.h"
#include "iodefine_enum.h"
#include "typedefine.h"

// -------------------------------------------------------
// ----------------------------------------------- Defines
// HW(brd) settings
#define XTAL_MHZ 12

#define steps(X) for(int j = 0; j < X; j++) { __asm("nop"); }

// -------------------------------------------------------
// ----------------------------------------------- Structs

// -------------------------------------------------------
// -------------------------------- Proto-type declaration
extern void SysCoreUnlock( void );
extern void SysCoreLock( void );
extern void SysMPCUnlock( void );
extern void SysMPCLock( void );
extern void SysMTU34Unlock( void );

extern void SysClkInit( void );
extern void SysMdlStopInit( void );

#endif