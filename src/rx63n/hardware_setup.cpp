/***********************************************************************/
/*                                                                     */
/*      PROJECT NAME :  sketch                                         */
/*      FILE         :  hardware_setup.cpp                             */
/*      DESCRIPTION  :  Hardware Initialization                        */
/*      CPU SERIES   :  RX600                                          */
/*      CPU TYPE     :  RX63N                                          */
/*                                                                     */
/*      This file is generated by e2studio.                        */
/*                                                                     */
/***********************************************************************/


#include "iodefine.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
extern void HardwareSetup(void);
#ifdef __cplusplus
}
#endif

void HardwareSetup(void)
{
    // Protection off
    SYSTEM.PRCR.WORD = 0xA503u;

    // Stop sub-clock
    SYSTEM.SOSCCR.BYTE = 0x01u;

    // Set main oscillator settling time to 10ms (131072 cycles @ 12MHz)
    SYSTEM.MOSCWTCR.BYTE = 0x0Du;

    // Set PLL circuit settling time to 10ms (2097152 cycles @ 192MHz)
    SYSTEM.PLLWTCR.BYTE = 0x0Eu;

    // Set PLL circuit to x16
    SYSTEM.PLLCR.WORD = 0x0F00u;

    // Start the external 12Mhz oscillator
    SYSTEM.MOSCCR.BYTE = 0x00u;

    // Turn on the PLL
    SYSTEM.PLLCR2.BYTE = 0x00u;

    // Wait over 12ms (~2075op/s @ 125KHz)
    for(volatile uint16_t i = 0; i < 2075u; i++)
    {
        __asm("nop");
    }

    // Configure the clocks as follows -
    //Clock Description              Frequency
    //----------------------------------------
    //PLL Clock frequency...............192MHz
    //System Clock Frequency.............96MHz
    //Peripheral Module Clock B..........48MHz
    //FlashIF Clock......................48MHz
    //External Bus Clock.................48MHz
    SYSTEM.SCKCR.LONG = 0x21021211u;

    // Configure the clocks as follows -
    //Clock Description              Frequency
    //----------------------------------------
    //USB Clock..........................48MHz
    //IEBus Clock........................24MHz
    SYSTEM.SCKCR2.WORD = 0x0033u;

    // Set the clock source to PLL
    SYSTEM.SCKCR3.WORD = 0x0400u;

    // Stop external bus
    SYSTEM.SYSCR0.WORD  = 0x5A01;

    //EtherC, EDMAC
    SYSTEM.MSTPCRB.BIT.MSTPB15 = 0;

#ifndef GRSAKURA
    // Protection on
    SYSTEM.PRCR.WORD = 0xA500u;
#else
    // Enable the PFSWE midification.
    MPC.PWPR.BIT.B0WI = 0;

    // Disable the PFS register protect.
    MPC.PWPR.BIT.PFSWE = 1;
#endif

}
