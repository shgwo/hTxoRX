
#include "PortUtils.h"

void MPCUnlock( void );
void MPCLock( void );

int  PortConfADC( st_mpc *addr_mpc, enum asel, enum isel, st_port4 *addr_port, uint8_t pin, enum pmr, enum pdr, uint8_t data )
{
  /* set MPC (func) */
  MPCUnlock();
  addr_mpc.BIT.ASEL  = asel;
  addr_mpc.BIT.ISEL  = isel;
  MPCLock();
  /* set individual port (mode, data) */
  uint8_t mask = 0x01;
  addr_port.PODR.BYTE = (mask & data << pin);
  addr_port.PIDR.BYTE = (mask & data << pin);
  addr_port.PDR.BYTE  = (mask & pdr << pin);  // PMR_GPIO / PMR_FUNC
  addr_port.PMR.BYTE  = (mask & pmr << pin);  // PMR_GPIO / PMR_FUNC

  /* read check */
  
  return(0);
};


int  PortConfGPIO( st_port4 *addr_port, uint8_t pin, enum pmr, enum pdr, uint8_t data )
{

  /* read check */
  
  return(0);
}

//
//  lock / unlock MPC reg edit
//
void MPCUnlock( void ){
  MPC.PWPR.BIT.B0WI  = B0WI_UNLOCK;
  MPC.PWPR.BIT.PFSWE = PFSWE_UNLOCK;
}


void MPCLock( void ){
  MPC.PWPR.BIT.PFSWE = PFSWE_LOCK;
  MPC.PWPR.BIT.B0WI  = B0WI_LOCK;
}
