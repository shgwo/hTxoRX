
#include "typedefine.h"
#include "hTxoRX.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  Application utilities 
//
void hTxInit( st_hTx *htx ){
  htx->opmd         = OPMD_BOOT;
  htx->opmd_old     = OPMD_BOOT;
  htx->opmd_tran    = 0;
  htx->opmd_log      = OPMD_LOG_OFF;
  htx->opmd_log_old  = OPMD_LOG_OFF;
  htx->opmd_log_tran = 0;
  htx->opmd_bat      = OPMD_BAT_LOW;
  htx->opmd_bat_old  = OPMD_BAT_LOW;
  htx->opmd_bat_tran = 0;
}

void hTxSetMode( st_hTx *htx, enum enum_AppMode md ){
  if( htx->opmd != md ){
    htx->opmd_tran = 1;
    htx->opmd_old = htx->opmd;
    htx->opmd     = md;
  }
}

void hTxSetModeLogOff( st_hTx *htx ){
  if( htx->opmd_log != OPMD_LOG_OFF )
    htx->opmd_log_tran = 1;
  htx->opmd_log_old = htx->opmd_log;
  htx->opmd_log     = OPMD_LOG_OFF;
}

void hTxSetModeLogOn( st_hTx *htx ){
  if( htx->opmd_log != OPMD_LOG_ON )
    htx->opmd_log_tran = 1;
  htx->opmd_log_old = htx->opmd_log;
  htx->opmd_log     = OPMD_LOG_ON;
}

/* end of hTxoRX.c */
