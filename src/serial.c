
#include "typedefine.h"
#include "serial.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  serial (SCI) utilities 
//
uint8_t SerGenBRR( st_Serial *ser,
		   uint32_t brate, uint8_t xtal_mhz ){
  uint8_t A = 1;
  uint8_t pclk = 1;
  uint8_t n_calc = 3;
  // pclk
  switch( SYSTEM.SCKCR.BIT.FCK ){
  case XCK_DIV_4:
    // XTAL: xtal_mhz(12MHz) 16x/4 -> 48MHz ){
    pclk = xtal_mhz * 16 / 4;
    break;
  default:
    pclk = 1;
    break;
  }
  // check A (64/32)
  volatile struct st_sci0 sci = *(ser->addr_base);
  A = ( sci.SEMR.BIT.ABCS ? 32 : 64 );
  // calc 2^(2n-1)
  switch( sci.SMR.BIT.CKS ){
  case SCI_CKS_PCLK:
    n_calc = 1 / 2;
    break;
  case SCI_CKS_PCLK_4:
    n_calc = 2;
    break;
  case SCI_CKS_PCLK_16:
    n_calc = 8;
    break;
  case SCI_CKS_PCLK_64:
    n_calc = 32;
    break;
  default:
    n_calc = 32;
    break;
  }
  // calc
  return( pclk * 1000000 / ( A * n_calc * brate ) );
}


void SerPortInit( enum enum_SERI_CHID ch ){
  switch( ch ){
  case SER_HOST:
    // PE1, PE2 -> UART Port setting (Rx/Tx) 
    PORTC.PCR.BIT.B2   = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
    PORTC.PODR.BIT.B3  = 0;
    PORTC.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
    PORTC.PMR.BIT.B3   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
    PORTC.PDR.BIT.B2   = PDR_IN;           // PDR_IN / PDR_OUT
    PORTC.PDR.BIT.B3   = PDR_OUT;          // PDR_IN / PDR_OUT
    MPC.PC2PFS.BIT.PSEL  = PC2PFS_RXD5;
    MPC.PC3PFS.BIT.PSEL  = PC3PFS_TXD5;
    PORTC.PMR.BIT.B2     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
    PORTC.PMR.BIT.B3     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
    break;
  default:
    break;
  }
}

void SerStop( st_Serial *ser ){
  volatile struct st_sci0 sci = *(ser->addr_base);
  // stop to communicate with UART  
  sci.SCR.BIT.TEIE = SCI_TEIE_DE;    // A TEI interrupt request is disabled
  sci.SCR.BIT.RE   = SCI_RE_DE;      // Serial reception is disabled
  sci.SCR.BIT.RIE  = SCI_RIE_DE;     // RXI and ERI interrupt requests are disabled
  sci.SCR.BIT.TE   = SCI_TE_DE;      // Serial transmission is disabled
  sci.SCR.BIT.TIE  = SCI_TIE_DE;     // A TXI interrupt request is disabled
}

void SerStart( st_Serial *ser ){
  volatile struct st_sci0 sci = *(ser->addr_base);
  // start to communicate with UART
  sci.SCR.BIT.TEIE = SCI_TEIE_DE;    // A TEI interrupt request is disabled
  sci.SCR.BIT.RE   = SCI_RE_EN;      // Serial reception is enabled
  sci.SCR.BIT.RIE  = SCI_RIE_EN;     // RXI and ERI interrupt requests are enabled
  sci.SCR.BIT.TE   = SCI_TE_EN;      // Serial transmission is enabled
  sci.SCR.BIT.TIE  = SCI_TIE_EN;     // A TXI interrupt request is enabled
}


uint8_t SerFuncInit( struct st_Serial *ser, uint32_t brate ){
  volatile struct st_sci0 sci = *(ser->addr_base);
  // UART function settings
  sci.SCR.BIT.CKE    = SCI_CKE_OCHPIO;  // On-chip baud rate generator (SCK as I/O)
  sci.SIMR1.BIT.IICM = SCI_IICM_SCIF;   // Serial interface mode
  sci.SPMR.BIT.CKPOL = SCI_CKPOL_NORM;  // Clock polarity is not inverted.
  sci.SPMR.BIT.CKPH  = SCI_CKPH_NORM;   // Clock is not delayed.
  sci.SCMR.BIT.SMIF  = SCI_SMIF_SCIF;   // Serial communications interface mode
  sci.SCMR.BIT.SINV  = SCI_SINV_NORM;   // TDR contents are transmitted as they are.
  sci.SCMR.BIT.SDIR  = SCI_SDIR_LSB;    // Transfer with LSB-first
  /* sci.SEMR.ACS0  = SCI_ACS0_EXCLK;  // External clock input */
  /* sci.SEMR.ABCS  = SCI_ABCS_16CLK;  // Selects 16 base clock cycles for 1-bit period */
  sci.SMR.BIT.CKS    = SCI_CKS_PCLK;   // PCLK / 1
  sci.SMR.BIT.CM     = SCI_CM_ASYNC;   // Asynchronous mode
  sci.SMR.BIT.CHR    = SCI_CHR_8;      // Selects 8 bits as the data length
  sci.SMR.BIT.STOP   = SCI_STOP_1;     // 1 stop bit
  sci.SMR.BIT.PE     = SCI_PE_DE;      // Parity bit addition is not performed
  /* sci.SMR.BIT.PM   = SCI_PM_ODD      // Select odd parity */
  sci.SMR.BIT.MP     = SCI_MP_DE;      // disabled
  /* sci.SCR.MPIE = SCI_MPIE_NORM;  // Normal reception */
  /* sci.SSR.MPB = SCI_MPB_ID;    // ID transmission cycles */
  /* sci.SSR.MPBT = SCI_MPBT_ID;    // ID transmission cycles */

  // BAUDRATE = 9600 bps
  // PCLK => 12MHz * 4
  // CKS => PCLK / 1, n=0
  // calc: N = 48 * 10^6 / ( 64 * 2^(2n-1) * 9600 ) - 1
  //         = 3*2*2 * 10^6 / ( 2^5 * 9600 )        - 1
  //         = 3 * 10^4 / ( 2^3 * 96 )              - 1
  //         = 10^4 / ( 2^3 * 2^5 )                 - 1
  //         = 10000 / 256                          - 1
  //         = 39.06 - 1
  sci.BRR = 38;
  sci.BRR = SerGenBRR( ser, brate, XTAL_MHZ );
  
}

uint8_t SerInit( st_Serial *ser, uint32_t brate, enum enum_SERI_CHID ch ){
  // gen SCI ch addr_base
  ser->addr_base =  (struct st_sci0    *)0x8A000 + (0x20 * ch);
  // UART port initialization
  SerPortInit( ch );
  SerStop( ser );
  SerFuncInit( ser, brate );
  // start to communicate with UART
  SerStart( ser );
  return( 0 );
}

void Ser12PortInit( void ){
  // PE1, PE2 -> UART Port setting (Rx/Tx)
  PORTE.PODR.BIT.B1  = 0;
  PORTE.PCR.BIT.B2   = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
  PORTE.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
  PORTE.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT
  PORTE.PDR.BIT.B2   = PDR_IN;           // PDR_IN / PDR_OUT
  MPC.PE1PFS.BIT.PSEL  = PE1PFS_TXD12;
  MPC.PE2PFS.BIT.PSEL  = PE2PFS_RXD12;
  PORTE.PMR.BIT.B1     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
  PORTE.PMR.BIT.B2     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
}

void Ser12Stop( void ){
  SCI12.SCR.BIT.TEIE = SCI_TEIE_DE;    // A TEI interrupt request is disabled
  SCI12.SCR.BIT.RE   = SCI_RE_DE;      // Serial reception is disabled
  SCI12.SCR.BIT.RIE  = SCI_RIE_DE;     // RXI and ERI interrupt requests are disabled
  SCI12.SCR.BIT.TE   = SCI_TE_DE;      // Serial transmission is disabled
  SCI12.SCR.BIT.TIE  = SCI_TIE_DE;     // A TXI interrupt request is disabled
}

void Ser12Start( void ){
  // start to communicate with UART
  SCI12.SCR.BIT.TEIE = SCI_TEIE_DE;    // A TEI interrupt request is disabled
  SCI12.SCR.BIT.RE   = SCI_RE_EN;      // Serial reception is enabled
  SCI12.SCR.BIT.RIE  = SCI_RIE_EN;     // RXI and ERI interrupt requests are enabled
  SCI12.SCR.BIT.TE   = SCI_TE_EN;      // Serial transmission is enabled
  SCI12.SCR.BIT.TIE  = SCI_TIE_EN;     // A TXI interrupt request is enabled
}

uint8_t Ser12FuncInit( struct st_Serial *ser, uint32_t brate ){
  // PE1, PE2 -> UART IN/OUT
  SCI12.SCR.BIT.CKE    = SCI_CKE_OCHPIO;  // On-chip baud rate generator (SCK as I/O)
  SCI12.SIMR1.BIT.IICM = SCI_IICM_SCIF;   // Serial interface mode
  SCI12.SPMR.BIT.CKPOL = SCI_CKPOL_NORM;  // Clock polarity is not inverted.
  SCI12.SPMR.BIT.CKPH  = SCI_CKPH_NORM;   // Clock is not delayed.
  SCI12.SCMR.BIT.SMIF  = SCI_SMIF_SCIF;   // Serial communications interface mode
  SCI12.SCMR.BIT.SINV  = SCI_SINV_NORM;   // TDR contents are transmitted as they are.
  SCI12.SCMR.BIT.SDIR  = SCI_SDIR_LSB;    // Transfer with LSB-first
  /* SCI12.SEMR.ACS0  = SCI_ACS0_EXCLK;  // External clock input */
  /* SCI12.SEMR.ABCS  = SCI_ABCS_16CLK;  // Selects 16 base clock cycles for 1-bit period */
  SCI12.SMR.BIT.CKS    = SCI_CKS_PCLK;   // PCLK / 1
  SCI12.SMR.BIT.CM     = SCI_CM_ASYNC;   // Asynchronous mode
  SCI12.SMR.BIT.CHR    = SCI_CHR_8;      // Selects 8 bits as the data length
  SCI12.SMR.BIT.STOP   = SCI_STOP_1;     // 1 stop bit
  SCI12.SMR.BIT.PE     = SCI_PE_DE;      // Parity bit addition is not performed
  /* SCI12.SMR.BIT.PM   = SCI_PM_ODD      // Select odd parity */
  SCI12.SMR.BIT.MP     = SCI_MP_DE;      // disabled
  /* SCI12.SCR.MPIE = SCI_MPIE_NORM;  // Normal reception */
  /* SCI12.SSR.MPB = SCI_MPB_ID;    // ID transmission cycles */
  /* SCI12.SSR.MPBT = SCI_MPBT_ID;    // ID transmission cycles */

  // BAUDRATE = 9600 bps
  // PCLK => 12MHz * 4
  // CKS => PCLK / 1, n=0
  // calc: N = 48 * 10^6 / ( 64 * 2^(2n-1) * 9600 ) - 1
  //         = 3*2*2 * 10^6 / ( 2^5 * 9600 )        - 1
  //         = 3 * 10^4 / ( 2^3 * 96 )              - 1
  //         = 10^4 / ( 2^3 * 2^5 )                 - 1
  //         = 10000 / 256                          - 1
  //         = 39.06 - 1
  SCI12.BRR = 38;
  SCI12.BRR = SerGenBRR( ser, brate, XTAL_MHZ );  
}

void Ser12Init( st_Serial *ser, uint32_t brate ){
  // UART port initialization
  Ser12PortInit();
  Ser12Stop();
  Ser12FuncInit( ser, brate );
  // start to communicate with UART
  Ser12Start();
}

uint8_t SerGetStat( st_Serial *ser){
  /* if( SCI12.SCR.TEND ) */
  /*   ser->stat[SER_BUSY] = SCI_TEND_DONE; */
  /* else */
  /*   ser->stat[SER_BUSY] = SCI_TEND_BUSY; */
  /* if( SCI12.SCR.PER ) */
  /*   ser->stat_err[SER_ERR_P] = SCI_PER_ERR; */
  /* else */
  /*   ser->stat_err[SER_ERR_P] = SCI_PER_OK; */
  /* if( SCI12.SCR.FER ) */
  /*   ser->stat_err[SER_ERR_F] = SCI_FER_ERR; */
  /* else */
  /*   ser->stat_err[SER_ERR_F] = SCI_FER_OK; */
  /* if( SCI12.SCR.ORER ) */
  /*   ser->stat_err[SER_ERR_OR] = SCI_FER_ERR; */
  /* else */
  /*   ser->stat_err[SER_ERR_OR] = SCI_FER_OK; */
  
/* enum enum_SCI_SSR_TEND { */
/*   // Transmit End Flag */
/*   SCI_TEND_BUSY, // A character is being transmitted. */
/*   SCI_TEND_DONE   // Character transfer has been completed. */
/* }; */

/* enum enum_SCI_SSR_PER { */
/*   // Parity Error Flag */
/*   SCI_PER_OK,   // No parity error occurred */
/*   SCI_PER_ERR   // A parity error has occurred */
/* }; */

/* enum enum_SCI_SSR_FER { */
/*   // Framing Error Flag */
/*   SCI_FER_OK,   // No framing error occurred */
/*   SCI_FER_ERR   // A framing error has occurred */
/* }; */

/* enum enum_SCI_SSR_ORER { */
/*   // Overrun Error Flag */
/*   SCI_ORER_OK,   // No overrun error occurred */
/*   SCI_ORER_ERR   // A overrun error has occurred */
/* }; */ 
  return( 0 );
}

/* end of serial.c */
