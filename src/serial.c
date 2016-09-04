
#include "typedefine.h"
#include "serial.h"
#include "dbg_utils.h"

// -------------------------------------------------------
// ----------------------------- Functions ( subroutines )
//
//  serial (SCI) utilities 
//
uint8_t SerGenBRR( st_Serial *ser,
		   uint32_t brate, uint8_t xtal_mhz ){
  uint8_t A = 32;
  uint8_t pclk = 48;
  uint8_t  n_calc = 3;
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
  volatile struct st_sci0 *sci = ser->addr_base;
  A = ( sci->SEMR.BIT.ABCS ? 32 : 64 );
  // calc 2^(2n-1) *2
  switch( sci->SMR.BIT.CKS ){
  case SCI_CKS_PCLK:  // n=0
    n_calc = 1;
    break;
  case SCI_CKS_PCLK_4:  // n=1
    n_calc = 4;
    break;
  case SCI_CKS_PCLK_16:  // n=2
    n_calc = 16;
    break;
  case SCI_CKS_PCLK_64:  // n=3
    n_calc = 64;
    break;
  default:
    n_calc = 64;
    break;
  }
  // calc
  /* return( pclk * 1000000 / ( A * n_calc * brate ) ); */
  return( pclk * 1000000 / ( A * n_calc / 2 * brate ) - 1 );
}


void SerPortInit( enum enum_SERI_CHID ch ){
  switch( ch ){
  case SER_MSP:
    // PE1, PE2 -> UART Port setting (Rx/Tx) 
    PORTC.PCR.BIT.B2   = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
    PORTC.PODR.BIT.B3  = 0;
    PORTC.PDR.BIT.B2   = PDR_IN;           // PDR_IN / PDR_OUT
    PORTC.PDR.BIT.B3   = PDR_OUT;          // PDR_IN / PDR_OUT    
    PORTC.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
    PORTC.PMR.BIT.B3   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
    
    MPC.PC2PFS.BIT.PSEL  = PC2PFS_RXD5;
    MPC.PC3PFS.BIT.PSEL  = PC3PFS_TXD5;
    PORTC.PMR.BIT.B2     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
    PORTC.PMR.BIT.B3     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
    break;
  case SER_TELEM:
    // PE1, PE2 -> UART Port setting (Rx/Tx)
    PORTE.PODR.BIT.B1  = 0;
    PORTE.PCR.BIT.B2   = PCR_PULLUP;         // PCR_OPEN / PCR_PULLUP
    PORTE.PDR.BIT.B1   = PDR_OUT;          // PDR_IN / PDR_OUT
    PORTE.PDR.BIT.B2   = PDR_IN;           // PDR_IN / PDR_OUT
    PORTE.PMR.BIT.B1   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC
    PORTE.PMR.BIT.B2   = PMR_GPIO;         // PMR_GPIO / PMR_FUNC

    MPC.PE1PFS.BIT.PSEL  = PE1PFS_TXD12;
    MPC.PE2PFS.BIT.PSEL  = PE2PFS_RXD12;
    PORTE.PMR.BIT.B1     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
    PORTE.PMR.BIT.B2     = PMR_FUNC;       // PMR_GPIO / PMR_FUNC
    break;
  default:
    break;
  }
}

void SerReadStart( st_Serial *ser ){
  volatile struct st_sci0 *sci = ser->addr_base;
  // IRQ base vector
  uint8_t vec = ser->vec_base; // 0: RXI, +1: TXI, +2: TEI
  // clear IRQ flag
  ICU.IR[vec].BIT.IR = 0;
  // reboot UART transmission
  sci->SCR.BIT.RIE  = SCI_RIE_EN;     // RXI and ERI interrupt requests are enabled
  sci->SCR.BIT.RE   = SCI_RE_EN;      // Serial reception is enabled
}

void SerWriteStart( st_Serial *ser ){
  volatile struct st_sci0 *sci = ser->addr_base;
  // IRQ base vector
  uint8_t vec = ser->vec_base; // 0: RXI, +1: TXI, +2: TEI
  // clear IRQ flag
  ICU.IR[vec+1].BIT.IR = 0;
  ICU.IR[vec+2].BIT.IR = 0;
  // reboot UART transmission
  sci->SCR.BIT.TEIE = SCI_TEIE_EN;    // A TEI interrupt request is enabled
  sci->SCR.BIT.TIE  = SCI_TIE_EN;     // A TXI interrupt request is enabled
  sci->SCR.BIT.TE   = SCI_TE_EN;      // Serial transmission is enabled (* after TIE set)
}

void SerStart( st_Serial *ser ){
  // start to communicate via UART
  // RX
  SerReadStart( ser );
  // TX
  SerWriteStart( ser );
}

void SerReadStop( st_Serial *ser ){
  // SFR base address
  volatile struct st_sci0 *sci = ser->addr_base;
  // IRQ base vector
  uint8_t vec = ser->vec_base; // 0: RXI, +1: TXI, +2: TEI
  // suspend UART transmission
  sci->SCR.BIT.RIE  = SCI_RIE_DE;     // RXI and ERI interrupt requests are disabled
  sci->SCR.BIT.RE   = SCI_RE_DE;      // Serial reception is disabled
  // clear IRQ flag
  ICU.IR[vec].BIT.IR = 0;
}

void SerWriteStop( st_Serial *ser ){
  // SFR base address
  volatile struct st_sci0 *sci = ser->addr_base;
  // IRQ base vector
  uint8_t vec = ser->vec_base; // 0: RXI, +1: TXI, +2: TEI
  // suspend UART transmission
  sci->SCR.BIT.TEIE = SCI_TEIE_DE;    // A TEI interrupt request is enabled
  sci->SCR.BIT.TIE  = SCI_TIE_DE;     // A TXI interrupt request is enabled
  sci->SCR.BIT.TE   = SCI_TE_DE;      // Serial transmission is enabled (* after TIE set)
  // clear IRQ flag
  ICU.IR[vec+1].BIT.IR = 0;
  ICU.IR[vec+2].BIT.IR = 0;
}

void SerStop( st_Serial *ser ){
  volatile struct st_sci0 *sci = ser->addr_base;
  // stop to communicate via UART
  // RX
  SerReadStop( ser );
  // TX
  SerWriteStop( ser );
}

uint8_t SerFuncInit( struct st_Serial *ser, uint32_t brate ){
  volatile struct st_sci0 *sci = ser->addr_base;
  // UART function settings
  sci->SCR.BIT.CKE    = SCI_CKE_OCHPIO;  // On-chip baud rate generator (SCK as I/O)
  sci->SIMR1.BIT.IICM = SCI_IICM_SCIF;   // Serial interface mode
  sci->SPMR.BIT.CKPOL = SCI_CKPOL_NORM;  // Clock polarity is not inverted.
  sci->SPMR.BIT.CKPH  = SCI_CKPH_NORM;   // Clock is not delayed.
  sci->SPMR.BIT.CTSE  = SCI_CTSE_DE;     // CTS pin function is disabled
  sci->SCMR.BIT.SMIF  = SCI_SMIF_SCIF;   // Serial communications interface mode
  sci->SCMR.BIT.SINV  = SCI_SINV_NORM;   // TDR contents are transmitted as they are.
  sci->SCMR.BIT.SDIR  = SCI_SDIR_LSB;    // Transfer with LSB-first
  /* sci.SEMR.ACS0  = SCI_ACS0_EXCLK;  // External clock input */
  sci->SEMR.BIT.ABCS  = SCI_ABCS_16CLK;  // Selects 16 base clock cycles for 1-bit period
  sci->SMR.BIT.CKS    = SCI_CKS_PCLK;   // PCLK / 1
  sci->SMR.BIT.CM     = SCI_CM_ASYNC;   // Asynchronous mode
  sci->SMR.BIT.CHR    = SCI_CHR_8;      // Selects 8 bits as the data length
  sci->SMR.BIT.STOP   = SCI_STOP_1;     // 1 stop bit
  sci->SMR.BIT.PE     = SCI_PE_DE;      // Parity bit addition is not performed
  /* sci.SMR.BIT.PM   = SCI_PM_ODD      // Select odd parity */
  sci->SMR.BIT.MP     = SCI_MP_DE;      // disabled
  /* sci.SCR.MPIE = SCI_MPIE_NORM;  // Normal reception */
  /* sci.SSR.MPB = SCI_MPB_ID;    // ID transmission cycles */
  /* sci.SSR.MPBT = SCI_MPBT_ID;    // ID transmission cycles */

  // BAUDRATE = 9600 bps
  // PCLK => 12MHz * 4
  // CKS => PCLK / 1, n=0
  // calc: N = 48 * 10^6 / ( 64 * 2^(2n-1) * 9600 ) - 1
  //         = 3*2*2 * 10^6 / ( 2^6 * 4800 )        - 1
  //         = 10^6 / ( 2^6 * 100 )                 - 1
  //         = 10^4 / ( 2^6 )                       - 1
  //         = 10000 / 64                          - 1
  //         = 1.562 * 100 - 1
  //sci->BRR = 155;
  sci->BRR = SerGenBRR( ser, brate, XTAL_MHZ );
}

uint8_t SerInit( st_Serial *ser, uint32_t brate, enum enum_SERI_CHID ch ){
  // var init
  ser->tx_stat = 0;
  ser->tx_head = 0;
  ser->tx_tail = 0;
  ser->rx_stat = 0;
  ser->rx_head = 0;
  ser->rx_tail = 0;
  // gen SCI SFR addr_base for each ch
  switch( ch ){
  case SER_TELEM:
    ser->addr_base     = (struct st_sci0 *)(0x0008B300);
    break;
  default:
    ser->addr_base     = (struct st_sci0 *)(0x0008A000 + (uint32_t)(0x20 * ch));
    break;
  }
  //    ser->addr_irq_base = IR(SCI0, TXI0) + (struct st_icu  *)(0x04 * 3 * ch);
  ser->vec_base = 214 + (3 * ch);    

  // UART port initialization
  SerPortInit( ch );
  SerStop( ser );
  SerFuncInit( ser, brate );
  // start to communicate with UART
  SerStart( ser );
  return( 0 );
}

uint8_t SerWriteEncue( st_Serial *ser, uint8_t byte ){
  // skip: full
  /* if( ser->tx_tail == (SER_BUFF - 1) ) */
  /*   return( 1 ); */
  /* if( (ser->tx_head == 0) && (ser->tx_tail == (SER_BUFF-1)) ) */
  /*   return( 1 ); */
  if( ser->tx_tail == (ser->tx_head - 1) )
    return( 1 );
  // run: byte store
  ser->tx_buff[ser->tx_tail++] = byte;
  //ser->tx_tail = ser->tx_tail % SER_BUFF;
  return( 0 );
}

uint8_t SerWriteDecue( st_Serial *ser ){
  // addr base conv
  volatile struct st_sci0 *sci = ser->addr_base;

  /* ser->tx_head = 10; */
  /* ser->tx_tail = SER_BUFF - 1; */
  // skip: empty
  if( (ser->tx_head) == (ser->tx_tail) ){
    //sci->TDR     = 0x30 + ((ser->tx_tail) % 10);
    return( 1 );
  }
  // run: byte transmit
  sci->TDR     = ser->tx_buff[ser->tx_head++];
  //sci->TDR     = ser->tx_tail;
  //sci->TDR     = ser->tx_head++;
  /* ser->tx_head = ser->tx_head % SER_BUFF; */
  return( 0 );
}

uint8_t SerWriteBG( st_Serial *ser ){
  // base addr
  volatile struct st_sci0 *sci = ser->addr_base;
  uint8_t vec = ser->vec_base + 1; // 0: RXI, +1: TXI, +2: TEI

  //  if( ICU.IR[ser->vec_base].BIT.IR ){
  if( ICU.IR[vec].BIT.IR ){
    // cue is empty
    if( SerWriteDecue( ser ) ){
      //SerWriteStop( ser );
      ser->tx_stat = 0;
      ser->tx_head = 0;
      ser->tx_tail = 0;
    }else{
      ICU.IR[vec].BIT.IR = 0;      
    }
    //    ICU.IR[ser->vec_base].BIT.IR = 0;      
  }

  return(0);
}

uint8_t SerWrite( st_Serial *ser, uint8_t byte ){
  // UART port initialization
  // start to transmission
  SerWriteEncue( ser, byte );
  //SerWriteStart( ser );

  return( 0 );
}

uint8_t SerBytesWrite( st_Serial *ser, uint8_t *bytes ){
  // stack to cue unless byte strings end
  uint8_t i = 0;
  while( bytes[i] != 0 ){
    SerWriteEncue( ser, bytes[i] );
    i++;
  }
  //SerWriteStart( ser );

  return( 0 );
}

/* uint8_t SerWriteTest( st_Serial *ser, uint8_t byte ){ */
/*   // temp var (vector) */
/*   volatile struct st_sci0 *sci = ser->addr_base;   */
/*   uint8_t vec = ser->vec_base + 1; // RXI, TXI, TEI */
/*   //  SerWriteStart( ser ); */
/*   SerStart( ser ); */
  
/*   if( ICU.IR[vec].BIT.IR ){ */
/*     sci->TDR = byte; */
/*     ICU.IR[vec].BIT.IR = 0; */
/*   } */

/*   /\* if( IR(SCI5, TXI5) ){ *\/ */
/*   /\*   SCI5.TDR = byte; *\/ */
/*   /\*   IR(SCI5, TXI5) = 0 ; *\/ */
/*   /\* } *\/ */
/*   return(0); */
/* } */

uint8_t SerReadEncue( st_Serial *ser ){
  // temp var
  volatile struct st_sci0 *sci = ser->addr_base;  
  // skip: full
  /* if( ser->rx_head == 0 && ser->rx_tail == SER_BUFF ) */
  /*   return(1); */
  if( ser->rx_tail == ser->rx_head - 1 )
    return(1);
  // run: data store
  ser->rx_buff[ser->rx_tail++] = sci->RDR;
  /* ser->rx_tail = (ser->rx_tail) % SER_BUFF; */
  return(0);
}

uint8_t SerReadDecue( st_Serial *ser ){
  // temp var
  uint8_t ret;
  // skip: empty
  if( ser->rx_head == ser->rx_tail )
    return(0);
  // run: data extract
  ret = ser->rx_buff[ser->rx_head++];
  /* ser->rx_head = ser->rx_head % SER_BUFF; */
  return( ret );
}

uint8_t SerReadBG( st_Serial *ser ){
  // temp var
  uint8_t vec = ser->vec_base; // RXI, TXI, TEI
  // next data is available in data register
  if( ICU.IR[vec].BIT.IR ){
    // cue is full
    if( SerReadEncue( ser ) ){
    }else{
      ICU.IR[vec].BIT.IR = 0;
    }
    //    ICU.IR[ser->vec_base].BIT.IR = 0;      
  }
  // start to receive
  return( 0 );
}

uint8_t SerRead( st_Serial *ser ){
  // temp var
  uint8_t ret;
  //
  if( ret = SerReadDecue( ser ) ){
  }else{
    ser->rx_stat = 0;
    ser->rx_head = 0;
    ser->rx_tail = 0;    
  }
  return( ret );
}

uint8_t *SerBytesRead( st_Serial *ser, uint8_t size ){
  // UART port initialization
  for( int i=0 ; i<size ; i++ ){
    
  }
  // start to receive
  return( 0 );
}

uint8_t SerReadTest( st_Serial *ser ){
  // temp var
  volatile struct st_sci0 *sci = ser->addr_base;  
  uint8_t vec = ser->vec_base; // RXI, TXI, TEI
  uint8_t ret;
  // test run
  if( IR( SCI12, RXI12 ) ){
    ret = SCI12.RDR;
    IR( SCI12, RXI12 ) = 0;
  }else{
    ret = 0x30 + IR( SCI12, RXI12 );
    SerReadStop( ser );
    SerReadStart( ser );
  }
  return( ret );
}



void Ser12PortInit( void ){
  // PE1, PE2 -> UART Port setting (Rx/Tx)
  PORTE.PODR.BIT.B1  = 0;
  PORTE.PCR.BIT.B1   = PCR_OPEN;         // PCR_OPEN / PCR_PULLUP
  PORTE.PCR.BIT.B2   = PCR_PULLUP;       // PCR_OPEN / PCR_PULLUP
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
  // gen SCI ch addr_base
  ser->addr_base =  (struct st_sci0 *)0x8B300;
  // UART port initialization
  Ser12PortInit();
  Ser12Stop();
  Ser12FuncInit( ser, brate );
  // start to communicate with UART
  Ser12Start();
  IEN( SCI12, RXI12 ) = 0;
  IEN( SCI12, TXI12 ) = 0;
  IEN( SCI12, TEI12 ) = 0;
  IPR( SCI12, RXI12 ) = 8;
  IPR( SCI12, TXI12 ) = 8;
  IPR( SCI12, TEI12 ) = 8;  
}

uint8_t Ser12Write( st_Serial *ser ){
  // UART port initialization
  
  // start to transmissions
  return( 0 );
}

uint8_t Ser12ReadEncue( st_Serial *ser ){
  // skip: full
  if( ser->rx_head == 0 && ser->rx_tail == SER_BUFF )
    return(1);
  if( ser->rx_tail == ser->rx_head - 1 )
    return(1);
  // run: data store
  ser->rx_buff[ser->rx_tail++] = SCI12.RDR;
  ser->rx_tail = (ser->rx_tail) % SER_BUFF;
  return(0);
}

uint8_t Ser12ReadDecue( st_Serial *ser ){
  // temp var
  uint8_t ret;

  // skip: empty
  if( ser->rx_head == ser->rx_tail ){
    return(0);
  }
  // run: data extract
  ret = ser->rx_buff[ser->rx_head++];
  ser->rx_head = ser->rx_head % SER_BUFF;
    
  return( ret );
}

uint8_t Ser12ReadBG( st_Serial *ser ){
  // start to back-ground routine for data receive
  if( IR( SCI12, RXI12 ) ){
    //    SCI12.SCR.BIT.RE   = SCI_RE_EN;      // Serial reception is disabled
    /* if( SCI12.SSR.BIT.ORER ){ */
    /*   ser->stat[SER_ORD]; */
    /* } */
    /* if( IR( SCI12, REI12) ){ */
    /*   ser->stat[SER_ERR_PF]; */
    /*   //      IR( SCI12, ERI12 ) = 0; */
    /*   return( 1 ); */
    /* } */
    Ser12ReadEncue( ser );
    //SCI12.SCR.BIT.RE   = SCI_RE_EN;      // Serial reception is disabled
    // clear flag for next read
    IR( SCI12, RXI12 ) = 0;
  }
  return( 0 );
}

uint8_t Ser12Read( st_Serial *ser ){
  // get buffered value
  return( Ser12ReadDecue( ser ) );
}

uint8_t Ser12ReadTest( st_Serial *ser ){
  // temp var
  uint8_t ret = 0xAA;
  // get buffered value
  if( IR( SCI12, RXI12 ) ){
    ret = SCI12.RDR;
    IR( SCI12, RXI12 ) = 0;
  }

  return( ret );
}

uint8_t SerGetStat( st_Serial *ser ){
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
