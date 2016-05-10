

// MTU setting (after, to be functional to another file )
  MTU3.TCR.BIT.TPSC = TPSC_PCLK_16;  // PCLK/16 ( PCLK: 50MHz )
  MTU3.TCR.BIT.CKEG = CKEG_PEDGE;    // Pos edge
  MTU3.TCR.BIT.CCLR = CCLR_TGRC;     // Both edge
  MTU3.TMDR.BIT.MD =  TMDR_MD_NORMAL;     // Normal mode
  MTU3.TMDR.BIT.BFA = TMDR_BFx_BUFF;     // Buffered operation
  MTU3.TMDR.BIT.BFB = TMDR_BFx_NORM;     // unbuffered	