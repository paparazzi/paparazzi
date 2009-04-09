#ifndef LPC_CAN_H
#define LPC_CAN_H

typedef struct
{
  REG32 afmr;
  REG32 sff_sa;
  REG32 sff_grp_sa;
  REG32 eff_sa;
  REG32 eff_grp_sa;
  REG32 end_of_table;
  REG32 lut_err_ad;
  REG32 lut_err_reg;
} can_accept_Regs_t;


typedef struct
{
  REG32 tx_sr;
  REG32 rx_sr;
  REG32 m_sr;
} can_central_Regs_t;



typedef struct
{
  REG32 can_mod;
  REG32 can_cmr;
  REG32 can_gsr;
  REG32 can_icr;
  REG32 can_ier;
  REG32 can_btr;
  REG32 can_ewl;
  REG32 can_sr;
  REG32 can_rfs;
  REG32 can_rid;
  REG32 can_rda;
  REG32 can_rdb;
  REG32 can_tfi1;
  REG32 can_tid1;
  REG32 can_tda1;
  REG32 can_tdb1;
  REG32 can_tfi2;
  REG32 can_tid2;
  REG32 can_tda2;
  REG32 can_tdb2;
  REG32 can_tfi3;
  REG32 can_tid3;
  REG32 can_tda3;
  REG32 can_tdb3;
} can_Regs_t;


#endif /* LPC_CAN_H */


