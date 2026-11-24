// tb_rec_top_jit: system test of ax7a200b_rec_top, scenarios S11 (ui_clk 100.3 MHz + jitter) (see rec_top_body.svh)
`define REC_MIG_PERIOD_PS 9970
`define REC_MIG_JITTER_PS 200
`define REC_TB_NAME tb_rec_top_jit
`define REC_SCEN_DEFAULT "S11"
`include "rec_top_body.svh"
