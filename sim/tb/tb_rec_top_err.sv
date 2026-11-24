// tb_rec_top_err: system test of ax7a200b_rec_top, scenarios S12 NEG_ABORT NEG_LINK (see rec_top_body.svh)
`define REC_IO_TIMEOUT_MS 800
`define REC_TB_NAME tb_rec_top_err
`define REC_SCEN_DEFAULT "S12,NEG_ABORT,NEG_LINK"
`include "rec_top_body.svh"
