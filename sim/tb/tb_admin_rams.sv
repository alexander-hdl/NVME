// tb_admin_rams.sv - same scenarios as tb_nvme_admin, but nvme_admin talks to the REAL
// hostmem_rams (sdp_ram, CQ clear FSM) instead of behavioural RAM models.
`timescale 1ns/1ps
module tb_admin_rams;
  tb_nvme_admin #(.REAL_RAMS(1'b1)) u_tb ();
endmodule
