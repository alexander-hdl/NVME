// =============================================================================
// vio_status_sim.sv - simulation stand-in for the Vivado VIO IP `vio_status` (26 probe_in, 12 probe_out), WP8.
// probe_out values are plain variables po0..po11 that a testbench sets hierarchically (<top>.u_vio.po1 = 1);
// the probe_in values are ignored.
// =============================================================================
module vio_status (
  input  logic         clk,
  input  logic [7:0]   probe_in0,  input logic [15:0] probe_in1,  input logic [7:0]   probe_in2,
  input  logic [1:0]   probe_in3,  input logic [6:0]  probe_in4,  input logic [5:0]   probe_in5,
  input  logic [63:0]  probe_in6,  input logic [63:0] probe_in7,  input logic [31:0]  probe_in8,
  input  logic [31:0]  probe_in9,  input logic [39:0] probe_in10, input logic [39:0]  probe_in11,
  input  logic [63:0]  probe_in12, input logic [31:0] probe_in13, input logic [63:0]  probe_in14,
  input  logic [63:0]  probe_in15, input logic [1:0]  probe_in16, input logic [63:0]  probe_in17,
  input  logic [3:0]   probe_in18, input logic [39:0] probe_in19, input logic [39:0]  probe_in20,
  input  logic [39:0]  probe_in21, input logic [32:0] probe_in22, input logic [103:0] probe_in23,
  input  logic [31:0]  probe_in24, input logic [15:0] probe_in25,
  output logic         probe_out0, output logic       probe_out1, output logic        probe_out2,
  output logic         probe_out3, output logic [1:0] probe_out4, output logic [1:0]  probe_out5,
  output logic         probe_out6, output logic [31:0] probe_out7, output logic [31:0] probe_out8,
  output logic         probe_out9, output logic [31:0] probe_out10, output logic [9:0] probe_out11
);
  logic        po0 = 1'b0, po1 = 1'b0, po2 = 1'b0, po3 = 1'b0, po6 = 1'b0, po9 = 1'b0;
  logic [1:0]  po4 = 2'd0, po5 = 2'd0;
  logic [31:0] po7 = 32'd0, po8 = 32'd0, po10 = 32'd0;
  logic [9:0]  po11 = 10'd0;
  assign {probe_out0, probe_out1, probe_out2, probe_out3, probe_out4, probe_out5, probe_out6, probe_out7,
          probe_out8, probe_out9, probe_out10, probe_out11} =
         {po0, po1, po2, po3, po4, po5, po6, po7, po8, po9, po10, po11};
endmodule
