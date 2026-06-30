`timescale 1ns/1ps

interface port_if (input logic clk, input logic rst_n);
  // Must be first items in the interface (legal SV ordering)
  timeunit 1ns;
  timeprecision 1ps;

  // TB -> DUT
  logic        valid_in;
  logic [3:0]  source_in;
  logic [3:0]  target_in;
  logic [7:0]  data_in;

  // DUT -> TB
  logic        valid_out;
  logic [3:0]  source_out;
  logic [3:0]  target_out;
  logic [7:0]  data_out;

`ifndef SYNTHESIS
  // Driver clocking (drive at posedge)
  clocking drv_cb @(posedge clk);
	output valid_in, source_in, target_in, data_in;
  endclocking

  // Monitor clocking (sample after NBA update)
  clocking mon_cb @(posedge clk);
	input  #1step valid_out, source_out, target_out, data_out;
  endclocking

  // TB modport only exists for simulation
  modport TB  (clocking drv_cb, clocking mon_cb, input clk, input rst_n);
`endif

  // DUT modport exists in both sim + synth
  modport DUT (
	input  clk, input rst_n,
	input  valid_in, source_in, target_in, data_in,
	output valid_out, source_out, target_out, data_out
  );

endinterface
