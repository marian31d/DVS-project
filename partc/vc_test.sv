//-----------------------------------------------------------------------------
// Module: vc_test
// Description: Simple test for verifying the packet verification component
//              Tests basic functionality of the OOP verification environment
//              (Minimal update: build VCs for all 4 ports so outputs are observed)
//-----------------------------------------------------------------------------

`timescale 1ns/1ps

module vc_test;

  // Import package
  import packet_pkg::*;

  //===========================================================================
  // Parameters
  //===========================================================================

  parameter int NUM_PORTS     = 4;
  parameter int DATA_WIDTH    = 8;
  parameter int CLK_PERIOD    = 10;
  parameter int NUM_PACKETS   = 5;   // Small number for quick test

  //===========================================================================
  // Clock and Reset
  //===========================================================================

  logic clk;
  logic rst_n;

  // Clock generation
  initial begin
	clk = 0;
	forever #(CLK_PERIOD/2) clk = ~clk;
  end

  // Reset generation
  initial begin
	rst_n = 0;
	repeat(5) @(posedge clk);
	rst_n = 1;
	$display("[%0t] Reset deasserted", $time);
  end

  //===========================================================================
  // Interface Instances
  //===========================================================================

  port_if port0(.clk(clk), .rst_n(rst_n));
  port_if port1(.clk(clk), .rst_n(rst_n));
  port_if port2(.clk(clk), .rst_n(rst_n));
  port_if port3(.clk(clk), .rst_n(rst_n));

  //===========================================================================
  // DUT Instantiation
  //===========================================================================

  switch_4port_wrap dut (
	  .clk(clk),
	  .rst_n(rst_n),
	  .port0(port0),
	  .port1(port1),
	  .port2(port2),
	  .port3(port3)
	);

  //===========================================================================
  // Verification Components
  //===========================================================================
  // vc0 drives port0 (NUM_PACKETS)
  // vc1/vc2/vc3 are passive (0 packets) but their monitors will observe outputs
  //===========================================================================

  packet_vc vc0, vc1, vc2, vc3;

  //===========================================================================
  // Optional: quick debug print to prove outputs toggle (can remove later)
  //===========================================================================

  always @(posedge clk) if (rst_n) begin
	if (port0.valid_out || port1.valid_out || port2.valid_out || port3.valid_out) begin
	  $display("[%0t] OUT v0=%0b v1=%0b v2=%0b v3=%0b | s0=%b t0=%b d0=%0h | s1=%b t1=%b d1=%0h | s2=%b t2=%b d2=%0h | s3=%b t3=%b d3=%0h",
			   $time,
			   port0.valid_out, port1.valid_out, port2.valid_out, port3.valid_out,
			   port0.source_out, port0.target_out, port0.data_out,
			   port1.source_out, port1.target_out, port1.data_out,
			   port2.source_out, port2.target_out, port2.data_out,
			   port3.source_out, port3.target_out, port3.data_out);
	end
  end

  //===========================================================================
  // Main Test
  //===========================================================================

  initial begin
	$display("\n");
	$display("############################################################");
	$display("#                                                          #");
	$display("#     Verification Component Test                          #");
	$display("#                                                          #");
	$display("############################################################");
	$display("\n");

	// Initialize all interface signals
	port0.valid_in = 0; port0.source_in = 0; port0.target_in = 0; port0.data_in = 0;
	port1.valid_in = 0; port1.source_in = 0; port1.target_in = 0; port1.data_in = 0;
	port2.valid_in = 0; port2.source_in = 0; port2.target_in = 0; port2.data_in = 0;
	port3.valid_in = 0; port3.source_in = 0; port3.target_in = 0; port3.data_in = 0;

	// Wait for reset
	@(posedge rst_n);
	repeat(3) @(posedge clk);

	//=========================================================================
	// Build Phase
	//=========================================================================
	$display("[%0t] === BUILD PHASE ===", $time);

	// vc0 drives port0
	vc0 = new("vc0", null);
	vc0.set_port_id(0);
	vc0.set_vif(port0);
	vc0.build();
	vc0.get_sequencer().set_num_packets(NUM_PACKETS);

	// vc1/vc2/vc3: passive (monitor/check outputs on ports 1/2/3)
	vc1 = new("vc1", null);
	vc1.set_port_id(1);
	vc1.set_vif(port1);
	vc1.build();
	vc1.get_sequencer().set_num_packets(0);

	vc2 = new("vc2", null);
	vc2.set_port_id(2);
	vc2.set_vif(port2);
	vc2.build();
	vc2.get_sequencer().set_num_packets(0);

	vc3 = new("vc3", null);
	vc3.set_port_id(3);
	vc3.set_vif(port3);
	vc3.build();
	vc3.get_sequencer().set_num_packets(0);

	// Hook up VCs so expected packets get routed to destination checkers
	vc0.set_other_vc(0, vc0); vc0.set_other_vc(1, vc1); vc0.set_other_vc(2, vc2); vc0.set_other_vc(3, vc3);
	vc1.set_other_vc(0, vc0); vc1.set_other_vc(1, vc1); vc1.set_other_vc(2, vc2); vc1.set_other_vc(3, vc3);
	vc2.set_other_vc(0, vc0); vc2.set_other_vc(1, vc1); vc2.set_other_vc(2, vc2); vc2.set_other_vc(3, vc3);
	vc3.set_other_vc(0, vc0); vc3.set_other_vc(1, vc1); vc3.set_other_vc(2, vc2); vc3.set_other_vc(3, vc3);


	//=========================================================================
	// Connect Phase
	//=========================================================================
	$display("[%0t] === CONNECT PHASE ===", $time);
	vc0.connect();
	vc1.connect();
	vc2.connect();
	vc3.connect();

	//=========================================================================
	// Run Phase
	//=========================================================================
	$display("[%0t] === RUN PHASE ===", $time);

	fork
		vc0.run();
		vc1.run();
		vc2.run();
		vc3.run();
	  join

	// Let packets drain
	repeat(100) @(posedge clk);
	
	//=======================================================================
	// Check Phase
	//=======================================================================
	$display("\n[%0t] === CHECK PHASE ===", $time);

	vc0.finalize_check();
	vc1.finalize_check();
	vc2.finalize_check();
	vc3.finalize_check();

	//=========================================================================
	// Report Phase
	//=========================================================================
	$display("\n[%0t] === REPORT PHASE ===", $time);
	vc0.report();
	vc1.report();
	vc2.report();
	vc3.report();
	
	if (vc0.is_passed() && vc1.is_passed() && vc2.is_passed() && vc3.is_passed())
		$display("\n*** TEST PASSED ***\n");
	  else
		$display("\n*** TEST FAILED ***\n");


	//=========================================================================
	// Final Status
	//=========================================================================
	$display("\n");
	$display("############################################################");
	$display("#                                                          #");
	$display("#              *** VC TEST COMPLETE ***                    #");
	$display("#                                                          #");
	$display("############################################################");
	$display("\n");

	#50;
	$finish;
  end

  //===========================================================================
  // Simulation Timeout
  //===========================================================================

  initial begin
	#10000;
	$display("\n[%0t] ERROR: Global simulation timeout!", $time);
	$finish;
  end

  //===========================================================================
  // Waveform Dump
  //===========================================================================

  initial begin
	$dumpfile("vc_test.vcd");
	$dumpvars(0, vc_test);
  end

endmodule
