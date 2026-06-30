import packet_pkg::*;

module switch_test;
  timeunit 1ns;
  timeprecision 1ps;

  parameter int CLK_PERIOD  = 10;
  parameter int NUM_PACKETS = 20;

  logic clk, rst_n;

  // Coverage sampled signals
  logic [3:0] in_source, in_target, out_source, out_target;

  // TB-derived ?state? (no hierarchical DUT peek)
  typedef enum logic [1:0] {ST_IDLE=2'd0, ST_ACTIVE=2'd1, ST_DRAIN=2'd2} tb_state_t;
  tb_state_t tb_state;

  // Clock
  initial begin
	clk = 0;
	forever #(CLK_PERIOD/2) clk = ~clk;
  end

  // Interfaces
  port_if port0(.clk(clk), .rst_n(rst_n));
  port_if port1(.clk(clk), .rst_n(rst_n));
  port_if port2(.clk(clk), .rst_n(rst_n));
  port_if port3(.clk(clk), .rst_n(rst_n));

  // DUT
  switch_4port dut (
	.clk(clk), .rst_n(rst_n),
	.port0(port0), .port1(port1), .port2(port2), .port3(port3)
  );

  // VCs
  packet_vc vc0, vc1, vc2, vc3;

  //-------------------------------------------------------------------------
  // Coverage
  //-------------------------------------------------------------------------
  covergroup cg_in;
	option.per_instance = 1;
	cp_in_source : coverpoint in_source {
	  bins src0 = {4'b0001}; bins src1 = {4'b0010};
	  bins src2 = {4'b0100}; bins src3 = {4'b1000};
	}
	cp_in_target : coverpoint in_target {
	  bins unicast   = {4'b0001,4'b0010,4'b0100,4'b1000};
	  bins multicast = {4'b0011,4'b0111,4'b0110,4'b1110,4'b1100,4'b1001,4'b1011,4'b1101};
	  bins broadcast = {4'b1111};
	}
	in_x : cross cp_in_source, cp_in_target;
  endgroup

  covergroup cg_out;
	option.per_instance = 1;
	cp_out_source : coverpoint out_source {
	  bins src0 = {4'b0001}; bins src1 = {4'b0010};
	  bins src2 = {4'b0100}; bins src3 = {4'b1000};
	}
	cp_out_target : coverpoint out_target {
	  bins unicast   = {4'b0001,4'b0010,4'b0100,4'b1000};
	  bins multicast = {4'b0011,4'b0111,4'b0110,4'b1110,4'b1100,4'b1001,4'b1011,4'b1101};
	  bins broadcast = {4'b1111};
	}
	out_x : cross cp_out_source, cp_out_target;
  endgroup

  covergroup cg_state;
	option.per_instance = 1;
	cp_state : coverpoint tb_state iff (rst_n) {
	  bins IDLE   = {ST_IDLE};
	  bins ACTIVE = {ST_ACTIVE};
	  bins DRAIN  = {ST_DRAIN};
	}
  endgroup

  cg_in    cov_in    = new();
  cg_out   cov_out   = new();
  cg_state cov_state = new();

  //-------------------------------------------------------------------------
  // Input coverage sampling
  //-------------------------------------------------------------------------
  always_ff @(posedge clk) if (rst_n) begin
	if (port0.valid_in) begin
	  in_source <= 4'b0001;
	  in_target <= port0.target_in;
	  cov_in.sample();
	end
	if (port1.valid_in) begin
	  in_source <= 4'b0010;
	  in_target <= port1.target_in;
	  cov_in.sample();
	end
	if (port2.valid_in) begin
	  in_source <= 4'b0100;
	  in_target <= port2.target_in;
	  cov_in.sample();
	end
	if (port3.valid_in) begin
	  in_source <= 4'b1000;
	  in_target <= port3.target_in;
	  cov_in.sample();
	end
  end

  //-------------------------------------------------------------------------
  // Output coverage sampling
  //-------------------------------------------------------------------------
  always_ff @(posedge clk) if (rst_n) begin
	if (port0.valid_out) begin
	  out_source <= port0.source_out;
	  out_target <= port0.target_out;
	  cov_out.sample();
	end
	if (port1.valid_out) begin
	  out_source <= port1.source_out;
	  out_target <= port1.target_out;
	  cov_out.sample();
	end
	if (port2.valid_out) begin
	  out_source <= port2.source_out;
	  out_target <= port2.target_out;
	  cov_out.sample();
	end
	if (port3.valid_out) begin
	  out_source <= port3.source_out;
	  out_target <= port3.target_out;
	  cov_out.sample();
	end
  end

  //-------------------------------------------------------------------------
  // TB-derived ?state? sampling (no DUT internals!)
  // ACTIVE  = any input valid
  // DRAIN   = no inputs but outputs are still producing
  // IDLE    = nothing happening
  //-------------------------------------------------------------------------
  always_ff @(posedge clk) if (rst_n) begin
	logic has_in, has_out;
	has_in  = port0.valid_in  | port1.valid_in  | port2.valid_in  | port3.valid_in;
	has_out = port0.valid_out | port1.valid_out | port2.valid_out | port3.valid_out;

	unique case ({has_in, has_out})
	  2'b00: tb_state <= ST_IDLE;
	  2'b10: tb_state <= ST_ACTIVE;
	  2'b01: tb_state <= ST_DRAIN;
	  2'b11: tb_state <= ST_ACTIVE;
	endcase

	cov_state.sample();
  end

  //-------------------------------------------------------------------------
  // Reset
  //-------------------------------------------------------------------------
  initial begin
	rst_n = 0;
	repeat (5) @(posedge clk);
	rst_n = 1;
	$display("[%0t] Reset deasserted", $time);
  end

  //-------------------------------------------------------------------------
  // Main test
  //-------------------------------------------------------------------------
  initial begin
	$display("\n############################################################");
	$display("#     4-Port Switch Verification Test                      #");
	$display("############################################################\n");

	// Init
	port0.valid_in=0; port0.source_in=0; port0.target_in=0; port0.data_in=0;
	port1.valid_in=0; port1.source_in=0; port1.target_in=0; port1.data_in=0;
	port2.valid_in=0; port2.source_in=0; port2.target_in=0; port2.data_in=0;
	port3.valid_in=0; port3.source_in=0; port3.target_in=0; port3.data_in=0;

	@(posedge rst_n);
	repeat (3) @(posedge clk);

	$display("[%0t] === BUILD PHASE ===", $time);
	vc0 = new("vc0", null); vc0.set_port_id(0); vc0.set_vif(port0); vc0.build();
	vc1 = new("vc1", null); vc1.set_port_id(1); vc1.set_vif(port1); vc1.build();
	vc2 = new("vc2", null); vc2.set_port_id(2); vc2.set_vif(port2); vc2.build();
	vc3 = new("vc3", null); vc3.set_port_id(3); vc3.set_vif(port3); vc3.build();

	vc0.get_sequencer().set_num_packets(NUM_PACKETS);
	vc1.get_sequencer().set_num_packets(NUM_PACKETS);
	vc2.get_sequencer().set_num_packets(NUM_PACKETS);
	vc3.get_sequencer().set_num_packets(NUM_PACKETS);

	vc0.set_other_vc(0,vc0); vc0.set_other_vc(1,vc1); vc0.set_other_vc(2,vc2); vc0.set_other_vc(3,vc3);
	vc1.set_other_vc(0,vc0); vc1.set_other_vc(1,vc1); vc1.set_other_vc(2,vc2); vc1.set_other_vc(3,vc3);
	vc2.set_other_vc(0,vc0); vc2.set_other_vc(1,vc1); vc2.set_other_vc(2,vc2); vc2.set_other_vc(3,vc3);
	vc3.set_other_vc(0,vc0); vc3.set_other_vc(1,vc1); vc3.set_other_vc(2,vc2); vc3.set_other_vc(3,vc3);

	$display("[%0t] === CONNECT PHASE ===", $time);
	vc0.connect(); vc1.connect(); vc2.connect(); vc3.connect();

	$display("[%0t] === RUN PHASE ===", $time);
	fork
	  vc0.run();
	  vc1.run();
	  vc2.run();
	  vc3.run();
	join

	$display("[%0t] Waiting for packets to drain...", $time);
	repeat (50) @(posedge clk);

	$display("\n[%0t] === CHECK PHASE ===", $time);
	vc0.finalize_check();
	vc1.finalize_check();
	vc2.finalize_check();
	vc3.finalize_check();

	$display("\n[%0t] === REPORT PHASE ===", $time);
	vc0.report(); vc1.report(); vc2.report(); vc3.report();

	$display("COV (STATE) = %0.2f%%", cov_state.get_coverage());
	$display("COV (IN)    = %0.2f%%", cov_in.get_coverage());
	$display("COV (OUT)   = %0.2f%%", cov_out.get_coverage());

	if (vc0.is_passed() && vc1.is_passed() && vc2.is_passed() && vc3.is_passed()) begin
	  $display("\n*** TEST PASSED ***\n");
	end else begin
	  $display("\n*** TEST FAILED ***\n");
	end

	#100;
	$finish;
  end

  // Timeout
  initial begin
	#50000;
	$display("\n[%0t] ERROR: Timeout!", $time);
	$finish;
  end

endmodule
