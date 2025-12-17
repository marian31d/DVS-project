//-----------------------------------------------------------------------------
// Module: tb_basic
// Description: Basic RTL QA Testbench for Stage A Verification
//              Tests single-destination, multicast, and broadcast routing
//              + Logs each input drive and each valid output event.
//-----------------------------------------------------------------------------

module tb_basic;

  //===========================================================================
  // Parameters
  //===========================================================================

  parameter int NUM_PORTS   = 4;
  parameter int DATA_WIDTH  = 8;
  parameter int CLK_PERIOD  = 10;  // 100 MHz

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
  // Interface Instances (for DUT connection)
  //===========================================================================

  port_if port0(.clk(clk), .rst_n(rst_n));
  port_if port1(.clk(clk), .rst_n(rst_n));
  port_if port2(.clk(clk), .rst_n(rst_n));
  port_if port3(.clk(clk), .rst_n(rst_n));

  //===========================================================================
  // DUT Instantiation
  //===========================================================================

  switch_4port #(
	.NUM_PORTS (NUM_PORTS),
	.DATA_WIDTH(DATA_WIDTH)
  ) dut (
	.clk   (clk),
	.rst_n (rst_n),
	.port0 (port0),
	.port1 (port1),
	.port2 (port2),
	.port3 (port3)
  );

  //===========================================================================
  // Test Counters
  //===========================================================================

  int test_pass = 0;
  int test_fail = 0;
  int packets_sent = 0;
  int packets_received [4];  // Per-port receive counter

  //===========================================================================
  // Logging helpers
  //===========================================================================

  function automatic string port_name(int p);
	case (p)
	  0: return "P0";
	  1: return "P1";
	  2: return "P2";
	  3: return "P3";
	  default: return "PX";
	endcase
  endfunction

  task automatic log_in(int p, logic [3:0] s, logic [3:0] t, logic [7:0] d);
	$display("[%0t] IN  %s : source=%b target=%b data=%b",
			 $time, port_name(p), s, t, d);
  endtask

  task automatic log_out(int p, logic v, logic [3:0] s, logic [3:0] t, logic [7:0] d);
	$display("[%0t] OUT %s : valid=%0d source=%b target=%b data=%b",
			 $time, port_name(p), v, s, t, d);
  endtask

  //===========================================================================
  // Output monitors: print only when valid_out is 1
  //===========================================================================

  task automatic monitor_output(int p);
	forever begin
	  @(posedge clk);
	  case (p)
		0: if (port0.valid_out) log_out(0, port0.valid_out, port0.source_out, port0.target_out, port0.data_out);
		1: if (port1.valid_out) log_out(1, port1.valid_out, port1.source_out, port1.target_out, port1.data_out);
		2: if (port2.valid_out) log_out(2, port2.valid_out, port2.source_out, port2.target_out, port2.data_out);
		3: if (port3.valid_out) log_out(3, port3.valid_out, port3.source_out, port3.target_out, port3.data_out);
	  endcase
	end
  endtask

  // Start monitors
  initial begin
	// Wait for reset release so you don't log garbage during reset
	@(posedge rst_n);
	fork
	  monitor_output(0);
	  monitor_output(1);
	  monitor_output(2);
	  monitor_output(3);
	join_none
  end

  //===========================================================================
  // Task: drive_packet
  // Drives a packet on specified port for one cycle
  //===========================================================================

  task automatic drive_packet(
	input int port_num,
	input logic [3:0] source,
	input logic [3:0] target,
	input logic [7:0] data
  );
	@(posedge clk);

	// Log at the cycle we assert valid_in
	log_in(port_num, source, target, data);

	case (port_num)
	  0: begin
		port0.valid_in  <= 1'b1;
		port0.source_in <= source;
		port0.target_in <= target;
		port0.data_in   <= data;
	  end
	  1: begin
		port1.valid_in  <= 1'b1;
		port1.source_in <= source;
		port1.target_in <= target;
		port1.data_in   <= data;
	  end
	  2: begin
		port2.valid_in  <= 1'b1;
		port2.source_in <= source;
		port2.target_in <= target;
		port2.data_in   <= data;
	  end
	  3: begin
		port3.valid_in  <= 1'b1;
		port3.source_in <= source;
		port3.target_in <= target;
		port3.data_in   <= data;
	  end
	endcase

	@(posedge clk);
	// Deassert after one cycle
	port0.valid_in <= 1'b0;
	port1.valid_in <= 1'b0;
	port2.valid_in <= 1'b0;
	port3.valid_in <= 1'b0;

	packets_sent++;
  endtask

  //===========================================================================
  // Task: wait_and_check_output
  // Waits for output on specified port(s) and verifies data
  //===========================================================================

  task automatic wait_and_check(
	input int expected_port,
	input logic [3:0] expected_source,
	input logic [3:0] expected_target,
	input logic [7:0] expected_data,
	input int timeout = 20
  );
	int cycles = 0;
	logic received = 0;

	while (cycles < timeout && !received) begin
	  @(posedge clk);
	  cycles++;

	  // Check port 0
	  if (expected_port == 0 && port0.valid_out) begin
		if (port0.source_out == expected_source &&
			port0.target_out == expected_target &&
			port0.data_out   == expected_data) begin
		  $display("[%0t] PASS: Port 0 received expected packet (data=0x%h)", $time, port0.data_out);
		  test_pass++; received = 1; packets_received[0]++;
		end else begin
		  $display("[%0t] FAIL: Port 0 mismatch! exp src=%h tgt=%h dat=%h got src=%h tgt=%h dat=%h",
				   $time, expected_source, expected_target, expected_data,
				   port0.source_out, port0.target_out, port0.data_out);
		  test_fail++; received = 1;
		end
	  end

	  // Check port 1
	  if (expected_port == 1 && port1.valid_out) begin
		if (port1.source_out == expected_source &&
			port1.target_out == expected_target &&
			port1.data_out   == expected_data) begin
		  $display("[%0t] PASS: Port 1 received expected packet (data=0x%h)", $time, port1.data_out);
		  test_pass++; received = 1; packets_received[1]++;
		end else begin
		  $display("[%0t] FAIL: Port 1 mismatch! exp src=%h tgt=%h dat=%h got src=%h tgt=%h dat=%h",
				   $time, expected_source, expected_target, expected_data,
				   port1.source_out, port1.target_out, port1.data_out);
		  test_fail++; received = 1;
		end
	  end

	  // Check port 2
	  if (expected_port == 2 && port2.valid_out) begin
		if (port2.source_out == expected_source &&
			port2.target_out == expected_target &&
			port2.data_out   == expected_data) begin
		  $display("[%0t] PASS: Port 2 received expected packet (data=0x%h)", $time, port2.data_out);
		  test_pass++; received = 1; packets_received[2]++;
		end else begin
		  $display("[%0t] FAIL: Port 2 mismatch! exp src=%h tgt=%h dat=%h got src=%h tgt=%h dat=%h",
				   $time, expected_source, expected_target, expected_data,
				   port2.source_out, port2.target_out, port2.data_out);
		  test_fail++; received = 1;
		end
	  end

	  // Check port 3
	  if (expected_port == 3 && port3.valid_out) begin
		if (port3.source_out == expected_source &&
			port3.target_out == expected_target &&
			port3.data_out   == expected_data) begin
		  $display("[%0t] PASS: Port 3 received expected packet (data=0x%h)", $time, port3.data_out);
		  test_pass++; received = 1; packets_received[3]++;
		end else begin
		  $display("[%0t] FAIL: Port 3 mismatch! exp src=%h tgt=%h dat=%h got src=%h tgt=%h dat=%h",
				   $time, expected_source, expected_target, expected_data,
				   port3.source_out, port3.target_out, port3.data_out);
		  test_fail++; received = 1;
		end
	  end
	end

	if (!received) begin
	  $display("[%0t] FAIL: Timeout waiting for packet on port %0d", $time, expected_port);
	  test_fail++;
	end
  endtask

  //===========================================================================
  // Task: collect_multicast_outputs
  // For multicast/broadcast, collect packets from multiple ports
  //===========================================================================

  task automatic collect_multicast(
	input logic [3:0] expected_target,
	input logic [3:0] expected_source,
	input logic [7:0] expected_data,
	input int timeout = 30
  );
	int cycles = 0;
	int received_count = 0;
	int expected_count;

	expected_count = 0;
	for (int i = 0; i < 4; i++) begin
	  if (expected_target[i]) expected_count++;
	end

	$display("[%0t] Expecting multicast to %0d ports (target=0x%h)",
			 $time, expected_count, expected_target);

	while (cycles < timeout && received_count < expected_count) begin
	  @(posedge clk);
	  cycles++;

	  if (expected_target[0] && port0.valid_out &&
		  port0.source_out == expected_source &&
		  port0.data_out   == expected_data) begin
		$display("[%0t]   -> Port 0 received multicast", $time);
		received_count++; packets_received[0]++;
	  end
	  if (expected_target[1] && port1.valid_out &&
		  port1.source_out == expected_source &&
		  port1.data_out   == expected_data) begin
		$display("[%0t]   -> Port 1 received multicast", $time);
		received_count++; packets_received[1]++;
	  end
	  if (expected_target[2] && port2.valid_out &&
		  port2.source_out == expected_source &&
		  port2.data_out   == expected_data) begin
		$display("[%0t]   -> Port 2 received multicast", $time);
		received_count++; packets_received[2]++;
	  end
	  if (expected_target[3] && port3.valid_out &&
		  port3.source_out == expected_source &&
		  port3.data_out   == expected_data) begin
		$display("[%0t]   -> Port 3 received multicast", $time);
		received_count++; packets_received[3]++;
	  end
	end

	if (received_count == expected_count) begin
	  $display("[%0t] PASS: Multicast delivered to all %0d destinations",
			   $time, expected_count);
	  test_pass++;
	end else begin
	  $display("[%0t] FAIL: Multicast delivered to %0d of %0d destinations",
			   $time, received_count, expected_count);
	  test_fail++;
	end
  endtask

  //===========================================================================
  // Main Test Sequence
  //===========================================================================

  initial begin
	for (int i = 0; i < 4; i++) packets_received[i] = 0;

	// Initialize interface signals
	port0.valid_in = 0; port0.source_in = 0; port0.target_in = 0; port0.data_in = 0;
	port1.valid_in = 0; port1.source_in = 0; port1.target_in = 0; port1.data_in = 0;
	port2.valid_in = 0; port2.source_in = 0; port2.target_in = 0; port2.data_in = 0;
	port3.valid_in = 0; port3.source_in = 0; port3.target_in = 0; port3.data_in = 0;

	$display("\n========================================================");
	$display("       TB_BASIC: Stage A RTL QA Test");
	$display("========================================================\n");

	@(posedge rst_n);
	repeat(4) @(posedge clk);

	$display("\n--- TEST 1: Single-Destination (Port 0 -> Port 3) ---");
	drive_packet(0, 4'b0001, 4'b1000, 8'hAB);
	wait_and_check(3, 4'b0001, 4'b1000, 8'hAB);
	repeat(5) @(posedge clk);

	$display("\n--- TEST 2: Single-Destination (Port 1 -> Port 0) ---");
	drive_packet(1, 4'b0010, 4'b0001, 8'hCD);
	wait_and_check(0, 4'b0010, 4'b0001, 8'hCD);
	repeat(5) @(posedge clk);

	$display("\n--- TEST 3: Single-Destination (Port 2 -> Port 1) ---");
	drive_packet(2, 4'b0100, 4'b0010, 8'hEF);
	wait_and_check(1, 4'b0100, 4'b0010, 8'hEF);
	repeat(5) @(posedge clk);

	$display("\n--- TEST 4: Multicast (Port 1 -> Ports 0 and 2) ---");
	drive_packet(1, 4'b0010, 4'b0101, 8'h42);
	collect_multicast(4'b0101, 4'b0010, 8'h42);
	repeat(5) @(posedge clk);

	$display("\n--- TEST 5: Multicast (Port 1 -> Ports 0, 2, 3) ---");
	drive_packet(1, 4'b0010, 4'b1101, 8'h77);
	collect_multicast(4'b1101, 4'b0010, 8'h77);
	repeat(5) @(posedge clk);

	$display("\n--- TEST 6: Broadcast (Port 2 -> All Ports) ---");
	drive_packet(2, 4'b0100, 4'b1111, 8'hFF);
	collect_multicast(4'b1111, 4'b0100, 8'hFF);
	repeat(5) @(posedge clk);

	$display("\n--- TEST 7: Contention (Ports 0,1 both -> Port 3) ---");
	fork
	  drive_packet(0, 4'b0001, 4'b1000, 8'h11);
	  drive_packet(1, 4'b0010, 4'b1000, 8'h22);
	join
	repeat(15) @(posedge clk);

	$display("\n========================================================");
	$display("                  TEST RESULTS");
	$display("========================================================");
	$display("  Packets sent:      %0d", packets_sent);
	$display("  Tests passed:      %0d", test_pass);
	$display("  Tests failed:      %0d", test_fail);
	$display("  Packets received:");
	$display("    Port 0: %0d", packets_received[0]);
	$display("    Port 1: %0d", packets_received[1]);
	$display("    Port 2: %0d", packets_received[2]);
	$display("    Port 3: %0d", packets_received[3]);
	$display("========================================================\n");

	#100;
	$finish;
  end

  //===========================================================================
  // Timeout Watchdog
  //===========================================================================

  initial begin
	#10000;
	$display("\n[%0t] ERROR: Simulation timeout!", $time);
	$finish;
  end

  //===========================================================================
  // Waveform Dump (optional - for debugging)
  //===========================================================================

  initial begin
	$dumpfile("tb_basic.vcd");
	$dumpvars(0, tb_basic);
  end

endmodule
