//-----------------------------------------------------------------------------
// Module: switch_4port
// Description: 4-port packet switch fabric with round-robin arbitration
//              and per-output FIFOs to prevent packet drops under contention.
//
// Architecture:
//   Input Ports ? switch_port FSMs ? Arbiter ? FIFO (per output) ? Output Ports
//
// Key feature: When multiple inputs target the same output, the arbiter grants
// one per cycle. The granted packet is pushed into that output's FIFO.
// Packets are then popped from FIFOs one at a time to the output interface.
// This ensures no drops due to contention (unless FIFO overflows).
//-----------------------------------------------------------------------------

//notes-----------------------------------------------------------------
// 1. need to enable handlinf of invalid request target, drop the packed if it's
//    invalid, a packet is invalid if ti[i]=1 for any i, if &ti==0.
//    if yes drop the packet by vi=0


module switch_4port #(
  parameter int NUM_PORTS = 4,
  parameter int DATA_WIDTH = 8,
  parameter int FIFO_DEPTH = 8    // Depth of per-output FIFOs
)(
  input  logic clk,
  input  logic rst_n,
  port_if port0,
  port_if port1,
  port_if port2,
  port_if port3
);

  //-------------------------------------------------------------------------
  // Local Parameters
  //-------------------------------------------------------------------------
  
  // Source port encodings (one-hot)
  localparam logic [3:0] PORT0_SOURCE = 4'b0001;
  localparam logic [3:0] PORT1_SOURCE = 4'b0010;
  localparam logic [3:0] PORT2_SOURCE = 4'b0100;
  localparam logic [3:0] PORT3_SOURCE = 4'b1000;
  
  // Broadcast target
  localparam logic [3:0] BROADCAST_TARGET = 4'b1111;
  
  // FIFO data width: {source[3:0], target[3:0], data[7:0]} = 16 bits
  localparam int FIFO_DATA_WIDTH = 16;
  
  //-------------------------------------------------------------------------
  // Internal Wires from switch_port instances
  //-------------------------------------------------------------------------
  
  // Valid signals from each switch_port
  logic v0, v1, v2, v3;
  
  // Source signals from each switch_port
  logic [3:0] s0, s1, s2, s3;
  
  // Target signals from each switch_port
  logic [3:0] t0, t1, t2, t3;
  
  // Data signals from each switch_port
  logic [7:0] d0, d1, d2, d3;

  // Filtered/validated valid signals: consider a packet invalid and drop it
  // if its target vector has no bits set. "Drop" here means the packet
  // will not generate requests or be written to any FIFO.
  logic v0_valid, v1_valid, v2_valid, v3_valid;
  
  // Request vectors for each output port
  logic [3:0] req0, req1, req2, req3;
  
  // Grant signals from arbiters (index of winning input)
  logic [1:0] grant0, grant1, grant2, grant3;
  
  //-------------------------------------------------------------------------
  // FIFO Signals (one FIFO per output port)
  // Packet format in FIFO: {source[3:0], target[3:0], data[7:0]}
  //-------------------------------------------------------------------------
  
  // FIFO 0 (feeds port0 output)
  logic                       fifo0_wr_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo0_wr_data;
  logic                       fifo0_rd_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo0_rd_data;
  logic                       fifo0_full;
  logic                       fifo0_empty;
  
  // FIFO 1 (feeds port1 output)
  logic                       fifo1_wr_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo1_wr_data;
  logic                       fifo1_rd_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo1_rd_data;
  logic                       fifo1_full;
  logic                       fifo1_empty;
  
  // FIFO 2 (feeds port2 output)
  logic                       fifo2_wr_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo2_wr_data;
  logic                       fifo2_rd_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo2_rd_data;
  logic                       fifo2_full;
  logic                       fifo2_empty;
  
  // FIFO 3 (feeds port3 output)
  logic                       fifo3_wr_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo3_wr_data;
  logic                       fifo3_rd_en;
  logic [FIFO_DATA_WIDTH-1:0] fifo3_rd_data;
  logic                       fifo3_full;
  logic                       fifo3_empty;
  
  //-------------------------------------------------------------------------
  // Helper: Extract fields from FIFO data
  // Format: {source[15:12], target[11:8], data[7:0]}
  //-------------------------------------------------------------------------
  
  function automatic logic [3:0] get_source(input logic [FIFO_DATA_WIDTH-1:0] fifo_data);
	return fifo_data[15:12];
  endfunction
  
  function automatic logic [3:0] get_target(input logic [FIFO_DATA_WIDTH-1:0] fifo_data);
	return fifo_data[11:8];
  endfunction
  
  function automatic logic [7:0] get_data(input logic [FIFO_DATA_WIDTH-1:0] fifo_data);
	return fifo_data[7:0];
  endfunction
  
  // Helper: Pack fields into FIFO data format
  function automatic logic [FIFO_DATA_WIDTH-1:0] pack_packet(
	input logic [3:0] source,
	input logic [3:0] target,
	input logic [7:0] data
  );
	return {source, target, data};
  endfunction

  //-------------------------------------------------------------------------
  // Local validation helpers
  // - `is_one_hot_local` checks that `source` is a valid one-hot encoding
  // - `packet_is_valid` implements the rule: original `v` must be set,
  //    `source` must be one-hot, target must be non-zero, and the packet
  //    must not target its own source unless the target is a broadcast
  //    (all ones).
  //-------------------------------------------------------------------------
  function automatic logic is_one_hot_local(input logic [3:0] value);
	int count;
	count = 0;
	for (int i = 0; i < NUM_PORTS; i++) begin
	  if (value[i]) count++;
	end
	return (count == 1);
  endfunction

  function automatic logic packet_is_valid(
	input logic        v_in,
	input logic [3:0]  src,
	input logic [3:0]  tgt
  );
	// require original valid, a well-defined one-hot source, non-empty target,
	// and either the target is broadcast or it does not include the source bit
	return v_in && is_one_hot_local(src) && (|tgt) && ((tgt == BROADCAST_TARGET) || !(|(tgt & src)));
  endfunction

  // Function to check if source is one-hot encoded
  function automatic logic is_one_hot(input logic [3:0] value);
	int count;
	count = 0;
	for (int i = 0; i < NUM_PORTS; i++) begin
	  if (value[i]) count++;
	end
	return (count == 1);
  endfunction
  
  //-------------------------------------------------------------------------
  // Round-Robin Arbiter Module (defined inline)
  //-------------------------------------------------------------------------
  
  // Round-robin arbiter for fair arbitration among requesters
  // Uses a rotating priority pointer to ensure fairness
  
  //-------------------------------------------------------------------------
  // Instantiate 4 switch_port modules
  //-------------------------------------------------------------------------
  
  switch_port #(.NUM_PORTS(NUM_PORTS)) sp0 (
	.clk       (clk),
	.rst_n     (rst_n),
	.valid_in  (port0.valid_in),
	.source_in (port0.source_in),
	.target_in (port0.target_in),
	.data_in   (port0.data_in),
	.valid_out (v0),
	.source_out(s0),
	.target_out(t0),
	.data_out  (d0)
  );
  
  switch_port #(.NUM_PORTS(NUM_PORTS)) sp1 (
	.clk       (clk),
	.rst_n     (rst_n),
	.valid_in  (port1.valid_in),
	.source_in (port1.source_in),
	.target_in (port1.target_in),
	.data_in   (port1.data_in),
	.valid_out (v1),
	.source_out(s1),
	.target_out(t1),
	.data_out  (d1)
  );
  
  switch_port #(.NUM_PORTS(NUM_PORTS)) sp2 (
	.clk       (clk),
	.rst_n     (rst_n),
	.valid_in  (port2.valid_in),
	.source_in (port2.source_in),
	.target_in (port2.target_in),
	.data_in   (port2.data_in),
	.valid_out (v2),
	.source_out(s2),
	.target_out(t2),
	.data_out  (d2)
  );
  
  switch_port #(.NUM_PORTS(NUM_PORTS)) sp3 (
	.clk       (clk),
	.rst_n     (rst_n),
	.valid_in  (port3.valid_in),
	.source_in (port3.source_in),
	.target_in (port3.target_in),
	.data_in   (port3.data_in),
	.valid_out (v3),
	.source_out(s3),
	.target_out(t3),
	.data_out  (d3)
  );
  
  //-------------------------------------------------------------------------
  // Request Vector Generation
  // For each output port j, reqj[i] = input i wants to send to output j
  //-------------------------------------------------------------------------
  
  
  
  always_comb begin
	// Compute filtered valid signals using helper function
	v0_valid = packet_is_valid(v0, s0, t0);
	v1_valid = packet_is_valid(v1, s1, t1);
	v2_valid = packet_is_valid(v2, s2, t2);
	v3_valid = packet_is_valid(v3, s3, t3);

	// Request for output port 0
	req0[0] = v0_valid && t0[0];
	req0[1] = v1_valid && t1[0];
	req0[2] = v2_valid && t2[0];
	req0[3] = v3_valid && t3[0];
	
	// Request for output port 1 (use filtered valids)
	req1[0] = v0_valid && t0[1];
	req1[1] = v1_valid && t1[1];
	req1[2] = v2_valid && t2[1];
	req1[3] = v3_valid && t3[1];
	
	// Request for output port 2 (use filtered valids)
	req2[0] = v0_valid && t0[2];
	req2[1] = v1_valid && t1[2];
	req2[2] = v2_valid && t2[2];
	req2[3] = v3_valid && t3[2];
	
	// Request for output port 3 (use filtered valids)
	req3[0] = v0_valid && t0[3];
	req3[1] = v1_valid && t1[3];
	req3[2] = v2_valid && t2[3];
	req3[3] = v3_valid && t3[3];
  end
  
  //-------------------------------------------------------------------------
  // Round-Robin Arbiter Instances
  //-------------------------------------------------------------------------
  
  rr_arbiter arb0 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req0),
	.grant (grant0)
  );
  
  rr_arbiter arb1 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req1),
	.grant (grant1)
  );
  
  rr_arbiter arb2 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req2),
	.grant (grant2)
  );
  
  rr_arbiter arb3 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req3),
	.grant (grant3)
  );
  
  //-------------------------------------------------------------------------
  // Per-Output FIFO Instances
  // Each FIFO buffers packets destined for that output port.
  // This prevents drops when multiple inputs target the same output.
  //-------------------------------------------------------------------------
  
  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) fifo0 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (fifo0_wr_en),
	.wr_data(fifo0_wr_data),
	.rd_en  (fifo0_rd_en),
	.rd_data(fifo0_rd_data),
	.full   (fifo0_full),
	.empty  (fifo0_empty)
  );
  
  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) fifo1 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (fifo1_wr_en),
	.wr_data(fifo1_wr_data),
	.rd_en  (fifo1_rd_en),
	.rd_data(fifo1_rd_data),
	.full   (fifo1_full),
	.empty  (fifo1_empty)
  );
  
  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) fifo2 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (fifo2_wr_en),
	.wr_data(fifo2_wr_data),
	.rd_en  (fifo2_rd_en),
	.rd_data(fifo2_rd_data),
	.full   (fifo2_full),
	.empty  (fifo2_empty)
  );
  
  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) fifo3 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (fifo3_wr_en),
	.wr_data(fifo3_wr_data),
	.rd_en  (fifo3_rd_en),
	.rd_data(fifo3_rd_data),
	.full   (fifo3_full),
	.empty  (fifo3_empty)
  );
  
  //-------------------------------------------------------------------------
  // FIFO Write Logic (Arbiter ? FIFO)
  // For each output, the arbiter selects which input wins.
  // The winning packet is written to the FIFO (if not full).
  //-------------------------------------------------------------------------
  
  // FIFO 0 write logic
  always_comb begin
	fifo0_wr_en   = 1'b0;
	fifo0_wr_data = '0;
	
	if (!fifo0_full) begin
	  case (grant0)
		2'd0: if (v0_valid && t0[0]) begin
		  fifo0_wr_en   = 1'b1;
		  fifo0_wr_data = pack_packet(s0, t0, d0);
		end
		2'd1: if (v1_valid && t1[0]) begin
		  fifo0_wr_en   = 1'b1;
		  fifo0_wr_data = pack_packet(s1, t1, d1);
		end
		2'd2: if (v2_valid && t2[0]) begin
		  fifo0_wr_en   = 1'b1;
		  fifo0_wr_data = pack_packet(s2, t2, d2);
		end
		2'd3: if (v3_valid && t3[0]) begin
		  fifo0_wr_en   = 1'b1;
		  fifo0_wr_data = pack_packet(s3, t3, d3);
		end
	  endcase
	end
  end
  
  // FIFO 1 write logic
  always_comb begin
	fifo1_wr_en   = 1'b0;
	fifo1_wr_data = '0;
	
	if (!fifo1_full) begin
	  case (grant1)
		2'd0: if (v0_valid && t0[1]) begin
		  fifo1_wr_en   = 1'b1;
		  fifo1_wr_data = pack_packet(s0, t0, d0);
		end
		2'd1: if (v1_valid && t1[1]) begin
		  fifo1_wr_en   = 1'b1;
		  fifo1_wr_data = pack_packet(s1, t1, d1);
		end
		2'd2: if (v2_valid && t2[1]) begin
		  fifo1_wr_en   = 1'b1;
		  fifo1_wr_data = pack_packet(s2, t2, d2);
		end
		2'd3: if (v3_valid && t3[1]) begin
		  fifo1_wr_en   = 1'b1;
		  fifo1_wr_data = pack_packet(s3, t3, d3);
		end
	  endcase
	end
  end
  
  // FIFO 2 write logic
  always_comb begin
	fifo2_wr_en   = 1'b0;
	fifo2_wr_data = '0;
	
	if (!fifo2_full) begin
	  case (grant2)
		2'd0: if (v0_valid && t0[2]) begin
		  fifo2_wr_en   = 1'b1;
		  fifo2_wr_data = pack_packet(s0, t0, d0);
		end
		2'd1: if (v1_valid && t1[2]) begin
		  fifo2_wr_en   = 1'b1;
		  fifo2_wr_data = pack_packet(s1, t1, d1);
		end
		2'd2: if (v2_valid && t2[2]) begin
		  fifo2_wr_en   = 1'b1;
		  fifo2_wr_data = pack_packet(s2, t2, d2);
		end
		2'd3: if (v3_valid && t3[2]) begin
		  fifo2_wr_en   = 1'b1;
		  fifo2_wr_data = pack_packet(s3, t3, d3);
		end
	  endcase
	end
  end
  
  // FIFO 3 write logic
  always_comb begin
	fifo3_wr_en   = 1'b0;
	fifo3_wr_data = '0;
	
	if (!fifo3_full) begin
	  case (grant3)
		2'd0: if (v0_valid && t0[3]) begin
		  fifo3_wr_en   = 1'b1;
		  fifo3_wr_data = pack_packet(s0, t0, d0);
		end
		2'd1: if (v1_valid && t1[3]) begin
		  fifo3_wr_en   = 1'b1;
		  fifo3_wr_data = pack_packet(s1, t1, d1);
		end
		2'd2: if (v2_valid && t2[3]) begin
		  fifo3_wr_en   = 1'b1;
		  fifo3_wr_data = pack_packet(s2, t2, d2);
		end
		2'd3: if (v3_valid && t3[3]) begin
		  fifo3_wr_en   = 1'b1;
		  fifo3_wr_data = pack_packet(s3, t3, d3);
		end
	  endcase
	end
  end
  
  //-------------------------------------------------------------------------
  // FIFO Read Logic (FIFO ? Output Port)
  // When FIFO is not empty, pop one packet per cycle and drive output.
  // valid_out is asserted for exactly one cycle per packet.
  //-------------------------------------------------------------------------
  
  // Read enable: pop from FIFO whenever not empty
  assign fifo0_rd_en = !fifo0_empty;
  assign fifo1_rd_en = !fifo1_empty;
  assign fifo2_rd_en = !fifo2_empty;
  assign fifo3_rd_en = !fifo3_empty;
  
  // Port 0 output: drive from FIFO
  always_comb begin
	if (!fifo0_empty) begin
	  port0.valid_out  = 1'b1;
	  port0.source_out = get_source(fifo0_rd_data);
	  port0.target_out = get_target(fifo0_rd_data);
	  port0.data_out   = get_data(fifo0_rd_data);
	end else begin
	  port0.valid_out  = 1'b0;
	  port0.source_out = 4'b0000;
	  port0.target_out = 4'b0000;
	  port0.data_out   = 8'h00;
	end
  end
  
  // Port 1 output: drive from FIFO
  always_comb begin
	if (!fifo1_empty) begin
	  port1.valid_out  = 1'b1;
	  port1.source_out = get_source(fifo1_rd_data);
	  port1.target_out = get_target(fifo1_rd_data);
	  port1.data_out   = get_data(fifo1_rd_data);
	end else begin
	  port1.valid_out  = 1'b0;
	  port1.source_out = 4'b0000;
	  port1.target_out = 4'b0000;
	  port1.data_out   = 8'h00;
	end
  end
  
  // Port 2 output: drive from FIFO
  always_comb begin
	if (!fifo2_empty) begin
	  port2.valid_out  = 1'b1;
	  port2.source_out = get_source(fifo2_rd_data);
	  port2.target_out = get_target(fifo2_rd_data);
	  port2.data_out   = get_data(fifo2_rd_data);
	end else begin
	  port2.valid_out  = 1'b0;
	  port2.source_out = 4'b0000;
	  port2.target_out = 4'b0000;
	  port2.data_out   = 8'h00;
	end
  end
  
  // Port 3 output: drive from FIFO
  always_comb begin
	if (!fifo3_empty) begin
	  port3.valid_out  = 1'b1;
	  port3.source_out = get_source(fifo3_rd_data);
	  port3.target_out = get_target(fifo3_rd_data);
	  port3.data_out   = get_data(fifo3_rd_data);
	end else begin
	  port3.valid_out  = 1'b0;
	  port3.source_out = 4'b0000;
	  port3.target_out = 4'b0000;
	  port3.data_out   = 8'h00;
	end
  end

endmodule

//-----------------------------------------------------------------------------
// Module: rr_arbiter
// Description: Round-Robin Arbiter for fair arbitration among 4 requesters
//-----------------------------------------------------------------------------

module rr_arbiter #(
  parameter int NUM_PORTS = 4
)(
  input  logic       clk,
  input  logic       rst_n,
  input  logic [3:0] req,
  output logic [1:0] grant
);

  //-------------------------------------------------------------------------
  // Internal Signals
  //-------------------------------------------------------------------------
  
  // Priority pointer - indicates starting priority for round-robin
  logic [1:0] priority_ptr;
  
  // Rotated request based on priority
  logic [3:0] rotated_req;
  
  // Grant before rotation
  logic [1:0] pre_grant;
  
  // Any request active
  logic any_req;
  
  //-------------------------------------------------------------------------
  // Functions
  //-------------------------------------------------------------------------
  
  // Function to rotate request vector based on priority pointer
  function automatic logic [3:0] rotate_left(input logic [3:0] value, input logic [1:0] amount);
	logic [3:0] result;
	case (amount)
	  2'd0: result = value;
	  2'd1: result = {value[2:0], value[3]};
	  2'd2: result = {value[1:0], value[3:2]};
	  2'd3: result = {value[0], value[3:1]};
	endcase
	return result;
  endfunction
  
  // Function to find first set bit (priority encoder)
  function automatic logic [1:0] find_first_set(input logic [3:0] value);
	logic [1:0] result;
	result = 2'd0;
	for (int i = 0; i < NUM_PORTS; i++) begin
	  if (value[i]) begin
		result = i[1:0];
		break;
	  end
	end
	return result;
  endfunction
  
  //-------------------------------------------------------------------------
  // Combinational Logic - Grant Calculation
  //-------------------------------------------------------------------------
  
  assign any_req = |req;
  
  always_comb begin
	// Rotate requests based on priority pointer
	rotated_req = rotate_left(req, priority_ptr);
	
	// Find first active request in rotated order
	pre_grant = find_first_set(rotated_req);
	
	// Convert back to original index (only grant if request exists)
	if (any_req) begin
	  // Use explicit wraparound instead of modulo to avoid synthesis issues
	  case (pre_grant + priority_ptr)
		4'd0: grant = 2'd0;
		4'd1: grant = 2'd1;
		4'd2: grant = 2'd2;
		4'd3: grant = 2'd3;
		4'd4: grant = 2'd0;  // wrap
		4'd5: grant = 2'd1;  // wrap
		4'd6: grant = 2'd2;  // wrap
		default: grant = 2'd0;
	  endcase
	end else begin
	  grant = 2'd0;  // Default to port 0 when no requests
	end
  end
  
  //-------------------------------------------------------------------------
  // Sequential Logic - Update Priority Pointer
  //-------------------------------------------------------------------------
  
  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  priority_ptr <= 2'd0;
	end else begin
	  // Always advance pointer on every cycle for consistent rotation
	  // This ensures fairness even when requests come and go
	  priority_ptr <= (priority_ptr == 2'd3) ? 2'd0 : priority_ptr + 1'b1;
	end
  end

endmodule

//-----------------------------------------------------------------------------
// Module: sync_fifo
// Description: Synchronous FIFO for buffering packets at each output port.
//              Uses a circular buffer implementation with read/write pointers.
//
// Purpose: Prevents packet drops when multiple inputs target the same output.
//          The arbiter grants one input per cycle, and the granted packet is
//          pushed into the FIFO. Packets are then popped one at a time.
//
// Parameters:
//   DATA_WIDTH - Width of data stored (default 16 for {src, tgt, data})
//   DEPTH      - Number of entries in FIFO (default 8)
//
// Interface:
//   wr_en/wr_data - Write interface (push)
//   rd_en/rd_data - Read interface (pop)
//   full/empty    - Status flags
//-----------------------------------------------------------------------------

module sync_fifo #(
  parameter int DATA_WIDTH = 16,
  parameter int DEPTH = 8
)(
  input  logic                    clk,
  input  logic                    rst_n,
  
  // Write interface
  input  logic                    wr_en,
  input  logic [DATA_WIDTH-1:0]   wr_data,
  
  // Read interface
  input  logic                    rd_en,
  output logic [DATA_WIDTH-1:0]   rd_data,
  
  // Status
  output logic                    full,
  output logic                    empty
);

  //-------------------------------------------------------------------------
  // Local Parameters
  //-------------------------------------------------------------------------
  
  localparam int PTR_WIDTH = $clog2(DEPTH);
  
  //-------------------------------------------------------------------------
  // Internal Storage
  //-------------------------------------------------------------------------
  
  // Memory array
  logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
  
  // Pointers
  logic [PTR_WIDTH-1:0] wr_ptr;
  logic [PTR_WIDTH-1:0] rd_ptr;
  
  // Count of entries in FIFO
  logic [PTR_WIDTH:0] count;  // One extra bit to hold DEPTH value
  
  //-------------------------------------------------------------------------
  // Status Flags
  //-------------------------------------------------------------------------
  
  assign empty = (count == 0);
  assign full  = (count == DEPTH);
  
  //-------------------------------------------------------------------------
  // Read Data Output
  // Provide data at current read pointer (combinational read)
  //-------------------------------------------------------------------------
  
  assign rd_data = mem[rd_ptr];
  
  //-------------------------------------------------------------------------
  // Sequential Logic - Pointer and Count Management
  //-------------------------------------------------------------------------
  
  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  // Reset: clear pointers and count
	  wr_ptr <= '0;
	  rd_ptr <= '0;
	  count  <= '0;
	end else begin
	  // Handle simultaneous read and write
	  case ({wr_en && !full, rd_en && !empty})
		2'b10: begin
		  // Write only
		  mem[wr_ptr] <= wr_data;
		  wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
		  count       <= count + 1'b1;
		end
		
		2'b01: begin
		  // Read only
		  rd_ptr <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
		  count  <= count - 1'b1;
		end
		
		2'b11: begin
		  // Simultaneous read and write - count stays same
		  mem[wr_ptr] <= wr_data;
		  wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
		  rd_ptr      <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
		  // count remains unchanged
		end
		
		default: begin
		  // No operation
		end
	  endcase
	end
  end

endmodule
