//-----------------------------------------------------------------------------
// Module: switch_4port
// Description: 4-port packet switch fabric with:
//   - switch_port FSMs per input
//   - per-input FIFOs (ingress buffering only)
//   - per-output round-robin arbiters
//   - direct outputs (no output FIFOs)
//   - top-level FSM: SW_IDLE, SW_RECEIVE, SW_ROUTE, SW_TRANSMIT
//-----------------------------------------------------------------------------

module switch_4port #(
  parameter int NUM_PORTS   = 4,
  parameter int DATA_WIDTH  = 8,
  parameter int FIFO_DEPTH  = 8   // Depth of per-input FIFOs
)(
  input  logic  clk,
  input  logic  rst_n,
  port_if       port0,
  port_if       port1,
  port_if       port2,
  port_if       port3
);

  //-------------------------------------------------------------------------
  // Local parameters
  //-------------------------------------------------------------------------

  // Broadcast target (all ports)
  localparam logic [3:0] BROADCAST_TARGET = 4'b1111;

  // Packet format: {source[3:0], target[3:0], data[7:0]} = 16 bits
  localparam int FIFO_DATA_WIDTH = 16;

  //-------------------------------------------------------------------------
  // Top-level switch fabric FSM
  //-------------------------------------------------------------------------

  typedef enum logic [1:0] {
	SW_IDLE     = 2'b00,
	SW_RECEIVE  = 2'b01,
	SW_ROUTE    = 2'b10,
	SW_TRANSMIT = 2'b11
  } sw_state_t;

  sw_state_t sw_state, sw_next_state;

  //-------------------------------------------------------------------------
  // Outputs of switch_port instances
  //-------------------------------------------------------------------------

  logic        v0, v1, v2, v3;
  logic [3:0]  s0, s1, s2, s3;
  logic [3:0]  t0, t1, t2, t3;
  logic [7:0]  d0, d1, d2, d3;

  //-------------------------------------------------------------------------
  // Input FIFOs (one per input port)
  //-------------------------------------------------------------------------

  // Write side
  logic                       in_fifo0_wr_en, in_fifo1_wr_en;
  logic                       in_fifo2_wr_en, in_fifo3_wr_en;
  logic [FIFO_DATA_WIDTH-1:0] in_fifo0_wr_data, in_fifo1_wr_data;
  logic [FIFO_DATA_WIDTH-1:0] in_fifo2_wr_data, in_fifo3_wr_data;

  // Read side
  logic                       in_fifo0_rd_en, in_fifo1_rd_en;
  logic                       in_fifo2_rd_en, in_fifo3_rd_en;
  logic [FIFO_DATA_WIDTH-1:0] in_fifo0_rd_data, in_fifo1_rd_data;
  logic [FIFO_DATA_WIDTH-1:0] in_fifo2_rd_data, in_fifo3_rd_data;

  // Status
  logic in_fifo0_full,  in_fifo1_full,  in_fifo2_full,  in_fifo3_full;
  logic in_fifo0_empty, in_fifo1_empty, in_fifo2_empty, in_fifo3_empty;

  //-------------------------------------------------------------------------
  // Latched packet state per input (one "current packet" per input)
  //-------------------------------------------------------------------------

  logic [3:0] pkt_src    [NUM_PORTS];
  logic [3:0] pkt_tgt    [NUM_PORTS];
  logic [7:0] pkt_data   [NUM_PORTS];
  logic       pkt_valid  [NUM_PORTS];

  // For each input i and output j, bit = 1 means "already sent to output j"
  logic [3:0] pkt_written     [NUM_PORTS];
  logic [3:0] pkt_written_next[NUM_PORTS];

  // Packet "done" flag per input
  logic pkt_done[NUM_PORTS];

  //-------------------------------------------------------------------------
  // Request vectors and grants (per output)
  //   reqX[i] = input i requests output X
  //-------------------------------------------------------------------------

  logic [NUM_PORTS-1:0] req0, req1, req2, req3;
  logic [1:0]           grant0, grant1, grant2, grant3;

  //-------------------------------------------------------------------------
  // Output mux signals (no output FIFOs)
  //-------------------------------------------------------------------------

  logic                       out0_valid, out1_valid, out2_valid, out3_valid;
  logic [FIFO_DATA_WIDTH-1:0] out0_data,  out1_data,  out2_data,  out3_data;

  //-------------------------------------------------------------------------
  // Helper: Pack/unpack FIFO data
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

  function automatic logic [FIFO_DATA_WIDTH-1:0] pack_packet(
	input logic [3:0] source,
	input logic [3:0] target,
	input logic [7:0] data
  );
	return {source, target, data};
  endfunction

  //-------------------------------------------------------------------------
  // Validation helpers (used at ingress)
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
	input logic       v_in,
	input logic [3:0] src,
	input logic [3:0] tgt
  );
	return v_in
		&& is_one_hot_local(src)
		&& (|tgt)
		&& ((tgt == BROADCAST_TARGET) || !(|(tgt & src)));
  endfunction

  //-------------------------------------------------------------------------
  // Instantiate switch_port modules (one per input port)
  //   (Use your updated switch_port implementation)
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
  // Input FIFO instances (one per input port)
  //-------------------------------------------------------------------------

  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) in_fifo0 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (in_fifo0_wr_en),
	.wr_data(in_fifo0_wr_data),
	.rd_en  (in_fifo0_rd_en),
	.rd_data(in_fifo0_rd_data),
	.full   (in_fifo0_full),
	.empty  (in_fifo0_empty)
  );

  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) in_fifo1 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (in_fifo1_wr_en),
	.wr_data(in_fifo1_wr_data),
	.rd_en  (in_fifo1_rd_en),
	.rd_data(in_fifo1_rd_data),
	.full   (in_fifo1_full),
	.empty  (in_fifo1_empty)
  );

  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) in_fifo2 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (in_fifo2_wr_en),
	.wr_data(in_fifo2_wr_data),
	.rd_en  (in_fifo2_rd_en),
	.rd_data(in_fifo2_rd_data),
	.full   (in_fifo2_full),
	.empty  (in_fifo2_empty)
  );

  sync_fifo #(
	.DATA_WIDTH(FIFO_DATA_WIDTH),
	.DEPTH     (FIFO_DEPTH)
  ) in_fifo3 (
	.clk    (clk),
	.rst_n  (rst_n),
	.wr_en  (in_fifo3_wr_en),
	.wr_data(in_fifo3_wr_data),
	.rd_en  (in_fifo3_rd_en),
	.rd_data(in_fifo3_rd_data),
	.full   (in_fifo3_full),
	.empty  (in_fifo3_empty)
  );

  //-------------------------------------------------------------------------
  // Ingress validation + write to input FIFOs
  //-------------------------------------------------------------------------

  logic v0_ok, v1_ok, v2_ok, v3_ok;

  assign v0_ok = packet_is_valid(v0, s0, t0);
  assign v1_ok = packet_is_valid(v1, s1, t1);
  assign v2_ok = packet_is_valid(v2, s2, t2);
  assign v3_ok = packet_is_valid(v3, s3, t3);

  assign in_fifo0_wr_en   = v0_ok && !in_fifo0_full;
  assign in_fifo0_wr_data = pack_packet(s0, t0, d0);

  assign in_fifo1_wr_en   = v1_ok && !in_fifo1_full;
  assign in_fifo1_wr_data = pack_packet(s1, t1, d1);

  assign in_fifo2_wr_en   = v2_ok && !in_fifo2_full;
  assign in_fifo2_wr_data = pack_packet(s2, t2, d2);

  assign in_fifo3_wr_en   = v3_ok && !in_fifo3_full;
  assign in_fifo3_wr_data = pack_packet(s3, t3, d3);

  //-------------------------------------------------------------------------
  // Packet done condition per input:
  // pkt_done[i] = pkt_valid && no remaining target bits that are not written
  //-------------------------------------------------------------------------

  always_comb begin
	for (int i = 0; i < NUM_PORTS; i++) begin
	  pkt_done[i] = pkt_valid[i] &&
					((pkt_tgt[i] & ~pkt_written[i]) == 4'b0000);
	end
  end

  //-------------------------------------------------------------------------
  // "Any work" flag for FSM
  //-------------------------------------------------------------------------

  logic any_work;

  always_comb begin
	any_work = !in_fifo0_empty || !in_fifo1_empty ||
			   !in_fifo2_empty || !in_fifo3_empty ||
			   pkt_valid[0] || pkt_valid[1] ||
			   pkt_valid[2] || pkt_valid[3];
  end

  //-------------------------------------------------------------------------
  // Switch fabric FSM: IDLE -> RECEIVE -> ROUTE -> TRANSMIT
  //-------------------------------------------------------------------------

  always_comb begin
	sw_next_state = sw_state;

	unique case (sw_state)
	  SW_IDLE: begin
		if (any_work)
		  sw_next_state = SW_RECEIVE;
	  end

	  SW_RECEIVE: begin
		sw_next_state = SW_ROUTE;
	  end

	  SW_ROUTE: begin
		sw_next_state = SW_TRANSMIT;
	  end

	  SW_TRANSMIT: begin
		if (any_work)
		  sw_next_state = SW_RECEIVE;
		else
		  sw_next_state = SW_IDLE;
	  end

	  default: sw_next_state = SW_IDLE;
	endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n)
	  sw_state <= SW_IDLE;
	else
	  sw_state <= sw_next_state;
  end

  //-------------------------------------------------------------------------
  // Compute next pkt_written (set bits for outputs served this cycle)
  // Only update written bits during SW_TRANSMIT.
  //-------------------------------------------------------------------------

  always_comb begin
	for (int i = 0; i < NUM_PORTS; i++) begin
	  pkt_written_next[i] = pkt_written[i];
	end

	if (sw_state == SW_TRANSMIT) begin
	  if (out0_valid) pkt_written_next[grant0][0] = 1'b1;
	  if (out1_valid) pkt_written_next[grant1][1] = 1'b1;
	  if (out2_valid) pkt_written_next[grant2][2] = 1'b1;
	  if (out3_valid) pkt_written_next[grant3][3] = 1'b1;
	end
  end

  //-------------------------------------------------------------------------
  // Input FIFO read + packet latching
  //-------------------------------------------------------------------------

  assign in_fifo0_rd_en = (sw_state == SW_RECEIVE) &&
						  (!pkt_valid[0] || pkt_done[0]) && !in_fifo0_empty;
  assign in_fifo1_rd_en = (sw_state == SW_RECEIVE) &&
						  (!pkt_valid[1] || pkt_done[1]) && !in_fifo1_empty;
  assign in_fifo2_rd_en = (sw_state == SW_RECEIVE) &&
						  (!pkt_valid[2] || pkt_done[2]) && !in_fifo2_empty;
  assign in_fifo3_rd_en = (sw_state == SW_RECEIVE) &&
						  (!pkt_valid[3] || pkt_done[3]) && !in_fifo3_empty;

  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  for (int i = 0; i < NUM_PORTS; i++) begin
		pkt_valid[i]   <= 1'b0;
		pkt_src[i]     <= '0;
		pkt_tgt[i]     <= '0;
		pkt_data[i]    <= '0;
		pkt_written[i] <= '0;
	  end
	end else begin
	  // Input 0
	  if (!pkt_valid[0] || pkt_done[0]) begin
		if (in_fifo0_rd_en) begin
		  pkt_valid[0]   <= 1'b1;
		  pkt_src[0]     <= get_source(in_fifo0_rd_data);
		  pkt_tgt[0]     <= get_target(in_fifo0_rd_data);
		  pkt_data[0]    <= get_data(in_fifo0_rd_data);
		  pkt_written[0] <= 4'b0000;
		end else if (pkt_done[0]) begin
		  pkt_valid[0]   <= 1'b0;
		  pkt_written[0] <= 4'b0000;
		end
	  end else begin
		pkt_written[0] <= pkt_written_next[0];
	  end

	  // Input 1
	  if (!pkt_valid[1] || pkt_done[1]) begin
		if (in_fifo1_rd_en) begin
		  pkt_valid[1]   <= 1'b1;
		  pkt_src[1]     <= get_source(in_fifo1_rd_data);
		  pkt_tgt[1]     <= get_target(in_fifo1_rd_data);
		  pkt_data[1]    <= get_data(in_fifo1_rd_data);
		  pkt_written[1] <= 4'b0000;
		end else if (pkt_done[1]) begin
		  pkt_valid[1]   <= 1'b0;
		  pkt_written[1] <= 4'b0000;
		end
	  end else begin
		pkt_written[1] <= pkt_written_next[1];
	  end

	  // Input 2
	  if (!pkt_valid[2] || pkt_done[2]) begin
		if (in_fifo2_rd_en) begin
		  pkt_valid[2]   <= 1'b1;
		  pkt_src[2]     <= get_source(in_fifo2_rd_data);
		  pkt_tgt[2]     <= get_target(in_fifo2_rd_data);
		  pkt_data[2]    <= get_data(in_fifo2_rd_data);
		  pkt_written[2] <= 4'b0000;
		end else if (pkt_done[2]) begin
		  pkt_valid[2]   <= 1'b0;
		  pkt_written[2] <= 4'b0000;
		end
	  end else begin
		pkt_written[2] <= pkt_written_next[2];
	  end

	  // Input 3
	  if (!pkt_valid[3] || pkt_done[3]) begin
		if (in_fifo3_rd_en) begin
		  pkt_valid[3]   <= 1'b1;
		  pkt_src[3]     <= get_source(in_fifo3_rd_data);
		  pkt_tgt[3]     <= get_target(in_fifo3_rd_data);
		  pkt_data[3]    <= get_data(in_fifo3_rd_data);
		  pkt_written[3] <= 4'b0000;
		end else if (pkt_done[3]) begin
		  pkt_valid[3]   <= 1'b0;
		  pkt_written[3] <= 4'b0000;
		end
	  end else begin
		pkt_written[3] <= pkt_written_next[3];
	  end
	end
  end

  //-------------------------------------------------------------------------
  // Request vector generation (from latched packets)
  //-------------------------------------------------------------------------

  always_comb begin
	// output 0
	req0[0] = pkt_valid[0] && pkt_tgt[0][0] && !pkt_written[0][0];
	req0[1] = pkt_valid[1] && pkt_tgt[1][0] && !pkt_written[1][0];
	req0[2] = pkt_valid[2] && pkt_tgt[2][0] && !pkt_written[2][0];
	req0[3] = pkt_valid[3] && pkt_tgt[3][0] && !pkt_written[3][0];

	// output 1
	req1[0] = pkt_valid[0] && pkt_tgt[0][1] && !pkt_written[0][1];
	req1[1] = pkt_valid[1] && pkt_tgt[1][1] && !pkt_written[1][1];
	req1[2] = pkt_valid[2] && pkt_tgt[2][1] && !pkt_written[2][1];
	req1[3] = pkt_valid[3] && pkt_tgt[3][1] && !pkt_written[3][1];

	// output 2
	req2[0] = pkt_valid[0] && pkt_tgt[0][2] && !pkt_written[0][2];
	req2[1] = pkt_valid[1] && pkt_tgt[1][2] && !pkt_written[1][2];
	req2[2] = pkt_valid[2] && pkt_tgt[2][2] && !pkt_written[2][2];
	req2[3] = pkt_valid[3] && pkt_tgt[3][2] && !pkt_written[3][2];

	// output 3
	req3[0] = pkt_valid[0] && pkt_tgt[0][3] && !pkt_written[0][3];
	req3[1] = pkt_valid[1] && pkt_tgt[1][3] && !pkt_written[1][3];
	req3[2] = pkt_valid[2] && pkt_tgt[2][3] && !pkt_written[2][3];
	req3[3] = pkt_valid[3] && pkt_tgt[3][3] && !pkt_written[3][3];
  end

  //-------------------------------------------------------------------------
  // Round-robin arbiters (per output)
  //-------------------------------------------------------------------------

  rr_arbiter #(.NUM_PORTS(NUM_PORTS)) arb0 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req0),
	.grant (grant0)
  );

  rr_arbiter #(.NUM_PORTS(NUM_PORTS)) arb1 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req1),
	.grant (grant1)
  );

  rr_arbiter #(.NUM_PORTS(NUM_PORTS)) arb2 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req2),
	.grant (grant2)
  );

  rr_arbiter #(.NUM_PORTS(NUM_PORTS)) arb3 (
	.clk   (clk),
	.rst_n (rst_n),
	.req   (req3),
	.grant (grant3)
  );

  //-------------------------------------------------------------------------
  // Output muxes (no output FIFOs)
  //-------------------------------------------------------------------------

  always_comb begin
	out0_valid = 1'b0;
	out0_data  = '0;
	unique case (grant0)
	  2'd0: if (req0[0]) begin
		out0_valid = 1'b1;
		out0_data  = pack_packet(pkt_src[0], pkt_tgt[0], pkt_data[0]);
	  end
	  2'd1: if (req0[1]) begin
		out0_valid = 1'b1;
		out0_data  = pack_packet(pkt_src[1], pkt_tgt[1], pkt_data[1]);
	  end
	  2'd2: if (req0[2]) begin
		out0_valid = 1'b1;
		out0_data  = pack_packet(pkt_src[2], pkt_tgt[2], pkt_data[2]);
	  end
	  2'd3: if (req0[3]) begin
		out0_valid = 1'b1;
		out0_data  = pack_packet(pkt_src[3], pkt_tgt[3], pkt_data[3]);
	  end
	endcase
  end

  always_comb begin
	out1_valid = 1'b0;
	out1_data  = '0;
	unique case (grant1)
	  2'd0: if (req1[0]) begin
		out1_valid = 1'b1;
		out1_data  = pack_packet(pkt_src[0], pkt_tgt[0], pkt_data[0]);
	  end
	  2'd1: if (req1[1]) begin
		out1_valid = 1'b1;
		out1_data  = pack_packet(pkt_src[1], pkt_tgt[1], pkt_data[1]);
	  end
	  2'd2: if (req1[2]) begin
		out1_valid = 1'b1;
		out1_data  = pack_packet(pkt_src[2], pkt_tgt[2], pkt_data[2]);
	  end
	  2'd3: if (req1[3]) begin
		out1_valid = 1'b1;
		out1_data  = pack_packet(pkt_src[3], pkt_tgt[3], pkt_data[3]);
	  end
	endcase
  end

  always_comb begin
	out2_valid = 1'b0;
	out2_data  = '0;
	unique case (grant2)
	  2'd0: if (req2[0]) begin
		out2_valid = 1'b1;
		out2_data  = pack_packet(pkt_src[0], pkt_tgt[0], pkt_data[0]);
	  end
	  2'd1: if (req2[1]) begin
		out2_valid = 1'b1;
		out2_data  = pack_packet(pkt_src[1], pkt_tgt[1], pkt_data[1]);
	  end
	  2'd2: if (req2[2]) begin
		out2_valid = 1'b1;
		out2_data  = pack_packet(pkt_src[2], pkt_tgt[2], pkt_data[2]);
	  end
	  2'd3: if (req2[3]) begin
		out2_valid = 1'b1;
		out2_data  = pack_packet(pkt_src[3], pkt_tgt[3], pkt_data[3]);
	  end
	endcase
  end

  always_comb begin
	out3_valid = 1'b0;
	out3_data  = '0;
	unique case (grant3)
	  2'd0: if (req3[0]) begin
		out3_valid = 1'b1;
		out3_data  = pack_packet(pkt_src[0], pkt_tgt[0], pkt_data[0]);
	  end
	  2'd1: if (req3[1]) begin
		out3_valid = 1'b1;
		out3_data  = pack_packet(pkt_src[1], pkt_tgt[1], pkt_data[1]);
	  end
	  2'd2: if (req3[2]) begin
		out3_valid = 1'b1;
		out3_data  = pack_packet(pkt_src[2], pkt_tgt[2], pkt_data[2]);
	  end
	  2'd3: if (req3[3]) begin
		out3_valid = 1'b1;
		out3_data  = pack_packet(pkt_src[3], pkt_tgt[3], pkt_data[3]);
	  end
	endcase
  end

  //-------------------------------------------------------------------------
  // Drive the actual port outputs from outX_valid/outX_data
  //-------------------------------------------------------------------------

  // Port 0
  always_comb begin
	port0.valid_out = (sw_state == SW_TRANSMIT) ? out0_valid : 1'b0;
	if ((sw_state == SW_TRANSMIT) && out0_valid) begin
	  port0.source_out = get_source(out0_data);
	  port0.target_out = get_target(out0_data);
	  port0.data_out   = get_data(out0_data);
	end else begin
	  port0.source_out = 4'b0000;
	  port0.target_out = 4'b0000;
	  port0.data_out   = '0;
	end
  end

  // Port 1
  always_comb begin
	port1.valid_out = (sw_state == SW_TRANSMIT) ? out1_valid : 1'b0;
	if ((sw_state == SW_TRANSMIT) && out1_valid) begin
	  port1.source_out = get_source(out1_data);
	  port1.target_out = get_target(out1_data);
	  port1.data_out   = get_data(out1_data);
	end else begin
	  port1.source_out = 4'b0000;
	  port1.target_out = 4'b0000;
	  port1.data_out   = '0;
	end
  end

  // Port 2
  always_comb begin
	port2.valid_out = (sw_state == SW_TRANSMIT) ? out2_valid : 1'b0;
	if ((sw_state == SW_TRANSMIT) && out2_valid) begin
	  port2.source_out = get_source(out2_data);
	  port2.target_out = get_target(out2_data);
	  port2.data_out   = get_data(out2_data);
	end else begin
	  port2.source_out = 4'b0000;
	  port2.target_out = 4'b0000;
	  port2.data_out   = '0;
	end
  end

  // Port 3
  always_comb begin
	port3.valid_out = (sw_state == SW_TRANSMIT) ? out3_valid : 1'b0;
	if ((sw_state == SW_TRANSMIT) && out3_valid) begin
	  port3.source_out = get_source(out3_data);
	  port3.target_out = get_target(out3_data);
	  port3.data_out   = get_data(out3_data);
	end else begin
	  port3.source_out = 4'b0000;
	  port3.target_out = 4'b0000;
	  port3.data_out   = '0;
	end
  end

endmodule

//-----------------------------------------------------------------------------
// Round-Robin Arbiter for NUM_PORTS requesters
//-----------------------------------------------------------------------------

module rr_arbiter #(
  parameter int NUM_PORTS = 4
)(
  input  logic                  clk,
  input  logic                  rst_n,
  input  logic [NUM_PORTS-1:0]  req,
  output logic [$clog2(NUM_PORTS)-1:0] grant
);

  localparam int GRANT_WIDTH = $clog2(NUM_PORTS);

  logic [GRANT_WIDTH-1:0] priority_ptr;
  logic [NUM_PORTS-1:0]   rotated_req;
  logic [GRANT_WIDTH-1:0] pre_grant;
  logic                   any_req;

  // Rotate left
  function automatic logic [NUM_PORTS-1:0] rotate_left(
	input logic [NUM_PORTS-1:0] value,
	input logic [GRANT_WIDTH-1:0] amount
  );
	logic [NUM_PORTS-1:0] result;
	result = (value << amount) | (value >> (NUM_PORTS - amount));
	return result;
  endfunction

  // First-set-bit encoder
  function automatic logic [GRANT_WIDTH-1:0] find_first_set(
	input logic [NUM_PORTS-1:0] value
  );
	logic [GRANT_WIDTH-1:0] result;
	result = '0;
	for (int i = 0; i < NUM_PORTS; i++) begin
	  if (value[i]) begin
		result = i[GRANT_WIDTH-1:0];
		break;
	  end
	end
	return result;
  endfunction

  assign any_req = |req;

  always_comb begin
	rotated_req = rotate_left(req, priority_ptr);
	pre_grant   = find_first_set(rotated_req);

	if (any_req) begin
	  grant = pre_grant + priority_ptr;
	  if (grant >= NUM_PORTS)
		grant = grant - NUM_PORTS;  // wrap
	end else begin
	  grant = '0;
	end
  end

  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n)
	  priority_ptr <= '0;
	else
	  priority_ptr <= (priority_ptr == NUM_PORTS-1) ? '0 : priority_ptr + 1'b1;
  end

endmodule

//-----------------------------------------------------------------------------
// Simple synchronous FIFO
//-----------------------------------------------------------------------------

module sync_fifo #(
  parameter int DATA_WIDTH = 16,
  parameter int DEPTH      = 8
)(
  input  logic                  clk,
  input  logic                  rst_n,
  input  logic                  wr_en,
  input  logic [DATA_WIDTH-1:0] wr_data,
  input  logic                  rd_en,
  output logic [DATA_WIDTH-1:0] rd_data,
  output logic                  full,
  output logic                  empty
);

  localparam int PTR_WIDTH = $clog2(DEPTH);

  logic [DATA_WIDTH-1:0] mem   [0:DEPTH-1];
  logic [PTR_WIDTH-1:0]  wr_ptr, rd_ptr;
  logic [PTR_WIDTH:0]    count;

  assign empty   = (count == 0);
  assign full    = (count == DEPTH);
  assign rd_data = mem[rd_ptr];

  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  wr_ptr <= '0;
	  rd_ptr <= '0;
	  count  <= '0;
	end else begin
	  unique case ({wr_en && !full, rd_en && !empty})
		2'b10: begin
		  mem[wr_ptr] <= wr_data;
		  wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
		  count       <= count + 1'b1;
		end
		2'b01: begin
		  rd_ptr <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
		  count  <= count - 1'b1;
		end
		2'b11: begin
		  mem[wr_ptr] <= wr_data;
		  wr_ptr      <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
		  rd_ptr      <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
		  // count unchanged
		end
		default: ; // no op
	  endcase
	end
  end

endmodule
