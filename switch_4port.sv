`timescale 1ns/1ps

module switch_4port (
  input  logic clk,
  input  logic rst_n,
  port_if port0,
  port_if port1,
  port_if port2,
  port_if port3
);

  localparam int DEPTH   = 64;
  localparam int ADDR_W  = $clog2(DEPTH);
  localparam int COUNT_W = $clog2(DEPTH+1);

  typedef struct packed {
	logic [3:0] source;
	logic [3:0] target;
	logic [7:0] data;
  } pkt_t;

  pkt_t fifo   [4][DEPTH];
  logic [ADDR_W-1:0]  head  [4];
  logic [ADDR_W-1:0]  tail  [4];
  logic [COUNT_W-1:0] count [4];

  logic in_valid [4];
  pkt_t  in_pkt  [4];

  always_comb begin
	in_valid[0]      = port0.valid_in;
	in_pkt[0].source = port0.source_in;
	in_pkt[0].target = port0.target_in;
	in_pkt[0].data   = port0.data_in;

	in_valid[1]      = port1.valid_in;
	in_pkt[1].source = port1.source_in;
	in_pkt[1].target = port1.target_in;
	in_pkt[1].data   = port1.data_in;

	in_valid[2]      = port2.valid_in;
	in_pkt[2].source = port2.source_in;
	in_pkt[2].target = port2.target_in;
	in_pkt[2].data   = port2.data_in;

	in_valid[3]      = port3.valid_in;
	in_pkt[3].source = port3.source_in;
	in_pkt[3].target = port3.target_in;
	in_pkt[3].data   = port3.data_in;
  end

  typedef enum logic [1:0] {
	ST_IDLE   = 2'd0,
	ST_ACTIVE = 2'd1,
	ST_DRAIN  = 2'd2,
	ST_ERR    = 2'd3
  } sw_state_e;

  sw_state_e sw_state_q, sw_state_d;

  logic has_in, has_out;

  always_comb begin
	has_in  = in_valid[0] | in_valid[1] | in_valid[2] | in_valid[3];
	has_out = (count[0] != 0) | (count[1] != 0) | (count[2] != 0) | (count[3] != 0);

	sw_state_d = sw_state_q;
	unique case (sw_state_q)
	  ST_IDLE: begin
		if (has_in) sw_state_d = ST_ACTIVE;
	  end
	  ST_ACTIVE: begin
		if (!has_in && has_out)       sw_state_d = ST_DRAIN;
		else if (!has_in && !has_out) sw_state_d = ST_IDLE;
		else                          sw_state_d = ST_ACTIVE;
	  end
	  ST_DRAIN: begin
		if (has_in)        sw_state_d = ST_ACTIVE;
		else if (!has_out) sw_state_d = ST_IDLE;
		else               sw_state_d = ST_DRAIN;
	  end
	  default: sw_state_d = ST_ERR;
	endcase
  end

  logic pop_valid [4];
  pkt_t pop_pkt   [4];

  always_comb begin
	for (int o = 0; o < 4; o++) begin
	  pop_valid[o] = (count[o] != 0);
	  pop_pkt[o]   = (count[o] != 0) ? fifo[o][head[o]] : '{default:'0};
	end
  end

  logic v_pipe [4];
  logic [3:0] s_pipe [4];
  logic [3:0] t_pipe [4];
  logic [7:0] d_pipe [4];

  switch_port u_out0 (
	.clk(clk), .rst_n(rst_n),
	.valid_in(pop_valid[0]),
	.source_in(pop_pkt[0].source),
	.target_in(pop_pkt[0].target),
	.data_in(pop_pkt[0].data),
	.valid_out(v_pipe[0]),
	.source_out(s_pipe[0]),
	.target_out(t_pipe[0]),
	.data_out(d_pipe[0])
  );

  switch_port u_out1 (
	.clk(clk), .rst_n(rst_n),
	.valid_in(pop_valid[1]),
	.source_in(pop_pkt[1].source),
	.target_in(pop_pkt[1].target),
	.data_in(pop_pkt[1].data),
	.valid_out(v_pipe[1]),
	.source_out(s_pipe[1]),
	.target_out(t_pipe[1]),
	.data_out(d_pipe[1])
  );

  switch_port u_out2 (
	.clk(clk), .rst_n(rst_n),
	.valid_in(pop_valid[2]),
	.source_in(pop_pkt[2].source),
	.target_in(pop_pkt[2].target),
	.data_in(pop_pkt[2].data),
	.valid_out(v_pipe[2]),
	.source_out(s_pipe[2]),
	.target_out(t_pipe[2]),
	.data_out(d_pipe[2])
  );

  switch_port u_out3 (
	.clk(clk), .rst_n(rst_n),
	.valid_in(pop_valid[3]),
	.source_in(pop_pkt[3].source),
	.target_in(pop_pkt[3].target),
	.data_in(pop_pkt[3].data),
	.valid_out(v_pipe[3]),
	.source_out(s_pipe[3]),
	.target_out(t_pipe[3]),
	.data_out(d_pipe[3])
  );

  assign port0.valid_out  = v_pipe[0];
  assign port0.source_out = s_pipe[0];
  assign port0.target_out = t_pipe[0];
  assign port0.data_out   = d_pipe[0];

  assign port1.valid_out  = v_pipe[1];
  assign port1.source_out = s_pipe[1];
  assign port1.target_out = t_pipe[1];
  assign port1.data_out   = d_pipe[1];

  assign port2.valid_out  = v_pipe[2];
  assign port2.source_out = s_pipe[2];
  assign port2.target_out = t_pipe[2];
  assign port2.data_out   = d_pipe[2];

  assign port3.valid_out  = v_pipe[3];
  assign port3.source_out = s_pipe[3];
  assign port3.target_out = t_pipe[3];
  assign port3.data_out   = d_pipe[3];

  always_ff @(posedge clk or negedge rst_n) begin
	logic [ADDR_W-1:0]  head_t [4];
	logic [ADDR_W-1:0]  tail_t [4];
	logic [COUNT_W-1:0] cnt_t  [4];

	if (!rst_n) begin
	  for (int o = 0; o < 4; o++) begin
		head[o]  <= '0;
		tail[o]  <= '0;
		count[o] <= '0;
	  end
	  sw_state_q <= ST_IDLE;

	end else begin
	  sw_state_q <= sw_state_d;

	  for (int o = 0; o < 4; o++) begin
		head_t[o] = head[o];
		tail_t[o] = tail[o];
		cnt_t[o]  = count[o];
	  end

	  // POP one per output
	  for (int o = 0; o < 4; o++) begin
		if (cnt_t[o] != 0) begin
		  head_t[o] = head_t[o] + 1'b1;
		  cnt_t[o]  = cnt_t[o] - 1'b1;
		end
	  end

	  // ENQUEUE all inputs into target outputs
	  for (int i = 0; i < 4; i++) begin
		if (in_valid[i]) begin
		  for (int o = 0; o < 4; o++) begin
			if (in_pkt[i].target[o]) begin
			  if (cnt_t[o] < DEPTH) begin
				// *** minimal synth fix: nonblocking write into sequential memory ***
				fifo[o][tail_t[o]] <= in_pkt[i];
				tail_t[o] = tail_t[o] + 1'b1;
				cnt_t[o]  = cnt_t[o] + 1'b1;
			  end else begin
`ifndef SYNTHESIS
				$error("[%0t] FIFO overflow on output %0d (packet dropped)!", $time, o);
`endif
			  end
			end
		  end
		end
	  end

	  for (int o = 0; o < 4; o++) begin
		head[o]  <= head_t[o];
		tail[o]  <= tail_t[o];
		count[o] <= cnt_t[o];
	  end
	end
  end

endmodule
