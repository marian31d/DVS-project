module switch_port #(
	parameter int NUM_PORTS = 4
  )(
	input  logic        clk,
	input  logic        rst_n,

	input  logic        valid_in,
	input  logic [3:0]  source_in,
	input  logic [3:0]  target_in,
	input  logic [7:0]  data_in,

	output logic        valid_out,
	output logic [3:0]  source_out,
	output logic [3:0]  target_out,
	output logic [7:0]  data_out
  );

	typedef enum logic { ST_IDLE, ST_SEND } port_state_t;
	port_state_t state, next_state;

	logic [3:0] src_q, tgt_q;
	logic [7:0] data_q;

	// State register
	always_ff @(posedge clk or negedge rst_n) begin
	  if (!rst_n)
		state <= ST_IDLE;
	  else
		state <= next_state;
	end

	// Next-state logic
	always_comb begin
	  next_state = state;
	  case (state)
		ST_IDLE: if (valid_in) next_state = ST_SEND;
		ST_SEND: next_state = ST_IDLE;
	  endcase
	end

	// Latch packet
	always_ff @(posedge clk or negedge rst_n) begin
	  if (!rst_n) begin
		src_q  <= '0;
		tgt_q  <= '0;
		data_q <= '0;
	  end else begin
		if (state == ST_IDLE && valid_in) begin
		  src_q  <= source_in;
		  tgt_q  <= target_in;
		  data_q <= data_in;
		end
	  end
	end

	// Outputs
	always_comb begin
	  valid_out  = (state == ST_SEND);
	  source_out = src_q;
	  target_out = tgt_q;
	  data_out   = data_q;
	end

  endmodule
