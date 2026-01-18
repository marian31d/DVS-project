`timescale 1ns/1ps
//============================================================
// switch_port.sv
// 1-cycle pipeline + FSM activity tracker (does NOT throttle)
//============================================================
module switch_port (
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

  // ----------------------------
  // Pipeline registers (same behavior you had)
  // ----------------------------
  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  valid_out  <= 1'b0;
	  source_out <= 4'b0;
	  target_out <= 4'b0;
	  data_out   <= 8'h00;
	end else begin
	  valid_out  <= valid_in;
	  source_out <= source_in;
	  target_out <= target_in;
	  data_out   <= data_in;
	end
  end

  // ----------------------------
  // FSM (for FSM coverage)
  // Tracks whether this port is "ACTIVE" (seeing traffic) or "IDLE"
  // Does not affect datapath => no packet drops.
  // ----------------------------
  typedef enum logic [0:0] { ST_IDLE=1'b0, ST_ACTIVE=1'b1 } port_state_e;
  port_state_e state_q, state_d;

  always_comb begin
	state_d = state_q;
	unique case (state_q)
	  ST_IDLE:   if (valid_in)  state_d = ST_ACTIVE;
	  ST_ACTIVE: if (!valid_in) state_d = ST_IDLE;    // go idle when no traffic
	  default:   state_d = ST_IDLE;
	endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) state_q <= ST_IDLE;
	else        state_q <= state_d;
  end

endmodule
