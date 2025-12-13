//-----------------------------------------------------------------------------
// Module: switch_port
// Description: Per-port FSM for packet processing in 4-port switch
// This module implements a 4-state FSM: IDLE -> RECEIVE -> ROUTE -> TRANSMIT
//-----------------------------------------------------------------------------

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

  //-------------------------------------------------------------------------
  // Local Parameters and Type Definitions
  //-------------------------------------------------------------------------
  
  // Source port encodings (one-hot)
  localparam logic [3:0] PORT0_SOURCE = 4'b0001;
  localparam logic [3:0] PORT1_SOURCE = 4'b0010;
  localparam logic [3:0] PORT2_SOURCE = 4'b0100;
  localparam logic [3:0] PORT3_SOURCE = 4'b1000;
  
  // FSM State enumeration
  typedef enum logic [1:0] {
	IDLE     = 2'b00,
	RECEIVE  = 2'b01,
	ROUTE    = 2'b10,
	TRANSMIT = 2'b11
  } state_t;
  
  //-------------------------------------------------------------------------
  // Internal Signals
  //-------------------------------------------------------------------------
  
  // FSM state registers
  state_t state, next_state;
  
  // Packet registers to hold stable data between RECEIVE and TRANSMIT
  logic [3:0] src_reg;
  logic [3:0] tgt_reg;
  logic [7:0] dat_reg;
  
  // Output registers
  logic        valid_out_reg;
  logic [3:0]  source_out_reg;
  logic [3:0]  target_out_reg;
  logic [7:0]  data_out_reg;
  
  //-------------------------------------------------------------------------
  // Functions
  //-------------------------------------------------------------------------
  

  
  // Function to extract port index from one-hot encoding
  function automatic logic [1:0] get_port_index(input logic [3:0] one_hot);
	logic [1:0] idx;
	idx = 2'b00;
	for (int i = 0; i < NUM_PORTS; i++) begin
	  if (one_hot[i]) idx = i[1:0];
	end
	return idx;
  endfunction
  
  // Function to count number of bits set in target
  function automatic int count_target_bits(input logic [3:0] target);
	int cnt;
	cnt = 0;
	for (int i = 0; i < NUM_PORTS; i++) begin
	  if (target[i]) cnt++;
	end
	return cnt;
  endfunction
  
  //-------------------------------------------------------------------------
  // Sequential Logic - State and Register Updates
  //-------------------------------------------------------------------------
  
  always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
	  // Reset state and registers
	  state          <= IDLE;
	  src_reg        <= 4'b0000;
	  tgt_reg        <= 4'b0000;
	  dat_reg        <= 8'h00;
	  valid_out_reg  <= 1'b0;
	  source_out_reg <= 4'b0000;
	  target_out_reg <= 4'b0000;
	  data_out_reg   <= 8'h00;
	end else begin
	  // State transition
	  state <= next_state;
	  
	  // Register updates based on current state
	  case (state)
		IDLE: begin
		  // Clear output valid when idle
		  valid_out_reg <= 1'b0;
		end
		
		RECEIVE: begin
		  // Latch input packet into registers
		  src_reg <= source_in;
		  tgt_reg <= target_in;
		  dat_reg <= data_in;
		  valid_out_reg <= 1'b0;
		end
		
		ROUTE: begin
		  // Pipeline stage - prepare for transmission
		  // Registers already hold the packet data
		  valid_out_reg <= 1'b0;
		end
		
		TRANSMIT: begin
		  // Drive outputs from registers
		  valid_out_reg  <= 1'b1;
		  source_out_reg <= src_reg;
		  target_out_reg <= tgt_reg;
		  data_out_reg   <= dat_reg;
		end
		
		default: begin
		  valid_out_reg <= 1'b0;
		end
	  endcase
	end
  end
  
  //-------------------------------------------------------------------------
  // Combinational Logic - Next State Determination
  //-------------------------------------------------------------------------
  
  always_comb begin
	// Default: stay in current state
	next_state = state;
	
	case (state)
	  IDLE: begin
		// Wait for valid input
		if (valid_in) begin
		  next_state = RECEIVE;
		end
	  end
	  
	  RECEIVE: begin
		// Move to route stage after latching
		next_state = ROUTE;
	  end
	  
	  ROUTE: begin
		// Move to transmit stage after routing preparation
		next_state = TRANSMIT;
	  end
	  
	  TRANSMIT: begin
		// Return to idle after transmission
		next_state = IDLE;
	  end
	  
	  default: begin
		next_state = IDLE;
	  end
	endcase
  end
  
  //-------------------------------------------------------------------------
  // Output Assignments
  //-------------------------------------------------------------------------
  
  assign valid_out  = valid_out_reg;
  assign source_out = source_out_reg;
  assign target_out = target_out_reg;
  assign data_out   = data_out_reg;

endmodule
