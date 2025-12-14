module switch_4port_tb_log;

localparam int NUM_PORTS = 4;

bit clk = 0;
always #5 clk = ~clk;

bit rst_n;

port_if port0(clk, rst_n);
port_if port1(clk, rst_n);
port_if port2(clk, rst_n);
port_if port3(clk, rst_n);

switch_4port dut (
  .clk (clk),
  .rst_n(rst_n),
  .port0(port0),
  .port1(port1),
  .port2(port2),
  .port3(port3)
);

typedef struct packed {
  logic [3:0] src;
  logic [3:0] tgt;
  logic [7:0] dat;
} pkt_t;

// Expected per output port
pkt_t exp_q[NUM_PORTS][$];
int pass_cnt, fail_cnt;

function automatic logic [3:0] onehot(int p);
  return (4'b0001 << p);
endfunction

function automatic bit pkt_equal(pkt_t a, pkt_t b);
  return (a.src==b.src) && (a.tgt==b.tgt) && (a.dat==b.dat);
endfunction

// SPEC assumption:
// - Broadcast (1111) -> all ports including source
// - Otherwise -> target without source bit
task automatic push_expected(pkt_t p);
  logic [3:0] mask;
  if (p.tgt == 4'b1111) mask = 4'b1111;
  else                 mask = (p.tgt & ~p.src);

  for (int o=0;o<NUM_PORTS;o++)
	if (mask[o]) exp_q[o].push_back(p);
endtask

function automatic bit pop_match(int o, pkt_t got);
  pop_match = 0;
  for (int k=0;k<exp_q[o].size();k++) begin
	if (pkt_equal(exp_q[o][k], got)) begin
	  exp_q[o].delete(k);
	  pop_match = 1;
	  return pop_match;
	end
  end
endfunction

// -------- IN driver (prints IN like screenshot) --------
task automatic send_pkt(int p, logic [3:0] tgt, logic [7:0] dat);
  pkt_t pk;
  pk.src = onehot(p);
  pk.tgt = tgt;
  pk.dat = dat;

  @(posedge clk);
  $display("[%0t] IN  p%0d : source=%b target=%b data=%b",
		   $time, p, pk.src, pk.tgt, pk.dat);

  // Record expected (only if legal-ish)
  if ((tgt == 4'b1111) || ((tgt != 4'b0000) && ((tgt & pk.src) == 4'b0000)))
	push_expected(pk);

  case (p)
	0: begin port0.valid_in <= 1; port0.source_in <= pk.src; port0.target_in <= pk.tgt; port0.data_in <= pk.dat; end
	1: begin port1.valid_in <= 1; port1.source_in <= pk.src; port1.target_in <= pk.tgt; port1.data_in <= pk.dat; end
	2: begin port2.valid_in <= 1; port2.source_in <= pk.src; port2.target_in <= pk.tgt; port2.data_in <= pk.dat; end
	3: begin port3.valid_in <= 1; port3.source_in <= pk.src; port3.target_in <= pk.tgt; port3.data_in <= pk.dat; end
  endcase

  @(posedge clk);
  port0.valid_in <= 0;
  port1.valid_in <= 0;
  port2.valid_in <= 0;
  port3.valid_in <= 0;
endtask

// -------- OUT monitors (prints OUT like screenshot + checks) --------
task automatic mon_out(int o);
  pkt_t got;
  bit ok;
  forever begin
	@(posedge clk);
	case (o)
	  0: if (!port0.valid_out) continue; else begin got.src=port0.source_out; got.tgt=port0.target_out; got.dat=port0.data_out; end
	  1: if (!port1.valid_out) continue; else begin got.src=port1.source_out; got.tgt=port1.target_out; got.dat=port1.data_out; end
	  2: if (!port2.valid_out) continue; else begin got.src=port2.source_out; got.tgt=port2.target_out; got.dat=port2.data_out; end
	  3: if (!port3.valid_out) continue; else begin got.src=port3.source_out; got.tgt=port3.target_out; got.dat=port3.data_out; end
	endcase

	// Print like screenshot
	$display("[%0t] OUT P%0d: source=%b target=%b data=%b",
			 $time, o, got.src, got.tgt, got.dat);

	ok = pop_match(o, got);
	if (ok) pass_cnt++;
	else    fail_cnt++;
  end
endtask

task automatic wait_empty(int max_cycles);
  int total, c=0;
  do begin
	total = 0;
	for (int o=0;o<NUM_PORTS;o++) total += exp_q[o].size();
	if (total==0) return;
	@(posedge clk); c++;
  end while (c < max_cycles);

  $display("[%0t] TIMEOUT: still %0d expected not observed!", $time, total);
  fail_cnt++;
endtask

initial begin
  pass_cnt=0; fail_cnt=0;

  // init
  port0.valid_in=0; port1.valid_in=0; port2.valid_in=0; port3.valid_in=0;
  port0.source_in=0; port1.source_in=0; port2.source_in=0; port3.source_in=0;
  port0.target_in=0; port1.target_in=0; port2.target_in=0; port3.target_in=0;
  port0.data_in  =0; port1.data_in  =0; port2.data_in  =0; port3.data_in  =0;

  // reset
  rst_n=0;
  repeat(5) @(posedge clk);
  rst_n=1;

  // start monitors
  fork
	mon_out(0);
	mon_out(1);
	mon_out(2);
	mon_out(3);
  join_none

  // ------------------- Stimulus (edit freely) -------------------
  send_pkt(0, 4'b0010, 8'b00010001); // p0 -> p1
  send_pkt(0, 4'b0110, 8'b00100010); // p0 -> p1,2
  send_pkt(0, 4'b0001, 8'b01000100); // p0 -> p1,2,3
  send_pkt(0, 4'b0000, 8'b10001000); // p0 -> p0,1,2,3
  
  //send_pkt(0, 4'b1111, 8'b00010001); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b00100010); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b01000100); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b10001000); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b00010001); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b00100010); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b01000100); // p0 -> p1,2,3,4
  //send_pkt(0, 4'b1111, 8'b10001000); // p0 -> p1,2,3,4
  
  //send_pkt(0, 4'b1110, 8'b00100010); // p0 -> p1,p2,p3
  //send_pkt(1, 4'b1001, 8'b00110011); // p1 -> p0,p3 (legal)
  //send_pkt(2, 4'b1111, 8'b01000100); // p2 -> broadcast
  // --------------------------------------------------------------

  wait_empty(2000);

  $display("End of test");
  $display("PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
  $finish;
end

endmodule
