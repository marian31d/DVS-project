import packet_pkg::*;

class driver extends component_base;

  virtual port_if vif;
  sequencer        seq;
  int              port_id;
  int              packets_driven;

  function new(string n, component_base p=null);
	super.new(n,p);
  endfunction

  function void set_vif(virtual port_if v);     vif = v; endfunction
  function void set_sequencer(sequencer s);     seq = s; endfunction
  function void set_port_id(int id);           port_id = id; endfunction

  virtual function void build();
	$display("Building driver for port %0d", port_id);
	packets_driven = 0;
  endfunction

  virtual function void connect();
	if (vif == null) $fatal(1, "Driver %0d: vif is NULL", port_id);
	if (seq == null) $fatal(1, "Driver %0d: seq is NULL", port_id);
	$display("Driver connected (port %0d)", port_id);
  endfunction

  task automatic drive_idle();
	vif.valid_in  <= 1'b0;
	vif.source_in <= 4'b0000;
	vif.target_in <= 4'b0000;
	vif.data_in   <= 8'h00;
  endtask

  // Drive inputs on NEGEDGE so they are stable at the next POSEDGE (where DUT samples).
  task automatic drive_packet(packet pkt);
	@(negedge vif.clk);
	vif.valid_in  <= 1'b1;
	vif.source_in <= pkt.source;
	vif.target_in <= pkt.target;
	vif.data_in   <= pkt.data;

	// Hold for one full cycle
	@(negedge vif.clk);
	drive_idle();

	packets_driven++;
  endtask

  virtual task run();
	packet pkt;

	$display("Driver starting (port %0d)", port_id);

	drive_idle();

	// Only wait for posedge if we are currently in reset
	if (!vif.rst_n) @(posedge vif.rst_n);

	// small gap after reset
	repeat (2) @(posedge vif.clk);

	forever begin
	  seq.get_next_item(pkt);
	  if (pkt == null) begin
		seq.item_done();
		continue;
	  end

	  drive_packet(pkt);
	  seq.item_done();

	  // optional small random bubble
	  repeat ($urandom_range(0,2)) @(posedge vif.clk);
	end
  endtask

  virtual function void report();
	$display("Packets driven: %0d", packets_driven);
  endfunction

endclass
