import packet_pkg::*;

class agent extends component_base;

  sequencer       seq;
  driver          drv;
  monitor         mon;
  packet_checker  chk;

  virtual port_if vif;
  int port_id;

  function new(string n, component_base p=null);
	super.new(n,p);
  endfunction

  function void set_vif(virtual port_if v); vif = v; endfunction
  function void set_port_id(int id); port_id = id; endfunction

  virtual function void build();
	$display("Building agent (port %0d)", port_id);

	seq = new($sformatf("seq_%0d", port_id), this);
	drv = new($sformatf("drv_%0d", port_id), this);
	mon = new($sformatf("mon_%0d", port_id), this);
	chk = new($sformatf("chk_%0d", port_id), this);

	seq.set_port_id(port_id);
	drv.set_port_id(port_id);
	mon.set_port_id(port_id);
	chk.set_port_id(port_id);

	seq.build();
	drv.build();
	mon.build();
	chk.build();
  endfunction

  virtual function void connect();
	$display("Connecting agent (port %0d)", port_id);

	drv.set_vif(vif);
	drv.set_sequencer(seq);
	mon.set_vif(vif);

	seq.connect();
	drv.connect();
	mon.connect();
	chk.connect();
  endfunction

  // ✅ IMPORTANT: start threads and RETURN immediately
  virtual task run();
	$display("Agent running (port %0d)", port_id);

	fork
	  seq.run();
	  drv.run();
	  mon.run();
	  monitor_and_check();   // feeds received pkts into checker
	join_none
  endtask

  protected task monitor_and_check();
	packet pkt;
	mailbox #(packet) mbx;
	mbx = mon.get_analysis_mbx();
	forever begin
	  mbx.get(pkt);
	  chk.add_received(pkt);
	end
  endtask

  function void register_expected(packet pkt); chk.add_expected(pkt); endfunction
  function void check_packets(); chk.check_all(); endfunction

  virtual function void report();
	$display("=== Agent Report (port %0d) ===", port_id);
	seq.report();
	drv.report();
	mon.report();
	chk.report();
  endfunction

  function sequencer get_sequencer(); return seq; endfunction
  function bit is_passed(); return chk.is_passed(); endfunction

endclass
