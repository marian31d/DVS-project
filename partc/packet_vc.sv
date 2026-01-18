import packet_pkg::*;

class packet_vc extends component_base;

  agent        agt;
  virtual port_if vif;
  int          port_id;
  packet_vc    other_vcs[4];

  function new(string n, component_base p=null);
	super.new(n,p);
  endfunction

  function void set_vif(virtual port_if v); vif = v; endfunction
  function void set_port_id(int id); port_id = id; endfunction
  function void set_other_vc(int idx, packet_vc vc); other_vcs[idx] = vc; endfunction

  virtual function void build();
	$display("Building Packet VC for port %0d", port_id);
	agt = new($sformatf("agt_%0d", port_id), this);
	agt.set_port_id(port_id);
	agt.set_vif(vif);
	agt.build();
  endfunction

  virtual function void connect();
	$display("Connecting Packet VC (port %0d)", port_id);
	agt.connect();
  endfunction

  virtual task run();
	  packet sent_pkts[$];
	  sequencer s;
	  $info("Packet VC starting (port %0d)", port_id);

	  // Start agent threads (agent.run must NOT block forever)
	  agt.run();

	  // ✅ Wait using events (NOT wait(function))
	  
	  s = agt.get_sequencer();

	  if (!s.has_started()) @(s.ev_started);
	  if (!s.has_done())    @(s.ev_done);


	  // small drain for registered DUT outputs
	  repeat (30) @(posedge vif.clk);

	  // Register expected packets into destination checkers
	  agt.get_sequencer().get_sent_packets(sent_pkts);
	  foreach (sent_pkts[i]) begin
		for (int j = 0; j < 4; j++) begin
		  if (sent_pkts[i].target[j] && other_vcs[j] != null) begin
			other_vcs[j].register_expected(sent_pkts[i]);
		  end
		end
	  end

	  $info("Packet VC done (port %0d)", port_id);
	endtask


  function void register_expected(packet pkt);
	agt.register_expected(pkt);
  endfunction

  function void finalize_check();
	agt.check_packets();
  endfunction

  virtual function void report();
	$display("=== Packet VC Port %0d Report ===", port_id);
	agt.report();
  endfunction

  function sequencer get_sequencer(); return agt.get_sequencer(); endfunction
  function bit is_passed(); return agt.is_passed(); endfunction

endclass