import packet_pkg::*;

class sequencer extends component_base;

  // --- events for safe synchronization ---
  event ev_started;
  event ev_done;

  // --- NEW: sticky flags so we don't miss events ---
  bit started;
  bit done;

  mailbox #(packet) req_mbx;
  mailbox #(bit)    rsp_mbx;

  int port_id;
  int packets_generated;
  int num_packets;
  bit running;

  packet sent_packets[$];

  function new(string n, component_base p=null);
	super.new(n,p);
  endfunction

  function void set_port_id(int id); port_id = id; endfunction
  function void set_num_packets(int n); num_packets = n; endfunction
  function bit  is_running(); return running; endfunction
  function void get_sent_packets(output packet pkts[$]); pkts = sent_packets; endfunction

  // --- NEW: accessors used by packet_vc ---
  function bit has_started(); return started; endfunction
  function bit has_done();    return done;    endfunction

  virtual function void build();
	$display("Building sequencer for port %0d", port_id);
	req_mbx = new();
	rsp_mbx = new();
	packets_generated = 0;
	running = 0;

	// --- NEW: init flags ---
	started = 0;
	done    = 0;
  endfunction

  virtual function void connect();
	$display("Sequencer connected");
  endfunction

  task get_next_item(output packet pkt);
	req_mbx.get(pkt);
  endtask

  task item_done();
	rsp_mbx.put(1);
  endtask

  task send_packet(packet pkt);
	bit dummy_rsp;
	req_mbx.put(pkt);
	rsp_mbx.get(dummy_rsp);
	packets_generated++;
  endtask

  virtual task run();
	packet pkt;
	single_packet    single_pkt;
	multicast_packet multi_pkt;
	broadcast_packet bcast_pkt;
	int pkt_type_sel;

	$display("Sequencer starting (port %0d), will generate %0d packets", port_id, num_packets);

	running  = 1;

	// --- NEW: set flag BEFORE triggering event ---
	started  = 1;
	-> ev_started;

	repeat (num_packets) begin
	  pkt_type_sel = $urandom_range(0, 9);

	  if (pkt_type_sel < 6) begin
		single_pkt = new($sformatf("single_pkt_%0d_%0d", port_id, packets_generated), port_id);
		if (!single_pkt.randomize()) begin $error("Single packet randomization failed!"); continue; end
		pkt = single_pkt;
	  end
	  else if (pkt_type_sel < 9) begin
		multi_pkt = new($sformatf("multi_pkt_%0d_%0d", port_id, packets_generated), port_id);
		if (!multi_pkt.randomize()) begin $error("Multicast packet randomization failed!"); continue; end
		pkt = multi_pkt;
	  end
	  else begin
		bcast_pkt = new($sformatf("bcast_pkt_%0d_%0d", port_id, packets_generated), port_id);
		if (!bcast_pkt.randomize()) begin $error("Broadcast packet randomization failed!"); continue; end
		pkt = bcast_pkt;
	  end

	  sent_packets.push_back(pkt.copy());
	  send_packet(pkt);
	end

	running = 0;

	// --- NEW: set flag BEFORE triggering event ---
	done    = 1;
	-> ev_done;

	$display("Sequencer done (port %0d). Generated=%0d", port_id, packets_generated);
  endtask

  virtual function void report();
	$display("Sequencer generated %0d packets", packets_generated);
  endfunction

endclass
