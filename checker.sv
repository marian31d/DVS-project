import packet_pkg::*;

class packet_checker extends component_base;

  int port_id;

  packet expected_packets[$];
  packet received_packets[$];

  int packets_matched     = 0;
  int packets_dropped     = 0;
  int packets_unexpected  = 0;
  int errors              = 0;

  bit test_passed = 1;

  function new(string n, component_base p=null);
	  super.new(n,p);
  endfunction

  function void set_port_id(int id); port_id = id; endfunction

  virtual function void build();
	$display("Building checker for port %0d", port_id);
  endfunction

  virtual function void connect();
	$display("Checker connected");
  endfunction

  // Expected packets are already filtered in packet_vc.run() by target bit,
  // so just push it here.
  function void add_expected(packet pkt);
	expected_packets.push_back(pkt.copy());
  endfunction

  function void add_received(packet pkt);
	received_packets.push_back(pkt.copy());
  endfunction

  // Match MUST include target as well, otherwise duplicates of {source,data}
  // will corrupt matching and cause dropped/unexpected.
  protected function bit find_and_remove_expected(packet rcv_pkt, output packet matched_pkt);
	foreach (expected_packets[i]) begin
	  if (expected_packets[i].source == rcv_pkt.source &&
		  expected_packets[i].target == rcv_pkt.target &&
		  expected_packets[i].data   == rcv_pkt.data) begin
		matched_pkt = expected_packets[i];
		expected_packets.delete(i);
		return 1;
	  end
	end
	matched_pkt = null;
	return 0;
  endfunction

  virtual task run();
	// no active behavior needed
  endtask

  function void check_all();
	packet matched_pkt;
	packet rcv_pkt;

	$display("Checking %0d received vs %0d expected",
			 received_packets.size(), expected_packets.size());

	while (received_packets.size() > 0) begin
	  rcv_pkt = received_packets.pop_front();

	  if (find_and_remove_expected(rcv_pkt, matched_pkt)) begin
		packets_matched++;
	  end else begin
		packets_unexpected++;
		test_passed = 0;   // unexpected is FAIL
	  end
	end

	packets_dropped = expected_packets.size();
	if (packets_dropped > 0) begin
	  $error("%0d packets dropped on port %0d!", packets_dropped, port_id);
	  test_passed = 0;
	end

	if (packets_unexpected > 0) begin
	  $error("%0d unexpected packets on port %0d!", packets_unexpected, port_id);
	  test_passed = 0;
	end
  endfunction

  virtual function void report();
	$display("Port %0d: Matched=%0d Dropped=%0d Unexpected=%0d %s",
			 port_id, packets_matched, packets_dropped, packets_unexpected,
			 test_passed ? "PASSED" : "FAILED");
  endfunction

  function bit is_passed(); return test_passed; endfunction

endclass
