import packet_pkg::*;

class monitor extends component_base;

  virtual port_if vif;
  int port_id;
  int packets_monitored;
  mailbox #(packet) analysis_mbx;

  function new(string n, component_base p=null);
	super.new(n,p);
  endfunction

  function void set_vif(virtual port_if v); vif = v; endfunction
  function void set_port_id(int id); port_id = id; endfunction
  function mailbox #(packet) get_analysis_mbx(); return analysis_mbx; endfunction

  virtual function void build();
	$display("Building monitor for port %0d", port_id);
	analysis_mbx = new();
	packets_monitored = 0;
  endfunction

  virtual function void connect();
	$display("Monitor connected (port %0d)", port_id);
	if (vif == null) $error("Monitor: vif is null");
  endfunction

  virtual task run();
	packet pkt;
	$display("Monitor starting (port %0d)", port_id);

	if (!vif.rst_n) @(posedge vif.rst_n);

	forever begin
	  @(vif.mon_cb);

	  if (vif.mon_cb.valid_out) begin
		// ✅ Drop unknown samples (prevents fake packets)
		if ($isunknown({vif.mon_cb.source_out, vif.mon_cb.target_out, vif.mon_cb.data_out})) begin
		  continue;
		end

		pkt = new();
		pkt.source = vif.mon_cb.source_out;
		pkt.target = vif.mon_cb.target_out;
		pkt.data   = vif.mon_cb.data_out;

		analysis_mbx.put(pkt);
		packets_monitored++;

		$display("[%0t] MON%0d valid_out src=%b tgt=%b data=%0h",
				 $time, port_id, pkt.source, pkt.target, pkt.data);
	  end
	end
  endtask

  virtual function void report();
	$display("Monitor observed %0d packets", packets_monitored);
  endfunction

endclass