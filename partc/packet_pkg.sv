package packet_pkg;

	typedef enum logic [1:0] {
	  SINGLE_DEST = 2'd0,
	  MULTICAST   = 2'd1,
	  BROADCAST   = 2'd2
	} packet_type_e;

	class packet;
	  rand logic [3:0] source;   // one-hot
	  rand logic [3:0] target;   // mask
	  rand logic [7:0] data;

	  string        name;
	  int           port_idx;
	  packet_type_e pkt_type;

	  function new(string name="packet", int port_idx=0);
		this.name     = name;
		this.port_idx = port_idx;
	  endfunction

	  virtual function packet copy();
		packet p = new(this.name, this.port_idx);
		p.source   = this.source;
		p.target   = this.target;
		p.data     = this.data;
		p.pkt_type = this.pkt_type;
		return p;
	  endfunction

	  virtual function bit compare(packet other);
		return (source==other.source && target==other.target && data==other.data);
	  endfunction

	  virtual function void print();
		$display("[%s] Type=%0d Source=%b Target=%b Data=%0h",
				 name, pkt_type, source, target, data);
	  endfunction

	  // Base constraints
	  constraint c_source_onehot { source inside {4'b0001,4'b0010,4'b0100,4'b1000}; }
	  constraint c_target_valid  { target != 4'b0000; }

	  // ? FIX:
	  // Allow self-routing ONLY for SINGLE_DEST packets.
	  // Multicast forbids self-routing.
	  // Broadcast allowed (1111).
	  constraint c_no_self_target {
		if (pkt_type != SINGLE_DEST && target != 4'b1111)
		  (source & target) == 4'b0000;
	  }
	endclass


	class single_packet extends packet;
	  function new(string name="single_packet", int port_idx=0);
		super.new(name, port_idx);
		pkt_type = SINGLE_DEST;
	  endfunction

	  constraint c_source_port   { source == (4'b0001 << port_idx); }
	  constraint c_single_target { target inside {4'b0001,4'b0010,4'b0100,4'b1000}; }
	endclass


	class multicast_packet extends packet;
	  function new(string name="multicast_packet", int port_idx=0);
		super.new(name, port_idx);
		pkt_type = MULTICAST;
	  endfunction

	  constraint c_source_port      { source == (4'b0001 << port_idx); }
	  constraint c_multicast_target { $countones(target) inside {2,3}; }
	endclass


	class broadcast_packet extends packet;
	  function new(string name="broadcast_packet", int port_idx=0);
		super.new(name, port_idx);
		pkt_type = BROADCAST;
	  endfunction

	  constraint c_source_port      { source == (4'b0001 << port_idx); }
	  constraint c_broadcast_target { target == 4'b1111; }
	endclass

  endpackage
