//-----------------------------------------------------------------------------
// File: packet_data.sv
// Description: Packet class definitions for switch verification
//-----------------------------------------------------------------------------

typedef enum {
  SINGLE_DEST,
  MULTICAST,
  BROADCAST
} packet_type_e;


//-----------------------------------------------------------------------------
// Base Packet Class
//-----------------------------------------------------------------------------

class packet;
  rand logic [3:0] source;
  rand logic [3:0] target;
  rand logic [7:0] data;

  string name;
  packet_type_e pkt_type;
  int port_idx;

  static int pkt_id_counter = 0;
  int pkt_id;

  function new(string name="packet", int port_idx=0);
	this.name     = name;
	this.port_idx = port_idx;
	this.pkt_id   = pkt_id_counter++;
  endfunction

  function int count_bits(logic [3:0] value);
	int cnt = 0;
	for (int i=0;i<4;i++) if (value[i]) cnt++;
	return cnt;
  endfunction

  function packet_type_e get_packet_type();
	int c = count_bits(target);
	if (target == 4'b1111) return BROADCAST;
	else if (c >= 2)       return MULTICAST;
	else                   return SINGLE_DEST;
  endfunction

  virtual function void print();
	pkt_type = get_packet_type();
	$display("[%s] ID=%0d Type=%0d Src=%b Tgt=%b Data=%0h",
			 name, pkt_id, pkt_type, source, target, data);
  endfunction

  virtual function packet copy();
	packet p = new(name, port_idx);
	p.source = source;
	p.target = target;
	p.data   = data;
	p.pkt_type = pkt_type;
	p.pkt_id   = pkt_id;
	return p;
  endfunction

  virtual function logic compare(packet other);
	return (source==other.source &&
			target==other.target &&
			data==other.data);
  endfunction

  //-------------------------------------------------------------------------
  // Constraints
  //-------------------------------------------------------------------------

  // Source must be one-hot
  constraint c_source_onehot {
	source inside {4'b0001,4'b0010,4'b0100,4'b1000};
  }

  // Target must not be zero
  constraint c_target_valid {
	target != 4'b0000;
  }

  // ? FIX for VCS:
  // Don't call get_packet_type() from constraints.
  //
  // Allow self-routing ONLY when SINGLE_DEST (one-hot target),
  // and allow broadcast (1111).
  //
  // For multicast (2 or 3 bits set), forbid (source & target).
  constraint c_no_self_target {
	if ((target != 4'b1111) && ($countones(target) != 1))
	  (source & target) == 4'b0000;
  }

endclass


//-----------------------------------------------------------------------------
// Single Destination Packet
//-----------------------------------------------------------------------------

class single_packet extends packet;

  function new(string name="single_packet", int port_idx=0);
	super.new(name, port_idx);
	pkt_type = SINGLE_DEST;
  endfunction

  constraint c_single_target {
	target inside {4'b0001,4'b0010,4'b0100,4'b1000};
  }

  constraint c_source_port {
	source == (4'b0001 << port_idx);
  }

endclass


//-----------------------------------------------------------------------------
// Multicast Packet
//-----------------------------------------------------------------------------

class multicast_packet extends packet;

  function new(string name="multicast_packet", int port_idx=0);
	super.new(name, port_idx);
	pkt_type = MULTICAST;
  endfunction

  constraint c_multicast_target {
	$countones(target) inside {2,3};
  }

  constraint c_source_port {
	source == (4'b0001 << port_idx);
  }

endclass


//-----------------------------------------------------------------------------
// Broadcast Packet
//-----------------------------------------------------------------------------

class broadcast_packet extends packet;

  function new(string name="broadcast_packet", int port_idx=0);
	super.new(name, port_idx);
	pkt_type = BROADCAST;
  endfunction

  constraint c_broadcast_target {
	target == 4'b1111;
  }

  constraint c_source_port {
	source == (4'b0001 << port_idx);
  }

  // Override no-self rule (broadcast allows self)
  constraint c_no_self_target { 1; }

endclass
