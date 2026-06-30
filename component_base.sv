virtual class component_base;
	string name;
	component_base parent;

	function new(string n, component_base p=null);
	  name   = n;
	  parent = p;
	endfunction

	pure virtual function void build();
	pure virtual function void connect();
	pure virtual task     run();
	pure virtual function void report();
  endclass	
