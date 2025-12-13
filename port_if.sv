interface port_if (input bit clk, input bit rst_n);
  logic        valid_in;
  logic [3:0]  source_in;
  logic [3:0]  target_in;
  logic [7:0]  data_in;
  logic        valid_out;
  logic [3:0]  source_out;
  logic [3:0]  target_out;
  logic [7:0]  data_out;
  task drive_packet(input packet p);
    @(posedge clk);
    valid_in   <= 1;
    source_in  <= p.source;
    target_in  <= p.target;
    data_in    <= p.data;
    @(posedge clk);
    valid_in   <= 0;
  endtask
  task collect_packet(output packet p);
    @(posedge clk iff valid_out);
    p = new("monitored");
    p.source = source_out;
    p.target = target_out;
    p.data   = data_out;
  endtask
endinterface
