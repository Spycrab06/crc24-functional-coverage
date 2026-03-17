module tb;

  // ============================================================
  //  Clock & DUT signals
  // ============================================================
  logic clk = 0;
  always #5 clk = ~clk;

  logic        rst_n, clear, data_valid, crc_valid;
  logic [7:0]  data_in, crc_out;

  // ============================================================
  //  Module-level variable for covergroup sampling.
  //  Must be module-scope so the covergroup's @(posedge clk)
  //  sample event can observe it.
  // ============================================================
  int pkt_len_d = 1;

  // ============================================================
  //  DUT instantiation
  // ============================================================
  crc8 dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .clear     (clear),
    .data_in   (data_in),
    .data_valid(data_valid),
    .crc_out   (crc_out),
    .crc_valid (crc_valid)
  );

  // ============================================================
  //  FUNCTIONAL COVERAGE  (sampled every posedge clk)
  //
  //  Six coverpoints track individual DUT signal states.
  //  Two cross coverpoints track joint combinations:
  //    cx_data_pktlen  — every data range in every packet size
  //    cx_clear_valid  — clear issued while CRC output is valid
  // ============================================================
  covergroup cov_crc8 @(posedge clk);
    option.per_instance = 1;

    // CP1 - Packet length (1 / 2-3 / 4-6 / 7-8 bytes)
    cp_pkt_len : coverpoint pkt_len_d {
      bins single_byte = {1};
      bins short_pkt   = {[2:3]};
      bins mid_pkt     = {[4:6]};
      bins long_pkt    = {[7:8]};
    }

    // CP2 - Input byte value (boundaries + mid-ranges)
    cp_data_in : coverpoint data_in {
      bins all_zeros = {8'h00};
      bins all_ones  = {8'hFF};
      bins low_vals  = {[8'h01 : 8'h7F]};
      bins high_vals = {[8'h80 : 8'hFE]};
    }

    // CP3 - CRC output value
    cp_crc_out : coverpoint crc_out {
      bins crc_zero   = {8'h00};
      bins crc_ones   = {8'hFF};
      bins crc_others = default;
    }

    // CP4 - Reset state
    cp_rst_n : coverpoint rst_n {
      bins in_reset = {1'b0};
      bins running  = {1'b1};
    }

    // CP5 - Clear signal
    cp_clear : coverpoint clear {
      bins clearing = {1'b1};
      bins no_clear = {1'b0};
    }

    // CP6 - CRC valid output
    cp_crc_valid : coverpoint crc_valid {
      bins not_valid = {1'b0};
      bins asserted  = {1'b1};
    }

    // CROSS 1: data byte value × packet length (4×4 = 16 bins)
    cx_data_pktlen : cross cp_data_in, cp_pkt_len;

    // CROSS 2: clear while crc_valid high (corner case) (2×2 = 4 bins)
    cx_clear_valid : cross cp_clear, cp_crc_valid;

  endgroup

  cov_crc8 cg = new();

  // ============================================================
  //  Software golden model  (CRC-8, poly 0x07)
  // ============================================================
  function automatic logic [7:0] sw_crc(logic [7:0] bytes[], int n);
    logic [7:0] crc = 0;
    for (int i = 0; i < n; i++)
      for (int b = 7; b >= 0; b--)
        crc = (crc[7] ^ bytes[i][b]) ? (crc << 1) ^ 8'h07 : (crc << 1);
    return crc;
  endfunction

  // ============================================================
  //  Task: send packet + verify CRC
  // ============================================================
  task automatic send_packet(logic [7:0] pkt[], int len);
    logic [7:0] expected;
    pkt_len_d = len;
    foreach (pkt[i]) begin
      data_in    <= pkt[i];
      data_valid <= 1;
      @(posedge clk);
    end
    data_valid <= 0;
    @(posedge clk);   // extra cycle: crc_valid goes high here
    expected = sw_crc(pkt, len);
    if (crc_out === expected)
      $display("  PASS  len=%0d  data[0]=0x%02h  crc=0x%02h",
               len, pkt[0], crc_out);
    else
      $display("  FAIL  len=%0d  got=0x%02h  expected=0x%02h",
               len, crc_out, expected);
  endtask

  // ============================================================
  //  Task: send packet then assert clear on the exact cycle
  //  crc_valid goes high, guaranteeing cx_clear_valid is hit.
  //
  //  DUT timing: valid_delay <= data_valid  (1-cycle registered)
  //  So crc_valid = valid_delay is high on the posedge AFTER
  //  the last data_valid=1 cycle.  De-assert data_valid and
  //  assert clear simultaneously; the NEXT posedge captures
  //  both crc_valid=1 and clear=1 in the covergroup.
  // ============================================================
  task automatic send_with_clear_overlap(logic [7:0] pkt[], int len);
    logic [7:0] expected;
    pkt_len_d = len;
    foreach (pkt[i]) begin
      data_in    <= pkt[i];
      data_valid <= 1;
      @(posedge clk);
    end
    // De-assert data_valid AND assert clear on same edge:
    // next posedge -> crc_valid=1, clear=1 -> hits cx_clear_valid
    data_valid <= 0;
    clear      <= 1;
    @(posedge clk);
    clear <= 0;
    @(posedge clk);
    expected = sw_crc(pkt, len);
    if (crc_out === expected)
      $display("  PASS  len=%0d  crc=0x%02h  [clear-overlap]", len, crc_out);
    else
      $display("  FAIL  len=%0d  got=0x%02h  exp=0x%02h  [clear-overlap]",
               len, crc_out, expected);
  endtask

  // Task: one-cycle clear pulse
  task do_clear();
    clear <= 1; @(posedge clk);
    clear <= 0; @(posedge clk);
  endtask

  // ============================================================
  //  COVERAGE REPORT TASK
  //  Uses coverpoint-level get_coverage() — supported on all
  //  SV simulators including VCS.
  // ============================================================
  task print_coverage_report();
    real cp1, cp2, cp3, cp4, cp5, cp6, cx1, cx2, overall;
    cp1     = cg.cp_pkt_len.get_coverage();
    cp2     = cg.cp_data_in.get_coverage();
    cp3     = cg.cp_crc_out.get_coverage();
    cp4     = cg.cp_rst_n.get_coverage();
    cp5     = cg.cp_clear.get_coverage();
    cp6     = cg.cp_crc_valid.get_coverage();
    cx1     = cg.cx_data_pktlen.get_coverage();
    cx2     = cg.cx_clear_valid.get_coverage();
    overall = cg.get_coverage();

    $display("");
    $display("+==========================================================+");
    $display("|          FUNCTIONAL COVERAGE REPORT                      |");
    $display("|          Design : CRC-8   Covergroup : cov_crc8          |");
    $display("+==========================================================+");
    $display("| Coverpoint             | Bins                  |   Cov%%  |");
    $display("+========================+=======================+=========+");
    $display("| cp_pkt_len             | 1 / 2-3 / 4-6 / 7-8  | %6.1f%% |", cp1);
    $display("| cp_data_in             | 0x00/0xFF/lo/hi       | %6.1f%% |", cp2);
    $display("| cp_crc_out             | 0x00 / 0xFF / other   | %6.1f%% |", cp3);
    $display("| cp_rst_n               | in_reset / running    | %6.1f%% |", cp4);
    $display("| cp_clear               | clearing / no_clear   | %6.1f%% |", cp5);
    $display("| cp_crc_valid           | not_valid / asserted  | %6.1f%% |", cp6);
    $display("+========================+=======================+=========+");
    $display("| cx_data_pktlen (cross) | data_in x pkt_len     | %6.1f%% |", cx1);
    $display("| cx_clear_valid (cross) | clear x crc_valid     | %6.1f%% |", cx2);
    $display("+========================+=======================+=========+");
    $display("|                                                           |");
    if (overall >= 100.0)
      $display("|  >>> OVERALL COVERAGE : %6.2f%%  -- ALL BINS HIT <<<  |",
               overall);
    else
      $display("|  >>> OVERALL COVERAGE : %6.2f%%  -- INCOMPLETE <<<    |",
               overall);
    $display("|                                                           |");
    $display("+==========================================================+");
    $display("");
  endtask

  // ============================================================
  //  MAIN TEST SEQUENCE
  // ============================================================
  initial begin
    logic [7:0] pkt[];
    logic [7:0] tmp_crc;

    $display("");
    $display("=== CRC-8 Functional Coverage Testbench ===");
    $display("");

    // -------------------------------------------------------
    //  Phase 1: Reset  -> hits cp_rst_n::in_reset
    // -------------------------------------------------------
    rst_n = 0; clear = 0; data_valid = 0; data_in = 0;
    repeat(4) @(posedge clk);
    rst_n = 1; @(posedge clk);

    // -------------------------------------------------------
    //  Phase 2: Directed tests for cx_data_pktlen cross coverage
    //
    //  cx_data_pktlen has 4×4=16 bins:
    //    {all_zeros, all_ones, low_vals, high_vals}
    //    × {single_byte, short_pkt, mid_pkt, long_pkt}
    //
    //  We explicitly drive every combination here.
    //  For multi-byte packets we fill ALL bytes with the target
    //  value so the covergroup sees it on every sample.
    // -------------------------------------------------------
    $display("[Phase 2] Directed cross-coverage tests");

    // --- all_zeros × all packet lengths ---
    // single (1 byte) - also triggers cx_clear_valid
    pkt = new[1]; foreach (pkt[i]) pkt[i] = 8'h00;
    send_with_clear_overlap(pkt, 1);   // hits cx_clear_valid corner case

    // short (2 bytes)
    pkt = new[2]; foreach (pkt[i]) pkt[i] = 8'h00;
    send_packet(pkt, 2); do_clear();

    // mid (4 bytes)
    pkt = new[4]; foreach (pkt[i]) pkt[i] = 8'h00;
    send_packet(pkt, 4); do_clear();

    // long (7 bytes)
    pkt = new[7]; foreach (pkt[i]) pkt[i] = 8'h00;
    send_packet(pkt, 7); do_clear();

    // --- all_ones × all packet lengths ---
    // single
    pkt = new[1]; foreach (pkt[i]) pkt[i] = 8'hFF;
    send_packet(pkt, 1); do_clear();

    // short (3 bytes)
    pkt = new[3]; foreach (pkt[i]) pkt[i] = 8'hFF;
    send_packet(pkt, 3); do_clear();

    // mid (5 bytes)
    pkt = new[5]; foreach (pkt[i]) pkt[i] = 8'hFF;
    send_packet(pkt, 5); do_clear();

    // long (8 bytes)
    pkt = new[8]; foreach (pkt[i]) pkt[i] = 8'hFF;
    send_packet(pkt, 8); do_clear();

    // --- low_vals × all packet lengths (0x01-0x7F) ---
    // single
    pkt = new[1]; foreach (pkt[i]) pkt[i] = 8'h42;
    send_packet(pkt, 1); do_clear();

    // short
    pkt = new[2]; foreach (pkt[i]) pkt[i] = 8'h5A;
    send_packet(pkt, 2); do_clear();

    // mid
    pkt = new[6]; foreach (pkt[i]) pkt[i] = 8'h33;
    send_packet(pkt, 6); do_clear();

    // long
    pkt = new[8]; foreach (pkt[i]) pkt[i] = 8'h7E;
    send_packet(pkt, 8); do_clear();

    // --- high_vals × all packet lengths (0x80-0xFE) ---
    // single
    pkt = new[1]; foreach (pkt[i]) pkt[i] = 8'hA5;
    send_packet(pkt, 1); do_clear();

    // short
    pkt = new[3]; foreach (pkt[i]) pkt[i] = 8'hC0;
    send_packet(pkt, 3); do_clear();

    // mid
    pkt = new[4]; foreach (pkt[i]) pkt[i] = 8'hD8;
    send_packet(pkt, 4); do_clear();

    // long
    pkt = new[7]; foreach (pkt[i]) pkt[i] = 8'hFE;
    send_packet(pkt, 7); do_clear();

    // --- CRC = 0x00 (residue property) ---
    pkt = new[1]; pkt[0] = 8'h31;
    tmp_crc = sw_crc(pkt, 1);
    pkt = new[2]; pkt[0] = 8'h31; pkt[1] = tmp_crc;
    send_packet(pkt, 2); do_clear();

    // --- CRC = 0xFF (exhaustive single-byte search) ---
    begin
      logic [7:0] trial[1];
      bit found = 0;
      for (int v = 0; v < 256; v++) begin
        trial[0] = v[7:0];
        if (sw_crc(trial, 1) == 8'hFF) begin
          pkt = new[1]; pkt[0] = v[7:0];
          $display("  [info] byte 0x%02h gives CRC=0xFF", v[7:0]);
          found = 1;
          break;
        end
      end
      if (found) begin
        send_packet(pkt, 1); do_clear();
      end
    end

    // -------------------------------------------------------
    //  Phase 3: Random tests — extra cross-coverage insurance
    // -------------------------------------------------------
    $display("");
    $display("[Phase 3] Random tests (30 packets)");
    repeat (30) begin
      int len;
      len = $urandom_range(1, 8);
      pkt = new[len];
      foreach (pkt[i]) pkt[i] = $urandom();
      send_packet(pkt, len);
      do_clear();
    end

    // -------------------------------------------------------
    //  Phase 4: Coverage report
    // -------------------------------------------------------
    print_coverage_report();

    $finish;
  end

endmodule
