// ============================================================
//  design.sv — CRC-8 DUT + Interface  (fully implemented)
//  DO NOT EDIT — this is the hardware you are verifying.
// ============================================================

// ------------ Interface (bundles all DUT signals) -----------
interface crc8_if (input logic clk);
  logic       rst_n;
  logic       clear;
  logic [7:0] data_in;
  logic       data_valid;
  logic [7:0] crc_out;
  logic       crc_valid;
endinterface

// ------------ CRC-8 RTL module ------------------------------
module crc8 (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       clear,
  input  logic [7:0] data_in,
  input  logic       data_valid,
  output logic [7:0] crc_out,
  output logic       crc_valid
);
  localparam [7:0] POLY = 8'h07;

  logic [7:0] crc_reg, crc_next;
  logic       valid_delay;
  logic [7:0]  crc_out_reg;

  // Combinational: process one byte, one bit at a time (MSB first)
  function automatic logic [7:0] compute_crc(
    input logic [7:0] crc_in, data
  );
    logic [7:0] crc;
    crc = crc_in;
    for (int i = 7; i >= 0; i--) begin
      if (crc[7] ^ data[i]) crc = (crc << 1) ^ POLY;
      else                   crc = (crc << 1);
    end
    return crc;
  endfunction

  assign crc_next = data_valid ? compute_crc(crc_reg, data_in) : crc_reg;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)      begin crc_reg <= 8'h00; valid_delay <= 0; crc_out_reg <= 8'h00; end
    else if (clear)  begin crc_reg <= 8'h00; crc_out_reg <= 8'h00; end
    else             begin crc_reg <= crc_next; valid_delay <= data_valid; crc_out_reg <= crc_reg; end
  end

  assign crc_out   = crc_out_reg;
  assign crc_valid = valid_delay;
endmodule
