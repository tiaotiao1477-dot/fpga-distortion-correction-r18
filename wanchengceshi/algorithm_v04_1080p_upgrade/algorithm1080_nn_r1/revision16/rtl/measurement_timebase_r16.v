`timescale 1ns/1ps
module measurement_timebase_r16 (
 input wire clk,
 input wire rst_n,
 output wire [63:0] ticks,
 output reg [63:0] time_gray
);
 reg [31:0] time_low;
 reg [31:0] time_high;
 wire [31:0] time_low_next = time_low + 32'd1;
 wire time_low_carry = &time_low;
 wire [31:0] time_high_next = time_high + time_low_carry;
 wire [63:0] time_binary_next = {time_high_next,time_low_next};
 wire [63:0] time_gray_next = time_binary_next ^ (time_binary_next >> 1);
 assign ticks = {time_high,time_low};
 always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
   time_low <= 32'd0;
   time_high <= 32'd0;
   time_gray <= 64'd0;
  end else begin
   time_low <= time_low_next;
   time_high <= time_high_next;
   time_gray <= time_gray_next;
  end
 end
endmodule
