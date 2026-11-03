`timescale 1ns/1ps
// Convert each observed active HREF row into exactly H_ACTIVE RGB565 pixels.
// Real pixels are forwarded in order; a short row is right-padded with black
// during horizontal blanking. There is intentionally no input backpressure.
module camera_line_normalizer #(
  parameter integer H_ACTIVE = 1280
)(
  input             pclk,
  input             rst_n,
  input             frame_in,
  input             line_in,
  input             valid_in,
  input      [15:0] pixel_in,
  output reg        frame_out,
  output reg        line_out,
  output reg        valid_out,
  output reg        pad_valid,
  output reg [15:0] pixel_out
);
  reg line_in_d;
  reg padding_active;
  reg [15:0] pixel_count;

  always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
      line_in_d      <= 1'b0;
      padding_active <= 1'b0;
      pixel_count    <= 16'd0;
      frame_out      <= 1'b0;
      line_out       <= 1'b0;
      valid_out      <= 1'b0;
      pad_valid      <= 1'b0;
      pixel_out      <= 16'h0000;
    end else begin
      // The frame qualifier is registered in the same PCLK domain as pixels.
      frame_out <= frame_in;
      line_in_d <= line_in;
      valid_out <= 1'b0;
      pad_valid <= 1'b0;

      if (line_in) begin
        // A new HREF row starts with a fresh row-local output count. Account
        // for a valid pixel that can coincide with the first sampled HREF.
        padding_active <= 1'b0;
        line_out <= 1'b1;
        if (!line_in_d) begin
          if (valid_in && (H_ACTIVE > 0)) begin
            pixel_out   <= pixel_in;
            valid_out   <= 1'b1;
            pixel_count <= 16'd1;
          end else begin
            pixel_count <= 16'd0;
          end
        end else if (valid_in && (pixel_count < H_ACTIVE)) begin
          pixel_out   <= pixel_in;
          valid_out   <= 1'b1;
          pixel_count <= pixel_count + 1'b1;
        end
      end else if (line_in_d) begin
        // HREF fell. A final valid pixel on this same PCLK edge is still part
        // of the row and must be forwarded before black padding begins.
        if (valid_in && (pixel_count < H_ACTIVE)) begin
          pixel_out <= pixel_in;
          valid_out <= 1'b1;
          pixel_count <= pixel_count + 1'b1;
          if ((pixel_count + 1'b1) < H_ACTIVE) begin
            padding_active <= 1'b1;
            line_out <= 1'b1;
          end else begin
            padding_active <= 1'b0;
            line_out <= 1'b1;
          end
        end else if (pixel_count < H_ACTIVE) begin
          // Keep the row open; the first pad pixel is emitted next cycle.
          padding_active <= 1'b1;
          line_out <= 1'b1;
        end else begin
          padding_active <= 1'b0;
          line_out <= 1'b0;
        end
      end else if (padding_active && (pixel_count < H_ACTIVE)) begin
        pixel_out   <= 16'h0000;
        valid_out   <= 1'b1;
        pad_valid   <= 1'b1;
        pixel_count <= pixel_count + 1'b1;
        line_out    <= 1'b1;
        if ((pixel_count + 1'b1) >= H_ACTIVE)
          padding_active <= 1'b0;
      end else begin
        // No HREF row is active: vertical blank must not create rows.
        line_out <= 1'b0;
        padding_active <= 1'b0;
        pixel_count <= 16'd0;
      end
    end
  end
endmodule
