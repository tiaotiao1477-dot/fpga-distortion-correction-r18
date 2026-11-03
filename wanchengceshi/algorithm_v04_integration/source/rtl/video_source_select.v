`timescale 1ns/1ps

// Compile-time video source selection.  All selected sources use the same
// downstream timing and HDMI transmitter.
//   0 = internal RGB565 test pattern directly to HDMI
//   1 = RGB565 test pattern written/read through the DDR frame buffer
//   2 = OV5640 RGB565 written/read through the DDR frame buffer
module video_source_select #(
    parameter integer SOURCE = 2
)(
    input              pattern_de,
    input      [15:0]  pattern_rgb565,
    input              ddr_de,
    input      [15:0]  ddr_rgb565,
    input              camera_de,
    input      [15:0]  camera_rgb565,
    output reg         selected_de,
    output reg [15:0]  selected_rgb565
);
    always @(*) begin
        selected_de = 1'b0;
        selected_rgb565 = 16'h0000;
        case (SOURCE)
            0: begin
                selected_de = pattern_de;
                selected_rgb565 = pattern_de ? pattern_rgb565 : 16'h0000;
            end
            1: begin
                selected_de = ddr_de;
                selected_rgb565 = ddr_de ? ddr_rgb565 : 16'h0000;
            end
            2: begin
                selected_de = camera_de;
                selected_rgb565 = camera_de ? camera_rgb565 : 16'h0000;
            end
            default: begin
                selected_de = 1'b0;
                selected_rgb565 = 16'h0000;
            end
        endcase
    end
endmodule
