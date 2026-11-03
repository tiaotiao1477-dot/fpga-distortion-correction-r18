`timescale 1ns/1ps

// Deterministic RGB565 source for the HDMI-only and DDR-loopback stages.
// The module is intentionally independent of the camera and DDR clocks.
module video_test_pattern (
    input              de,
    input      [11:0]  x,
    input      [11:0]  y,
    input      [2:0]   pattern_select,
    output reg [15:0]  rgb565
);
    always @(*) begin
        rgb565 = 16'h0000;
        if (de) begin
            case (pattern_select)
                3'd0: rgb565 = 16'hf800; // red
                3'd1: rgb565 = 16'h07e0; // green
                3'd2: rgb565 = 16'h001f; // blue
                3'd3: rgb565 = 16'hffff; // white
                3'd4: rgb565 = 16'h0000; // black
                3'd5: begin                // eight 160-pixel bars
                    if      (x < 12'd160)  rgb565 = 16'hf800; // red
                    else if (x < 12'd320)  rgb565 = 16'h07e0; // green
                    else if (x < 12'd480)  rgb565 = 16'h001f; // blue
                    else if (x < 12'd640)  rgb565 = 16'hffff; // white
                    else if (x < 12'd800)  rgb565 = 16'hffe0; // yellow
                    else if (x < 12'd960)  rgb565 = 16'h07ff; // cyan
                    else if (x < 12'd1120) rgb565 = 16'hf81f; // magenta
                    else                   rgb565 = 16'h0000; // black
                end
                3'd6: rgb565 = (x[5] ^ y[5]) ? 16'h0000 : 16'hffff;
                default: rgb565 = 16'h0000;
            endcase
        end
    end
endmodule
