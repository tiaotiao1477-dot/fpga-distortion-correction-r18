`timescale 1ns/1ps
// Pack the frozen RGB888 result-pixel stream into row-aligned 256-bit DDR beats.
module result_rgb888_beat_packer #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720
) (
    input wire clk,
    input wire rst_n,
    input wire frame_clear,

    input wire in_valid,
    output wire in_ready,
    input wire [15:0] in_x,
    input wire [15:0] in_y,
    input wire in_sof,
    input wire in_eol,
    input wire [23:0] in_pixel_rgb,

    output reg out_valid,
    input wire out_ready,
    output reg [255:0] out_data,
    output reg out_frame_last,
    output reg protocol_error
);
    reg [15:0] expected_x;
    reg [15:0] expected_y;
    reg [2:0] lane;
    reg [255:0] assembly_data;
    reg [255:0] data_with_pixel;
    integer pad_lane;

    assign in_ready = rst_n && !frame_clear && !out_valid;

    always @* begin
        data_with_pixel = assembly_data;
        data_with_pixel[lane*32 +: 32] = {8'h00, in_pixel_rgb};
        if (in_eol) begin
            for (pad_lane = 0; pad_lane < 8; pad_lane = pad_lane + 1)
                if (pad_lane > lane)
                    data_with_pixel[pad_lane*32 +: 32] = 32'd0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            expected_x <= 16'd0;
            expected_y <= 16'd0;
            lane <= 3'd0;
            assembly_data <= 256'd0;
            out_valid <= 1'b0;
            out_data <= 256'd0;
            out_frame_last <= 1'b0;
            protocol_error <= 1'b0;
        end else if (frame_clear) begin
            expected_x <= 16'd0;
            expected_y <= 16'd0;
            lane <= 3'd0;
            assembly_data <= 256'd0;
            out_valid <= 1'b0;
            out_data <= 256'd0;
            out_frame_last <= 1'b0;
            protocol_error <= 1'b0;
        end else begin
            if (out_valid && out_ready)
                out_valid <= 1'b0;

            if (in_valid && in_ready) begin
                if ((in_x != expected_x) || (in_y != expected_y) ||
                    (in_sof != ((expected_x == 16'd0) && (expected_y == 16'd0))) ||
                    (in_eol != (expected_x == FRAME_WIDTH - 1)) ||
                    (in_x >= FRAME_WIDTH) || (in_y >= FRAME_HEIGHT))
                    protocol_error <= 1'b1;

                assembly_data <= data_with_pixel;
                if ((lane == 3'd7) || in_eol) begin
                    out_data <= data_with_pixel;
                    out_valid <= 1'b1;
                    out_frame_last <= in_eol && (in_y == FRAME_HEIGHT - 1);
                    assembly_data <= 256'd0;
                    lane <= 3'd0;
                end else begin
                    lane <= lane + 1'b1;
                end

                if (in_eol) begin
                    expected_x <= 16'd0;
                    if (in_y == FRAME_HEIGHT - 1)
                        expected_y <= 16'd0;
                    else
                        expected_y <= in_y + 1'b1;
                end else begin
                    expected_x <= expected_x + 1'b1;
                end
            end
        end
    end
endmodule
