`timescale 1ns/1ps
// Convert tagged RGB565 pixels into 256-bit DDR beats containing eight
// {8'h00,R,G,B} words. Each EOL pads the final beat with black words so the
// following row begins on a 32-byte boundary.
module rgb565_beat_packer #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer PIXEL_COUNT_WIDTH = 32,
    parameter integer INPUT_RGB888 = 0
) (
    input wire clk,
    input wire rst_n,
    input wire frame_clear,

    // in_data = {SOF, EOL, EOF, RGB565 or RGB888}, selected at elaboration.
    input wire in_valid,
    output wire in_ready,
    input wire [(INPUT_RGB888 ? 27 : 19)-1:0] in_data,

    output reg out_valid,
    input wire out_ready,
    output reg [255:0] out_data,
    output reg out_frame_last,
    output reg protocol_error
);
    localparam [31:0] FRAME_PIXELS = FRAME_WIDTH * FRAME_HEIGHT;

    reg [2:0] lane_count;
    reg [15:0] column_count;
    reg [PIXEL_COUNT_WIDTH-1:0] pixel_count;
    reg [255:0] partial_beat;

    localparam integer COLOR_BITS = INPUT_RGB888 ? 24 : 16;
    wire sof_in = in_data[COLOR_BITS+2];
    wire eol_in = in_data[COLOR_BITS+1];
    wire eof_in = in_data[COLOR_BITS];
    wire [15:0] rgb565_in = in_data[15:0];
    wire [7:0] red8 = {rgb565_in[15:11], rgb565_in[15:13]};
    wire [7:0] green8 = {rgb565_in[10:5], rgb565_in[10:9]};
    wire [7:0] blue8 = {rgb565_in[4:0], rgb565_in[4:2]};
    wire [31:0] rgb888_word;
    generate
        if (INPUT_RGB888) begin : g_rgb888
            assign rgb888_word = {8'h00, in_data[23:0]};
        end else begin : g_rgb565
            assign rgb888_word = {8'h00, red8, green8, blue8};
        end
    endgenerate

    reg [255:0] merged_beat;
    integer fill_lane;
    always @* begin
        merged_beat = partial_beat;
        merged_beat[lane_count*32 +: 32] = rgb888_word;
        if (eol_in) begin
            for (fill_lane = 0; fill_lane < 8; fill_lane = fill_lane + 1)
                if (fill_lane > lane_count)
                    merged_beat[fill_lane*32 +: 32] = 32'd0;
        end
    end

    assign in_ready = !out_valid || out_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane_count <= 3'd0;
            column_count <= 16'd0;
            pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
            partial_beat <= 256'd0;
            out_valid <= 1'b0;
            out_data <= 256'd0;
            out_frame_last <= 1'b0;
            protocol_error <= 1'b0;
        end else if (frame_clear) begin
            lane_count <= 3'd0;
            column_count <= 16'd0;
            pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
            partial_beat <= 256'd0;
            out_valid <= 1'b0;
            out_data <= 256'd0;
            out_frame_last <= 1'b0;
            protocol_error <= 1'b0;
        end else begin
            if (out_valid && out_ready)
                out_valid <= 1'b0;

            if (in_valid && in_ready) begin
                if ((sof_in != (pixel_count == 0)) ||
                    (eol_in != (column_count == FRAME_WIDTH - 1)) ||
                    (eof_in != (pixel_count == FRAME_PIXELS - 1)) ||
                    (pixel_count >= FRAME_PIXELS))
                    protocol_error <= 1'b1;

                if (eof_in) begin
                    pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
                    column_count <= 16'd0;
                end else begin
                    pixel_count <= pixel_count + 1'b1;
                    if (eol_in)
                        column_count <= 16'd0;
                    else
                        column_count <= column_count + 1'b1;
                end

                if (eol_in || (lane_count == 3'd7)) begin
                    out_data <= merged_beat;
                    out_frame_last <= eof_in;
                    out_valid <= 1'b1;
                    partial_beat <= 256'd0;
                    lane_count <= 3'd0;
                end else begin
                    partial_beat <= merged_beat;
                    lane_count <= lane_count + 1'b1;
                end
            end
        end
    end
endmodule
