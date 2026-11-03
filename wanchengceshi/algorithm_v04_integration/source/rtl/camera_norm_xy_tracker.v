`timescale 1ns/1ps
// Generate zero-based coordinates for the normalized RGB565 stream.
// Coordinates advance only on pixel_valid; frame/line edges may coincide with
// a valid first pixel and still map it to x=0 and the correct first-line y.
module camera_norm_xy_tracker (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_active,
    input  wire        line_active,
    input  wire        pixel_valid,
    output wire        frame_start,
    output wire [11:0] pixel_x,
    output wire [11:0] pixel_y
);
    reg frame_active_d;
    reg line_active_d;
    reg line_seen_in_frame;
    reg [11:0] x_count;
    reg [11:0] y_count;

    wire frame_rise = frame_active && !frame_active_d;
    wire line_rise = line_active && !line_active_d;

    function [11:0] increment_saturated;
        input [11:0] value;
        begin
            increment_saturated = (value == 12'hFFF) ? value : value + 12'd1;
        end
    endfunction

    assign frame_start = frame_rise;
    assign pixel_x = (frame_rise || line_rise) ? 12'd0 : x_count;
    assign pixel_y = frame_rise ? 12'd0 :
                     line_rise ? ((!line_seen_in_frame) ? 12'd0 :
                                  increment_saturated(y_count)) : y_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_active_d <= 1'b0;
            line_active_d <= 1'b0;
            line_seen_in_frame <= 1'b0;
            x_count <= 12'd0;
            y_count <= 12'd0;
        end else begin
            frame_active_d <= frame_active;
            line_active_d <= line_active;

            if (frame_rise) begin
                y_count <= 12'd0;
                line_seen_in_frame <= line_active;
                x_count <= pixel_valid ? 12'd1 : 12'd0;
            end else if (line_rise) begin
                if (line_seen_in_frame)
                    y_count <= increment_saturated(y_count);
                else
                    y_count <= 12'd0;
                line_seen_in_frame <= 1'b1;
                x_count <= pixel_valid ? 12'd1 : 12'd0;
            end else if (pixel_valid) begin
                x_count <= increment_saturated(x_count);
            end
        end
    end
endmodule
