`timescale 1ns/1ps
// Per-frame center/left/right RGB565 sums with request-correlated publish.
// Each region is packed into 80 bits: count[15:0], R sum[20:0],
// G sum[21:0], and B sum[20:0]. The top 16 snapshot bits carry a format
// marker and per-region truncation flags.
module camera_rgb565_roi_diag #(
    parameter integer H_ACTIVE = 1280,
    parameter integer V_ACTIVE = 720
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         frame_start,
    input  wire         pixel_valid,
    input  wire [11:0]  pixel_x,
    input  wire [11:0]  pixel_y,
    input  wire [15:0]  pixel_rgb565,
    input  wire         request_toggle_async,
    output reg  [255:0] snapshot_bus,
    output reg          ack_toggle
);
    reg request_meta;
    reg request_sync;
    reg request_seen;
    reg have_frame;

    reg [31:0] center_count;
    reg [31:0] center_red_sum;
    reg [31:0] center_green_sum;
    reg [31:0] center_blue_sum;
    reg [31:0] left_count;
    reg [31:0] left_red_sum;
    reg [31:0] left_green_sum;
    reg [31:0] left_blue_sum;
    reg [31:0] right_count;
    reg [31:0] right_red_sum;
    reg [31:0] right_green_sum;
    reg [31:0] right_blue_sum;

    wire pixel_in_bounds = (pixel_x < H_ACTIVE) && (pixel_y < V_ACTIVE);
    wire pixel_in_center = pixel_in_bounds &&
                           (pixel_x >= 12'd480) && (pixel_x <= 12'd799) &&
                           (pixel_y >= 12'd270) && (pixel_y <= 12'd449);
    wire pixel_in_left = pixel_in_bounds &&
                         (pixel_x <= 12'd319) &&
                         (pixel_y >= 12'd270) && (pixel_y <= 12'd449);
    wire pixel_in_right = pixel_in_bounds &&
                          (pixel_x >= 12'd960) && (pixel_x <= 12'd1279) &&
                          (pixel_y >= 12'd270) && (pixel_y <= 12'd449);

    wire [4:0] pixel_red = pixel_rgb565[15:11];
    wire [5:0] pixel_green = pixel_rgb565[10:5];
    wire [4:0] pixel_blue = pixel_rgb565[4:0];

    function [79:0] pack_region;
        input [31:0] count;
        input [31:0] red_sum;
        input [31:0] green_sum;
        input [31:0] blue_sum;
        begin
            pack_region = {blue_sum[20:0], green_sum[21:0],
                           red_sum[20:0], count[15:0]};
        end
    endfunction

    wire [3:0] center_overflow = {
        (center_blue_sum[31:21] != 11'd0),
        (center_green_sum[31:22] != 10'd0),
        (center_red_sum[31:21] != 11'd0),
        (center_count[31:16] != 16'd0)
    };
    wire [3:0] left_overflow = {
        (left_blue_sum[31:21] != 11'd0),
        (left_green_sum[31:22] != 10'd0),
        (left_red_sum[31:21] != 11'd0),
        (left_count[31:16] != 16'd0)
    };
    wire [3:0] right_overflow = {
        (right_blue_sum[31:21] != 11'd0),
        (right_green_sum[31:22] != 10'd0),
        (right_red_sum[31:21] != 11'd0),
        (right_count[31:16] != 16'd0)
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            request_meta <= 1'b0;
            request_sync <= 1'b0;
            request_seen <= 1'b0;
            have_frame <= 1'b0;
            center_count <= 32'd0;
            center_red_sum <= 32'd0;
            center_green_sum <= 32'd0;
            center_blue_sum <= 32'd0;
            left_count <= 32'd0;
            left_red_sum <= 32'd0;
            left_green_sum <= 32'd0;
            left_blue_sum <= 32'd0;
            right_count <= 32'd0;
            right_red_sum <= 32'd0;
            right_green_sum <= 32'd0;
            right_blue_sum <= 32'd0;
            snapshot_bus <= 256'd0;
            ack_toggle <= 1'b0;
        end else begin
            request_meta <= request_toggle_async;
            request_sync <= request_meta;

            if (frame_start) begin
                if (have_frame && (request_sync != request_seen)) begin
                    snapshot_bus <= {4'hA, right_overflow,
                                     left_overflow, center_overflow,
                                     pack_region(right_count, right_red_sum,
                                                 right_green_sum, right_blue_sum),
                                     pack_region(left_count, left_red_sum,
                                                 left_green_sum, left_blue_sum),
                                     pack_region(center_count, center_red_sum,
                                                 center_green_sum, center_blue_sum)};
                    ack_toggle <= request_sync;
                    request_seen <= request_sync;
                end

                // frame_start starts the new frame. A valid first pixel on
                // this same edge belongs to that frame and is not discarded.
                center_count <= (pixel_valid && pixel_in_center) ? 32'd1 : 32'd0;
                center_red_sum <= (pixel_valid && pixel_in_center) ? pixel_red : 32'd0;
                center_green_sum <= (pixel_valid && pixel_in_center) ? pixel_green : 32'd0;
                center_blue_sum <= (pixel_valid && pixel_in_center) ? pixel_blue : 32'd0;
                left_count <= (pixel_valid && pixel_in_left) ? 32'd1 : 32'd0;
                left_red_sum <= (pixel_valid && pixel_in_left) ? pixel_red : 32'd0;
                left_green_sum <= (pixel_valid && pixel_in_left) ? pixel_green : 32'd0;
                left_blue_sum <= (pixel_valid && pixel_in_left) ? pixel_blue : 32'd0;
                right_count <= (pixel_valid && pixel_in_right) ? 32'd1 : 32'd0;
                right_red_sum <= (pixel_valid && pixel_in_right) ? pixel_red : 32'd0;
                right_green_sum <= (pixel_valid && pixel_in_right) ? pixel_green : 32'd0;
                right_blue_sum <= (pixel_valid && pixel_in_right) ? pixel_blue : 32'd0;
                have_frame <= 1'b1;
            end else if (pixel_valid) begin
                if (pixel_in_center) begin
                    center_count <= center_count + 1'b1;
                    center_red_sum <= center_red_sum + pixel_red;
                    center_green_sum <= center_green_sum + pixel_green;
                    center_blue_sum <= center_blue_sum + pixel_blue;
                end
                if (pixel_in_left) begin
                    left_count <= left_count + 1'b1;
                    left_red_sum <= left_red_sum + pixel_red;
                    left_green_sum <= left_green_sum + pixel_green;
                    left_blue_sum <= left_blue_sum + pixel_blue;
                end
                if (pixel_in_right) begin
                    right_count <= right_count + 1'b1;
                    right_red_sum <= right_red_sum + pixel_red;
                    right_green_sum <= right_green_sum + pixel_green;
                    right_blue_sum <= right_blue_sum + pixel_blue;
                end
            end
        end
    end
endmodule
