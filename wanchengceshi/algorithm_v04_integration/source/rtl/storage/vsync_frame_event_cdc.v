`timescale 1ns/1ps
// Convert a pixel-domain VSYNC active edge to a one-cycle DDR-domain event.
// The toggle bridge is intended for frame-rate events, with source events
// separated by substantially more than the destination synchronizer latency.
module vsync_frame_event_cdc #(
    parameter integer VSYNC_ACTIVE_LEVEL = 1
) (
    input  wire src_clk,
    input  wire dst_clk,
    input  wire arst_n,
    input  wire src_vsync,
    output reg  dst_frame_start
);
    localparam VSYNC_ACTIVE = (VSYNC_ACTIVE_LEVEL != 0);
    localparam VSYNC_INACTIVE = !VSYNC_ACTIVE;

    reg [1:0] src_reset_release = 2'b00;
    reg [1:0] dst_reset_release = 2'b00;
    wire src_domain_rst_n = src_reset_release[1];
    wire dst_domain_rst_n = dst_reset_release[1];

    reg src_vsync_d;
    reg src_armed;
    reg src_event_toggle;
    reg dst_event_meta;
    reg dst_event_sync;
    reg dst_event_seen;

    // Asynchronous assertion with independent synchronous release in each domain.
    always @(posedge src_clk or negedge arst_n) begin
        if (!arst_n)
            src_reset_release <= 2'b00;
        else
            src_reset_release <= {src_reset_release[0], 1'b1};
    end

    always @(posedge dst_clk or negedge arst_n) begin
        if (!arst_n)
            dst_reset_release <= 2'b00;
        else
            dst_reset_release <= {dst_reset_release[0], 1'b1};
    end

    // Ignore a partial VSYNC already active during reset. Arm after observing
    // an inactive level, then toggle once on the configured active edge.
    always @(posedge src_clk) begin
        if (!src_domain_rst_n) begin
            src_vsync_d <= VSYNC_INACTIVE;
            src_armed <= 1'b0;
            src_event_toggle <= 1'b0;
        end else begin
            src_vsync_d <= src_vsync;
            if (!src_armed) begin
                if (src_vsync == VSYNC_INACTIVE)
                    src_armed <= 1'b1;
            end else if ((src_vsync == VSYNC_ACTIVE) &&
                         (src_vsync_d == VSYNC_INACTIVE)) begin
                src_event_toggle <= !src_event_toggle;
            end
        end
    end

    // Two-flop toggle synchronizer followed by edge detection in dst_clk.
    always @(posedge dst_clk) begin
        if (!dst_domain_rst_n) begin
            dst_event_meta <= 1'b0;
            dst_event_sync <= 1'b0;
            dst_event_seen <= 1'b0;
            dst_frame_start <= 1'b0;
        end else begin
            dst_event_meta <= src_event_toggle;
            dst_event_sync <= dst_event_meta;
            dst_frame_start <= 1'b0;
            if (dst_event_sync != dst_event_seen) begin
                dst_event_seen <= dst_event_sync;
                dst_frame_start <= 1'b1;
            end
        end
    end
endmodule
