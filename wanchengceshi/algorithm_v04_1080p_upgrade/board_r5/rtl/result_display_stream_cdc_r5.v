`timescale 1ns/1ps
// Cross result-frame pixels and sideband metadata from the DDR reader clock
// into the HDMI pixel clock. A pixel-clock consumer asserts dst_ready only
// for active video pixels; DDR-side backpressure is propagated via src_ready.
module result_display_stream_cdc #(
    parameter integer FIFO_DEPTH = 1024
) (
    input wire src_clk,
    input wire dst_clk,
    input wire rst_n,

    input wire src_valid,
    output wire src_ready,
    input wire [23:0] src_rgb,
    input wire [15:0] src_x,
    input wire [15:0] src_y,
    input wire src_sof,
    input wire src_eol,
    input wire src_eof,
    input wire [31:0] src_frame_id,
    input wire [31:0] src_source_frame_id,
    input wire [15:0] src_cfg_version,

    output wire dst_valid,
    input wire dst_ready,
    output wire [23:0] dst_rgb,
    output wire [15:0] dst_x,
    output wire [15:0] dst_y,
    output wire dst_sof,
    output wire dst_eol,
    output wire dst_eof,
    output wire [31:0] dst_frame_id,
    output wire [31:0] dst_source_frame_id,
    output wire [15:0] dst_cfg_version,

    output reg [31:0] source_stall_count,
    output reg [31:0] display_underflow_count
);
    // Common assertion; release and handshake gating belong to each domain.
    (* ASYNC_REG = "TRUE" *) reg [2:0] src_reset_sync,dst_reset_sync;
    always @(posedge src_clk or negedge rst_n)
        if(!rst_n)src_reset_sync<=0;
        else src_reset_sync<={src_reset_sync[1:0],1'b1};
    always @(posedge dst_clk or negedge rst_n)
        if(!rst_n)dst_reset_sync<=0;
        else dst_reset_sync<={dst_reset_sync[1:0],1'b1};
    wire src_rst_n=src_reset_sync[2];
    wire dst_rst_n=dst_reset_sync[2];
    localparam integer STREAM_WIDTH = 139;
    localparam integer CFG_LSB = 0;
    localparam integer SOURCE_FRAME_ID_LSB = CFG_LSB + 16;
    localparam integer FRAME_ID_LSB = SOURCE_FRAME_ID_LSB + 32;
    localparam integer EOF_BIT = FRAME_ID_LSB + 32;
    localparam integer EOL_BIT = EOF_BIT + 1;
    localparam integer SOF_BIT = EOL_BIT + 1;
    localparam integer Y_LSB = SOF_BIT + 1;
    localparam integer X_LSB = Y_LSB + 16;
    localparam integer RGB_LSB = X_LSB + 16;

    wire [STREAM_WIDTH-1:0] fifo_wr_data = {
        src_rgb, src_x, src_y, src_sof, src_eol, src_eof,
        src_frame_id, src_source_frame_id, src_cfg_version
    };
    wire [STREAM_WIDTH-1:0] fifo_rd_data;
    wire fifo_wr_ready;
    wire fifo_rd_valid;

    assign src_ready = src_rst_n && fifo_wr_ready;
    assign dst_valid = dst_rst_n && fifo_rd_valid;

    // FWFT payload is only exposed when valid, so reset/empty never leaks
    // uninitialized FIFO memory into the video datapath.
    assign dst_cfg_version = dst_valid ? fifo_rd_data[CFG_LSB +: 16] : 16'd0;
    assign dst_source_frame_id = dst_valid ? fifo_rd_data[SOURCE_FRAME_ID_LSB +: 32] : 32'd0;
    assign dst_frame_id = dst_valid ? fifo_rd_data[FRAME_ID_LSB +: 32] : 32'd0;
    assign dst_eof = dst_valid && fifo_rd_data[EOF_BIT];
    assign dst_eol = dst_valid && fifo_rd_data[EOL_BIT];
    assign dst_sof = dst_valid && fifo_rd_data[SOF_BIT];
    assign dst_y = dst_valid ? fifo_rd_data[Y_LSB +: 16] : 16'd0;
    assign dst_x = dst_valid ? fifo_rd_data[X_LSB +: 16] : 16'd0;
    assign dst_rgb = dst_valid ? fifo_rd_data[RGB_LSB +: 24] : 24'd0;

    raw_async_fifo_dualreset_r1 #(
        .DATA_WIDTH(STREAM_WIDTH), .DEPTH(FIFO_DEPTH)
    ) u_result_pixel_fifo (
        .wr_clk(src_clk), .rd_clk(dst_clk), .wr_rst_n(src_rst_n), .rd_rst_n(dst_rst_n),
        .wr_valid(src_valid && src_rst_n), .wr_ready(fifo_wr_ready),
        .wr_data(fifo_wr_data), .rd_valid(fifo_rd_valid),
        .rd_ready(dst_ready && dst_valid), .rd_data(fifo_rd_data)
    );

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)
            source_stall_count <= 32'd0;
        else if (src_valid && !src_ready)
            source_stall_count <= source_stall_count + 1'b1;
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n)
            display_underflow_count <= 32'd0;
        else if (dst_ready && !fifo_rd_valid)
            display_underflow_count <= display_underflow_count + 1'b1;
    end
endmodule
