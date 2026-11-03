`timescale 1ns/1ps
// Composes the source/result write and fetch/display read arbiters with one
// byte-addressed DDR adapter. Physical DDR ownership remains at the system top.
module storage_ddr_client_mux #(
    parameter integer CTRL_ADDR_WIDTH = 28,
    parameter integer DISPLAY_RESPONSE_BUFFER_ENABLE = 0,
    parameter integer ADDR_UNIT_SHIFT = 2,
    parameter integer MEM_BEAT_BYTES = 32,
    parameter integer DATA_WIDTH = 256,
    parameter integer REQ_ID_W = 8,
    parameter integer MAX_BURST_BEATS = 16
) (
    input wire  nn_wr_req_valid,
    output wire  nn_wr_req_ready,
    input wire [63:0] nn_wr_req_addr,
    input wire [31:0] nn_wr_req_len,
    input wire [7:0] nn_wr_req_id,
    input wire  nn_wr_data_valid,
    output wire  nn_wr_data_ready,
    input wire [255:0] nn_wr_data,
    input wire  nn_wr_data_last,
    output wire  nn_wr_done_valid,
    output wire [7:0] nn_wr_done_id,
    output wire  nn_wr_done_error,
    input wire  nn_rd_req_valid,
    output wire  nn_rd_req_ready,
    input wire [63:0] nn_rd_req_addr,
    input wire [31:0] nn_rd_req_len,
    input wire [7:0] nn_rd_req_id,
    output wire  nn_rd_rsp_valid,
    input wire  nn_rd_rsp_ready,
    output wire [255:0] nn_rd_rsp_data,
    output wire [7:0] nn_rd_rsp_id,
    output wire  nn_rd_rsp_last,
    output wire  nn_rd_rsp_error,
    input wire ddr_clk,
    input wire ddr_rst_n,
    input wire ddr_init_done,

    input wire src_mem_wr_req_valid,
    output wire src_mem_wr_req_ready,
    input wire [63:0] src_mem_wr_req_addr,
    input wire [31:0] src_mem_wr_req_len,
    input wire [REQ_ID_W-1:0] src_mem_wr_req_id,
    input wire src_mem_wr_data_valid,
    output wire src_mem_wr_data_ready,
    input wire [DATA_WIDTH-1:0] src_mem_wr_data,
    input wire src_mem_wr_data_last,
    output wire src_mem_wr_done_valid,
    output wire [REQ_ID_W-1:0] src_mem_wr_done_id,
    output wire src_mem_wr_done_error,

    input wire res_mem_wr_req_valid,
    output wire res_mem_wr_req_ready,
    input wire [63:0] res_mem_wr_req_addr,
    input wire [31:0] res_mem_wr_req_len,
    input wire [REQ_ID_W-1:0] res_mem_wr_req_id,
    input wire res_mem_wr_data_valid,
    output wire res_mem_wr_data_ready,
    input wire [DATA_WIDTH-1:0] res_mem_wr_data,
    input wire res_mem_wr_data_last,
    output wire res_mem_wr_done_valid,
    output wire [REQ_ID_W-1:0] res_mem_wr_done_id,
    output wire res_mem_wr_done_error,

    input wire fetch_mem_rd_req_valid,
    output wire fetch_mem_rd_req_ready,
    input wire [63:0] fetch_mem_rd_req_addr,
    input wire [31:0] fetch_mem_rd_req_len,
    input wire [REQ_ID_W-1:0] fetch_mem_rd_req_id,
    output wire fetch_mem_rd_rsp_valid,
    input wire fetch_mem_rd_rsp_ready,
    output wire [DATA_WIDTH-1:0] fetch_mem_rd_rsp_data,
    output wire [REQ_ID_W-1:0] fetch_mem_rd_rsp_id,
    output wire fetch_mem_rd_rsp_last,
    output wire fetch_mem_rd_rsp_error,

    input wire display_mem_rd_req_valid,
    output wire display_mem_rd_req_ready,
    input wire [63:0] display_mem_rd_req_addr,
    input wire [31:0] display_mem_rd_req_len,
    input wire [REQ_ID_W-1:0] display_mem_rd_req_id,
    output wire display_mem_rd_rsp_valid,
    input wire display_mem_rd_rsp_ready,
    output wire [DATA_WIDTH-1:0] display_mem_rd_rsp_data,
    output wire [REQ_ID_W-1:0] display_mem_rd_rsp_id,
    output wire display_mem_rd_rsp_last,
    output wire display_mem_rd_rsp_error,

    output wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr,
    output wire [3:0] axi_awuser_id,
    output wire [3:0] axi_awlen,
    output wire axi_awvalid,
    input wire axi_awready,
    output wire [DATA_WIDTH-1:0] axi_wdata,
    output wire [DATA_WIDTH/8-1:0] axi_wstrb,
    input wire axi_wready,
    input wire axi_wlast,
    output wire [CTRL_ADDR_WIDTH-1:0] axi_araddr,
    output wire [3:0] axi_aruser_id,
    output wire [3:0] axi_arlen,
    output wire axi_arvalid,
    input wire axi_arready,
    input wire [DATA_WIDTH-1:0] axi_rdata,
    input wire [3:0] axi_rid,
    input wire axi_rlast,
    input wire axi_rvalid
);
    wire vid_wr_req_valid;
    wire vid_wr_req_ready;
    wire [63:0] vid_wr_req_addr;
    wire [31:0] vid_wr_req_len;
    wire [REQ_ID_W-1:0] vid_wr_req_id;
    wire vid_wr_data_valid;
    wire vid_wr_data_ready;
    wire [DATA_WIDTH-1:0] vid_wr_data;
    wire vid_wr_data_last;
    wire vid_wr_done_valid;
    wire [REQ_ID_W-1:0] vid_wr_done_id;
    wire vid_wr_done_error;

    wire vid_rd_req_valid;
    wire vid_rd_req_ready;
    wire [63:0] vid_rd_req_addr;
    wire [31:0] vid_rd_req_len;
    wire [REQ_ID_W-1:0] vid_rd_req_id;
    wire vid_rd_rsp_valid;
    wire vid_rd_rsp_ready;
    wire [DATA_WIDTH-1:0] vid_rd_rsp_data;
    wire [REQ_ID_W-1:0] vid_rd_rsp_id;
    wire vid_rd_rsp_last;
    wire vid_rd_rsp_error;

    storage_write_arbiter #(
        .REQ_ID_W(REQ_ID_W), .DATA_WIDTH(DATA_WIDTH)
    ) write_arbiter (
        .ddr_clk(ddr_clk), .ddr_rst_n(ddr_rst_n),
        .src_mem_wr_req_valid(src_mem_wr_req_valid),
        .src_mem_wr_req_ready(src_mem_wr_req_ready),
        .src_mem_wr_req_addr(src_mem_wr_req_addr),
        .src_mem_wr_req_len(src_mem_wr_req_len),
        .src_mem_wr_req_id(src_mem_wr_req_id),
        .src_mem_wr_data_valid(src_mem_wr_data_valid),
        .src_mem_wr_data_ready(src_mem_wr_data_ready),
        .src_mem_wr_data(src_mem_wr_data), .src_mem_wr_data_last(src_mem_wr_data_last),
        .src_mem_wr_done_valid(src_mem_wr_done_valid),
        .src_mem_wr_done_id(src_mem_wr_done_id),
        .src_mem_wr_done_error(src_mem_wr_done_error),
        .res_mem_wr_req_valid(res_mem_wr_req_valid),
        .res_mem_wr_req_ready(res_mem_wr_req_ready),
        .res_mem_wr_req_addr(res_mem_wr_req_addr),
        .res_mem_wr_req_len(res_mem_wr_req_len),
        .res_mem_wr_req_id(res_mem_wr_req_id),
        .res_mem_wr_data_valid(res_mem_wr_data_valid),
        .res_mem_wr_data_ready(res_mem_wr_data_ready),
        .res_mem_wr_data(res_mem_wr_data), .res_mem_wr_data_last(res_mem_wr_data_last),
        .res_mem_wr_done_valid(res_mem_wr_done_valid),
        .res_mem_wr_done_id(res_mem_wr_done_id),
        .res_mem_wr_done_error(res_mem_wr_done_error),
        .mem_wr_req_valid(vid_wr_req_valid), .mem_wr_req_ready(vid_wr_req_ready),
        .mem_wr_req_addr(vid_wr_req_addr), .mem_wr_req_len(vid_wr_req_len),
        .mem_wr_req_id(vid_wr_req_id), .mem_wr_data_valid(vid_wr_data_valid),
        .mem_wr_data_ready(vid_wr_data_ready), .mem_wr_data(vid_wr_data),
        .mem_wr_data_last(vid_wr_data_last), .mem_wr_done_valid(vid_wr_done_valid),
        .mem_wr_done_id(vid_wr_done_id), .mem_wr_done_error(vid_wr_done_error)
    );

    wire display_raw_valid,display_raw_ready,display_raw_last,display_raw_error;
    wire [DATA_WIDTH-1:0] display_raw_data;
    wire [REQ_ID_W-1:0] display_raw_id;
    generate if(DISPLAY_RESPONSE_BUFFER_ENABLE!=0)begin:g_display_response_buffer
        display_read_response_buffer #(.DATA_WIDTH(DATA_WIDTH),.REQ_ID_W(REQ_ID_W),.DEPTH(32)) u_buffer(
            .clk(ddr_clk),.rst_n(ddr_rst_n),
            .in_valid(display_raw_valid),.in_ready(display_raw_ready),.in_data(display_raw_data),
            .in_id(display_raw_id),.in_last(display_raw_last),.in_error(display_raw_error),
            .out_valid(display_mem_rd_rsp_valid),.out_ready(display_mem_rd_rsp_ready),.out_data(display_mem_rd_rsp_data),
            .out_id(display_mem_rd_rsp_id),.out_last(display_mem_rd_rsp_last),.out_error(display_mem_rd_rsp_error));
    end else begin:g_display_direct
        assign display_mem_rd_rsp_valid=display_raw_valid;
        assign display_raw_ready=display_mem_rd_rsp_ready;
        assign display_mem_rd_rsp_data=display_raw_data;
        assign display_mem_rd_rsp_id=display_raw_id;
        assign display_mem_rd_rsp_last=display_raw_last;
        assign display_mem_rd_rsp_error=display_raw_error;
    end endgenerate

    storage_read_arbiter #(
        .REQ_ID_W(REQ_ID_W), .DATA_WIDTH(DATA_WIDTH)
    ) read_arbiter (
        .ddr_clk(ddr_clk), .ddr_rst_n(ddr_rst_n),
        .fetch_mem_rd_req_valid(fetch_mem_rd_req_valid),
        .fetch_mem_rd_req_ready(fetch_mem_rd_req_ready),
        .fetch_mem_rd_req_addr(fetch_mem_rd_req_addr),
        .fetch_mem_rd_req_len(fetch_mem_rd_req_len),
        .fetch_mem_rd_req_id(fetch_mem_rd_req_id),
        .fetch_mem_rd_rsp_valid(fetch_mem_rd_rsp_valid),
        .fetch_mem_rd_rsp_ready(fetch_mem_rd_rsp_ready),
        .fetch_mem_rd_rsp_data(fetch_mem_rd_rsp_data),
        .fetch_mem_rd_rsp_id(fetch_mem_rd_rsp_id),
        .fetch_mem_rd_rsp_last(fetch_mem_rd_rsp_last),
        .fetch_mem_rd_rsp_error(fetch_mem_rd_rsp_error),
        .display_mem_rd_req_valid(display_mem_rd_req_valid),
        .display_mem_rd_req_ready(display_mem_rd_req_ready),
        .display_mem_rd_req_addr(display_mem_rd_req_addr),
        .display_mem_rd_req_len(display_mem_rd_req_len),
        .display_mem_rd_req_id(display_mem_rd_req_id),
        .display_mem_rd_rsp_valid(display_raw_valid),
        .display_mem_rd_rsp_ready(display_raw_ready),
        .display_mem_rd_rsp_data(display_raw_data),
        .display_mem_rd_rsp_id(display_raw_id),
        .display_mem_rd_rsp_last(display_raw_last),
        .display_mem_rd_rsp_error(display_raw_error),
        .mem_rd_req_valid(vid_rd_req_valid), .mem_rd_req_ready(vid_rd_req_ready),
        .mem_rd_req_addr(vid_rd_req_addr), .mem_rd_req_len(vid_rd_req_len),
        .mem_rd_req_id(vid_rd_req_id), .mem_rd_rsp_valid(vid_rd_rsp_valid),
        .mem_rd_rsp_ready(vid_rd_rsp_ready), .mem_rd_rsp_data(vid_rd_rsp_data),
        .mem_rd_rsp_id(vid_rd_rsp_id), .mem_rd_rsp_last(vid_rd_rsp_last),
        .mem_rd_rsp_error(vid_rd_rsp_error)
    );

    wire  mem_wr_req_valid;
    wire  mem_wr_req_ready;
    wire [63:0] mem_wr_req_addr;
    wire [31:0] mem_wr_req_len;
    wire [7:0] mem_wr_req_id;
    wire  mem_wr_data_valid;
    wire  mem_wr_data_ready;
    wire [255:0] mem_wr_data;
    wire  mem_wr_data_last;
    wire  mem_wr_done_valid;
    wire [7:0] mem_wr_done_id;
    wire  mem_wr_done_error;
    wire  mem_rd_req_valid;
    wire  mem_rd_req_ready;
    wire [63:0] mem_rd_req_addr;
    wire [31:0] mem_rd_req_len;
    wire [7:0] mem_rd_req_id;
    wire  mem_rd_rsp_valid;
    wire  mem_rd_rsp_ready;
    wire [255:0] mem_rd_rsp_data;
    wire [7:0] mem_rd_rsp_id;
    wire  mem_rd_rsp_last;
    wire  mem_rd_rsp_error;
    nn_low_priority_memory_mux nn_priority(
        .clk(ddr_clk),.rst_n(ddr_rst_n),
        .vid_wr_req_valid(vid_wr_req_valid),
        .vid_wr_req_ready(vid_wr_req_ready),
        .vid_wr_req_addr(vid_wr_req_addr),
        .vid_wr_req_len(vid_wr_req_len),
        .vid_wr_req_id(vid_wr_req_id),
        .vid_wr_data_valid(vid_wr_data_valid),
        .vid_wr_data_ready(vid_wr_data_ready),
        .vid_wr_data(vid_wr_data),
        .vid_wr_data_last(vid_wr_data_last),
        .vid_wr_done_valid(vid_wr_done_valid),
        .vid_wr_done_id(vid_wr_done_id),
        .vid_wr_done_error(vid_wr_done_error),
        .vid_rd_req_valid(vid_rd_req_valid),
        .vid_rd_req_ready(vid_rd_req_ready),
        .vid_rd_req_addr(vid_rd_req_addr),
        .vid_rd_req_len(vid_rd_req_len),
        .vid_rd_req_id(vid_rd_req_id),
        .vid_rd_rsp_valid(vid_rd_rsp_valid),
        .vid_rd_rsp_ready(vid_rd_rsp_ready),
        .vid_rd_rsp_data(vid_rd_rsp_data),
        .vid_rd_rsp_id(vid_rd_rsp_id),
        .vid_rd_rsp_last(vid_rd_rsp_last),
        .vid_rd_rsp_error(vid_rd_rsp_error),
        .nn_wr_req_valid(nn_wr_req_valid),
        .nn_wr_req_ready(nn_wr_req_ready),
        .nn_wr_req_addr(nn_wr_req_addr),
        .nn_wr_req_len(nn_wr_req_len),
        .nn_wr_req_id(nn_wr_req_id),
        .nn_wr_data_valid(nn_wr_data_valid),
        .nn_wr_data_ready(nn_wr_data_ready),
        .nn_wr_data(nn_wr_data),
        .nn_wr_data_last(nn_wr_data_last),
        .nn_wr_done_valid(nn_wr_done_valid),
        .nn_wr_done_id(nn_wr_done_id),
        .nn_wr_done_error(nn_wr_done_error),
        .nn_rd_req_valid(nn_rd_req_valid),
        .nn_rd_req_ready(nn_rd_req_ready),
        .nn_rd_req_addr(nn_rd_req_addr),
        .nn_rd_req_len(nn_rd_req_len),
        .nn_rd_req_id(nn_rd_req_id),
        .nn_rd_rsp_valid(nn_rd_rsp_valid),
        .nn_rd_rsp_ready(nn_rd_rsp_ready),
        .nn_rd_rsp_data(nn_rd_rsp_data),
        .nn_rd_rsp_id(nn_rd_rsp_id),
        .nn_rd_rsp_last(nn_rd_rsp_last),
        .nn_rd_rsp_error(nn_rd_rsp_error),
        .mem_wr_req_valid(mem_wr_req_valid),
        .mem_wr_req_ready(mem_wr_req_ready),
        .mem_wr_req_addr(mem_wr_req_addr),
        .mem_wr_req_len(mem_wr_req_len),
        .mem_wr_req_id(mem_wr_req_id),
        .mem_wr_data_valid(mem_wr_data_valid),
        .mem_wr_data_ready(mem_wr_data_ready),
        .mem_wr_data(mem_wr_data),
        .mem_wr_data_last(mem_wr_data_last),
        .mem_wr_done_valid(mem_wr_done_valid),
        .mem_wr_done_id(mem_wr_done_id),
        .mem_wr_done_error(mem_wr_done_error),
        .mem_rd_req_valid(mem_rd_req_valid),
        .mem_rd_req_ready(mem_rd_req_ready),
        .mem_rd_req_addr(mem_rd_req_addr),
        .mem_rd_req_len(mem_rd_req_len),
        .mem_rd_req_id(mem_rd_req_id),
        .mem_rd_rsp_valid(mem_rd_rsp_valid),
        .mem_rd_rsp_ready(mem_rd_rsp_ready),
        .mem_rd_rsp_data(mem_rd_rsp_data),
        .mem_rd_rsp_id(mem_rd_rsp_id),
        .mem_rd_rsp_last(mem_rd_rsp_last),
        .mem_rd_rsp_error(mem_rd_rsp_error)
    );
    ddr_adapter #(
        .CTRL_ADDR_WIDTH(CTRL_ADDR_WIDTH), .ADDR_UNIT_SHIFT(ADDR_UNIT_SHIFT),
        .MEM_BEAT_BYTES(MEM_BEAT_BYTES), .DATA_WIDTH(DATA_WIDTH),
        .REQ_ID_W(REQ_ID_W), .MAX_BURST_BEATS(MAX_BURST_BEATS)
    ) adapter (
        .ddr_clk(ddr_clk), .ddr_rst_n(ddr_rst_n), .ddr_init_done(ddr_init_done),
        .mem_wr_req_valid(mem_wr_req_valid), .mem_wr_req_ready(mem_wr_req_ready),
        .mem_wr_req_addr(mem_wr_req_addr), .mem_wr_req_len(mem_wr_req_len),
        .mem_wr_req_id(mem_wr_req_id), .mem_wr_data_valid(mem_wr_data_valid),
        .mem_wr_data_ready(mem_wr_data_ready), .mem_wr_data(mem_wr_data),
        .mem_wr_data_last(mem_wr_data_last), .mem_wr_done_valid(mem_wr_done_valid),
        .mem_wr_done_id(mem_wr_done_id), .mem_wr_done_error(mem_wr_done_error),
        .mem_rd_req_valid(mem_rd_req_valid), .mem_rd_req_ready(mem_rd_req_ready),
        .mem_rd_req_addr(mem_rd_req_addr), .mem_rd_req_len(mem_rd_req_len),
        .mem_rd_req_id(mem_rd_req_id), .mem_rd_rsp_valid(mem_rd_rsp_valid),
        .mem_rd_rsp_ready(mem_rd_rsp_ready), .mem_rd_rsp_data(mem_rd_rsp_data),
        .mem_rd_rsp_id(mem_rd_rsp_id), .mem_rd_rsp_last(mem_rd_rsp_last),
        .mem_rd_rsp_error(mem_rd_rsp_error),
        .axi_awaddr(axi_awaddr), .axi_awuser_id(axi_awuser_id),
        .axi_awlen(axi_awlen), .axi_awvalid(axi_awvalid), .axi_awready(axi_awready),
        .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb), .axi_wready(axi_wready),
        .axi_wlast(axi_wlast), .axi_araddr(axi_araddr), .axi_aruser_id(axi_aruser_id),
        .axi_arlen(axi_arlen), .axi_arvalid(axi_arvalid), .axi_arready(axi_arready),
        .axi_rdata(axi_rdata), .axi_rid(axi_rid), .axi_rlast(axi_rlast),
        .axi_rvalid(axi_rvalid)
    );
endmodule
