`timescale 1ns/1ps
// Absorb a complete display read burst at DDR beat speed. The arbiter releases
// its transaction on accepted LAST; pixel unpacking can then stall independently.
// Shared coherent reset required. Status bits are queued with their own beat.
module display_read_response_buffer #(
    parameter integer DATA_WIDTH=256,
    parameter integer REQ_ID_W=8,
    parameter integer DEPTH=32
)(
    input wire clk,rst_n,
    input wire in_valid,
    output wire in_ready,
    input wire [DATA_WIDTH-1:0] in_data,
    input wire [REQ_ID_W-1:0] in_id,
    input wire in_last,in_error,
    output wire out_valid,
    input wire out_ready,
    output wire [DATA_WIDTH-1:0] out_data,
    output wire [REQ_ID_W-1:0] out_id,
    output wire out_last,out_error
);
    localparam WIDTH=DATA_WIDTH+REQ_ID_W+2;
    wire wr_ready,rd_valid;
    wire [WIDTH-1:0] queued;
    async_fifo #(.DATA_WIDTH(WIDTH),.DEPTH(DEPTH)) u_fifo(
        .wr_clk(clk),.rd_clk(clk),.rst_n(rst_n),
        .wr_valid(in_valid&&rst_n),.wr_ready(wr_ready),.wr_data({in_error,in_last,in_id,in_data}),
        .rd_valid(rd_valid),.rd_ready(out_ready&&rst_n),.rd_data(queued));
    assign in_ready=rst_n&&wr_ready;
    assign out_valid=rst_n&&rd_valid;
    assign {out_error,out_last,out_id,out_data}=queued;
endmodule
