`timescale 1ns/1ps
// Round-robin request/response arbiter for source fetch and result display.
// A reader owns the adapter lane through acceptance of the request's last beat.
module storage_read_arbiter #(
    parameter integer REQ_ID_W = 8,
    parameter integer DATA_WIDTH = 256
) (
    input wire ddr_clk,
    input wire ddr_rst_n,

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

    output wire mem_rd_req_valid,
    input wire mem_rd_req_ready,
    output wire [63:0] mem_rd_req_addr,
    output wire [31:0] mem_rd_req_len,
    output wire [REQ_ID_W-1:0] mem_rd_req_id,
    input wire mem_rd_rsp_valid,
    output wire mem_rd_rsp_ready,
    input wire [DATA_WIDTH-1:0] mem_rd_rsp_data,
    input wire [REQ_ID_W-1:0] mem_rd_rsp_id,
    input wire mem_rd_rsp_last,
    input wire mem_rd_rsp_error
);
    reg transaction_active;
    reg transaction_owner_display;
    reg prefer_display;
    reg selection_held;
    reg held_selection_display;

    wire live_selection_display = display_mem_rd_req_valid &&
        (!fetch_mem_rd_req_valid || prefer_display);
    wire selected_display = selection_held ? held_selection_display : live_selection_display;
    wire selected_request_valid = selection_held ?
        (held_selection_display ? display_mem_rd_req_valid : fetch_mem_rd_req_valid) :
        (fetch_mem_rd_req_valid || display_mem_rd_req_valid);
    wire request_fire = mem_rd_req_valid && mem_rd_req_ready;
    wire response_fire = mem_rd_rsp_valid && mem_rd_rsp_ready;

    assign mem_rd_req_valid = ddr_rst_n && !transaction_active && selected_request_valid;
    assign mem_rd_req_addr = selected_display ? display_mem_rd_req_addr : fetch_mem_rd_req_addr;
    assign mem_rd_req_len = selected_display ? display_mem_rd_req_len : fetch_mem_rd_req_len;
    assign mem_rd_req_id = selected_display ? display_mem_rd_req_id : fetch_mem_rd_req_id;

    assign fetch_mem_rd_req_ready = ddr_rst_n && !transaction_active &&
        !selected_display && selected_request_valid && mem_rd_req_ready;
    assign display_mem_rd_req_ready = ddr_rst_n && !transaction_active &&
        selected_display && selected_request_valid && mem_rd_req_ready;

    assign fetch_mem_rd_rsp_valid = ddr_rst_n && transaction_active &&
        !transaction_owner_display && mem_rd_rsp_valid;
    assign fetch_mem_rd_rsp_data = fetch_mem_rd_rsp_valid ? mem_rd_rsp_data : {DATA_WIDTH{1'b0}};
    assign fetch_mem_rd_rsp_id = fetch_mem_rd_rsp_valid ? mem_rd_rsp_id : {REQ_ID_W{1'b0}};
    assign fetch_mem_rd_rsp_last = fetch_mem_rd_rsp_valid && mem_rd_rsp_last;
    assign fetch_mem_rd_rsp_error = fetch_mem_rd_rsp_valid && mem_rd_rsp_error;

    assign display_mem_rd_rsp_valid = ddr_rst_n && transaction_active &&
        transaction_owner_display && mem_rd_rsp_valid;
    assign display_mem_rd_rsp_data = display_mem_rd_rsp_valid ? mem_rd_rsp_data : {DATA_WIDTH{1'b0}};
    assign display_mem_rd_rsp_id = display_mem_rd_rsp_valid ? mem_rd_rsp_id : {REQ_ID_W{1'b0}};
    assign display_mem_rd_rsp_last = display_mem_rd_rsp_valid && mem_rd_rsp_last;
    assign display_mem_rd_rsp_error = display_mem_rd_rsp_valid && mem_rd_rsp_error;

    assign mem_rd_rsp_ready = ddr_rst_n && transaction_active &&
        (transaction_owner_display ? display_mem_rd_rsp_ready : fetch_mem_rd_rsp_ready);

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            transaction_active <= 1'b0;
            transaction_owner_display <= 1'b0;
            prefer_display <= 1'b0;
            selection_held <= 1'b0;
            held_selection_display <= 1'b0;
        end else begin
            if (!transaction_active) begin
                if (request_fire) begin
                    transaction_active <= 1'b1;
                    transaction_owner_display <= selected_display;
                    prefer_display <= !selected_display;
                    selection_held <= 1'b0;
                end else if (mem_rd_req_valid && !mem_rd_req_ready && !selection_held) begin
                    // Keep the request owner stable if the other client starts
                    // requesting while the adapter applies backpressure.
                    selection_held <= 1'b1;
                    held_selection_display <= selected_display;
                end
            end else if (response_fire && mem_rd_rsp_last) begin
                transaction_active <= 1'b0;
            end
        end
    end
endmodule
