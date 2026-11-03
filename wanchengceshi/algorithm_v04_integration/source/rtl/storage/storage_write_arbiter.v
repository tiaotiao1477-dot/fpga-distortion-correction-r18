`timescale 1ns/1ps
// Round-robin ingress arbiter for the source and result frame writers.
// Ownership is held from one accepted client request through its completion.
module storage_write_arbiter #(
    parameter integer REQ_ID_W = 8,
    parameter integer DATA_WIDTH = 256
) (
    input wire ddr_clk,
    input wire ddr_rst_n,

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

    output wire mem_wr_req_valid,
    input wire mem_wr_req_ready,
    output wire [63:0] mem_wr_req_addr,
    output wire [31:0] mem_wr_req_len,
    output wire [REQ_ID_W-1:0] mem_wr_req_id,
    output wire mem_wr_data_valid,
    input wire mem_wr_data_ready,
    output wire [DATA_WIDTH-1:0] mem_wr_data,
    output wire mem_wr_data_last,
    input wire mem_wr_done_valid,
    input wire [REQ_ID_W-1:0] mem_wr_done_id,
    input wire mem_wr_done_error
);
    reg transaction_active;
    reg transaction_owner_result;
    reg prefer_result;
    reg selection_held;
    reg held_selection_result;

    wire live_selection_result = res_mem_wr_req_valid &&
        (!src_mem_wr_req_valid || prefer_result);
    wire selected_result = selection_held ? held_selection_result : live_selection_result;
    wire selected_request_valid = selection_held ?
        (held_selection_result ? res_mem_wr_req_valid : src_mem_wr_req_valid) :
        (src_mem_wr_req_valid || res_mem_wr_req_valid);
    wire request_fire = mem_wr_req_valid && mem_wr_req_ready;

    assign mem_wr_req_valid = ddr_rst_n && !transaction_active && selected_request_valid;
    assign mem_wr_req_addr = selected_result ? res_mem_wr_req_addr : src_mem_wr_req_addr;
    assign mem_wr_req_len = selected_result ? res_mem_wr_req_len : src_mem_wr_req_len;
    assign mem_wr_req_id = selected_result ? res_mem_wr_req_id : src_mem_wr_req_id;

    assign src_mem_wr_req_ready = ddr_rst_n && !transaction_active &&
        !selected_result && selected_request_valid && mem_wr_req_ready;
    assign res_mem_wr_req_ready = ddr_rst_n && !transaction_active &&
        selected_result && selected_request_valid && mem_wr_req_ready;

    assign mem_wr_data_valid = ddr_rst_n && transaction_active &&
        (transaction_owner_result ? res_mem_wr_data_valid : src_mem_wr_data_valid);
    assign mem_wr_data = transaction_owner_result ? res_mem_wr_data : src_mem_wr_data;
    assign mem_wr_data_last = transaction_owner_result ? res_mem_wr_data_last : src_mem_wr_data_last;
    assign src_mem_wr_data_ready = ddr_rst_n && transaction_active &&
        !transaction_owner_result && mem_wr_data_ready;
    assign res_mem_wr_data_ready = ddr_rst_n && transaction_active &&
        transaction_owner_result && mem_wr_data_ready;

    assign src_mem_wr_done_valid = ddr_rst_n && transaction_active &&
        !transaction_owner_result && mem_wr_done_valid;
    assign src_mem_wr_done_id = src_mem_wr_done_valid ? mem_wr_done_id : {REQ_ID_W{1'b0}};
    assign src_mem_wr_done_error = src_mem_wr_done_valid && mem_wr_done_error;
    assign res_mem_wr_done_valid = ddr_rst_n && transaction_active &&
        transaction_owner_result && mem_wr_done_valid;
    assign res_mem_wr_done_id = res_mem_wr_done_valid ? mem_wr_done_id : {REQ_ID_W{1'b0}};
    assign res_mem_wr_done_error = res_mem_wr_done_valid && mem_wr_done_error;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            transaction_active <= 1'b0;
            transaction_owner_result <= 1'b0;
            prefer_result <= 1'b0;
            selection_held <= 1'b0;
            held_selection_result <= 1'b0;
        end else begin
            if (!transaction_active) begin
                if (request_fire) begin
                    transaction_active <= 1'b1;
                    transaction_owner_result <= selected_result;
                    prefer_result <= !selected_result;
                    selection_held <= 1'b0;
                end else if (mem_wr_req_valid && !mem_wr_req_ready && !selection_held) begin
                    // Keep the selected client stable if the other client
                    // becomes valid while this request is stalled.
                    selection_held <= 1'b1;
                    held_selection_result <= selected_result;
                end
            end else if (mem_wr_done_valid) begin
                transaction_active <= 1'b0;
            end
        end
    end
endmodule
