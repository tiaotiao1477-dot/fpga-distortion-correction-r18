`timescale 1ns/1ps
// Independent low-priority NN muxes for the read and write paths.
// Downstream adapters must not return a response in the same cycle as accepting
// a request; at least one clock of request-to-response latency is required.
module nn_low_priority_memory_mux (
    input wire clk,
    input wire rst_n,

    // Video write client
    input wire vid_wr_req_valid,
    output wire vid_wr_req_ready,
    input wire [63:0] vid_wr_req_addr,
    input wire [31:0] vid_wr_req_len,
    input wire [7:0] vid_wr_req_id,
    input wire vid_wr_data_valid,
    output wire vid_wr_data_ready,
    input wire [255:0] vid_wr_data,
    input wire vid_wr_data_last,
    output wire vid_wr_done_valid,
    output wire [7:0] vid_wr_done_id,
    output wire vid_wr_done_error,

    // Neural-network write client
    input wire nn_wr_req_valid,
    output wire nn_wr_req_ready,
    input wire [63:0] nn_wr_req_addr,
    input wire [31:0] nn_wr_req_len,
    input wire [7:0] nn_wr_req_id,
    input wire nn_wr_data_valid,
    output wire nn_wr_data_ready,
    input wire [255:0] nn_wr_data,
    input wire nn_wr_data_last,
    output wire nn_wr_done_valid,
    output wire [7:0] nn_wr_done_id,
    output wire nn_wr_done_error,

    // Shared downstream write interface
    output wire mem_wr_req_valid,
    input wire mem_wr_req_ready,
    output wire [63:0] mem_wr_req_addr,
    output wire [31:0] mem_wr_req_len,
    output wire [7:0] mem_wr_req_id,
    output wire mem_wr_data_valid,
    input wire mem_wr_data_ready,
    output wire [255:0] mem_wr_data,
    output wire mem_wr_data_last,
    input wire mem_wr_done_valid,
    input wire [7:0] mem_wr_done_id,
    input wire mem_wr_done_error,

    // Video read client
    input wire vid_rd_req_valid,
    output wire vid_rd_req_ready,
    input wire [63:0] vid_rd_req_addr,
    input wire [31:0] vid_rd_req_len,
    input wire [7:0] vid_rd_req_id,
    output wire vid_rd_rsp_valid,
    input wire vid_rd_rsp_ready,
    output wire [255:0] vid_rd_rsp_data,
    output wire [7:0] vid_rd_rsp_id,
    output wire vid_rd_rsp_last,
    output wire vid_rd_rsp_error,

    // Neural-network read client
    input wire nn_rd_req_valid,
    output wire nn_rd_req_ready,
    input wire [63:0] nn_rd_req_addr,
    input wire [31:0] nn_rd_req_len,
    input wire [7:0] nn_rd_req_id,
    output wire nn_rd_rsp_valid,
    input wire nn_rd_rsp_ready,
    output wire [255:0] nn_rd_rsp_data,
    output wire [7:0] nn_rd_rsp_id,
    output wire nn_rd_rsp_last,
    output wire nn_rd_rsp_error,

    // Shared downstream read interface
    output wire mem_rd_req_valid,
    input wire mem_rd_req_ready,
    output wire [63:0] mem_rd_req_addr,
    output wire [31:0] mem_rd_req_len,
    output wire [7:0] mem_rd_req_id,
    input wire mem_rd_rsp_valid,
    output wire mem_rd_rsp_ready,
    input wire [255:0] mem_rd_rsp_data,
    input wire [7:0] mem_rd_rsp_id,
    input wire mem_rd_rsp_last,
    input wire mem_rd_rsp_error
);
    parameter [63:0] NN_BASE = 64'h03000000;
    parameter [63:0] NN_LIMIT = 64'h03400000;

    localparam [1:0] WR_IDLE  = 2'd0;
    localparam [1:0] WR_ACTIVE = 2'd1;
    localparam [1:0] WR_DRAIN = 2'd2;
    localparam [1:0] WR_LOCAL_DONE = 2'd3;
    localparam [1:0] RD_IDLE = 2'd0;
    localparam [1:0] RD_ACTIVE = 2'd1;
    localparam [1:0] RD_LOCAL_RSP = 2'd2;

    reg [1:0] wr_state;
    reg wr_owner_nn;
    reg wr_selection_held;
    reg wr_held_nn;
    reg [7:0] wr_local_done_id;

    reg [1:0] rd_state;
    reg rd_owner_nn;
    reg rd_selection_held;
    reg rd_held_nn;
    reg [7:0] rd_local_rsp_id;

    wire nn_wr_in_range = (nn_wr_req_len == 32'd32) &&
        (nn_wr_req_addr[4:0] == 5'b00000) &&
        (nn_wr_req_addr >= NN_BASE) && (nn_wr_req_addr < NN_LIMIT) &&
        ((nn_wr_req_addr + 64'd32) <= NN_LIMIT);
    wire nn_rd_in_range = (nn_rd_req_len == 32'd32) &&
        (nn_rd_req_addr[4:0] == 5'b00000) &&
        (nn_rd_req_addr >= NN_BASE) && (nn_rd_req_addr < NN_LIMIT) &&
        ((nn_rd_req_addr + 64'd32) <= NN_LIMIT);

    // Write request selection is video-first while idle. If the selected
    // downstream request stalls, retain its client until that request fires.
    wire wr_selected_nn = wr_selection_held ? wr_held_nn :
        (nn_wr_req_valid && !vid_wr_req_valid);
    wire wr_selected_valid = wr_selection_held ?
        (wr_held_nn ? nn_wr_req_valid : vid_wr_req_valid) :
        (vid_wr_req_valid || nn_wr_req_valid);
    wire wr_selected_nn_invalid = wr_selected_nn && !nn_wr_in_range;
    wire wr_mem_request_fire = mem_wr_req_valid && mem_wr_req_ready;
    wire wr_local_invalid_fire = rst_n && (wr_state == WR_IDLE) &&
        wr_selected_valid && wr_selected_nn_invalid && nn_wr_req_valid;

    assign mem_wr_req_valid = rst_n && (wr_state == WR_IDLE) &&
        wr_selected_valid && !wr_selected_nn_invalid;
    assign mem_wr_req_addr = wr_selected_nn ? nn_wr_req_addr : vid_wr_req_addr;
    assign mem_wr_req_len = wr_selected_nn ? nn_wr_req_len / 32'd32 : vid_wr_req_len;
    assign mem_wr_req_id = wr_selected_nn ? nn_wr_req_id : vid_wr_req_id;

    assign vid_wr_req_ready = rst_n && (wr_state == WR_IDLE) &&
        wr_selected_valid && !wr_selected_nn && mem_wr_req_ready;
    assign nn_wr_req_ready = rst_n && (wr_state == WR_IDLE) &&
        wr_selected_valid && wr_selected_nn &&
        (wr_selected_nn_invalid || mem_wr_req_ready);

    assign mem_wr_data_valid = rst_n && (wr_state == WR_ACTIVE) &&
        (wr_owner_nn ? nn_wr_data_valid : vid_wr_data_valid);
    assign mem_wr_data = wr_owner_nn ? nn_wr_data : vid_wr_data;
    assign mem_wr_data_last = wr_owner_nn ? nn_wr_data_last : vid_wr_data_last;
    assign vid_wr_data_ready = rst_n && (wr_state == WR_ACTIVE) &&
        !wr_owner_nn && mem_wr_data_ready;
    assign nn_wr_data_ready = rst_n &&
        (((wr_state == WR_ACTIVE) && wr_owner_nn && mem_wr_data_ready) ||
         (wr_state == WR_DRAIN));

    assign vid_wr_done_valid = rst_n && (wr_state == WR_ACTIVE) &&
        !wr_owner_nn && mem_wr_done_valid;
    assign vid_wr_done_id = vid_wr_done_valid ? mem_wr_done_id : 8'd0;
    assign vid_wr_done_error = vid_wr_done_valid && mem_wr_done_error;
    assign nn_wr_done_valid = rst_n &&
        (((wr_state == WR_ACTIVE) && wr_owner_nn && mem_wr_done_valid) ||
         (wr_state == WR_LOCAL_DONE));
    assign nn_wr_done_id = (wr_state == WR_LOCAL_DONE) ? wr_local_done_id :
        (nn_wr_done_valid ? mem_wr_done_id : 8'd0);
    assign nn_wr_done_error = (wr_state == WR_LOCAL_DONE) ? 1'b1 :
        (nn_wr_done_valid && mem_wr_done_error);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
            wr_owner_nn <= 1'b0;
            wr_selection_held <= 1'b0;
            wr_held_nn <= 1'b0;
            wr_local_done_id <= 8'd0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (wr_local_invalid_fire) begin
                        wr_local_done_id <= nn_wr_req_id;
                        wr_state <= WR_DRAIN;
                        wr_selection_held <= 1'b0;
                    end else if (wr_mem_request_fire) begin
                        wr_owner_nn <= wr_selected_nn;
                        wr_state <= WR_ACTIVE;
                        wr_selection_held <= 1'b0;
                    end else if (mem_wr_req_valid && !mem_wr_req_ready &&
                                 !wr_selection_held) begin
                        wr_selection_held <= 1'b1;
                        wr_held_nn <= wr_selected_nn;
                    end
                end
                WR_ACTIVE: begin
                    if (mem_wr_done_valid)
                        wr_state <= WR_IDLE;
                end
                WR_DRAIN: begin
                    if (nn_wr_data_valid)
                        wr_state <= WR_LOCAL_DONE;
                end
                WR_LOCAL_DONE: wr_state <= WR_IDLE;
                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    // Read request selection and ownership are independent of the write path.
    // A valid NN request is constrained to one 256-bit beat in its reserved
    // DDR window; invalid NN reads receive one local error response.
    wire rd_selected_nn = rd_selection_held ? rd_held_nn :
        (nn_rd_req_valid && !vid_rd_req_valid);
    wire rd_selected_valid = rd_selection_held ?
        (rd_held_nn ? nn_rd_req_valid : vid_rd_req_valid) :
        (vid_rd_req_valid || nn_rd_req_valid);
    wire rd_selected_nn_invalid = rd_selected_nn && !nn_rd_in_range;
    wire rd_mem_request_fire = mem_rd_req_valid && mem_rd_req_ready;
    wire rd_local_invalid_fire = rst_n && (rd_state == RD_IDLE) &&
        rd_selected_valid && rd_selected_nn_invalid && nn_rd_req_valid;
    wire rd_response_fire = mem_rd_rsp_valid && mem_rd_rsp_ready;

    assign mem_rd_req_valid = rst_n && (rd_state == RD_IDLE) &&
        rd_selected_valid && !rd_selected_nn_invalid;
    assign mem_rd_req_addr = rd_selected_nn ? nn_rd_req_addr : vid_rd_req_addr;
    assign mem_rd_req_len = rd_selected_nn ? nn_rd_req_len / 32'd32 : vid_rd_req_len;
    assign mem_rd_req_id = rd_selected_nn ? nn_rd_req_id : vid_rd_req_id;

    assign vid_rd_req_ready = rst_n && (rd_state == RD_IDLE) &&
        rd_selected_valid && !rd_selected_nn && mem_rd_req_ready;
    assign nn_rd_req_ready = rst_n && (rd_state == RD_IDLE) &&
        rd_selected_valid && rd_selected_nn &&
        (rd_selected_nn_invalid || mem_rd_req_ready);

    assign vid_rd_rsp_valid = rst_n && (rd_state == RD_ACTIVE) &&
        !rd_owner_nn && mem_rd_rsp_valid;
    assign vid_rd_rsp_data = vid_rd_rsp_valid ? mem_rd_rsp_data : 256'd0;
    assign vid_rd_rsp_id = vid_rd_rsp_valid ? mem_rd_rsp_id : 8'd0;
    assign vid_rd_rsp_last = vid_rd_rsp_valid && mem_rd_rsp_last;
    assign vid_rd_rsp_error = vid_rd_rsp_valid && mem_rd_rsp_error;

    assign nn_rd_rsp_valid = rst_n &&
        (((rd_state == RD_ACTIVE) && rd_owner_nn && mem_rd_rsp_valid) ||
         (rd_state == RD_LOCAL_RSP));
    assign nn_rd_rsp_data = (rd_state == RD_LOCAL_RSP) ? 256'd0 :
        (nn_rd_rsp_valid ? mem_rd_rsp_data : 256'd0);
    assign nn_rd_rsp_id = (rd_state == RD_LOCAL_RSP) ? rd_local_rsp_id :
        (nn_rd_rsp_valid ? mem_rd_rsp_id : 8'd0);
    assign nn_rd_rsp_last = (rd_state == RD_LOCAL_RSP) ? 1'b1 :
        (nn_rd_rsp_valid && mem_rd_rsp_last);
    assign nn_rd_rsp_error = (rd_state == RD_LOCAL_RSP) ? 1'b1 :
        (nn_rd_rsp_valid && mem_rd_rsp_error);

    assign mem_rd_rsp_ready = rst_n && (rd_state == RD_ACTIVE) &&
        (rd_owner_nn ? nn_rd_rsp_ready : vid_rd_rsp_ready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE;
            rd_owner_nn <= 1'b0;
            rd_selection_held <= 1'b0;
            rd_held_nn <= 1'b0;
            rd_local_rsp_id <= 8'd0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (rd_local_invalid_fire) begin
                        rd_local_rsp_id <= nn_rd_req_id;
                        rd_state <= RD_LOCAL_RSP;
                        rd_selection_held <= 1'b0;
                    end else if (rd_mem_request_fire) begin
                        rd_owner_nn <= rd_selected_nn;
                        rd_state <= RD_ACTIVE;
                        rd_selection_held <= 1'b0;
                    end else if (mem_rd_req_valid && !mem_rd_req_ready &&
                                 !rd_selection_held) begin
                        rd_selection_held <= 1'b1;
                        rd_held_nn <= rd_selected_nn;
                    end
                end
                RD_ACTIVE: begin
                    if (rd_response_fire && mem_rd_rsp_last)
                        rd_state <= RD_IDLE;
                end
                RD_LOCAL_RSP: begin
                    if (nn_rd_rsp_ready)
                        rd_state <= RD_IDLE;
                end
                default: rd_state <= RD_IDLE;
            endcase
        end
    end
endmodule
