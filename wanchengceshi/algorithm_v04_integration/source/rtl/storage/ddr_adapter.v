`timescale 1ns/1ps
// Byte-addressed, valid/ready client adapter for the Pango DDR3 user port.
// The native port has no WVALID, RREADY, BRESP, or RRESP: writes are buffered
// one complete burst at a time and reads are buffered one non-backpressurable
// burst at a time. Read and write lanes each allow one outstanding request.
module ddr_adapter #(
    parameter integer CTRL_ADDR_WIDTH = 28,
    parameter integer ADDR_UNIT_SHIFT = 2,
    parameter integer MEM_BEAT_BYTES = 32,
    parameter integer DATA_WIDTH = 256,
    parameter integer REQ_ID_W = 8,
    parameter integer MAX_BURST_BEATS = 16,
    parameter integer BEAT_SHIFT = $clog2(MEM_BEAT_BYTES)
) (
    input wire ddr_clk,
    input wire ddr_rst_n,
    input wire ddr_init_done,

    input wire mem_wr_req_valid,
    output wire mem_wr_req_ready,
    input wire [63:0] mem_wr_req_addr,
    input wire [31:0] mem_wr_req_len,
    input wire [REQ_ID_W-1:0] mem_wr_req_id,
    input wire mem_wr_data_valid,
    output wire mem_wr_data_ready,
    input wire [DATA_WIDTH-1:0] mem_wr_data,
    input wire mem_wr_data_last,
    output wire mem_wr_done_valid,
    output wire [REQ_ID_W-1:0] mem_wr_done_id,
    output wire mem_wr_done_error,

    input wire mem_rd_req_valid,
    output wire mem_rd_req_ready,
    input wire [63:0] mem_rd_req_addr,
    input wire [31:0] mem_rd_req_len,
    input wire [REQ_ID_W-1:0] mem_rd_req_id,
    output wire mem_rd_rsp_valid,
    input wire mem_rd_rsp_ready,
    output wire [DATA_WIDTH-1:0] mem_rd_rsp_data,
    output wire [REQ_ID_W-1:0] mem_rd_rsp_id,
    output wire mem_rd_rsp_last,
    output wire mem_rd_rsp_error,

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
    localparam [2:0] W_IDLE = 3'd0;
    localparam [2:0] W_FILL = 3'd1;
    localparam [2:0] W_ADDR = 3'd2;
    localparam [2:0] W_DATA = 3'd3;
    localparam [2:0] W_DONE = 3'd4;
    localparam [2:0] W_CHECK = 3'd5;
    localparam [2:0] R_IDLE = 3'd0;
    localparam [2:0] R_ADDR = 3'd1;
    localparam [2:0] R_DATA = 3'd2;
    localparam [2:0] R_DRAIN = 3'd3;
    localparam [2:0] R_INVALID = 3'd4;
    localparam [2:0] R_CHECK = 3'd5;
    localparam [63:0] CTRL_ADDR_SPACE_BYTES = 64'd1 << (CTRL_ADDR_WIDTH + ADDR_UNIT_SHIFT);

    reg [2:0] wr_state;
    reg [63:0] wr_active_addr;
    reg [31:0] wr_remaining;
    reg [REQ_ID_W-1:0] wr_active_id;
    reg [4:0] wr_burst_beats;
    reg [4:0] wr_fill_count;
    reg [4:0] wr_send_count;
    reg wr_error;
    reg wr_done_error;
    reg [DATA_WIDTH-1:0] wr_buffer [0:MAX_BURST_BEATS-1];

    reg [2:0] rd_state;
    reg [63:0] rd_active_addr;
    reg [31:0] rd_remaining;
    reg [REQ_ID_W-1:0] rd_active_id;
    reg [4:0] rd_burst_beats;
    reg [4:0] rd_capture_count;
    reg [DATA_WIDTH-1:0] rd_fifo_data [0:MAX_BURST_BEATS-1];
    reg rd_fifo_last [0:MAX_BURST_BEATS-1];
    reg rd_fifo_error [0:MAX_BURST_BEATS-1];
    reg [3:0] rd_fifo_write_ptr;
    reg [3:0] rd_fifo_read_ptr;
    reg [4:0] rd_fifo_count;

    function request_address_valid;
        input [63:0] byte_addr;
        input [31:0] beat_len;
        reg [63:0] request_bytes;
        reg [63:0] end_addr;
        begin
            request_bytes = {32'd0, beat_len} * MEM_BEAT_BYTES;
            end_addr = byte_addr + request_bytes;
            request_address_valid = (MEM_BEAT_BYTES > 0) &&
                ((MEM_BEAT_BYTES & (MEM_BEAT_BYTES - 1)) == 0) &&
                (beat_len != 32'd0) &&
                ((byte_addr & (MEM_BEAT_BYTES - 1)) == 64'd0) &&
                (end_addr >= byte_addr) && (end_addr <= CTRL_ADDR_SPACE_BYTES);
        end
    endfunction

    function [4:0] first_burst_size;
        input [31:0] beats_left;
        begin
            if (beats_left >= MAX_BURST_BEATS)
                first_burst_size = MAX_BURST_BEATS;
            else
                first_burst_size = beats_left[4:0];
        end
    endfunction

    wire wr_req_fire = mem_wr_req_valid && mem_wr_req_ready;
    wire rd_req_fire = mem_rd_req_valid && mem_rd_req_ready;
    wire wr_input_fire = mem_wr_data_valid && mem_wr_data_ready;
    wire wr_data_last_expected = (wr_remaining == wr_burst_beats) &&
                                 (wr_fill_count == (wr_burst_beats - 1'b1));
    wire wr_axi_last_expected = (wr_send_count == (wr_burst_beats - 1'b1));
    wire wr_axi_last_mismatch = (axi_wlast != wr_axi_last_expected);
    wire rd_fifo_push = (rd_state == R_DATA) && axi_rvalid;
    wire rd_fifo_pop = (rd_state != R_INVALID) && mem_rd_rsp_valid && mem_rd_rsp_ready;
    wire rd_last_expected = (rd_capture_count == (rd_burst_beats - 1'b1));
    // Validate only captured requests. The 64-bit aperture/overflow check
    // must not follow the client arbitration mux in the acceptance cycle.
    wire rd_req_valid_range = request_address_valid(rd_active_addr, rd_remaining);
    wire wr_req_valid_range = request_address_valid(wr_active_addr, wr_remaining);

    assign mem_wr_req_ready = ddr_init_done && (wr_state == W_IDLE);
    assign mem_wr_data_ready = (wr_state == W_FILL) && (wr_fill_count < wr_burst_beats);
    assign mem_wr_done_valid = (wr_state == W_DONE);
    assign mem_wr_done_id = wr_active_id;
    assign mem_wr_done_error = wr_done_error;

    assign axi_awaddr = wr_active_addr >> ADDR_UNIT_SHIFT;
    assign axi_awuser_id = wr_active_id[3:0];
    assign axi_awlen = wr_burst_beats - 1'b1;
    assign axi_awvalid = (wr_state == W_ADDR);
    assign axi_wdata = (wr_state == W_DATA) ? wr_buffer[wr_send_count] : {DATA_WIDTH{1'b0}};
    assign axi_wstrb = {(DATA_WIDTH/8){1'b1}};

    assign mem_rd_req_ready = ddr_init_done && (rd_state == R_IDLE);
    assign mem_rd_rsp_valid = (rd_state == R_INVALID) || (rd_fifo_count != 0);
    assign mem_rd_rsp_data = (rd_state == R_INVALID) ? {DATA_WIDTH{1'b0}} : rd_fifo_data[rd_fifo_read_ptr];
    assign mem_rd_rsp_id = rd_active_id;
    assign mem_rd_rsp_last = (rd_state == R_INVALID) ? 1'b1 : rd_fifo_last[rd_fifo_read_ptr];
    assign mem_rd_rsp_error = (rd_state == R_INVALID) ? 1'b1 : rd_fifo_error[rd_fifo_read_ptr];

    assign axi_araddr = rd_active_addr >> ADDR_UNIT_SHIFT;
    assign axi_aruser_id = rd_active_id[3:0];
    assign axi_arlen = rd_burst_beats - 1'b1;
    assign axi_arvalid = (rd_state == R_ADDR);

    // Control reset masks the payload. W_ADDR/W_DATA are reachable only after
    // all beats of the current sub-burst have been refilled in W_FILL.
    // Keep wide payload storage off the asynchronous reset distribution chain.
    always @(posedge ddr_clk) begin
        if (wr_input_fire)
            wr_buffer[wr_fill_count] <= mem_wr_data;
    end

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            wr_state <= W_IDLE;
            wr_active_addr <= 64'd0;
            wr_remaining <= 32'd0;
            wr_active_id <= {REQ_ID_W{1'b0}};
            wr_burst_beats <= 5'd0;
            wr_fill_count <= 5'd0;
            wr_send_count <= 5'd0;
            wr_error <= 1'b0;
            wr_done_error <= 1'b0;
        end else begin
            case (wr_state)
                W_IDLE: begin
                    if (wr_req_fire) begin
                        wr_active_addr <= mem_wr_req_addr;
                        wr_active_id <= mem_wr_req_id;
                        wr_remaining <= mem_wr_req_len;
                        wr_error <= 1'b0;
                        wr_fill_count <= 5'd0;
                        wr_send_count <= 5'd0;
                        wr_state <= W_CHECK;
                    end
                end

                W_CHECK: begin
                        if (!wr_req_valid_range) begin
                            wr_done_error <= 1'b1;
                            wr_state <= W_DONE;
                        end else begin
                            wr_done_error <= 1'b0;
                            wr_burst_beats <= first_burst_size(wr_remaining);
                            wr_state <= W_FILL;
                        end
                end

                W_FILL: begin
                    if (wr_input_fire) begin
                        if (mem_wr_data_last != wr_data_last_expected)
                            wr_error <= 1'b1;
                        if (wr_fill_count == (wr_burst_beats - 1'b1)) begin
                            wr_fill_count <= 5'd0;
                            wr_state <= W_ADDR;
                        end else begin
                            wr_fill_count <= wr_fill_count + 1'b1;
                        end
                    end
                end

                W_ADDR: begin
                    if (axi_awvalid && axi_awready) begin
                        wr_send_count <= 5'd0;
                        wr_state <= W_DATA;
                    end
                end

                W_DATA: begin
                    if (axi_wready) begin
                        if (wr_axi_last_mismatch)
                            wr_error <= 1'b1;
                        if (wr_send_count == (wr_burst_beats - 1'b1)) begin
                            if (wr_remaining == wr_burst_beats) begin
                                wr_done_error <= wr_error | wr_axi_last_mismatch;
                                wr_state <= W_DONE;
                            end else begin
                                wr_remaining <= wr_remaining - wr_burst_beats;
                                wr_active_addr <= wr_active_addr + (wr_burst_beats * MEM_BEAT_BYTES);
                                wr_burst_beats <= first_burst_size(wr_remaining - wr_burst_beats);
                                wr_fill_count <= 5'd0;
                                wr_send_count <= 5'd0;
                                wr_state <= W_FILL;
                            end
                        end else begin
                            wr_send_count <= wr_send_count + 1'b1;
                        end
                    end
                end

                W_DONE: wr_state <= W_IDLE;
                default: wr_state <= W_IDLE;
            endcase
        end
    end

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            rd_state <= R_IDLE;
            rd_active_addr <= 64'd0;
            rd_remaining <= 32'd0;
            rd_active_id <= {REQ_ID_W{1'b0}};
            rd_burst_beats <= 5'd0;
            rd_capture_count <= 5'd0;
        end else begin
            case (rd_state)
                R_IDLE: begin
                    if (rd_req_fire) begin
                        rd_active_addr <= mem_rd_req_addr;
                        rd_active_id <= mem_rd_req_id;
                        rd_remaining <= mem_rd_req_len;
                        rd_capture_count <= 5'd0;
                        rd_state <= R_CHECK;
                    end
                end

                R_CHECK: begin
                        if (!rd_req_valid_range) begin
                            rd_state <= R_INVALID;
                        end else begin
                            rd_burst_beats <= first_burst_size(rd_remaining);
                            rd_state <= R_ADDR;
                        end
                end

                R_ADDR: begin
                    if (axi_arvalid && axi_arready) begin
                        rd_capture_count <= 5'd0;
                        rd_state <= R_DATA;
                    end
                end

                R_DATA: begin
                    if (axi_rvalid) begin
                        rd_capture_count <= rd_capture_count + 1'b1;
                        if (rd_capture_count == (rd_burst_beats - 1'b1))
                            rd_state <= R_DRAIN;
                    end
                end

                R_DRAIN: begin
                    if ((rd_fifo_count == 0) && (rd_capture_count == rd_burst_beats)) begin
                        if (rd_remaining == rd_burst_beats) begin
                            rd_state <= R_IDLE;
                        end else begin
                            rd_remaining <= rd_remaining - rd_burst_beats;
                            rd_active_addr <= rd_active_addr + (rd_burst_beats * MEM_BEAT_BYTES);
                            rd_burst_beats <= first_burst_size(rd_remaining - rd_burst_beats);
                            rd_capture_count <= 5'd0;
                            rd_state <= R_ADDR;
                        end
                    end
                end

                R_INVALID: begin
                    if (mem_rd_rsp_ready)
                        rd_state <= R_IDLE;
                end

                default: rd_state <= R_IDLE;
            endcase
        end
    end

    integer rd_reset_i;
    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            rd_fifo_write_ptr <= 4'd0;
            rd_fifo_read_ptr <= 4'd0;
            rd_fifo_count <= 5'd0;
            for (rd_reset_i = 0; rd_reset_i < MAX_BURST_BEATS; rd_reset_i = rd_reset_i + 1) begin
                rd_fifo_data[rd_reset_i] <= {DATA_WIDTH{1'b0}};
                rd_fifo_last[rd_reset_i] <= 1'b0;
                rd_fifo_error[rd_reset_i] <= 1'b0;
            end
        end else begin
            // Request acceptance only occurs in R_IDLE after the previous
            // burst has drained. Reset FIFO control state for every new
            // request, then keep the wide data RAM write enable independent
            // from the address-range comparator on the request path.
            if (rd_req_fire) begin
                rd_fifo_write_ptr <= 4'd0;
                rd_fifo_read_ptr <= 4'd0;
                rd_fifo_count <= 5'd0;
            end else begin
                if (rd_fifo_push) begin
                    rd_fifo_write_ptr <= rd_fifo_write_ptr + 1'b1;
                end
                if (rd_fifo_pop)
                    rd_fifo_read_ptr <= rd_fifo_read_ptr + 1'b1;

                case ({rd_fifo_push, rd_fifo_pop})
                    2'b10: rd_fifo_count <= rd_fifo_count + 1'b1;
                    2'b01: rd_fifo_count <= rd_fifo_count - 1'b1;
                    default: rd_fifo_count <= rd_fifo_count;
                endcase
            end
            if (rd_fifo_push) begin
                rd_fifo_data[rd_fifo_write_ptr] <= axi_rdata;
                rd_fifo_last[rd_fifo_write_ptr] <= (rd_remaining == rd_burst_beats) && rd_last_expected;
                rd_fifo_error[rd_fifo_write_ptr] <=
                    (axi_rid != rd_active_id[3:0]) || (axi_rlast != rd_last_expected);
            end
        end
    end
endmodule
