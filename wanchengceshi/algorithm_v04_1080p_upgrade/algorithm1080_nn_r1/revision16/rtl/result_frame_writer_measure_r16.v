`timescale 1ns/1ps
// Store the frozen RGB888 result_px stream in a result-role frame pool.
// The pixel stream may be in a processing clock domain unrelated to DDR.
module result_frame_writer_idle_v04 #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer FRAME_ERROR_ENABLE = 0,
    parameter integer FIFO_DEPTH = 1024,
    parameter integer NUM_RESULT_SLOTS = 2,
    parameter [63:0] POOL_BASE_ADDR = 64'd0,
    parameter [63:0] USABLE_DDR_BASE_ADDR = 64'd0,
    parameter [63:0] USABLE_DDR_SIZE_BYTES = 64'd0,
    parameter integer MAX_BURST_BEATS = 16,
    parameter integer REQ_ID_W = 8,
    parameter [REQ_ID_W-1:0] WRITE_REQ_ID = 8'h01,
    parameter integer SLOT_W = (NUM_RESULT_SLOTS <= 2) ? 1 : $clog2(NUM_RESULT_SLOTS)
) (
    input wire result_clk,
    input wire result_rst_n,
    input wire result_px_valid,
    output reg result_px_ready,
    input wire [23:0] result_px_rgb,
    input wire [15:0] result_px_x,
    input wire [15:0] result_px_y,
    input wire result_px_sof,
    input wire result_px_eol,
    input wire [31:0] result_px_source_frame_id,
    input wire [15:0] result_px_cfg_version,
    input wire [31:0] result_px_pixel_tag,
    // Independent monotonic frame status in result_clk domain. Must assert
    // before the final result pixel is accepted and remain through that pixel.
    // Default-disabled for legacy producers; no metadata is used as a sentinel.
    input wire result_frame_error,

    input wire ddr_clk,
    input wire ddr_rst_n,
    input wire ddr_init_done,

    input wire reader_claim_valid,
    output wire reader_claim_ready,
    output wire reader_pending,
    output wire [SLOT_W-1:0] reader_claim_slot,
    output wire [63:0] reader_desc_base_addr,
    output wire [31:0] reader_desc_frame_id,
    output wire [31:0] reader_desc_source_frame_id,
    output wire [15:0] reader_desc_cfg_version,
    output wire [15:0] reader_desc_width,
    output wire [15:0] reader_desc_height,
    output wire [31:0] reader_desc_stride_bytes,
    output wire [7:0] reader_desc_format,
    output wire [31:0] reader_desc_process_cycles,
    output wire [2:0] reader_desc_state,
    output wire reader_desc_valid,
    input wire reader_release_valid,
    input wire [SLOT_W-1:0] reader_release_slot,
    output wire reader_release_ready,

    output wire mem_wr_req_valid,
    input wire mem_wr_req_ready,
    output wire [63:0] mem_wr_req_addr,
    output wire [31:0] mem_wr_req_len,
    output wire [REQ_ID_W-1:0] mem_wr_req_id,
    output wire mem_wr_data_valid,
    input wire mem_wr_data_ready,
    output wire [255:0] mem_wr_data,
    output wire mem_wr_data_last,
    input wire mem_wr_done_valid,
    input wire [REQ_ID_W-1:0] mem_wr_done_id,
    input wire mem_wr_done_error,

    output reg [31:0] result_commit_count,
    output reg [31:0] result_drop_count,
    output reg [31:0] fifo_stall_count,
    output wire processing_idle,
    output reg [31:0] protocol_error_count
    // R16_RESULT_MEASURE_PORTS_BEGIN
    , output wire measurement_commit_event
    , output wire [31:0] measurement_commit_source_id
    , output wire [15:0] measurement_commit_cfg
    // R16_RESULT_MEASURE_PORTS_END
);
    localparam integer ROW_BEATS = (FRAME_WIDTH + 7) / 8;
    localparam integer ROW_STRIDE_BYTES = ROW_BEATS * 32;
    localparam [63:0] FRAME_SLOT_BYTES = ROW_STRIDE_BYTES * FRAME_HEIGHT;
    // One frame's byte offset, including its completed end position. Keep
    // external addresses 64-bit while avoiding a 64-bit counter enable chain.
    localparam integer FRAME_OFFSET_W = (FRAME_SLOT_BYTES < 64'd2) ? 1 :
        $clog2(FRAME_SLOT_BYTES) +
        (((FRAME_SLOT_BYTES & (FRAME_SLOT_BYTES - 64'd1)) == 64'd0) ? 1 : 0);
    localparam integer FIFO_WIDTH = 171;
    localparam integer RESULT_CYCLE_W = 32;

    localparam [1:0] RES_IDLE = 2'd0;
    localparam [1:0] RES_WAIT_GRANT = 2'd1;
    localparam [1:0] RES_CAPTURE = 2'd2;
    localparam [1:0] RES_DROP = 2'd3;

    localparam [3:0] WR_IDLE = 4'd0;
    localparam [3:0] WR_ALLOC = 4'd1;
    localparam [3:0] WR_COLLECT = 4'd2;
    localparam [3:0] WR_REQ = 4'd3;
    localparam [3:0] WR_SEND = 4'd4;
    localparam [3:0] WR_WAIT = 4'd5;
    localparam [3:0] WR_COMMIT = 4'd6;
    localparam [3:0] WR_ABORT = 4'd7;
    localparam [3:0] WR_DRAIN = 4'd8;

    wire [31:0] layout_stride_bytes;
    wire [63:0] layout_frame_size_bytes;
    wire [63:0] layout_pool_size_bytes;
    wire [63:0] layout_pool_end_addr;
    wire layout_valid;
    wire layout_overflow;
    wire layout_slot_valid;
    wire [63:0] layout_slot_base_addr;
    storage_layout #(.NUM_SLOTS(NUM_RESULT_SLOTS), .MEM_BEAT_BYTES(32)) u_layout (
        .image_width(FRAME_WIDTH[15:0]), .image_height(FRAME_HEIGHT[15:0]),
        .usable_base_addr(USABLE_DDR_BASE_ADDR), .usable_size_bytes(USABLE_DDR_SIZE_BYTES),
        .pool_base_addr(POOL_BASE_ADDR), .slot_index({SLOT_W{1'b0}}),
        .stride_bytes(layout_stride_bytes), .frame_size_bytes(layout_frame_size_bytes),
        .pool_size_bytes(layout_pool_size_bytes), .pool_end_addr(layout_pool_end_addr),
        .layout_valid(layout_valid), .arithmetic_overflow(layout_overflow),
        .slot_base_valid(layout_slot_valid), .slot_base_addr(layout_slot_base_addr)
    );

    // Result-stream clock domain: wait for a DDR-domain slot grant before
    // accepting SOF. When the result pool is full, consume and drop the whole
    // result frame so the producer can advance to its next frame.
    reg [1:0] result_state;
    reg result_req_toggle;
    reg result_end_toggle;
    reg result_grant_sync1;
    reg result_grant_sync2;
    reg result_grant_seen;
    reg result_accept_sync1;
    reg result_accept_sync2;
    reg grant_toggle_ddr;
    reg grant_accept_ddr;
    reg [RESULT_CYCLE_W-1:0] result_cycle_count;
    wire result_grant_event = (result_grant_sync2 != result_grant_seen);
    wire result_frame_last = result_px_eol && (result_px_y == FRAME_HEIGHT - 1);
    wire result_fifo_wr_valid = result_px_valid && (result_state == RES_CAPTURE);
    wire result_fifo_wr_ready;
    wire [FIFO_WIDTH-1:0] result_fifo_wr_data = {
        (FRAME_ERROR_ENABLE != 0) && result_frame_error,
        result_cycle_count,
        result_px_sof,
        result_px_eol,
        result_px_x,
        result_px_y,
        result_px_source_frame_id,
        result_px_cfg_version,
        result_px_pixel_tag,
        result_px_rgb
    };

    always @* begin
        case (result_state)
            RES_IDLE:       result_px_ready = !result_px_sof;
            RES_WAIT_GRANT: result_px_ready = 1'b0;
            RES_CAPTURE:    result_px_ready = result_fifo_wr_ready;
            RES_DROP:       result_px_ready = 1'b1;
            default:        result_px_ready = 1'b0;
        endcase
    end

    always @(posedge result_clk or negedge result_rst_n) begin
        if (!result_rst_n) begin
            result_state <= RES_IDLE;
            result_req_toggle <= 1'b0;
            result_end_toggle <= 1'b0;
            result_grant_sync1 <= 1'b0;
            result_grant_sync2 <= 1'b0;
            result_grant_seen <= 1'b0;
            result_accept_sync1 <= 1'b0;
            result_accept_sync2 <= 1'b0;
            result_cycle_count <= 0;
            fifo_stall_count <= 0;
        end else begin
            result_grant_sync1 <= grant_toggle_ddr;
            result_grant_sync2 <= result_grant_sync1;
            result_accept_sync1 <= grant_accept_ddr;
            result_accept_sync2 <= result_accept_sync1;

            if ((result_state == RES_IDLE) && result_px_valid && result_px_sof) begin
                result_req_toggle <= ~result_req_toggle;
                result_state <= RES_WAIT_GRANT;
                result_cycle_count <= 0;
            end else if ((result_state == RES_IDLE) && result_px_valid && !result_px_sof && result_px_ready) begin
                result_state <= RES_DROP;
            end

            if (result_grant_event) begin
                result_grant_seen <= result_grant_sync2;
                if (result_state == RES_WAIT_GRANT) begin
                    if (result_accept_sync2) begin
                        result_state <= RES_CAPTURE;
                        result_cycle_count <= 0;
                    end else begin
                        result_state <= RES_DROP;
                    end
                end
            end

            if (result_state == RES_CAPTURE) begin
                if (result_px_valid && !result_fifo_wr_ready)
                    fifo_stall_count <= fifo_stall_count + 1'b1;
                if (result_px_valid && result_px_ready) begin
                    if (result_frame_last) begin
                        result_end_toggle <= ~result_end_toggle;
                        result_state <= RES_IDLE;
                    end else begin
                        result_cycle_count <= result_cycle_count + 1'b1;
                    end
                end
            end else if (result_state == RES_DROP) begin
                if (result_px_valid && result_px_ready && result_frame_last) begin
                    result_end_toggle <= ~result_end_toggle;
                    result_state <= RES_IDLE;
                end
            end
        end
    end

    wire fifo_async_rst_n = result_rst_n && ddr_rst_n;
    wire fifo_rd_valid;
    wire fifo_rd_ready;
    wire [FIFO_WIDTH-1:0] fifo_rd_data;
    wire fifo_stage_in_valid;
    wire fifo_stage_in_ready;
    wire fifo_stage_out_valid;
    wire fifo_stage_out_ready;
    wire [FIFO_WIDTH-1:0] fifo_stage_out_data;
    wire fifo_stage_clear;
    async_fifo #(.DATA_WIDTH(FIFO_WIDTH), .DEPTH(FIFO_DEPTH)) u_result_fifo (
        .wr_clk(result_clk), .rd_clk(ddr_clk), .rst_n(fifo_async_rst_n),
        .wr_valid(result_fifo_wr_valid), .wr_ready(result_fifo_wr_ready), .wr_data(result_fifo_wr_data),
        .rd_valid(fifo_rd_valid), .rd_ready(fifo_rd_ready), .rd_data(fifo_rd_data)
    );

    // DDR-domain result pool ownership and writer.
    wire writer_alloc_valid;
    wire writer_alloc_ready;
    wire [SLOT_W-1:0] writer_alloc_slot;
    wire [63:0] writer_alloc_base_addr;
    wire writer_commit_valid;
    wire writer_commit_ready;
    wire writer_abort_valid;
    wire writer_abort_ready;
    reg [SLOT_W-1:0] writer_commit_slot;
    reg [31:0] writer_commit_frame_id;
    reg [31:0] writer_commit_source_frame_id;
    reg [15:0] writer_commit_cfg_version;
    reg [15:0] writer_commit_width;
    reg [15:0] writer_commit_height;
    reg [31:0] writer_commit_stride_bytes;
    reg [7:0] writer_commit_format;
    reg [31:0] writer_commit_process_cycles;
    wire [NUM_RESULT_SLOTS*3-1:0] slot_state_debug;

    frame_pool_manager #(
        .NUM_SLOTS(NUM_RESULT_SLOTS), .ADDR_WIDTH(64), .POOL_BASE_ADDR(POOL_BASE_ADDR),
        .SLOT_STRIDE_BYTES(FRAME_SLOT_BYTES), .POOL_ROLE(1), .SLOT_W(SLOT_W)
    ) u_result_pool (
        .clk(ddr_clk), .rst_n(ddr_rst_n),
        .writer_alloc_valid(writer_alloc_valid), .writer_alloc_ready(writer_alloc_ready),
        .writer_alloc_slot(writer_alloc_slot), .writer_alloc_base_addr(writer_alloc_base_addr),
        .writer_commit_valid(writer_commit_valid), .writer_commit_ready(writer_commit_ready),
        .writer_abort_valid(writer_abort_valid), .writer_abort_ready(writer_abort_ready),
        .writer_commit_slot(writer_commit_slot), .writer_commit_frame_id(writer_commit_frame_id),
        .writer_commit_source_frame_id(writer_commit_source_frame_id),
        .writer_commit_cfg_version(writer_commit_cfg_version), .writer_commit_width(writer_commit_width),
        .writer_commit_height(writer_commit_height), .writer_commit_stride_bytes(writer_commit_stride_bytes),
        .writer_commit_format(writer_commit_format), .writer_commit_process_cycles(writer_commit_process_cycles),
        .reader_claim_valid(reader_claim_valid), .reader_claim_ready(reader_claim_ready),
        .reader_pending(reader_pending),
        .reader_claim_slot(reader_claim_slot), .reader_desc_base_addr(reader_desc_base_addr),
        .reader_desc_frame_id(reader_desc_frame_id), .reader_desc_source_frame_id(reader_desc_source_frame_id),
        .reader_desc_cfg_version(reader_desc_cfg_version), .reader_desc_width(reader_desc_width),
        .reader_desc_height(reader_desc_height), .reader_desc_stride_bytes(reader_desc_stride_bytes),
        .reader_desc_format(reader_desc_format), .reader_desc_process_cycles(reader_desc_process_cycles),
        .reader_desc_state(reader_desc_state), .reader_desc_valid(reader_desc_valid),
        .reader_release_valid(reader_release_valid), .reader_release_slot(reader_release_slot),
        .reader_release_ready(reader_release_ready), .slot_state_debug(slot_state_debug)
    );

    reg [3:0] writer_state;
    reg [1:0] req_sync;
    reg req_seen;
    reg [1:0] end_sync;
    reg end_seen;
    reg frame_owned;
    reg abort_pending;
    reg frame_end_pending;
    reg metadata_captured;
    reg [2:0] drain_empty_count;
    reg [63:0] active_base_addr;
    reg [SLOT_W-1:0] active_slot;
    reg [31:0] next_frame_id;
    reg [FRAME_OFFSET_W-1:0] bytes_written;
    reg [255:0] burst_buffer [0:MAX_BURST_BEATS-1];
    reg [4:0] burst_count;
    reg [31:0] burst_length;
    reg burst_frame_last;
    reg [4:0] send_index;
    reg packer_clear;

    wire req_event = (req_sync[1] != req_seen);
    wire end_event = (end_sync[1] != end_seen);
    wire packer_in_ready;
    wire packer_out_valid;
    wire packer_out_ready;
    wire [255:0] packer_out_data;
    wire packer_frame_last;
    wire packer_protocol_error;
    // Validate the registered pixel being consumed, not the raw async RAM
    // read port. Raw metadata comparison on packer_out_ready put FIFO address
    // decode and its high-fanout nets on the burst-buffer register CE path.
    wire [31:0] fifo_source_frame_id = fifo_stage_out_data[103:72];
    wire [15:0] fifo_cfg_version = fifo_stage_out_data[71:56];
    wire [31:0] fifo_process_cycles = fifo_stage_out_data[169:138];
    wire fifo_pixel_fire = fifo_stage_out_valid && fifo_stage_out_ready && (writer_state == WR_COLLECT);
    wire fifo_metadata_error = metadata_captured && fifo_pixel_fire &&
        ((fifo_source_frame_id != writer_commit_source_frame_id) ||
         (fifo_cfg_version != writer_commit_cfg_version));
    wire fifo_algorithm_error = fifo_pixel_fire && fifo_stage_out_data[170];
    wire packer_in_valid = fifo_stage_out_valid && (writer_state == WR_COLLECT) && frame_owned &&
                           !packer_out_valid && !fifo_metadata_error && !fifo_algorithm_error;
    wire fifo_drain_ready = (writer_state == WR_DRAIN);

    assign fifo_stage_in_valid = fifo_rd_valid && (writer_state == WR_COLLECT) &&
                                 frame_owned && !abort_pending;
    assign fifo_stage_out_ready = (writer_state == WR_COLLECT) && frame_owned &&
                                  !abort_pending && !packer_protocol_error &&
                                  !packer_out_valid && packer_in_ready;
    assign fifo_stage_clear = (writer_state == WR_ABORT) || (writer_state == WR_DRAIN) ||
                              ((writer_state == WR_COLLECT) &&
                               (packer_protocol_error || fifo_metadata_error || fifo_algorithm_error));
    result_writer_fifo_stage_v04 #(.DATA_WIDTH(FIFO_WIDTH)) u_fifo_pixel_stage (
        .clk(ddr_clk), .rst_n(ddr_rst_n), .clear(fifo_stage_clear),
        .in_valid(fifo_stage_in_valid), .in_ready(fifo_stage_in_ready), .in_data(fifo_rd_data),
        .out_valid(fifo_stage_out_valid), .out_ready(fifo_stage_out_ready),
        .out_data(fifo_stage_out_data)
    );
    assign fifo_rd_ready = fifo_drain_ready ||
        ((writer_state == WR_COLLECT) && frame_owned && !abort_pending && fifo_stage_in_ready);
    assign packer_out_ready = (writer_state == WR_COLLECT) && frame_owned &&
        !abort_pending && !packer_protocol_error && !fifo_metadata_error &&
        !fifo_algorithm_error && (burst_count < MAX_BURST_BEATS);

    result_rgb888_beat_packer #(.FRAME_WIDTH(FRAME_WIDTH), .FRAME_HEIGHT(FRAME_HEIGHT)) u_result_packer (
        .clk(ddr_clk), .rst_n(ddr_rst_n), .frame_clear(packer_clear),
        .in_valid(packer_in_valid), .in_ready(packer_in_ready),
        .in_x(fifo_stage_out_data[135:120]), .in_y(fifo_stage_out_data[119:104]),
        .in_sof(fifo_stage_out_data[137]), .in_eol(fifo_stage_out_data[136]),
        .in_pixel_rgb(fifo_stage_out_data[23:0]),
        .out_valid(packer_out_valid), .out_ready(packer_out_ready),
        .out_data(packer_out_data), .out_frame_last(packer_frame_last),
        .protocol_error(packer_protocol_error)
    );

    assign writer_alloc_valid = (writer_state == WR_ALLOC) && layout_valid && ddr_init_done;
    assign writer_commit_valid = (writer_state == WR_COMMIT);
    // R16_RESULT_MEASURE_ASSIGN_BEGIN
    assign measurement_commit_event = (writer_state == WR_COMMIT) && writer_commit_ready;
    assign measurement_commit_source_id = writer_commit_source_frame_id;
    assign measurement_commit_cfg = writer_commit_cfg_version;
    // R16_RESULT_MEASURE_ASSIGN_END
    assign writer_abort_valid = (writer_state == WR_ABORT);

    assign mem_wr_req_valid = (writer_state == WR_REQ);
    assign mem_wr_req_addr = active_base_addr + bytes_written;
    assign mem_wr_req_len = burst_length;
    assign mem_wr_req_id = WRITE_REQ_ID;
    assign mem_wr_data_valid = (writer_state == WR_SEND);
    assign mem_wr_data = burst_buffer[send_index];
    assign mem_wr_data_last = (send_index == (burst_length - 1'b1));

    // Each accepted packer beat defines all 256 bits, including row padding.
    // burst_count/length and writer_state mask uninitialized or aborted entries;
    // only the payload storage omits asynchronous reset.
    always @(posedge ddr_clk) begin
        if (ddr_rst_n && writer_state == WR_COLLECT && packer_out_valid && packer_out_ready)
            burst_buffer[burst_count] <= packer_out_data;
    end

    integer sync_i;
    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            req_sync <= 2'b00;
            req_seen <= 1'b0;
            end_sync <= 2'b00;
            end_seen <= 1'b0;
            grant_toggle_ddr <= 1'b0;
            grant_accept_ddr <= 1'b0;
            writer_state <= WR_IDLE;
            frame_owned <= 1'b0;
            abort_pending <= 1'b0;
            frame_end_pending <= 1'b0;
            metadata_captured <= 1'b0;
            drain_empty_count <= 3'd0;
            active_base_addr <= 64'd0;
            active_slot <= {SLOT_W{1'b0}};
            next_frame_id <= 32'd1;
            bytes_written <= {FRAME_OFFSET_W{1'b0}};
            burst_count <= 5'd0;
            burst_length <= 32'd0;
            burst_frame_last <= 1'b0;
            send_index <= 5'd0;
            packer_clear <= 1'b0;
            result_commit_count <= 32'd0;
            result_drop_count <= 32'd0;
            protocol_error_count <= 32'd0;
            writer_commit_slot <= {SLOT_W{1'b0}};
            writer_commit_frame_id <= 32'd0;
            writer_commit_source_frame_id <= 32'd0;
            writer_commit_cfg_version <= 16'd0;
            writer_commit_width <= FRAME_WIDTH;
            writer_commit_height <= FRAME_HEIGHT;
            writer_commit_stride_bytes <= ROW_STRIDE_BYTES;
            writer_commit_format <= 8'h01;
            writer_commit_process_cycles <= 32'd0;
        end else begin
            req_sync <= {req_sync[0], result_req_toggle};
            end_sync <= {end_sync[0], result_end_toggle};
            packer_clear <= 1'b0;

            if (end_event) begin
                end_seen <= end_sync[1];
                if (frame_owned || (writer_state == WR_DRAIN)) begin
                    frame_end_pending <= 1'b1;
                    drain_empty_count <= 3'd0;
                end
            end

            if (packer_protocol_error && frame_owned && writer_state == WR_COLLECT)
                protocol_error_count <= protocol_error_count + 1'b1;

            case (writer_state)
                WR_IDLE: begin
                    if (req_event) begin
                        req_seen <= req_sync[1];
                        if (layout_valid && ddr_init_done && writer_alloc_ready)
                            writer_state <= WR_ALLOC;
                        else begin
                            grant_accept_ddr <= 1'b0;
                            grant_toggle_ddr <= ~grant_toggle_ddr;
                            result_drop_count <= result_drop_count + 1'b1;
                        end
                    end
                end

                WR_ALLOC: begin
                    if (writer_alloc_ready && layout_valid && ddr_init_done) begin
                        active_base_addr <= writer_alloc_base_addr;
                        active_slot <= writer_alloc_slot;
                        writer_commit_slot <= writer_alloc_slot;
                        writer_commit_frame_id <= next_frame_id;
                        next_frame_id <= next_frame_id + 1'b1;
                        writer_commit_source_frame_id <= 32'd0;
                        writer_commit_cfg_version <= 16'd0;
                        writer_commit_width <= FRAME_WIDTH;
                        writer_commit_height <= FRAME_HEIGHT;
                        writer_commit_stride_bytes <= layout_stride_bytes;
                        writer_commit_format <= 8'h01;
                        writer_commit_process_cycles <= 32'd0;
                        frame_owned <= 1'b1;
                        abort_pending <= 1'b0;
                        frame_end_pending <= 1'b0;
                        metadata_captured <= 1'b0;
                        bytes_written <= {FRAME_OFFSET_W{1'b0}};
                        burst_count <= 5'd0;
                        burst_length <= 32'd0;
                        burst_frame_last <= 1'b0;
                        grant_accept_ddr <= 1'b1;
                        grant_toggle_ddr <= ~grant_toggle_ddr;
                        writer_state <= WR_COLLECT;
                    end else begin
                        grant_accept_ddr <= 1'b0;
                        grant_toggle_ddr <= ~grant_toggle_ddr;
                        result_drop_count <= result_drop_count + 1'b1;
                        writer_state <= WR_IDLE;
                    end
                end

                WR_COLLECT: begin
                    if (packer_protocol_error || fifo_metadata_error || fifo_algorithm_error) begin
                        abort_pending <= 1'b1;
                        writer_state <= WR_ABORT;
                        burst_count <= 5'd0;
                        packer_clear <= 1'b1;
                        if (fifo_metadata_error)
                            protocol_error_count <= protocol_error_count + 1'b1;
                    end else if (fifo_pixel_fire) begin
                        if (!metadata_captured) begin
                            writer_commit_source_frame_id <= fifo_source_frame_id;
                            writer_commit_cfg_version <= fifo_cfg_version;
                            metadata_captured <= 1'b1;
                        end
                        if (fifo_stage_out_data[137] && ((fifo_stage_out_data[135:120] != 0) || (fifo_stage_out_data[119:104] != 0))) begin
                            abort_pending <= 1'b1;
                            writer_state <= WR_ABORT;
                            burst_count <= 5'd0;
                            packer_clear <= 1'b1;
                            protocol_error_count <= protocol_error_count + 1'b1;
                        end
                        if (fifo_stage_out_data[136] && (fifo_stage_out_data[119:104] == FRAME_HEIGHT - 1))
                            writer_commit_process_cycles <= fifo_process_cycles;
                    end

                    if (packer_out_valid && packer_out_ready) begin
                        if ((burst_count == (MAX_BURST_BEATS - 1)) || packer_frame_last) begin
                            burst_length <= burst_count + 1'b1;
                            burst_frame_last <= packer_frame_last;
                            send_index <= 5'd0;
                            writer_state <= WR_REQ;
                        end else begin
                            burst_count <= burst_count + 1'b1;
                        end
                    end
                end

                WR_REQ: begin
                    if (mem_wr_req_ready) begin
                        send_index <= 5'd0;
                        writer_state <= WR_SEND;
                    end
                end

                WR_SEND: begin
                    if (mem_wr_data_ready) begin
                        if (send_index == (burst_length - 1'b1))
                            writer_state <= WR_WAIT;
                        else
                            send_index <= send_index + 1'b1;
                    end
                end

                WR_WAIT: begin
                    if (mem_wr_done_valid) begin
                        bytes_written <= bytes_written + (burst_length * 32);
                        burst_count <= 5'd0;
                        if ((mem_wr_done_id != WRITE_REQ_ID) || mem_wr_done_error) begin
                            abort_pending <= 1'b1;
                            writer_state <= WR_ABORT;
                            packer_clear <= 1'b1;
                            protocol_error_count <= protocol_error_count + 1'b1;
                        end else if (burst_frame_last) begin
                            writer_state <= WR_COMMIT;
                        end else begin
                            writer_state <= WR_COLLECT;
                        end
                    end
                end

                WR_COMMIT: begin
                    if (writer_commit_ready) begin
                        frame_owned <= 1'b0;
                        abort_pending <= 1'b0;
                        frame_end_pending <= 1'b0;
                        result_commit_count <= result_commit_count + 1'b1;
                        writer_state <= WR_IDLE;
                    end
                end

                WR_ABORT: begin
                    packer_clear <= 1'b1;
                    if (writer_abort_ready) begin
                        frame_owned <= 1'b0;
                        result_drop_count <= result_drop_count + 1'b1;
                        burst_count <= 5'd0;
                        drain_empty_count <= 3'd0;
                        writer_state <= WR_DRAIN;
                    end
                end

                WR_DRAIN: begin
                    packer_clear <= 1'b1;
                    if (fifo_rd_valid) begin
                        drain_empty_count <= 3'd0;
                    end else if (frame_end_pending) begin
                        if (drain_empty_count == 3'd3) begin
                            drain_empty_count <= 3'd0;
                            frame_end_pending <= 1'b0;
                            abort_pending <= 1'b0;
                            writer_state <= WR_IDLE;
                        end else begin
                            drain_empty_count <= drain_empty_count + 1'b1;
                        end
                    end
                end

                default: writer_state <= WR_IDLE;
            endcase
        end
    end
    reg [1:0] result_idle_sync;
    always @(posedge ddr_clk or negedge ddr_rst_n)
        if(!ddr_rst_n)result_idle_sync<=0;
        else result_idle_sync<={result_idle_sync[0],result_state==RES_IDLE};
    assign processing_idle=ddr_rst_n&&result_idle_sync[1]&&writer_state==WR_IDLE&&
        req_sync[1]==req_seen&&end_sync[1]==end_seen&&
        !fifo_rd_valid&&!fifo_stage_in_valid&&!fifo_stage_out_valid;

endmodule

// One-entry elastic register between the async FIFO read port and the DDR
// clock-domain pixel packer. It breaks the FIFO pointer/RAM mux path while
// preserving one-pixel-per-clock throughput and holding data under stalls.
module result_writer_fifo_stage_v04 #(
    parameter integer DATA_WIDTH = 170
) (
    input wire clk,
    input wire rst_n,
    input wire clear,
    input wire in_valid,
    output wire in_ready,
    input wire [DATA_WIDTH-1:0] in_data,
    output wire out_valid,
    input wire out_ready,
    output wire [DATA_WIDTH-1:0] out_data
);
    reg valid_reg;
    reg [DATA_WIDTH-1:0] data_reg;

    assign in_ready = !valid_reg || out_ready;
    assign out_valid = valid_reg;
    assign out_data = data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_reg <= 1'b0;
            data_reg <= {DATA_WIDTH{1'b0}};
        end else begin
            if (clear)
                valid_reg <= 1'b0;
            else if (in_ready)
                valid_reg <= in_valid;

            // Payload capture is independent of the error/flush decision.
            // Clearing valid discards a bad pixel without putting the FIFO
            // data comparator on the wide payload-register enable path.
            if (in_valid && in_ready)
                data_reg <= in_data;
        end
    end
endmodule
