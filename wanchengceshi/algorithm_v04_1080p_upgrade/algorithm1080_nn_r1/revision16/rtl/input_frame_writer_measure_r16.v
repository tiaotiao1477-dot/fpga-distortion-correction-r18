`timescale 1ns/1ps
// Parameterized camera-to-DDR source-frame writer.
// A frame-slot request crosses from cam_clk to ddr_clk before any pixel is
// accepted. Tagged RGB565 (default) or RGB888 pixels cross an async FIFO, packed into
// RGB888 32-bit words, written as row-aligned 256-bit DDR beats, and committed
// to the source pool only after the final burst completes.
module input_frame_writer #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer FIFO_DEPTH = 1024,
    parameter integer CAMERA_RGB888_ENABLE = 0,
    parameter integer NUM_SOURCE_SLOTS = 3,
    parameter [63:0] POOL_BASE_ADDR = 64'd0,
    parameter [63:0] USABLE_DDR_BASE_ADDR = 64'd0,
    parameter [63:0] USABLE_DDR_SIZE_BYTES = 64'd0,
    parameter integer MAX_BURST_BEATS = 16,
    parameter integer REQ_ID_W = 8,
    parameter [REQ_ID_W-1:0] WRITE_REQ_ID = 8'h01,
    parameter integer SLOT_W = (NUM_SOURCE_SLOTS <= 2) ? 1 : $clog2(NUM_SOURCE_SLOTS)
) (
    input wire cam_clk,
    input wire cam_rst_n,
    input wire camera_frame_active,
    input wire camera_pixel_valid,
    input wire [15:0] camera_pixel_rgb565,
    input wire [23:0] camera_pixel_rgb888,

    input wire ddr_clk,
    input wire ddr_rst_n,
    input wire ddr_init_done,
    input wire [15:0] cfg_version,

    input wire reader_claim_valid,
    output wire reader_claim_ready,
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

    output reg [31:0] frame_commit_count,
    output reg [31:0] frame_drop_count,
    output reg [31:0] fifo_overflow_count,
    // R7 observer-only: {late-grant,excess,short,not-ready}, 8-bit modular counts in cam_clk.
    output reg [31:0] debug_abort_counts,
    output wire [3:0] debug_writer_state,
    output wire debug_writer_alloc_ready,
    output wire debug_frame_owned,
    output wire [NUM_SOURCE_SLOTS*3-1:0] debug_slot_state
    // R16_SOURCE_MEASURE_PORTS_BEGIN
    , input wire measurement_camera_time_valid
    , input wire [63:0] measurement_camera_t0
    , input wire [31:0] measurement_camera_sensor_id
    , output wire measurement_source_event
    , output wire [31:0] measurement_source_id
    , output wire [31:0] measurement_source_sensor_id
    , output wire [63:0] measurement_source_t0
    , output wire measurement_source_time_valid
    // R16_SOURCE_MEASURE_PORTS_END
);
    localparam integer ROW_BEATS = (FRAME_WIDTH + 7) / 8;
    localparam integer ROW_STRIDE_BYTES = ROW_BEATS * 32;
    localparam [63:0] FRAME_SLOT_BYTES = ROW_STRIDE_BYTES * FRAME_HEIGHT;
    localparam [31:0] FRAME_PIXEL_COUNT = FRAME_WIDTH * FRAME_HEIGHT;
    localparam integer CAMERA_COLOR_BITS = CAMERA_RGB888_ENABLE ? 24 : 16;
    localparam integer CAMERA_FIFO_BITS = CAMERA_COLOR_BITS + 3;

    localparam [2:0] CAM_IDLE = 3'd0;
    localparam [2:0] CAM_WAIT_GRANT = 3'd1;
    localparam [2:0] CAM_CAPTURE = 3'd2;
    localparam [2:0] CAM_DROP = 3'd3;

    localparam [3:0] WR_IDLE = 4'd0;
    localparam [3:0] WR_ALLOC = 4'd1;
    localparam [3:0] WR_COLLECT = 4'd2;
    localparam [3:0] WR_REQ = 4'd3;
    localparam [3:0] WR_SEND = 4'd4;
    localparam [3:0] WR_WAIT = 4'd5;
    localparam [3:0] WR_COMMIT = 4'd6;
    localparam [3:0] WR_ABORT = 4'd7;
    localparam [3:0] WR_DRAIN = 4'd8;

    // Layout checks use the same byte/beat contract as the writer address math.
    wire [31:0] layout_stride_bytes;
    wire [63:0] layout_frame_size_bytes;
    wire [63:0] layout_pool_size_bytes;
    wire [63:0] layout_pool_end_addr;
    wire layout_valid;
    wire layout_overflow;
    wire layout_slot_valid;
    wire [63:0] layout_slot_base_addr;
    storage_layout #(.NUM_SLOTS(NUM_SOURCE_SLOTS), .MEM_BEAT_BYTES(32)) u_layout (
        .image_width(FRAME_WIDTH[15:0]), .image_height(FRAME_HEIGHT[15:0]),
        .usable_base_addr(USABLE_DDR_BASE_ADDR), .usable_size_bytes(USABLE_DDR_SIZE_BYTES),
        .pool_base_addr(POOL_BASE_ADDR), .slot_index({SLOT_W{1'b0}}),
        .stride_bytes(layout_stride_bytes), .frame_size_bytes(layout_frame_size_bytes),
        .pool_size_bytes(layout_pool_size_bytes), .pool_end_addr(layout_pool_end_addr),
        .layout_valid(layout_valid), .arithmetic_overflow(layout_overflow),
        .slot_base_valid(layout_slot_valid), .slot_base_addr(layout_slot_base_addr)
    );

    // Camera-domain slot request and pixel framing.
    reg [2:0] cam_state;
    reg cam_frame_active_d;
    reg cam_req_toggle;
    reg cam_grant_sync1;
    reg cam_grant_sync2;
    // Delay the notification one cycle beyond the independently synchronized
    // accept bit, so a one-cycle CDC skew cannot turn an allocation into DROP.
    reg cam_grant_sync3;
    reg cam_grant_seen;
    reg cam_grant_accept_sync1;
    reg cam_grant_accept_sync2;
    reg cam_abort_toggle;
    reg cam_end_toggle;
    reg [31:0] cam_pixel_count;
    reg cam_early_pixel;
    reg cam_abort_sent;
    reg cam_ended_wait;

    wire cam_frame_rise = camera_frame_active && !cam_frame_active_d;
    // R16_SOURCE_MEASURE_DECL_BEGIN
    reg measurement_cam_time_valid_held;
    reg [63:0] measurement_cam_t0_held;
    reg [31:0] measurement_cam_sensor_id_held;
    // R16_SOURCE_MEASURE_DECL_END
    wire cam_frame_fall = !camera_frame_active && cam_frame_active_d;
    wire cam_grant_event = (cam_grant_sync3 != cam_grant_seen);
    wire cam_grant_now = (cam_state == CAM_WAIT_GRANT) && cam_grant_event &&
                         cam_grant_accept_sync2 && !cam_early_pixel && camera_frame_active;
    wire cam_can_capture = (cam_state == CAM_CAPTURE) || cam_grant_now;

    wire [CAMERA_COLOR_BITS-1:0] camera_color;
    generate
        if (CAMERA_RGB888_ENABLE) begin : g_camera_rgb888
            assign camera_color = camera_pixel_rgb888;
        end else begin : g_camera_rgb565
            assign camera_color = camera_pixel_rgb565;
        end
    endgenerate
    wire cam_pixel_candidate = camera_frame_active && camera_pixel_valid && cam_can_capture &&
                               (cam_pixel_count < FRAME_PIXEL_COUNT);
    wire [CAMERA_FIFO_BITS-1:0] fifo_wr_data = {
        (cam_pixel_count == 0),
        ((cam_pixel_count % FRAME_WIDTH) == (FRAME_WIDTH - 1)),
        (cam_pixel_count == (FRAME_PIXEL_COUNT - 1)),
        camera_color
    };
    wire fifo_wr_valid = cam_pixel_candidate;
    wire fifo_wr_ready;

    // Toggle grant is held stable long enough for a two-flop camera-clock crossing.
    reg grant_toggle_ddr;
    reg grant_accept_ddr;
    // R16_SOURCE_MEASURE_CAPTURE_BEGIN
    always @(posedge cam_clk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            measurement_cam_time_valid_held <= 1'b0;
            measurement_cam_t0_held <= 64'd0;
            measurement_cam_sensor_id_held <= 32'd0;
        end else if (cam_frame_rise && (cam_state == CAM_IDLE)) begin
            measurement_cam_time_valid_held <= measurement_camera_time_valid;
            measurement_cam_t0_held <= measurement_camera_t0;
            measurement_cam_sensor_id_held <= measurement_camera_sensor_id;
        end
    end
    // R16_SOURCE_MEASURE_CAPTURE_END
    always @(posedge cam_clk or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            cam_state <= CAM_IDLE;
            cam_frame_active_d <= 1'b0;
            cam_req_toggle <= 1'b0;
            cam_grant_sync1 <= 1'b0;
            cam_grant_sync2 <= 1'b0;
            cam_grant_sync3 <= 1'b0;
            cam_grant_seen <= 1'b0;
            cam_grant_accept_sync1 <= 1'b0;
            cam_grant_accept_sync2 <= 1'b0;
            cam_abort_toggle <= 1'b0;
            debug_abort_counts <= 32'd0; // R7_OBSERVER
            cam_end_toggle <= 1'b0;
            cam_pixel_count <= 32'd0;
            cam_early_pixel <= 1'b0;
            cam_abort_sent <= 1'b0;
            cam_ended_wait <= 1'b0;
        end else begin
            cam_frame_active_d <= camera_frame_active;
            cam_grant_sync1 <= grant_toggle_ddr;
            cam_grant_sync2 <= cam_grant_sync1;
            cam_grant_sync3 <= cam_grant_sync2;
            cam_grant_accept_sync1 <= grant_accept_ddr;
            cam_grant_accept_sync2 <= cam_grant_accept_sync1;

            if (cam_frame_rise && (cam_state == CAM_IDLE)) begin
                cam_req_toggle <= ~cam_req_toggle;
                cam_state <= CAM_WAIT_GRANT;
                cam_pixel_count <= 32'd0;
                cam_early_pixel <= 1'b0;
                cam_abort_sent <= 1'b0;
                cam_ended_wait <= 1'b0;
            end

            if (cam_grant_event) begin
                cam_grant_seen <= cam_grant_sync3;
                if (cam_state == CAM_WAIT_GRANT) begin
                    if (!cam_grant_accept_sync2) begin
                        cam_state <= CAM_DROP;
                    end else if (!camera_frame_active || cam_early_pixel) begin
                        cam_state <= CAM_DROP;
                        if (!cam_abort_sent) begin
                            cam_abort_toggle <= ~cam_abort_toggle;
                        debug_abort_counts[31:24] <= debug_abort_counts[31:24] + 8'd1; // R7_OBSERVER
                            cam_abort_sent <= 1'b1;
                        end
                    end else begin
                        cam_state <= CAM_CAPTURE;
                    end
                end
            end

            if (camera_frame_active && camera_pixel_valid) begin
                if ((cam_state == CAM_WAIT_GRANT) && !cam_grant_now)
                    cam_early_pixel <= 1'b1;

                if (cam_can_capture) begin
                    if (cam_pixel_count >= FRAME_PIXEL_COUNT) begin
                        cam_state <= CAM_DROP;
                        if (!cam_abort_sent) begin
                            cam_abort_toggle <= ~cam_abort_toggle;
                        debug_abort_counts[23:16] <= debug_abort_counts[23:16] + 8'd1; // R7_OBSERVER
                            cam_abort_sent <= 1'b1;
                        end
                    end else if (!fifo_wr_ready) begin
                        cam_state <= CAM_DROP;
                        if (!cam_abort_sent) begin
                            cam_abort_toggle <= ~cam_abort_toggle;
                        debug_abort_counts[7:0] <= debug_abort_counts[7:0] + 8'd1; // R7_OBSERVER
                            cam_abort_sent <= 1'b1;
                        end
                    end else begin
                        cam_pixel_count <= cam_pixel_count + 1'b1;
                    end
                end
            end

            if (cam_frame_fall) begin
                cam_end_toggle <= ~cam_end_toggle;
                if (cam_state == CAM_CAPTURE) begin
                    if ((cam_pixel_count != FRAME_PIXEL_COUNT) && !cam_abort_sent) begin
                        cam_abort_toggle <= ~cam_abort_toggle;
                        debug_abort_counts[15:8] <= debug_abort_counts[15:8] + 8'd1; // R7_OBSERVER
                        cam_abort_sent <= 1'b1;
                    end
                    cam_state <= CAM_IDLE;
                end else if (cam_state == CAM_DROP) begin
                    cam_state <= CAM_IDLE;
                end else if (cam_state == CAM_WAIT_GRANT) begin
                    cam_ended_wait <= 1'b1;
                end
            end
        end
    end

    wire fifo_async_rst_n = cam_rst_n && ddr_rst_n;
    wire fifo_rd_valid;
    wire fifo_rd_ready;
    wire [CAMERA_FIFO_BITS-1:0] fifo_rd_data;
    async_fifo #(.DATA_WIDTH(CAMERA_FIFO_BITS), .DEPTH(FIFO_DEPTH)) u_camera_fifo (
        .wr_clk(cam_clk), .rd_clk(ddr_clk), .rst_n(fifo_async_rst_n),
        .wr_valid(fifo_wr_valid), .wr_ready(fifo_wr_ready), .wr_data(fifo_wr_data),
        .rd_valid(fifo_rd_valid), .rd_ready(fifo_rd_ready), .rd_data(fifo_rd_data)
    );

    // DDR-domain frame-slot owner and source reader interface.
    wire writer_alloc_valid;
    wire writer_alloc_ready;
    wire [SLOT_W-1:0] writer_alloc_slot;
    wire [63:0] writer_alloc_base_addr;
    wire writer_commit_valid;
    wire writer_commit_ready;
    wire writer_abort_valid;
    wire writer_abort_ready;
    wire reader_pending_unused;
    reg [SLOT_W-1:0] writer_commit_slot;
    reg [31:0] writer_commit_frame_id;
    reg [31:0] writer_commit_source_frame_id;
    reg [15:0] writer_commit_cfg_version;
    reg [15:0] writer_commit_width;
    reg [15:0] writer_commit_height;
    reg [31:0] writer_commit_stride_bytes;
    reg [7:0] writer_commit_format;
    reg [31:0] writer_commit_process_cycles;
    wire [NUM_SOURCE_SLOTS*3-1:0] slot_state_debug;

    frame_pool_manager #(
        .NUM_SLOTS(NUM_SOURCE_SLOTS), .ADDR_WIDTH(64), .POOL_BASE_ADDR(POOL_BASE_ADDR),
        .SLOT_STRIDE_BYTES(FRAME_SLOT_BYTES), .POOL_ROLE(0), .SLOT_W(SLOT_W)
    ) u_source_pool (
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
        .reader_pending(reader_pending_unused),
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
    reg [1:0] abort_sync;
    reg abort_seen;
    reg [1:0] end_sync;
    reg end_seen;
    reg frame_owned;
    reg abort_pending;
    reg frame_end_pending;
    reg [2:0] drain_empty_count;
    reg [63:0] active_base_addr;
    reg [SLOT_W-1:0] active_slot;
    reg [31:0] next_frame_id;
    reg [31:0] active_frame_id;
    reg [63:0] bytes_written;
    reg [255:0] burst_buffer [0:MAX_BURST_BEATS-1];
    reg [4:0] burst_count;
    reg [31:0] burst_length;
    reg burst_frame_last;
    reg [4:0] send_index;
    reg [4:0] send_index_local0 /* synthesis syn_preserve=1 syn_keep=1 */;
    reg [4:0] send_index_local1 /* synthesis syn_preserve=1 syn_keep=1 */;
    reg [4:0] send_index_local2 /* synthesis syn_preserve=1 syn_keep=1 */;
    reg [4:0] send_index_local3 /* synthesis syn_preserve=1 syn_keep=1 */;
    integer reset_burst_i;
    reg packer_clear;

    wire req_event = (req_sync[1] != req_seen);
    wire abort_event = (abort_sync[1] != abort_seen);
    wire end_event = (end_sync[1] != end_seen);
    wire [CAMERA_FIFO_BITS-1:0] packer_in_data = fifo_rd_data;
    wire packer_in_ready;
    wire packer_out_valid;
    wire packer_out_ready;
    wire [255:0] packer_out_data;
    wire packer_frame_last;
    wire packer_protocol_error;
    wire packer_in_valid = fifo_rd_valid && (writer_state == WR_COLLECT) && frame_owned &&
                           !packer_out_valid;
    wire fifo_drain_ready = (writer_state == WR_DRAIN);

    assign fifo_rd_ready = fifo_drain_ready ||
        ((writer_state == WR_COLLECT) && frame_owned && !abort_pending && !abort_event &&
         !packer_out_valid && packer_in_ready);

    assign packer_out_ready = (writer_state == WR_COLLECT) && frame_owned &&
        !abort_pending && !abort_event && (burst_count < MAX_BURST_BEATS);

    rgb565_beat_packer #(.FRAME_WIDTH(FRAME_WIDTH), .FRAME_HEIGHT(FRAME_HEIGHT),
        .INPUT_RGB888(CAMERA_RGB888_ENABLE)) u_packer (
        .clk(ddr_clk), .rst_n(ddr_rst_n), .frame_clear(packer_clear),
        .in_valid(packer_in_valid), .in_ready(packer_in_ready), .in_data(packer_in_data),
        .out_valid(packer_out_valid), .out_ready(packer_out_ready),
        .out_data(packer_out_data), .out_frame_last(packer_frame_last),
        .protocol_error(packer_protocol_error)
    );

    // R16_SOURCE_MEASURE_ASSIGN_BEGIN
    assign measurement_source_event = writer_alloc_valid && writer_alloc_ready;
    assign measurement_source_id = next_frame_id;
    assign measurement_source_sensor_id = measurement_cam_sensor_id_held;
    assign measurement_source_t0 = measurement_cam_t0_held;
    assign measurement_source_time_valid = measurement_cam_time_valid_held;
    // R16_SOURCE_MEASURE_ASSIGN_END
    assign writer_alloc_valid = (writer_state == WR_ALLOC) && layout_valid && ddr_init_done;
    assign writer_commit_valid = (writer_state == WR_COMMIT);
    assign writer_abort_valid = (writer_state == WR_ABORT);
    assign debug_writer_state = writer_state;
    assign debug_writer_alloc_ready = writer_alloc_ready;
    assign debug_frame_owned = frame_owned;
    assign debug_slot_state = slot_state_debug;

    assign mem_wr_req_valid = (writer_state == WR_REQ);
    assign mem_wr_req_addr = active_base_addr + bytes_written;
    assign mem_wr_req_len = burst_length;
    assign mem_wr_req_id = WRITE_REQ_ID;
    assign mem_wr_data_valid = (writer_state == WR_SEND);
    assign mem_wr_data = {burst_buffer[send_index_local3][255:192],
                          burst_buffer[send_index_local2][191:128],
                          burst_buffer[send_index_local1][127:64],
                          burst_buffer[send_index_local0][63:0]};
    assign mem_wr_data_last = (send_index == (burst_length - 1'b1));

    integer sync_i;
    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            req_sync <= 2'b00;
            req_seen <= 1'b0;
            abort_sync <= 2'b00;
            abort_seen <= 1'b0;
            end_sync <= 2'b00;
            end_seen <= 1'b0;
            grant_toggle_ddr <= 1'b0;
            grant_accept_ddr <= 1'b0;
            writer_state <= WR_IDLE;
            frame_owned <= 1'b0;
            abort_pending <= 1'b0;
            frame_end_pending <= 1'b0;
            drain_empty_count <= 3'd0;
            active_base_addr <= 64'd0;
            active_slot <= {SLOT_W{1'b0}};
            next_frame_id <= 32'd1;
            active_frame_id <= 32'd0;
            bytes_written <= 64'd0;
            burst_count <= 5'd0;
            burst_length <= 32'd0;
            burst_frame_last <= 1'b0;
            send_index <= 5'd0;
            send_index_local0 <= 5'd0;
            send_index_local1 <= 5'd0;
            send_index_local2 <= 5'd0;
            send_index_local3 <= 5'd0;
            packer_clear <= 1'b0;
            frame_commit_count <= 32'd0;
            frame_drop_count <= 32'd0;
            fifo_overflow_count <= 32'd0;
            writer_commit_slot <= {SLOT_W{1'b0}};
            writer_commit_frame_id <= 32'd0;
            writer_commit_source_frame_id <= 32'd0;
            writer_commit_cfg_version <= 16'd0;
            writer_commit_width <= FRAME_WIDTH;
            writer_commit_height <= FRAME_HEIGHT;
            writer_commit_stride_bytes <= ROW_STRIDE_BYTES;
            writer_commit_format <= 8'h01;
            writer_commit_process_cycles <= 32'd0;
            for (reset_burst_i = 0; reset_burst_i < MAX_BURST_BEATS; reset_burst_i = reset_burst_i + 1)
                burst_buffer[reset_burst_i] <= 256'd0;
        end else begin
            req_sync <= {req_sync[0], cam_req_toggle};
            abort_sync <= {abort_sync[0], cam_abort_toggle};
            end_sync <= {end_sync[0], cam_end_toggle};
            packer_clear <= 1'b0;

            if (abort_event) begin
                abort_seen <= abort_sync[1];
                if (frame_owned) begin
                    abort_pending <= 1'b1;
                    fifo_overflow_count <= fifo_overflow_count + 1'b1;
                end
            end
            if (end_event) begin
                end_seen <= end_sync[1];
                if (frame_owned || (writer_state == WR_DRAIN)) begin
                    frame_end_pending <= 1'b1;
                    drain_empty_count <= 3'd0;
                end
            end
            if (packer_protocol_error && frame_owned)
                abort_pending <= 1'b1;

            case (writer_state)
                WR_IDLE: begin
                    if (req_event) begin
                        req_seen <= req_sync[1];
                        if (layout_valid && ddr_init_done && writer_alloc_ready)
                            writer_state <= WR_ALLOC;
                        else begin
                            grant_accept_ddr <= 1'b0;
                            grant_toggle_ddr <= ~grant_toggle_ddr;
                            frame_drop_count <= frame_drop_count + 1'b1;
                        end
                    end
                end

                WR_ALLOC: begin
                    if (writer_alloc_ready && layout_valid && ddr_init_done) begin
                        active_base_addr <= writer_alloc_base_addr;
                        active_slot <= writer_alloc_slot;
                        writer_commit_slot <= writer_alloc_slot;
                        active_frame_id <= next_frame_id;
                        next_frame_id <= next_frame_id + 1'b1;
                        writer_commit_frame_id <= next_frame_id;
                        writer_commit_source_frame_id <= next_frame_id;
                        writer_commit_cfg_version <= cfg_version;
                        writer_commit_width <= FRAME_WIDTH;
                        writer_commit_height <= FRAME_HEIGHT;
                        writer_commit_stride_bytes <= layout_stride_bytes;
                        writer_commit_format <= 8'h01;
                        writer_commit_process_cycles <= 32'd0;
                        frame_owned <= 1'b1;
                        abort_pending <= 1'b0;
                        frame_end_pending <= 1'b0;
                        bytes_written <= 64'd0;
                        burst_count <= 5'd0;
                        burst_length <= 32'd0;
                        burst_frame_last <= 1'b0;
                        grant_accept_ddr <= 1'b1;
                        grant_toggle_ddr <= ~grant_toggle_ddr;
                        writer_state <= WR_COLLECT;
                    end else begin
                        grant_accept_ddr <= 1'b0;
                        grant_toggle_ddr <= ~grant_toggle_ddr;
                        frame_drop_count <= frame_drop_count + 1'b1;
                        writer_state <= WR_IDLE;
                    end
                end

                WR_COLLECT: begin
                    if (abort_event || abort_pending || packer_protocol_error) begin
                        writer_state <= WR_ABORT;
                        burst_count <= 5'd0;
                        packer_clear <= 1'b1;
                    end else if (packer_out_valid && packer_out_ready) begin
                        burst_buffer[burst_count] <= packer_out_data;
                        if ((burst_count == (MAX_BURST_BEATS - 1)) || packer_frame_last) begin
                            burst_length <= burst_count + 1'b1;
                            burst_frame_last <= packer_frame_last;
                            send_index <= 5'd0;
                            send_index_local0 <= 5'd0;
                            send_index_local1 <= 5'd0;
                            send_index_local2 <= 5'd0;
                            send_index_local3 <= 5'd0;
                            writer_state <= WR_REQ;
                        end else begin
                            burst_count <= burst_count + 1'b1;
                        end
                    end
                end

                WR_REQ: begin
                    if (mem_wr_req_ready) begin
                        send_index <= 5'd0;
                        send_index_local0 <= 5'd0;
                        send_index_local1 <= 5'd0;
                        send_index_local2 <= 5'd0;
                        send_index_local3 <= 5'd0;
                        writer_state <= WR_SEND;
                    end
                end

                WR_SEND: begin
                    if (mem_wr_data_ready) begin
                        if (send_index == (burst_length - 1'b1))
                            writer_state <= WR_WAIT;
                        else begin
                            send_index <= send_index + 1'b1;
                            send_index_local0 <= send_index + 1'b1;
                            send_index_local1 <= send_index + 1'b1;
                            send_index_local2 <= send_index + 1'b1;
                            send_index_local3 <= send_index + 1'b1;
                        end
                    end
                end

                WR_WAIT: begin
                    if (mem_wr_done_valid) begin
                        bytes_written <= bytes_written + (burst_length * 32);
                        burst_count <= 5'd0;
                        if ((mem_wr_done_id != WRITE_REQ_ID) || mem_wr_done_error || abort_pending || abort_event) begin
                            abort_pending <= 1'b1;
                            writer_state <= WR_ABORT;
                            packer_clear <= 1'b1;
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
                        frame_commit_count <= frame_commit_count + 1'b1;
                        writer_state <= WR_IDLE;
                    end
                end

                WR_ABORT: begin
                    packer_clear <= 1'b1;
                    if (writer_abort_ready) begin
                        frame_owned <= 1'b0;
                        frame_drop_count <= frame_drop_count + 1'b1;
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
endmodule
