`timescale 1ns/1ps
// Reusable camera/source-pool/algorithm-client/result-pool/display storage
// boundary. The physical project top is intentionally not changed here.
//
// Geometry and pool addresses default to invalid/empty values so a future top
// must explicitly provide measured dimensions and a verified DDR map. The
// fetch client is synchronous to ddr_clk; an algorithm running in another
// clock domain must cross that boundary before using this native client port.
module storage_pipeline_core_v04 #(
    parameter integer SOURCE_FRAME_WIDTH = 0,
    parameter integer SOURCE_FRAME_HEIGHT = 0,
    parameter integer RESULT_FRAME_WIDTH = 0,
    parameter integer RESULT_FRAME_HEIGHT = 0,
    parameter integer SOURCE_FIFO_DEPTH = 1024,
    parameter integer CAMERA_RGB888_ENABLE = 0,
    parameter integer DISPLAY_RD_BUFFER_ENABLE = 0,
    parameter integer RESULT_FRAME_ERROR_ENABLE = 0,
    parameter integer RESULT_FIFO_DEPTH = 1024,
    parameter integer DISPLAY_FIFO_DEPTH = 1024,
    parameter integer NUM_SOURCE_SLOTS = 3,
    parameter integer NUM_RESULT_SLOTS = 2,
    parameter [63:0] SOURCE_POOL_BASE_ADDR = 64'd0,
    parameter [63:0] RESULT_POOL_BASE_ADDR = 64'd0,
    parameter [63:0] USABLE_DDR_BASE_ADDR = 64'd0,
    parameter [63:0] USABLE_DDR_SIZE_BYTES = 64'd0,
    parameter integer CTRL_ADDR_WIDTH = 28,
    parameter integer REQ_ID_W = 8,
    parameter integer MAX_BURST_BEATS = 16,
    parameter integer VSYNC_ACTIVE_LEVEL = 1,
    parameter [REQ_ID_W-1:0] SOURCE_WRITE_REQ_ID = 8'h11,
    parameter [REQ_ID_W-1:0] RESULT_WRITE_REQ_ID = 8'h21,
    parameter [REQ_ID_W-1:0] DISPLAY_READ_REQ_ID = 8'hA5,
    parameter integer SOURCE_SLOT_W = (NUM_SOURCE_SLOTS <= 2) ? 1 : $clog2(NUM_SOURCE_SLOTS),
    parameter integer RESULT_SLOT_W = (NUM_RESULT_SLOTS <= 2) ? 1 : $clog2(NUM_RESULT_SLOTS)
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
    output wire result_processing_idle,
    input wire cam_clk,
    input wire cam_rst_n,
    input wire camera_frame_active,
    input wire camera_pixel_valid,
    input wire [15:0] camera_pixel_rgb565,
    input wire [23:0] camera_pixel_rgb888,
    input wire [15:0] camera_cfg_version,

    input wire ddr_clk,
    input wire ddr_rst_n,
    input wire ddr_init_done,

    // Atomic source-frame ownership boundary for the algorithm.
    input wire source_desc_claim_valid,
    output wire source_desc_claim_ready,
    output wire [SOURCE_SLOT_W-1:0] source_desc_claim_slot,
    output wire [63:0] source_desc_base_addr,
    output wire [31:0] source_desc_frame_id,
    output wire [31:0] source_desc_source_frame_id,
    output wire [15:0] source_desc_cfg_version,
    output wire [15:0] source_desc_width,
    output wire [15:0] source_desc_height,
    output wire [31:0] source_desc_stride_bytes,
    output wire [7:0] source_desc_format,
    output wire [31:0] source_desc_process_cycles,
    output wire [2:0] source_desc_state,
    output wire source_desc_valid,
    input wire source_desc_release_valid,
    input wire [SOURCE_SLOT_W-1:0] source_desc_release_slot,
    output wire source_desc_release_ready,

    // Pixel Fetch's byte-addressed DDR client. All these handshakes are in
    // ddr_clk; response ownership is retained through the accepted last beat.
    input wire fetch_rd_req_valid,
    output wire fetch_rd_req_ready,
    input wire [63:0] fetch_rd_req_addr,
    input wire [31:0] fetch_rd_req_len,
    input wire [REQ_ID_W-1:0] fetch_rd_req_id,
    output wire fetch_rd_rsp_valid,
    input wire fetch_rd_rsp_ready,
    output wire [255:0] fetch_rd_rsp_data,
    output wire [REQ_ID_W-1:0] fetch_rd_rsp_id,
    output wire fetch_rd_rsp_last,
    output wire fetch_rd_rsp_error,

    // Algorithm result-pixel input stream. The producer holds a SOF payload
    // until ready, then holds every subsequent payload while valid && !ready.
    input wire result_clk,
    input wire result_rst_n,
    input wire result_px_valid,
    output wire result_px_ready,
    input wire [23:0] result_px_rgb,
    input wire [15:0] result_px_x,
    input wire [15:0] result_px_y,
    input wire result_px_sof,
    input wire result_px_eol,
    input wire [31:0] result_px_source_frame_id,
    input wire [15:0] result_px_cfg_version,
    input wire [31:0] result_px_pixel_tag,
    input wire result_frame_error,

    // Pixel-domain display consumer. display_pixel_req means one active
    // output pixel is due on this pix_clk edge, even when the FIFO is empty.
    input wire pix_clk,
    input wire pix_rst_n,
    input wire pix_vsync,
    input wire display_pixel_req,
    output wire display_pixel_valid,
    output wire [23:0] display_pixel_rgb,
    output wire [15:0] display_pixel_x,
    output wire [15:0] display_pixel_y,
    output wire display_pixel_sof,
    output wire display_pixel_eol,
    output wire display_pixel_eof,
    output wire [31:0] display_frame_id,
    output wire [31:0] display_source_frame_id,
    output wire [15:0] display_cfg_version,
    output wire display_scan_done,

    // Pango native DDR3 user interface. This is the sole native client port
    // of this core and must have one physical owner in the eventual system.
    output wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr,
    output wire [3:0] axi_awuser_id,
    output wire [3:0] axi_awlen,
    output wire axi_awvalid,
    input wire axi_awready,
    output wire [255:0] axi_wdata,
    output wire [31:0] axi_wstrb,
    input wire axi_wready,
    input wire axi_wlast,
    output wire [CTRL_ADDR_WIDTH-1:0] axi_araddr,
    output wire [3:0] axi_aruser_id,
    output wire [3:0] axi_arlen,
    output wire axi_arvalid,
    input wire axi_arready,
    input wire [255:0] axi_rdata,
    input wire [3:0] axi_rid,
    input wire axi_rlast,
    input wire axi_rvalid,

    output wire [31:0] source_frame_commit_count,
    output wire [31:0] source_frame_drop_count,
    output wire [31:0] source_fifo_overflow_count,
    output wire [31:0] source_abort_cause_counts, // R7_OBSERVER cam_clk domain
    output wire [31:0] result_commit_count,
    output wire [31:0] result_drop_count,
    output wire [31:0] result_fifo_stall_count,
    output wire [31:0] result_protocol_error_count,
    output wire [31:0] display_repeat_count,
    output wire [31:0] display_reader_underflow_count,
    output wire [31:0] display_scan_overrun_count,
    output wire [31:0] display_stream_source_stall_count,
    output wire [31:0] display_stream_underflow_count,
    output wire [3:0] source_writer_state_debug,
    output wire source_writer_alloc_ready_debug,
    output wire source_writer_frame_owned_debug,
    output wire [NUM_SOURCE_SLOTS*3-1:0] source_slot_state_debug
    // R16_STORAGE_MEASURE_PORTS_BEGIN
    , input wire measurement_camera_time_valid
    , input wire [63:0] measurement_camera_t0
    , input wire [31:0] measurement_camera_sensor_id
    , output wire measurement_source_event
    , output wire [31:0] measurement_source_id
    , output wire [31:0] measurement_source_sensor_id
    , output wire [63:0] measurement_source_t0
    , output wire measurement_source_time_valid
    , output wire measurement_commit_event
    , output wire [31:0] measurement_commit_source_id
    , output wire [15:0] measurement_commit_cfg
    // R16_STORAGE_MEASURE_PORTS_END
);
    // R16_STORAGE_MEASURE_DECL_BEGIN
    wire [31:0] measurement_source_id_internal;
    wire [31:0] measurement_source_sensor_id_internal;
    wire [63:0] measurement_source_t0_internal;
    wire measurement_source_time_valid_internal;
    wire measurement_commit_event_internal;
    wire [31:0] measurement_commit_source_id_internal;
    wire [15:0] measurement_commit_cfg_internal;
    // R16_STORAGE_MEASURE_DECL_END
    wire source_wr_req_valid;
    wire source_wr_req_ready;
    wire [63:0] source_wr_req_addr;
    wire [31:0] source_wr_req_len;
    wire [REQ_ID_W-1:0] source_wr_req_id;
    wire source_wr_data_valid;
    wire source_wr_data_ready;
    wire [255:0] source_wr_data;
    wire source_wr_data_last;
    wire source_wr_done_valid;
    wire [REQ_ID_W-1:0] source_wr_done_id;
    wire source_wr_done_error;

    wire result_wr_req_valid;
    wire result_wr_req_ready;
    wire [63:0] result_wr_req_addr;
    wire [31:0] result_wr_req_len;
    wire [REQ_ID_W-1:0] result_wr_req_id;
    wire result_wr_data_valid;
    wire result_wr_data_ready;
    wire [255:0] result_wr_data;
    wire result_wr_data_last;
    wire result_wr_done_valid;
    wire [REQ_ID_W-1:0] result_wr_done_id;
    wire result_wr_done_error;

    wire fetch_mem_rd_req_ready;
    wire fetch_mem_rd_rsp_valid;
    wire [255:0] fetch_mem_rd_rsp_data;
    wire [REQ_ID_W-1:0] fetch_mem_rd_rsp_id;
    wire fetch_mem_rd_rsp_last;
    wire fetch_mem_rd_rsp_error;

    assign fetch_rd_req_ready = fetch_mem_rd_req_ready;
    assign fetch_rd_rsp_valid = fetch_mem_rd_rsp_valid;
    assign fetch_rd_rsp_data = fetch_mem_rd_rsp_data;
    assign fetch_rd_rsp_id = fetch_mem_rd_rsp_id;
    assign fetch_rd_rsp_last = fetch_mem_rd_rsp_last;
    assign fetch_rd_rsp_error = fetch_mem_rd_rsp_error;

    wire display_mem_rd_req_valid;
    wire display_mem_rd_req_ready;
    wire [63:0] display_mem_rd_req_addr;
    wire [31:0] display_mem_rd_req_len;
    wire [REQ_ID_W-1:0] display_mem_rd_req_id;
    wire display_mem_rd_rsp_valid;
    wire display_mem_rd_rsp_ready;
    wire [255:0] display_mem_rd_rsp_data;
    wire [REQ_ID_W-1:0] display_mem_rd_rsp_id;
    wire display_mem_rd_rsp_last;
    wire display_mem_rd_rsp_error;

    wire result_reader_pending;
    wire result_reader_claim_valid;
    wire result_reader_claim_ready;
    wire [RESULT_SLOT_W-1:0] result_reader_claim_slot;
    wire [63:0] result_reader_desc_base_addr;
    wire [31:0] result_reader_desc_frame_id;
    wire [31:0] result_reader_desc_source_frame_id;
    wire [15:0] result_reader_desc_cfg_version;
    wire [15:0] result_reader_desc_width;
    wire [15:0] result_reader_desc_height;
    wire [31:0] result_reader_desc_stride_bytes;
    wire [7:0] result_reader_desc_format;
    wire [31:0] result_reader_desc_process_cycles;
    wire [2:0] result_reader_desc_state;
    wire result_reader_desc_valid;
    wire result_reader_release_valid;
    wire [RESULT_SLOT_W-1:0] result_reader_release_slot;
    wire result_reader_release_ready;

    wire result_pixel_out_valid;
    wire result_pixel_out_ready;
    wire [23:0] result_pixel_out_rgb;
    wire [15:0] result_pixel_out_x;
    wire [15:0] result_pixel_out_y;
    wire result_pixel_out_sof;
    wire result_pixel_out_eol;
    wire result_pixel_out_eof;
    wire [31:0] result_display_frame_id;
    wire [31:0] result_display_source_frame_id;
    wire [15:0] result_display_cfg_version;
    wire frame_scan_start;
    wire display_reset_n = ddr_rst_n && pix_rst_n;

    input_frame_writer #(
        .FRAME_WIDTH(SOURCE_FRAME_WIDTH), .FRAME_HEIGHT(SOURCE_FRAME_HEIGHT),
        .FIFO_DEPTH(SOURCE_FIFO_DEPTH), .NUM_SOURCE_SLOTS(NUM_SOURCE_SLOTS),
        .CAMERA_RGB888_ENABLE(CAMERA_RGB888_ENABLE),
        .POOL_BASE_ADDR(SOURCE_POOL_BASE_ADDR),
        .USABLE_DDR_BASE_ADDR(USABLE_DDR_BASE_ADDR),
        .USABLE_DDR_SIZE_BYTES(USABLE_DDR_SIZE_BYTES),
        .MAX_BURST_BEATS(MAX_BURST_BEATS), .REQ_ID_W(REQ_ID_W),
        .WRITE_REQ_ID(SOURCE_WRITE_REQ_ID), .SLOT_W(SOURCE_SLOT_W)
    ) u_source_writer (
        .cam_clk(cam_clk), .cam_rst_n(cam_rst_n),
        .camera_frame_active(camera_frame_active), .camera_pixel_valid(camera_pixel_valid),
        .camera_pixel_rgb565(camera_pixel_rgb565),
        .camera_pixel_rgb888(camera_pixel_rgb888),
        .ddr_clk(ddr_clk), .ddr_rst_n(ddr_rst_n), .ddr_init_done(ddr_init_done),
        .cfg_version(camera_cfg_version),
        .reader_claim_valid(source_desc_claim_valid), .reader_claim_ready(source_desc_claim_ready),
        .reader_claim_slot(source_desc_claim_slot), .reader_desc_base_addr(source_desc_base_addr),
        .reader_desc_frame_id(source_desc_frame_id),
        .reader_desc_source_frame_id(source_desc_source_frame_id),
        .reader_desc_cfg_version(source_desc_cfg_version), .reader_desc_width(source_desc_width),
        .reader_desc_height(source_desc_height), .reader_desc_stride_bytes(source_desc_stride_bytes),
        .reader_desc_format(source_desc_format), .reader_desc_process_cycles(source_desc_process_cycles),
        .reader_desc_state(source_desc_state), .reader_desc_valid(source_desc_valid),
        .reader_release_valid(source_desc_release_valid),
        .reader_release_slot(source_desc_release_slot),
        .reader_release_ready(source_desc_release_ready),
        .mem_wr_req_valid(source_wr_req_valid), .mem_wr_req_ready(source_wr_req_ready),
        .mem_wr_req_addr(source_wr_req_addr), .mem_wr_req_len(source_wr_req_len),
        .mem_wr_req_id(source_wr_req_id), .mem_wr_data_valid(source_wr_data_valid),
        .mem_wr_data_ready(source_wr_data_ready), .mem_wr_data(source_wr_data),
        .mem_wr_data_last(source_wr_data_last), .mem_wr_done_valid(source_wr_done_valid),
        .mem_wr_done_id(source_wr_done_id), .mem_wr_done_error(source_wr_done_error),
        .frame_commit_count(source_frame_commit_count), .frame_drop_count(source_frame_drop_count),
        .fifo_overflow_count(source_fifo_overflow_count),
        .debug_abort_counts(source_abort_cause_counts), // R7_OBSERVER
        .debug_writer_state(source_writer_state_debug),
        .debug_writer_alloc_ready(source_writer_alloc_ready_debug),
        .debug_frame_owned(source_writer_frame_owned_debug),
        .debug_slot_state(source_slot_state_debug)
        // R16_STORAGE_SOURCE_WIRING_BEGIN
        , .measurement_camera_time_valid(measurement_camera_time_valid)
        , .measurement_camera_t0(measurement_camera_t0)
        , .measurement_camera_sensor_id(measurement_camera_sensor_id)
        , .measurement_source_event(measurement_source_event)
        , .measurement_source_id(measurement_source_id_internal)
        , .measurement_source_sensor_id(measurement_source_sensor_id_internal)
        , .measurement_source_t0(measurement_source_t0_internal)
        , .measurement_source_time_valid(measurement_source_time_valid_internal)
        // R16_STORAGE_SOURCE_WIRING_END
    );
    // R16_STORAGE_SOURCE_ASSIGN_BEGIN
    assign measurement_source_id = measurement_source_id_internal;
    assign measurement_source_sensor_id = measurement_source_sensor_id_internal;
    assign measurement_source_t0 = measurement_source_t0_internal;
    assign measurement_source_time_valid = measurement_source_time_valid_internal;
    // R16_STORAGE_SOURCE_ASSIGN_END

    result_frame_writer_idle_v04 #(
        .FRAME_WIDTH(RESULT_FRAME_WIDTH), .FRAME_HEIGHT(RESULT_FRAME_HEIGHT),
        .FRAME_ERROR_ENABLE(RESULT_FRAME_ERROR_ENABLE),
        .FIFO_DEPTH(RESULT_FIFO_DEPTH), .NUM_RESULT_SLOTS(NUM_RESULT_SLOTS),
        .POOL_BASE_ADDR(RESULT_POOL_BASE_ADDR),
        .USABLE_DDR_BASE_ADDR(USABLE_DDR_BASE_ADDR),
        .USABLE_DDR_SIZE_BYTES(USABLE_DDR_SIZE_BYTES),
        .MAX_BURST_BEATS(MAX_BURST_BEATS), .REQ_ID_W(REQ_ID_W),
        .WRITE_REQ_ID(RESULT_WRITE_REQ_ID), .SLOT_W(RESULT_SLOT_W)
    ) u_result_writer (
        .processing_idle(result_processing_idle),
        .result_clk(result_clk), .result_rst_n(result_rst_n),
        .result_px_valid(result_px_valid), .result_px_ready(result_px_ready),
        .result_px_rgb(result_px_rgb), .result_px_x(result_px_x), .result_px_y(result_px_y),
        .result_px_sof(result_px_sof), .result_px_eol(result_px_eol),
        .result_px_source_frame_id(result_px_source_frame_id),
        .result_px_cfg_version(result_px_cfg_version), .result_px_pixel_tag(result_px_pixel_tag),
        .result_frame_error(result_frame_error),
        .ddr_clk(ddr_clk), .ddr_rst_n(ddr_rst_n), .ddr_init_done(ddr_init_done),
        .reader_claim_valid(result_reader_claim_valid), .reader_claim_ready(result_reader_claim_ready),
        .reader_pending(result_reader_pending), .reader_claim_slot(result_reader_claim_slot),
        .reader_desc_base_addr(result_reader_desc_base_addr),
        .reader_desc_frame_id(result_reader_desc_frame_id),
        .reader_desc_source_frame_id(result_reader_desc_source_frame_id),
        .reader_desc_cfg_version(result_reader_desc_cfg_version),
        .reader_desc_width(result_reader_desc_width), .reader_desc_height(result_reader_desc_height),
        .reader_desc_stride_bytes(result_reader_desc_stride_bytes),
        .reader_desc_format(result_reader_desc_format),
        .reader_desc_process_cycles(result_reader_desc_process_cycles),
        .reader_desc_state(result_reader_desc_state), .reader_desc_valid(result_reader_desc_valid),
        .reader_release_valid(result_reader_release_valid),
        .reader_release_slot(result_reader_release_slot),
        .reader_release_ready(result_reader_release_ready),
        .mem_wr_req_valid(result_wr_req_valid), .mem_wr_req_ready(result_wr_req_ready),
        .mem_wr_req_addr(result_wr_req_addr), .mem_wr_req_len(result_wr_req_len),
        .mem_wr_req_id(result_wr_req_id), .mem_wr_data_valid(result_wr_data_valid),
        .mem_wr_data_ready(result_wr_data_ready), .mem_wr_data(result_wr_data),
        .mem_wr_data_last(result_wr_data_last), .mem_wr_done_valid(result_wr_done_valid),
        .mem_wr_done_id(result_wr_done_id), .mem_wr_done_error(result_wr_done_error),
        .result_commit_count(result_commit_count), .result_drop_count(result_drop_count),
        .fifo_stall_count(result_fifo_stall_count), .protocol_error_count(result_protocol_error_count)
        // R16_STORAGE_RESULT_WIRING_BEGIN
        , .measurement_commit_event(measurement_commit_event_internal)
        , .measurement_commit_source_id(measurement_commit_source_id_internal)
        , .measurement_commit_cfg(measurement_commit_cfg_internal)
        // R16_STORAGE_RESULT_WIRING_END
    );
    // R16_STORAGE_RESULT_ASSIGN_BEGIN
    assign measurement_commit_event = measurement_commit_event_internal;
    assign measurement_commit_source_id = measurement_commit_source_id_internal;
    assign measurement_commit_cfg = measurement_commit_cfg_internal;
    // R16_STORAGE_RESULT_ASSIGN_END

    result_frame_reader #(
        .MAX_BURST_BEATS(MAX_BURST_BEATS), .REQ_ID_W(REQ_ID_W),
        .READ_REQ_ID(DISPLAY_READ_REQ_ID), .SLOT_W(RESULT_SLOT_W)
    ) u_result_reader (
        .clk(ddr_clk), .rst_n(ddr_rst_n), .scan_start(frame_scan_start),
        .pool_reader_pending(result_reader_pending), .pool_claim_valid(result_reader_claim_valid),
        .pool_claim_ready(result_reader_claim_ready), .pool_claim_slot(result_reader_claim_slot),
        .pool_desc_base_addr(result_reader_desc_base_addr),
        .pool_desc_frame_id(result_reader_desc_frame_id),
        .pool_desc_source_frame_id(result_reader_desc_source_frame_id),
        .pool_desc_cfg_version(result_reader_desc_cfg_version),
        .pool_desc_width(result_reader_desc_width), .pool_desc_height(result_reader_desc_height),
        .pool_desc_stride_bytes(result_reader_desc_stride_bytes),
        .pool_desc_format(result_reader_desc_format),
        .pool_desc_process_cycles(result_reader_desc_process_cycles),
        .pool_desc_valid(result_reader_desc_valid),
        .pool_release_valid(result_reader_release_valid),
        .pool_release_slot(result_reader_release_slot), .pool_release_ready(result_reader_release_ready),
        .mem_rd_req_valid(display_mem_rd_req_valid), .mem_rd_req_ready(display_mem_rd_req_ready),
        .mem_rd_req_addr(display_mem_rd_req_addr), .mem_rd_req_len(display_mem_rd_req_len),
        .mem_rd_req_id(display_mem_rd_req_id), .mem_rd_rsp_valid(display_mem_rd_rsp_valid),
        .mem_rd_rsp_ready(display_mem_rd_rsp_ready), .mem_rd_rsp_data(display_mem_rd_rsp_data),
        .mem_rd_rsp_id(display_mem_rd_rsp_id), .mem_rd_rsp_last(display_mem_rd_rsp_last),
        .mem_rd_rsp_error(display_mem_rd_rsp_error),
        .pixel_out_valid(result_pixel_out_valid), .pixel_out_ready(result_pixel_out_ready),
        .pixel_out_rgb(result_pixel_out_rgb), .pixel_out_x(result_pixel_out_x),
        .pixel_out_y(result_pixel_out_y), .pixel_out_sof(result_pixel_out_sof),
        .pixel_out_eol(result_pixel_out_eol), .pixel_out_eof(result_pixel_out_eof),
        .display_frame_id(result_display_frame_id),
        .display_source_frame_id(result_display_source_frame_id),
        .display_cfg_version(result_display_cfg_version), .scan_done(display_scan_done),
        .display_repeat_count(display_repeat_count),
        .display_underflow_count(display_reader_underflow_count),
        .scan_overrun_count(display_scan_overrun_count)
    );

    vsync_frame_event_cdc #(.VSYNC_ACTIVE_LEVEL(VSYNC_ACTIVE_LEVEL)) u_vsync_to_ddr (
        .src_clk(pix_clk), .dst_clk(ddr_clk), .arst_n(display_reset_n),
        .src_vsync(pix_vsync), .dst_frame_start(frame_scan_start)
    );

    result_display_stream_cdc #(.FIFO_DEPTH(DISPLAY_FIFO_DEPTH)) u_display_pixel_cdc (
        .src_clk(ddr_clk), .dst_clk(pix_clk), .rst_n(display_reset_n),
        .src_valid(result_pixel_out_valid), .src_ready(result_pixel_out_ready),
        .src_rgb(result_pixel_out_rgb), .src_x(result_pixel_out_x), .src_y(result_pixel_out_y),
        .src_sof(result_pixel_out_sof), .src_eol(result_pixel_out_eol),
        .src_eof(result_pixel_out_eof), .src_frame_id(result_display_frame_id),
        .src_source_frame_id(result_display_source_frame_id),
        .src_cfg_version(result_display_cfg_version),
        .dst_valid(display_pixel_valid), .dst_ready(display_pixel_req),
        .dst_rgb(display_pixel_rgb), .dst_x(display_pixel_x), .dst_y(display_pixel_y),
        .dst_sof(display_pixel_sof), .dst_eol(display_pixel_eol), .dst_eof(display_pixel_eof),
        .dst_frame_id(display_frame_id), .dst_source_frame_id(display_source_frame_id),
        .dst_cfg_version(display_cfg_version),
        .source_stall_count(display_stream_source_stall_count),
        .display_underflow_count(display_stream_underflow_count)
    );

    storage_ddr_client_mux #(
        .DISPLAY_RESPONSE_BUFFER_ENABLE(DISPLAY_RD_BUFFER_ENABLE),
        .CTRL_ADDR_WIDTH(CTRL_ADDR_WIDTH), .ADDR_UNIT_SHIFT(2), .MEM_BEAT_BYTES(32),
        .DATA_WIDTH(256), .REQ_ID_W(REQ_ID_W), .MAX_BURST_BEATS(MAX_BURST_BEATS)
    ) u_ddr_clients (
        .ddr_clk(ddr_clk), .ddr_rst_n(ddr_rst_n), .ddr_init_done(ddr_init_done),
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
        .src_mem_wr_req_valid(source_wr_req_valid), .src_mem_wr_req_ready(source_wr_req_ready),
        .src_mem_wr_req_addr(source_wr_req_addr), .src_mem_wr_req_len(source_wr_req_len),
        .src_mem_wr_req_id(source_wr_req_id), .src_mem_wr_data_valid(source_wr_data_valid),
        .src_mem_wr_data_ready(source_wr_data_ready), .src_mem_wr_data(source_wr_data),
        .src_mem_wr_data_last(source_wr_data_last), .src_mem_wr_done_valid(source_wr_done_valid),
        .src_mem_wr_done_id(source_wr_done_id), .src_mem_wr_done_error(source_wr_done_error),
        .res_mem_wr_req_valid(result_wr_req_valid), .res_mem_wr_req_ready(result_wr_req_ready),
        .res_mem_wr_req_addr(result_wr_req_addr), .res_mem_wr_req_len(result_wr_req_len),
        .res_mem_wr_req_id(result_wr_req_id), .res_mem_wr_data_valid(result_wr_data_valid),
        .res_mem_wr_data_ready(result_wr_data_ready), .res_mem_wr_data(result_wr_data),
        .res_mem_wr_data_last(result_wr_data_last), .res_mem_wr_done_valid(result_wr_done_valid),
        .res_mem_wr_done_id(result_wr_done_id), .res_mem_wr_done_error(result_wr_done_error),
        .fetch_mem_rd_req_valid(fetch_rd_req_valid), .fetch_mem_rd_req_ready(fetch_mem_rd_req_ready),
        .fetch_mem_rd_req_addr(fetch_rd_req_addr), .fetch_mem_rd_req_len(fetch_rd_req_len),
        .fetch_mem_rd_req_id(fetch_rd_req_id), .fetch_mem_rd_rsp_valid(fetch_mem_rd_rsp_valid),
        .fetch_mem_rd_rsp_ready(fetch_rd_rsp_ready), .fetch_mem_rd_rsp_data(fetch_mem_rd_rsp_data),
        .fetch_mem_rd_rsp_id(fetch_mem_rd_rsp_id), .fetch_mem_rd_rsp_last(fetch_mem_rd_rsp_last),
        .fetch_mem_rd_rsp_error(fetch_mem_rd_rsp_error),
        .display_mem_rd_req_valid(display_mem_rd_req_valid),
        .display_mem_rd_req_ready(display_mem_rd_req_ready),
        .display_mem_rd_req_addr(display_mem_rd_req_addr),
        .display_mem_rd_req_len(display_mem_rd_req_len),
        .display_mem_rd_req_id(display_mem_rd_req_id),
        .display_mem_rd_rsp_valid(display_mem_rd_rsp_valid),
        .display_mem_rd_rsp_ready(display_mem_rd_rsp_ready),
        .display_mem_rd_rsp_data(display_mem_rd_rsp_data),
        .display_mem_rd_rsp_id(display_mem_rd_rsp_id),
        .display_mem_rd_rsp_last(display_mem_rd_rsp_last),
        .display_mem_rd_rsp_error(display_mem_rd_rsp_error),
        .axi_awaddr(axi_awaddr), .axi_awuser_id(axi_awuser_id), .axi_awlen(axi_awlen),
        .axi_awvalid(axi_awvalid), .axi_awready(axi_awready),
        .axi_wdata(axi_wdata), .axi_wstrb(axi_wstrb), .axi_wready(axi_wready),
        .axi_wlast(axi_wlast), .axi_araddr(axi_araddr), .axi_aruser_id(axi_aruser_id),
        .axi_arlen(axi_arlen), .axi_arvalid(axi_arvalid), .axi_arready(axi_arready),
        .axi_rdata(axi_rdata), .axi_rid(axi_rid), .axi_rlast(axi_rlast), .axi_rvalid(axi_rvalid)
    );
endmodule
