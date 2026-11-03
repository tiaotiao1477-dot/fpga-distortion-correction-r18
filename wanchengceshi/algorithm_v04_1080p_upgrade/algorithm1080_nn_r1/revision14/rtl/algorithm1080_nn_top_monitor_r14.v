// Native1080 R11 transport with frozen v0.4 distortion/rotation/single bilinear algorithm.
// GENERATED integration candidate. Profile origin is recorded in manifest.json.
`timescale 1ns / 1ps
module video1080_stream_top#(
    // Native1080 candidate supports 0: direct HDMI pattern, 3: RAW camera + v0.4 geometry.
    // Legacy 720p modes 1/2 are intentionally rejected in this candidate.
    parameter integer VIDEO_SOURCE      = 3,
    parameter integer SPATIAL_COLOR_ENABLE = 1,
    parameter [2:0]   TEST_PATTERN       = 3'd5,
    parameter MEM_ROW_ADDR_WIDTH   = 15,
    parameter MEM_COL_ADDR_WIDTH   = 10,
    parameter MEM_BADDR_WIDTH      = 3,
    parameter MEM_DQ_WIDTH         = 32,
    parameter MEM_DQS_WIDTH        = 32/8,
    parameter integer ADAPTIVE_GAMMA_ENABLE = 1,
    parameter integer FORCE_BLACK_BORDER = 1,
    parameter GAMMA_LUT_FILE = "../revision11/data/gamma_lut.hex"
)(
    input                                sys_clk,
    input                                clk_p,
    input                                clk_n,
    input                                rst_in,
    input                                uart_rx,
    output                               uart_tx,
    input                                cam_pclk,
    input                                cam_vsync,
    input                                cam_href,
    input       [7:0]                    cam_data,
    inout                                cam_scl,
    inout                                cam_sda,
    output                               cam_reset_n,
    output                               cam_pwdn,
    output                               mem_rst_n,
    output                               mem_ck,
    output                               mem_ck_n,
    output                               mem_cke,
    output                               mem_cs_n,
    output                               mem_ras_n,
    output                               mem_cas_n,
    output                               mem_we_n,
    output                               mem_odt,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm,
    output reg                           heart_beat_led,
    output                               ddr_init_done,
    output                               camera_init_done_led,
    output                               camera_init_pass_led,
    output                               rstn_out,
    output                               hd_scl,
    inout                                hd_sda,
    output                               hdmi_int_led,
    output                               pix_clk,
    output reg                           vs_out,
    output reg                           hs_out,
    output reg                           de_out,
    output reg [7:0]                     r_out,
    output reg [7:0]                     g_out,
    output reg [7:0]                     b_out
);
    wire [7:0] dvp_sample_data;
    wire dvp_sample_href,dvp_sample_vsync;
    // External DVP pads have one shared, coherent IO-register destination.
    dvp_input_ioreg_r5 u_dvp_input(.pclk(cam_pclk),.data(cam_data),.href(cam_href),.vsync(cam_vsync),
       .sample_data(dvp_sample_data),.sample_href(dvp_sample_href),.sample_vsync(dvp_sample_vsync));
    generate if(VIDEO_SOURCE!=0 && VIDEO_SOURCE!=3)begin:g_unsupported_native1080_mode
        NATIVE1080_SUPPORTS_ONLY_VIDEO_SOURCE_0_OR_3 unsupported_mode();
    end endgenerate
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd33000000;
/////////////////////////////////////////////////////////////////////////////////////
    reg  [15:0]                 rstn_1ms            ;
    wire[15:0]                  o_rgb565            ;

    // Keep clock/status nets explicit; implicit Verilog wires can hide
    // misspelled clock or reset connections in a permissive compile.
    wire                        locked;
    wire                        cfg_locked;
    wire                        cfg_clk;
    wire                        core_clk;
    wire                        init_done;
localparam [539:0] COLOR_RED = {54{10'd256}};
localparam [539:0] COLOR_BLUE = {54{10'd256}};
localparam [2047:0] COLOR_TONE = 2048'hfffefdfdfcfbfaf9f9f8f7f6f5f5f4f3f2f1f0f0efeeedececebeae9e8e8e7e6e5e4e3e3e2e1e0dfdededddcdbdad9d9d8d7d6d5d4d4d3d2d1d0cfcfcecdcccbcacac9c8c7c6c5c4c4c3c2c1c0bfbebebdbcbbbab9b8b7b7b6b5b4b3b2b1b1b0afaeadacabaaa9a9a8a7a6a5a4a3a2a1a1a09f9e9d9c9b9a99989797969594939291908f8e8d8c8c8b8a898887868584838281807f7e7d7c7b7b7a797877767574737271706f6e6d6c6b6a696867666564636261605f5e5d5c5b5a5958565554535251504f4e4d4c4b4a4947464544434241403f3d3c3b3a39383635343332302f2e2d2c2a29282725242321201f1d1c1a191816151312100e0d0b0907050300;
localparam [2047:0] IDENTITY_TONE = 2048'hfffefdfcfbfaf9f8f7f6f5f4f3f2f1f0efeeedecebeae9e8e7e6e5e4e3e2e1e0dfdedddcdbdad9d8d7d6d5d4d3d2d1d0cfcecdcccbcac9c8c7c6c5c4c3c2c1c0bfbebdbcbbbab9b8b7b6b5b4b3b2b1b0afaeadacabaaa9a8a7a6a5a4a3a2a1a09f9e9d9c9b9a999897969594939291908f8e8d8c8b8a898887868584838281807f7e7d7c7b7a797877767574737271706f6e6d6c6b6a696867666564636261605f5e5d5c5b5a595857565554535251504f4e4d4c4b4a494847464544434241403f3e3d3c3b3a393837363534333231302f2e2d2c2b2a292827262524232221201f1e1d1c1b1a191817161514131211100f0e0d0c0b0a09080706050403020100;
    wire nn_filtered_rx,nn_cmd_valid,nn_cmd_ready,nn_legacy_frame_active;
    wire [7:0] nn_cmd_op;
    wire [31:0] nn_cmd_seq,nn_cmd_addr,nn_uart_bad,nn_uart_busy;
    wire [255:0] nn_cmd_data;
    wire [1023:0] nn_telemetry;
    wire [1151:0] nn_telemetry_held;
    wire [127:0] gamma_observation;
    wire nn_telemetry_ack;
    wire [1151:0] nn_telemetry_snapshot_source={gamma_observation,nn_telemetry};
    wire  nn_wr_req_valid;
    wire  nn_wr_req_ready;
    wire [63:0] nn_wr_req_addr;
    wire [31:0] nn_wr_req_len;
    wire [7:0] nn_wr_req_id;
    wire  nn_wr_data_valid;
    wire  nn_wr_data_ready;
    wire [255:0] nn_wr_data;
    wire  nn_wr_data_last;
    wire  nn_wr_done_valid;
    wire [7:0] nn_wr_done_id;
    wire  nn_wr_done_error;
    wire  nn_rd_req_valid;
    wire  nn_rd_req_ready;
    wire [63:0] nn_rd_req_addr;
    wire [31:0] nn_rd_req_len;
    wire [7:0] nn_rd_req_id;
    wire  nn_rd_rsp_valid;
    wire  nn_rd_rsp_ready;
    wire [255:0] nn_rd_rsp_data;
    wire [7:0] nn_rd_rsp_id;
    wire  nn_rd_rsp_last;
    wire  nn_rd_rsp_error;
    wire video_result_idle;
    wire v04_diag_rx,v04_cfg_valid,v04_cfg_ready;
    wire [735:0] v04_cfg_payload;
    wire [31:0] v04_cfg_seq,v04_cfg_bad_count,v04_cfg_busy_count;
    wire v04_cfg_received,v04_cfg_rejected,v04_cfg_applied;
    wire [31:0] v04_received_seq,v04_applied_seq,v04_applied_frame;
    wire [15:0] v04_active_version;
    reg [31:0] v04_received_count,v04_rejected_count,v04_applied_count;
    reg [1:0] v04_sys_reset_sync;
    wire v04_control_arst_n=rstn_out&&ddr_init_done;
    always @(posedge sys_clk or negedge v04_control_arst_n)
        if(!v04_control_arst_n)v04_sys_reset_sync<=0;
        else v04_sys_reset_sync<={v04_sys_reset_sync[0],1'b1};
    v04_uart_config_bridge #(.CLK_FREQ_HZ(27000000)) u_v04_uart_control(
        .sys_clk(sys_clk),.sys_rst_n(v04_sys_reset_sync[1]),.uart_rx(nn_filtered_rx),.diagnostic_rx(v04_diag_rx),.legacy_frame_active(nn_legacy_frame_active),
        .ddr_clk(core_clk),.ddr_rst_n(storage_ddr_rst_n),.cfg_valid(v04_cfg_valid),.cfg_ready(v04_cfg_ready),
        .cfg_payload(v04_cfg_payload),.cfg_seq(v04_cfg_seq),.bad_count(v04_cfg_bad_count),.busy_count(v04_cfg_busy_count));
    always @(posedge core_clk or negedge storage_ddr_rst_n)begin
        if(!storage_ddr_rst_n)begin v04_received_count<=0;v04_rejected_count<=0;v04_applied_count<=0;end
        else begin
            if(v04_cfg_received)v04_received_count<=v04_received_count+1'b1;
            if(v04_cfg_rejected)v04_rejected_count<=v04_rejected_count+1'b1;
            if(v04_cfg_applied)v04_applied_count<=v04_applied_count+1'b1;
        end
    end
    nn_uart_bridge nn_uart_control(
        .legacy_frame_active(nn_legacy_frame_active),
        .sys_clk(sys_clk),.sys_rst_n(v04_sys_reset_sync[1]),.uart_rx(uart_rx),.diagnostic_rx(nn_filtered_rx),
        .core_clk(core_clk),.core_rst_n(storage_ddr_rst_n),
        .cmd_valid(nn_cmd_valid),.cmd_ready(nn_cmd_ready),.cmd_op(nn_cmd_op),
        .cmd_seq(nn_cmd_seq),.cmd_addr(nn_cmd_addr),.cmd_data(nn_cmd_data),
        .bad_count(nn_uart_bad),.busy_count(nn_uart_busy));
    camera_diag_snapshot_cdc #(.WIDTH(1152)) nn_telemetry_snapshot(
        .src_clk(core_clk),.src_rst_n(storage_ddr_rst_n),.request_toggle_async(diag_req_toggle),
        .counter_bus(nn_telemetry_snapshot_source),.snapshot_bus(nn_telemetry_held),.ack_toggle(nn_telemetry_ack));
    // Independent of DDR readiness: SCCB/readback/UART must start first.
    wire sys_diag_rst_n;
    wire [3:0] sys_diag_status;
    sys_diag_cdc_r5 u_sys_diag_cdc(
        .sys_clk(sys_clk),.arst_n(rstn_out),
        .status_async({cfg_locked,ddr_init_done,tx_init_over,capture_rst_n}),
        .sys_rst_n(sys_diag_rst_n),.status_sync(sys_diag_status));
//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_awvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_wvalid_fb              ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_arvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    wire                        frame_commit_pulse;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
    reg  [31:0]                 diag_axi_wlast_count       ;
    reg  [31:0]                 diag_frame_commit_count    ;
    reg  [31:0]                 diag_axi_rvalid_count      ;
/////////////////////////////////////////////////////////////////////////////////////
//PLL
// 720P30 output pixel clock: 27 MHz input -> 37.125 MHz.
pll_1080p30_candidate_r1 pll_gen_clk (
    .clkin1   (sys_clk),
    .clkout0  (pix_clk),
    .clkout1  (),
    .clkout2  (),
    .lock     (locked)
);

cfg_pll cfg_pll_inst (
  .clkout0(cfg_clk),    // output
  .lock(cfg_locked),    // output
  .clkin1(sys_clk)       // input
);




// OV5640 power/reset and SCCB configuration.
wire camera_scl_int;
wire sccb_rb_scl_out;
wire sccb_rb_sda_out;
wire sccb_rb_sda_oe;
wire sccb_rb_busy;
wire sccb_rb_done;
wire sccb_rb_error;
wire [15:0] sccb_rb_chip_id;
wire [7:0] sccb_rb_3007;
wire [7:0] sccb_rb_3017;
wire [7:0] sccb_rb_3008;
wire [7:0] sccb_rb_3018;
wire [7:0] sccb_rb_4740;
wire [7:0] sccb_rb_4300;
wire [7:0] sccb_rb_501f;
wire [15:0] sccb_rb_output_width;
wire [15:0] sccb_rb_output_height;
wire [127:0] sccb_rb_timing_readback;
wire [31:0] sccb_rb_lsc_readback;
wire diag_req_toggle;
wire sccb_rb_awb_done_toggle;
wire sccb_rb_awb_valid;
wire [2:0] sccb_rb_awb_error_code;
wire [79:0] sccb_rb_awb_data;
// OV5640 is the SCCB slave: actively drive SIO_C high/low. The write/read
// masters actively drive SIO_D data and release it for D-C/read-data phases.
// Configuration writes and readback execute in distinct, non-overlapping phases.
assign cam_scl = camera_scl_int && sccb_rb_scl_out;
assign cam_sda = sccb_rb_sda_oe ? sccb_rb_sda_out : 1'bz;
wire camera_power_ready;
wire camera_init_done;
wire [8:0] camera_reg_index;
wire camera_clock_20k;
wire camera_ack_error;
wire [15:0] camera_ack_error_count;
wire [8:0] camera_last_error_index;
reg camera_dvp_qualified_latched = 1'b0;
ov5640_power_ctrl u_camera_power (
    .sys_clk(sys_clk), .rst_n(rst_in),
    .cam_reset_n(cam_reset_n), .cam_pwdn(cam_pwdn),
    .camera_power_ready(camera_power_ready)
);
reg_config_raw1080_candidate u_camera_config (
    .clk_25M(sys_clk),
    .camera_rstn(cam_reset_n),
    .initial_en(camera_power_ready),
    .reg_conf_done(camera_init_done),
    .i2c_sclk(camera_scl_int),
    .i2c_sdat(cam_sda),
    .clock_20k(camera_clock_20k),
    .reg_index(camera_reg_index),
    .camera_ack_error(camera_ack_error),
    .ack_error_count(camera_ack_error_count),
    .last_error_index(camera_last_error_index)
);
ov5640_sccb_readback u_camera_sccb_readback (
    .clk(sys_clk), .rst_n(sys_diag_rst_n), .start(camera_init_done), .sda_in(cam_sda),
    .awb_req_toggle(diag_req_toggle),
    .scl_out(sccb_rb_scl_out), .sda_out(sccb_rb_sda_out), .sda_oe(sccb_rb_sda_oe),
    .busy(sccb_rb_busy), .done(sccb_rb_done), .error(sccb_rb_error),
    .chip_id(sccb_rb_chip_id), .clock_enable(sccb_rb_3007),
    .pad_output_enable(sccb_rb_3017), .system_control(sccb_rb_3008),
    .pad_output_enable2(sccb_rb_3018), .polarity_control(sccb_rb_4740),
    .format_control(sccb_rb_4300), .isp_format_control(sccb_rb_501f),
    .output_width(sccb_rb_output_width), .output_height(sccb_rb_output_height),
    .timing_readback(sccb_rb_timing_readback),
    .lsc_readback(sccb_rb_lsc_readback),
    .awb_done_toggle(sccb_rb_awb_done_toggle),
    .awb_valid(sccb_rb_awb_valid), .awb_error_code(sccb_rb_awb_error_code),
    .awb_data(sccb_rb_awb_data)
);
assign camera_init_done_led = camera_init_done;
// LED2 is the SCCB result indicator.  DVP activity remains on hdmi_int_led;
// keeping these independent distinguishes a camera ACK problem from a
// camera-output timing problem during board bring-up.
assign camera_init_pass_led = sccb_rb_done && !sccb_rb_error;

// HDMI TX-only initialization. There is no HDMI RX device in this camera path.
wire tx_init_over;
wire [7:0] tx_device_id;
wire tx_iic_trig;
wire tx_w_r;
wire [15:0] tx_addr;
wire [7:0] tx_data_in;
wire tx_busy;
wire [7:0] tx_data_out;
wire tx_byte_over;
wire tx_scl;
wire tx_sda_in;
wire tx_sda_out;
wire tx_sda_out_en;
ms7210_ctl u_ms7210_tx (
    .clk(cfg_clk), .rstn(rstn_out), .init_over(tx_init_over), .device_id(tx_device_id),
    .iic_trig(tx_iic_trig), .w_r(tx_w_r), .addr(tx_addr), .data_in(tx_data_in),
    .busy(tx_busy), .data_out(tx_data_out), .byte_over(tx_byte_over)
);
iic_dri #(
    .CLK_FRE(27'd10_000_000), .IIC_FREQ(20'd400_000), .T_WR(10'd1),
    .ADDR_BYTE(2'd2), .LEN_WIDTH(8'd3), .DATA_BYTE(2'd1)
) u_hdmi_tx_iic (
    .clk(cfg_clk), .rstn(rstn_out), .pluse(tx_iic_trig), .device_id(tx_device_id),
    .w_r(tx_w_r), .byte_len(4'd1), .addr(tx_addr), .data_in(tx_data_in),
    .busy(tx_busy), .byte_over(tx_byte_over), .data_out(tx_data_out), .scl(tx_scl),
    .sda_in(tx_sda_in), .sda_out(tx_sda_out), .sda_out_en(tx_sda_out_en)
);
assign hd_scl = tx_scl;
assign hd_sda = tx_sda_out_en ? tx_sda_out : 1'bz;
assign tx_sda_in = hd_sda;

// Declare this net before the generate block so the DVP diagnostic and the
// camera capture path both resolve to the same module-level reset net.
wire capture_rst_n;
wire [2:0] capture_reset_state;
assign capture_rst_n = capture_reset_state[2];
// These module-level nets are referenced from g_camera_dvp_led below. Keep
// declarations ahead of that generate block: PDS otherwise creates scoped
// implicit nets for the diagnostic taps and ties those taps low.
wire video_rst_n;
wire [2:0] video_reset_state;
wire video_reset_qual;
wire de_o;
wire diag_rd_cmd_en,diag_rd_cmd_ready,diag_rd_cmd_done;
wire [2:0] storage_ddr_reset_state;
wire storage_ddr_rst_n /* synthesis syn_maxfan=128 */ = storage_ddr_reset_state[2];
wire storage_claim_valid, storage_claim_ready;
wire [1:0] storage_claim_slot;
wire [63:0] storage_desc_base_addr;
wire [31:0] storage_desc_frame_id, storage_desc_source_frame_id;
wire [15:0] storage_desc_cfg_version, storage_desc_width, storage_desc_height;
wire [31:0] storage_desc_stride_bytes;
wire [7:0] storage_desc_format;
wire [31:0] storage_desc_process_cycles;
wire [2:0] storage_desc_state;
wire storage_desc_valid, storage_release_valid, storage_release_ready;
wire [1:0] storage_release_slot;
wire storage_fetch_req_valid, storage_fetch_req_ready;
wire [63:0] storage_fetch_req_addr;
wire [31:0] storage_fetch_req_len;
wire [7:0] storage_fetch_req_id;
wire storage_fetch_rsp_valid, storage_fetch_rsp_ready, storage_fetch_rsp_last, storage_fetch_rsp_error;
wire [255:0] storage_fetch_rsp_data;
wire [7:0] storage_fetch_rsp_id;
wire storage_result_px_valid, storage_result_px_ready;
wire [23:0] storage_result_px_rgb;
wire [15:0] storage_result_px_x, storage_result_px_y;
wire storage_result_px_sof, storage_result_px_eol;
wire [31:0] storage_result_px_source_frame_id, storage_result_px_pixel_tag;
wire storage_algorithm_frame_error;
wire [15:0] storage_result_px_cfg_version;
wire storage_display_pixel_valid, storage_display_scan_done;
wire [23:0] storage_display_pixel_rgb;
wire [15:0] storage_display_pixel_x, storage_display_pixel_y;
wire storage_display_pixel_sof, storage_display_pixel_eol, storage_display_pixel_eof;
wire [31:0] storage_display_frame_id, storage_display_source_frame_id;
wire [15:0] storage_display_cfg_version;
wire [31:0] storage_source_frame_commit_count, storage_source_frame_drop_count;
wire [31:0] storage_source_fifo_overflow_count, storage_result_commit_count;
wire [31:0] storage_source_abort_cause_counts; // R7_OBSERVER both writer clocks=core_clk
wire [31:0] storage_raw_protocol_errors; // R7_OBSERVER local core_clk
wire [31:0] storage_raw_abort_branches,storage_raw_abort_detail; // R8_OBSERVER core_clk
wire [31:0] storage_result_drop_count, storage_result_fifo_stall_count;
wire [31:0] storage_result_protocol_error_count, storage_display_repeat_count;
wire [31:0] storage_display_reader_underflow_count, storage_display_scan_overrun_count;
wire [31:0] storage_display_stream_source_stall_count, storage_display_stream_underflow_count;
wire [31:0] storage_bypass_error_count;
wire [3:0] storage_bypass_debug_state;
wire [10:0] storage_bypass_debug_x, storage_bypass_debug_y;
wire [3:0] storage_source_writer_state;
wire storage_source_writer_alloc_ready, storage_source_writer_frame_owned;
wire [8:0] storage_source_slot_state_debug;
wire [14:0] storage_source_writer_debug = {
    storage_source_slot_state_debug,
    storage_source_writer_frame_owned,
    storage_source_writer_alloc_ready,
    storage_source_writer_state
};
reg [31:0] storage_source_commit_count_d;
reg storage_source_commit_pulse;
reg tx_init_sync1;
reg tx_init_sync2;
wire camera_de,camera_line,camera_frame;
wire [15:0] camera_rgb565;
wire camera_norm_frame,camera_norm_line,camera_norm_valid,camera_norm_pad;
wire [15:0] camera_norm_rgb565;
// Declare all diagnostic taps before the first generate block. PDS can
// otherwise create generate-scoped implicit nets for these cross-block paths.
wire selected_video_de;
wire [15:0] selected_video_rgb565;
wire [23:0] selected_video_rgb888;
wire [23:0] color_corrected_rgb888;
generate
    if ((VIDEO_SOURCE != 3) || !SPATIAL_COLOR_ENABLE) begin : g_unused_color_rgb888
        assign color_corrected_rgb888=24'd0;
    end
endgenerate
wire [11:0] act_x;
wire [11:0] act_y;
wire vs_reg;
wire hs_reg;
wire rd_en;
assign video_rst_n = video_reset_state[2];

generate
    if (VIDEO_SOURCE == 1) begin : g_ddr_test_led
        // Stage 2 deliberately has no camera timing path; the LED is kept
        // low so raw DVP inputs cannot create an unrelated cam_pclk hold path.
        // This source has no A5 diagnostic request producer.
        assign diag_req_toggle = 1'b0;
        assign hdmi_int_led = 1'b0;
    end else begin : g_camera_dvp_led
        // Camera diagnostic is retained for the OV5640 source stage only.
        // LED3 is the first physical checkpoint in the DVP path: it latches
        // as soon as any cam_pclk edge is observed, independent of HREF or
        // data polarity.  Frame/line/data qualification remains in the
        // camera_dvp_diag counters below.
        reg       dvp_pclk_seen_cam = 1'b0;
        reg       dvp_pclk_sync1    = 1'b0;
        reg       dvp_pclk_sync2    = 1'b0;
        wire      diag_pclk_seen_sys;
        wire [31:0] diag_pclk_transition_count_sys;

        cam_pclk_input_probe u_cam_pclk_input_probe (
            .sys_clk(sys_clk), .rst_n(sys_diag_rst_n), .async_pclk(cam_pclk),
            .pclk_seen_sys(diag_pclk_seen_sys),
            .transition_count_sys(diag_pclk_transition_count_sys)
        );

        always @(posedge cam_pclk) begin
            dvp_pclk_seen_cam <= 1'b1;
        end
        always @(posedge sys_clk) begin
            dvp_pclk_sync1 <= dvp_pclk_seen_cam;
            dvp_pclk_sync2 <= dvp_pclk_sync1;
        end
        assign hdmi_int_led = dvp_pclk_sync2 | diag_pclk_seen_sys;

        wire        diag_ack_toggle_cam;
        wire [31:0] diag_snap_frame_count;
        wire [31:0] diag_snap_pclk_count;
        wire [15:0] diag_snap_frame_lines;
        wire [15:0] diag_snap_last_line_bytes;
        wire [15:0] diag_snap_min_line_bytes;
        wire [15:0] diag_snap_max_line_bytes;
        wire [15:0] diag_snap_last_pair_count;
        wire [1:0] diag_snap_byte_phase;
        wire [15:0] diag_snap_first_bytes;
        wire [31:0] diag_snap_data_sum;
        wire [31:0] diag_snap_data_xor;
        wire [31:0] diag_snap_capture_pixel_count;
        wire [31:0] diag_snap_norm_pixel_count;
        wire [31:0] diag_snap_norm_pad_count;
        wire [31:0] diag_snap_norm_line_count;
        wire [4:0] diag_fresh_mask;
        wire diag_snapshot_valid;
        wire [655:0] diag_ddr_snapshot;
        wire [138:0] diag_pix_snapshot;
        wire diag_ack_toggle_ddr;
        wire diag_ack_toggle_pix;
        wire [31:0] diag_read_request_count;
        wire [31:0] diag_arvalid_cycle_count;
        wire [31:0] diag_ar_handshake_count;
        wire [31:0] diag_busy_request_count;
        wire [4:0] diag_status_sync;
        wire [11:0] diag_camera_x, diag_camera_y;
        wire diag_camera_frame_start;
        wire [255:0] diag_camera_roi_snapshot;
        wire diag_ack_toggle_cam_color;
        wire [255:0] diag_display_roi_snapshot;
        wire diag_ack_toggle_disp_color;
        wire diag_display_frame_start;
        reg diag_display_vs_d;
        wire diag_video_reset_qual = (VIDEO_SOURCE == 0) ?
            (diag_status_sync[0] && diag_status_sync[1] && tx_init_sync2) :
            (diag_status_sync[0] && diag_status_sync[1] &&
             diag_status_sync[2] && diag_status_sync[3]);

        camera_readpath_diag u_camera_readpath_diag (
            .core_clk(core_clk), .ddr_init_done(ddr_init_done),
            .rd_cmd_en(diag_rd_cmd_en), .rd_cmd_ready(diag_rd_cmd_ready),
            .axi_arvalid(axi_arvalid),
            .axi_arready(axi_arready),
            .rd_request_count(diag_read_request_count),
            .arvalid_cycle_count(diag_arvalid_cycle_count),
            .ar_handshake_count(diag_ar_handshake_count),
            .busy_request_count(diag_busy_request_count)
        );

        camera_diag_status_sync u_camera_diag_status_sync (
            .pix_clk(pix_clk), .rst_n(rst_in),
            .status_async({video_reset_state[2], init_done, ddr_init_done, locked, rst_in}),
            .status_sync(diag_status_sync)
        );

        // DDR-domain counters describe completed AXI activity, not merely
        // requests issued by the frame-buffer controller.
        always @(posedge core_clk) begin
            if (!ddr_init_done) begin
                diag_axi_wlast_count <= 32'd0;
                diag_frame_commit_count <= 32'd0;
                diag_axi_rvalid_count <= 32'd0;
            end else begin
                // The PG2L DDR user port has no WVALID input. WREADY gates
                // accepted data beats and WUSERO_LAST marks their burst end.
                if (axi_wready && axi_wusero_last)
                    diag_axi_wlast_count <= diag_axi_wlast_count + 1'b1;
                if (frame_commit_pulse)
                    diag_frame_commit_count <= diag_frame_commit_count + 1'b1;
                if (axi_rvalid)
                    diag_axi_rvalid_count <= diag_axi_rvalid_count + 1'b1;
            end
        end

        // Pixel-domain frame totals distinguish framebuffer DE from the
        // actual HDMI DE after video selection. Edge totals are cumulative
        // since video reset, while the DE totals are latched per VS frame.
        reg [31:0] diag_deo_running;
        reg [31:0] diag_deout_running;
        reg [31:0] diag_deo_last_frame;
        reg [31:0] diag_deout_last_frame;
        reg [31:0] diag_hs_rise_count;
        reg [31:0] diag_vs_rise_count;
        reg diag_hs_out_d;
        reg diag_vs_out_d;
        always @(posedge pix_clk) begin
            if (!rst_in || !locked || !video_rst_n) begin
                diag_deo_running <= 32'd0;
                diag_deout_running <= 32'd0;
                diag_deo_last_frame <= 32'd0;
                diag_deout_last_frame <= 32'd0;
                diag_hs_rise_count <= 32'd0;
                diag_vs_rise_count <= 32'd0;
                diag_hs_out_d <= 1'b0;
                diag_vs_out_d <= 1'b0;
            end else begin
                if (hs_out && !diag_hs_out_d)
                    diag_hs_rise_count <= diag_hs_rise_count + 1'b1;
                if (vs_out && !diag_vs_out_d) begin
                    diag_vs_rise_count <= diag_vs_rise_count + 1'b1;
                    diag_deo_last_frame <= diag_deo_running + (de_o ? 32'd1 : 32'd0);
                    diag_deout_last_frame <= diag_deout_running + (de_out ? 32'd1 : 32'd0);
                    diag_deo_running <= 32'd0;
                    diag_deout_running <= 32'd0;
                end else begin
                    if (de_o)
                        diag_deo_running <= diag_deo_running + 1'b1;
                    if (de_out)
                        diag_deout_running <= diag_deout_running + 1'b1;
                end
                diag_hs_out_d <= hs_out;
                diag_vs_out_d <= vs_out;
            end
        end

        camera_diag_snapshot_cdc #(.WIDTH(656)) u_diag_ddr_snapshot (
            .src_clk(core_clk), .src_rst_n(rst_in),
            .request_toggle_async(diag_req_toggle),
            .counter_bus({v04_applied_count,v04_rejected_count,v04_received_count,
                          16'd0,v04_active_version,v04_applied_frame,v04_applied_seq,v04_received_seq,
                          28'd0,video_result_idle,storage_algorithm_frame_error,v04_cfg_ready,v04_cfg_valid,
                          {storage_bypass_error_count[5:0],storage_bypass_debug_y,storage_bypass_debug_x,storage_bypass_debug_state},storage_result_fifo_stall_count,
                          storage_source_abort_cause_counts, // R7_OBSERVER same core_clk as held snapshot
                          storage_source_frame_drop_count,
                          storage_source_fifo_overflow_count[29:0],
                          diag_rd_cmd_done, diag_rd_cmd_ready,
                          storage_result_commit_count,
                          storage_source_writer_debug,
                          storage_raw_abort_detail, storage_raw_abort_branches, // R8_OBSERVER local core_clk
                          storage_raw_protocol_errors, diag_axi_rvalid_count, // R7_OBSERVER same core_clk
                          diag_frame_commit_count, diag_axi_wlast_count, init_done}),
            .snapshot_bus(diag_ddr_snapshot), .ack_toggle(diag_ack_toggle_ddr)
        );
        camera_diag_snapshot_cdc #(.WIDTH(139)) u_diag_pix_snapshot (
            .src_clk(pix_clk), .src_rst_n(rst_in),
            .request_toggle_async(diag_req_toggle),
            .counter_bus({diag_status_sync[3:0], video_reset_state[2:0], diag_video_reset_qual,
                          diag_vs_rise_count, diag_hs_rise_count,
                          diag_deout_last_frame, diag_deo_last_frame,
                          de_out, de_o, diag_status_sync[4]}),
            .snapshot_bus(diag_pix_snapshot), .ack_toggle(diag_ack_toggle_pix)
        );
        camera_diag_snapshot_status u_diag_snapshot_status (
            .sys_clk(sys_clk), .rst_n(sys_diag_rst_n),
            .request_toggle(diag_req_toggle),
            .ack_toggle_cam_async(diag_ack_toggle_cam),
            .ack_toggle_ddr_async(diag_ack_toggle_ddr),
            .ack_toggle_pix_async(diag_ack_toggle_pix),
            .ack_toggle_cam_color_async(diag_ack_toggle_cam_color),
            .ack_toggle_disp_color_async(diag_ack_toggle_disp_color),
            .fresh_mask(diag_fresh_mask), .snapshot_valid(diag_snapshot_valid)
        );

        camera_norm_xy_tracker u_camera_norm_xy_tracker (
            .clk(cam_pclk), .rst_n(capture_rst_n),
            .frame_active(camera_norm_frame), .line_active(camera_norm_line),
            .pixel_valid(camera_norm_valid), .frame_start(diag_camera_frame_start),
            .pixel_x(diag_camera_x), .pixel_y(diag_camera_y)
        );

        camera_rgb565_roi_diag u_diag_camera_roi (
            .clk(cam_pclk), .rst_n(capture_rst_n),
            .frame_start(diag_camera_frame_start), .pixel_valid(camera_norm_valid),
            .pixel_x(diag_camera_x), .pixel_y(diag_camera_y),
            .pixel_rgb565(camera_norm_rgb565),
            .request_toggle_async(diag_req_toggle),
            .snapshot_bus(diag_camera_roi_snapshot),
            .ack_toggle(diag_ack_toggle_cam_color)
        );

        // The pixel-domain VS rising edge starts a new output frame before
        // active DE. Measure the selected RGB565 stream, before RGB888 expansion.
        always @(posedge pix_clk or negedge video_rst_n) begin
            if (!video_rst_n)
                diag_display_vs_d <= 1'b0;
            else
                diag_display_vs_d <= vs_reg;
        end
        assign diag_display_frame_start = vs_reg && !diag_display_vs_d;

        camera_rgb565_roi_diag u_diag_display_roi (
            .clk(pix_clk), .rst_n(video_rst_n),
            .frame_start(diag_display_frame_start), .pixel_valid(selected_video_de),
            .pixel_x(act_x), .pixel_y(act_y),
            .pixel_rgb565(selected_video_rgb565),
            .request_toggle_async(diag_req_toggle),
            .snapshot_bus(diag_display_roi_snapshot),
            .ack_toggle(diag_ack_toggle_disp_color)
        );

        // Use the camera-domain synchronized reset shared with the capture
        // path.  rstn_out is released by cfg_clk and must not asynchronously
        // reset registers clocked by cam_pclk.
        camera_dvp_diag #(.HREF_ACTIVE_LEVEL(1'b0)) u_camera_dvp_diag (
            .cam_pclk(cam_pclk), .rst_n(capture_rst_n),
            .cam_vsync(dvp_sample_vsync), .cam_href(dvp_sample_href), .cam_data(dvp_sample_data),
            .capture_valid(camera_de),
            .norm_valid(camera_norm_valid), .pad_valid(camera_norm_pad),
            .norm_line(camera_norm_line),
            .req_toggle_sys(diag_req_toggle), .ack_toggle_cam(diag_ack_toggle_cam),
            .snap_frame_count(diag_snap_frame_count),
            .snap_pclk_count(diag_snap_pclk_count),
            .snap_frame_lines(diag_snap_frame_lines),
            .snap_last_line_bytes(diag_snap_last_line_bytes),
            .snap_min_line_bytes(diag_snap_min_line_bytes),
            .snap_max_line_bytes(diag_snap_max_line_bytes),
            .snap_last_pair_count(diag_snap_last_pair_count),
            .snap_byte_phase(diag_snap_byte_phase),
            .snap_first_bytes(diag_snap_first_bytes),
            .snap_data_sum(diag_snap_data_sum),
            .snap_data_xor(diag_snap_data_xor),
            .snap_capture_pixel_count(diag_snap_capture_pixel_count),
            .snap_norm_pixel_count(diag_snap_norm_pixel_count),
            .snap_norm_pad_count(diag_snap_norm_pad_count),
            .snap_norm_line_count(diag_snap_norm_line_count)
        );

        // UART payload: 61 little-endian 32-bit words (244 bytes).
        // W0 status: bit9=capture reset released, bit8=sys_clk sampled PCLK transition seen,
        // bit7=cam_pclk clock-domain edge seen, bit6=HDMI init, bit5=config table done,
        // bit4=DDR init, bit3=register readback mismatch, bit2=power ready, bit1=reset
        // release, bit0=cfg PLL lock. W3 counts sys_clk-sampled transitions.
        reg [1951:0] diag_payload;
        always @(*) begin
            diag_payload = 1952'd0;
            diag_payload[31:0]   = {22'd0, sys_diag_status[0], diag_pclk_seen_sys, dvp_pclk_sync2, sys_diag_status[1],
                                     camera_init_done, sys_diag_status[2],
                                     sccb_rb_error, camera_power_ready,
                                     sys_diag_rst_n, sys_diag_status[3]};
            diag_payload[63:32]  = diag_snap_pclk_count;
            diag_payload[95:64]  = diag_snap_frame_count;
            diag_payload[127:96] = diag_pclk_transition_count_sys;
            // W4..W7: SCCB readback of chip ID, 0x3007, 0x3017, 0x3008.
            diag_payload[159:128] = {16'd0, sccb_rb_chip_id};
            diag_payload[191:160] = {24'd0, sccb_rb_3007};
            diag_payload[223:192] = {24'd0, sccb_rb_3017};
            diag_payload[255:224] = {24'd0, sccb_rb_3008};
            // W8 bytes: 0x4740, 0x4300, 0x501F, 0x3018 readback, in that order.
            diag_payload[287:256] = {sccb_rb_3018, sccb_rb_501f,
                                     sccb_rb_4300, sccb_rb_4740};
            // W9 bit0=readback complete, bit1=register readback mismatch.
            diag_payload[319:288] = {30'd0, sccb_rb_error, sccb_rb_done};
            // W10: low half=output width, high half=output height.
            diag_payload[351:320] = {sccb_rb_output_height, sccb_rb_output_width};
            // W11: low half=last active-HREF line byte count,
            // high half=lines/frame.
            diag_payload[383:352] = {diag_snap_frame_lines, diag_snap_last_line_bytes};
            // W12: low/high halves=min/max active-HREF line byte count.
            diag_payload[415:384] = {diag_snap_max_line_bytes, diag_snap_min_line_bytes};
            // W13: low half=first two sampled bytes, high half=last line RGB565 pairs.
            diag_payload[447:416] = {diag_snap_last_pair_count,
                                     diag_snap_first_bytes};
            // W14: low/high halves=low 16 bits of data sum/XOR.
            diag_payload[479:448] = {diag_snap_data_xor[15:0], diag_snap_data_sum[15:0]};
            // W15 SDA6: RAW diagnostics plus frame-atomic v0.4 configuration observations.
            diag_payload[511:480] = 32'h36414453;
            // W16..W19: raw crop/timing/subsample/test-pattern readbacks.
            // Bytes in payload order: 3800..3807, 380C..380F, 3814,3815,503D,00.
            diag_payload[543:512] = sccb_rb_timing_readback[31:0];
            diag_payload[575:544] = sccb_rb_timing_readback[63:32];
            diag_payload[607:576] = sccb_rb_timing_readback[95:64];
            diag_payload[639:608] = {sccb_rb_timing_readback[127:120],
                                      sccb_rb_timing_readback[119:96]};
            // W20: completed frame's RGB565 pixels accepted after byte unpacking.
            diag_payload[671:640] = diag_snap_capture_pixel_count;
            // W21..W23: normalized valid pixels, black padding pixels, and rows.
            diag_payload[703:672] = diag_snap_norm_pixel_count;
            diag_payload[735:704] = diag_snap_norm_pad_count;
            diag_payload[767:736] = diag_snap_norm_line_count;
            // W24 status: bits0..3 retain framebuffer/video/DE meanings;
            // bits4..8 are fresh camera/DDR/pixel/camera-color/display-color flags.
            diag_payload[799:768] = 32'd0;
            diag_payload[768] = diag_ddr_snapshot[0];
            diag_payload[769] = diag_pix_snapshot[0];
            diag_payload[770] = diag_pix_snapshot[1];
            diag_payload[771] = diag_pix_snapshot[2];
            diag_payload[772] = diag_fresh_mask[0];
            diag_payload[773] = diag_fresh_mask[1];
            diag_payload[774] = diag_fresh_mask[2];
            diag_payload[775] = 1'b0; // RGB565 camera ROI is not RAW color calibration
            diag_payload[776] = diag_fresh_mask[4];
            // W25..W27: accepted AXI write-last, committed frames, AXI read beats.
            diag_payload[831:800] = diag_ddr_snapshot[32:1];
            diag_payload[863:832] = diag_ddr_snapshot[64:33];
            diag_payload[895:864] = diag_ddr_snapshot[96:65];
            // R7 W28=source abort modular8 counters: notready/short/excess/lategrant. W29=HDMI DE.
            diag_payload[927:896] = diag_ddr_snapshot[335:304];
            diag_payload[959:928] = diag_pix_snapshot[66:35];
            // W30=result writer pixel FIFO backpressure; W31=VSYNC rising edges.
            diag_payload[991:960] = diag_ddr_snapshot[367:336];
            diag_payload[1023:992] = diag_pix_snapshot[130:99];
            // W32 synchronized reset/clock prerequisites: bit0 rst_in,
            // bit1 PLL locked, bit2 DDR init, bit3 framebuffer init,
            // bit4 pixel/video reset released. The video-reset status is
            // carried once at snapshot bit0 and reordered here to avoid an
            // aliased duplicate in the bundled-data bus.
            diag_payload[1055:1024] = {27'd0, diag_pix_snapshot[0],
                                        diag_pix_snapshot[138:135]};
            // R8 W33=RAW protocol; W34=packet/newSOF/dirtyEND/shortEND modular8 counts.
            // W35=last abort {2zero,22bit input_tag,4END flags,short,dirty,newSOF,packet}.
            diag_payload[1087:1056] = diag_ddr_snapshot[128:97];
            diag_payload[1119:1088] = diag_ddr_snapshot[160:129];
            diag_payload[1151:1120] = diag_ddr_snapshot[192:161];
            // W36 source writer state/slot debug; W37 result frame commits.
            diag_payload[1183:1152] = {17'd0, diag_ddr_snapshot[207:193]};
            diag_payload[1215:1184] = diag_ddr_snapshot[239:208];
            // W38: algorithm state[3:0], x[10:0], y[10:0], low 6 error bits (mod64).
            diag_payload[1247:1216] = diag_ddr_snapshot[399:368];
            // W39=source frame drops; W40={FIFO overflows[29:0], read done, ready}.
            diag_payload[1279:1248] = diag_ddr_snapshot[303:272];
            diag_payload[1311:1280] = diag_ddr_snapshot[271:240];
            // W41 bytes are SCCB readbacks 5000, 5001, 5800, and 583D.
            // Expected HEX order: A7 83 23 CE (LSC enabled, AWB enabled).
            diag_payload[1343:1312] = sccb_rb_lsc_readback;

            // SDA6 W42..49: core-clock algorithm observations from the held snapshot.
            // W24 DDR-fresh bit qualifies all eight words, including sequence/version.
            diag_payload[1599:1344] = diag_ddr_snapshot[655:400];
            // W50-W57: same packed records for the selected display RGB565
            // stream before RGB888 expansion; W24 bit8 is its freshness bit.
            if (diag_fresh_mask[4]) begin
                diag_payload[1855:1600] = diag_display_roi_snapshot;
            end
            // W58-W60: fresh AWB register values, low byte first.
            diag_payload[1887:1856] = sccb_rb_awb_data[31:0];
            diag_payload[1919:1888] = sccb_rb_awb_data[63:32];
            diag_payload[1951:1920] = {5'd0, sccb_rb_awb_error_code, 7'd0,
                                        sccb_rb_awb_valid, sccb_rb_awb_data[79:64]};
        end

        camera_diag_uart u_camera_diag_uart (
            .clk(sys_clk), .rst_n(sys_diag_rst_n), .uart_rx(v04_diag_rx), .uart_tx(uart_tx),
            .snapshot_req_toggle(diag_req_toggle),
            .snapshot_valid(diag_snapshot_valid), .snapshot_payload(diag_payload),
            .awb_done_toggle(sccb_rb_awb_done_toggle),
            .nn_snapshot_payload(nn_telemetry_held),.nn_ack_toggle_async(nn_telemetry_ack)
        );
    end
endgenerate

    always @(posedge cfg_clk)
    begin
    	if(!rst_in || !locked || !cfg_locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    



    reg    rstn_d0;
    reg    rstn_d1;




    assign rstn_out = (rstn_1ms == 16'h2710);

// Capture reset deasserts synchronously after SCCB and DDR are ready.
reset_sync u_capture_reset(.clk(cam_pclk),
 .arst_n(rst_in && sccb_rb_done && !sccb_rb_error && ddr_init_done),
 .sync_state(capture_reset_state));
// RAW8 camera diagnostics count one byte per pixel, not RGB565 pairs.
wire [7:0] diagnostic_raw;
wire diagnostic_raw_valid;
raw8_dvp_capture_r2 #(.WIDTH(1920),.HEIGHT(1080)) u_capture(
 .pclk(cam_pclk),.rst_n(capture_rst_n),.vsync(dvp_sample_vsync),.href(dvp_sample_href),.data(dvp_sample_data),
 .raw_data(diagnostic_raw),.pixel_valid(diagnostic_raw_valid),.frame_active(camera_frame),
 .sof(),.eol(),.eof(),.px_x(),.px_y(),.error_count());
assign camera_rgb565={diagnostic_raw[7:3],diagnostic_raw[7:2],diagnostic_raw[7:3]};
assign camera_de=diagnostic_raw_valid;
// The diagnostic pixel stream is already registered by raw8 capture.
// Its valid is also the aligned row-active qualifier; do not feed raw VSYNC
// through the normalizer's multi-level CE/data path.
assign camera_line=diagnostic_raw_valid;
camera_line_normalizer #(.H_ACTIVE(1920)) u_camera_line_normalizer (
 .pclk(cam_pclk), .rst_n(capture_rst_n),
 .frame_in(camera_frame), .line_in(camera_line),
 .valid_in(camera_de), .pixel_in(camera_rgb565),
 .frame_out(camera_norm_frame), .line_out(camera_norm_line),
 .valid_out(camera_norm_valid), .pad_valid(camera_norm_pad),
 .pixel_out(camera_norm_rgb565)
);
// Legacy paths retain diagnostics only; native source pixels come from
// the independent RAW->RGB core below, already in the DDR clock domain.
wire frame_write_clk=cam_pclk;
wire frame_write_fsync=camera_norm_frame;
wire frame_write_en=camera_norm_valid;
wire frame_write_line=camera_norm_line;
wire [15:0] frame_write_data=camera_norm_rgb565;
wire pll_lock,phy_pll_lock,gpll_lock,rst_gpll_lock,ddrphy_cpd_lock;
wire source_pattern_de;
wire [15:0] source_pattern_rgb565;
wire source_ddr_de;
wire [15:0] source_ddr_rgb565;
wire source_camera_de;
wire [15:0] source_camera_rgb565;
assign video_reset_qual = (VIDEO_SOURCE == 0) ?
    (rst_in && locked && tx_init_sync2) :
    (rst_in && locked && ddr_init_done && init_done);

always @(posedge pix_clk) begin
    if (!rst_in) begin
        tx_init_sync1 <= 1'b0;
        tx_init_sync2 <= 1'b0;
    end else begin
        tx_init_sync1 <= tx_init_over;
        tx_init_sync2 <= tx_init_sync1;
    end
end
// Keep the video reset independent of the slow HDMI-I2C configuration clock;
// cfg_clk is asynchronous to pix_clk and must not become a timing launch.
generate
    if (VIDEO_SOURCE == 0) begin : g_hdmi_only_reset
        reset_sync u_video_reset(.clk(pix_clk),
            .arst_n(video_reset_qual), .sync_state(video_reset_state));
    end else begin : g_framebuffer_reset
        reset_sync u_video_reset(.clk(pix_clk),
            .arst_n(video_reset_qual), .sync_state(video_reset_state));
    end
endgenerate
reg [1:0] hs_pipe=0,vs_pipe=0;
always @(posedge pix_clk) begin hs_pipe<={hs_pipe[0],hs_reg};vs_pipe<={vs_pipe[0],vs_reg};end

 

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// The validated baseline keeps fram_buf as its DDR owner. VIDEO_SOURCE=3
// selects the independent storage-core candidate with a temporary identity
// readback engine between the source and result pools.
generate
    if (VIDEO_SOURCE != 3) begin : g_legacy_framebuffer
    assign gamma_observation = 128'd0;
    fram_buf #(.H_NUM(12'd1280), .V_NUM(12'd720)) fram_buf(
        .ddr_clk        (  core_clk             ),//input                         ddr_clk,
        .ddr_rstn       (  ddr_init_done        ),//input                         ddr_rstn,
        //data_in                                  
        .vin_clk        (frame_write_clk),//input                         vin_clk,
        .wr_fsync       (frame_write_fsync),//input                         wr_fsync,
        .wr_en          (frame_write_en),
        .wr_line        (frame_write_line),//input                         wr_en,
        .wr_data        (frame_write_data),//input  [15 : 0]  wr_data,
        //data_out
        .vout_clk       (  pix_clk              ),//input                         vout_clk,
        .rd_fsync       (  vs_reg               ),//input                         rd_fsync,
        .rd_en          (  rd_en                ),//input                         rd_en,
        .vout_de        (  de_o               ),//output                        vout_de,
        .vout_data      (  o_rgb565             ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        .init_done      (  init_done            ),//output reg                    init_done,
        .frame_commit_pulse ( frame_commit_pulse ),
        .diag_rd_cmd_en (  diag_rd_cmd_en       ),
        .diag_rd_cmd_ready ( diag_rd_cmd_ready   ),
        .diag_rd_cmd_done ( diag_rd_cmd_done     ),
        //axi bus
        .axi_awaddr     (  axi_awaddr           ),// output[27:0]
        .axi_awid       (  axi_awuser_id        ),// output[3:0]
        .axi_awlen      (  axi_awlen            ),// output[3:0]
        .axi_awsize     (                       ),// output[2:0]
        .axi_awburst    (                       ),// output[1:0]
        .axi_awready    (  axi_awready          ),// input
        .axi_awvalid    (  axi_awvalid          ),// output               
        .axi_wdata      (  axi_wdata            ),// output[255:0]
        .axi_wstrb      (  axi_wstrb            ),// output[31:0]
        .axi_wlast      (  axi_wusero_last      ),// input
        .axi_wvalid     (  axi_wvalid_fb        ),// output
        .axi_wready     (  axi_wready           ),// input
        .axi_bid        (  4'd0                 ),// input[3:0]
        .axi_araddr     (  axi_araddr           ),// output[27:0]
        .axi_arid       (  axi_aruser_id        ),// output[3:0]
        .axi_arlen      (  axi_arlen            ),// output[3:0]
        .axi_arsize     (                       ),// output[2:0]
        .axi_arburst    (                       ),// output[1:0]
        .axi_arvalid    (  axi_arvalid          ),// output
        .axi_arready    (  axi_arready          ),// input
        .axi_rready     (                       ),// output
        .axi_rdata      (  axi_rdata            ),// input[255:0]
        .axi_rvalid     (  axi_rvalid           ),// input
        .axi_rlast      (  axi_rlast            ),// input
        .axi_rid        (  axi_rid              ) // input[3:0]
    );
    end else begin : g_storage_bypass_framebuffer
        reset_sync u_storage_ddr_reset (
            .clk(core_clk), .arst_n(rstn_out && ddr_init_done),
            .sync_state(storage_ddr_reset_state)
        );

        wire storage_camera_valid,raw_last,raw_abort;
        wire [15:0] raw_x,raw_y;
        wire corrected_frame,corrected_valid;
        wire [23:0] corrected_rgb;
        wire [23:0] storage_camera_rgb888;
        wire [31:0] raw_capture_errors,raw_fifo_errors,raw_protocol_errors,raw_input_pixels,raw_started_frames;
        assign storage_raw_protocol_errors=raw_protocol_errors; // R7_OBSERVER local core_clk
        reg storage_camera_frame=0;
        reg [31:0] seen_raw_start=0;
        raw8_camera_rgb_candidate_r3 #(.WIDTH(1920),.HEIGHT(1080)) raw_camera(
            .pclk(cam_pclk),.clk(core_clk),.rst_n(capture_rst_n&&storage_ddr_rst_n),
            .vsync(dvp_sample_vsync),.href(dvp_sample_href),.data(dvp_sample_data),
            .out_valid(storage_camera_valid),.out_ready(1'b1),.out_rgb(storage_camera_rgb888),
            .out_x(raw_x),.out_y(raw_y),.out_frame_id(),.out_pixel_tag(),.out_cfg_version(),
            .out_last(raw_last),.out_sof(),.out_eol(),
            .capture_errors(raw_capture_errors),.fifo_overflow_errors(raw_fifo_errors),
            .protocol_errors(raw_protocol_errors),.input_pixels(raw_input_pixels),
            .started_frames(raw_started_frames),.frame_abort(raw_abort),
            .debug_abort_branches(storage_raw_abort_branches),.debug_abort_detail(storage_raw_abort_detail)); // R8_OBSERVER
        // Open a source slot before the first restored row; a dirty END token
        // drops frame_active and makes the unchanged source writer reject it.
        always @(posedge core_clk)begin
            if(!storage_ddr_rst_n)begin storage_camera_frame<=0;seen_raw_start<=0;end
            else begin
                if(raw_started_frames!=seen_raw_start)begin storage_camera_frame<=1;seen_raw_start<=raw_started_frames;end
                if(raw_abort||(storage_camera_valid&&raw_last))storage_camera_frame<=0;
            end
        end
        source_spatial_color_r1 #(
            .RED_GRID(SPATIAL_COLOR_ENABLE?COLOR_RED:{54{10'd256}}),
            .BLUE_GRID(SPATIAL_COLOR_ENABLE?COLOR_BLUE:{54{10'd256}})) u_source_color(
            .clk(core_clk),.rst_n(storage_ddr_rst_n),.frame_active_in(storage_camera_frame),
            .valid_in(storage_camera_valid),.rgb_in(storage_camera_rgb888),.x_in(raw_x),.y_in(raw_y),
            .frame_active_out(corrected_frame),.valid_out(corrected_valid),.rgb_out(corrected_rgb));
        reg [3:0] nn_raw_abort_pipe;
        always @(posedge core_clk)begin
            if(!storage_ddr_rst_n)nn_raw_abort_pipe<=0;
            else nn_raw_abort_pipe<={nn_raw_abort_pipe[2:0],raw_abort};
        end
        nn_video_sidecar #(.EXPECTED_CRC32(32'h4ce634dd)) nn_sidecar(
            .clk(core_clk),.rst_n(storage_ddr_rst_n),.frame_active(corrected_frame),
            .pixel_valid(corrected_valid),.pixel_abort(nn_raw_abort_pipe[3]),.rgb(corrected_rgb),
            .source_frame_id(raw_started_frames),
            .cmd_valid(nn_cmd_valid),.cmd_ready(nn_cmd_ready),.cmd_op(nn_cmd_op),
            .cmd_seq(nn_cmd_seq),.cmd_addr(nn_cmd_addr),.cmd_data(nn_cmd_data),.telemetry(nn_telemetry),
        .wr_req_valid(nn_wr_req_valid),
        .wr_req_ready(nn_wr_req_ready),
        .wr_req_addr(nn_wr_req_addr),
        .wr_req_len(nn_wr_req_len),
        .wr_req_id(nn_wr_req_id),
        .wr_data_valid(nn_wr_data_valid),
        .wr_data_ready(nn_wr_data_ready),
        .wr_data(nn_wr_data),
        .wr_data_last(nn_wr_data_last),
        .wr_done_valid(nn_wr_done_valid),
        .wr_done_id(nn_wr_done_id),
        .wr_done_error(nn_wr_done_error),
        .rd_req_valid(nn_rd_req_valid),
        .rd_req_ready(nn_rd_req_ready),
        .rd_req_addr(nn_rd_req_addr),
        .rd_req_len(nn_rd_req_len),
        .rd_req_id(nn_rd_req_id),
        .rd_rsp_valid(nn_rd_rsp_valid),
        .rd_rsp_ready(nn_rd_rsp_ready),
        .rd_rsp_data(nn_rd_rsp_data),
        .rd_rsp_id(nn_rd_rsp_id),
        .rd_rsp_last(nn_rd_rsp_last),
        .rd_rsp_error(nn_rd_rsp_error)
        );
        storage_pipeline_core_v04 #(
            .CAMERA_RGB888_ENABLE(1), .RESULT_FRAME_ERROR_ENABLE(1), .DISPLAY_RD_BUFFER_ENABLE(1),
            .SOURCE_FRAME_WIDTH(1920), .SOURCE_FRAME_HEIGHT(1080),
            .RESULT_FRAME_WIDTH(1920), .RESULT_FRAME_HEIGHT(1080),
            .SOURCE_FIFO_DEPTH(4096), .RESULT_FIFO_DEPTH(1024), .DISPLAY_FIFO_DEPTH(4096),
            .NUM_SOURCE_SLOTS(3), .NUM_RESULT_SLOTS(3),
            .SOURCE_POOL_BASE_ADDR(64'h00000000), .RESULT_POOL_BASE_ADDR(64'h017BB000),
            .USABLE_DDR_BASE_ADDR(64'h00000000), .USABLE_DDR_SIZE_BYTES(64'h04000000),
            .CTRL_ADDR_WIDTH(CTRL_ADDR_WIDTH), .REQ_ID_W(8), .MAX_BURST_BEATS(16),
            .VSYNC_ACTIVE_LEVEL(1)
        ) u_storage_pipeline_core (
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
            .result_processing_idle(video_result_idle),
            .cam_clk(core_clk), .cam_rst_n(storage_ddr_rst_n),
            .camera_frame_active(corrected_frame), .camera_pixel_valid(corrected_valid),
            .camera_pixel_rgb565(16'd0), .camera_pixel_rgb888(corrected_rgb),
            .camera_cfg_version(16'd7),
            .ddr_clk(core_clk), .ddr_rst_n(storage_ddr_rst_n), .ddr_init_done(ddr_init_done),
            .source_desc_claim_valid(storage_claim_valid), .source_desc_claim_ready(storage_claim_ready),
            .source_desc_claim_slot(storage_claim_slot), .source_desc_base_addr(storage_desc_base_addr),
            .source_desc_frame_id(storage_desc_frame_id), .source_desc_source_frame_id(storage_desc_source_frame_id),
            .source_desc_cfg_version(storage_desc_cfg_version), .source_desc_width(storage_desc_width),
            .source_desc_height(storage_desc_height), .source_desc_stride_bytes(storage_desc_stride_bytes),
            .source_desc_format(storage_desc_format), .source_desc_process_cycles(storage_desc_process_cycles),
            .source_desc_state(storage_desc_state), .source_desc_valid(storage_desc_valid),
            .source_desc_release_valid(storage_release_valid), .source_desc_release_slot(storage_release_slot),
            .source_desc_release_ready(storage_release_ready),
            .fetch_rd_req_valid(storage_fetch_req_valid), .fetch_rd_req_ready(storage_fetch_req_ready),
            .fetch_rd_req_addr(storage_fetch_req_addr), .fetch_rd_req_len(storage_fetch_req_len),
            .fetch_rd_req_id(storage_fetch_req_id), .fetch_rd_rsp_valid(storage_fetch_rsp_valid),
            .fetch_rd_rsp_ready(storage_fetch_rsp_ready), .fetch_rd_rsp_data(storage_fetch_rsp_data),
            .fetch_rd_rsp_id(storage_fetch_rsp_id), .fetch_rd_rsp_last(storage_fetch_rsp_last),
            .fetch_rd_rsp_error(storage_fetch_rsp_error),
            .result_clk(core_clk), .result_rst_n(storage_ddr_rst_n),
            .result_px_valid(storage_result_px_valid), .result_px_ready(storage_result_px_ready),
            .result_px_rgb(storage_result_px_rgb), .result_px_x(storage_result_px_x),
            .result_px_y(storage_result_px_y), .result_px_sof(storage_result_px_sof),
            .result_px_eol(storage_result_px_eol),
            .result_px_source_frame_id(storage_result_px_source_frame_id),
            .result_px_cfg_version(storage_result_px_cfg_version),
            .result_px_pixel_tag(storage_result_px_pixel_tag), .result_frame_error(storage_algorithm_frame_error),
            .pix_clk(pix_clk), .pix_rst_n(video_rst_n), .pix_vsync(vs_reg),
            .display_pixel_req(rd_en), .display_pixel_valid(storage_display_pixel_valid),
            .display_pixel_rgb(storage_display_pixel_rgb), .display_pixel_x(storage_display_pixel_x),
            .display_pixel_y(storage_display_pixel_y), .display_pixel_sof(storage_display_pixel_sof),
            .display_pixel_eol(storage_display_pixel_eol), .display_pixel_eof(storage_display_pixel_eof),
            .display_frame_id(storage_display_frame_id),
            .display_source_frame_id(storage_display_source_frame_id),
            .display_cfg_version(storage_display_cfg_version), .display_scan_done(storage_display_scan_done),
            .axi_awaddr(axi_awaddr), .axi_awuser_id(axi_awuser_id), .axi_awlen(axi_awlen),
            .axi_awvalid(axi_awvalid), .axi_awready(axi_awready), .axi_wdata(axi_wdata),
            .axi_wstrb(axi_wstrb), .axi_wready(axi_wready), .axi_wlast(axi_wusero_last),
            .axi_araddr(axi_araddr), .axi_aruser_id(axi_aruser_id), .axi_arlen(axi_arlen),
            .axi_arvalid(axi_arvalid), .axi_arready(axi_arready), .axi_rdata(axi_rdata),
            .axi_rid(axi_rid), .axi_rlast(axi_rlast), .axi_rvalid(axi_rvalid),
            .source_frame_commit_count(storage_source_frame_commit_count),
            .source_frame_drop_count(storage_source_frame_drop_count),
            .source_fifo_overflow_count(storage_source_fifo_overflow_count),
            .source_abort_cause_counts(storage_source_abort_cause_counts), // R7_OBSERVER
            .result_commit_count(storage_result_commit_count), .result_drop_count(storage_result_drop_count),
            .result_fifo_stall_count(storage_result_fifo_stall_count),
            .result_protocol_error_count(storage_result_protocol_error_count),
            .display_repeat_count(storage_display_repeat_count),
            .display_reader_underflow_count(storage_display_reader_underflow_count),
            .display_scan_overrun_count(storage_display_scan_overrun_count),
            .display_stream_source_stall_count(storage_display_stream_source_stall_count),
            .display_stream_underflow_count(storage_display_stream_underflow_count),
            .source_writer_state_debug(storage_source_writer_state),
            .source_writer_alloc_ready_debug(storage_source_writer_alloc_ready),
            .source_writer_frame_owned_debug(storage_source_writer_frame_owned),
            .source_slot_state_debug(storage_source_slot_state_debug)
        );

        // Native v0.4 combined inverse mapping and one bilinear sample.
        distortion_storage_engine_monitor_r14 #(
            .FRAME_WIDTH(1920),.FRAME_HEIGHT(1080),.MAX_WIDTH(1920),.SLOT_W(2),
            .COORD_W(26),.COORD_F(10),.BORDER_CLAMP(1),.CACHE_SET_BITS(10),
            .ADAPTIVE_GAMMA_ENABLE(ADAPTIVE_GAMMA_ENABLE),.FORCE_BLACK_BORDER(FORCE_BLACK_BORDER),
            .GAMMA_LUT_FILE(GAMMA_LUT_FILE)) u_algorithm(
            .cfg_valid(v04_cfg_valid),.cfg_ready(v04_cfg_ready),.cfg_payload(v04_cfg_payload),.cfg_seq(v04_cfg_seq),
            .cfg_received(v04_cfg_received),.cfg_rejected(v04_cfg_rejected),.cfg_applied(v04_cfg_applied),
            .cfg_received_seq(v04_received_seq),.cfg_applied_seq(v04_applied_seq),.cfg_applied_frame_id(v04_applied_frame),
            .active_cfg_version(v04_active_version),.result_writer_idle(video_result_idle),.profile_valid(1'b1),
            .cfg_fx_q16(32'sd134217728),.cfg_fy_q16(32'sd134217728),
            .cfg_cx_q16(32'sd62914560),.cfg_cy_q16(32'sd35389440),
            .cfg_inv_fx_q24(32'sd8192),.cfg_inv_fy_q24(32'sd8192),
            .cfg_k1_q24(32'sd0),.cfg_k2_q24(32'sd0),.cfg_p1_q24(32'sd0),.cfg_p2_q24(32'sd0),
            .clk(core_clk), .rst_n(storage_ddr_rst_n),
            .source_desc_claim_valid(storage_claim_valid), .source_desc_claim_ready(storage_claim_ready),
            .source_desc_claim_slot(storage_claim_slot), .source_desc_base_addr(storage_desc_base_addr),
            .source_desc_frame_id(storage_desc_frame_id),
            .source_desc_source_frame_id(storage_desc_source_frame_id),
            .source_desc_cfg_version(storage_desc_cfg_version), .source_desc_width(storage_desc_width),
            .source_desc_height(storage_desc_height), .source_desc_stride_bytes(storage_desc_stride_bytes),
            .source_desc_release_valid(storage_release_valid), .source_desc_release_slot(storage_release_slot),
            .source_desc_release_ready(storage_release_ready),
            .fetch_rd_req_valid(storage_fetch_req_valid), .fetch_rd_req_ready(storage_fetch_req_ready),
            .fetch_rd_req_addr(storage_fetch_req_addr), .fetch_rd_req_len(storage_fetch_req_len),
            .fetch_rd_req_id(storage_fetch_req_id), .fetch_rd_rsp_valid(storage_fetch_rsp_valid),
            .fetch_rd_rsp_ready(storage_fetch_rsp_ready), .fetch_rd_rsp_data(storage_fetch_rsp_data),
            .fetch_rd_rsp_id(storage_fetch_rsp_id), .fetch_rd_rsp_last(storage_fetch_rsp_last),
            .fetch_rd_rsp_error(storage_fetch_rsp_error),
            .result_px_valid(storage_result_px_valid), .result_px_ready(storage_result_px_ready),
            .result_px_rgb(storage_result_px_rgb), .result_px_x(storage_result_px_x),
            .result_px_y(storage_result_px_y), .result_px_sof(storage_result_px_sof),
            .result_px_eol(storage_result_px_eol),
            .result_px_source_frame_id(storage_result_px_source_frame_id),
            .result_px_cfg_version(storage_result_px_cfg_version),
            .result_px_pixel_tag(storage_result_px_pixel_tag), .result_frame_error(storage_algorithm_frame_error), .error_count(storage_bypass_error_count),
            .debug_state(storage_bypass_debug_state),
            .debug_x(storage_bypass_debug_x), .debug_y(storage_bypass_debug_y),
            .gamma_observation(gamma_observation)
        );
// Original engine supplies per-frame fault status to the result writer.


        assign init_done = ddr_init_done;
        assign de_o = storage_display_pixel_valid;
        assign o_rgb565 = storage_display_pixel_valid ?
            {storage_display_pixel_rgb[23:19], storage_display_pixel_rgb[15:10], storage_display_pixel_rgb[7:3]} : 16'd0;
        assign diag_rd_cmd_en = storage_fetch_req_valid;
        assign diag_rd_cmd_ready = storage_fetch_req_ready;
        assign diag_rd_cmd_done = storage_display_scan_done;

        always @(posedge core_clk or negedge storage_ddr_rst_n) begin
            if (!storage_ddr_rst_n) begin
                storage_source_commit_count_d <= 32'd0;
                storage_source_commit_pulse <= 1'b0;
            end else begin
                storage_source_commit_pulse <= (storage_source_frame_commit_count != storage_source_commit_count_d);
                storage_source_commit_count_d <= storage_source_frame_commit_count;
            end
        end
        assign frame_commit_pulse = storage_source_commit_pulse;
    end
endgenerate

   /*  always@(posedge pix_clk) begin
        r_out<=8'hff;
        g_out<=8'hff;
        b_out<=8'h00; 
        vs_out<=vs_o;
        hs_out<=hs_o;
        de_out<=de_re;
     end*/
//assign pix_clk = pixclk_in;
// BEGIN R10 OUTPUT COLOR — independent of RAW/DDR/UART diagnostic payloads.
wire [23:0] display_rgb_r10;
wire display_de_r10,display_hs_r10,display_vs_r10;
generate
 if ((VIDEO_SOURCE == 3) && (SPATIAL_COLOR_ENABLE != 0)) begin : g_display_color_r10
// Spatial R/B gains are applied in source coordinates before geometry.
  native1080_display_color_r10 #(.RED_GRID({54{10'd256}}),.BLUE_GRID({54{10'd256}}),.TONE_LUT((ADAPTIVE_GAMMA_ENABLE!=0)?IDENTITY_TONE:COLOR_TONE)) u_display_color (
   .clk(pix_clk),.reset(!video_rst_n),.rgb_in(selected_video_rgb888),
   .de_in(selected_video_de),.hs_in(hs_pipe[1]),.vs_in(vs_pipe[1]),
   .rgb_out(display_rgb_r10),.de_out(display_de_r10),.hs_out(display_hs_r10),.vs_out(display_vs_r10));
 end else begin : g_display_color_bypass_r10
  assign display_rgb_r10=selected_video_rgb888;
  assign display_de_r10=selected_video_de;
  assign display_hs_r10=hs_pipe[1];assign display_vs_r10=vs_pipe[1];
 end
endgenerate
// END R10 OUTPUT COLOR
always  @(posedge pix_clk)begin


        vs_out       <=  display_vs_r10        ;
        hs_out       <=  display_hs_r10        ;
        de_out       <=  display_de_r10        ;
        r_out        <=  display_rgb_r10[23:16]         ;
        g_out        <=  display_rgb_r10[15:8]         ;
        b_out        <=  display_rgb_r10[7:0]         ;

end
/////////////////////////////////////////////////////////////////////////////////////
//??visa?? 
wire                        hs         ;
wire                        vs         ;
wire                        de         ;
// MODE_720p30 output timing.
parameter V_TOTAL = 12'd1125;
parameter V_FP = 12'd4;
parameter V_BP = 12'd36;
parameter V_SYNC = 12'd5;
parameter V_ACT = 12'd1080;
parameter H_TOTAL = 12'd2200;
parameter H_FP = 12'd88;
parameter H_BP = 12'd148;
parameter H_SYNC = 12'd44;
parameter H_ACT = 12'd1920;
parameter HV_OFFSET = 12'd0;parameter   X_WIDTH = 4'd12;
parameter   Y_WIDTH = 4'd12; 
sync_vg #(
    .X_BITS               (  X_WIDTH              ), 
    .Y_BITS               (  Y_WIDTH              ),
    .V_TOTAL              (  V_TOTAL              ),//                        
    .V_FP                 (  V_FP                 ),//                        
    .V_BP                 (  V_BP                 ),//                        
    .V_SYNC               (  V_SYNC               ),//                        
    .V_ACT                (  V_ACT                ),//                        
    .H_TOTAL              (  H_TOTAL              ),//                        
    .H_FP                 (  H_FP                 ),//                        
    .H_BP                 (  H_BP                 ),//                        
    .H_SYNC               (  H_SYNC               ),//                        
    .H_ACT                (  H_ACT                ) //                        

) sync_vg                                         
(                                                 
    .clk                  (  pix_clk               ),//input                   clk,                                 
    .rstn                 (  video_rst_n                    ),//input                   rstn,                             
    .vs_out               (  vs_reg                   ),//output reg              vs_out,                                                                                                                                      
    .hs_out               (  hs_reg                   ),//output reg              hs_out,            
    .de_out               (  rd_en                   ),//output reg              de_out,             
    .x_act                (  act_x                ),//output reg [X_BITS-1:0] x_out,             
    .y_act                (  act_y                ) //output reg [Y_BITS:0]   y_out,             
); 

assign source_pattern_de = rd_en;
assign source_ddr_de = de_o;
assign source_camera_de = de_o;
assign source_ddr_rgb565 = o_rgb565;
assign source_camera_rgb565 = o_rgb565;

video_test_pattern u_video_test_pattern (
    .de             (source_pattern_de),
    .x              (act_x),
    .y              (act_y),
    .pattern_select (TEST_PATTERN),
    .rgb565         (source_pattern_rgb565)
);

generate
    if (VIDEO_SOURCE == 3) begin : g_storage_bypass_video_select
        if (SPATIAL_COLOR_ENABLE) begin : g_rgb888_hdmi
            assign selected_video_rgb888 = storage_display_pixel_valid ? storage_display_pixel_rgb : 24'd0;
        end else begin : g_legacy_storage_hdmi
            assign selected_video_rgb888 = {selected_video_rgb565[15:11],3'd0,
                selected_video_rgb565[10:5],2'd0,selected_video_rgb565[4:0],3'd0};
        end
        assign selected_video_de = rd_en;
        assign selected_video_rgb565 = storage_display_pixel_valid ?
            {storage_display_pixel_rgb[23:19], storage_display_pixel_rgb[15:10], storage_display_pixel_rgb[7:3]} : 16'd0;
    end else begin : g_legacy_video_select
        assign selected_video_rgb888 = {selected_video_rgb565[15:11],3'd0,
            selected_video_rgb565[10:5],2'd0,selected_video_rgb565[4:0],3'd0};
        video_source_select #(.SOURCE(VIDEO_SOURCE)) u_video_source_select (
            .pattern_de     (source_pattern_de),
            .pattern_rgb565 (source_pattern_rgb565),
            .ddr_de         (source_ddr_de),
            .ddr_rgb565     (source_ddr_rgb565),
            .camera_de      (source_camera_de),
            .camera_rgb565  (source_camera_rgb565),
            .selected_de    (selected_video_de),
            .selected_rgb565(selected_video_rgb565)
        );
    end
endgenerate


////////////////////////////////////////////////////////////////////////////////////////////

wire clk_125Mhz ;
wire [2:0] ddr_ref_reset_sync;
reset_sync u_ddr_ref_reset(.clk(clk_125Mhz),.arst_n(rstn_out),
    .sync_state(ddr_ref_reset_sync));

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);

//ddr    
        ddr3_test u_ddr3_test_h (
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (ddr_ref_reset_sync[2]),// input
             .ddr_init_done             (ddr_init_done      ),// output

             .pll_lock                  (pll_lock           ),// output

             .core_clk                  (core_clk),                                  // output

             .phy_pll_lock              (phy_pll_lock),                          // output
             .gpll_lock                 (gpll_lock),                                // output
             .rst_gpll_lock             (rst_gpll_lock),                        // output
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
             //.ddr_init_done             (ddr_init_done),                        // output


             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug

  .dbg_gate_start(1'b0),                      // input
  .dbg_cpd_start(1'b0),                        // input
  .dbg_ddrphy_rst_n(1'b1),                  // input
  .dbg_gpll_scan_rst(1'b0),                // input
  .samp_position_dyn_adj(1'b0),        // input
  .init_samp_position_even(32'd0),    // input [31:0]
  .init_samp_position_odd(32'd0),      // input [31:0]
  .wrcal_position_dyn_adj(1'b0),      // input
  .init_wrcal_position(32'd0),            // input [31:0]
  .force_read_clk_ctrl(1'b0),            // input
  .init_slip_step(16'd0),                      // input [15:0]
  .init_read_clk_ctrl(12'd0),              // input [11:0]
  .debug_calib_ctrl(),                  // output [33:0]
  .dbg_slice_status(),                  // output [67:0]
  .dbg_slice_state(),                    // output [87:0]
  .debug_data(),                              // output [275:0]
  .dbg_dll_upd_state(),                // output [1:0]
  .debug_gpll_dps_phase(),          // output [8:0]
  .dbg_rst_dps_state(),                // output [2:0]
  .dbg_tran_err_rst_cnt(),          // output [5:0]
  .dbg_ddrphy_init_fail(),          // output
  .debug_cpd_offset_adj(1'b0),          // input
  .debug_cpd_offset_dir(1'b0),          // input
  .debug_cpd_offset(10'd0),                  // input [9:0]
  .debug_dps_cnt_dir0(),              // output [9:0]
  .debug_dps_cnt_dir1(),              // output [9:0]
  .ck_dly_en(1'b0),                                // input
  .init_ck_dly_step(8'h0),                  // input [7:0]
  .ck_dly_set_bin(),                      // output [7:0]
  .align_error(),                            // output
  .debug_rst_state(),                    // output [3:0]
             .debug_cpd_state()                     // output [3:0]
       );

//????
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////
endmodule


