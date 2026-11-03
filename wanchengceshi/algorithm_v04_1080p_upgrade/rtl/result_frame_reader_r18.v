`timescale 1ns/1ps
// Sequential DDR reader for the result pool. It retains the currently
// DISPLAYING slot for repeat scans and swaps only when another RESULT_READY
// descriptor is pending at a scan boundary.
module result_frame_reader #(
    parameter integer MAX_BURST_BEATS = 16,
    parameter integer REQ_ID_W = 8,
    parameter [REQ_ID_W-1:0] READ_REQ_ID = 8'hA5,
    parameter integer SLOT_W = 1
) (
    input wire clk,
    input wire rst_n,
    input wire scan_start,

    input wire pool_reader_pending,
    output wire pool_claim_valid,
    input wire pool_claim_ready,
    input wire [SLOT_W-1:0] pool_claim_slot,
    input wire [63:0] pool_desc_base_addr,
    input wire [31:0] pool_desc_frame_id,
    input wire [31:0] pool_desc_source_frame_id,
    input wire [15:0] pool_desc_cfg_version,
    input wire [15:0] pool_desc_width,
    input wire [15:0] pool_desc_height,
    input wire [31:0] pool_desc_stride_bytes,
    input wire [7:0] pool_desc_format,
    input wire [31:0] pool_desc_process_cycles,
    input wire pool_desc_valid,
    output wire pool_release_valid,
    output wire [SLOT_W-1:0] pool_release_slot,
    input wire pool_release_ready,

    output wire mem_rd_req_valid,
    input wire mem_rd_req_ready,
    output wire [63:0] mem_rd_req_addr,
    output wire [31:0] mem_rd_req_len,
    output wire [REQ_ID_W-1:0] mem_rd_req_id,
    input wire mem_rd_rsp_valid,
    output wire mem_rd_rsp_ready,
    input wire [255:0] mem_rd_rsp_data,
    input wire [REQ_ID_W-1:0] mem_rd_rsp_id,
    input wire mem_rd_rsp_last,
    input wire mem_rd_rsp_error,

    output wire pixel_out_valid,
    input wire pixel_out_ready,
    output wire [23:0] pixel_out_rgb,
    output wire [15:0] pixel_out_x,
    output wire [15:0] pixel_out_y,
    output wire pixel_out_sof,
    output wire pixel_out_eol,
    output wire pixel_out_eof,
    output wire [31:0] display_frame_id,
    output wire [31:0] display_source_frame_id,
    output wire [15:0] display_cfg_version,
    output reg scan_done,
    output reg [31:0] display_repeat_count,
    output reg [31:0] display_underflow_count,
    output reg [31:0] scan_overrun_count
);
    localparam [2:0] RD_WAIT_SCAN = 3'd0;
    localparam [2:0] RD_RELEASE = 3'd1;
    localparam [2:0] RD_CLAIM = 3'd2;
    localparam [2:0] RD_REQUEST = 3'd3;
    localparam [2:0] RD_RESPONSE = 3'd4;
    localparam [2:0] RD_PIXELS = 3'd5;
    localparam [2:0] RD_ERROR_DRAIN = 3'd6;
    localparam [2:0] RD_CHECK = 3'd7;

    reg [2:0] reader_state;
    reg release_then_claim;
    reg current_desc_valid;
    reg claimed_pool_valid;
    reg [SLOT_W-1:0] current_slot;
    reg [63:0] current_base_addr;
    reg [63:0] current_frame_base_addr;
    reg [31:0] current_frame_id;
    reg [31:0] current_source_frame_id;
    reg [15:0] current_cfg_version;
    reg [15:0] current_width;
    reg [15:0] current_height;
    reg [31:0] current_stride_bytes;
    reg [7:0] current_format;
    reg [31:0] current_process_cycles;
    reg [63:0] current_frame_beats;
    reg [63:0] read_beats_remaining;
    reg [31:0] row_beats_remaining;
    reg [31:0] request_beats_remaining;

    reg beat_valid;
    reg beat_last;
    reg [255:0] beat_data;
    reg [2:0] beat_lane;
    reg [3:0] beat_pixel_count;
    reg [15:0] row_pixel_x;
    reg [15:0] row_pixel_y;

    wire [16:0] remaining_pixels_in_row = {1'b0, current_width} - {1'b0, row_pixel_x};
    wire [31:0] request_beats = (row_beats_remaining > MAX_BURST_BEATS) ?
        MAX_BURST_BEATS : row_beats_remaining;
    wire result_desc_valid = claimed_pool_valid && (current_width != 0) &&
        (current_height != 0) && (current_format == 8'h01) &&
        (current_stride_bytes >= ({16'd0, current_width} << 2)) &&
        (current_stride_bytes[4:0] == 5'b0);
    // Preserve the original 32-bit guard expression. Beat assignments below
    // retain their original 64-bit expression context.
    wire [31:0] current_guard_bytes = current_stride_bytes * current_height;

    assign pool_claim_valid = (reader_state == RD_CLAIM);
    assign pool_release_valid = (reader_state == RD_RELEASE);
    assign pool_release_slot = current_slot;

    assign mem_rd_req_valid = (reader_state == RD_REQUEST);
    assign mem_rd_req_addr = current_base_addr;
    assign mem_rd_req_len = request_beats;
    assign mem_rd_req_id = READ_REQ_ID;
    assign mem_rd_rsp_ready = ((reader_state == RD_RESPONSE) && !beat_valid) ||
                              (reader_state == RD_ERROR_DRAIN);

    assign pixel_out_valid = (reader_state == RD_PIXELS) && beat_valid &&
                             (beat_lane < beat_pixel_count);
    assign pixel_out_rgb = beat_data[beat_lane*32 +: 24];
    assign pixel_out_x = row_pixel_x;
    assign pixel_out_y = row_pixel_y;
    assign pixel_out_sof = (row_pixel_x == 0) && (row_pixel_y == 0);
    assign pixel_out_eol = (row_pixel_x == current_width - 1'b1);
    assign pixel_out_eof = pixel_out_eol && (row_pixel_y == current_height - 1'b1);
    assign display_frame_id = current_frame_id;
    assign display_source_frame_id = current_source_frame_id;
    assign display_cfg_version = current_cfg_version;

    task finish_beat;
        begin
            beat_valid <= 1'b0;
            beat_lane <= 3'd0;
            if (row_beats_remaining == 32'd1) begin
                row_beats_remaining <= current_stride_bytes >> 5;
                row_pixel_x <= 16'd0;
                if (row_pixel_y != current_height - 1'b1)
                    row_pixel_y <= row_pixel_y + 1'b1;
            end else if (row_beats_remaining != 0) begin
                row_beats_remaining <= row_beats_remaining - 1'b1;
            end
            if (beat_last) begin
                if (read_beats_remaining == 0) begin
                    reader_state <= RD_WAIT_SCAN;
                    scan_done <= 1'b1;
                end else begin
                    reader_state <= RD_REQUEST;
                end
            end else begin
                reader_state <= RD_RESPONSE;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reader_state <= RD_WAIT_SCAN;
            release_then_claim <= 1'b0;
            current_desc_valid <= 1'b0;
            claimed_pool_valid <= 1'b0;
            current_slot <= {SLOT_W{1'b0}};
            current_base_addr <= 64'd0;
            current_frame_base_addr <= 64'd0;
            current_frame_id <= 32'd0;
            current_source_frame_id <= 32'd0;
            current_cfg_version <= 16'd0;
            current_width <= 16'd0;
            current_height <= 16'd0;
            current_stride_bytes <= 32'd0;
            current_format <= 8'd0;
            current_process_cycles <= 32'd0;
            current_frame_beats <= 64'd0;
            read_beats_remaining <= 64'd0;
            row_beats_remaining <= 32'd0;
            request_beats_remaining <= 32'd0;
            beat_valid <= 1'b0;
            beat_last <= 1'b0;
            beat_data <= 256'd0;
            beat_lane <= 3'd0;
            beat_pixel_count <= 4'd0;
            row_pixel_x <= 16'd0;
            row_pixel_y <= 16'd0;
            scan_done <= 1'b0;
            display_repeat_count <= 32'd0;
            display_underflow_count <= 32'd0;
            scan_overrun_count <= 32'd0;
        end else begin
            scan_done <= 1'b0;
            if (scan_start && (reader_state != RD_WAIT_SCAN))
                scan_overrun_count <= scan_overrun_count + 1'b1;

            case (reader_state)
                RD_WAIT_SCAN: begin
                    if (scan_start) begin
                        if (current_desc_valid) begin
                            if (pool_reader_pending) begin
                                release_then_claim <= 1'b1;
                                reader_state <= RD_RELEASE;
                            end else begin
                                display_repeat_count <= display_repeat_count + 1'b1;
                                current_base_addr <= current_frame_base_addr;
                                read_beats_remaining <= current_frame_beats;
                                row_beats_remaining <= current_stride_bytes >> 5;
                                row_pixel_x <= 16'd0;
                                row_pixel_y <= 16'd0;
                                reader_state <= RD_REQUEST;
                            end
                        end else if (pool_reader_pending) begin
                            reader_state <= RD_CLAIM;
                        end else begin
                            display_underflow_count <= display_underflow_count + 1'b1;
                        end
                    end
                end

                RD_RELEASE: begin
                    if (pool_release_ready) begin
                        current_desc_valid <= 1'b0;
                        reader_state <= release_then_claim ? RD_CLAIM : RD_WAIT_SCAN;
                        release_then_claim <= 1'b0;
                    end
                end

                RD_CLAIM: begin
                    if (pool_claim_ready) begin
                        current_slot <= pool_claim_slot;
                        current_base_addr <= pool_desc_base_addr;
                        current_frame_base_addr <= pool_desc_base_addr;
                        current_frame_id <= pool_desc_frame_id;
                        current_source_frame_id <= pool_desc_source_frame_id;
                        current_cfg_version <= pool_desc_cfg_version;
                        current_width <= pool_desc_width;
                        current_height <= pool_desc_height;
                        current_stride_bytes <= pool_desc_stride_bytes;
                        current_format <= pool_desc_format;
                        current_process_cycles <= pool_desc_process_cycles;
                        claimed_pool_valid <= pool_desc_valid;
                        row_beats_remaining <= pool_desc_stride_bytes >> 5;
                        request_beats_remaining <= 32'd0;
                        current_desc_valid <= 1'b1;
                        row_pixel_x <= 16'd0;
                        row_pixel_y <= 16'd0;
                        beat_valid <= 1'b0;
                        beat_lane <= 3'd0;
                        reader_state <= RD_CHECK;
                    end
                end

                RD_CHECK: begin
                    current_frame_beats <= (current_stride_bytes * current_height) >> 5;
                    read_beats_remaining <= (current_stride_bytes * current_height) >> 5;
                    if (result_desc_valid && (current_guard_bytes >= 32)) begin
                        reader_state <= RD_REQUEST;
                    end else begin
                        display_underflow_count <= display_underflow_count + 1'b1;
                        release_then_claim <= 1'b0;
                        reader_state <= RD_RELEASE;
                    end
                end

                RD_REQUEST: begin
                    if (mem_rd_req_ready) begin
                        beat_valid <= 1'b0;
                        beat_lane <= 3'd0;
                        request_beats_remaining <= mem_rd_req_len;
                        current_base_addr <= current_base_addr + ({32'd0, mem_rd_req_len} << 5);
                        reader_state <= RD_RESPONSE;
                    end
                end

                RD_RESPONSE: begin
                    if (mem_rd_rsp_valid && mem_rd_rsp_ready) begin
                        if (read_beats_remaining != 0)
                            read_beats_remaining <= read_beats_remaining - 1'b1;
                        if (request_beats_remaining != 0)
                            request_beats_remaining <= request_beats_remaining - 1'b1;
                        if (mem_rd_rsp_error || (mem_rd_rsp_id != READ_REQ_ID) ||
                            (mem_rd_rsp_last != (request_beats_remaining == 32'd1))) begin
                            display_underflow_count <= display_underflow_count + 1'b1;
                            beat_valid <= 1'b0;
                            release_then_claim <= 1'b0;
                            if (mem_rd_rsp_last || (request_beats_remaining == 32'd1))
                                reader_state <= RD_RELEASE;
                            else
                                reader_state <= RD_ERROR_DRAIN;
                        end else begin
                            beat_data <= mem_rd_rsp_data;
                            beat_last <= mem_rd_rsp_last;
                            beat_lane <= 3'd0;
                            beat_valid <= 1'b1;
                            if (remaining_pixels_in_row >= 17'd8)
                                beat_pixel_count <= 4'd8;
                            else
                                beat_pixel_count <= remaining_pixels_in_row[3:0];
                            reader_state <= RD_PIXELS;
                        end
                    end
                end

                RD_ERROR_DRAIN: begin
                    if (mem_rd_rsp_valid && mem_rd_rsp_ready) begin
                        if (read_beats_remaining != 0)
                            read_beats_remaining <= read_beats_remaining - 1'b1;
                        if (request_beats_remaining != 0)
                            request_beats_remaining <= request_beats_remaining - 1'b1;
                        if (mem_rd_rsp_last || (request_beats_remaining == 32'd1)) begin
                            release_then_claim <= 1'b0;
                            reader_state <= RD_RELEASE;
                        end
                    end
                end

                RD_PIXELS: begin
                    if (beat_valid) begin
                        if (beat_lane < beat_pixel_count) begin
                            if (pixel_out_ready) begin
                                if (row_pixel_x == current_width - 1'b1) begin
                                    row_pixel_x <= current_width;
                                end else begin
                                    row_pixel_x <= row_pixel_x + 1'b1;
                                end
                                if (beat_lane == 3'd7)
                                    finish_beat();
                                else
                                    beat_lane <= beat_lane + 1'b1;
                            end
                        end else begin
                            // Consume padded words at the end of each row without
                            // exposing them as active display pixels.
                            if (beat_lane == 3'd7)
                                finish_beat();
                            else
                                beat_lane <= beat_lane + 1'b1;
                        end
                    end
                end

                default: reader_state <= RD_WAIT_SCAN;
            endcase
        end
    end
endmodule
