`timescale 1ns / 1ps

// OV5640 DVP diagnostic collector.
//
// All live counters are kept in the camera PCLK domain.  A UART request is
// transported as a toggle and the captured bus is held until the next
// request, so the system-clock UART can read a coherent frame snapshot.
module camera_dvp_diag #(
    // Keep diagnostic sampling aligned with the current FPGA-side low-active
    // HREF A/B; this does not change the OV5640 SCCB register table.
    parameter HREF_ACTIVE_LEVEL = 1'b0
) (
    input              cam_pclk,
    input              rst_n,
    input              cam_vsync,
    input              cam_href,
    input       [7:0]  cam_data,
    input              capture_valid,
    input              norm_valid,
    input              pad_valid,
    input              norm_line,
    input              req_toggle_sys,

    output reg         ack_toggle_cam,
    output reg [31:0]  snap_frame_count,
    output reg [31:0]  snap_pclk_count,
    output reg [15:0]  snap_frame_lines,
    output reg [15:0]  snap_last_line_bytes,
    output reg [15:0]  snap_min_line_bytes,
    output reg [15:0]  snap_max_line_bytes,
    output reg [15:0]  snap_last_pair_count,
    output reg [1:0]   snap_byte_phase,
    output reg [15:0]  snap_first_bytes,
    output reg [31:0]  snap_data_sum,
    output reg [31:0]  snap_data_xor,
    output reg [31:0]  snap_capture_pixel_count,
    output reg [31:0]  snap_norm_pixel_count,
    output reg [31:0]  snap_norm_pad_count,
    output reg [31:0]  snap_norm_line_count
);
    reg req_meta;
    reg req_sync;
    reg req_seen;
    reg href_d;
    reg href_rise_d;
    reg href_fall_d;
    reg [7:0] cam_data_d;
    reg vsync_d;
    reg vsync_rise_d;
    reg vsync_fall_d;
    reg norm_line_d;
    reg byte_phase_live;
    reg [4:0] sample_count_live;
    reg [15:0] line_byte_count_live;
    reg [15:0] frame_line_count_live;
    reg [15:0] last_line_bytes_live;
    reg [15:0] min_line_bytes_live;
    reg [15:0] max_line_bytes_live;
    reg [15:0] last_pair_count_live;
    reg [15:0] first_bytes_live;
    reg [31:0] pclk_count_live;
    reg [31:0] frame_count_live;
    reg [31:0] data_sum_live;
    reg [31:0] data_xor_live;
    reg [31:0] capture_pixel_count_live;
    reg [31:0] norm_pixel_count_live;
    reg [31:0] norm_pad_count_live;
    reg [31:0] norm_line_count_live;
    reg [15:0] completed_frame_lines_live;
    reg [15:0] completed_last_line_bytes_live;
    reg [15:0] completed_min_line_bytes_live;
    reg [15:0] completed_max_line_bytes_live;
    reg [15:0] completed_last_pair_count_live;
    reg [15:0] completed_first_bytes_live;
    reg [31:0] completed_data_sum_live;
    reg [31:0] completed_data_xor_live;
    reg [31:0] completed_capture_pixel_count_live;
    reg [31:0] completed_norm_pixel_count_live;
    reg [31:0] completed_norm_pad_count_live;
    reg [31:0] completed_norm_line_count_live;

    wire href_active = (cam_href == HREF_ACTIVE_LEVEL);
    wire href_rise = (~href_d) & href_active;
    wire href_fall = href_d & (~href_active);
    wire vsync_rise = (~vsync_d) & cam_vsync;
    wire vsync_fall = vsync_d & (~cam_vsync);

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            req_meta <= 1'b0;
            req_sync <= 1'b0;
            req_seen <= 1'b0;
            href_d <= 1'b0;
            href_rise_d <= 1'b0;
            href_fall_d <= 1'b0;
            cam_data_d <= 8'd0;
            vsync_d <= 1'b1;
            vsync_rise_d <= 1'b0;
            vsync_fall_d <= 1'b0;
            norm_line_d <= 1'b0;
            byte_phase_live <= 1'b0;
            sample_count_live <= 5'd0;
            line_byte_count_live <= 16'd0;
            frame_line_count_live <= 16'd0;
            last_line_bytes_live <= 16'd0;
            min_line_bytes_live <= 16'hffff;
            max_line_bytes_live <= 16'd0;
            last_pair_count_live <= 16'd0;
            first_bytes_live <= 16'd0;
            pclk_count_live <= 32'd0;
            frame_count_live <= 32'd0;
            data_sum_live <= 32'd0;
            data_xor_live <= 32'd0;
            capture_pixel_count_live <= 32'd0;
            norm_pixel_count_live <= 32'd0;
            norm_pad_count_live <= 32'd0;
            norm_line_count_live <= 32'd0;
            completed_frame_lines_live <= 16'd0;
            completed_last_line_bytes_live <= 16'd0;
            completed_min_line_bytes_live <= 16'd0;
            completed_max_line_bytes_live <= 16'd0;
            completed_last_pair_count_live <= 16'd0;
            completed_first_bytes_live <= 16'd0;
            completed_data_sum_live <= 32'd0;
            completed_data_xor_live <= 32'd0;
            completed_capture_pixel_count_live <= 32'd0;
            completed_norm_pixel_count_live <= 32'd0;
            completed_norm_pad_count_live <= 32'd0;
            completed_norm_line_count_live <= 32'd0;
            ack_toggle_cam <= 1'b0;
            snap_frame_count <= 32'd0;
            snap_pclk_count <= 32'd0;
            snap_frame_lines <= 16'd0;
            snap_last_line_bytes <= 16'd0;
            snap_min_line_bytes <= 16'd0;
            snap_max_line_bytes <= 16'd0;
            snap_last_pair_count <= 16'd0;
            snap_byte_phase <= 2'd0;
            snap_first_bytes <= 16'd0;
            snap_data_sum <= 32'd0;
            snap_data_xor <= 32'd0;
            snap_capture_pixel_count <= 32'd0;
            snap_norm_pixel_count <= 32'd0;
            snap_norm_pad_count <= 32'd0;
            snap_norm_line_count <= 32'd0;
        end else begin
            req_meta <= req_toggle_sys;
            req_sync <= req_meta;
            href_d <= href_active;
            href_rise_d <= href_rise;
            href_fall_d <= href_fall;
            cam_data_d <= cam_data;
            vsync_d <= cam_vsync;
            vsync_rise_d <= vsync_rise;
            vsync_fall_d <= vsync_fall;
            norm_line_d <= norm_line;
            pclk_count_live <= pclk_count_live + 1'b1;

            // The OV5640 default polarity is active-low VSYNC.  Start a
            // fresh sample window at the falling edge of VSYNC.
            if (vsync_fall_d) begin
                frame_line_count_live <= 16'd0;
                line_byte_count_live <= 16'd0;
                last_line_bytes_live <= 16'd0;
                last_pair_count_live <= 16'd0;
                min_line_bytes_live <= 16'hffff;
                max_line_bytes_live <= 16'd0;
                sample_count_live <= 5'd0;
                first_bytes_live <= 16'd0;
                data_sum_live <= 32'd0;
                data_xor_live <= 32'd0;
                capture_pixel_count_live <= 32'd0;
                norm_pixel_count_live <= norm_valid ? 32'd1 : 32'd0;
                norm_pad_count_live <= pad_valid ? 32'd1 : 32'd0;
                norm_line_count_live <= (norm_line && !norm_line_d) ? 32'd1 : 32'd0;
            end

            if (vsync_rise_d) begin
                frame_count_live <= frame_count_live + 1'b1;
                // Include a line-start event delayed into this cycle so a
                // frame ending immediately after HREF rises is still counted.
                completed_frame_lines_live <= frame_line_count_live +
                                              (href_rise_d ? 16'd1 : 16'd0);
                completed_last_line_bytes_live <= last_line_bytes_live;
                completed_min_line_bytes_live <= (min_line_bytes_live == 16'hffff) ?
                                                  16'd0 : min_line_bytes_live;
                completed_max_line_bytes_live <= max_line_bytes_live;
                completed_last_pair_count_live <= last_pair_count_live;
                completed_first_bytes_live <= first_bytes_live;
                completed_data_sum_live <= data_sum_live;
                completed_data_xor_live <= data_xor_live;
                completed_capture_pixel_count_live <= capture_pixel_count_live;
                completed_norm_pixel_count_live <= norm_pixel_count_live +
                                                   (norm_valid ? 32'd1 : 32'd0);
                completed_norm_pad_count_live <= norm_pad_count_live +
                                                 (pad_valid ? 32'd1 : 32'd0);
                completed_norm_line_count_live <= norm_line_count_live +
                                                  ((norm_line && !norm_line_d) ? 32'd1 : 32'd0);
            end

            // Register the external HREF edge before incrementing the
            // multi-bit counter; this shortens the cam_href input timing path.
            if (href_rise_d && !vsync_fall_d)
                frame_line_count_live <= frame_line_count_live + 1'b1;

            // Count the preceding PCLK cycle from registered HREF. This
            // removes cam_href from the 16-bit line-counter enable path.
            if (href_d) begin
                if (!vsync_fall_d)
                    line_byte_count_live <= line_byte_count_live + 1'b1;
            end

            // Pair the registered DVP data with the registered HREF state.
            // The first active byte is accounted one PCLK later, and the
            // final active byte is still counted on the first inactive cycle.
            // This keeps external HREF/data off the wide checksum path.
            if (href_d) begin
                data_sum_live <= data_sum_live + {24'd0, cam_data_d};
                data_xor_live <= data_xor_live ^ {24'd0, cam_data_d};
                if (sample_count_live < 5'd2) begin
                    first_bytes_live <= {first_bytes_live[7:0], cam_data_d};
                    sample_count_live <= sample_count_live + 1'b1;
                end
            end

            if (href_active) begin
                byte_phase_live <= ~byte_phase_live;
            end else begin
                // HREF is the byte-phase boundary.  This is also the
                // condition the repaired CMOS bridge uses.
                byte_phase_live <= 1'b0;
            end

            if (capture_valid)
                capture_pixel_count_live <= capture_pixel_count_live + 1'b1;

            if (!vsync_fall_d) begin
                if (norm_valid)
                    norm_pixel_count_live <= norm_pixel_count_live + 1'b1;
                if (pad_valid)
                    norm_pad_count_live <= norm_pad_count_live + 1'b1;
                if (norm_line && !norm_line_d)
                    norm_line_count_live <= norm_line_count_live + 1'b1;
            end

            // Delay the line-end update as well, so cam_href drives only the
            // small edge-detection register and not the wide min/max counters.
            if (href_fall_d) begin
                last_line_bytes_live <= line_byte_count_live;
                last_pair_count_live <= line_byte_count_live[15:1];
                if (line_byte_count_live < min_line_bytes_live)
                    min_line_bytes_live <= line_byte_count_live;
                if (line_byte_count_live > max_line_bytes_live)
                    max_line_bytes_live <= line_byte_count_live;
                line_byte_count_live <= 16'd0;
            end

            // Capture a coherent snapshot and acknowledge the request.
            if (req_sync != req_seen) begin
                req_seen <= req_sync;
                snap_frame_count <= frame_count_live;
                snap_pclk_count <= pclk_count_live;
                snap_frame_lines <= completed_frame_lines_live;
                snap_last_line_bytes <= completed_last_line_bytes_live;
                snap_min_line_bytes <= completed_min_line_bytes_live;
                snap_max_line_bytes <= completed_max_line_bytes_live;
                snap_last_pair_count <= completed_last_pair_count_live;
                snap_byte_phase <= {1'b0, byte_phase_live};
                snap_first_bytes <= completed_first_bytes_live;
                snap_data_sum <= completed_data_sum_live;
                snap_data_xor <= completed_data_xor_live;
                snap_capture_pixel_count <= completed_capture_pixel_count_live;
                snap_norm_pixel_count <= completed_norm_pixel_count_live;
                snap_norm_pad_count <= completed_norm_pad_count_live;
                snap_norm_line_count <= completed_norm_line_count_live;
                ack_toggle_cam <= ~ack_toggle_cam;
            end
        end
    end
endmodule
