`timescale 1ns/1ps
// Pipelined fixed-raster NN thumbnail capture candidate.
// The ports and memory/read contract match nn_thumbnail_capture. This source
// is a staging experiment only; it has not been synthesized or qualified.
module nn_thumbnail_capture (
    input wire clk,
    input wire rst_n,
    input wire capture_enable,
    input wire frame_active,
    input wire pixel_valid,
    input wire pixel_abort,
    input wire [23:0] rgb,
    input wire [31:0] source_frame_id,
    output wire capture_busy,
    output reg image_valid,
    input wire image_ready,
    output reg [31:0] image_frame_id,
    input wire read_en,
    input wire [13:0] read_address,
    output reg read_valid,
    output reg [7:0] read_data,
    output reg [31:0] capture_error_count,
    output reg [31:0] skipped_frame_count
);
    localparam [10:0] LAST_X = 11'd1919;
    localparam [10:0] LAST_Y = 11'd1079;
    localparam [13:0] THUMBNAIL_BYTES = 14'd9216;

    // Native 3x3 stage: three native rows of 3-pixel horizontal RGB sums.
    reg [29:0] rgb_line0 [0:639] /* synthesis syn_ramstyle = "lut_ram" */;
    reg [29:0] rgb_line1 [0:639] /* synthesis syn_ramstyle = "lut_ram" */;
    reg [29:0] rgb_line2 [0:639] /* synthesis syn_ramstyle = "lut_ram" */;

    // Reduced-gray 5x5 stage: five rows of 5-pixel horizontal sums.
    reg [10:0] gray_line0 [0:127] /* synthesis syn_ramstyle = "lut_ram" */;
    reg [10:0] gray_line1 [0:127] /* synthesis syn_ramstyle = "lut_ram" */;
    reg [10:0] gray_line2 [0:127] /* synthesis syn_ramstyle = "lut_ram" */;
    reg [10:0] gray_line3 [0:127] /* synthesis syn_ramstyle = "lut_ram" */;
    reg [10:0] gray_line4 [0:127] /* synthesis syn_ramstyle = "lut_ram" */;

    // Only the 128x72 thumbnail is stored. Canvas margins read as zero.
    reg [7:0] thumbnail_ram [0:9215] /* synthesis syn_ramstyle = "block_ram" */;

    reg frame_active_d;
    reg frame_tracking;
    reg in_capture;
    reg frame_bad;
    reg frame_error_counted;
    reg raster_complete;
    reg frame_ending;
    reg [31:0] capture_frame_id;
    reg [10:0] x_count;
    reg [10:0] y_count;

    reg [1:0] rgb_x_phase;
    reg [1:0] rgb_y_phase;
    reg [9:0] reduced_x;
    reg [9:0] rgb_line0_x /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [9:0] rgb_line1_x /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [8:0] reduced_y;
    reg [9:0] rgb_r_acc;
    reg [9:0] rgb_g_acc;
    reg [9:0] rgb_b_acc;

    reg [2:0] gray_x_phase;
    reg [2:0] gray_y_phase;
    reg [6:0] gray_reduced_x;
    reg [6:0] gray_line0_x /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [6:0] gray_line1_x /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [6:0] gray_line2_x /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [6:0] gray_line3_x /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [6:0] gray_reduced_y;
    reg [10:0] gray_acc;

    reg buffer_valid;
    reg [13:0] write_count;

    // T: capture three vertical native sums. T+1: round RGB averages /9.
    reg s1_valid;
    reg [11:0] s1_r, s1_g, s1_b;
    reg s2_valid;
    reg [7:0] s2_r, s2_g, s2_b;
    // T+2: exact legacy weighted-gray round. T+3: reduced gray row/column.
    reg s3_valid;
    reg [7:0] s3_gray;
    // T+3: capture the completed 5x5 vertical sum/address.
    reg s4_valid;
    reg [12:0] s4_vsum;
    reg [13:0] s4_address;
    // T+4: round /25. T+5: synchronous write into thumbnail RAM.
    reg s5_valid;
    reg [7:0] s5_pixel;
    reg [13:0] s5_address;

    wire [9:0] rgb_hsum_r = rgb_r_acc + {2'b00, rgb[23:16]};
    wire [9:0] rgb_hsum_g = rgb_g_acc + {2'b00, rgb[15:8]};
    wire [9:0] rgb_hsum_b = rgb_b_acc + {2'b00, rgb[7:0]};

    wire [11:0] rgb_vsum_r = {2'b00, rgb_hsum_r} +
        {2'b00, rgb_line0[rgb_line0_x][29:20]} +
        {2'b00, rgb_line1[rgb_line1_x][29:20]};
    wire [11:0] rgb_vsum_g = {2'b00, rgb_hsum_g} +
        {2'b00, rgb_line0[rgb_line0_x][19:10]} +
        {2'b00, rgb_line1[rgb_line1_x][19:10]};
    wire [11:0] rgb_vsum_b = {2'b00, rgb_hsum_b} +
        {2'b00, rgb_line0[rgb_line0_x][9:0]} +
        {2'b00, rgb_line1[rgb_line1_x][9:0]};

    wire [7:0] s2_r_next = (s1_r + 12'd4) / 12'd9;
    wire [7:0] s2_g_next = (s1_g + 12'd4) / 12'd9;
    wire [7:0] s2_b_next = (s1_b + 12'd4) / 12'd9;
    wire [15:0] s3_gray_sum =
        ({8'd0, s2_r} * 16'd77) + ({8'd0, s2_g} * 16'd150) +
        ({8'd0, s2_b} * 16'd29) + 16'd128;
    wire [7:0] s3_gray_next = s3_gray_sum[15:8];
    wire [10:0] gray_hsum = gray_acc + {3'b000, s3_gray};
    wire [12:0] gray_vsum = {2'b00, gray_hsum} +
        {2'b00, gray_line0[gray_line0_x]} +
        {2'b00, gray_line1[gray_line1_x]} +
        {2'b00, gray_line2[gray_line2_x]} +
        {2'b00, gray_line3[gray_line3_x]};
    wire [7:0] s5_pixel_next = (s4_vsum + 13'd12) / 13'd25;
    wire [13:0] gray_address = {gray_reduced_y, gray_reduced_x};

    wire terminal_pixel = in_capture && pixel_valid &&
        (x_count == LAST_X) && (y_count == LAST_Y) && !pixel_abort;
    wire extra_pixel = frame_tracking && raster_complete && pixel_valid;
    wire frame_rise = frame_active && !frame_active_d;
    wire frame_fall = !frame_active && frame_active_d;
    wire rgb_emit = in_capture && !frame_bad && pixel_valid && !pixel_abort &&
        !raster_complete && (rgb_x_phase == 2'd2) && (rgb_y_phase == 2'd2);
    wire pipeline_empty = !s1_valid && !s2_valid && !s3_valid &&
        !s4_valid && !s5_valid;
    wire good_frame_fall = (raster_complete || terminal_pixel) && !frame_bad &&
        !pixel_abort && !extra_pixel;
    wire bad_frame_fall = frame_fall && frame_tracking && !good_frame_fall;
    wire overlap_rise = frame_rise && frame_tracking;
    wire abort_pipeline = (pixel_abort && frame_tracking) || extra_pixel ||
        bad_frame_fall || overlap_rise;

    assign capture_busy = frame_tracking;

    integer read_row;
    integer read_col;
    integer thumbnail_index;

    always @(posedge clk) begin
        if (!rst_n) begin
            frame_active_d <= 1'b0;
            frame_tracking <= 1'b0;
            in_capture <= 1'b0;
            frame_bad <= 1'b0;
            frame_error_counted <= 1'b0;
            raster_complete <= 1'b0;
            frame_ending <= 1'b0;
            capture_frame_id <= 32'd0;
            x_count <= 11'd0;
            y_count <= 11'd0;
            rgb_x_phase <= 2'd0;
            rgb_y_phase <= 2'd0;
            reduced_x <= 10'd0;
            rgb_line0_x<=10'd0;rgb_line1_x<=10'd0;
            reduced_y <= 9'd0;
            rgb_r_acc <= 10'd0;
            rgb_g_acc <= 10'd0;
            rgb_b_acc <= 10'd0;
            gray_x_phase <= 3'd0;
            gray_y_phase <= 3'd0;
            gray_reduced_x <= 7'd0;
            gray_line0_x<=7'd0;gray_line1_x<=7'd0;gray_line2_x<=7'd0;gray_line3_x<=7'd0;
            gray_reduced_y <= 7'd0;
            gray_acc <= 11'd0;
            buffer_valid <= 1'b0;
            write_count <= 14'd0;
            s1_valid <= 1'b0;
            s1_r <= 12'd0;
            s1_g <= 12'd0;
            s1_b <= 12'd0;
            s2_valid <= 1'b0;
            s2_r <= 8'd0;
            s2_g <= 8'd0;
            s2_b <= 8'd0;
            s3_valid <= 1'b0;
            s3_gray <= 8'd0;
            s4_valid <= 1'b0;
            s4_vsum <= 13'd0;
            s4_address <= 14'd0;
            s5_valid <= 1'b0;
            s5_pixel <= 8'd0;
            s5_address <= 14'd0;
            image_valid <= 1'b0;
            image_frame_id <= 32'd0;
            read_valid <= 1'b0;
            read_data <= 8'd0;
            capture_error_count <= 32'd0;
            skipped_frame_count <= 32'd0;
        end else begin
            frame_active_d <= frame_active;

            if (image_valid && image_ready)
                image_valid <= 1'b0;

            // Preserve the original one-cycle synchronous read behavior.
            // image_ready consumes only the notification, not buffer_valid.
            read_valid <= read_en && buffer_valid;
            if (read_en && buffer_valid) begin
                read_row = read_address[13:7];
                read_col = read_address[6:0];
                if ((read_row < 28) || (read_row >= 100)) begin
                    read_data <= 8'd0;
                end else begin
                    thumbnail_index = ((read_row - 28) * 128) + read_col;
                    read_data <= thumbnail_ram[thumbnail_index];
                end
            end else begin
                read_data <= 8'd0;
            end

            // Arithmetic pipeline. Nonblocking assignments make the stage
            // boundaries T, T+1, T+2, T+3, T+4, and T+5 respectively.
            s1_valid <= rgb_emit;
            if (rgb_emit) begin
                s1_r <= rgb_vsum_r;
                s1_g <= rgb_vsum_g;
                s1_b <= rgb_vsum_b;
            end
            s2_valid <= s1_valid;
            if (s1_valid) begin
                s2_r <= s2_r_next;
                s2_g <= s2_g_next;
                s2_b <= s2_b_next;
            end
            s3_valid <= s2_valid;
            if (s2_valid)
                s3_gray <= s3_gray_next;
            s4_valid <= 1'b0;
            s5_valid <= s4_valid;
            if (s4_valid) begin
                s5_pixel <= s5_pixel_next;
                s5_address <= s4_address;
            end

            if (s3_valid && frame_tracking && !frame_bad && !abort_pipeline) begin
                case (gray_x_phase)
                    3'd0: begin
                        gray_acc <= {3'b000, s3_gray};
                        gray_x_phase <= 3'd1;
                    end
                    3'd1: begin
                        gray_acc <= gray_acc + {3'b000, s3_gray};
                        gray_x_phase <= 3'd2;
                    end
                    3'd2: begin
                        gray_acc <= gray_acc + {3'b000, s3_gray};
                        gray_x_phase <= 3'd3;
                    end
                    3'd3: begin
                        gray_acc <= gray_acc + {3'b000, s3_gray};
                        gray_x_phase <= 3'd4;
                    end
                    default: begin
                        gray_x_phase <= 3'd0;
                        case (gray_y_phase)
                            3'd0: gray_line0[gray_line0_x] <= gray_hsum;
                            3'd1: gray_line1[gray_line1_x] <= gray_hsum;
                            3'd2: gray_line2[gray_line2_x] <= gray_hsum;
                            3'd3: gray_line3[gray_line3_x] <= gray_hsum;
                            default: begin
                                gray_line4[gray_reduced_x] <= gray_hsum;
                                s4_valid <= 1'b1;
                                s4_vsum <= gray_vsum;
                                s4_address <= gray_address;
                            end
                        endcase
                        if (gray_reduced_x == 7'd127) begin
                            gray_reduced_x <= 7'd0;
                            gray_line0_x<=7'd0;gray_line1_x<=7'd0;gray_line2_x<=7'd0;gray_line3_x<=7'd0;
                            if (gray_y_phase == 3'd4) begin
                                gray_y_phase <= 3'd0;
                                if (gray_reduced_y != 7'd71)
                                    gray_reduced_y <= gray_reduced_y + 7'd1;
                            end else begin
                                gray_y_phase <= gray_y_phase + 3'd1;
                            end
                        end else begin
                            gray_reduced_x <= gray_reduced_x + 7'd1;
                            gray_line0_x<=gray_reduced_x+7'd1;gray_line1_x<=gray_reduced_x+7'd1;
                            gray_line2_x<=gray_reduced_x+7'd1;gray_line3_x<=gray_reduced_x+7'd1;
                        end
                    end
                endcase
            end

            if (s5_valid && frame_tracking && !frame_bad && !abort_pipeline) begin
                if (s5_address == write_count && s5_address < THUMBNAIL_BYTES &&
                    write_count < THUMBNAIL_BYTES) begin
                    thumbnail_ram[s5_address] <= s5_pixel;
                    write_count <= write_count + 14'd1;
                end else begin
                    frame_bad <= 1'b1;
                    frame_ending <= 1'b0;
                    buffer_valid <= 1'b0;
                    if (frame_ending)
                        frame_tracking <= 1'b0;
                    s1_valid <= 1'b0;
                    s2_valid <= 1'b0;
                    s3_valid <= 1'b0;
                    s4_valid <= 1'b0;
                    s5_valid <= 1'b0;
                    if (!frame_error_counted) begin
                        capture_error_count <= capture_error_count + 32'd1;
                        frame_error_counted <= 1'b1;
                    end
                end
            end

            if (frame_rise && !frame_tracking) begin
                frame_bad <= 1'b0;
                frame_error_counted <= 1'b0;
                raster_complete <= 1'b0;
                frame_ending <= 1'b0;
                write_count <= 14'd0;
                x_count <= 11'd0;
                y_count <= 11'd0;
                rgb_x_phase <= 2'd0;
                rgb_y_phase <= 2'd0;
                reduced_x <= 10'd0;
                rgb_line0_x<=10'd0;rgb_line1_x<=10'd0;
                reduced_y <= 9'd0;
                rgb_r_acc <= 10'd0;
                rgb_g_acc <= 10'd0;
                rgb_b_acc <= 10'd0;
                gray_x_phase <= 3'd0;
                gray_y_phase <= 3'd0;
                gray_reduced_x <= 7'd0;
                gray_line0_x<=7'd0;gray_line1_x<=7'd0;gray_line2_x<=7'd0;gray_line3_x<=7'd0;
                gray_reduced_y <= 7'd0;
                gray_acc <= 11'd0;
                s1_valid <= 1'b0;
                s2_valid <= 1'b0;
                s3_valid <= 1'b0;
                s4_valid <= 1'b0;
                s5_valid <= 1'b0;
                if (capture_enable && !image_valid) begin
                    frame_tracking <= 1'b1;
                    in_capture <= 1'b1;
                    buffer_valid <= 1'b0;
                    capture_frame_id <= source_frame_id;
                end else begin
                    frame_tracking <= 1'b0;
                    in_capture <= 1'b0;
                    skipped_frame_count <= skipped_frame_count + 32'd1;
                end
            end

            if (pixel_abort && frame_tracking) begin
                in_capture <= 1'b0;
                frame_bad <= 1'b1;
                frame_ending <= 1'b0;
                buffer_valid <= 1'b0;
                if (frame_ending)
                    frame_tracking <= 1'b0;
                s1_valid <= 1'b0;
                s2_valid <= 1'b0;
                s3_valid <= 1'b0;
                s4_valid <= 1'b0;
                s5_valid <= 1'b0;
                if (!frame_error_counted) begin
                    capture_error_count <= capture_error_count + 32'd1;
                    frame_error_counted <= 1'b1;
                end
            end else if (in_capture && pixel_valid) begin
                if (raster_complete) begin
                    in_capture <= 1'b0;
                    frame_bad <= 1'b1;
                    frame_ending <= 1'b0;
                    buffer_valid <= 1'b0;
                    s1_valid <= 1'b0;
                    s2_valid <= 1'b0;
                    s3_valid <= 1'b0;
                    s4_valid <= 1'b0;
                    s5_valid <= 1'b0;
                    if (!frame_error_counted) begin
                        capture_error_count <= capture_error_count + 32'd1;
                        frame_error_counted <= 1'b1;
                    end
                end else begin
                    // Preserve original horizontal triplets and rotating rows.
                    case (rgb_x_phase)
                        2'd0: begin
                            rgb_r_acc <= {2'b00, rgb[23:16]};
                            rgb_g_acc <= {2'b00, rgb[15:8]};
                            rgb_b_acc <= {2'b00, rgb[7:0]};
                            rgb_x_phase <= 2'd1;
                        end
                        2'd1: begin
                            rgb_r_acc <= rgb_r_acc + {2'b00, rgb[23:16]};
                            rgb_g_acc <= rgb_g_acc + {2'b00, rgb[15:8]};
                            rgb_b_acc <= rgb_b_acc + {2'b00, rgb[7:0]};
                            rgb_x_phase <= 2'd2;
                        end
                        default: begin
                            rgb_x_phase <= 2'd0;
                            case (rgb_y_phase)
                                2'd0: rgb_line0[rgb_line0_x] <=
                                    {rgb_hsum_r, rgb_hsum_g, rgb_hsum_b};
                                2'd1: rgb_line1[rgb_line1_x] <=
                                    {rgb_hsum_r, rgb_hsum_g, rgb_hsum_b};
                                default: begin
                                    rgb_line2[reduced_x] <=
                                        {rgb_hsum_r, rgb_hsum_g, rgb_hsum_b};
                                end
                            endcase
                            if (reduced_x == 10'd639) begin
                                reduced_x <= 10'd0;
                                rgb_line0_x<=10'd0;rgb_line1_x<=10'd0;
                                if (rgb_y_phase == 2'd2) begin
                                    rgb_y_phase <= 2'd0;
                                    reduced_y <= reduced_y + 9'd1;
                                end else begin
                                    rgb_y_phase <= rgb_y_phase + 2'd1;
                                end
                            end else begin
                                reduced_x <= reduced_x + 10'd1;
                                rgb_line0_x<=reduced_x+10'd1;rgb_line1_x<=reduced_x+10'd1;
                            end
                        end
                    endcase

                    // Raster position is internal; no external x/y is trusted.
                    if (x_count == LAST_X) begin
                        x_count <= 11'd0;
                        if (y_count == LAST_Y) begin
                            y_count <= LAST_Y;
                            in_capture <= 1'b0;
                            raster_complete <= 1'b1;
                        end else begin
                            y_count <= y_count + 11'd1;
                        end
                    end else begin
                        x_count <= x_count + 11'd1;
                    end
                end
            end

            if (extra_pixel) begin
                in_capture <= 1'b0;
                frame_bad <= 1'b1;
                frame_ending <= 1'b0;
                buffer_valid <= 1'b0;
                if (frame_ending)
                    frame_tracking <= 1'b0;
                s1_valid <= 1'b0;
                s2_valid <= 1'b0;
                s3_valid <= 1'b0;
                s4_valid <= 1'b0;
                s5_valid <= 1'b0;
                if (!frame_error_counted) begin
                    capture_error_count <= capture_error_count + 32'd1;
                    frame_error_counted <= 1'b1;
                end
            end

            if (frame_fall && frame_tracking) begin
                in_capture <= 1'b0;
                if (good_frame_fall) begin
                    // Keep tracking through pipeline drain; publish is delayed.
                    frame_ending <= 1'b1;
                end else begin
                    frame_tracking <= 1'b0;
                    frame_bad <= 1'b1;
                    frame_ending <= 1'b0;
                    buffer_valid <= 1'b0;
                    s1_valid <= 1'b0;
                    s2_valid <= 1'b0;
                    s3_valid <= 1'b0;
                    s4_valid <= 1'b0;
                    s5_valid <= 1'b0;
                    if (!frame_error_counted && !pixel_abort && !extra_pixel) begin
                        capture_error_count <= capture_error_count + 32'd1;
                        frame_error_counted <= 1'b1;
                    end
                end
            end

            // A new frame beginning before drain invalidates the old capture
            // and is skipped. Count the failed tracked frame only once.
            if (overlap_rise) begin
                in_capture <= 1'b0;
                frame_bad <= 1'b1;
                frame_ending <= 1'b0;
                buffer_valid <= 1'b0;
                s1_valid <= 1'b0;
                s2_valid <= 1'b0;
                s3_valid <= 1'b0;
                s4_valid <= 1'b0;
                s5_valid <= 1'b0;
                skipped_frame_count <= skipped_frame_count + 32'd1;
                if (!frame_error_counted) begin
                    capture_error_count <= capture_error_count + 32'd1;
                    frame_error_counted <= 1'b1;
                end
            end

            // Do not gate draining on in_capture: the final six stages must
            // finish after the full-raster frame_active falling edge.
            if (frame_ending && pipeline_empty && frame_tracking && !frame_bad &&
                !abort_pipeline) begin
                if (write_count == THUMBNAIL_BYTES) begin
                    image_valid <= 1'b1;
                    buffer_valid <= 1'b1;
                    image_frame_id <= capture_frame_id;
                end else begin
                    buffer_valid <= 1'b0;
                    if (!frame_error_counted) begin
                        capture_error_count <= capture_error_count + 32'd1;
                        frame_error_counted <= 1'b1;
                    end
                end
                frame_tracking <= 1'b0;
                in_capture <= 1'b0;
                frame_ending <= 1'b0;
            end

            if (abort_pipeline) begin
                s1_valid <= 1'b0;
                s2_valid <= 1'b0;
                s3_valid <= 1'b0;
                s4_valid <= 1'b0;
                s5_valid <= 1'b0;
            end
        end
    end
endmodule
