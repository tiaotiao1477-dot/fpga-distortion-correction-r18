// Verilog-2001 candidate: inverse affine + distortion, with atomic frame config.
// All ports are in coord_clk. A UART/CDC adapter must supply complete checked
// packets. Config payload layout is documented in docs/dynamic_distortion.md.
// Frame start is a handshake, NOT HDMI VSYNC. The scanner must mark the last
// accepted request with coord_req_last. downstream_idle covers Fetch/Bilinear
// and result-writer requests from the preceding processing frame.
module dynamic_distortion_coord_staged_v04 #(
    parameter integer COORD_W = 26,
    parameter integer COORD_F = 10,
    parameter integer MAX_OUTPUT_WIDTH = 32768
) (
    input coord_clk,
    input coord_rst_n,
    input cfg_valid,
    output cfg_ready,
    input [735:0] cfg_payload,
    input [31:0] cfg_seq,
    output reg cfg_received,
    output reg cfg_rejected,
    output reg [31:0] cfg_received_seq,
    output reg cfg_applied,
    output reg [31:0] cfg_applied_seq,
    output reg [31:0] cfg_applied_frame_id,
    output reg [15:0] active_cfg_version,
    output reg active_valid,
    output reg pending_valid,
    input frame_start_valid,
    output frame_start_ready,
    input [31:0] frame_start_id,
    input downstream_idle,
    output [15:0] active_output_width,
    output [15:0] active_output_height,
    input coord_req_valid,
    output coord_req_ready,
    input [15:0] coord_req_x,
    input [15:0] coord_req_y,
    input coord_req_last,
    input [31:0] coord_req_pixel_tag,
    output coord_res_valid,
    input coord_res_ready,
    output signed [COORD_W-1:0] src_x_fixed,
    output signed [COORD_W-1:0] src_y_fixed,
    output [1:0] border,
    output [31:0] coord_res_frame_id,
    output [15:0] coord_res_cfg_version,
    output [31:0] coord_res_pixel_tag,
    output coord_pipe_empty,
    output coord_res_invalid,
    output [15:0] active_source_width,
    output [15:0] active_source_height,
    output active_border_mode,
    output [10:0] active_edge_threshold,
    output [1:0] active_edge_mode
);
    reg [735:0] pending_payload;
    reg [735:0] active_payload;
    reg [31:0] pending_seq;
    reg [31:0] processing_frame_id;
    reg frame_open;
    wire frame_transfer = frame_start_valid && frame_start_ready;
    wire request_transfer = coord_req_valid && coord_req_ready;
    // Capture, multiply, compare, then acknowledge the held complete payload.
    // This removes the combinational determinant from the Pending write enable.
    reg [1:0] cfg_phase;
    reg [735:0] cfg_candidate;
    reg [31:0] cfg_candidate_seq;
    reg signed [63:0] det_left_reg,det_right_reg;
    reg cfg_scalar_legal,cfg_qualified;
    assign cfg_ready = coord_rst_n && cfg_phase==3;
    assign active_source_width = active_payload[0*32 +: 16];
    assign active_source_height = active_payload[1*32 +: 16];
    assign active_border_mode = active_payload[20*32];
    assign active_edge_threshold = active_payload[21*32 +: 11];
    assign active_edge_mode = active_payload[22*32 +: 2];
    assign active_output_width = active_payload[18*32 +: 16];
    assign active_output_height = active_payload[19*32 +: 16];

    wire signed [31:0] a00 = active_payload[12*32 +: 32];
    wire signed [31:0] a01 = active_payload[13*32 +: 32];
    wire signed [31:0] a02 = active_payload[14*32 +: 32];
    wire signed [31:0] a10 = active_payload[15*32 +: 32];
    wire signed [31:0] a11 = active_payload[16*32 +: 32];
    wire signed [31:0] a12 = active_payload[17*32 +: 32];

    // Each signed32 Q16 * unsigned16 product needs 49 bits. Explicit 64-bit
    // sign/zero extension also leaves guard bits for the two-product sum.
    wire signed [63:0] u_ext = $signed({48'd0, coord_req_x});
    wire signed [63:0] v_ext = $signed({48'd0, coord_req_y});
    wire signed [63:0] ax_wide =
        $signed({{32{a00[31]}}, a00}) * u_ext +
        $signed({{32{a01[31]}}, a01}) * v_ext +
        $signed({{32{a02[31]}}, a02});
    wire signed [63:0] ay_wide =
        $signed({{32{a10[31]}}, a10}) * u_ext +
        $signed({{32{a11[31]}}, a11}) * v_ext +
        $signed({{32{a12[31]}}, a12});
    wire affine_overflow = (ax_wide > 64'sd2147483647) ||
        (ax_wide < -64'sd2147483648) || (ay_wide > 64'sd2147483647) ||
        (ay_wide < -64'sd2147483648);

    reg affine_valid;
    reg signed [31:0] ideal_x_q16;
    reg signed [31:0] ideal_y_q16;
    reg affine_invalid;
    reg [31:0] affine_frame_id;
    reg [15:0] affine_cfg_version;
    reg [31:0] affine_pixel_tag;
    wire core_ready;
    wire core_empty;
    wire affine_ready = !affine_valid || core_ready;
    assign coord_req_ready = coord_rst_n && active_valid && frame_open && affine_ready;
    assign coord_pipe_empty = !affine_valid && core_empty;
    assign frame_start_ready = coord_rst_n && !frame_open && coord_pipe_empty &&
        downstream_idle && (active_valid || pending_valid);

    // Structural checks only. Intermediate polynomial range qualification is
    // performed by the host scan for each supported profile/angle/canvas.
    wire signed [31:0] new_a00 = cfg_candidate[12*32 +: 32];
    wire signed [31:0] new_a01 = cfg_candidate[13*32 +: 32];
    wire signed [31:0] new_a10 = cfg_candidate[15*32 +: 32];
    wire signed [31:0] new_a11 = cfg_candidate[16*32 +: 32];
    wire signed [63:0] det_left_comb =
        $signed({{32{new_a00[31]}},new_a00}) * $signed({{32{new_a11[31]}},new_a11});
    wire signed [63:0] det_right_comb =
        $signed({{32{new_a01[31]}},new_a01}) * $signed({{32{new_a10[31]}},new_a10});
    wire cfg_shape_legal =
        (cfg_candidate[0*32 +: 32] > 0) && (cfg_candidate[0*32 +: 32] <= 32768) &&
        (cfg_candidate[1*32 +: 32] > 0) && (cfg_candidate[1*32 +: 32] <= 32768) &&
        (cfg_candidate[18*32 +: 32] > 0) && (cfg_candidate[18*32 +: 32] <= MAX_OUTPUT_WIDTH) &&
        (cfg_candidate[19*32 +: 32] > 0) && (cfg_candidate[19*32 +: 32] <= 32768) &&
        ($signed(cfg_candidate[2*32 +: 32]) > 0) &&
        ($signed(cfg_candidate[3*32 +: 32]) > 0) &&
        ($signed(cfg_candidate[6*32 +: 32]) > 0) &&
        ($signed(cfg_candidate[7*32 +: 32]) > 0) &&
        (cfg_candidate[20*32 +: 32] <= 1) &&
        (cfg_candidate[21*32 +: 32] <= 2047) &&
        (cfg_candidate[22*32 +: 32] <= 3);

    wire cfg_legal=cfg_qualified;
    always @(posedge coord_clk or negedge coord_rst_n)begin
        if(!coord_rst_n)cfg_phase<=0;
        else if(!cfg_valid)cfg_phase<=0;
        else case(cfg_phase)
            0:cfg_phase<=1;
            1:cfg_phase<=2;
            2:cfg_phase<=3;
            3:cfg_phase<=0;
        endcase
    end
    // Data registers intentionally have no async reset; validity/phase masks
    // them until a complete post-reset packet and frame have been accepted.
    always @(posedge coord_clk)if(coord_rst_n)begin
        if(cfg_phase==0&&cfg_valid)begin cfg_candidate<=cfg_payload;cfg_candidate_seq<=cfg_seq;end
        if(cfg_phase==1&&cfg_valid)begin
            det_left_reg<=det_left_comb;det_right_reg<=det_right_comb;
            cfg_scalar_legal<=cfg_shape_legal;
        end
        if(cfg_phase==2&&cfg_valid)cfg_qualified<=cfg_scalar_legal&&(det_left_reg!=det_right_reg);
        if(frame_transfer&&pending_valid)active_payload<=pending_payload;
        if(cfg_valid&&cfg_ready&&cfg_legal)pending_payload<=cfg_candidate;
    end

    always @(posedge coord_clk or negedge coord_rst_n) begin
        if (!coord_rst_n) begin
            pending_seq <= 32'd0;
            processing_frame_id <= 32'd0;
            frame_open <= 1'b0;
            active_valid <= 1'b0;
            pending_valid <= 1'b0;
            active_cfg_version <= 16'd0;
            cfg_received <= 1'b0;
            cfg_rejected <= 1'b0;
            cfg_received_seq <= 32'd0;
            cfg_applied <= 1'b0;
            cfg_applied_seq <= 32'd0;
            cfg_applied_frame_id <= 32'd0;
            affine_valid <= 1'b0;
        end else begin
            cfg_received <= 1'b0;
            cfg_rejected <= 1'b0;
            cfg_applied <= 1'b0;
            if (frame_transfer) begin
                frame_open <= 1'b1;
                processing_frame_id <= frame_start_id;
                if (pending_valid) begin
                    active_valid <= 1'b1;
                    pending_valid <= 1'b0;
                    active_cfg_version <= active_cfg_version + 16'd1;
                    cfg_applied <= 1'b1;
                    cfg_applied_seq <= pending_seq;
                    cfg_applied_frame_id <= frame_start_id;
                end
            end
            // A packet arriving on the commit edge belongs to the NEXT frame.
            // The old Pending is committed, then this assignment keeps new
            // Pending valid. Latest complete legal packet wins between frames.
            if (cfg_valid && cfg_ready) begin
                cfg_received_seq <= cfg_candidate_seq;
                if (cfg_legal) begin
                    pending_seq <= cfg_candidate_seq;
                    pending_valid <= 1'b1;
                    cfg_received <= 1'b1;
                end else cfg_rejected <= 1'b1;
            end
            if (request_transfer && coord_req_last) frame_open <= 1'b0;
            if (affine_ready) begin
                affine_valid <= request_transfer;
                if (request_transfer) begin
                    // Invalid affine results are not source addresses. Feed
                    // a safe zero pair and preserve the invalid border bit.
                    ideal_x_q16 <= affine_overflow ? 32'sd0 : ax_wide[31:0];
                    ideal_y_q16 <= affine_overflow ? 32'sd0 : ay_wide[31:0];
                    affine_invalid <= affine_overflow;
                    affine_frame_id <= processing_frame_id;
                    affine_cfg_version <= active_cfg_version;
                    affine_pixel_tag <= coord_req_pixel_tag;
                end
            end
        end
    end

    basic_distortion_coord_q16_v04 #(.COORD_W(COORD_W), .COORD_F(COORD_F)) core (
        .coord_clk(coord_clk), .coord_rst_n(coord_rst_n),
        .coord_req_valid(affine_valid), .coord_req_ready(core_ready),
        .coord_req_x(ideal_x_q16), .coord_req_y(ideal_y_q16),
        .coord_req_invalid(affine_invalid), .coord_req_frame_id(affine_frame_id),
        .coord_req_cfg_version(affine_cfg_version), .coord_req_pixel_tag(affine_pixel_tag),
        .cfg_active_source_width(active_payload[0*32 +: 16]),
        .cfg_active_source_height(active_payload[1*32 +: 16]),
        .cfg_active_fx_q16(active_payload[2*32 +: 32]),
        .cfg_active_fy_q16(active_payload[3*32 +: 32]),
        .cfg_active_cx_q16(active_payload[4*32 +: 32]),
        .cfg_active_cy_q16(active_payload[5*32 +: 32]),
        .cfg_active_inv_fx_q24(active_payload[6*32 +: 32]),
        .cfg_active_inv_fy_q24(active_payload[7*32 +: 32]),
        .cfg_active_k1_q24(active_payload[8*32 +: 32]),
        .cfg_active_k2_q24(active_payload[9*32 +: 32]),
        .cfg_active_p1_q24(active_payload[10*32 +: 32]),
        .cfg_active_p2_q24(active_payload[11*32 +: 32]),
        .coord_res_valid(coord_res_valid), .coord_res_ready(coord_res_ready),
        .src_x_fixed(src_x_fixed), .src_y_fixed(src_y_fixed), .border(border),
        .coord_res_frame_id(coord_res_frame_id), .coord_res_cfg_version(coord_res_cfg_version),
        .coord_res_pixel_tag(coord_res_pixel_tag), .coord_pipe_empty(core_empty), .coord_res_invalid(coord_res_invalid)
    );
endmodule
