// PDS-importable Verilog-2001 coordinate engine for basic lens correction.
//
// Supported model: second-order radial k1/k2 and tangential p1/p2.
// Geometry is identity. Rotation, crop, scale, and affine composition are not
// implemented in this module.
//
// Candidate formats (provisional until a real camera profile is scanned):
//   fx/fy/cx/cy:                 signed 32-bit Q16
//   inv_fx/inv_fy/k1/k2/p1/p2: signed 32-bit Q24
//   normalized coordinates:     signed 32-bit Q16
//   source coordinates:        signed COORD_W-bit, COORD_F fractional bits
//
// Eight elastic stages separate normalization, polynomial products, radial
// gain, tangential terms, projection, and output classification. COORD_W must
// be COORD_F+2..32 and COORD_F must be 1..16. The active camera profile must remain
// stable until coord_pipe_empty is high. Commit a new profile only at a frame
// boundary after the pipeline drains.

module basic_distortion_coord #(
    parameter integer COORD_W = 26,
    parameter integer COORD_F = 10
) (
    input               coord_clk,
    input               coord_rst_n,

    input               coord_req_valid,
    output              coord_req_ready,
    input      [15:0]   coord_req_x,
    input      [15:0]   coord_req_y,
    input      [31:0]   coord_req_frame_id,
    input      [15:0]   coord_req_cfg_version,
    input      [31:0]   coord_req_pixel_tag,

    // Frame-boundary Active Camera Profile. Keep every cfg_active_* input
    // stable until coord_pipe_empty is high and no old result is stalled.
    input      [15:0]   cfg_active_source_width,
    input      [15:0]   cfg_active_source_height,
    input signed [31:0] cfg_active_fx_q16,
    input signed [31:0] cfg_active_fy_q16,
    input signed [31:0] cfg_active_cx_q16,
    input signed [31:0] cfg_active_cy_q16,
    input signed [31:0] cfg_active_inv_fx_q24,
    input signed [31:0] cfg_active_inv_fy_q24,
    input signed [31:0] cfg_active_k1_q24,
    input signed [31:0] cfg_active_k2_q24,
    input signed [31:0] cfg_active_p1_q24,
    input signed [31:0] cfg_active_p2_q24,

    output              coord_res_valid,
    input               coord_res_ready,
    output reg signed [COORD_W-1:0] src_x_fixed,
    output reg signed [COORD_W-1:0] src_y_fixed,
    output reg        [1:0]  border,
    output reg       [31:0]  coord_res_frame_id,
    output reg       [15:0]  coord_res_cfg_version,
    output reg       [31:0]  coord_res_pixel_tag,
    output              coord_pipe_empty
);

    localparam signed [31:0] Q16_ONE = 32'sd65536;
    localparam integer COORD_SHIFT = 16 - COORD_F;
    localparam [63:0] COORD_ROUND_BIAS =
        64'd1 << ((COORD_F == 16) ? 0 : (COORD_SHIFT - 1));
    localparam signed [63:0] COORD_MAX_64 =
        (64'sd1 <<< (COORD_W - 1)) - 64'sd1;
    localparam signed [63:0] COORD_MIN_64 =
        -(64'sd1 <<< (COORD_W - 1));

    reg [7:0] valid_pipe;
    wire [7:0] stage_ready;

    reg signed [31:0] x_pipe [0:3];
    reg signed [31:0] y_pipe [0:3];
    reg signed [31:0] x2_pipe [1:4];
    reg signed [31:0] y2_pipe [1:4];
    reg signed [31:0] xy_pipe [1:3];
    reg signed [31:0] r2_pipe [1:4];
    reg signed [31:0] r4_pipe [2:2];
    reg signed [31:0] k1r2_pipe [2:2];
    reg signed [31:0] gain_pipe [3:3];
    reg signed [31:0] radial_x_pipe [4:4];
    reg signed [31:0] radial_y_pipe [4:4];
    reg signed [31:0] p1xy_pipe [4:4];
    reg signed [31:0] p2xy_pipe [4:4];
    reg signed [31:0] xd_pipe [5:5];
    reg signed [31:0] yd_pipe [5:5];
    reg signed [31:0] src_x_q16_pipe [6:6];
    reg signed [31:0] src_y_q16_pipe [6:6];

    reg [15:0] width_pipe [0:6];
    reg [15:0] height_pipe [0:6];
    reg [31:0] frame_id_pipe [0:6];
    reg [15:0] cfg_version_pipe [0:6];
    reg [31:0] pixel_tag_pipe [0:6];

    function signed [31:0] mul_q16_to_q16;
        input signed [31:0] a_q16;
        input signed [31:0] b_q16;
        reg signed [63:0] a_ext;
        reg signed [63:0] b_ext;
        reg signed [63:0] product;
        reg [63:0] magnitude;
        reg [63:0] rounded;
        begin
            a_ext = {{32{a_q16[31]}}, a_q16};
            b_ext = {{32{b_q16[31]}}, b_q16};
            product = a_ext * b_ext;
            if (product[63]) begin
                magnitude = (~product) + 64'd1;
                rounded = (magnitude + 64'd32768) >> 16;
                mul_q16_to_q16 = -$signed(rounded[31:0]);
            end else begin
                magnitude = product;
                rounded = (magnitude + 64'd32768) >> 16;
                mul_q16_to_q16 = $signed(rounded[31:0]);
            end
        end
    endfunction

    function signed [31:0] mul_q24_q16_to_q16;
        input signed [31:0] a_q24;
        input signed [31:0] b_q16;
        reg signed [63:0] a_ext;
        reg signed [63:0] b_ext;
        reg signed [63:0] product;
        reg [63:0] magnitude;
        reg [63:0] rounded;
        begin
            a_ext = {{32{a_q24[31]}}, a_q24};
            b_ext = {{32{b_q16[31]}}, b_q16};
            product = a_ext * b_ext;
            if (product[63]) begin
                magnitude = (~product) + 64'd1;
                rounded = (magnitude + 64'd8388608) >> 24;
                mul_q24_q16_to_q16 = -$signed(rounded[31:0]);
            end else begin
                magnitude = product;
                rounded = (magnitude + 64'd8388608) >> 24;
                mul_q24_q16_to_q16 = $signed(rounded[31:0]);
            end
        end
    endfunction

    function signed [63:0] q16_to_coord;
        input signed [31:0] value_q16;
        reg signed [63:0] value_ext;
        reg [63:0] magnitude;
        reg [63:0] rounded;
        begin
            value_ext = {{32{value_q16[31]}}, value_q16};
            if (COORD_F == 16) begin
                q16_to_coord = value_ext;
            end else if (value_ext[63]) begin
                magnitude = (~value_ext) + 64'd1;
                rounded = (magnitude + COORD_ROUND_BIAS) >> COORD_SHIFT;
                q16_to_coord = -$signed(rounded);
            end else begin
                magnitude = value_ext;
                rounded = (magnitude + COORD_ROUND_BIAS) >> COORD_SHIFT;
                q16_to_coord = $signed(rounded);
            end
        end
    endfunction

    wire signed [31:0] in_x_q16 =
        $signed({16'd0, coord_req_x}) <<< 16;
    wire signed [31:0] in_y_q16 =
        $signed({16'd0, coord_req_y}) <<< 16;
    wire signed [31:0] delta_x_q16 = in_x_q16 - cfg_active_cx_q16;
    wire signed [31:0] delta_y_q16 = in_y_q16 - cfg_active_cy_q16;
    wire signed [31:0] normalized_x =
        mul_q24_q16_to_q16(cfg_active_inv_fx_q24, delta_x_q16);
    wire signed [31:0] normalized_y =
        mul_q24_q16_to_q16(cfg_active_inv_fy_q24, delta_y_q16);

    wire signed [31:0] x2_stage1 =
        mul_q16_to_q16(x_pipe[0], x_pipe[0]);
    wire signed [31:0] y2_stage1 =
        mul_q16_to_q16(y_pipe[0], y_pipe[0]);
    wire signed [31:0] xy_stage1 =
        mul_q16_to_q16(x_pipe[0], y_pipe[0]);

    wire signed [31:0] max_x_q16 =
        ($signed({16'd0, width_pipe[6]}) - 32'sd1) <<< 16;
    wire signed [31:0] max_y_q16 =
        ($signed({16'd0, height_pipe[6]}) - 32'sd1) <<< 16;
    wire signed [63:0] src_x_coord_wide =
        q16_to_coord(src_x_q16_pipe[6]);
    wire signed [63:0] src_y_coord_wide =
        q16_to_coord(src_y_q16_pipe[6]);

    assign stage_ready[7] = (~valid_pipe[7]) | coord_res_ready;
    assign stage_ready[6] = (~valid_pipe[6]) | stage_ready[7];
    assign stage_ready[5] = (~valid_pipe[5]) | stage_ready[6];
    assign stage_ready[4] = (~valid_pipe[4]) | stage_ready[5];
    assign stage_ready[3] = (~valid_pipe[3]) | stage_ready[4];
    assign stage_ready[2] = (~valid_pipe[2]) | stage_ready[3];
    assign stage_ready[1] = (~valid_pipe[1]) | stage_ready[2];
    assign stage_ready[0] = (~valid_pipe[0]) | stage_ready[1];

    assign coord_req_ready = stage_ready[0];
    assign coord_res_valid = valid_pipe[7];
    assign coord_pipe_empty = ~(|valid_pipe);

    always @(posedge coord_clk or negedge coord_rst_n) begin
        if (!coord_rst_n) begin
            valid_pipe <= 8'b0;
            src_x_fixed <= {COORD_W{1'b0}};
            src_y_fixed <= {COORD_W{1'b0}};
            border <= 2'b10;
            coord_res_frame_id <= 32'd0;
            coord_res_cfg_version <= 16'd0;
            coord_res_pixel_tag <= 32'd0;
        end else begin
            if (stage_ready[0]) begin
                valid_pipe[0] <= coord_req_valid;
                if (coord_req_valid) begin
                    x_pipe[0] <= normalized_x;
                    y_pipe[0] <= normalized_y;
                    width_pipe[0] <= cfg_active_source_width;
                    height_pipe[0] <= cfg_active_source_height;
                    frame_id_pipe[0] <= coord_req_frame_id;
                    cfg_version_pipe[0] <= coord_req_cfg_version;
                    pixel_tag_pipe[0] <= coord_req_pixel_tag;
                end
            end

            if (stage_ready[1]) begin
                valid_pipe[1] <= valid_pipe[0];
                if (valid_pipe[0]) begin
                    x_pipe[1] <= x_pipe[0];
                    y_pipe[1] <= y_pipe[0];
                    x2_pipe[1] <= x2_stage1;
                    y2_pipe[1] <= y2_stage1;
                    xy_pipe[1] <= xy_stage1;
                    r2_pipe[1] <= x2_stage1 + y2_stage1;
                    width_pipe[1] <= width_pipe[0];
                    height_pipe[1] <= height_pipe[0];
                    frame_id_pipe[1] <= frame_id_pipe[0];
                    cfg_version_pipe[1] <= cfg_version_pipe[0];
                    pixel_tag_pipe[1] <= pixel_tag_pipe[0];
                end
            end

            if (stage_ready[2]) begin
                valid_pipe[2] <= valid_pipe[1];
                if (valid_pipe[1]) begin
                    x_pipe[2] <= x_pipe[1];
                    y_pipe[2] <= y_pipe[1];
                    x2_pipe[2] <= x2_pipe[1];
                    y2_pipe[2] <= y2_pipe[1];
                    xy_pipe[2] <= xy_pipe[1];
                    r2_pipe[2] <= r2_pipe[1];
                    r4_pipe[2] <= mul_q16_to_q16(r2_pipe[1], r2_pipe[1]);
                    k1r2_pipe[2] <=
                        mul_q24_q16_to_q16(cfg_active_k1_q24, r2_pipe[1]);
                    width_pipe[2] <= width_pipe[1];
                    height_pipe[2] <= height_pipe[1];
                    frame_id_pipe[2] <= frame_id_pipe[1];
                    cfg_version_pipe[2] <= cfg_version_pipe[1];
                    pixel_tag_pipe[2] <= pixel_tag_pipe[1];
                end
            end

            if (stage_ready[3]) begin
                valid_pipe[3] <= valid_pipe[2];
                if (valid_pipe[2]) begin
                    x_pipe[3] <= x_pipe[2];
                    y_pipe[3] <= y_pipe[2];
                    x2_pipe[3] <= x2_pipe[2];
                    y2_pipe[3] <= y2_pipe[2];
                    xy_pipe[3] <= xy_pipe[2];
                    r2_pipe[3] <= r2_pipe[2];
                    gain_pipe[3] <= Q16_ONE
                        + k1r2_pipe[2]
                        + mul_q24_q16_to_q16(cfg_active_k2_q24, r4_pipe[2]);
                    width_pipe[3] <= width_pipe[2];
                    height_pipe[3] <= height_pipe[2];
                    frame_id_pipe[3] <= frame_id_pipe[2];
                    cfg_version_pipe[3] <= cfg_version_pipe[2];
                    pixel_tag_pipe[3] <= pixel_tag_pipe[2];
                end
            end

            if (stage_ready[4]) begin
                valid_pipe[4] <= valid_pipe[3];
                if (valid_pipe[3]) begin
                    x2_pipe[4] <= x2_pipe[3];
                    y2_pipe[4] <= y2_pipe[3];
                    r2_pipe[4] <= r2_pipe[3];
                    radial_x_pipe[4] <=
                        mul_q16_to_q16(x_pipe[3], gain_pipe[3]);
                    radial_y_pipe[4] <=
                        mul_q16_to_q16(y_pipe[3], gain_pipe[3]);
                    p1xy_pipe[4] <=
                        mul_q24_q16_to_q16(cfg_active_p1_q24, xy_pipe[3]);
                    p2xy_pipe[4] <=
                        mul_q24_q16_to_q16(cfg_active_p2_q24, xy_pipe[3]);
                    width_pipe[4] <= width_pipe[3];
                    height_pipe[4] <= height_pipe[3];
                    frame_id_pipe[4] <= frame_id_pipe[3];
                    cfg_version_pipe[4] <= cfg_version_pipe[3];
                    pixel_tag_pipe[4] <= pixel_tag_pipe[3];
                end
            end

            if (stage_ready[5]) begin
                valid_pipe[5] <= valid_pipe[4];
                if (valid_pipe[4]) begin
                    xd_pipe[5] <= radial_x_pipe[4]
                        + (p1xy_pipe[4] <<< 1)
                        + mul_q24_q16_to_q16(
                            cfg_active_p2_q24,
                            r2_pipe[4] + (x2_pipe[4] <<< 1)
                          );
                    yd_pipe[5] <= radial_y_pipe[4]
                        + mul_q24_q16_to_q16(
                            cfg_active_p1_q24,
                            r2_pipe[4] + (y2_pipe[4] <<< 1)
                          )
                        + (p2xy_pipe[4] <<< 1);
                    width_pipe[5] <= width_pipe[4];
                    height_pipe[5] <= height_pipe[4];
                    frame_id_pipe[5] <= frame_id_pipe[4];
                    cfg_version_pipe[5] <= cfg_version_pipe[4];
                    pixel_tag_pipe[5] <= pixel_tag_pipe[4];
                end
            end

            if (stage_ready[6]) begin
                valid_pipe[6] <= valid_pipe[5];
                if (valid_pipe[5]) begin
                    src_x_q16_pipe[6] <=
                        mul_q16_to_q16(cfg_active_fx_q16, xd_pipe[5])
                        + cfg_active_cx_q16;
                    src_y_q16_pipe[6] <=
                        mul_q16_to_q16(cfg_active_fy_q16, yd_pipe[5])
                        + cfg_active_cy_q16;
                    width_pipe[6] <= width_pipe[5];
                    height_pipe[6] <= height_pipe[5];
                    frame_id_pipe[6] <= frame_id_pipe[5];
                    cfg_version_pipe[6] <= cfg_version_pipe[5];
                    pixel_tag_pipe[6] <= pixel_tag_pipe[5];
                end
            end

            if (stage_ready[7]) begin
                valid_pipe[7] <= valid_pipe[6];
                if (valid_pipe[6]) begin
                    if (src_x_coord_wide > COORD_MAX_64)
                        src_x_fixed <= {1'b0, {(COORD_W-1){1'b1}}};
                    else if (src_x_coord_wide < COORD_MIN_64)
                        src_x_fixed <= {1'b1, {(COORD_W-1){1'b0}}};
                    else
                        src_x_fixed <= src_x_coord_wide[COORD_W-1:0];

                    if (src_y_coord_wide > COORD_MAX_64)
                        src_y_fixed <= {1'b0, {(COORD_W-1){1'b1}}};
                    else if (src_y_coord_wide < COORD_MIN_64)
                        src_y_fixed <= {1'b1, {(COORD_W-1){1'b0}}};
                    else
                        src_y_fixed <= src_y_coord_wide[COORD_W-1:0];

                    if ((width_pipe[6] == 16'd0) ||
                        (height_pipe[6] == 16'd0) ||
                        (src_x_q16_pipe[6] < 32'sd0) ||
                        (src_y_q16_pipe[6] < 32'sd0) ||
                        (src_x_q16_pipe[6] > max_x_q16) ||
                        (src_y_q16_pipe[6] > max_y_q16) ||
                        (src_x_coord_wide > COORD_MAX_64) ||
                        (src_x_coord_wide < COORD_MIN_64) ||
                        (src_y_coord_wide > COORD_MAX_64) ||
                        (src_y_coord_wide < COORD_MIN_64)) begin
                        border <= 2'b10;
                    end else if ((width_pipe[6] < 16'd2) ||
                                 (height_pipe[6] < 16'd2) ||
                                 (src_x_q16_pipe[6] >= max_x_q16) ||
                                 (src_y_q16_pipe[6] >= max_y_q16)) begin
                        border <= 2'b01;
                    end else begin
                        border <= 2'b00;
                    end

                    coord_res_frame_id <= frame_id_pipe[6];
                    coord_res_cfg_version <= cfg_version_pipe[6];
                    coord_res_pixel_tag <= pixel_tag_pipe[6];
                end
            end
        end
    end

endmodule
