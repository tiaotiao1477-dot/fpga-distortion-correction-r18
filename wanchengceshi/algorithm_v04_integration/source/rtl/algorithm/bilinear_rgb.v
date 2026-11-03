// PDS-importable Verilog-2001 Fetch4 RGB bilinear interpolator.
//
// Fraction fields use COORD_F, defaulting to 10 to match candidate S(26,10).
// RGB packing follows the project convention: R[23:16], G[15:8], B[7:0].
// Border handling and DDR reads belong to Pixel Fetch before this module.
// COORD_F must be in 1..16; final rounding follows fixed_point_spec.md.

module bilinear_rgb #(
    parameter integer COORD_F = 10
) (
    input              bilin_clk,
    input              bilin_rst_n,

    input              bilin_valid,
    output             bilin_ready,
    input      [23:0]  P00_rgb,
    input      [23:0]  P10_rgb,
    input      [23:0]  P01_rgb,
    input      [23:0]  P11_rgb,
    input      [COORD_F-1:0] dx,
    input      [COORD_F-1:0] dy,
    input      [31:0]  bilin_frame_id,
    input      [15:0]  bilin_cfg_version,
    input      [31:0]  bilin_pixel_tag,

    output             bilin_out_valid,
    input              bilin_out_ready,
    output reg [23:0]  pixel_rgb,
    output reg [31:0]  bilin_out_frame_id,
    output reg [15:0]  bilin_out_cfg_version,
    output reg [31:0]  bilin_out_pixel_tag
);

    localparam integer H_W = COORD_F + 9;
    localparam integer NUM_W = 2 * COORD_F + 10;

    reg [3:0] valid_pipe;
    wire [3:0] stage_ready;

    reg [23:0] p00_pipe [0:0];
    reg [23:0] p10_pipe [0:0];
    reg [23:0] p01_pipe [0:0];
    reg [23:0] p11_pipe [0:0];
    reg [COORD_F-1:0] dx_pipe [0:0];
    reg [COORD_F-1:0] dy_pipe [0:1];

    reg [H_W-1:0] h0_r_pipe [1:1];
    reg [H_W-1:0] h0_g_pipe [1:1];
    reg [H_W-1:0] h0_b_pipe [1:1];
    reg [H_W-1:0] h1_r_pipe [1:1];
    reg [H_W-1:0] h1_g_pipe [1:1];
    reg [H_W-1:0] h1_b_pipe [1:1];

    reg signed [NUM_W-1:0] numerator_r_pipe [2:2];
    reg signed [NUM_W-1:0] numerator_g_pipe [2:2];
    reg signed [NUM_W-1:0] numerator_b_pipe [2:2];

    reg [31:0] frame_id_pipe [0:2];
    reg [15:0] cfg_version_pipe [0:2];
    reg [31:0] pixel_tag_pipe [0:2];

    function [H_W-1:0] horizontal_qf;
        input [7:0] p0;
        input [7:0] p1;
        input [COORD_F-1:0] frac;
        reg signed [8:0] difference;
        reg signed [63:0] difference_ext;
        reg signed [63:0] fraction_ext;
        reg signed [63:0] product_full;
        reg signed [63:0] base;
        reg signed [63:0] total;
        begin
            difference = $signed({1'b0, p1}) - $signed({1'b0, p0});
            difference_ext = {{55{difference[8]}}, difference};
            fraction_ext = $signed({{(63-COORD_F){1'b0}}, 1'b0, frac});
            product_full = difference_ext * fraction_ext;
            base = $signed({56'd0, p0}) <<< COORD_F;
            total = base + product_full;
            horizontal_qf = total[H_W-1:0];
        end
    endfunction

    function signed [NUM_W-1:0] vertical_numerator_qf;
        input [H_W-1:0] h0;
        input [H_W-1:0] h1;
        input [COORD_F-1:0] frac;
        reg signed [H_W:0] difference;
        reg signed [63:0] difference_ext;
        reg signed [63:0] fraction_ext;
        reg signed [63:0] product_full;
        reg signed [63:0] base;
        reg signed [63:0] total;
        begin
            difference = $signed({1'b0, h1}) - $signed({1'b0, h0});
            difference_ext = {{(63-H_W){difference[H_W]}}, difference};
            fraction_ext = $signed({{(63-COORD_F){1'b0}}, 1'b0, frac});
            product_full = difference_ext * fraction_ext;
            base = $signed({{(64-H_W){1'b0}}, h0}) <<< COORD_F;
            total = base + product_full;
            vertical_numerator_qf = total[NUM_W-1:0];
        end
    endfunction

    function [7:0] round_channel_qf;
        input signed [NUM_W-1:0] numerator;
        reg [63:0] rounded_sum;
        reg [63:0] channel_value;
        begin
            if (numerator[NUM_W-1]) begin
                round_channel_qf = 8'd0;
            end else begin
                rounded_sum = numerator + (64'd1 << (2 * COORD_F - 1));
                channel_value = rounded_sum >> (2 * COORD_F);
                if (channel_value > 64'd255)
                    round_channel_qf = 8'd255;
                else
                    round_channel_qf = channel_value[7:0];
            end
        end
    endfunction

    assign stage_ready[3] = (~valid_pipe[3]) | bilin_out_ready;
    assign stage_ready[2] = (~valid_pipe[2]) | stage_ready[3];
    assign stage_ready[1] = (~valid_pipe[1]) | stage_ready[2];
    assign stage_ready[0] = (~valid_pipe[0]) | stage_ready[1];
    assign bilin_ready = stage_ready[0];
    assign bilin_out_valid = valid_pipe[3];

    always @(posedge bilin_clk or negedge bilin_rst_n) begin
        if (!bilin_rst_n) begin
            valid_pipe <= 4'b0;
            pixel_rgb <= 24'd0;
            bilin_out_frame_id <= 32'd0;
            bilin_out_cfg_version <= 16'd0;
            bilin_out_pixel_tag <= 32'd0;
        end else begin
            if (stage_ready[0]) begin
                valid_pipe[0] <= bilin_valid;
                if (bilin_valid) begin
                    p00_pipe[0] <= P00_rgb;
                    p10_pipe[0] <= P10_rgb;
                    p01_pipe[0] <= P01_rgb;
                    p11_pipe[0] <= P11_rgb;
                    dx_pipe[0] <= dx;
                    dy_pipe[0] <= dy;
                    frame_id_pipe[0] <= bilin_frame_id;
                    cfg_version_pipe[0] <= bilin_cfg_version;
                    pixel_tag_pipe[0] <= bilin_pixel_tag;
                end
            end

            if (stage_ready[1]) begin
                valid_pipe[1] <= valid_pipe[0];
                if (valid_pipe[0]) begin
                    h0_r_pipe[1] <= horizontal_qf(
                        p00_pipe[0][23:16], p10_pipe[0][23:16], dx_pipe[0]
                    );
                    h0_g_pipe[1] <= horizontal_qf(
                        p00_pipe[0][15:8], p10_pipe[0][15:8], dx_pipe[0]
                    );
                    h0_b_pipe[1] <= horizontal_qf(
                        p00_pipe[0][7:0], p10_pipe[0][7:0], dx_pipe[0]
                    );
                    h1_r_pipe[1] <= horizontal_qf(
                        p01_pipe[0][23:16], p11_pipe[0][23:16], dx_pipe[0]
                    );
                    h1_g_pipe[1] <= horizontal_qf(
                        p01_pipe[0][15:8], p11_pipe[0][15:8], dx_pipe[0]
                    );
                    h1_b_pipe[1] <= horizontal_qf(
                        p01_pipe[0][7:0], p11_pipe[0][7:0], dx_pipe[0]
                    );
                    dy_pipe[1] <= dy_pipe[0];
                    frame_id_pipe[1] <= frame_id_pipe[0];
                    cfg_version_pipe[1] <= cfg_version_pipe[0];
                    pixel_tag_pipe[1] <= pixel_tag_pipe[0];
                end
            end

            if (stage_ready[2]) begin
                valid_pipe[2] <= valid_pipe[1];
                if (valid_pipe[1]) begin
                    numerator_r_pipe[2] <= vertical_numerator_qf(
                        h0_r_pipe[1], h1_r_pipe[1], dy_pipe[1]
                    );
                    numerator_g_pipe[2] <= vertical_numerator_qf(
                        h0_g_pipe[1], h1_g_pipe[1], dy_pipe[1]
                    );
                    numerator_b_pipe[2] <= vertical_numerator_qf(
                        h0_b_pipe[1], h1_b_pipe[1], dy_pipe[1]
                    );
                    frame_id_pipe[2] <= frame_id_pipe[1];
                    cfg_version_pipe[2] <= cfg_version_pipe[1];
                    pixel_tag_pipe[2] <= pixel_tag_pipe[1];
                end
            end

            if (stage_ready[3]) begin
                valid_pipe[3] <= valid_pipe[2];
                if (valid_pipe[2]) begin
                    pixel_rgb <= {
                        round_channel_qf(numerator_r_pipe[2]),
                        round_channel_qf(numerator_g_pipe[2]),
                        round_channel_qf(numerator_b_pipe[2])
                    };
                    bilin_out_frame_id <= frame_id_pipe[2];
                    bilin_out_cfg_version <= cfg_version_pipe[2];
                    bilin_out_pixel_tag <= pixel_tag_pipe[2];
                end
            end
        end
    end

endmodule
