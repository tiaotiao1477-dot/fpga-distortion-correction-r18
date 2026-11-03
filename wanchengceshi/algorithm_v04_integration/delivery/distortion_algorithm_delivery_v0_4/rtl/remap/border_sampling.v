// One elastic stage: safe Fetch4 indices/fractions, BLACK or CLAMP.
// geom_invalid is separate from border=2 (ordinary outside coordinates).
module border_sampling #(
    parameter integer COORD_W=26,
    parameter integer COORD_F=10
) (
    input clk, input rst_n,
    input in_valid, output in_ready,
    input signed [COORD_W-1:0] src_x, input signed [COORD_W-1:0] src_y,
    input [1:0] border, input geom_invalid, input clamp_mode,
    input [15:0] source_width, input [15:0] source_height,
    input [31:0] in_frame_id, input [15:0] in_cfg_version, input [31:0] in_pixel_tag,
    output reg out_valid, input out_ready,
    output reg [15:0] x0, output reg [15:0] x1,
    output reg [15:0] y0, output reg [15:0] y1,
    output reg [COORD_F-1:0] dx, output reg [COORD_F-1:0] dy,
    output reg sample_enable, output reg outside, output reg boundary,
    output reg [31:0] out_frame_id, output reg [15:0] out_cfg_version,
    output reg [31:0] out_pixel_tag, output pipe_empty
);
    wire signed [63:0] sx={{(64-COORD_W){src_x[COORD_W-1]}},src_x};
    wire signed [63:0] sy={{(64-COORD_W){src_y[COORD_W-1]}},src_y};
    wire signed [63:0] xmax=($signed({48'd0,source_width})-64'sd1) <<< COORD_F;
    wire signed [63:0] ymax=($signed({48'd0,source_height})-64'sd1) <<< COORD_F;
    wire invalid_dims=(source_width==0)||(source_height==0)||
        (source_width>32768)||(source_height>32768);
    wire is_outside=(border==2)||sx<0||sy<0||sx>xmax||sy>ymax;
    wire enabled=!geom_invalid && !invalid_dims && (clamp_mode || !is_outside);
    wire signed [63:0] cx=sx<0 ? 64'sd0 : (sx>xmax ? xmax : sx);
    wire signed [63:0] cy=sy<0 ? 64'sd0 : (sy>ymax ? ymax : sy);
    wire [63:0] ix=cx >> COORD_F;
    wire [63:0] iy=cy >> COORD_F;
    assign in_ready=rst_n && (!out_valid || out_ready);
    assign pipe_empty=!out_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid<=0; sample_enable<=0; outside<=0; boundary<=0;
            x0<=0; x1<=0; y0<=0; y1<=0; dx<=0; dy<=0;
            out_frame_id<=0; out_cfg_version<=0; out_pixel_tag<=0;
        end else if (in_ready) begin
            out_valid<=in_valid;
            if (in_valid) begin
                sample_enable<=enabled;
                outside<=is_outside || geom_invalid || invalid_dims;
                boundary<=enabled && (is_outside || ix>=source_width-1 || iy>=source_height-1);
                x0<=enabled ? ix[15:0] : 16'd0;
                x1<=enabled ? ((ix+64'd1>=source_width) ? source_width-16'd1 : ix[15:0]+16'd1) : 16'd0;
                y0<=enabled ? iy[15:0] : 16'd0;
                y1<=enabled ? ((iy+64'd1>=source_height) ? source_height-16'd1 : iy[15:0]+16'd1) : 16'd0;
                dx<=enabled ? cx[COORD_F-1:0] : {COORD_F{1'b0}};
                dy<=enabled ? cy[COORD_F-1:0] : {COORD_F{1'b0}};
                out_frame_id<=in_frame_id; out_cfg_version<=in_cfg_version; out_pixel_tag<=in_pixel_tag;
            end
        end
    end
endmodule
