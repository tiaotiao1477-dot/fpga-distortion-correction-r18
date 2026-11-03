// Raster RGB888 Sobel with replicate image borders and masked fill pixels.
// Two grayscale/mask rows, one center RGB/tag row. All progress is handshaken.
module sobel_rgb_stream #(parameter integer MAX_WIDTH=1920) (
    input clk, input rst_n,
    input frame_start_valid, output frame_start_ready,
    input [15:0] frame_width, input [15:0] frame_height,
    input [31:0] frame_id, input [15:0] cfg_version,
    input [10:0] edge_threshold, input [1:0] edge_mode,
    input in_valid, output in_ready,
    input [23:0] in_rgb, input in_sample_valid, input [31:0] in_pixel_tag,
    output reg out_valid, input out_ready,
    output reg [23:0] out_rgb, output reg [10:0] magnitude,
    output reg signed [11:0] gx, output reg signed [11:0] gy,
    output reg edge_detected, output reg window_valid, output reg sample_valid,
    output reg [15:0] out_x, output reg [15:0] out_y, output reg out_last,
    output reg [31:0] out_frame_id, output reg [15:0] out_cfg_version,
    output reg [31:0] out_pixel_tag, output idle
);
    localparam IDLE=0, INPUT_ROW=1, RIGHT=2, BOTTOM=3;
    reg [1:0] state;
    reg [15:0] width,height,x,y;
    reg [31:0] current_frame;
    reg [15:0] current_version;
    reg [10:0] threshold;
    reg [1:0] mode;
    reg [7:0] row1_gray [0:MAX_WIDTH-1], row2_gray [0:MAX_WIDTH-1];
    reg row1_valid [0:MAX_WIDTH-1], row2_valid [0:MAX_WIDTH-1];
    reg [23:0] row1_rgb [0:MAX_WIDTH-1];
    reg [31:0] row1_tag [0:MAX_WIDTH-1];
    reg [7:0] t0,t1,m0,m1,b0,b1;
    reg tv0,tv1,mv0,mv1,bv0,bv1;
    reg [23:0] center_rgb;
    reg [31:0] center_tag;
    wire slot_ready=!out_valid || out_ready;
    assign idle=(state==IDLE) && !out_valid;
    assign frame_start_ready=rst_n && idle && frame_width>0 && frame_width<=MAX_WIDTH && frame_height>0;
    assign in_ready=rst_n && state==INPUT_ROW && slot_ready;
    wire [15:0] read_x=(width==0) ? 16'd0 : ((x<width) ? x : width-16'd1);
    function [7:0] gray;
        input [23:0] rgb;
        reg [31:0] total;
        begin
            total={24'd0,rgb[23:16]}*32'd77 + {24'd0,rgb[15:8]}*32'd150 +
                  {24'd0,rgb[7:0]}*32'd29 + 32'd128;
            gray=total[15:8];
        end
    endfunction
    wire [7:0] ct=(state==RIGHT) ? t1 :
        (((y==1) || (state==BOTTOM && height==1)) ? row1_gray[read_x] : row2_gray[read_x]);
    wire [7:0] cm=(state==RIGHT) ? m1 : row1_gray[read_x];
    wire [7:0] cb=(state==RIGHT) ? b1 : ((state==BOTTOM) ? row1_gray[read_x] : gray(in_rgb));
    wire ctv=(state==RIGHT) ? tv1 :
        (((y==1) || (state==BOTTOM && height==1)) ? row1_valid[read_x] : row2_valid[read_x]);
    wire cmv=(state==RIGHT) ? mv1 : row1_valid[read_x];
    wire cbv=(state==RIGHT) ? bv1 : ((state==BOTTOM) ? row1_valid[read_x] : in_sample_valid);
    wire [23:0] current_mid_rgb=(state==RIGHT) ? center_rgb : row1_rgb[read_x];
    wire [31:0] current_mid_tag=(state==RIGHT) ? center_tag : row1_tag[read_x];
    function signed [11:0] derivative;
        input [7:0] n0,n1,n2,p0,p1,p2;
        reg signed [31:0] total;
        begin
            total=$signed({24'd0,p0}) + ($signed({24'd0,p1}) <<< 1) + $signed({24'd0,p2})
                - $signed({24'd0,n0}) - ($signed({24'd0,n1}) <<< 1) - $signed({24'd0,n2});
            derivative=total[11:0];
        end
    endfunction
    wire signed [11:0] gx_calc=derivative(t0,m0,b0,ct,cm,cb);
    wire signed [11:0] gy_calc=derivative(t0,t1,ct,b0,b1,cb);
    wire [11:0] abs_gx=gx_calc[11] ? -gx_calc : gx_calc;
    wire [11:0] abs_gy=gy_calc[11] ? -gy_calc : gy_calc;
    wire [12:0] mag_calc={1'b0,abs_gx}+{1'b0,abs_gy};
    wire mask_calc=tv0 && tv1 && ctv && mv0 && mv1 && cmv && bv0 && bv1 && cbv;
    wire detected=mask_calc && (mag_calc!=0) && (mag_calc>={2'd0,threshold});
    wire [12:0] strength_wide=(mag_calc+13'd2) >> 2;
    wire [7:0] strength=mask_calc ? ((strength_wide>255) ? 8'd255 : strength_wide[7:0]) : 8'd0;
    wire advance=slot_ready && ((state==RIGHT)||(state==BOTTOM)||(state==INPUT_ROW && in_valid));
    wire emit_pixel=(y!=0) && ((state==RIGHT)||(x!=0));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; width<=0; height<=0; x<=0; y<=0;
            out_valid<=0; out_rgb<=0; magnitude<=0; gx<=0; gy<=0;
            edge_detected<=0; window_valid<=0; sample_valid<=0;
            out_x<=0; out_y<=0; out_last<=0;
            out_frame_id<=0; out_cfg_version<=0; out_pixel_tag<=0;
        end else begin
            if (slot_ready) out_valid<=0;
            if (frame_start_valid && frame_start_ready) begin
                width<=frame_width; height<=frame_height; current_frame<=frame_id;
                current_version<=cfg_version; threshold<=edge_threshold; mode<=edge_mode;
                x<=0; y<=0; state<=INPUT_ROW;
            end
            if (advance && state!=IDLE) begin
                if (state==INPUT_ROW) begin
                    row2_gray[x]<=row1_gray[x]; row2_valid[x]<=row1_valid[x];
                    row1_gray[x]<=gray(in_rgb); row1_valid[x]<=in_sample_valid;
                    row1_rgb[x]<=in_rgb; row1_tag[x]<=in_pixel_tag;
                end
                if (emit_pixel) begin
                    out_valid<=1;
                    gx<=mask_calc ? gx_calc : 12'sd0; gy<=mask_calc ? gy_calc : 12'sd0;
                    magnitude<=mask_calc ? mag_calc[10:0] : 11'd0;
                    edge_detected<=detected; window_valid<=mask_calc; sample_valid<=mv1;
                    out_x<=(state==RIGHT) ? width-16'd1 : x-16'd1;
                    out_y<=y-16'd1;
                    out_last<=(y==height) && (state==RIGHT);
                    out_frame_id<=current_frame; out_cfg_version<=current_version; out_pixel_tag<=center_tag;
                    case (mode)
                        2'd0: out_rgb<=center_rgb;
                        2'd1: out_rgb<={strength,strength,strength};
                        2'd2: out_rgb<=detected ? 24'hffffff : 24'd0;
                        2'd3: out_rgb<=detected ? 24'hff0000 : center_rgb;
                    endcase
                end
                if (state==RIGHT) begin
                    x<=0;
                    if (y==height) state<=IDLE;
                    else if (y==height-16'd1) begin state<=BOTTOM; y<=height; end
                    else begin state<=INPUT_ROW; y<=y+16'd1; end
                end else begin
                    if (x==0) begin
                        t0<=ct; t1<=ct; m0<=cm; m1<=cm; b0<=cb; b1<=cb;
                        tv0<=ctv; tv1<=ctv; mv0<=cmv; mv1<=cmv; bv0<=cbv; bv1<=cbv;
                    end else begin
                        t0<=t1; t1<=ct; m0<=m1; m1<=cm; b0<=b1; b1<=cb;
                        tv0<=tv1; tv1<=ctv; mv0<=mv1; mv1<=cmv; bv0<=bv1; bv1<=cbv;
                    end
                    center_rgb<=current_mid_rgb; center_tag<=current_mid_tag;
                    x<=x+16'd1;
                    if (x==width-16'd1) state<=RIGHT;
                end
            end
        end
    end
endmodule
