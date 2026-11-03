// Raster RGB888 Sobel with replicate image borders and masked fill pixels.
// Two grayscale/mask rows, one center RGB/tag row. All progress is handshaken.
module sobel_rgb_stream_pipe_v04 #(parameter integer MAX_WIDTH=1920) (
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
    // Producer cursor advances on RAM-read issue. Consumer cursor travels with
    // the registered RAM result; both stages may advance on the same clock.
    reg [1:0] stage_state;
    reg [15:0] stage_x,stage_y;
    reg [1:0] state;
    reg [15:0] width,height,x,y;
    reg [31:0] current_frame;
    reg [15:0] current_version;
    reg [10:0] threshold;
    reg [1:0] mode;
    // Synchronous read rows; separate no-reset memory process infers block RAM.
    reg [64:0] row1_mem [0:MAX_WIDTH-1];
    reg [8:0] row2_mem [0:MAX_WIDTH-1];
    reg [64:0] row1_read;
    reg [8:0] row2_read;
    reg [23:0] accepted_rgb;
    reg accepted_valid,loaded,calc_valid,calc_mask,calc_sample,calc_last;
    reg [31:0] accepted_tag,calc_tag;
    reg signed [11:0] calc_gx,calc_gy;
    reg [23:0] calc_rgb;
    reg [15:0] calc_x,calc_y;
    reg [7:0] t0,t1,m0,m1,b0,b1;
    reg tv0,tv1,mv0,mv1,bv0,bv1;
    reg [23:0] center_rgb;
    reg [31:0] center_tag;
    wire output_ready=!out_valid || out_ready;
    wire slot_ready=!calc_valid || output_ready;
    assign idle=(state==IDLE) && !out_valid && !calc_valid && !loaded;
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
    wire [7:0] ct=(stage_state==RIGHT) ? t1 :
        (((stage_y==1) || (stage_state==BOTTOM && height==1)) ? row1_read[8:1] : row2_read[8:1]);
    wire [7:0] cm=(stage_state==RIGHT) ? m1 : row1_read[8:1];
    wire [7:0] cb=(stage_state==RIGHT) ? b1 : ((stage_state==BOTTOM) ? row1_read[8:1] : gray(accepted_rgb));
    wire ctv=(stage_state==RIGHT) ? tv1 :
        (((stage_y==1) || (stage_state==BOTTOM && height==1)) ? row1_read[0] : row2_read[0]);
    wire cmv=(stage_state==RIGHT) ? mv1 : row1_read[0];
    wire cbv=(stage_state==RIGHT) ? bv1 : ((stage_state==BOTTOM) ? row1_read[0] : accepted_valid);
    wire [23:0] current_mid_rgb=(stage_state==RIGHT) ? center_rgb : row1_read[32:9];
    wire [31:0] current_mid_tag=(stage_state==RIGHT) ? center_tag : row1_read[64:33];
    function signed [11:0] derivative;
        input [7:0] n0,n1,n2,p0,p1,p2;
        reg signed [8:0] difference0,difference1,difference2;
        reg signed [9:0] outer_sum,middle_double;
        reg signed [11:0] total;
        begin
            difference0=$signed({1'b0,p0})-$signed({1'b0,n0});
            difference1=$signed({1'b0,p1})-$signed({1'b0,n1});
            difference2=$signed({1'b0,p2})-$signed({1'b0,n2});
            outer_sum=$signed({difference0[8],difference0}) + $signed({difference2[8],difference2});
            middle_double=$signed({difference1[8],difference1}) <<< 1;
            total=$signed({{2{outer_sum[9]}},outer_sum}) + $signed({{2{middle_double[9]}},middle_double});
            derivative=total;
        end
    endfunction
    wire signed [11:0] gx_calc=derivative(t0,m0,b0,ct,cm,cb);
    wire signed [11:0] gy_calc=derivative(t0,t1,ct,b0,b1,cb);
    wire [11:0] abs_gx=calc_gx[11] ? -calc_gx : calc_gx;
    wire [11:0] abs_gy=calc_gy[11] ? -calc_gy : calc_gy;
    wire [12:0] mag_calc={1'b0,abs_gx}+{1'b0,abs_gy};
    wire mask_calc=tv0 && tv1 && ctv && mv0 && mv1 && cmv && bv0 && bv1 && cbv;
    wire detected=calc_mask && (mag_calc!=0) && (mag_calc>={2'd0,threshold});
    wire [12:0] strength_wide=(mag_calc+13'd2) >> 2;
    wire [7:0] strength=calc_mask ? ((strength_wide>255) ? 8'd255 : strength_wide[7:0]) : 8'd0;
    wire prepare=rst_n && slot_ready && ((state==RIGHT)||(state==BOTTOM)||(state==INPUT_ROW && in_valid));
    wire advance=rst_n && slot_ready && loaded;
    always @(posedge clk)begin
        if(prepare)begin
            row1_read<=row1_mem[read_x];row2_read<=row2_mem[read_x];
            stage_state<=state;stage_x<=x;stage_y<=y;
            accepted_rgb<=in_rgb;accepted_valid<=in_sample_valid;accepted_tag<=in_pixel_tag;
        end
        if(advance&&stage_state==INPUT_ROW)begin
            row2_mem[stage_x]<={row1_read[8:1],row1_read[0]};
            row1_mem[stage_x]<={accepted_tag,accepted_rgb,gray(accepted_rgb),accepted_valid};
        end
    end
    wire emit_pixel=(stage_y!=0) && ((stage_state==RIGHT)||(stage_x!=0));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; loaded<=0;calc_valid<=0; width<=0; height<=0; x<=0; y<=0;
            out_valid<=0; out_rgb<=0; magnitude<=0; gx<=0; gy<=0;
            edge_detected<=0; window_valid<=0; sample_valid<=0;
            out_x<=0; out_y<=0; out_last<=0;
            out_frame_id<=0; out_cfg_version<=0; out_pixel_tag<=0;
        end else begin
            if (slot_ready) loaded<=prepare;
            if (slot_ready) calc_valid<=0;
            if (output_ready)begin
                out_valid<=calc_valid;
                if(calc_valid)begin
                    gx<=calc_mask?calc_gx:12'sd0;gy<=calc_mask?calc_gy:12'sd0;
                    magnitude<=calc_mask?mag_calc[10:0]:11'd0;
                    edge_detected<=detected;window_valid<=calc_mask;sample_valid<=calc_sample;
                    out_x<=calc_x;out_y<=calc_y;out_last<=calc_last;
                    out_frame_id<=current_frame;out_cfg_version<=current_version;out_pixel_tag<=calc_tag;
                    case(mode)
                        2'd0:out_rgb<=calc_rgb;
                        2'd1:out_rgb<={strength,strength,strength};
                        2'd2:out_rgb<=detected?24'hffffff:24'd0;
                        2'd3:out_rgb<=detected?24'hff0000:calc_rgb;
                    endcase
                end
            end
            if (frame_start_valid && frame_start_ready) begin
                width<=frame_width; height<=frame_height; current_frame<=frame_id;
                current_version<=cfg_version; threshold<=edge_threshold; mode<=edge_mode;
                x<=0; y<=0; state<=INPUT_ROW;
            end
            if (advance && stage_state!=IDLE) begin
                if (emit_pixel) begin
                    calc_valid<=1;calc_gx<=gx_calc;calc_gy<=gy_calc;
                    calc_mask<=mask_calc;calc_sample<=mv1;calc_rgb<=center_rgb;calc_tag<=center_tag;
                    calc_x<=(stage_state==RIGHT)?width-16'd1:stage_x-16'd1;calc_y<=stage_y-16'd1;
                    calc_last<=(stage_y==height)&&(stage_state==RIGHT);
                end
                if(stage_state!=RIGHT)begin
                    if (stage_x==0) begin
                        t0<=ct; t1<=ct; m0<=cm; m1<=cm; b0<=cb; b1<=cb;
                        tv0<=ctv; tv1<=ctv; mv0<=cmv; mv1<=cmv; bv0<=cbv; bv1<=cbv;
                    end else begin
                        t0<=t1; t1<=ct; m0<=m1; m1<=cm; b0<=b1; b1<=cb;
                        tv0<=tv1; tv1<=ctv; mv0<=mv1; mv1<=cmv; bv0<=bv1; bv1<=cbv;
                    end
                    center_rgb<=current_mid_rgb; center_tag<=current_mid_tag;
                end
            end
            if(prepare)begin
                if(state==RIGHT)begin
                    x<=0;
                    if(y==height)state<=IDLE;
                    else if(y==height-16'd1)begin state<=BOTTOM;y<=height;end
                    else begin state<=INPUT_ROW;y<=y+16'd1;end
                end else begin
                    x<=x+16'd1;
                    if(x==width-16'd1)state<=RIGHT;
                end
            end
        end
    end
endmodule
