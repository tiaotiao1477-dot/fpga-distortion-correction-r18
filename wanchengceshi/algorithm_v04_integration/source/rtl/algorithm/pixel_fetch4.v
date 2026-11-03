`timescale 1ns/1ps
// Correctness reference: one coordinate and one DDR beat outstanding.
// RGB888 in 32-bit words; byte addresses and 32-byte aligned single-beat reads.
// Neighbor order P00/P10/P01/P11. Metadata and output stay stable under stall.
// Border modes match the delivery reference; clamp coordinates before floor.
// Per-coordinate recent-beat reuse merges adjacent taps, never crosses frames.
module pixel_fetch4 #(
    parameter integer COORD_W=26,
    parameter integer COORD_F=10,
    parameter [7:0] REQ_ID=8'hA6
)(
    input wire clk,rst_n,
    input wire in_valid,
    output wire in_ready,
    input wire signed [COORD_W-1:0] src_x_fixed,src_y_fixed,
    input wire [15:0] source_width,source_height,
    input wire [63:0] source_base,
    input wire [31:0] source_stride,frame_id,pixel_tag,
    input wire [15:0] cfg_version,
    input wire border_clamp,
    output wire fetch_rd_req_valid,
    input wire fetch_rd_req_ready,
    output wire [63:0] fetch_rd_req_addr,
    output wire [31:0] fetch_rd_req_len,
    output wire [7:0] fetch_rd_req_id,
    input wire fetch_rd_rsp_valid,
    output wire fetch_rd_rsp_ready,
    input wire [255:0] fetch_rd_rsp_data,
    input wire [7:0] fetch_rd_rsp_id,
    input wire fetch_rd_rsp_last,fetch_rd_rsp_error,
    output wire out_valid,
    input wire out_ready,
    output wire [23:0] P00_rgb,P10_rgb,P01_rgb,P11_rgb,
    output reg [COORD_F-1:0] dx,dy,
    output reg [31:0] out_frame_id,out_pixel_tag,
    output reg [15:0] out_cfg_version,
    output reg out_error,out_border
);
    localparam IDLE=0,PREPARE=1,REQUEST=2,RESPONSE=3,OUTPUT=4,DRAIN=5;
    reg [2:0] state;
    reg [15:0] x0,x1,y0,y1;
    reg [63:0] active_base,request_addr;
    reg [31:0] active_stride;
    reg [1:0] tap;
    reg [2:0] lane;
    reg [23:0] pixels[0:3];
    reg recent_valid;
    reg [63:0] recent_addr;
    reg [255:0] recent_data;
    integer k;
    wire signed [63:0] input_x={{(64-COORD_W){src_x_fixed[COORD_W-1]}},src_x_fixed};
    wire signed [63:0] input_y={{(64-COORD_W){src_y_fixed[COORD_W-1]}},src_y_fixed};
    wire signed [63:0] max_x=($signed({48'd0,source_width})-64'sd1)<<<COORD_F;
    wire signed [63:0] max_y=($signed({48'd0,source_height})-64'sd1)<<<COORD_F;
    wire input_outside=input_x<0||input_y<0||input_x>max_x||input_y>max_y;
    wire signed [63:0] clipped_x=input_x<0?64'sd0:(input_x>max_x?max_x:input_x);
    wire signed [63:0] clipped_y=input_y<0?64'sd0:(input_y>max_y?max_y:input_y);
    wire [63:0] floor_x=clipped_x>>>COORD_F;
    wire [63:0] floor_y=clipped_y>>>COORD_F;
    wire descriptor_ok=source_width!=0&&source_height!=0&&source_base[4:0]==0&&
        source_stride[4:0]==0&&source_stride>=({14'd0,source_width,2'b00});
    wire [15:0] tap_x=tap[0]?x1:x0;
    wire [15:0] tap_y=tap[1]?y1:y0;
    wire [63:0] row_offset={48'd0,tap_y}*{32'd0,active_stride};
    wire [64:0] address_sum={1'b0,active_base}+{1'b0,row_offset}+{47'd0,tap_x,2'b00};
    wire [63:0] aligned_addr={address_sum[63:5],5'd0};
    wire response_bad=fetch_rd_rsp_id!=REQ_ID||fetch_rd_rsp_error||!fetch_rd_rsp_last;
    assign in_ready=rst_n&&state==IDLE;
    assign out_valid=rst_n&&state==OUTPUT;
    assign fetch_rd_req_valid=rst_n&&state==REQUEST;
    assign fetch_rd_req_addr=request_addr;
    assign fetch_rd_req_len=32'd1;
    assign fetch_rd_req_id=REQ_ID;
    assign fetch_rd_rsp_ready=rst_n&&(state==RESPONSE||state==DRAIN);
    assign P00_rgb=pixels[0]; assign P10_rgb=pixels[1];
    assign P01_rgb=pixels[2]; assign P11_rgb=pixels[3];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;tap<=0;lane<=0;
            active_base<=0;active_stride<=0;request_addr<=0;
            x0<=0;x1<=0;y0<=0;y1<=0;dx<=0;dy<=0;
            recent_valid<=0;recent_addr<=0;recent_data<=0;
            out_frame_id<=0;out_pixel_tag<=0;out_cfg_version<=0;out_error<=0;out_border<=0;
            for(k=0;k<4;k=k+1)pixels[k]<=0;
        end else case(state)
            IDLE:if(in_valid&&in_ready)begin
                out_frame_id<=frame_id;out_pixel_tag<=pixel_tag;out_cfg_version<=cfg_version;
                out_error<=!descriptor_ok;out_border<=input_outside;
                active_base<=source_base;active_stride<=source_stride;
                recent_valid<=0;tap<=0;
                for(k=0;k<4;k=k+1)pixels[k]<=0;
                if(!descriptor_ok||(!border_clamp&&input_outside))begin
                    dx<=0;dy<=0;state<=OUTPUT;
                end else begin
                    x0<=floor_x[15:0];y0<=floor_y[15:0];
                    x1<=floor_x+1<source_width?floor_x[15:0]+1'b1:floor_x[15:0];
                    y1<=floor_y+1<source_height?floor_y[15:0]+1'b1:floor_y[15:0];
                    dx<=clipped_x[COORD_F-1:0];dy<=clipped_y[COORD_F-1:0];state<=PREPARE;
                end
            end
            PREPARE:begin
                if(address_sum[64])begin
                    out_error<=1;for(k=0;k<4;k=k+1)pixels[k]<=0;state<=OUTPUT;
                end else if(recent_valid&&recent_addr==aligned_addr)begin
                    pixels[tap]<=recent_data[address_sum[4:2]*32+:24];
                    if(tap==3)state<=OUTPUT;else tap<=tap+1'b1;
                end else begin
                    request_addr<=aligned_addr;lane<=address_sum[4:2];state<=REQUEST;
                end
            end
            REQUEST:if(fetch_rd_req_valid&&fetch_rd_req_ready)state<=RESPONSE;
            RESPONSE:if(fetch_rd_rsp_valid&&fetch_rd_rsp_ready)begin
                if(response_bad)begin
                    out_error<=1;recent_valid<=0;
                    for(k=0;k<4;k=k+1)pixels[k]<=0;
                    state<=fetch_rd_rsp_last?OUTPUT:DRAIN;
                end else begin
                    pixels[tap]<=fetch_rd_rsp_data[lane*32+:24];
                    recent_valid<=1;recent_addr<=request_addr;recent_data<=fetch_rd_rsp_data;
                    if(tap==3)state<=OUTPUT;
                    else begin tap<=tap+1'b1;state<=PREPARE;end
                end
            end
            DRAIN:if(fetch_rd_rsp_valid&&fetch_rd_rsp_ready&&fetch_rd_rsp_last)state<=OUTPUT;
            OUTPUT:if(out_valid&&out_ready)state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
