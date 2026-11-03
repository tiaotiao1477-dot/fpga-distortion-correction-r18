// Four-neighbor descriptor -> stable DDR burst cache, no second interpolation.
module v04_cached_fetch_adapter #(
    parameter integer COORD_W=26,COORD_F=10,
    parameter [7:0] REQ_ID=8'hA6
)(
    input clk,rst_n,
    input [63:0] source_base,
    input [31:0] source_stride,
    input [15:0] source_width,source_height,
    input fetch_req_valid,output fetch_req_ready,
    input [15:0] fetch_x0,fetch_x1,fetch_y0,fetch_y1,
    input [31:0] fetch_frame_id,fetch_pixel_tag,
    input [15:0] fetch_cfg_version,
    output fetch_res_valid,input fetch_res_ready,
    output [23:0] P00_rgb,P10_rgb,P01_rgb,P11_rgb,
    output [31:0] fetch_res_frame_id,fetch_res_pixel_tag,
    output [15:0] fetch_res_cfg_version,
    output fetch_error,output reg descriptor_error,
    output fetch_rd_req_valid,input fetch_rd_req_ready,
    output [63:0] fetch_rd_req_addr,output [31:0] fetch_rd_req_len,
    output [7:0] fetch_rd_req_id,
    input fetch_rd_rsp_valid,output fetch_rd_rsp_ready,
    input [255:0] fetch_rd_rsp_data,input [7:0] fetch_rd_rsp_id,
    input fetch_rd_rsp_last,fetch_rd_rsp_error
);
    wire signed [COORD_W-1:0] sx=$signed({1'b0,fetch_x0})<<<COORD_F;
    wire signed [COORD_W-1:0] sy=$signed({1'b0,fetch_y0})<<<COORD_F;
    wire neighbors_ok=fetch_x0<source_width&&fetch_y0<source_height&&
        fetch_x1==((fetch_x0<source_width-1)?fetch_x0+16'd1:fetch_x0)&&
        fetch_y1==((fetch_y0<source_height-1)?fetch_y0+16'd1:fetch_y0);
    always @(posedge clk or negedge rst_n)
        if(!rst_n)descriptor_error<=0;
        else if(fetch_req_valid&&fetch_req_ready&&!neighbors_ok)descriptor_error<=1;
    pixel_fetch4_cached #(.COORD_W(COORD_W),.COORD_F(COORD_F),.REQ_ID(REQ_ID)) cache(
        .clk(clk),.rst_n(rst_n),.in_valid(fetch_req_valid),.in_ready(fetch_req_ready),
        .src_x_fixed(sx),.src_y_fixed(sy),.source_width(source_width),.source_height(source_height),
        .source_base(source_base),.source_stride(source_stride),.frame_id(fetch_frame_id),
        .cfg_version(fetch_cfg_version),.pixel_tag(fetch_pixel_tag),.border_clamp(1'b1),
        .fetch_rd_req_valid(fetch_rd_req_valid),.fetch_rd_req_ready(fetch_rd_req_ready),
        .fetch_rd_req_addr(fetch_rd_req_addr),.fetch_rd_req_len(fetch_rd_req_len),.fetch_rd_req_id(fetch_rd_req_id),
        .fetch_rd_rsp_valid(fetch_rd_rsp_valid),.fetch_rd_rsp_ready(fetch_rd_rsp_ready),
        .fetch_rd_rsp_data(fetch_rd_rsp_data),.fetch_rd_rsp_id(fetch_rd_rsp_id),
        .fetch_rd_rsp_last(fetch_rd_rsp_last),.fetch_rd_rsp_error(fetch_rd_rsp_error),
        .out_valid(fetch_res_valid),.out_ready(fetch_res_ready),.P00_rgb(P00_rgb),.P10_rgb(P10_rgb),
        .P01_rgb(P01_rgb),.P11_rgb(P11_rgb),.dx(),.dy(),.out_frame_id(fetch_res_frame_id),
        .out_cfg_version(fetch_res_cfg_version),.out_pixel_tag(fetch_res_pixel_tag),
        .out_error(fetch_error),.out_border());
endmodule
