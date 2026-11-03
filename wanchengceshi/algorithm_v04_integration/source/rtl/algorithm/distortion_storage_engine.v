`timescale 1ns/1ps
// Frame adapter around the actual delivered coordinate and bilinear modules.
// Native storage contracts preserved: descriptors/Fetch DDR/result pixels.
// Single-outstanding Fetch4 correctness mode; throughput must be measured.
// Read failures set a monotonic independent frame-error status. The result
// writer must enable the explicit error input to abort this frame's result slot.
module distortion_storage_engine #(
    parameter integer SLOT_W=2,
    parameter integer COORD_W=26,
    parameter integer COORD_F=10,
    parameter integer BORDER_CLAMP=1,
    parameter integer USE_BURST_CACHE=1,
    parameter integer USE_PIPELINED_COORD=1,
    parameter [7:0] REQ_ID=8'hA6
)(
    input wire clk,rst_n,
    input wire profile_valid,
    input wire signed [31:0] cfg_fx_q16,cfg_fy_q16,cfg_cx_q16,cfg_cy_q16,
    input wire signed [31:0] cfg_inv_fx_q24,cfg_inv_fy_q24,
    input wire signed [31:0] cfg_k1_q24,cfg_k2_q24,cfg_p1_q24,cfg_p2_q24,
    output wire source_desc_claim_valid,
    input wire source_desc_claim_ready,
    input wire [SLOT_W-1:0] source_desc_claim_slot,
    input wire [63:0] source_desc_base_addr,
    input wire [31:0] source_desc_frame_id,source_desc_source_frame_id,
    input wire [15:0] source_desc_cfg_version,source_desc_width,source_desc_height,
    input wire [31:0] source_desc_stride_bytes,
    output wire source_desc_release_valid,
    output wire [SLOT_W-1:0] source_desc_release_slot,
    input wire source_desc_release_ready,
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
    output wire result_px_valid,
    input wire result_px_ready,
    output wire [23:0] result_px_rgb,
    output wire [15:0] result_px_x,result_px_y,
    output wire result_px_sof,result_px_eol,
    output wire [31:0] result_px_source_frame_id,result_px_pixel_tag,
    output wire [15:0] result_px_cfg_version,
    output wire result_frame_error,
    output reg [31:0] error_count,
    output wire [3:0] debug_state,
    output wire [10:0] debug_x,debug_y
);
    localparam CLAIM=0,ACTIVE=1,RELEASE=2;
    reg [1:0] state;
    reg [SLOT_W-1:0] active_slot;
    reg [63:0] base;
    reg [31:0] stride,frame_id,issue_tag,retire_tag;
    reg [15:0] width,height,cfg_version,issue_x,issue_y,retire_x,retire_y;
    reg issue_done;
    reg frame_fault;
    reg signed [31:0] fx,fy,cx,cy,inv_fx,inv_fy,k1,k2,p1,p2;
    // Valid bits drain between frames. Use the coherent DDR-domain reset;
    // decoding ACTIVE onto DSP asynchronous reset caused recovery violations.
    wire frame_rst_n=rst_n;
    wire coord_req_ready,coord_valid,coord_ready,coord_empty;
    wire signed [COORD_W-1:0] sx,sy;
    wire [1:0] coord_border;
    wire [31:0] coord_frame,coord_tag;
    wire [15:0] coord_cfg;
    localparam integer COORD_BUNDLE_W=2*COORD_W+82;
    wire queued_valid,queued_ready,queued_empty;
    wire signed [COORD_W-1:0] queued_sx,queued_sy;
    wire [1:0] queued_border;
    wire [31:0] queued_frame,queued_tag;
    wire [15:0] queued_cfg;
    coord_result_queue #(.WIDTH(COORD_BUNDLE_W)) u_coord_queue(
        .clk(clk),.rst_n(frame_rst_n),.in_valid(coord_valid),.in_ready(coord_ready),
        .in_data({sx,sy,coord_border,coord_frame,coord_cfg,coord_tag}),
        .out_valid(queued_valid),.out_ready(queued_ready),
        .out_data({queued_sx,queued_sy,queued_border,queued_frame,queued_cfg,queued_tag}),
        .empty(queued_empty));
    wire fetch_valid,fetch_ready,fetch_error,fetch_border;
    wire [23:0] P00,P10,P01,P11;
    wire [COORD_F-1:0] dx,dy;
    wire [31:0] fetch_frame,fetch_tag;
    wire [15:0] fetch_cfg;
    wire bilin_valid;
    wire [31:0] bilin_frame,bilin_tag;
    wire [15:0] bilin_cfg;
    wire [64:0] source_end={1'b0,source_desc_base_addr}+
        ({49'd0,source_desc_height}*{33'd0,source_desc_stride_bytes});
    wire descriptor_ok=source_desc_width!=0&&source_desc_height!=0&&
        source_desc_width<=16'd32768&&source_desc_height<=16'd32768&&
        source_desc_base_addr[4:0]==0&&source_desc_stride_bytes[4:0]==0&&
        source_desc_stride_bytes>={14'd0,source_desc_width,2'b00}&&!source_end[64]&&
        cfg_fx_q16>0&&cfg_fy_q16>0&&cfg_inv_fx_q24>0&&cfg_inv_fy_q24>0;
    assign source_desc_claim_valid=rst_n&&state==CLAIM&&profile_valid;
    assign source_desc_release_valid=rst_n&&state==RELEASE&&coord_empty&&queued_empty&&
        !fetch_valid&&!bilin_valid&&!fetch_rd_req_valid;
    assign source_desc_release_slot=active_slot;
    assign result_px_valid=rst_n&&state==ACTIVE&&bilin_valid;
    assign result_px_x=retire_x;assign result_px_y=retire_y;
    assign result_px_sof=retire_tag==0;
    assign result_px_eol=retire_x==width-1'b1;
    assign result_px_source_frame_id=bilin_frame;
    assign result_px_cfg_version=bilin_cfg;
    assign result_px_pixel_tag=bilin_tag;
    wire metadata_bad=bilin_frame!=frame_id||bilin_cfg!=cfg_version||bilin_tag!=retire_tag;
    assign result_frame_error=frame_fault||(bilin_valid&&metadata_bad);
    assign debug_state={2'd0,state};assign debug_x=retire_x[10:0];assign debug_y=retire_y[10:0];
    generate if(USE_PIPELINED_COORD!=0)begin:g_pipelined_coord
    basic_distortion_coord_pipelined #(.COORD_W(COORD_W),.COORD_F(COORD_F)) u_coord(
        .coord_clk(clk),.coord_rst_n(frame_rst_n),
        .coord_req_valid(state==ACTIVE&&!issue_done),.coord_req_ready(coord_req_ready),
        .coord_req_x(issue_x),.coord_req_y(issue_y),.coord_req_frame_id(frame_id),
        .coord_req_cfg_version(cfg_version),.coord_req_pixel_tag(issue_tag),
        .cfg_active_source_width(width),.cfg_active_source_height(height),
        .cfg_active_fx_q16(fx),.cfg_active_fy_q16(fy),.cfg_active_cx_q16(cx),.cfg_active_cy_q16(cy),
        .cfg_active_inv_fx_q24(inv_fx),.cfg_active_inv_fy_q24(inv_fy),
        .cfg_active_k1_q24(k1),.cfg_active_k2_q24(k2),.cfg_active_p1_q24(p1),.cfg_active_p2_q24(p2),
        .coord_res_valid(coord_valid),.coord_res_ready(coord_ready),.src_x_fixed(sx),.src_y_fixed(sy),
        .border(coord_border),.coord_res_frame_id(coord_frame),.coord_res_cfg_version(coord_cfg),
        .coord_res_pixel_tag(coord_tag),.coord_pipe_empty(coord_empty));
    end else begin:g_original_coord
    basic_distortion_coord #(.COORD_W(COORD_W),.COORD_F(COORD_F)) u_coord(
        .coord_clk(clk),.coord_rst_n(frame_rst_n),
        .coord_req_valid(state==ACTIVE&&!issue_done),.coord_req_ready(coord_req_ready),
        .coord_req_x(issue_x),.coord_req_y(issue_y),.coord_req_frame_id(frame_id),
        .coord_req_cfg_version(cfg_version),.coord_req_pixel_tag(issue_tag),
        .cfg_active_source_width(width),.cfg_active_source_height(height),
        .cfg_active_fx_q16(fx),.cfg_active_fy_q16(fy),.cfg_active_cx_q16(cx),.cfg_active_cy_q16(cy),
        .cfg_active_inv_fx_q24(inv_fx),.cfg_active_inv_fy_q24(inv_fy),
        .cfg_active_k1_q24(k1),.cfg_active_k2_q24(k2),.cfg_active_p1_q24(p1),.cfg_active_p2_q24(p2),
        .coord_res_valid(coord_valid),.coord_res_ready(coord_ready),.src_x_fixed(sx),.src_y_fixed(sy),
        .border(coord_border),.coord_res_frame_id(coord_frame),.coord_res_cfg_version(coord_cfg),
        .coord_res_pixel_tag(coord_tag),.coord_pipe_empty(coord_empty));
    end endgenerate
    generate if(USE_BURST_CACHE!=0)begin:g_cached
    pixel_fetch4_cached #(.COORD_W(COORD_W),.COORD_F(COORD_F),.REQ_ID(REQ_ID)) u_fetch(
        .clk(clk),.rst_n(frame_rst_n),.in_valid(queued_valid),.in_ready(queued_ready),
        .src_x_fixed(queued_sx),.src_y_fixed(queued_sy),.source_width(width),.source_height(height),
        .source_base(base),.source_stride(stride),.frame_id(queued_frame),.cfg_version(queued_cfg),
        .pixel_tag(queued_tag),.border_clamp(BORDER_CLAMP!=0),
        .fetch_rd_req_valid(fetch_rd_req_valid),.fetch_rd_req_ready(fetch_rd_req_ready),
        .fetch_rd_req_addr(fetch_rd_req_addr),.fetch_rd_req_len(fetch_rd_req_len),.fetch_rd_req_id(fetch_rd_req_id),
        .fetch_rd_rsp_valid(fetch_rd_rsp_valid),.fetch_rd_rsp_ready(fetch_rd_rsp_ready),
        .fetch_rd_rsp_data(fetch_rd_rsp_data),.fetch_rd_rsp_id(fetch_rd_rsp_id),
        .fetch_rd_rsp_last(fetch_rd_rsp_last),.fetch_rd_rsp_error(fetch_rd_rsp_error),
        .out_valid(fetch_valid),.out_ready(fetch_ready),.P00_rgb(P00),.P10_rgb(P10),.P01_rgb(P01),.P11_rgb(P11),
        .dx(dx),.dy(dy),.out_frame_id(fetch_frame),.out_cfg_version(fetch_cfg),.out_pixel_tag(fetch_tag),
        .out_error(fetch_error),.out_border(fetch_border));
    end else begin:g_reference
    pixel_fetch4 #(.COORD_W(COORD_W),.COORD_F(COORD_F),.REQ_ID(REQ_ID)) u_fetch(
        .clk(clk),.rst_n(frame_rst_n),.in_valid(queued_valid),.in_ready(queued_ready),
        .src_x_fixed(queued_sx),.src_y_fixed(queued_sy),.source_width(width),.source_height(height),
        .source_base(base),.source_stride(stride),.frame_id(queued_frame),.cfg_version(queued_cfg),
        .pixel_tag(queued_tag),.border_clamp(BORDER_CLAMP!=0),
        .fetch_rd_req_valid(fetch_rd_req_valid),.fetch_rd_req_ready(fetch_rd_req_ready),
        .fetch_rd_req_addr(fetch_rd_req_addr),.fetch_rd_req_len(fetch_rd_req_len),.fetch_rd_req_id(fetch_rd_req_id),
        .fetch_rd_rsp_valid(fetch_rd_rsp_valid),.fetch_rd_rsp_ready(fetch_rd_rsp_ready),
        .fetch_rd_rsp_data(fetch_rd_rsp_data),.fetch_rd_rsp_id(fetch_rd_rsp_id),
        .fetch_rd_rsp_last(fetch_rd_rsp_last),.fetch_rd_rsp_error(fetch_rd_rsp_error),
        .out_valid(fetch_valid),.out_ready(fetch_ready),.P00_rgb(P00),.P10_rgb(P10),.P01_rgb(P01),.P11_rgb(P11),
        .dx(dx),.dy(dy),.out_frame_id(fetch_frame),.out_cfg_version(fetch_cfg),.out_pixel_tag(fetch_tag),
        .out_error(fetch_error),.out_border(fetch_border));
    end endgenerate
    bilinear_rgb #(.COORD_F(COORD_F)) u_bilinear(
        .bilin_clk(clk),.bilin_rst_n(frame_rst_n),.bilin_valid(fetch_valid),.bilin_ready(fetch_ready),
        .P00_rgb(P00),.P10_rgb(P10),.P01_rgb(P01),.P11_rgb(P11),.dx(dx),.dy(dy),
        .bilin_frame_id(fetch_frame),.bilin_cfg_version(fetch_cfg),.bilin_pixel_tag(fetch_tag),
        .bilin_out_valid(bilin_valid),.bilin_out_ready(state==ACTIVE&&result_px_ready),
        .pixel_rgb(result_px_rgb),.bilin_out_frame_id(bilin_frame),
        .bilin_out_cfg_version(bilin_cfg),.bilin_out_pixel_tag(bilin_tag));
    // Parameters are payload: load all ten on the frame claim. State and
    // coordinate validity reset below guarantee no use before that claim.
    // Asynchronous parameter clear was absorbed into DSP input registers,
    // reintroducing recovery paths despite the unreset coordinate datapath.
    always @(posedge clk)begin
        if(source_desc_claim_valid&&source_desc_claim_ready)begin
            stride<=source_desc_stride_bytes;width<=source_desc_width;height<=source_desc_height;
            fx<=cfg_fx_q16;fy<=cfg_fy_q16;cx<=cfg_cx_q16;cy<=cfg_cy_q16;
            inv_fx<=cfg_inv_fx_q24;inv_fy<=cfg_inv_fy_q24;
            k1<=cfg_k1_q24;k2<=cfg_k2_q24;p1<=cfg_p1_q24;p2<=cfg_p2_q24;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=CLAIM;active_slot<=0;base<=0;frame_id<=0;cfg_version<=0;
            issue_x<=0;issue_y<=0;issue_tag<=0;issue_done<=0;
            retire_x<=0;retire_y<=0;retire_tag<=0;error_count<=0;
            frame_fault<=0;
        end else begin
            case(state)
                CLAIM:if(source_desc_claim_valid&&source_desc_claim_ready)begin
                    active_slot<=source_desc_claim_slot;base<=source_desc_base_addr;
                    frame_id<=source_desc_source_frame_id;cfg_version<=source_desc_cfg_version;
                    issue_x<=0;issue_y<=0;issue_tag<=0;issue_done<=0;
                    retire_x<=0;retire_y<=0;retire_tag<=0;
                    frame_fault<=0;
                    if(descriptor_ok)state<=ACTIVE;
                    else begin error_count<=error_count+1'b1;state<=RELEASE;end
                end
                ACTIVE:begin
                    if(!issue_done&&coord_req_ready)begin
                        issue_tag<=issue_tag+1'b1;
                        if(issue_x==width-1'b1)begin
                            issue_x<=0;
                            if(issue_y==height-1'b1)issue_done<=1;
                            else issue_y<=issue_y+1'b1;
                        end else issue_x<=issue_x+1'b1;
                    end
                    if(fetch_valid&&fetch_ready&&fetch_error)begin
                        error_count<=error_count+1'b1;
                        frame_fault<=1;
                    end
                    if(result_px_valid&&result_px_ready)begin
                        if(metadata_bad)begin
                            error_count<=error_count+1'b1;
                            frame_fault<=1;
                        end
                        retire_tag<=retire_tag+1'b1;
                        if(retire_x==width-1'b1)begin
                            retire_x<=0;
                            if(retire_y==height-1'b1)state<=RELEASE;
                            else retire_y<=retire_y+1'b1;
                        end else retire_x<=retire_x+1'b1;
                    end
                end
                RELEASE:if(source_desc_release_valid&&source_desc_release_ready)state<=CLAIM;
                default:state<=CLAIM;
            endcase
        end
    end
endmodule

// Two entries isolate upstream readiness from downstream combinational
// backpressure. Payload is unreset; count/pointers mask and flush old bundles.
module coord_result_queue #(parameter integer WIDTH=134)(
    input wire clk,rst_n,
    input wire in_valid,
    output wire in_ready,
    input wire [WIDTH-1:0] in_data,
    output wire out_valid,
    input wire out_ready,
    output wire [WIDTH-1:0] out_data,
    output wire empty
);
    reg [WIDTH-1:0] payload[0:1];
    reg [1:0] count;
    reg write_ptr,read_ptr;
    assign in_ready=rst_n&&(count<2);
    assign out_valid=rst_n&&(count!=0);
    assign out_data=payload[read_ptr];
    assign empty=(count==0);
    wire push=in_valid&&in_ready;
    wire pop=out_valid&&out_ready;
    always @(posedge clk)begin
        if(push)payload[write_ptr]<=in_data;
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin count<=0;write_ptr<=0;read_ptr<=0;end
        else begin
            if(push)write_ptr<=~write_ptr;
            if(pop)read_ptr<=~read_ptr;
            case({push,pop})
                2'b10:count<=count+1'b1;
                2'b01:count<=count-1'b1;
                default:count<=count;
            endcase
        end
    end
endmodule
