`timescale 1ns/1ps
// Frame adapter around the actual delivered coordinate and bilinear modules.
// Native storage contracts preserved: descriptors/Fetch DDR/result pixels.
// Continuous coordinate queue feeds the native single-outstanding burst cache.
// Read failures set a monotonic independent frame-error status. The result
// writer must enable the explicit error input to abort this frame's result slot.
module distortion_storage_engine_v04 #(
    parameter integer FRAME_WIDTH=1280,FRAME_HEIGHT=720,AUTO_BOOT=1,MAX_WIDTH=1920,
    parameter integer SLOT_W=2,
    parameter integer COORD_W=26,
    parameter integer COORD_F=10,
    parameter integer BORDER_CLAMP=1,
    parameter integer USE_BURST_CACHE=1,
    parameter integer USE_PIPELINED_COORD=1,
    parameter [7:0] REQ_ID=8'hA6
)(
    input wire cfg_valid,output wire cfg_ready,input wire [735:0] cfg_payload,input wire [31:0] cfg_seq,
    output wire cfg_received,cfg_rejected,cfg_applied,
    output wire [31:0] cfg_received_seq,cfg_applied_seq,cfg_applied_frame_id,
    output wire [15:0] active_cfg_version,
    input wire result_writer_idle,
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

    localparam CLAIM=0,ACTIVE=1,DRAIN=2,RELEASE=3;
    reg [1:0] state;
    reg [SLOT_W-1:0] active_slot;
    reg [63:0] base;
    reg [31:0] stride,frame_id,issue_tag,retire_tag;
    reg [15:0] issue_x,issue_y;
    reg issue_done,frame_fault,boot_complete;
    wire pipe_start_ready,pipe_idle,coord_ready;
    wire [15:0] output_width,output_height,ox,oy;
    wire out_last,pipe_out_valid,pipe_error;
    wire [31:0] out_tag;
    wire fetch_req_valid,fetch_req_ready,fetch_res_valid,fetch_res_ready;
    wire [15:0] x0,x1,y0,y1,fetch_cfg,fetch_res_cfg;
    wire [31:0] fetch_frame,fetch_tag,fetch_res_frame,fetch_res_tag;
    wire [23:0] P00,P10,P01,P11;
    wire fetch_error,descriptor_error,pipe_cfg_ready;
    wire boot_valid=AUTO_BOOT!=0&&!boot_complete&&profile_valid;
    wire [735:0] boot_payload={
        32'd0,32'd200,(BORDER_CLAMP!=0?32'd1:32'd0),
        FRAME_HEIGHT,FRAME_WIDTH,
        32'd0,32'd65536,32'd0,32'd0,32'd0,32'd65536,
        cfg_p2_q24,cfg_p1_q24,cfg_k2_q24,cfg_k1_q24,
        cfg_inv_fy_q24,cfg_inv_fx_q24,cfg_cy_q16,cfg_cx_q16,cfg_fy_q16,cfg_fx_q16,
        FRAME_HEIGHT,FRAME_WIDTH};
    wire dimensions_ok=cfg_payload[0+:32]==FRAME_WIDTH&&cfg_payload[32+:32]==FRAME_HEIGHT&&
        cfg_payload[18*32+:32]==FRAME_WIDTH&&cfg_payload[19*32+:32]==FRAME_HEIGHT;
    // Reject size changes using the delivery's existing illegal-config path.
    wire [735:0] user_payload=dimensions_ok?cfg_payload:{cfg_payload[735:32],32'd0};
    assign cfg_ready=pipe_cfg_ready&&boot_complete;
    wire [64:0] source_end={1'b0,source_desc_base_addr}+
        ({49'd0,source_desc_height}*{33'd0,source_desc_stride_bytes});
    wire descriptor_ok=source_desc_width==FRAME_WIDTH&&source_desc_height==FRAME_HEIGHT&&
        source_desc_base_addr[4:0]==0&&source_desc_stride_bytes[4:0]==0&&
        source_desc_stride_bytes>={14'd0,source_desc_width,2'b00}&&!source_end[64];
    assign source_desc_claim_valid=rst_n&&state==CLAIM&&pipe_start_ready&&result_writer_idle;
    wire claim=source_desc_claim_valid&&source_desc_claim_ready;
    assign source_desc_release_valid=rst_n&&state==RELEASE&&pipe_idle&&result_writer_idle;
    assign source_desc_release_slot=active_slot;
    assign result_px_valid=state==ACTIVE&&pipe_out_valid;
    assign result_px_sof=out_tag==0;
    assign result_px_eol=ox==FRAME_WIDTH-1;
    assign result_px_pixel_tag=out_tag;
    wire metadata_bad=pipe_out_valid&&(out_tag!=retire_tag||result_px_source_frame_id!=frame_id);
    assign result_frame_error=frame_fault||(fetch_res_valid&&fetch_error)||descriptor_error||pipe_error||metadata_bad;
    assign debug_state={2'd0,state};assign debug_x=ox[10:0];assign debug_y=oy[10:0];

    wire coord_valid,coord_res_ready,coord_empty,coord_invalid;
    wire signed [COORD_W-1:0] sx,sy,qsx,qsy;
    wire [1:0] border,qborder;
    wire [31:0] coord_frame,coord_tag,qframe,qtag;
    wire [15:0] coord_cfg,qcfg;
    wire qvalid,qready,qempty,qinvalid;
    wire active_clamp,sobel_idle,sobel_start_ready,bilin_empty,bilin_valid,bilin_sample,sobel_ready;
    wire [10:0] active_threshold;
    wire [1:0] active_mode;
    wire [COORD_F-1:0] dx,dy;
    wire [23:0] bilin_rgb;
    wire [31:0] bilin_frame,bilin_tag;
    wire [15:0] bilin_cfg;
    reg start_pending,fetch_sample_valid;
    wire qbad=qinvalid||(!active_clamp&&qborder==2);
    wire signed [COORD_W-1:0] safe_sx=qbad?-(1<<COORD_F):qsx;
    wire signed [COORD_W-1:0] safe_sy=qbad?-(1<<COORD_F):qsy;
    assign pipe_error=1'b0;
    assign descriptor_error=1'b0;
    assign pipe_idle=coord_empty&&qempty&&!fetch_res_valid&&!bilin_valid&&sobel_idle&&!start_pending;
    dynamic_distortion_coord_cached_v04 #(.COORD_W(COORD_W),.COORD_F(COORD_F),.MAX_OUTPUT_WIDTH(MAX_WIDTH)) coord(
        .coord_clk(clk),.coord_rst_n(rst_n),
        .cfg_valid((cfg_valid&&boot_complete)||boot_valid),.cfg_ready(pipe_cfg_ready),
        .cfg_payload(boot_valid?boot_payload:user_payload),.cfg_seq(boot_valid?32'd0:cfg_seq),
        .cfg_received(cfg_received),.cfg_rejected(cfg_rejected),.cfg_applied(cfg_applied),
        .cfg_received_seq(cfg_received_seq),.cfg_applied_seq(cfg_applied_seq),.cfg_applied_frame_id(cfg_applied_frame_id),
        .active_cfg_version(active_cfg_version),.active_valid(),.pending_valid(),
        .frame_start_valid(claim&&descriptor_ok),.frame_start_ready(pipe_start_ready),
        .frame_start_id(source_desc_source_frame_id),.downstream_idle(pipe_idle&&result_writer_idle),
        .active_output_width(output_width),.active_output_height(output_height),
        .coord_req_valid(state==ACTIVE&&!issue_done&&!start_pending),.coord_req_ready(coord_ready),
        .coord_req_x(issue_x),.coord_req_y(issue_y),.coord_req_last(issue_tag==FRAME_WIDTH*FRAME_HEIGHT-1),
        .coord_req_pixel_tag(issue_tag),.coord_res_valid(coord_valid),.coord_res_ready(coord_res_ready),
        .src_x_fixed(sx),.src_y_fixed(sy),.border(border),.coord_res_invalid(coord_invalid),
        .coord_res_frame_id(coord_frame),.coord_res_cfg_version(coord_cfg),.coord_res_pixel_tag(coord_tag),
        .coord_pipe_empty(coord_empty),.active_source_width(),.active_source_height(),
        .active_border_mode(active_clamp),.active_edge_threshold(active_threshold),.active_edge_mode(active_mode));
    v04_coord_queue #(.WIDTH(2*COORD_W+83)) queue(
        .clk(clk),.rst_n(rst_n),.in_valid(coord_valid),.in_ready(coord_res_ready),
        .in_data({sx,sy,border,coord_invalid,coord_frame,coord_cfg,coord_tag}),
        .out_valid(qvalid),.out_ready(qready),.out_data({qsx,qsy,qborder,qinvalid,qframe,qcfg,qtag}),.empty(qempty));
    // Cache output and next input may handshake on the same clock.
    // This sideband advances with input; the interpolator samples the old value.
    always @(posedge clk)if(qvalid&&qready)fetch_sample_valid<=!qbad;
    pixel_fetch4_cached #(.COORD_W(COORD_W),.COORD_F(COORD_F),.REQ_ID(REQ_ID)) fetch(
        .clk(clk),.rst_n(rst_n),.in_valid(qvalid),.in_ready(qready),
        .src_x_fixed(safe_sx),.src_y_fixed(safe_sy),
        .source_width(FRAME_WIDTH[15:0]),.source_height(FRAME_HEIGHT[15:0]),
        .source_base(base),.source_stride(stride),.frame_id(qframe),.cfg_version(qcfg),.pixel_tag(qtag),
        .border_clamp(active_clamp&&!qbad),
        .fetch_rd_req_valid(fetch_rd_req_valid),.fetch_rd_req_ready(fetch_rd_req_ready),
        .fetch_rd_req_addr(fetch_rd_req_addr),.fetch_rd_req_len(fetch_rd_req_len),.fetch_rd_req_id(fetch_rd_req_id),
        .fetch_rd_rsp_valid(fetch_rd_rsp_valid),.fetch_rd_rsp_ready(fetch_rd_rsp_ready),
        .fetch_rd_rsp_data(fetch_rd_rsp_data),.fetch_rd_rsp_id(fetch_rd_rsp_id),
        .fetch_rd_rsp_last(fetch_rd_rsp_last),.fetch_rd_rsp_error(fetch_rd_rsp_error),
        .out_valid(fetch_res_valid),.out_ready(fetch_res_ready),.P00_rgb(P00),.P10_rgb(P10),.P01_rgb(P01),.P11_rgb(P11),
        .dx(dx),.dy(dy),.out_frame_id(fetch_res_frame),.out_cfg_version(fetch_res_cfg),.out_pixel_tag(fetch_res_tag),
        .out_error(fetch_error),.out_border());
    bilinear_rgb_v04 #(.COORD_F(COORD_F)) bilin(
        .bilin_clk(clk),.bilin_rst_n(rst_n),.bilin_valid(fetch_res_valid),.bilin_ready(fetch_res_ready),
        .sample_valid(fetch_sample_valid&&!fetch_error),.P00_rgb(P00),.P10_rgb(P10),.P01_rgb(P01),.P11_rgb(P11),
        .dx(dx),.dy(dy),.bilin_frame_id(fetch_res_frame),.bilin_cfg_version(fetch_res_cfg),.bilin_pixel_tag(fetch_res_tag),
        .bilin_out_valid(bilin_valid),.bilin_out_ready(sobel_ready),.pixel_rgb(bilin_rgb),
        .bilin_out_frame_id(bilin_frame),.bilin_out_cfg_version(bilin_cfg),.bilin_out_pixel_tag(bilin_tag),
        .pixel_sample_valid(bilin_sample),.bilin_pipe_empty(bilin_empty));
    always @(posedge clk or negedge rst_n)
        if(!rst_n)start_pending<=0;
        else if(claim&&descriptor_ok)start_pending<=1;
        else if(start_pending&&sobel_start_ready)start_pending<=0;
    sobel_rgb_stream_cached_v04 #(.MAX_WIDTH(MAX_WIDTH)) sobel(
        .clk(clk),.rst_n(rst_n),.frame_start_valid(start_pending),.frame_start_ready(sobel_start_ready),
        .frame_width(FRAME_WIDTH[15:0]),.frame_height(FRAME_HEIGHT[15:0]),
        .frame_id(frame_id),.cfg_version(active_cfg_version),.edge_threshold(active_threshold),.edge_mode(active_mode),
        .in_valid(bilin_valid),.in_ready(sobel_ready),.in_rgb(bilin_rgb),.in_sample_valid(bilin_sample),.in_pixel_tag(bilin_tag),
        .out_valid(pipe_out_valid),.out_ready(state==ACTIVE&&result_px_ready),.out_rgb(result_px_rgb),
        .magnitude(),.gx(),.gy(),.edge_detected(),.window_valid(),.sample_valid(),
        .out_x(ox),.out_y(oy),.out_last(out_last),.out_frame_id(result_px_source_frame_id),
        .out_cfg_version(result_px_cfg_version),.out_pixel_tag(out_tag),.idle(sobel_idle));
    assign result_px_x=ox;assign result_px_y=oy;
    always @(posedge clk)if(claim)begin base<=source_desc_base_addr;stride<=source_desc_stride_bytes;end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=CLAIM;active_slot<=0;frame_id<=0;issue_x<=0;issue_y<=0;issue_tag<=0;
            retire_tag<=0;issue_done<=0;frame_fault<=0;error_count<=0;boot_complete<=AUTO_BOOT==0;
        end else begin
            if(boot_valid&&pipe_cfg_ready)boot_complete<=1;
            if(claim)begin
                active_slot<=source_desc_claim_slot;frame_id<=source_desc_source_frame_id;
                issue_x<=0;issue_y<=0;issue_tag<=0;retire_tag<=0;issue_done<=0;frame_fault<=0;
                if(descriptor_ok)state<=ACTIVE;
                else begin state<=RELEASE;error_count<=error_count+1'b1;end
            end
            if(state==ACTIVE)begin
                if(!issue_done&&!start_pending&&coord_ready)begin
                    issue_tag<=issue_tag+1'b1;
                    if(issue_x==FRAME_WIDTH-1)begin issue_x<=0;
                        if(issue_y==FRAME_HEIGHT-1)issue_done<=1;else issue_y<=issue_y+1'b1;
                    end else issue_x<=issue_x+1'b1;
                end
                if(result_frame_error&&!frame_fault)begin frame_fault<=1;error_count<=error_count+1'b1;end
                if(result_px_valid&&result_px_ready)begin
                    retire_tag<=retire_tag+1'b1;if(out_last)state<=DRAIN;
                end
            end
            if(state==DRAIN&&pipe_idle&&result_writer_idle)state<=RELEASE;
            if(source_desc_release_valid&&source_desc_release_ready)state<=CLAIM;
        end
    end
endmodule

module v04_coord_queue #(parameter integer WIDTH=134)(
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
