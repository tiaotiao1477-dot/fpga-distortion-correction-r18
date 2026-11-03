`timescale 1ns/1ps
// Frame adapter around the actual delivered coordinate and bilinear modules.
// Native storage contracts preserved: descriptors/Fetch DDR/result pixels.
// Continuous coordinate queue feeds the native single-outstanding burst cache.
// Read failures set a monotonic independent frame-error status. The result
// writer must enable the explicit error input to abort this frame's result slot.
module distortion_storage_engine_monitor_r14 #(
    parameter integer FRAME_WIDTH=1280,FRAME_HEIGHT=720,AUTO_BOOT=1,MAX_WIDTH=1920,
    parameter integer SLOT_W=2,
    parameter integer COORD_W=26,
    parameter integer COORD_F=10,
    parameter integer BORDER_CLAMP=1,
    parameter integer CACHE_SET_BITS=10,
    parameter integer USE_BURST_CACHE=1,
    parameter integer USE_PIPELINED_COORD=1,
    parameter [7:0] REQ_ID=8'hA6,
    parameter integer ADAPTIVE_GAMMA_ENABLE=1,
    parameter integer FORCE_BLACK_BORDER=1,
    parameter GAMMA_LUT_FILE="../revision11/data/gamma_lut.hex"
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
    output wire [10:0] debug_x,debug_y,
    output wire [127:0] gamma_observation
);

    localparam CLAIM=0,ACTIVE=1,DRAIN=2,RELEASE=3;
    reg [1:0] state;
    reg start_pending;
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
    wire gamma_idle,gamma_start_ready,gamma_in_ready,gamma_valid,gamma_sample;
    wire [23:0] gamma_rgb;
    wire [31:0] gamma_tag;
    wire [31:0] gamma_out_frame,gamma_stats_frame;
    wire [15:0] gamma_out_cfg,gamma_stats_cfg;
    wire [39:0] gamma_last_sum;
    wire [31:0] gamma_last_count;
    wire gamma_stats_valid;
    wire [2:0] gamma_active_bank,gamma_stats_applied_bank,gamma_stats_next_bank;
    reg gamma_history_valid;
    reg [15:0] gamma_history_cfg_version;
    wire gamma_history_reset=!gamma_history_valid || (active_cfg_version!=gamma_history_cfg_version);
    wire pixel_stages_start=start_pending&&sobel_start_ready&&gamma_start_ready;
    wire gamma_clean_frame_commit=(state==DRAIN)&&pipe_idle&&result_writer_idle&&!result_frame_error;
    // Read-only publication point: pipe_idle includes gamma_idle, so the
    // accumulator's registered frame/config statistics have finalized.
    wire gamma_stats_match_commit=(gamma_stats_frame==frame_id)&&
        (gamma_stats_cfg==active_cfg_version);
    gamma_clean_stats_observer_r14 u_gamma_observer(
        .clk(clk),.rst_n(rst_n),
        .frame_start(pixel_stages_start),
        .history_reset_at_frame_start(pixel_stages_start && gamma_history_reset),
        .clean_frame_commit(gamma_clean_frame_commit),
        .commit_frame_id(frame_id),.commit_cfg_version(active_cfg_version),
        .stats_valid(gamma_stats_valid),.stats_match(gamma_stats_match_commit),
        .stats_sum(gamma_last_sum),.stats_count(gamma_last_count),
        .stats_frame_id(gamma_stats_frame),.stats_cfg_version(gamma_stats_cfg),
        .stats_applied_bank(gamma_stats_applied_bank),.stats_next_bank(gamma_stats_next_bank),
        .adaptive_enable(ADAPTIVE_GAMMA_ENABLE!=0),.observation(gamma_observation));
    wire [10:0] active_threshold;
    wire [1:0] active_mode;
    wire [COORD_F-1:0] dx,dy;
    wire [23:0] bilin_rgb;
    wire [31:0] bilin_frame,bilin_tag;
    wire [15:0] bilin_cfg;
    reg [7:0] fetch_sample_fifo;
    reg [2:0] fetch_sample_head,fetch_sample_tail;
    wire fetch_sample_valid=fetch_sample_fifo[fetch_sample_head];
    reg [3:0] fetch_pending /* synthesis syn_preserve = 1 */;
    wire pixel_valid,pixel_ready,pixel_empty,pixel_sample,pixel_error;
    wire [23:0] pixel_P00,pixel_P10,pixel_P01,pixel_P11;
    wire [COORD_F-1:0] pixel_dx,pixel_dy;
    wire [31:0] pixel_frame,pixel_tag;
    wire [15:0] pixel_cfg;
    // qborder==01 is also asserted at the exact final source coordinate, so
    // black-fill qualification uses the signed Q-format coordinate itself.
    wire signed [63:0] qx64={{(64-COORD_W){qsx[COORD_W-1]}},qsx};
    wire signed [63:0] qy64={{(64-COORD_W){qsy[COORD_W-1]}},qsy};
    wire signed [63:0] max_source_x_q=({32'd0,FRAME_WIDTH}-64'd1)<<<COORD_F;
    wire signed [63:0] max_source_y_q=({32'd0,FRAME_HEIGHT}-64'd1)<<<COORD_F;
    wire qoutside=(FORCE_BLACK_BORDER!=0)&&
        (qx64<0||qy64<0||qx64>max_source_x_q||qy64>max_source_y_q);
    wire qbad=qinvalid||(!active_clamp&&qborder==2)||qoutside;
    wire signed [COORD_W-1:0] safe_sx=qbad?-(1<<COORD_F):qsx;
    wire signed [COORD_W-1:0] safe_sy=qbad?-(1<<COORD_F):qsy;
    assign pipe_error=1'b0;
    assign descriptor_error=1'b0;
    // Independent candidate: eight ordered outstanding quads. Occupancy and
    // sample-valid sideband both advance on accepted/retired transactions.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin fetch_pending<=0;fetch_sample_head<=0;fetch_sample_tail<=0;end
        else begin
            case({qvalid&&qready,fetch_res_valid&&fetch_res_ready})
                2'b10:fetch_pending<=fetch_pending+1'b1;
                2'b01:fetch_pending<=fetch_pending-1'b1;
                default:;
            endcase
            if(qvalid&&qready)fetch_sample_tail<=fetch_sample_tail+1'b1;
            if(fetch_res_valid&&fetch_res_ready)fetch_sample_head<=fetch_sample_head+1'b1;
        end
    end
    assign pipe_idle=coord_empty&&qempty&&!fetch_pending&&pixel_empty&&!bilin_valid&&bilin_empty&&
        gamma_idle&&sobel_idle&&!start_pending;
    dynamic_distortion_coord_pipeline_v04 #(.COORD_W(COORD_W),.COORD_F(COORD_F),.MAX_OUTPUT_WIDTH(MAX_WIDTH)) coord(
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
    v04_coord_queue_pipeline #(.WIDTH(2*COORD_W+83)) queue(
        .clk(clk),.rst_n(rst_n),.in_valid(coord_valid),.in_ready(coord_res_ready),
        .in_data({sx,sy,border,coord_invalid,coord_frame,coord_cfg,coord_tag}),
        .out_valid(qvalid),.out_ready(qready),.out_data({qsx,qsy,qborder,qinvalid,qframe,qcfg,qtag}),.empty(qempty));
    // The sample-valid ring follows every accepted request until ordered return.
    // A scalar sideband would be overwritten by younger in-flight quads.
    always @(posedge clk)if(qvalid&&qready)fetch_sample_fifo[fetch_sample_tail]<=!qbad&&!qoutside;
    wire fetch_zero,pixel_zero;
    pixel_fetch4_stream2_cached_r24 #(.COORD_W(COORD_W),.COORD_F(COORD_F),.REQ_ID(REQ_ID),.CACHE_SET_BITS(CACHE_SET_BITS),.CACHE_BLOCK_BITS(9),.DEFER_ZERO_MASK(1)) fetch(
        .clk(clk),.rst_n(rst_n),.in_valid(qvalid),.in_ready(qready),
        .src_x_fixed(safe_sx),.src_y_fixed(safe_sy),
        .source_width(FRAME_WIDTH[15:0]),.source_height(FRAME_HEIGHT[15:0]),
        .source_base(base),.source_stride(stride),.frame_id(qframe),.cfg_version(qcfg),.pixel_tag(qtag),
        // Outside taps become the fetcher's existing zero/no-read result.
        // This does not set fetch_error; the queued sample-valid bit prevents
        // them from entering adaptive statistics or Sobel validity counts.
        .border_clamp(active_clamp&&!qbad),
        .fetch_rd_req_valid(fetch_rd_req_valid),.fetch_rd_req_ready(fetch_rd_req_ready),
        .fetch_rd_req_addr(fetch_rd_req_addr),.fetch_rd_req_len(fetch_rd_req_len),.fetch_rd_req_id(fetch_rd_req_id),
        .fetch_rd_rsp_valid(fetch_rd_rsp_valid),.fetch_rd_rsp_ready(fetch_rd_rsp_ready),
        .fetch_rd_rsp_data(fetch_rd_rsp_data),.fetch_rd_rsp_id(fetch_rd_rsp_id),
        .fetch_rd_rsp_last(fetch_rd_rsp_last),.fetch_rd_rsp_error(fetch_rd_rsp_error),
        .out_valid(fetch_res_valid),.out_ready(fetch_res_ready),.P00_rgb(P00),.P10_rgb(P10),.P01_rgb(P01),.P11_rgb(P11),
        .dx(dx),.dy(dy),.out_frame_id(fetch_res_frame),.out_cfg_version(fetch_res_cfg),.out_pixel_tag(fetch_res_tag),
        .out_error(fetch_error),.out_border(),.out_pixels_zero(fetch_zero));
    // No combinational fall-through: Fetch hit/tag logic ends at queue storage.
    // Two entries preserve simultaneous output/input without routing downstream
    // ready back through tag comparison. Every sideband follows the four taps.
    // Defer zero masking across this register boundary. Raw taps may contain
    // stale data for an invalid sample, but the queued flag zeros them before
    // bilinear. Zero, tags and all RGB values share one atomic queue entry.
    v04_coord_queue_pipeline #(.WIDTH(179+2*COORD_F),.CAPTURE_FREE(1)) pixel_queue(
        .clk(clk),.rst_n(rst_n),.in_valid(fetch_res_valid),.in_ready(fetch_res_ready),
        .in_data({P00,P10,P01,P11,dx,dy,fetch_res_frame,fetch_res_cfg,fetch_res_tag,fetch_sample_valid,fetch_error,fetch_zero}),
        .out_valid(pixel_valid),.out_ready(pixel_ready),
        .out_data({pixel_P00,pixel_P10,pixel_P01,pixel_P11,pixel_dx,pixel_dy,pixel_frame,pixel_cfg,pixel_tag,pixel_sample,pixel_error,pixel_zero}),
        .empty(pixel_empty));
    bilinear_rgb_v04 #(.COORD_F(COORD_F)) bilin(
        .bilin_clk(clk),.bilin_rst_n(rst_n),.bilin_valid(pixel_valid),.bilin_ready(pixel_ready),
        .sample_valid(pixel_sample&&!pixel_error),.P00_rgb(pixel_zero?24'd0:pixel_P00),.P10_rgb(pixel_zero?24'd0:pixel_P10),
        .P01_rgb(pixel_zero?24'd0:pixel_P01),.P11_rgb(pixel_zero?24'd0:pixel_P11),
        .dx(pixel_dx),.dy(pixel_dy),.bilin_frame_id(pixel_frame),.bilin_cfg_version(pixel_cfg),.bilin_pixel_tag(pixel_tag),
        .bilin_out_valid(bilin_valid),.bilin_out_ready(gamma_in_ready),.pixel_rgb(bilin_rgb),
        .bilin_out_frame_id(bilin_frame),.bilin_out_cfg_version(bilin_cfg),.bilin_out_pixel_tag(bilin_tag),
        .pixel_sample_valid(bilin_sample),.bilin_pipe_empty(bilin_empty));
    always @(posedge clk or negedge rst_n)
        if(!rst_n)start_pending<=0;
        else if(claim&&descriptor_ok)start_pending<=1;
        else if(pixel_stages_start)start_pending<=0;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin gamma_history_valid<=0;gamma_history_cfg_version<=0;end
        else if(pixel_stages_start)begin
            gamma_history_valid<=1;
            gamma_history_cfg_version<=active_cfg_version;
        end
    adaptive_gamma_rgb_v06 #(.LUT_FILE(GAMMA_LUT_FILE)) gamma(
        .clk(clk),.rst_n(rst_n),.frame_start_valid(pixel_stages_start),.frame_start_ready(gamma_start_ready),
        .frame_width(FRAME_WIDTH[15:0]),.frame_height(FRAME_HEIGHT[15:0]),
        .frame_id(frame_id),.cfg_version(active_cfg_version),
        .enable(ADAPTIVE_GAMMA_ENABLE!=0),.history_reset(gamma_history_reset),
        .in_valid(bilin_valid),.in_ready(gamma_in_ready),.in_rgb(bilin_rgb),
        .in_sample_valid(bilin_sample),.in_pixel_tag(bilin_tag),
        .out_valid(gamma_valid),.out_ready(sobel_ready),.out_rgb(gamma_rgb),
        .out_sample_valid(gamma_sample),.out_pixel_tag(gamma_tag),
        .out_frame_id(gamma_out_frame),.out_cfg_version(gamma_out_cfg),.out_last(),
        .active_bank(gamma_active_bank),.last_luminance_sum(gamma_last_sum),
        .last_valid_count(gamma_last_count),.stats_valid(gamma_stats_valid),
        .stats_frame_id(gamma_stats_frame),.stats_cfg_version(gamma_stats_cfg),
        .stats_applied_bank(gamma_stats_applied_bank),.stats_next_bank(gamma_stats_next_bank),
        .clean_frame_commit(gamma_clean_frame_commit),.idle(gamma_idle));
    sobel_rgb_stream_pipe_v04 #(.MAX_WIDTH(MAX_WIDTH)) sobel(
        .clk(clk),.rst_n(rst_n),.frame_start_valid(pixel_stages_start),.frame_start_ready(sobel_start_ready),
        .frame_width(FRAME_WIDTH[15:0]),.frame_height(FRAME_HEIGHT[15:0]),
        .frame_id(frame_id),.cfg_version(active_cfg_version),.edge_threshold(active_threshold),.edge_mode(active_mode),
        .in_valid(gamma_valid),.in_ready(sobel_ready),.in_rgb(gamma_rgb),.in_sample_valid(gamma_sample),.in_pixel_tag(gamma_tag),
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

module v04_coord_queue_pipeline #(parameter integer WIDTH=134, CAPTURE_FREE=0)(
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
        // With room, write_ptr always names an unoccupied slot. Pre-capture
        // may update invalid data only; count/pointers still advance on push.
        // Full queues suppress writes, including the full+pop clock. Thus the
        // occupied head is stable while rawvalid no longer drives wide CEs.
        if(CAPTURE_FREE ? in_ready : push)payload[write_ptr]<=in_data;
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

// Bundled-data observation register. It has no path back into the algorithm.
module gamma_clean_stats_observer_r14(
    input wire clk,rst_n,
    input wire frame_start,
    input wire history_reset_at_frame_start,
    input wire clean_frame_commit,
    input wire [31:0] commit_frame_id,
    input wire [15:0] commit_cfg_version,
    input wire stats_valid,stats_match,
    input wire [39:0] stats_sum,
    input wire [31:0] stats_count,stats_frame_id,
    input wire [15:0] stats_cfg_version,
    input wire [2:0] stats_applied_bank,stats_next_bank,
    input wire adaptive_enable,
    output reg [127:0] observation
);
    reg pending_valid,pending_match,pending_enable;
    reg [39:0] pending_sum;
    reg [31:0] pending_count,pending_frame_id;
    reg [15:0] pending_cfg_version;
    reg [2:0] pending_applied_bank,pending_next_bank;
    wire pending_matches_commit=pending_match &&
        pending_frame_id==commit_frame_id && pending_cfg_version==commit_cfg_version;
    wire pulse_matches_commit=stats_match &&
        stats_frame_id==commit_frame_id && stats_cfg_version==commit_cfg_version;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            observation<=128'd0;
            pending_valid<=1'b0;pending_match<=1'b0;pending_enable<=1'b0;
            pending_sum<=0;pending_count<=0;pending_frame_id<=0;pending_cfg_version<=0;
            pending_applied_bank<=0;pending_next_bank<=0;
        end else begin
            // Any new actual Gamma frame owns future statistics. A dirty prior
            // frame's pending pulse must never leak into this frame's commit.
            if(frame_start) pending_valid<=1'b0;
            if(history_reset_at_frame_start) observation[127]<=1'b0;

            // Gamma stats_valid is a one-cycle finalize pulse. Hold its complete
            // bundled record until the later result-frame clean commit.
            if(stats_valid) begin
                pending_valid<=1'b1;
                pending_match<=stats_match;
                pending_sum<=stats_sum;
                pending_count<=stats_count;
                pending_frame_id<=stats_frame_id;
                pending_cfg_version<=stats_cfg_version;
                pending_applied_bank<=stats_applied_bank;
                pending_next_bank<=stats_next_bank;
                pending_enable<=adaptive_enable;
            end

            if(clean_frame_commit) begin
                if(stats_valid) begin
                    // Also support coincident finalize/commit without waiting a cycle.
                    pending_valid<=1'b0;
                    if(pulse_matches_commit) begin
                        observation[39:0]<=stats_sum;
                        observation[71:40]<=stats_count;
                        observation[103:72]<=stats_frame_id;
                        observation[119:104]<=stats_cfg_version;
                        observation[122:120]<=stats_applied_bank;
                        observation[125:123]<=stats_next_bank;
                        observation[126]<=adaptive_enable;
                        observation[127]<=1'b1;
                    end else observation[127]<=1'b0;
                end else if(pending_valid) begin
                    pending_valid<=1'b0;
                    if(pending_matches_commit) begin
                        observation[39:0]<=pending_sum;
                        observation[71:40]<=pending_count;
                        observation[103:72]<=pending_frame_id;
                        observation[119:104]<=pending_cfg_version;
                        observation[122:120]<=pending_applied_bank;
                        observation[125:123]<=pending_next_bank;
                        observation[126]<=pending_enable;
                        observation[127]<=1'b1;
                    end else observation[127]<=1'b0;
                end else observation[127]<=1'b0;
            end
        end
    end
endmodule
