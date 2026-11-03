// Coordinate/border/Fetch4/bilinear/Sobel integration candidate.
// Fetch4 is deliberately single-outstanding; connect an actual DDR adapter.
module dynamic_distortion_edge_cached_v04 #(
    parameter integer COORD_W=26, parameter integer COORD_F=10,
    parameter integer MAX_WIDTH=1920
) (
    input clk, input rst_n,
    input cfg_valid, output cfg_ready, input [735:0] cfg_payload, input [31:0] cfg_seq,
    output cfg_received, output cfg_rejected, output cfg_applied,
    output [31:0] cfg_received_seq, output [31:0] cfg_applied_seq,
    output [31:0] cfg_applied_frame_id, output [15:0] active_cfg_version,
    input frame_start_valid, output frame_start_ready, input [31:0] frame_start_id,
    input result_writer_idle,
    output [15:0] active_output_width, output [15:0] active_output_height,
    input coord_req_valid, output coord_req_ready,
    input [15:0] coord_req_x, input [15:0] coord_req_y,
    input coord_req_last, input [31:0] coord_req_pixel_tag,
    output fetch_req_valid, input fetch_req_ready,
    output [15:0] fetch_x0, output [15:0] fetch_x1,
    output [15:0] fetch_y0, output [15:0] fetch_y1,
    output [31:0] fetch_frame_id, output [15:0] fetch_cfg_version, output [31:0] fetch_pixel_tag,
    input fetch_res_valid, output fetch_res_ready,
    input [23:0] P00_rgb, input [23:0] P10_rgb, input [23:0] P01_rgb, input [23:0] P11_rgb,
    input [31:0] fetch_res_frame_id, input [15:0] fetch_res_cfg_version, input [31:0] fetch_res_pixel_tag,
    output out_valid, input out_ready, output [23:0] out_rgb,
    output [10:0] magnitude, output signed [11:0] gx, output signed [11:0] gy,
    output edge_detected, output window_valid, output sample_valid,
    output [15:0] out_x, output [15:0] out_y, output out_last,
    output [31:0] out_frame_id, output [15:0] out_cfg_version, output [31:0] out_pixel_tag,
    output reg protocol_error, output idle
);
    wire active_valid,pending_valid,coord_start_ready,coord_empty,coord_valid,coord_ready;
    wire signed [COORD_W-1:0] sx,sy;
    wire [1:0] coord_border;
    wire geom_invalid,clamp_mode;
    wire [15:0] source_width,source_height;
    wire [10:0] threshold;
    wire [1:0] mode;
    wire [31:0] coord_frame,coord_tag;
    wire [15:0] coord_version;
    wire sampler_valid,sampler_ready,sampler_empty,sample_enable;
    wire [15:0] sampler_x0,sampler_x1,sampler_y0,sampler_y1;
    wire [COORD_F-1:0] sampler_dx,sampler_dy;
    wire [31:0] sampler_frame,sampler_tag;
    wire [15:0] sampler_version;
    wire bilin_ready,bilin_valid,bilin_empty,bilin_sample;
    wire [23:0] bilin_rgb;
    wire [31:0] bilin_tag,bilin_frame;
    wire [15:0] bilin_version;
    wire sobel_start_ready,sobel_in_ready,sobel_idle;
    reg pending_sobel_start,processing;
    reg [31:0] starting_frame;
    localparam FETCH_IDLE=0,FETCH_REQUEST=1,FETCH_RESPONSE=2,INTERPOLATE=3;
    reg [1:0] fetch_state;
    reg [15:0] x0,x1,y0,y1;
    reg [COORD_F-1:0] dx,dy;
    reg [31:0] pixel_frame,pixel_tag;
    reg [15:0] pixel_version;
    reg pixel_sample;
    reg [23:0] p00,p10,p01,p11;
    wire stages_idle=(fetch_state==FETCH_IDLE) && sampler_empty && bilin_empty && sobel_idle;
    assign idle=stages_idle && coord_empty && !processing && !pending_sobel_start;
    assign frame_start_ready=coord_start_ready && !processing && !pending_sobel_start && stages_idle;
    wire start_transfer=frame_start_valid && frame_start_ready;
    wire coord_input_ready;
    assign coord_req_ready=processing && !pending_sobel_start && coord_input_ready;

    dynamic_distortion_coord_cached_v04 #(.COORD_W(COORD_W),.COORD_F(COORD_F),.MAX_OUTPUT_WIDTH(MAX_WIDTH)) coord (
        .coord_clk(clk),.coord_rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_ready(cfg_ready),
        .cfg_payload(cfg_payload),.cfg_seq(cfg_seq),.cfg_received(cfg_received),.cfg_rejected(cfg_rejected),
        .cfg_received_seq(cfg_received_seq),.cfg_applied(cfg_applied),.cfg_applied_seq(cfg_applied_seq),
        .cfg_applied_frame_id(cfg_applied_frame_id),.active_cfg_version(active_cfg_version),
        .active_valid(active_valid),.pending_valid(pending_valid),
        .frame_start_valid(frame_start_valid && !processing && !pending_sobel_start && stages_idle),
        .frame_start_ready(coord_start_ready),.frame_start_id(frame_start_id),
        .downstream_idle(stages_idle && result_writer_idle),
        .active_output_width(active_output_width),.active_output_height(active_output_height),
        .coord_req_valid(coord_req_valid && processing && !pending_sobel_start),.coord_req_ready(coord_input_ready),
        .coord_req_x(coord_req_x),.coord_req_y(coord_req_y),.coord_req_last(coord_req_last),
        .coord_req_pixel_tag(coord_req_pixel_tag),.coord_res_valid(coord_valid),.coord_res_ready(coord_ready),
        .src_x_fixed(sx),.src_y_fixed(sy),.border(coord_border),.coord_res_invalid(geom_invalid),
        .coord_res_frame_id(coord_frame),.coord_res_cfg_version(coord_version),.coord_res_pixel_tag(coord_tag),
        .coord_pipe_empty(coord_empty),.active_source_width(source_width),.active_source_height(source_height),
        .active_border_mode(clamp_mode),.active_edge_threshold(threshold),.active_edge_mode(mode)
    );
    border_sampling #(.COORD_W(COORD_W),.COORD_F(COORD_F)) sampler (
        .clk(clk),.rst_n(rst_n),.in_valid(coord_valid),.in_ready(coord_ready),
        .src_x(sx),.src_y(sy),.border(coord_border),.geom_invalid(geom_invalid),.clamp_mode(clamp_mode),
        .source_width(source_width),.source_height(source_height),.in_frame_id(coord_frame),
        .in_cfg_version(coord_version),.in_pixel_tag(coord_tag),.out_valid(sampler_valid),.out_ready(sampler_ready),
        .x0(sampler_x0),.x1(sampler_x1),.y0(sampler_y0),.y1(sampler_y1),.dx(sampler_dx),.dy(sampler_dy),
        .sample_enable(sample_enable),.outside(),.boundary(),.out_frame_id(sampler_frame),
        .out_cfg_version(sampler_version),.out_pixel_tag(sampler_tag),.pipe_empty(sampler_empty)
    );
    assign sampler_ready=(fetch_state==FETCH_IDLE);
    assign fetch_req_valid=(fetch_state==FETCH_REQUEST);
    assign fetch_res_ready=(fetch_state==FETCH_RESPONSE);
    assign fetch_x0=x0; assign fetch_x1=x1; assign fetch_y0=y0; assign fetch_y1=y1;
    assign fetch_frame_id=pixel_frame; assign fetch_cfg_version=pixel_version; assign fetch_pixel_tag=pixel_tag;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fetch_state<=FETCH_IDLE; pending_sobel_start<=0; processing<=0; protocol_error<=0;
        end else begin
            if (start_transfer) begin pending_sobel_start<=1; starting_frame<=frame_start_id; end
            if (pending_sobel_start && sobel_start_ready) begin pending_sobel_start<=0; processing<=1; end
            if (out_valid && out_ready && out_last) processing<=0;
            case(fetch_state)
                FETCH_IDLE: if (sampler_valid && sampler_ready) begin
                    x0<=sampler_x0; x1<=sampler_x1; y0<=sampler_y0; y1<=sampler_y1;
                    dx<=sampler_dx; dy<=sampler_dy; pixel_frame<=sampler_frame;
                    pixel_version<=sampler_version; pixel_tag<=sampler_tag; pixel_sample<=sample_enable;
                    if (sample_enable) fetch_state<=FETCH_REQUEST;
                    else begin p00<=0; p10<=0; p01<=0; p11<=0; fetch_state<=INTERPOLATE; end
                end
                FETCH_REQUEST: if (fetch_req_ready) fetch_state<=FETCH_RESPONSE;
                FETCH_RESPONSE: if (fetch_res_valid) begin
                    if (fetch_res_frame_id==pixel_frame && fetch_res_cfg_version==pixel_version && fetch_res_pixel_tag==pixel_tag) begin
                        p00<=P00_rgb; p10<=P10_rgb; p01<=P01_rgb; p11<=P11_rgb; fetch_state<=INTERPOLATE;
                    end else protocol_error<=1;
                end
                INTERPOLATE: if (bilin_ready) fetch_state<=FETCH_IDLE;
            endcase
        end
    end
    bilinear_rgb_v04 #(.COORD_F(COORD_F)) interpolation (
        .bilin_clk(clk),.bilin_rst_n(rst_n),.bilin_valid(fetch_state==INTERPOLATE),.bilin_ready(bilin_ready),
        .sample_valid(pixel_sample),.P00_rgb(p00),.P10_rgb(p10),.P01_rgb(p01),.P11_rgb(p11),.dx(dx),.dy(dy),
        .bilin_frame_id(pixel_frame),.bilin_cfg_version(pixel_version),.bilin_pixel_tag(pixel_tag),
        .bilin_out_valid(bilin_valid),.bilin_out_ready(sobel_in_ready),.pixel_rgb(bilin_rgb),
        .bilin_out_frame_id(bilin_frame),.bilin_out_cfg_version(bilin_version),.bilin_out_pixel_tag(bilin_tag),
        .pixel_sample_valid(bilin_sample),.bilin_pipe_empty(bilin_empty)
    );
    sobel_rgb_stream #(.MAX_WIDTH(MAX_WIDTH)) edges (
        .clk(clk),.rst_n(rst_n),.frame_start_valid(pending_sobel_start),.frame_start_ready(sobel_start_ready),
        .frame_width(active_output_width),.frame_height(active_output_height),.frame_id(starting_frame),
        .cfg_version(active_cfg_version),.edge_threshold(threshold),.edge_mode(mode),
        .in_valid(bilin_valid),.in_ready(sobel_in_ready),.in_rgb(bilin_rgb),.in_sample_valid(bilin_sample),
        .in_pixel_tag(bilin_tag),.out_valid(out_valid),.out_ready(out_ready),.out_rgb(out_rgb),
        .magnitude(magnitude),.gx(gx),.gy(gy),.edge_detected(edge_detected),.window_valid(window_valid),
        .sample_valid(sample_valid),.out_x(out_x),.out_y(out_y),.out_last(out_last),
        .out_frame_id(out_frame_id),.out_cfg_version(out_cfg_version),.out_pixel_tag(out_pixel_tag),.idle(sobel_idle)
    );
endmodule
