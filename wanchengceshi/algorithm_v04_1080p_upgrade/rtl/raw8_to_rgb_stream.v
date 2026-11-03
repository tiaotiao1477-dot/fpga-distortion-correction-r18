`timescale 1ns/1ps
// Independent RAW8->RGB888 stream. Sensor cannot obey backpressure directly;
// integration requires the camera-domain FIFO, separate from this component.
module raw8_to_rgb_stream #(parameter integer MAX_WIDTH=1920,BAYER_PHASE=0)(
    input wire clk,rst_n,frame_start_valid,output wire frame_start_ready,
    input wire [15:0] frame_width,frame_height,input wire [31:0] frame_id,
    input wire [15:0] cfg_version,input wire in_valid,output wire in_ready,
    input wire [7:0] in_raw,input wire [31:0] in_pixel_tag,
    output wire out_valid,input wire out_ready,output wire [23:0] out_rgb,
    output wire [15:0] out_x,out_y,output wire out_last,
    output wire [31:0] out_frame_id,out_pixel_tag,
    output wire [15:0] out_cfg_version,output wire idle
);
    bayer_rgb_stream_core #(.MAX_WIDTH(MAX_WIDTH),.BAYER_PHASE(BAYER_PHASE)) core(
        .clk(clk),.rst_n(rst_n),.frame_start_valid(frame_start_valid),.frame_start_ready(frame_start_ready),
        .frame_width(frame_width),.frame_height(frame_height),.frame_id(frame_id),.cfg_version(cfg_version),
        .edge_threshold(11'd0),.edge_mode(2'd0),.in_valid(in_valid),.in_ready(in_ready),
        .in_rgb({in_raw,in_raw,in_raw}),.in_sample_valid(1'b1),.in_pixel_tag(in_pixel_tag),
        .out_valid(out_valid),.out_ready(out_ready),.out_rgb(out_rgb),.magnitude(),.gx(),.gy(),
        .edge_detected(),.window_valid(),.sample_valid(),.out_x(out_x),.out_y(out_y),.out_last(out_last),
        .out_frame_id(out_frame_id),.out_cfg_version(out_cfg_version),.out_pixel_tag(out_pixel_tag),.idle(idle));
endmodule
