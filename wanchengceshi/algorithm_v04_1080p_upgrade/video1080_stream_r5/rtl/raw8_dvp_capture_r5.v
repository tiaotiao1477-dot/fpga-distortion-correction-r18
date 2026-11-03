`timescale 1ns/1ps
// Offline candidate: one RAW8 sample per active PCLK, no byte pairing.
// DVP timing and raw data are registered together, then consumed together.
// This does not configure a sensor or infer the Bayer phase of a real module.
module raw8_dvp_capture_r2 #(
    parameter integer WIDTH=1920,HEIGHT=1080,
    parameter HREF_ACTIVE_LEVEL=1'b0
)(
    input wire pclk,rst_n,vsync,href,input wire [7:0] data,
    output reg [7:0] raw_data,
    output reg pixel_valid,sof,eol,eof,frame_active,
    output reg [15:0] px_x,px_y,
    output reg [31:0] error_count
);
    reg vsync_q,href_active_q,previous_href;
    reg [7:0] data_q;
    reg [15:0] column,row;
    reg [31:0] frame_pixels;
    reg frame_open;
    always @(posedge pclk or negedge rst_n)begin
        if(!rst_n)begin
            vsync_q<=1;href_active_q<=0;previous_href<=0;data_q<=0;
            column<=0;row<=0;frame_pixels<=0;frame_open<=0;
            raw_data<=0;pixel_valid<=0;sof<=0;eol<=0;eof<=0;frame_active<=0;
            px_x<=0;px_y<=0;error_count<=0;
        end else begin
            vsync_q<=vsync;href_active_q<=(href==HREF_ACTIVE_LEVEL);data_q<=data;
            previous_href<=href_active_q;
            frame_active<=!vsync_q;pixel_valid<=0;sof<=0;eol<=0;eof<=0;
            if(vsync_q)begin
                previous_href<=0; // A blanking HREF tail cannot end a new-frame row.
                if(frame_open&&frame_pixels!=WIDTH*HEIGHT)error_count<=error_count+1'b1;
                column<=0;row<=0;frame_pixels<=0;frame_open<=0;
            end else begin
                frame_open<=1;
                if(href_active_q)begin
                    if(column<WIDTH&&row<HEIGHT)begin
                        raw_data<=data_q;pixel_valid<=1;px_x<=column;px_y<=row;
                        sof<=column==0&&row==0;eol<=column==WIDTH-1;eof<=column==WIDTH-1&&row==HEIGHT-1;
                        column<=column+1'b1;frame_pixels<=frame_pixels+1'b1;
                    end else error_count<=error_count+1'b1;
                end else if(previous_href)begin
                    if(column!=WIDTH)error_count<=error_count+1'b1;
                    column<=0;row<=row+1'b1;
                end
            end
        end
    end
endmodule
