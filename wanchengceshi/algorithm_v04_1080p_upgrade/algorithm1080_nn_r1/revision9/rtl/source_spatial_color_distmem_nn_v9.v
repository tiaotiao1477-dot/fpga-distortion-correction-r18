// Source-coordinate spatial gains. Four stages, including frame ownership.
// Valid coordinates are native 1920x1080; invalid-cycle coordinates are ignored.
(* syn_romstyle = "lut_rom" *)
module source_spatial_color_r1 #(
 parameter [539:0] RED_GRID={54{10'd256}},BLUE_GRID={54{10'd256}}
)(input clk,rst_n,frame_active_in,valid_in,input[23:0]rgb_in,input[15:0]x_in,y_in,
 output frame_active_out,valid_out,output[23:0]rgb_out);
wire [15:0] px=valid_in ? x_in : 16'd0;
wire [15:0] py=valid_in ? y_in : 16'd0;
wire [5:0] node_index=({3'd0,py[10:8]}*6'd9)+{3'd0,px[10:8]};
function [9:0] interpolate;
 input [9:0] a,b;input [7:0] weight;
 reg [8:0] inverse;reg [19:0] total;
 begin
  inverse=9'd256-{1'b0,weight};
  total=(a*inverse)+(b*{1'b0,weight})+20'd128;
  interpolate=total[17:8];
 end
endfunction
function [7:0] scale_saturate;
 input [7:0] channel;input [9:0] gain;reg [17:0] rounded;
 begin
  rounded=(channel*gain)+18'd128;
  scale_saturate=(|rounded[17:16])?8'd255:rounded[15:8];
 end
endfunction
reg [9:0] r00,r01,r10,r11,b00,b01,b10,b11;
reg [7:0] wx1,wy1,wy2;
reg [9:0] rt2,rb2,bt2,bb2,rg3,bg3;
reg [23:0] rgb1,rgb2,rgb3,rgb4;
reg [3:0] active_pipe,valid_pipe;
always @(posedge clk)begin
 if(!rst_n)begin
  r00<=0;r01<=0;r10<=0;r11<=0;b00<=0;b01<=0;b10<=0;b11<=0;
  wx1<=0;wy1<=0;wy2<=0;rt2<=0;rb2<=0;bt2<=0;bb2<=0;rg3<=0;bg3<=0;
  rgb1<=0;rgb2<=0;rgb3<=0;rgb4<=0;active_pipe<=0;valid_pipe<=0;
 end else begin
  active_pipe<={active_pipe[2:0],frame_active_in};valid_pipe<={valid_pipe[2:0],valid_in};
  r00<=RED_GRID[node_index*10+:10];r01<=RED_GRID[(node_index+6'd1)*10+:10];
  r10<=RED_GRID[(node_index+6'd9)*10+:10];r11<=RED_GRID[(node_index+6'd10)*10+:10];
  b00<=BLUE_GRID[node_index*10+:10];b01<=BLUE_GRID[(node_index+6'd1)*10+:10];
  b10<=BLUE_GRID[(node_index+6'd9)*10+:10];b11<=BLUE_GRID[(node_index+6'd10)*10+:10];
  wx1<=px[7:0];wy1<=py[7:0];rgb1<=rgb_in;
  rt2<=interpolate(r00,r01,wx1);rb2<=interpolate(r10,r11,wx1);
  bt2<=interpolate(b00,b01,wx1);bb2<=interpolate(b10,b11,wx1);wy2<=wy1;rgb2<=rgb1;
  rg3<=interpolate(rt2,rb2,wy2);bg3<=interpolate(bt2,bb2,wy2);rgb3<=rgb2;
  rgb4<={scale_saturate(rgb3[23:16],rg3),rgb3[15:8],scale_saturate(rgb3[7:0],bg3)};
 end
end
assign frame_active_out=active_pipe[3];assign valid_out=valid_pipe[3];assign rgb_out=rgb4;
endmodule
