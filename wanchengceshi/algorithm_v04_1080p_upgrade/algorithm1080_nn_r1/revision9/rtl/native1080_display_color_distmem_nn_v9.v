// RGB888 output-only spatial correction. Grid index=row*9+column;
// index0 is the low10bits; tone index0 is the low byte.
// Five registered stages; DE/HS/VS share exactly the RGB latency.
(* syn_romstyle = "lut_rom" *)
module native1080_display_color_r10 #(
 parameter [539:0] RED_GRID={54{10'd256}},
 parameter [539:0] BLUE_GRID={54{10'd256}},
 parameter [2047:0] TONE_LUT=2048'd0
)(input wire clk,input wire reset,input wire [23:0] rgb_in,
 input wire de_in,input wire hs_in,input wire vs_in,
 output wire [23:0] rgb_out,output wire de_out,output wire hs_out,output wire vs_out);
reg de_previous;
reg [10:0] x_next,y_current;
wire [10:0] x_position=(vs_in || !de_previous) ? 11'd0 : x_next;
wire [10:0] y_position=vs_in ? 11'd0 : y_current;
wire [5:0] node_index=({3'd0,y_position[10:8]}*6'd9)+{3'd0,x_position[10:8]};

// Observe geometry without masking errors or changing DE on malformed runs.
always @(posedge clk) begin
 if(reset)begin de_previous<=0;x_next<=0;y_current<=0;end
 else begin
  de_previous<=de_in;
  if(vs_in)begin x_next<=0;y_current<=0;end
  else begin
   if(de_in)begin
    if(!de_previous)x_next<=11'd1;
    else if(x_next<11'd1919)x_next<=x_next+11'd1;
   end else x_next<=0;
   if(de_previous && !de_in && y_current<11'd1079)y_current<=y_current+11'd1;
  end
 end
end

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
 input [7:0] channel;input [9:0] gain;
 reg [17:0] rounded;
 begin
  rounded=(channel*gain)+18'd128;
  scale_saturate=(|rounded[17:16]) ? 8'd255 : rounded[15:8];
 end
endfunction
function [7:0] tone;
 input [7:0] channel;
 begin tone=TONE_LUT[channel*8+:8];end
endfunction

reg [9:0] r00,r01,r10,r11,b00,b01,b10,b11;
reg [7:0] wx1,wy1,wy2;
reg [9:0] rt2,rb2,bt2,bb2,rg3,bg3;
reg [23:0] rgb1,rgb2,rgb3,rgb4,rgb5;
reg [4:0] de_pipe,hs_pipe,vs_pipe;
always @(posedge clk) begin
 if(reset)begin
  r00<=0;r01<=0;r10<=0;r11<=0;b00<=0;b01<=0;b10<=0;b11<=0;
  wx1<=0;wy1<=0;wy2<=0;rt2<=0;rb2<=0;bt2<=0;bb2<=0;rg3<=0;bg3<=0;
  rgb1<=0;rgb2<=0;rgb3<=0;rgb4<=0;rgb5<=0;de_pipe<=0;hs_pipe<=0;vs_pipe<=0;
 end else begin
  de_pipe<={de_pipe[3:0],de_in};hs_pipe<={hs_pipe[3:0],hs_in};vs_pipe<={vs_pipe[3:0],vs_in};
  // Stage1: actual input coordinates and four corners per channel.
  r00<=RED_GRID[node_index*10+:10];r01<=RED_GRID[(node_index+6'd1)*10+:10];
  r10<=RED_GRID[(node_index+6'd9)*10+:10];r11<=RED_GRID[(node_index+6'd10)*10+:10];
  b00<=BLUE_GRID[node_index*10+:10];b01<=BLUE_GRID[(node_index+6'd1)*10+:10];
  b10<=BLUE_GRID[(node_index+6'd9)*10+:10];b11<=BLUE_GRID[(node_index+6'd10)*10+:10];
  wx1<=x_position[7:0];wy1<=y_position[7:0];rgb1<=rgb_in;
  // Stage2/3: independently rounded horizontal and vertical interpolation.
  rt2<=interpolate(r00,r01,wx1);rb2<=interpolate(r10,r11,wx1);
  bt2<=interpolate(b00,b01,wx1);bb2<=interpolate(b10,b11,wx1);wy2<=wy1;rgb2<=rgb1;
  rg3<=interpolate(rt2,rb2,wy2);bg3<=interpolate(bt2,bb2,wy2);rgb3<=rgb2;
  // Stage4: preserve RGB888, saturate before tone indexing.
  rgb4<={scale_saturate(rgb3[23:16],rg3),rgb3[15:8],scale_saturate(rgb3[7:0],bg3)};
  // Stage5: same common tone for all channels, including blanking RGB.
  rgb5<={tone(rgb4[23:16]),tone(rgb4[15:8]),tone(rgb4[7:0])};
 end
end
assign rgb_out=rgb5;
assign de_out=de_pipe[4];assign hs_out=hs_pipe[4];assign vs_out=vs_pipe[4];
endmodule
