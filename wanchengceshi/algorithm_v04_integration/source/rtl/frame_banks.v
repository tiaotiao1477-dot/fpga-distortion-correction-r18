`timescale 1ns/1ps
// All decisions take place in the DDR clock domain. Three frame slots:
// a reader, a completed/latest frame, and a writer. Drop older completed
// frames when input is faster; repeat the last frame when input is slower.
module frame_banks(input clk,rst_n,write_start,write_done,read_start,
 output reg [1:0] write_bank,read_bank,output reg ready);
 reg [1:0] latest;
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin write_bank<=1; read_bank<=0; latest<=0; ready<=0; end
   else begin
     if(write_done) begin latest<=write_bank; ready<=1; end
     if(read_start && ready) read_bank<=latest;
     if(write_start) begin
       // Select the only bank that is different from both the currently
       // displayed bank and the most recently completed bank.  The previous
       // pair of independent tests could select `latest` when read_bank was
       // zero, overwriting the frame that HDMI was about to display.
       if ((read_bank==0) && (latest==0)) write_bank<=1;
       else if ((read_bank==0) && (latest==1)) write_bank<=2;
       else if ((read_bank==0) && (latest==2)) write_bank<=1;
       else if ((read_bank==1) && (latest==0)) write_bank<=2;
       else if ((read_bank==1) && (latest==1)) write_bank<=0;
       else if ((read_bank==1) && (latest==2)) write_bank<=0;
       else if ((read_bank==2) && (latest==0)) write_bank<=1;
       else if ((read_bank==2) && (latest==1)) write_bank<=0;
       else write_bank<=0;
     end
   end
 end
endmodule
