`timescale 1ns/1ps
// Sole external DVP data/control sampling boundary. No enable/reset: samples
// remain coherent while downstream reset is asserted or PCLK stops/restarts.
module dvp_input_ioreg_r5(
 input wire pclk,input wire [7:0] data,input wire href,vsync,
 output wire [7:0] sample_data,output wire sample_href,sample_vsync
);
 (* PAP_IO_REGISTER=1 *) reg [9:0] samples_q;
 always @(posedge pclk) samples_q <= {vsync,href,data};
 assign {sample_vsync,sample_href,sample_data}=samples_q;
endmodule
