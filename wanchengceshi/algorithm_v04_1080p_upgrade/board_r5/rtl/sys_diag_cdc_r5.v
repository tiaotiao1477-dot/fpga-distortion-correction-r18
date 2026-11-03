`timescale 1ns/1ps
// System-domain diagnostic reset release and independent scalar status bits.
// Each bit is a level indication; this is not a coherent multi-bit bus CDC.
module sys_diag_cdc_r5(
    input wire sys_clk,
    input wire arst_n,
    input wire [3:0] status_async,
    output wire sys_rst_n,
    output wire [3:0] status_sync
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync;
    (* ASYNC_REG = "TRUE" *) reg [3:0] status_meta,status_reg;
    always @(posedge sys_clk or negedge arst_n)
        if(!arst_n)reset_sync<=3'b000;
        else reset_sync<={reset_sync[1:0],1'b1};
    assign sys_rst_n=reset_sync[2];
    always @(posedge sys_clk or negedge sys_rst_n)
        if(!sys_rst_n)begin status_meta<=0;status_reg<=0;end
        else begin status_meta<=status_async;status_reg<=status_meta;end
    assign status_sync=status_reg;
endmodule
