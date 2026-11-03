`timescale 1ns/1ps
// RGB888 cache payload: preserve all 24 color bits, omit only unused word padding.
// External DDR remains 32-bit padded pixels; cache outputs reinsert zero padding.
// Bank selection follows each accepted read. Invalid payload is unspecified.
// No capacity/precision change and no payload reset.
module payload_rgb24_store_banked_r21 #(parameter integer ADDR_W=14)(
    input wire clk,rst_n,wr_en,input wire [1023:0] wr_data,
    input wire wr_capture,input wire [1:0] wr_capture_way,
    input wire [ADDR_W-1:0] wr_capture_word,
    input wire rd0_en,rd1_en,input wire [ADDR_W-1:0] rd0_addr,rd1_addr,
    input wire rd0_issue,rd1_issue,
    input wire [ADDR_W-1:0] rd0_issue_word,rd1_issue_word,
    output wire rd_ready,output reg out0_valid,out1_valid,
    output wire [1023:0] out0_data,out1_data
);
    localparam ROW_W=ADDR_W>11?11:ADDR_W;
    localparam BANK_W=ADDR_W>ROW_W?ADDR_W-ROW_W:1;
    localparam BANKS=1<<(ADDR_W-ROW_W);
    function [BANK_W-1:0] bank_of;
        input [ADDR_W-1:0] word_addr;
        begin
            if(ADDR_W>ROW_W)bank_of=word_addr>>ROW_W;
            else bank_of=0;
        end
    endfunction
    reg [BANK_W-1:0] read_bank0,read_bank1;
    reg read_pending0,read_pending1,bank_pending0,bank_pending1;
    assign rd_ready=rst_n&&!wr_en;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin out0_valid<=0;out1_valid<=0;read_pending0<=0;read_pending1<=0;bank_pending0<=0;bank_pending1<=0;end
        else begin
            read_pending0<=rd0_en&&!wr_en;read_pending1<=rd1_en&&!wr_en;
            bank_pending0<=read_pending0;bank_pending1<=read_pending1;
            out0_valid<=bank_pending0;out1_valid<=bank_pending1;
        end
    always @(posedge clk)begin
        if(rst_n&&rd0_en&&!wr_en)read_bank0<=rd0_addr>>ROW_W;
        if(rst_n&&rd1_en&&!wr_en)read_bank1<=rd1_addr>>ROW_W;
    end
    wire [BANK_W-1:0] wr_capture_bank=bank_of(wr_capture_word);
    wire [BANK_W-1:0] rd0_issue_bank=bank_of(rd0_issue_word);
    wire [BANK_W-1:0] rd1_issue_bank=bank_of(rd1_issue_word);
    genvar pixel,way,bank;
    generate for(way=0;way<4;way=way+1)begin:g_way
        wire [191:0] packed_write;
        for(pixel=0;pixel<8;pixel=pixel+1)begin:g_pack
            assign packed_write[pixel*24+:24]=wr_data[way*256+pixel*32+:24];
        end
        wire [BANKS*192-1:0] words0,words1;
        // Per-way copies capture the same two bank tags on the original edge.
        // They have no reset, matching the shared source registers.
        reg [BANK_W-1:0] output_bank0 /* synthesis syn_preserve=1 syn_keep=1 */;
        reg [BANK_W-1:0] output_bank1 /* synthesis syn_preserve=1 syn_keep=1 */;
        always @(posedge clk)begin
            if(rst_n&&read_pending0)output_bank0<=read_bank0;
            if(rst_n&&read_pending1)output_bank1<=read_bank1;
        end
        wire [191:0] selected0=words0[output_bank0*192+:192],selected1=words1[output_bank1*192+:192];
        reg [191:0] selected0_pipe,selected1_pipe;
        always @(posedge clk)begin selected0_pipe<=selected0;selected1_pipe<=selected1;end
        for(pixel=0;pixel<8;pixel=pixel+1)begin:g_unpack
            assign out0_data[way*256+pixel*32+:32]={8'd0,selected0_pipe[pixel*24+:24]};
            assign out1_data[way*256+pixel*32+:32]={8'd0,selected1_pipe[pixel*24+:24]};
        end
        for(bank=0;bank<BANKS;bank=bank+1)begin:g_bank
            reg [191:0] memory[0:(1<<ROW_W)-1];
            reg [191:0] word0,word1;
            reg [191:0] word0_pipe,word1_pipe;
            // Local request/commit rows are captured on the existing request
            // and payload-commit edges. These Qs feed RAM pins directly.
            reg [ROW_W-1:0] rd0_row_local /* synthesis syn_preserve=1 syn_keep=1 */;
            reg [ROW_W-1:0] rd1_row_local /* synthesis syn_preserve=1 syn_keep=1 */;
            reg [ROW_W-1:0] wr_row_local /* synthesis syn_preserve=1 syn_keep=1 */;
            reg wr_valid_local /* synthesis syn_preserve=1 syn_keep=1 */;
            reg rd0_valid_local /* synthesis syn_preserve=1 syn_keep=1 */;
            reg rd1_valid_local /* synthesis syn_preserve=1 syn_keep=1 */;
            always @(posedge clk or negedge rst_n)
                if(!rst_n)begin
                    wr_valid_local<=0;rd0_valid_local<=0;rd1_valid_local<=0;
                end else begin
                    // Fold global write suppression into the same request
                    // capture edge; no separate global-busy state is needed.
                    rd0_valid_local<=rd0_issue&&(rd0_issue_bank==bank)&&!wr_capture;
                    rd1_valid_local<=rd1_issue&&(rd1_issue_bank==bank)&&!wr_capture;
                    wr_valid_local<=wr_capture&&(wr_capture_way==way)&&(wr_capture_bank==bank);
                end
            // Address Qs intentionally have no reset; their local valid Qs
            // are reset and qualify every RAM operation.
            always @(posedge clk)begin
                if(rst_n&&rd0_issue&&(rd0_issue_bank==bank))rd0_row_local<=rd0_issue_word[ROW_W-1:0];
                if(rst_n&&rd1_issue&&(rd1_issue_bank==bank))rd1_row_local<=rd1_issue_word[ROW_W-1:0];
                if(rst_n&&wr_capture&&(wr_capture_way==way)&&(wr_capture_bank==bank))
                    wr_row_local<=wr_capture_word[ROW_W-1:0];
            end
            wire write_here /* synthesis syn_keep = 1 syn_maxfan=4 */=wr_valid_local;
            wire read_a /* synthesis syn_keep = 1 syn_maxfan=4 */=rd0_valid_local&&!wr_valid_local;
            wire read_b /* synthesis syn_keep = 1 syn_maxfan=4 */=rd1_valid_local&&!wr_valid_local;
            wire [ROW_W-1:0] a_addr_local /* synthesis syn_keep = 1 syn_maxfan=4 */=
                write_here?wr_row_local:rd0_row_local;
            wire [ROW_W-1:0] b_addr_local /* synthesis syn_keep = 1 syn_maxfan=4 */=rd1_row_local;
            assign words0[bank*192+:192]=word0_pipe;
            assign words1[bank*192+:192]=word1_pipe;
            always @(posedge clk)if(rst_n&&(write_here||read_a))begin
                if(write_here)memory[a_addr_local]<=packed_write;
                word0<=memory[a_addr_local];
            end
            always @(posedge clk)if(rst_n&&read_b)word1<=memory[b_addr_local];
            // Unreset output registers are adjacent to each RAM bank.
            // Inspect actual DOA_REG/DOB_REG mapping rather than assuming it.
            always @(posedge clk)begin word0_pipe<=word0;word1_pipe<=word1;end
        end
    end endgenerate
endmodule
