`timescale 1ns/1ps
// One outstanding 16-bit operation, 256 direct-mapped 32-byte write-back lines.
// All ports are core_clk. Flush is unnecessary between CNN layers: reads see
// dirty resident lines. invalidate is legal only while the CNN is quiescent;
// it discards activations before a new model upload, rather than flushing them.
module nn_word_cache #(
    parameter [63:0] WEIGHTS_BASE=64'h03000000,
    parameter [63:0] WEIGHTS_LIMIT=64'h03092e10,
    parameter [63:0] A0_BASE=64'h03100000,
    parameter [63:0] A1_BASE=64'h03200000
)(
    input clk,input rst_n,
    input invalidate_valid,output invalidate_ready,
    input req_valid,output req_ready,input req_write,
    input [63:0] req_addr,input [15:0] req_wdata,
    output rsp_valid,input rsp_ready,output [15:0] rsp_rdata,output rsp_error,
    output idle,
    output rd_req_valid,input rd_req_ready,output [63:0] rd_req_addr,
    output [31:0] rd_req_len,output [7:0] rd_req_id,
    input rd_rsp_valid,output rd_rsp_ready,input [255:0] rd_rsp_data,
    input [7:0] rd_rsp_id,input rd_rsp_last,input rd_rsp_error,
    output wr_req_valid,input wr_req_ready,output [63:0] wr_req_addr,
    output [31:0] wr_req_len,output [7:0] wr_req_id,
    output wr_data_valid,input wr_data_ready,output [255:0] wr_data,
    output wr_data_last,input wr_done_valid,input [7:0] wr_done_id,input wr_done_error
);
    localparam IDLE=0,LOOKUP=1,CHECK=2,EVICT_REQ=3,EVICT_DATA=4,
               EVICT_WAIT=5,FILL_REQ=6,FILL_WAIT=7,ACCESS=8,REPLY=9;
    reg [3:0] state;
    // Four 64-bit data slices share the original 256-entry index space.
    reg [63:0] lines0[0:255] /* synthesis syn_ramstyle = "block_ram" */;
    reg [63:0] lines1[0:255] /* synthesis syn_ramstyle = "block_ram" */;
    reg [63:0] lines2[0:255] /* synthesis syn_ramstyle = "block_ram" */;
    reg [63:0] lines3[0:255] /* synthesis syn_ramstyle = "block_ram" */;
    reg [12:0] tags[0:255];
    reg [255:0] valid_bits,dirty_bits;
    reg [63:0] address;
    reg [7:0] index_local0 /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [7:0] index_local1 /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [7:0] index_local2 /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [7:0] index_local3 /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg [63:0] evict_address_q,fill_address_q;
    reg write_op;
    reg [15:0] write_value,response_value;
    reg response_error;
    reg [255:0] line_q;
    reg [12:0] tag_q;
    reg valid_q,dirty_q;
    wire [7:0] index=address[12:5];
    wire [3:0] lane=address[4:1];
    wire in_weights=(req_addr>=WEIGHTS_BASE)&&(req_addr<WEIGHTS_LIMIT);
    wire in_a0=(req_addr>=A0_BASE)&&(req_addr<A0_BASE+64'd65536);
    wire in_a1=(req_addr>=A1_BASE)&&(req_addr<A1_BASE+64'd262144);
    wire address_legal=!req_addr[0]&&(req_addr[63:26]==0)&&
                       (in_a0||in_a1||(in_weights&&!req_write));
    reg [255:0] updated_line;
    always @* begin
        updated_line=line_q;
        updated_line[16*lane+:16]=write_value;
    end
    assign idle=state==IDLE;
    assign invalidate_ready=rst_n&&idle;
    assign req_ready=rst_n&&idle&&!invalidate_valid;
    assign rsp_valid=rst_n&&state==REPLY;
    assign rsp_rdata=response_value;
    assign rsp_error=response_error;
    assign rd_req_valid=rst_n&&state==FILL_REQ;
    assign rd_req_addr=fill_address_q;
    assign rd_req_len=32'd32;
    assign rd_req_id=8'he0;
    assign rd_rsp_ready=rst_n&&state==FILL_WAIT;
    assign wr_req_valid=rst_n&&state==EVICT_REQ;
    assign wr_req_addr=evict_address_q;
    assign wr_req_len=32'd32;
    assign wr_req_id=8'he1;
    assign wr_data_valid=rst_n&&state==EVICT_DATA;
    assign wr_data=line_q;
    assign wr_data_last=1'b1;

    // Dedicated synchronous read/write process; data/tag arrays have no reset.
    always @(posedge clk) begin
        if(rst_n&&state==LOOKUP) begin
            line_q<={lines3[index_local3],lines2[index_local2],
                     lines1[index_local1],lines0[index_local0]};
            tag_q<=tags[index];
            valid_q<=valid_bits[index];dirty_q<=dirty_bits[index];
        end
        if(rst_n&&state==FILL_WAIT&&rd_rsp_valid&&rd_rsp_ready&&
           !rd_rsp_error&&rd_rsp_last&&rd_rsp_id==8'he0) begin
            line_q<=rd_rsp_data;
            lines0[index_local0]<=rd_rsp_data[63:0];
            lines1[index_local1]<=rd_rsp_data[127:64];
            lines2[index_local2]<=rd_rsp_data[191:128];
            lines3[index_local3]<=rd_rsp_data[255:192];
            tags[index]<=address[25:13];
        end
        if(rst_n&&state==ACCESS&&write_op) begin
            lines0[index_local0]<=updated_line[63:0];
            lines1[index_local1]<=updated_line[127:64];
            lines2[index_local2]<=updated_line[191:128];
            lines3[index_local3]<=updated_line[255:192];
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE;valid_bits<=0;dirty_bits<=0;
            address<=0;
            index_local0<=0;index_local1<=0;index_local2<=0;index_local3<=0;
            evict_address_q<=0;fill_address_q<=0;
            write_op<=0;write_value<=0;
            response_value<=0;response_error<=0;
        end else case(state)
            IDLE: begin
                if(invalidate_valid) begin valid_bits<=0;dirty_bits<=0;end
                else if(req_valid&&req_ready) begin
                    address<=req_addr;
                    index_local0<=req_addr[12:5];index_local1<=req_addr[12:5];
                    index_local2<=req_addr[12:5];index_local3<=req_addr[12:5];
                    write_op<=req_write;write_value<=req_wdata;
                    response_error<=!address_legal;response_value<=0;
                    state<=address_legal?LOOKUP:REPLY;
                end
            end
            LOOKUP: state<=CHECK;
            CHECK: begin
                if(valid_q&&tag_q==address[25:13]) state<=ACCESS;
                else begin
                    // Capture requests before asserting valid. This cuts the
                    // tag-RAM-to-client-mux-to-DDR-adapter address path while
                    // preserving the existing CHECK -> request timing.
                    fill_address_q<={address[63:5],5'd0};
                    if(valid_q&&dirty_q) begin
                        evict_address_q<={38'd0,tag_q,index,5'd0};
                        state<=EVICT_REQ;
                    end else state<=FILL_REQ;
                end
            end
            EVICT_REQ: if(wr_req_valid&&wr_req_ready)state<=EVICT_DATA;
            EVICT_DATA: if(wr_data_valid&&wr_data_ready)state<=EVICT_WAIT;
            EVICT_WAIT: if(wr_done_valid)begin
                if(wr_done_error||wr_done_id!=8'he1)begin
                    response_error<=1;state<=REPLY;
                end else begin dirty_bits[index]<=0;state<=FILL_REQ;end
            end
            FILL_REQ: if(rd_req_valid&&rd_req_ready)state<=FILL_WAIT;
            FILL_WAIT: if(rd_rsp_valid&&rd_rsp_ready)begin
                if(rd_rsp_error||!rd_rsp_last||rd_rsp_id!=8'he0)begin
                    valid_bits[index]<=0;dirty_bits[index]<=0;
                    response_error<=1;state<=REPLY;
                end else begin
                    valid_bits[index]<=1;dirty_bits[index]<=0;state<=ACCESS;
                end
            end
            ACCESS: begin
                response_value<=write_op?write_value:line_q[16*lane+:16];
                if(write_op)dirty_bits[index]<=1;
                state<=REPLY;
            end
            REPLY: if(rsp_valid&&rsp_ready)state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
