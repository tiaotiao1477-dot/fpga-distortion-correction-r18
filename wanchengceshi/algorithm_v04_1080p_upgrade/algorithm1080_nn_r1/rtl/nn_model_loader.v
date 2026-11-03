`timescale 1ns/1ps
// Fixed-region model loader and guarded model/control state machine.
module nn_model_loader #(
    parameter [31:0] EXPECTED_CRC32 = 32'h00000000
)(
    input wire clk,
    input wire rst_n,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [7:0] cmd_op,
    input wire [31:0] cmd_seq,
    input wire [31:0] cmd_addr,
    input wire [255:0] cmd_data,
    input wire inference_idle,
    input wire cache_idle,
    output reg model_loaded,
    output reg nn_enable,
    output reg [63:0] k_packed,
    output reg k_valid,
    output wire cache_invalidate_valid,
    input wire cache_invalidate_ready,
    output wire wr_req_valid,
    input wire wr_req_ready,
    output reg [63:0] wr_req_addr,
    output wire [31:0] wr_req_len,
    output wire [7:0] wr_req_id,
    output wire wr_data_valid,
    input wire wr_data_ready,
    output reg [255:0] wr_data,
    output wire wr_last,
    input wire wr_done_valid,
    input wire [7:0] wr_done_id,
    input wire wr_done_error,
    output reg [31:0] last_completed_seq,
    output reg [7:0] last_error,
    output reg [31:0] loaded_bytes,
    output reg [31:0] computed_crc32
);
localparam [31:0] WEIGHTS_BASE = 32'h03000000;
localparam [31:0] MODEL_BYTES = 32'd601632;
localparam [31:0] CRC_PAYLOAD_BYTES = 32'd601616;
localparam [2:0] L_IDLE=0,L_INVALIDATE=1,L_WR_REQ=2,L_WR_DATA=3,L_WR_DONE=4,L_CRC=5;
localparam [7:0] E_BAD_CMD=8'h01,E_NOT_IDLE=8'h02,E_SEQUENCE=8'h03,
                 E_RANGE=8'h04,E_MEMORY=8'h05,E_STATE=8'h06,E_CRC=8'h07;
reg [2:0] state;
reg session_started;
reg [31:0] begin_seq,write_seq,crc32_state;
reg [31:0] crc_work;
reg [4:0] crc_byte_index;
reg [5:0] write_crc_byte_count;
reg [255:0] write_data_buf;
reg [31:0] write_addr_buf;
wire [5:0] crc_bytes_this_write = (loaded_bytes >= CRC_PAYLOAD_BYTES) ? 6'd0 :
    ((CRC_PAYLOAD_BYTES-loaded_bytes >= 32) ? 6'd32 : CRC_PAYLOAD_BYTES-loaded_bytes);
wire [31:0] crc_next_byte = crc32_update_byte(crc_work,write_data_buf[crc_byte_index*8+:8]);
assign cmd_ready = state == L_IDLE;
assign cache_invalidate_valid = state == L_INVALIDATE;
assign wr_req_valid = state == L_WR_REQ;
assign wr_req_len = 32'd32;
assign wr_req_id = 8'he2;
assign wr_data_valid = state == L_WR_DATA;
assign wr_last = wr_data_valid;

function [31:0] crc32_update_byte;
    input [31:0] current_crc;
    input [7:0] data_byte;
    reg [31:0] c;
    integer bit_no;
    begin
        c=current_crc ^ data_byte;
        for (bit_no=0;bit_no<8;bit_no=bit_no+1)
            if (c[0]) c=(c>>1) ^ 32'hedb88320;
            else c=c>>1;
        crc32_update_byte=c;
    end
endfunction

always @(posedge clk) begin
    if (!rst_n) begin
        state<=L_IDLE;session_started<=0;begin_seq<=0;write_seq<=0;
        model_loaded<=0;nn_enable<=0;k_packed<=0;k_valid<=0;
        wr_req_addr<=0;wr_data<=0;write_data_buf<=0;write_addr_buf<=0;
        crc_work<=0;crc_byte_index<=0;write_crc_byte_count<=0;
        last_completed_seq<=0;last_error<=0;loaded_bytes<=0;crc32_state<=32'hffffffff;computed_crc32<=0;
    end else begin
        case (state)
            L_IDLE: if (cmd_valid && cmd_ready) begin
                last_error<=0;
                case (cmd_op)
                    8'd1: begin
                        if (!inference_idle || !cache_idle) last_error<=E_NOT_IDLE;
                        else begin
                            model_loaded<=0;nn_enable<=0;k_packed<=0;k_valid<=0;
                            loaded_bytes<=0;crc32_state<=32'hffffffff;computed_crc32<=0;
                            session_started<=0;begin_seq<=cmd_seq;state<=L_INVALIDATE;
                        end
                    end
                    8'd2: begin
                        if (!session_started || nn_enable) last_error<=E_STATE;
                        else if (cmd_seq!=last_completed_seq+1'b1) last_error<=E_SEQUENCE;
                        else if (!inference_idle || !cache_idle) last_error<=E_NOT_IDLE;
                        else if (loaded_bytes>=MODEL_BYTES || cmd_addr!=(WEIGHTS_BASE+loaded_bytes)) last_error<=E_RANGE;
                        else begin
                            model_loaded<=0;nn_enable<=0;
                            write_seq<=cmd_seq;write_addr_buf<=cmd_addr;write_data_buf<=cmd_data;
                            wr_req_addr<={32'd0,cmd_addr};wr_data<=cmd_data;last_error<=0;state<=L_WR_REQ;
                        end
                    end
                    8'd3: begin
                        if (cmd_seq!=last_completed_seq+1'b1) last_error<=E_SEQUENCE;
                        else if (!session_started || loaded_bytes!=MODEL_BYTES) last_error<=E_STATE;
                        else if (computed_crc32!=EXPECTED_CRC32) last_error<=E_CRC;
                        else begin model_loaded<=1;nn_enable<=0;last_completed_seq<=cmd_seq;last_error<=0;end
                    end
                    8'd4: begin
                        if (cmd_seq!=last_completed_seq+1'b1) last_error<=E_SEQUENCE;
                        else if (cmd_data[0] && (!model_loaded || !k_valid)) last_error<=E_STATE;
                        else begin
                            // This is observation enable only; no video geometry is applied here.
                            nn_enable<=cmd_data[0];last_completed_seq<=cmd_seq;last_error<=0;
                        end
                    end
                    8'd5: begin
                        if (cmd_seq!=last_completed_seq+1'b1) last_error<=E_SEQUENCE;
                        else if (!inference_idle || !cache_idle) last_error<=E_NOT_IDLE;
                        else if ($signed(cmd_data[15:0])<=0 || $signed(cmd_data[31:16])<=0) last_error<=E_BAD_CMD;
                        else begin
                            k_packed<=cmd_data[63:0];k_valid<=1;nn_enable<=0;last_completed_seq<=cmd_seq;last_error<=0;
                        end
                    end
                    default: last_error<=E_BAD_CMD;
                endcase
            end
            L_INVALIDATE: if (cache_invalidate_ready) begin
                last_completed_seq<=begin_seq;session_started<=1;state<=L_IDLE;
            end
            L_WR_REQ: if (wr_req_ready) state<=L_WR_DATA;
            L_WR_DATA: if (wr_data_ready) state<=L_WR_DONE;
            L_WR_DONE: if (wr_done_valid) begin
                if (wr_done_id!=8'he2 || wr_done_error) begin
                    last_error<=E_MEMORY;model_loaded<=0;nn_enable<=0;state<=L_IDLE;
                end else begin
                    crc_work<=crc32_state;crc_byte_index<=0;
                    write_crc_byte_count<=crc_bytes_this_write;state<=L_CRC;
                end
            end
            L_CRC: begin
                if (crc_byte_index < write_crc_byte_count) crc_work<=crc_next_byte;
                if (crc_byte_index==31) begin
                    if (crc_byte_index < write_crc_byte_count) begin
                        crc32_state<=crc_next_byte;computed_crc32<=~crc_next_byte;
                    end else begin
                        crc32_state<=crc_work;computed_crc32<=~crc_work;
                    end
                    loaded_bytes<=loaded_bytes+32;last_completed_seq<=write_seq;last_error<=0;state<=L_IDLE;
                end else crc_byte_index<=crc_byte_index+1'b1;
            end
            default: state<=L_IDLE;
        endcase
    end
end
endmodule
