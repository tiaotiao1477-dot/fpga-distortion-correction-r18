`timescale 1ns/1ps
// Fixed-length UART command parser and sys_clk-to-core_clk command CDC.
module nn_uart_bridge #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer BAUD_RATE = 115200
)(
    input wire legacy_frame_active,
    input wire sys_clk,
    input wire sys_rst_n,
    input wire uart_rx,
    output wire diagnostic_rx,
    input wire core_clk,
    input wire core_rst_n,
    output wire cmd_valid,
    input wire cmd_ready,
    output wire [7:0] cmd_op,
    output wire [31:0] cmd_seq,
    output wire [31:0] cmd_addr,
    output wire [255:0] cmd_data,
    output reg [31:0] bad_count,
    output reg [31:0] busy_count
);
localparam integer TIMEOUT_CYCLES = CLK_FREQ_HZ / 50;
localparam [2:0] P_IDLE=0,P_MAGIC2=1,P_BODY=2,P_CRC_LO=3,P_CRC_HI=4;
reg [2:0] parser_state;
wire [7:0] rx_byte;
wire rx_byte_valid;
reg [7:0] body [0:41];
reg [5:0] body_index;
reg [15:0] crc16;
reg [7:0] received_crc_low;
reg [31:0] timeout_count;
reg pending_valid;
reg [7:0] pending_op;
reg [31:0] pending_seq,pending_addr;
reg [255:0] pending_data;
wire cdc_src_ready;
wire cdc_dst_valid;
wire [327:0] cdc_dst_data;
wire [327:0] cdc_src_data = {pending_op,pending_seq,pending_addr,pending_data};

camera_diag_uart_rx #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) uart_capture(
    .clk(sys_clk),.rst_n(sys_rst_n),.rxd(uart_rx),.data(rx_byte),.data_valid(rx_byte_valid)
);
assign diagnostic_rx = (parser_state == P_IDLE) ? uart_rx : 1'b1;
assign cmd_valid = cdc_dst_valid;
assign {cmd_op,cmd_seq,cmd_addr,cmd_data} = cdc_dst_data;
v04_config_cdc #(.WIDTH(328)) command_cdc(
    .src_clk(sys_clk),.src_rst_n(sys_rst_n),.src_valid(pending_valid),.src_ready(cdc_src_ready),.src_data(cdc_src_data),
    .dst_clk(core_clk),.dst_rst_n(core_rst_n),.dst_valid(cdc_dst_valid),.dst_ready(cmd_ready),.dst_data(cdc_dst_data)
);

function [15:0] crc16_next;
    input [15:0] current_crc;
    input [7:0] next_byte;
    reg [15:0] c;
    integer bit_no;
    begin
        c = current_crc ^ {next_byte,8'h00};
        for (bit_no=0;bit_no<8;bit_no=bit_no+1)
            if (c[15]) c = {c[14:0],1'b0} ^ 16'h1021;
            else c = {c[14:0],1'b0};
        crc16_next = c;
    end
endfunction

always @(posedge sys_clk) begin
    if (!sys_rst_n) begin
        parser_state<=P_IDLE;body_index<=0;crc16<=16'hffff;received_crc_low<=0;
        timeout_count<=0;pending_valid<=0;pending_op<=0;pending_seq<=0;pending_addr<=0;pending_data<=0;
        bad_count<=0;busy_count<=0;
    end else begin
        if (pending_valid && cdc_src_ready) pending_valid<=0;
        if (parser_state == P_IDLE) timeout_count<=0;
        else if (rx_byte_valid) timeout_count<=0;
        else if (timeout_count >= TIMEOUT_CYCLES-1) begin
            parser_state<=P_IDLE;timeout_count<=0;
        end else timeout_count<=timeout_count+1'b1;

        if (rx_byte_valid) begin
            case (parser_state)
                P_IDLE: if (!legacy_frame_active && rx_byte==8'hc5) begin parser_state<=P_MAGIC2;timeout_count<=0;end
                P_MAGIC2: begin
                    timeout_count<=0;
                    if (rx_byte==8'h5c) begin
                        parser_state<=P_BODY;body_index<=0;crc16<=16'hffff;
                    end else if (rx_byte==8'hc5) parser_state<=P_MAGIC2;
                    else parser_state<=P_IDLE;
                end
                P_BODY: begin
                    body[body_index]<=rx_byte;
                    crc16<=crc16_next(crc16,rx_byte);
                    if (body_index==41) parser_state<=P_CRC_LO;
                    else body_index<=body_index+1'b1;
                end
                P_CRC_LO: begin received_crc_low<=rx_byte;parser_state<=P_CRC_HI;end
                P_CRC_HI: begin
                    parser_state<=P_IDLE;
                    if (body[0]!=8'h01 || received_crc_low!=crc16[7:0] || rx_byte!=crc16[15:8]) begin
                        bad_count<=bad_count+1'b1;
                    end else if (pending_valid || !cdc_src_ready) begin
                        busy_count<=busy_count+1'b1;
                    end else begin
                        pending_op<=body[1];
                        pending_seq<={body[5],body[4],body[3],body[2]};
                        pending_addr<={body[9],body[8],body[7],body[6]};
                        pending_data<={body[41],body[40],body[39],body[38],body[37],body[36],body[35],body[34],
                                       body[33],body[32],body[31],body[30],body[29],body[28],body[27],body[26],
                                       body[25],body[24],body[23],body[22],body[21],body[20],body[19],body[18],
                                       body[17],body[16],body[15],body[14],body[13],body[12],body[11],body[10]};
                        pending_valid<=1'b1;
                    end
                end
                default: parser_state<=P_IDLE;
            endcase
        end
    end
end
endmodule
