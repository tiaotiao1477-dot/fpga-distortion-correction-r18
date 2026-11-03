// UART byte framing: C3 3C 04 5C sequenceLE32 payload92 CRC16LE.
// CRC-16/CCITT-FALSE covers version,length,sequence,payload; init FFFF.
// Published data remains unchanged until packet_ready acknowledges it.
module v04_config_packet_rx #(parameter integer TIMEOUT_CYCLES=540000)(
    input clk,rst_n,input rx_valid,input [7:0] rx_data,
    output reg packet_valid,input packet_ready,output reg [735:0] payload,
    output reg [31:0] sequence_id,output frame_active,
    output reg [31:0] bad_count,busy_count
);
    localparam MAGIC0=0,MAGIC1=1,VERSION=2,LENGTH=3,DATA=4,CRC0=5,CRC1=6;
    reg [2:0] state;
    reg [6:0] index;
    reg [31:0] timeout_count;
    reg [15:0] crc;
    reg [7:0] crc_low;
    reg [735:0] assembling_payload;
    reg [31:0] assembling_sequence;
    assign frame_active=state!=MAGIC0;
    function [15:0] crc_byte;
        input [15:0] old;input [7:0] b;
        integer k;reg [15:0] c;
        begin c=old^{b,8'd0};
            for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;
            crc_byte=c;
        end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=MAGIC0;packet_valid<=0;index<=0;timeout_count<=0;
            bad_count<=0;busy_count<=0;crc<=16'hffff;crc_low<=0;end
        else begin
            if(packet_valid&&packet_ready)packet_valid<=0;
            if(rx_valid)begin
                timeout_count<=0;
                case(state)
                    MAGIC0:if(rx_data==8'hc3)state<=MAGIC1;
                    MAGIC1:begin
                        if(rx_data==8'h3c)state<=VERSION;
                        else if(rx_data!=8'hc3)state<=MAGIC0;
                    end
                    VERSION:if(rx_data==4)begin crc<=crc_byte(16'hffff,rx_data);state<=LENGTH;end
                        else begin state<=MAGIC0;bad_count<=bad_count+1'b1;end
                    LENGTH:if(rx_data==92)begin crc<=crc_byte(crc,rx_data);index<=0;state<=DATA;end
                        else begin state<=MAGIC0;bad_count<=bad_count+1'b1;end
                    DATA:begin
                        if(index<4)assembling_sequence[index*8+:8]<=rx_data;
                        else assembling_payload[(index-4)*8+:8]<=rx_data;
                        crc<=crc_byte(crc,rx_data);
                        if(index==95)state<=CRC0;else index<=index+1'b1;
                    end
                    CRC0:begin crc_low<=rx_data;state<=CRC1;end
                    CRC1:begin
                        state<=MAGIC0;
                        if({rx_data,crc_low}!=crc)bad_count<=bad_count+1'b1;
                        else if(packet_valid&&!packet_ready)busy_count<=busy_count+1'b1;
                        else begin payload<=assembling_payload;sequence_id<=assembling_sequence;packet_valid<=1;end
                    end
                    default:state<=MAGIC0;
                endcase
            end else if(state!=MAGIC0)begin
                if(timeout_count>=TIMEOUT_CYCLES-1)begin state<=MAGIC0;timeout_count<=0;bad_count<=bad_count+1'b1;end
                else timeout_count<=timeout_count+1'b1;
            end
        end
    end
endmodule
