// Reuse the existing, independently verified UART receiver; TX diagnostics stay
// on their original transmitter. CRC-qualified complete packets cross as a unit.
module v04_uart_config_bridge #(parameter integer CLK_FREQ_HZ=27000000,BAUD_RATE=115200)(
    input sys_clk,sys_rst_n,input uart_rx,output diagnostic_rx,output legacy_frame_active,
    input ddr_clk,ddr_rst_n,output cfg_valid,input cfg_ready,
    output [735:0] cfg_payload,output [31:0] cfg_seq,
    output [31:0] bad_count,busy_count
);
    wire [7:0] byte_data;wire byte_valid;
    wire packet_valid,packet_ready,frame_active;
    wire [735:0] payload;wire [31:0] sequence_id;
    camera_diag_uart_rx #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) rx(
        .clk(sys_clk),.rst_n(sys_rst_n),.rxd(uart_rx),.data(byte_data),.data_valid(byte_valid));
    v04_config_packet_rx #(.TIMEOUT_CYCLES(CLK_FREQ_HZ/50)) parser(
        .clk(sys_clk),.rst_n(sys_rst_n),.rx_valid(byte_valid),.rx_data(byte_data),
        .packet_valid(packet_valid),.packet_ready(packet_ready),.payload(payload),.sequence_id(sequence_id),
        .frame_active(frame_active),.bad_count(bad_count),.busy_count(busy_count));
    assign diagnostic_rx=frame_active?1'b1:uart_rx;
    assign legacy_frame_active=frame_active;
    v04_config_cdc #(.WIDTH(768)) cdc(
        .src_clk(sys_clk),.src_rst_n(sys_rst_n),.src_valid(packet_valid),.src_ready(packet_ready),
        .src_data({sequence_id,payload}),.dst_clk(ddr_clk),.dst_rst_n(ddr_rst_n),
        .dst_valid(cfg_valid),.dst_ready(cfg_ready),.dst_data({cfg_seq,cfg_payload}));
endmodule
