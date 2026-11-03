`timescale 1ns/1ps
// DDR-backed wrapper preserving the original atomic 14-word prediction packet.
module nn_geometry_predictor_ddr #(
    parameter BIASES_FILE="../vendor/nn_v0_1/data/biases.hex",
    parameter GRAY_FILE="../vendor/nn_v0_1/data/gray_q15.hex",
    parameter [63:0] WEIGHTS_BASE=64'h03000000,
    parameter [63:0] A0_BASE=64'h03100000,
    parameter [63:0] A1_BASE=64'h03200000
)(
    input wire clk,input wire rst_n,
    input wire start_valid,output wire start_ready,input wire [31:0] frame_id,
    input wire signed [15:0] k_fx,input wire signed [15:0] k_fy,input wire signed [15:0] k_cx,input wire signed [15:0] k_cy,
    input wire pixel_valid,output wire pixel_ready,input wire [7:0] pixel_gray,
    output wire prediction_valid,input wire prediction_ready,
    output wire [447:0] prediction_data,output wire [31:0] prediction_frame_id,
    output wire busy,output wire compute_done,output wire saturated,
    output wire [1:0] error_code,output wire protocol_error,output wire [31:0] cycle_count,
    output wire mem_req_valid,input wire mem_req_ready,output wire mem_req_write,
    output wire [63:0] mem_req_addr,output wire [15:0] mem_req_wdata,
    input wire mem_rsp_valid,output wire mem_rsp_ready,input wire [15:0] mem_rsp_rdata,input wire mem_rsp_error
);
wire core_ready,core_busy,word_valid,word_ready;
wire [3:0] word_index;
wire [31:0] word_data,word_frame_id;
assign start_ready=core_ready && !prediction_valid;
assign busy=core_busy || prediction_valid;
nn_cnn_ddr_serial #(
    .BIASES_FILE(BIASES_FILE),.GRAY_FILE(GRAY_FILE),
    .WEIGHTS_BASE(WEIGHTS_BASE),.A0_BASE(A0_BASE),.A1_BASE(A1_BASE)
) core(
    .clk(clk),.rst_n(rst_n),.start_valid(start_valid && !prediction_valid),.start_ready(core_ready),.frame_id(frame_id),
    .k_fx(k_fx),.k_fy(k_fy),.k_cx(k_cx),.k_cy(k_cy),.pixel_valid(pixel_valid),.pixel_ready(pixel_ready),.pixel_gray(pixel_gray),
    .result_valid(word_valid),.result_ready(word_ready),.result_index(word_index),.result_data(word_data),.result_frame_id(word_frame_id),
    .busy(core_busy),.done(compute_done),.saturated(saturated),.error_code(error_code),.cycle_count(cycle_count),
    .mem_req_valid(mem_req_valid),.mem_req_ready(mem_req_ready),.mem_req_write(mem_req_write),
    .mem_req_addr(mem_req_addr),.mem_req_wdata(mem_req_wdata),.mem_rsp_valid(mem_rsp_valid),.mem_rsp_ready(mem_rsp_ready),
    .mem_rsp_rdata(mem_rsp_rdata),.mem_rsp_error(mem_rsp_error)
);
nn_prediction_registers packet(
    .clk(clk),.rst_n(rst_n),.word_valid(word_valid),.word_ready(word_ready),.word_index(word_index),.word_data(word_data),.word_frame_id(word_frame_id),
    .prediction_valid(prediction_valid),.prediction_ready(prediction_ready),.prediction_data(prediction_data),.prediction_frame_id(prediction_frame_id),.protocol_error(protocol_error)
);
endmodule
