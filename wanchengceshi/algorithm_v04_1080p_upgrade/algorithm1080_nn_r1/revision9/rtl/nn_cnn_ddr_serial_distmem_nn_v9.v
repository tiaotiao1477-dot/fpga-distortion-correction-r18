`timescale 1ns/1ps
// External-memory serial core. All memory operations complete only after an
// individually accepted response; only the small bias and gray ROMs are local.
module nn_cnn_ddr_serial #(
    parameter BIASES_FILE = "../vendor/nn_v0_1/data/biases.hex",
    parameter GRAY_FILE = "../vendor/nn_v0_1/data/gray_q15.hex",
    parameter [63:0] WEIGHTS_BASE = 64'h03000000,
    parameter [63:0] A0_BASE = 64'h03100000,
    parameter [63:0] A1_BASE = 64'h03200000
)(
    input wire clk,
    input wire rst_n,
    input wire start_valid,
    output wire start_ready,
    input wire [31:0] frame_id,
    input wire signed [15:0] k_fx,
    input wire signed [15:0] k_fy,
    input wire signed [15:0] k_cx,
    input wire signed [15:0] k_cy,
    input wire pixel_valid,
    output wire pixel_ready,
    input wire [7:0] pixel_gray,
    output wire result_valid,
    input wire result_ready,
    output wire [3:0] result_index,
    output wire signed [31:0] result_data,
    output wire [31:0] result_frame_id,
    output wire busy,
    output reg done,
    output reg saturated,
    output reg [1:0] error_code,
    output reg [31:0] cycle_count,
    output wire mem_req_valid,
    input wire mem_req_ready,
    output wire mem_req_write,
    output reg [63:0] mem_req_addr,
    output reg [15:0] mem_req_wdata,
    input wire mem_rsp_valid,
    output wire mem_rsp_ready,
    input wire [15:0] mem_rsp_rdata,
    input wire mem_rsp_error
);
`include "../vendor/nn_v0_1/rtl/nn_model_config.vh"
localparam [4:0] S_IDLE=0,S_LOAD=1,S_LOAD_REQ=2,S_LOAD_WAIT=3,
                 S_LAYER=4,S_BIAS=5,S_BEGIN=6,S_W_REQ=7,S_W_WAIT=8,
                 S_A_REQ=9,S_A_WAIT=10,S_MAC=11,S_WRITE_REQ=12,
                 S_WRITE_WAIT=13,S_OUT=14,S_ERROR=15,
                 S_ROUND_ABS=16,S_ROUND_BIAS=17,S_ROUND_SHIFT=18,
                 S_ROUND_SIGN=19,S_ROUND_CLAMP=20,
                 S_LOAD_PREP=21,S_W_PREP=22,S_A_PREP=23,S_WRITE_PREP=24;
reg [4:0] state;
reg [3:0] layer,out_index;
reg [13:0] load_index;
reg [31:0] active_frame;
reg signed [15:0] active_k[0:3];
reg signed [31:0] results[0:13];
reg signed [63:0] biases[0:317] /* synthesis syn_romstyle = "lut_rom" */;
reg [15:0] gray_lut[0:255] /* synthesis syn_romstyle = "lut_rom" */;
reg [15:0] pending_pixel;
reg signed [15:0] operand_a,operand_w;
reg signed [63:0] bias_q,accumulator;
wire signed [31:0] product = operand_a * operand_w;
integer input_log2,output_log2,input_dim,output_dim,stride,taps,channels_out,shift;
integer weight_channel_base,bias_base,out_channel,out_x,out_y,store_index;
integer tap_index,in_channel,kernel_x,kernel_y;
integer image_x,image_y,read_address;
reg padding_zero;
reg signed [63:0] round_abs,round_biased,round_shifted,round_signed;
reg signed [63:0] store_value_reg;
reg clipped_reg;

initial begin
    $readmemh(BIASES_FILE,biases);
    $readmemh(GRAY_FILE,gray_lut);
end

assign start_ready = state == S_IDLE;
assign pixel_ready = state == S_LOAD;
assign result_valid = state == S_OUT;
assign result_index = out_index;
assign result_data = results[out_index];
assign result_frame_id = active_frame;
assign busy = state != S_IDLE;
assign mem_req_valid = (state == S_LOAD_REQ) || (state == S_W_REQ) ||
                       (state == S_A_REQ) || ((state == S_WRITE_REQ) && layer < 9);
assign mem_req_write = (state == S_LOAD_REQ) || ((state == S_WRITE_REQ) && layer < 9);
assign mem_rsp_ready = (state == S_LOAD_WAIT) || (state == S_W_WAIT) ||
                       (state == S_A_WAIT) || (state == S_WRITE_WAIT);

always @* begin
    // The fixed model configuration uses only stride 1 or 2 on layers 0..7.
    // Preserve those legal products with pass-through/shift logic, not a
    // general integer multiplier. The default config stride is zero.
    if (stride == 2) begin
        image_x = (out_x << 1) + kernel_x - 1;
        image_y = (out_y << 1) + kernel_y - 1;
    end else if (stride == 1) begin
        image_x = out_x + kernel_x - 1;
        image_y = out_y + kernel_y - 1;
    end else begin
        image_x = kernel_x - 1;
        image_y = kernel_y - 1;
    end
    padding_zero = image_x < 0 || image_x >= input_dim || image_y < 0 || image_y >= input_dim;
    read_address = (((in_channel << input_log2) + image_y) << input_log2) + image_x;
    if (layer >= 8) begin
        padding_zero = 0;
        read_address = tap_index;
    end
end

always @(posedge clk) begin
    if (!rst_n) begin
        state<=S_IDLE;layer<=0;out_index<=0;load_index<=0;active_frame<=0;
        done<=0;saturated<=0;error_code<=0;cycle_count<=0;
        input_log2<=0;output_log2<=0;input_dim<=0;output_dim<=0;stride<=0;taps<=0;channels_out<=0;shift<=0;
        weight_channel_base<=0;bias_base<=0;out_channel<=0;out_x<=0;out_y<=0;store_index<=0;
        tap_index<=0;in_channel<=0;kernel_x<=0;kernel_y<=0;accumulator<=0;
        pending_pixel<=0;operand_a<=0;operand_w<=0;bias_q<=0;
        round_abs<=0;round_biased<=0;round_shifted<=0;round_signed<=0;
        store_value_reg<=0;clipped_reg<=0;mem_req_addr<=0;mem_req_wdata<=0;
    end else begin
        done<=0;
        if (busy) cycle_count<=cycle_count+1;
        case (state)
            S_IDLE: if (start_valid) begin
                active_frame<=frame_id;active_k[0]<=k_fx;active_k[1]<=k_fy;active_k[2]<=k_cx;active_k[3]<=k_cy;
                load_index<=0;layer<=0;out_index<=0;cycle_count<=0;saturated<=0;error_code<=0;
                if (k_fx<=0 || k_fy<=0) begin error_code<=1;state<=S_ERROR;end
                else state<=S_LOAD;
            end
            S_LOAD: if (pixel_valid) begin
                pending_pixel<=gray_lut[pixel_gray];state<=S_LOAD_PREP;
            end
            S_LOAD_PREP: begin
                mem_req_addr<=A0_BASE + (64'd2 * load_index);
                mem_req_wdata<=pending_pixel;
                state<=S_LOAD_REQ;
            end
            S_LOAD_REQ: if (mem_req_ready) state<=S_LOAD_WAIT;
            S_LOAD_WAIT: if (mem_rsp_valid) begin
                if (mem_rsp_error) begin error_code<=2;state<=S_ERROR;end
                else if (load_index==16383) state<=S_LAYER;
                else begin load_index<=load_index+1;state<=S_LOAD;end
            end
            S_LAYER: begin
                input_log2<=cfg_in_log2(layer);output_log2<=cfg_out_log2(layer);
                input_dim<=1 << cfg_in_log2(layer);output_dim<=1 << cfg_out_log2(layer);
                stride<=cfg_stride(layer);taps<=cfg_taps(layer);channels_out<=cfg_cout(layer);shift<=cfg_shift(layer);
                weight_channel_base<=cfg_wbase(layer);bias_base<=cfg_bbase(layer);
                out_channel<=0;out_x<=0;out_y<=0;store_index<=0;state<=S_BIAS;
            end
            S_BIAS: begin bias_q<=biases[bias_base+out_channel];state<=S_BEGIN;end
            S_BEGIN: begin
                accumulator<=bias_q;tap_index<=0;in_channel<=0;kernel_x<=0;kernel_y<=0;state<=S_W_PREP;
            end
            S_W_PREP: begin
                mem_req_addr<=WEIGHTS_BASE + (64'd2 * (weight_channel_base + tap_index));
                mem_req_wdata<=16'd0;
                state<=S_W_REQ;
            end
            S_W_REQ: if (mem_req_ready) state<=S_W_WAIT;
            S_W_WAIT: if (mem_rsp_valid) begin
                if (mem_rsp_error) begin error_code<=2;state<=S_ERROR;end
                else begin
                    operand_w<=mem_rsp_rdata;
                    if (padding_zero) begin operand_a<=0;state<=S_MAC;end
                    else if (layer==8 && tap_index>=2048) begin
                        operand_a<=active_k[tap_index-2048];state<=S_MAC;
                    end else state<=S_A_PREP;
                end
            end
            S_A_PREP: begin
                if (layer[0]) mem_req_addr<=A1_BASE + (64'd2 * read_address);
                else mem_req_addr<=A0_BASE + (64'd2 * read_address);
                mem_req_wdata<=16'd0;
                state<=S_A_REQ;
            end
            S_A_REQ: if (mem_req_ready) state<=S_A_WAIT;
            S_A_WAIT: if (mem_rsp_valid) begin
                if (mem_rsp_error) begin error_code<=2;state<=S_ERROR;end
                else begin operand_a<=mem_rsp_rdata;state<=S_MAC;end
            end
            S_MAC: begin
                accumulator<=accumulator+{{32{product[31]}},product};
                if (tap_index==taps-1) state<=S_ROUND_ABS;
                else begin
                    tap_index<=tap_index+1;
                    if (kernel_x==2) begin
                        kernel_x<=0;
                        if (kernel_y==2) begin kernel_y<=0;in_channel<=in_channel+1;end
                        else kernel_y<=kernel_y+1;
                    end else kernel_x<=kernel_x+1;
                    state<=S_W_PREP;
                end
            end
            S_ROUND_ABS: begin
                // Keep 64-bit two's-complement wrap, including -INT64_MIN.
                round_abs<=accumulator<0 ? -accumulator : accumulator;
                state<=S_ROUND_BIAS;
            end
            S_ROUND_BIAS: begin
                if (shift>0) round_biased<=round_abs+(64'sd1 << (shift-1));
                else round_biased<=round_abs;
                state<=S_ROUND_SHIFT;
            end
            S_ROUND_SHIFT: begin
                if (shift>0) round_shifted<=round_biased >>> shift;
                else if (shift<0) round_shifted<=round_biased <<< (-shift);
                else round_shifted<=round_biased;
                state<=S_ROUND_SIGN;
            end
            S_ROUND_SIGN: begin
                round_signed<=accumulator<0 ? -round_shifted : round_shifted;
                state<=S_ROUND_CLAMP;
            end
            S_ROUND_CLAMP: begin
                if (layer<9) begin
                    if (round_signed<0) begin
                        store_value_reg<=64'sd0;
                        clipped_reg<=1'b0;
                    end else if (round_signed>64'sd32767) begin
                        store_value_reg<=64'sd32767;
                        clipped_reg<=1'b1;
                    end else begin
                        store_value_reg<=round_signed;
                        clipped_reg<=1'b0;
                    end
                    state<=S_WRITE_PREP;
                end else begin
                    if (round_signed < -64'sd2147483648) begin
                        store_value_reg<=-64'sd2147483648;
                        clipped_reg<=1'b1;
                    end else if (round_signed > 64'sd2147483647) begin
                        store_value_reg<=64'sd2147483647;
                        clipped_reg<=1'b1;
                    end else begin
                        store_value_reg<=round_signed;
                        clipped_reg<=1'b0;
                    end
                    state<=S_WRITE_REQ;
                end
            end
            S_WRITE_PREP: begin
                if (layer[0]) mem_req_addr<=A0_BASE + (64'd2 * store_index);
                else mem_req_addr<=A1_BASE + (64'd2 * store_index);
                mem_req_wdata<=store_value_reg[15:0];
                state<=S_WRITE_REQ;
            end
            S_WRITE_REQ: if (layer==9) begin
                if (clipped_reg) saturated<=1;
                results[out_channel]<=store_value_reg[31:0];
                store_index<=store_index+1;
                if (out_x==output_dim-1) begin
                    out_x<=0;
                    if (out_y==output_dim-1) begin
                        out_y<=0;
                        if (out_channel==channels_out-1) begin out_index<=0;state<=S_OUT;end
                        else begin out_channel<=out_channel+1;weight_channel_base<=weight_channel_base+taps;state<=S_BIAS;end
                    end else begin out_y<=out_y+1;state<=S_BIAS;end
                end else begin out_x<=out_x+1;state<=S_BIAS;end
            end else if (mem_req_ready) state<=S_WRITE_WAIT;
            S_WRITE_WAIT: if (mem_rsp_valid) begin
                if (mem_rsp_error) begin error_code<=2;state<=S_ERROR;end
                else begin
                    if (clipped_reg) saturated<=1;
                    store_index<=store_index+1;
                    if (out_x==output_dim-1) begin
                        out_x<=0;
                        if (out_y==output_dim-1) begin
                            out_y<=0;
                            if (out_channel==channels_out-1) begin
                                if (layer==9) begin out_index<=0;state<=S_OUT;end
                                else begin layer<=layer+1;state<=S_LAYER;end
                            end else begin out_channel<=out_channel+1;weight_channel_base<=weight_channel_base+taps;state<=S_BIAS;end
                        end else begin out_y<=out_y+1;state<=S_BIAS;end
                    end else begin out_x<=out_x+1;state<=S_BIAS;end
                end
            end
            S_OUT: if (result_ready) begin
                if (out_index==13) begin done<=1;state<=S_IDLE;end
                else out_index<=out_index+1;
            end
            S_ERROR: begin done<=1;state<=S_IDLE;end
            default: state<=S_IDLE;
        endcase
    end
end
endmodule
