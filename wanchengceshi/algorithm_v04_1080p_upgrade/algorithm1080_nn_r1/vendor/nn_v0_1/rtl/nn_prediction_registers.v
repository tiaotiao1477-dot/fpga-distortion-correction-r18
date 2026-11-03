`timescale 1ns/1ps
// Assemble the CNN's ordered 14 words into one atomic, backpressured packet.
module nn_prediction_registers(
    input wire clk,input wire rst_n,
    input wire word_valid,output wire word_ready,
    input wire [3:0] word_index,input wire [31:0] word_data,input wire [31:0] word_frame_id,
    output reg prediction_valid,input wire prediction_ready,
    output reg [447:0] prediction_data,output reg [31:0] prediction_frame_id,
    output reg protocol_error
);
reg [447:0] pending;
reg [3:0] expected_index;
reg [31:0] pending_frame;
assign word_ready=!prediction_valid;
always @(posedge clk) begin
    if (!rst_n) begin
        prediction_valid<=0;prediction_data<=0;prediction_frame_id<=0;
        expected_index<=0;pending<=0;pending_frame<=0;protocol_error<=0;
    end else begin
        if (prediction_valid && prediction_ready) prediction_valid<=0;
        if (word_valid && word_ready) begin
            if (word_index!=expected_index || (expected_index!=0 && word_frame_id!=pending_frame)) begin
                protocol_error<=1;expected_index<=0;
            end else begin
                pending[32*word_index+:32]<=word_data;
                if (word_index==0) pending_frame<=word_frame_id;
                if (word_index==13) begin
                    prediction_data<={word_data,pending[415:0]};prediction_frame_id<=pending_frame;
                    prediction_valid<=1;expected_index<=0;
                end else expected_index<=expected_index+1;
            end
        end
    end
end
endmodule
