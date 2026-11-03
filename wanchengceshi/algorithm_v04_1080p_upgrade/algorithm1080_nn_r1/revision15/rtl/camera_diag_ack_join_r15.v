`timescale 1ns/1ps
// Wait for both held snapshot sources. Only the first stage of each async
// toggle synchronizer is a CDC endpoint; the local stages and join are timed.
module camera_diag_ack_join_r15 (
    input wire sys_clk,
    input wire sys_rst_n,
    input wire request_toggle_sys,
    input wire nn_ack_toggle_async,
    input wire display_gamma_ack_toggle_async,
    output reg combined_ack_toggle_sys
);
    reg nn_ack_meta_sys, nn_ack_sync2_sys, nn_ack_sync3_sys;
    reg display_gamma_ack_meta_sys, display_gamma_ack_sync2_sys, display_gamma_ack_sync3_sys;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            nn_ack_meta_sys <= 1'b0;
            nn_ack_sync2_sys <= 1'b0;
            nn_ack_sync3_sys <= 1'b0;
            display_gamma_ack_meta_sys <= 1'b0;
            display_gamma_ack_sync2_sys <= 1'b0;
            display_gamma_ack_sync3_sys <= 1'b0;
            combined_ack_toggle_sys <= 1'b0;
        end else begin
            nn_ack_meta_sys <= nn_ack_toggle_async;
            nn_ack_sync2_sys <= nn_ack_meta_sys;
            nn_ack_sync3_sys <= nn_ack_sync2_sys;
            display_gamma_ack_meta_sys <= display_gamma_ack_toggle_async;
            display_gamma_ack_sync2_sys <= display_gamma_ack_meta_sys;
            display_gamma_ack_sync3_sys <= display_gamma_ack_sync2_sys;
            if (nn_ack_sync3_sys == request_toggle_sys &&
                display_gamma_ack_sync3_sys == request_toggle_sys)
                combined_ack_toggle_sys <= request_toggle_sys;
        end
    end
endmodule
