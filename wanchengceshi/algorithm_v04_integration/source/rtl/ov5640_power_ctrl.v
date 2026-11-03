`timescale 1ns / 1ps
// Stage B: conservative OV5640 power/reset sequencing.
// The camera module supplies its own 24 MHz XCLK; this block generates no XCLK.
module ov5640_power_ctrl #(
    parameter integer SYS_CLK_HZ = 27000000,
    parameter integer RESET_HOLD_CYCLES = SYS_CLK_HZ / 100,   // 10 ms
    parameter integer PWDN_RELEASE_CYCLES = SYS_CLK_HZ / 500, // 2 ms
    parameter integer RESET_RELEASE_WAIT_CYCLES = SYS_CLK_HZ / 40 // 25 ms
)(
    input  wire sys_clk,
    input  wire rst_n,
    output reg  cam_reset_n = 1'b0,
    output reg  cam_pwdn = 1'b1,
    output reg  camera_power_ready = 1'b0
);

    localparam [1:0] ST_RESET_HOLD = 2'd0;
    localparam [1:0] ST_PWDN_LOW   = 2'd1;
    localparam [1:0] ST_WAIT_READY = 2'd2;
    localparam [1:0] ST_READY      = 2'd3;

    reg [1:0]  state = ST_RESET_HOLD;
    reg [31:0] count = 32'd0;

    always @(posedge sys_clk) begin
        if (!rst_n) begin
            state <= ST_RESET_HOLD;
            count <= 32'd0;
        end else case (state)
            ST_RESET_HOLD: begin
                if (count >= RESET_HOLD_CYCLES - 1) begin
                    count <= 32'd0;
                    state <= ST_PWDN_LOW;
                end else begin
                    count <= count + 1'b1;
                end
            end
            ST_PWDN_LOW: begin
                if (count >= PWDN_RELEASE_CYCLES - 1) begin
                    count <= 32'd0;
                    state <= ST_WAIT_READY;
                end else begin
                    count <= count + 1'b1;
                end
            end
            ST_WAIT_READY: begin
                if (count >= RESET_RELEASE_WAIT_CYCLES - 1) begin
                    count <= 32'd0;
                    state <= ST_READY;
                end else begin
                    count <= count + 1'b1;
                end
            end
            default: begin
                count <= 32'd0;
                state <= ST_READY;
            end
        endcase
    end

    always @* begin
        case (state)
            ST_RESET_HOLD: begin
                cam_reset_n = 1'b0;
                cam_pwdn = 1'b1;
                camera_power_ready = 1'b0;
            end
            ST_PWDN_LOW: begin
                cam_reset_n = 1'b0;
                cam_pwdn = 1'b0;
                camera_power_ready = 1'b0;
            end
            ST_WAIT_READY: begin
                cam_reset_n = 1'b1;
                cam_pwdn = 1'b0;
                camera_power_ready = 1'b0;
            end
            default: begin
                cam_reset_n = 1'b1;
                cam_pwdn = 1'b0;
                camera_power_ready = 1'b1;
            end
        endcase
    end
endmodule
