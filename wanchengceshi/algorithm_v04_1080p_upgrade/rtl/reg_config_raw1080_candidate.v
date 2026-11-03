`timescale 1ns / 1ps

// OV5640 SCCB writer with explicit table-valid handling.
// The legacy configuration block advanced through sparse table holes and
// wrote 0xFFFF to the sensor.  This replacement skips invalid entries, keeps
// START asserted until the transaction really ends. SCCB has no ACK; the
// sensor's configuration is confirmed by register readback after the table.
module reg_config_raw1080_candidate #(
    parameter integer SYS_CLK_HZ = 27_000_000,
    parameter integer I2C_HZ     = 20_000,
    parameter integer LAST_INDEX  = 359
) (
    input        clk_25M,
    input        camera_rstn,
    input        initial_en,
    output reg   reg_conf_done,
    output       i2c_sclk,
    inout        i2c_sdat,
    output reg   clock_20k,
    output reg [8:0] reg_index,
    output reg   camera_ack_error,
    output reg [15:0] ack_error_count,
    output reg [8:0] last_error_index
);
    localparam integer HALF_DIV = SYS_CLK_HZ / (I2C_HZ * 2);
    localparam [2:0] ST_WAIT    = 3'd0;
    localparam [2:0] ST_SETTLE  = 3'd1;
    localparam [2:0] ST_LOAD    = 3'd2;
    localparam [2:0] ST_WRITE   = 3'd3;
    localparam [2:0] ST_RELEASE = 3'd4;
    localparam [2:0] ST_DONE    = 3'd5;
    localparam [2:0] ST_RESET_WAIT = 3'd6;

    reg [15:0] div_count;
    reg [2:0] state;
    reg [5:0] settle_count;
    reg [7:0] reset_wait_count;
    reg sccb_start;
    wire [31:0] i2c_data;
    wire [23:0] reg_data;
    wire reg_valid;
    wire tr_end;

    ov5640_reg_table_raw1080_candidate u_reg_table (
        .index(reg_index), .reg_data(reg_data), .reg_valid(reg_valid)
    );

    assign i2c_data = {8'h78, reg_data};

    i2c_com u_i2c_com (
        .clock_i2c(clock_20k), .camera_rstn(camera_rstn), .ack(),
        .i2c_data(i2c_data), .start(sccb_start), .tr_end(tr_end),
        .i2c_sclk(i2c_sclk), .i2c_sdat(i2c_sdat)
    );

    always @(posedge clk_25M or negedge camera_rstn) begin
        if (!camera_rstn) begin
            div_count <= 16'd0;
            clock_20k <= 1'b0;
        end else if (div_count == HALF_DIV - 1) begin
            div_count <= 16'd0;
            clock_20k <= ~clock_20k;
        end else begin
            div_count <= div_count + 1'b1;
        end
    end

    always @(posedge clock_20k or negedge camera_rstn) begin
        if (!camera_rstn) begin
            state <= ST_WAIT;
            settle_count <= 6'd0;
            reset_wait_count <= 8'd0;
            sccb_start <= 1'b0;
            reg_conf_done <= 1'b0;
            reg_index <= 9'd0;
            camera_ack_error <= 1'b0;
            ack_error_count <= 16'd0;
            last_error_index <= 9'd0;
        end else begin
            case (state)
                ST_WAIT: begin
                    reg_conf_done <= 1'b0;
                    sccb_start <= 1'b0;
                    if (initial_en) begin
                        settle_count <= 6'd0;
                        state <= ST_SETTLE;
                    end
                end

                ST_SETTLE: begin
                    if (settle_count == 6'd31) begin
                        reg_index <= 9'd0;
                        state <= ST_LOAD;
                    end else begin
                        settle_count <= settle_count + 1'b1;
                    end
                end

                ST_LOAD: begin
                    if (reg_index > LAST_INDEX) begin
                        state <= ST_DONE;
                    end else if (!reg_valid) begin
                        reg_index <= reg_index + 1'b1;
                    end else begin
                        sccb_start <= 1'b1;
                        state <= ST_WRITE;
                    end
                end

                ST_WRITE: begin
                    // i2c_com reports tr_end while the last cycle is active.
                    // Keep start high until that event, then release it.
                    if (tr_end) begin
                        sccb_start <= 1'b0;
                        state <= ST_RELEASE;
                    end
                end

                ST_RELEASE: begin
                    // Wait for i2c_com to return tr_end low before loading
                    // the next table word.
                    if (!tr_end) begin
                        if (reg_index == 9'd1) begin
                            // OV5640 software reset is not instantaneous;
                            // wait >5 ms before issuing the next SCCB write.
                            reset_wait_count <= 8'd0;
                            state <= ST_RESET_WAIT;
                        end else begin
                            reg_index <= reg_index + 1'b1;
                            state <= ST_LOAD;
                        end
                    end
                end

                ST_RESET_WAIT: begin
                    if (reset_wait_count == 8'd127) begin
                        reg_index <= reg_index + 1'b1;
                        state <= ST_LOAD;
                    end else begin
                        reset_wait_count <= reset_wait_count + 1'b1;
                    end
                end

                ST_DONE: begin
                    sccb_start <= 1'b0;
                    reg_conf_done <= 1'b1;
                end

                default: state <= ST_WAIT;
            endcase
        end
    end
endmodule
