`timescale 1ns / 1ps

// Read back OV5640 identity and DVP clock/output control registers after the
// write-only initialization table has completed. SIO_C is an actively driven
// logic level; SIO_D is actively driven for master data and released while
// the sensor supplies read data or the protocol's don't-care bit.
module ov5640_sccb_readback #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer SCCB_HZ     = 20_000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        sda_in,
    input  wire        awb_req_toggle,
    output reg         scl_out,
    output reg         sda_out,
    output reg         sda_oe,
    output reg         busy,
    output reg         done,
    output reg         error,
    output reg  [15:0] chip_id,
    output reg  [7:0]  clock_enable,
    output reg  [7:0]  pad_output_enable,
    output reg  [7:0]  system_control,
    output reg  [7:0]  pad_output_enable2,
    output reg  [7:0]  polarity_control,
    output reg  [7:0]  format_control,
    output reg  [7:0]  isp_format_control,
    output reg  [15:0] output_width,
    output reg  [15:0] output_height,
    // Bytes in this vector, from least to most significant, are:
    // 3800..3807, 380C..380F, 3814, 3815, 503D, and 3035.
    output reg  [127:0] timing_readback,
    // Bytes in this vector, from least to most significant, are:
    // 5000, 5001, 5800, and 583D.
    output reg  [31:0]  lsc_readback,
    output reg          awb_done_toggle,
    output reg          awb_valid,
    output reg  [2:0]   awb_error_code,
    output reg  [79:0]  awb_data
);
    localparam integer HALF_PERIOD_CYCLES =
        (CLK_FREQ_HZ + (SCCB_HZ * 2) - 1) / (SCCB_HZ * 2);
    localparam [4:0]
        ST_IDLE             = 5'd0,
        ST_START_IDLE       = 5'd1,
        ST_START_ASSERT     = 5'd2,
        ST_START_HOLD       = 5'd3,
        ST_TX_SETUP         = 5'd4,
        ST_TX_HIGH          = 5'd5,
        ST_TX_FALL          = 5'd6,
        ST_DC_SETUP         = 5'd7,
        ST_DC_HIGH          = 5'd8,
        ST_DC_CLOCK         = 5'd9,
        ST_RESTART_IDLE     = 5'd10,
        ST_RESTART_ASSERT   = 5'd11,
        ST_RESTART_HOLD     = 5'd12,
        ST_READ_RAISE       = 5'd13,
        ST_READ_SAMPLE      = 5'd14,
        ST_READ_DC_SETUP    = 5'd15,
        ST_READ_DC_HIGH     = 5'd16,
        ST_READ_DC_FALL     = 5'd17,
        ST_STOP_SETUP       = 5'd18,
        ST_STOP_HIGH        = 5'd19,
        ST_STOP_RELEASE     = 5'd20;

    reg [31:0] divider_count;
    reg [4:0] state;
    reg started;
    reg [5:0] register_index;
    reg [1:0] tx_stage;
    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg [2:0] bit_index;
    reg initialization_complete;
    reg awb_req_meta;
    reg awb_req_sync;
    reg awb_req_seen;
    reg awb_active;
    reg awb_transaction_toggle;
    reg [79:0] awb_capture_buffer;
    wire tick = (divider_count == HALF_PERIOD_CYCLES - 1);
    wire [15:0] selected_register;

    function [15:0] register_address(input [5:0] index);
        begin
            case (index)
                5'd0:  register_address = 16'h300A;
                5'd1:  register_address = 16'h300B;
                5'd2:  register_address = 16'h3007;
                5'd3:  register_address = 16'h3017;
                5'd4:  register_address = 16'h3008;
                5'd5:  register_address = 16'h3018;
                5'd6:  register_address = 16'h4740;
                5'd7:  register_address = 16'h4300;
                5'd8:  register_address = 16'h501F;
                5'd9:  register_address = 16'h3808;
                5'd10: register_address = 16'h3809;
                5'd11: register_address = 16'h380A;
                5'd12: register_address = 16'h380B;
                5'd13: register_address = 16'h3800;
                5'd14: register_address = 16'h3801;
                5'd15: register_address = 16'h3802;
                5'd16: register_address = 16'h3803;
                5'd17: register_address = 16'h3804;
                5'd18: register_address = 16'h3805;
                5'd19: register_address = 16'h3806;
                5'd20: register_address = 16'h3807;
                5'd21: register_address = 16'h380C;
                5'd22: register_address = 16'h380D;
                5'd23: register_address = 16'h380E;
                5'd24: register_address = 16'h380F;
                5'd25: register_address = 16'h3814;
                5'd26: register_address = 16'h3815;
                5'd27: register_address = 16'h503D;
                5'd28: register_address = 16'h3035;
                5'd29: register_address = 16'h5000;
                5'd30: register_address = 16'h5001;
                5'd31: register_address = 16'h5800;
                6'd32: register_address = 16'h583D;
                6'd33: register_address = 16'h3406;
                6'd34: register_address = 16'h51A5;
                6'd35: register_address = 16'h51A6;
                6'd36: register_address = 16'h51A7;
                6'd37: register_address = 16'h519F;
                6'd38: register_address = 16'h51A0;
                6'd39: register_address = 16'h51A1;
                6'd40: register_address = 16'h51A2;
                6'd41: register_address = 16'h51A3;
                default: register_address = 16'h51A4;
            endcase
        end
    endfunction

    assign selected_register = register_address(register_index);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            divider_count <= 32'd0;
        else if (tick)
            divider_count <= 32'd0;
        else
            divider_count <= divider_count + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= ST_IDLE;
            started           <= 1'b0;
            register_index    <= 6'd0;
            tx_stage          <= 2'd0;
            tx_byte           <= 8'd0;
            rx_byte           <= 8'd0;
            bit_index         <= 3'd0;
            scl_out           <= 1'b1;
            sda_out           <= 1'b1;
            sda_oe            <= 1'b0;
            busy              <= 1'b0;
            done              <= 1'b0;
            error             <= 1'b0;
            chip_id           <= 16'd0;
            clock_enable      <= 8'd0;
            pad_output_enable <= 8'd0;
            system_control    <= 8'd0;
            pad_output_enable2 <= 8'd0;
            polarity_control   <= 8'd0;
            format_control     <= 8'd0;
            isp_format_control <= 8'd0;
            output_width       <= 16'd0;
            output_height      <= 16'd0;
            timing_readback    <= 128'd0;
            lsc_readback       <= 32'd0;
            initialization_complete <= 1'b0;
            awb_req_meta       <= 1'b0;
            awb_req_sync       <= 1'b0;
            awb_req_seen       <= 1'b0;
            awb_active         <= 1'b0;
            awb_transaction_toggle <= 1'b0;
            awb_capture_buffer <= 80'd0;
            awb_done_toggle    <= 1'b0;
            awb_valid          <= 1'b0;
            awb_error_code     <= 3'd0;
            awb_data           <= 80'd0;
        end else begin
            awb_req_meta <= awb_req_toggle;
            awb_req_sync <= awb_req_meta;

            if (awb_req_sync != awb_req_seen) begin
                awb_req_seen <= awb_req_sync;
                awb_data <= 80'd0;
                awb_valid <= 1'b0;
                awb_error_code <= 3'd0;
                if (busy) begin
                    awb_error_code <= 3'd1;
                    awb_done_toggle <= awb_req_sync;
                end else if (!initialization_complete) begin
                    awb_error_code <= 3'd4;
                    awb_done_toggle <= awb_req_sync;
                end else if (sda_in !== 1'b1) begin
                    // SCCB's ninth bit is don't-care, so a NACK cannot be
                    // distinguished from the bus pull-up. Only report an
                    // observable stuck/invalid idle bus here.
                    awb_error_code <= 3'd2;
                    awb_done_toggle <= awb_req_sync;
                end else begin
                    awb_active <= 1'b1;
                    awb_transaction_toggle <= awb_req_sync;
                    awb_capture_buffer <= 80'd0;
                    register_index <= 6'd33;
                    busy <= 1'b1;
                    state <= ST_START_IDLE;
                end
            end

            if ((state == ST_IDLE) && start && !started) begin
                started        <= 1'b1;
                busy           <= 1'b1;
                initialization_complete <= 1'b0;
                done           <= 1'b0;
                error          <= 1'b0;
                register_index <= 6'd0;
                state          <= ST_START_IDLE;
            end else if (tick && busy) begin
                case (state)
                    ST_START_IDLE: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b1;
                        state <= ST_START_ASSERT;
                    end
                    ST_START_ASSERT: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b0;
                        sda_oe <= 1'b1;
                        state <= ST_START_HOLD;
                    end
                    ST_START_HOLD: begin
                        scl_out <= 1'b0;
                        sda_out <= 1'b0;
                        sda_oe <= 1'b1;
                        tx_byte <= 8'h78;
                        tx_stage <= 2'd0;
                        bit_index <= 3'd7;
                        state <= ST_TX_SETUP;
                    end
                    ST_TX_SETUP: begin
                        scl_out <= 1'b0;
                        sda_out <= tx_byte[bit_index];
                        sda_oe <= 1'b1;
                        state <= ST_TX_HIGH;
                    end
                    ST_TX_HIGH: begin
                        scl_out <= 1'b1;
                        state <= ST_TX_FALL;
                    end
                    ST_TX_FALL: begin
                        scl_out <= 1'b0;
                        if (bit_index == 3'd0)
                            state <= ST_DC_SETUP;
                        else begin
                            bit_index <= bit_index - 1'b1;
                            state <= ST_TX_SETUP;
                        end
                    end
                    ST_DC_SETUP: begin
                        scl_out <= 1'b0;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b0;
                        state <= ST_DC_HIGH;
                    end
                    ST_DC_HIGH: begin
                        scl_out <= 1'b1;
                        state <= ST_DC_CLOCK;
                    end
                    ST_DC_CLOCK: begin
                        scl_out <= 1'b0;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b0;
                        case (tx_stage)
                            2'd0: begin
                                tx_stage <= 2'd1;
                                tx_byte <= selected_register[15:8];
                                bit_index <= 3'd7;
                                state <= ST_TX_SETUP;
                            end
                            2'd1: begin
                                tx_stage <= 2'd2;
                                tx_byte <= selected_register[7:0];
                                bit_index <= 3'd7;
                                state <= ST_TX_SETUP;
                            end
                            2'd2: state <= ST_RESTART_IDLE;
                            default: begin
                                rx_byte <= 8'd0;
                                bit_index <= 3'd7;
                                state <= ST_READ_RAISE;
                            end
                        endcase
                    end
                    ST_RESTART_IDLE: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b1;
                        state <= ST_RESTART_ASSERT;
                    end
                    ST_RESTART_ASSERT: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b0;
                        sda_oe <= 1'b1;
                        state <= ST_RESTART_HOLD;
                    end
                    ST_RESTART_HOLD: begin
                        scl_out <= 1'b0;
                        sda_out <= 1'b0;
                        sda_oe <= 1'b1;
                        tx_byte <= 8'h79;
                        tx_stage <= 2'd3;
                        bit_index <= 3'd7;
                        state <= ST_TX_SETUP;
                    end
                    ST_READ_RAISE: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b0;
                        state <= ST_READ_SAMPLE;
                    end
                    ST_READ_SAMPLE: begin
                        rx_byte[bit_index] <= sda_in;
                        scl_out <= 1'b0;
                        if (bit_index == 3'd0)
                            state <= ST_READ_DC_SETUP;
                        else begin
                            bit_index <= bit_index - 1'b1;
                            state <= ST_READ_RAISE;
                        end
                    end
                    ST_READ_DC_SETUP: begin
                        scl_out <= 1'b0;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b1;
                        state <= ST_READ_DC_HIGH;
                    end
                    ST_READ_DC_HIGH: begin
                        scl_out <= 1'b1;
                        state <= ST_READ_DC_FALL;
                    end
                    ST_READ_DC_FALL: begin
                        scl_out <= 1'b0;
                        state <= ST_STOP_SETUP;
                    end
                    ST_STOP_SETUP: begin
                        scl_out <= 1'b0;
                        sda_out <= 1'b0;
                        sda_oe <= 1'b1;
                        state <= ST_STOP_HIGH;
                    end
                    ST_STOP_HIGH: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b0;
                        sda_oe <= 1'b1;
                        state <= ST_STOP_RELEASE;
                    end
                    ST_STOP_RELEASE: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b1;
                        case (register_index)
                            5'd0:  chip_id[15:8] <= rx_byte;
                            5'd1:  chip_id[7:0] <= rx_byte;
                            5'd2:  clock_enable <= rx_byte;
                            5'd3:  pad_output_enable <= rx_byte;
                            5'd4:  system_control <= rx_byte;
                            5'd5:  pad_output_enable2 <= rx_byte;
                            5'd6:  polarity_control <= rx_byte;
                            5'd7:  format_control <= rx_byte;
                            5'd8:  isp_format_control <= rx_byte;
                            5'd9:  output_width[15:8] <= rx_byte;
                            5'd10: output_width[7:0] <= rx_byte;
                            5'd11: output_height[15:8] <= rx_byte;
                            5'd12: output_height[7:0] <= rx_byte;
                            5'd13: timing_readback[7:0] <= rx_byte;
                            5'd14: timing_readback[15:8] <= rx_byte;
                            5'd15: timing_readback[23:16] <= rx_byte;
                            5'd16: timing_readback[31:24] <= rx_byte;
                            5'd17: timing_readback[39:32] <= rx_byte;
                            5'd18: timing_readback[47:40] <= rx_byte;
                            5'd19: timing_readback[55:48] <= rx_byte;
                            5'd20: timing_readback[63:56] <= rx_byte;
                            5'd21: timing_readback[71:64] <= rx_byte;
                            5'd22: timing_readback[79:72] <= rx_byte;
                            5'd23: timing_readback[87:80] <= rx_byte;
                            5'd24: timing_readback[95:88] <= rx_byte;
                            5'd25: timing_readback[103:96] <= rx_byte;
                            5'd26: timing_readback[111:104] <= rx_byte;
                            5'd27: timing_readback[119:112] <= rx_byte;
                            5'd28: timing_readback[127:120] <= rx_byte;
                            5'd29: lsc_readback[7:0] <= rx_byte;
                            5'd30: lsc_readback[15:8] <= rx_byte;
                            5'd31: lsc_readback[23:16] <= rx_byte;
                            6'd32: lsc_readback[31:24] <= rx_byte;
                            6'd33: awb_capture_buffer[7:0] <= rx_byte;
                            6'd34: awb_capture_buffer[15:8] <= rx_byte;
                            6'd35: awb_capture_buffer[23:16] <= rx_byte;
                            6'd36: awb_capture_buffer[31:24] <= rx_byte;
                            6'd37: awb_capture_buffer[39:32] <= rx_byte;
                            6'd38: awb_capture_buffer[47:40] <= rx_byte;
                            6'd39: awb_capture_buffer[55:48] <= rx_byte;
                            6'd40: awb_capture_buffer[63:56] <= rx_byte;
                            6'd41: awb_capture_buffer[71:64] <= rx_byte;
                            6'd42: awb_capture_buffer[79:72] <= rx_byte;
                        endcase
                        if (register_index == 6'd32) begin
                            if (chip_id !== 16'h5640 ||
                                clock_enable !== 8'hFF ||
                                pad_output_enable !== 8'hFF ||
                                system_control !== 8'h02 ||
                                pad_output_enable2 !== 8'hFF ||
                                polarity_control !== 8'h22 ||
                                format_control !== 8'h00 ||
                                isp_format_control !== 8'h03 ||
                                output_width !== 16'd1920 ||
                                output_height[15:8] !== 8'h04 ||
                                output_height[7:0] !== 8'h38 ||
                                timing_readback[119:0] !== {8'h00,8'h11,8'h11,8'h60,8'h04,8'hC4,8'h09,8'hF1,8'h05,8'hEF,8'h08,8'hB2,8'h01,8'h50,8'h01} ||
                                timing_readback[127:120] !== 8'h10 ||
                                lsc_readback[23:0] !== 24'h2383A7 ||
                                rx_byte !== 8'hCE)
                                error <= 1'b1;
                            initialization_complete <= 1'b1;
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= ST_IDLE;
                        end else if (register_index == 6'd42) begin
                            busy <= 1'b0;
                            awb_active <= 1'b0;
                            state <= ST_IDLE;
                            // Commit all ten bytes atomically only if no newer
                            // request superseded this sample while it ran.
                            if (awb_active &&
                                (awb_transaction_toggle == awb_req_seen) &&
                                (awb_req_sync == awb_req_seen)) begin
                                awb_data <= {rx_byte, awb_capture_buffer[71:0]};
                                awb_valid <= 1'b1;
                                awb_error_code <= 3'd0;
                                awb_done_toggle <= awb_transaction_toggle;
                            end
                        end else begin
                            register_index <= register_index + 1'b1;
                            state <= ST_START_IDLE;
                        end
                    end
                    default: begin
                        scl_out <= 1'b1;
                        sda_out <= 1'b1;
                        sda_oe <= 1'b0;
                        busy <= 1'b0;
                        error <= 1'b1;
                        done <= 1'b1;
                        state <= ST_IDLE;
                        if (awb_active) begin
                            awb_active <= 1'b0;
                            awb_data <= 80'd0;
                            awb_valid <= 1'b0;
                            awb_error_code <= 3'd2;
                            awb_done_toggle <= awb_transaction_toggle;
                        end
                    end
                endcase
            end
        end
    end
endmodule
