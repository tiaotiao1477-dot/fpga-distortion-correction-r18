`timescale 1ns / 1ps

// Diagnostic UART with legacy A5, compatible A7 NN and R14 A8 Gamma packets.
module camera_diag_uart #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer BAUD_RATE   = 115_200
) (
    input              clk,
    input              rst_n,
    input              uart_rx,
    output             uart_tx,
    output reg         snapshot_req_toggle,
    input              snapshot_valid,
    input      [1951:0] snapshot_payload,
    input              awb_done_toggle,
    input      [1151:0] nn_snapshot_payload,
    input              nn_ack_toggle_async,
    output reg         measurement_req_toggle,
    input      [1023:0] measurement_snapshot_payload,
    input              measurement_ack_toggle_async
);
    localparam integer PAYLOAD_BYTES = 244;
    localparam integer PACKET_BYTES = 4 + PAYLOAD_BYTES + 3;
    localparam integer NN_PAYLOAD_BYTES = 128;
    localparam integer NN_PACKET_BYTES = 4 + NN_PAYLOAD_BYTES + 1 + 2;
    localparam integer GAMMA_PACKET_BYTES = 27;
    localparam integer MEASUREMENT_PACKET_BYTES = 136;
    localparam integer BAUD_DIV = (CLK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;
    localparam integer SNAPSHOT_TIMEOUT_CYCLES = CLK_FREQ_HZ / 10;

    wire [7:0] rx_data;
    wire rx_valid;
    wire tx_busy;
    wire tx_start;
    reg [7:0] tx_data;
    reg [1951:0] payload_reg;
    reg [1023:0] nn_payload_reg;
    reg [127:0] gamma_payload_reg;
    reg [1023:0] measurement_payload_reg;
    reg [15:0] measurement_crc;
    reg packet_kind_measure;
    reg waiting_measurement;
    (* ASYNC_REG = "TRUE" *) reg measurement_ack_meta;
    (* ASYNC_REG = "TRUE" *) reg measurement_ack_sync1;
    (* ASYNC_REG = "TRUE" *) reg measurement_ack_sync2;
    reg measurement_ack_last;
    wire measurement_ack_transition = (measurement_ack_sync2 != measurement_ack_last);
    reg packet_kind_gamma;
    reg [7:0] packet_index;
    reg packet_active;
    reg packet_kind_nn;
    reg waiting_snapshot;
    reg waiting_nn;
    reg nn_fresh;
    reg [31:0] snapshot_wait_count;
    reg nn_ack_meta;
    reg nn_ack_sync;
    reg nn_ack_last;
    wire nn_ack_transition = nn_ack_sync != nn_ack_last;

    camera_diag_uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk(clk), .rst_n(rst_n), .rxd(uart_rx),
        .data(rx_data), .data_valid(rx_valid)
    );

    camera_diag_uart_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_tx (
        .clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .busy(tx_busy), .txd(uart_tx)
    );

    assign tx_start = packet_active && !tx_busy;

    function [7:0] payload_byte;
        input [1951:0] payload;
        input [7:0] index;
        begin
            payload_byte = payload >> (index * 8);
        end
    endfunction

    function [7:0] payload_checksum;
        input [1951:0] payload;
        integer i;
        reg [7:0] checksum;
        begin
            checksum = 8'd0;
            for (i = 0; i < PAYLOAD_BYTES; i = i + 1)
                checksum = checksum ^ (payload >> (i * 8));
            payload_checksum = checksum;
        end
    endfunction

    function [7:0] nn_payload_byte;
        input [1023:0] payload;
        input [7:0] index;
        begin
            nn_payload_byte = payload >> (index * 8);
        end
    endfunction

    function [7:0] nn_payload_checksum;
        input [1023:0] payload;
        integer i;
        reg [7:0] checksum;
        begin
            checksum = 8'd0;
            for (i = 0; i < NN_PAYLOAD_BYTES; i = i + 1)
                checksum = checksum ^ (payload >> (i * 8));
            nn_payload_checksum = checksum;
        end
    endfunction

    function [7:0] gamma_payload_byte;
        input [127:0] payload;
        input [7:0] index;
        begin
            case(index)
                8'd0: gamma_payload_byte=8'h01;
                8'd1: gamma_payload_byte=8'h00;
                8'd2: gamma_payload_byte=8'h4D;
                8'd3: gamma_payload_byte=8'h47;
                default: gamma_payload_byte=payload >> ((index-8'd4)*8);
            endcase
        end
    endfunction

    function [7:0] gamma_payload_checksum;
        input [127:0] payload;
        integer i;
        reg [7:0] checksum;
        begin
            checksum=8'd0;
            for(i=0;i<20;i=i+1) checksum=checksum ^ gamma_payload_byte(payload,i);
            gamma_payload_checksum=checksum;
        end
    endfunction

    function [15:0] crc16_ccitt_byte;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] crc_work;
        begin
            crc_work = crc_in ^ {data_in, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc_work = crc_work[15] ? ((crc_work << 1) ^ 16'h1021) : (crc_work << 1);
            crc16_ccitt_byte = crc_work;
        end
    endfunction

    function [1951:0] payload_at_deadline;
        input [1951:0] payload;
        input         awb_complete;
        begin
            payload_at_deadline = payload;
            if (!awb_complete) begin
                payload_at_deadline[58*32 +: 32] = 32'd0;
                payload_at_deadline[59*32 +: 32] = 32'd0;
                payload_at_deadline[60*32 +: 32] = 32'h03000000;
            end
        end
    endfunction

    always @(*) begin
        if (packet_kind_measure) begin
            if (packet_index == 8'd0) tx_data=8'h55;
            else if (packet_index == 8'd1) tx_data=8'hAA;
            else if (packet_index == 8'd2) tx_data=8'hE5;
            else if (packet_index == 8'd3) tx_data=8'h80;
            else if ((packet_index >= 8'd4) && (packet_index <= 8'd131))
                tx_data=measurement_payload_reg[(packet_index-8'd4)*8 +: 8];
            else if (packet_index == 8'd132) tx_data=measurement_crc[7:0];
            else if (packet_index == 8'd133) tx_data=measurement_crc[15:8];
            else if (packet_index == 8'd134) tx_data=8'h0D;
            else if (packet_index == 8'd135) tx_data=8'h0A;
            else tx_data=8'h00;
        end else if (packet_kind_gamma) begin
            if (packet_index == 8'd0) tx_data=8'h55;
            else if (packet_index == 8'd1) tx_data=8'hAA;
            else if (packet_index == 8'd2) tx_data=8'hE4;
            else if (packet_index == 8'd3) tx_data=8'h14;
            else if ((packet_index>=8'd4)&&(packet_index<=8'd23))
                tx_data=nn_fresh ? gamma_payload_byte(gamma_payload_reg,packet_index-8'd4) : 8'h00;
            else if (packet_index==8'd24) tx_data=nn_fresh ? gamma_payload_checksum(gamma_payload_reg) : 8'h00;
            else if (packet_index==8'd25) tx_data=8'h0D;
            else if (packet_index==8'd26) tx_data=8'h0A;
            else tx_data=8'h00;
        end else if (packet_kind_nn) begin
            if (packet_index == 8'd0)
                tx_data = 8'h55;
            else if (packet_index == 8'd1)
                tx_data = 8'hAA;
            else if (packet_index == 8'd2)
                tx_data = 8'hE3;
            else if (packet_index == 8'd3)
                tx_data = 8'h80;
            else if ((packet_index >= 8'd4) && (packet_index <= 8'd131))
                tx_data = nn_payload_byte(nn_payload_reg, packet_index - 8'd4);
            else if (packet_index == 8'd132)
                tx_data = nn_payload_checksum(nn_payload_reg);
            else if (packet_index == 8'd133)
                tx_data = 8'h0D;
            else if (packet_index == 8'd134)
                tx_data = 8'h0A;
            else
                tx_data = 8'h00;
        end else begin
            if (packet_index == 8'd0)
                tx_data = 8'h55;
            else if (packet_index == 8'd1)
                tx_data = 8'hAA;
            else if (packet_index == 8'd2)
                tx_data = 8'hD1;
            else if (packet_index == 8'd3)
                tx_data = 8'hF4;
            else if ((packet_index >= 8'd4) && (packet_index <= 8'd247))
                tx_data = payload_byte(payload_reg, packet_index - 8'd4);
            else if (packet_index == 8'd248)
                tx_data = payload_checksum(payload_reg);
            else if (packet_index == 8'd249)
                tx_data = 8'h0D;
            else if (packet_index == 8'd250)
                tx_data = 8'h0A;
            else
                tx_data = 8'h00;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_req_toggle <= 1'b0;
            payload_reg <= 1952'd0;
            nn_payload_reg <= 1024'd0;
            gamma_payload_reg <= 128'd0;
            measurement_req_toggle <= 1'b0;
            measurement_payload_reg <= 1024'd0;
            measurement_crc <= 16'hFFFF;
            packet_kind_measure <= 1'b0;
            waiting_measurement <= 1'b0;
            measurement_ack_meta <= 1'b0;
            measurement_ack_sync1 <= 1'b0;
            measurement_ack_sync2 <= 1'b0;
            measurement_ack_last <= 1'b0;
            packet_kind_gamma <= 1'b0;
            packet_index <= 8'd0;
            packet_active <= 1'b0;
            packet_kind_nn <= 1'b0;
            waiting_snapshot <= 1'b0;
            waiting_nn <= 1'b0;
            nn_fresh <= 1'b0;
            snapshot_wait_count <= 32'd0;
            nn_ack_meta <= 1'b0;
            nn_ack_sync <= 1'b0;
            nn_ack_last <= 1'b0;
        end else begin
            nn_ack_meta <= nn_ack_toggle_async;
            nn_ack_sync <= nn_ack_meta;
            nn_ack_last <= nn_ack_sync;
            measurement_ack_meta <= measurement_ack_toggle_async;
            measurement_ack_sync1 <= measurement_ack_meta;
            measurement_ack_sync2 <= measurement_ack_sync1;
            measurement_ack_last <= measurement_ack_sync2;

            if (rx_valid && !packet_active && !waiting_snapshot && !waiting_nn &&
                !waiting_measurement && !tx_busy) begin
                if (rx_data == 8'hA5) begin
                    snapshot_req_toggle <= ~snapshot_req_toggle;
                    waiting_snapshot <= 1'b1;
                    packet_kind_nn <= 1'b0;
                    packet_kind_gamma <= 1'b0;
                    packet_kind_measure <= 1'b0;
                    snapshot_wait_count <= 32'd0;
                end else if (rx_data == 8'hA7) begin
                    snapshot_req_toggle <= ~snapshot_req_toggle;
                    waiting_nn <= 1'b1;
                    nn_fresh <= 1'b0;
                    packet_kind_nn <= 1'b1;
                    packet_kind_gamma <= 1'b0;
                    packet_kind_measure <= 1'b0;
                    snapshot_wait_count <= 32'd0;
                end else if (rx_data == 8'hA8) begin
                    snapshot_req_toggle <= ~snapshot_req_toggle;
                    waiting_nn <= 1'b1;
                    packet_kind_nn <= 1'b0;
                    packet_kind_gamma <= 1'b1;
                    packet_kind_measure <= 1'b0;
                    snapshot_wait_count <= 32'd0;
                end else if (rx_data == 8'hA9) begin
                    measurement_req_toggle <= ~measurement_req_toggle;
                    waiting_measurement <= 1'b1;
                    packet_kind_nn <= 1'b0;
                    packet_kind_gamma <= 1'b0;
                    packet_kind_measure <= 1'b1;
                    snapshot_wait_count <= 32'd0;
                end
            end

            if (waiting_snapshot && snapshot_valid &&
                (awb_done_toggle == snapshot_req_toggle)) begin
                payload_reg <= snapshot_payload;
                packet_index <= 8'd0;
                packet_active <= 1'b1;
                waiting_snapshot <= 1'b0;
                snapshot_wait_count <= 32'd0;
            end else if (waiting_snapshot &&
                         (snapshot_wait_count >= SNAPSHOT_TIMEOUT_CYCLES - 1)) begin
                payload_reg <= payload_at_deadline(
                    snapshot_payload, (awb_done_toggle == snapshot_req_toggle));
                packet_index <= 8'd0;
                packet_active <= 1'b1;
                waiting_snapshot <= 1'b0;
                snapshot_wait_count <= 32'd0;
            end else if (waiting_snapshot) begin
                snapshot_wait_count <= snapshot_wait_count + 1'b1;
            end else if (waiting_nn && nn_ack_transition &&
                         (nn_ack_sync == snapshot_req_toggle)) begin
                // Ack toggling means the core-held bundled payload is stable for capture.
                if (packet_kind_gamma)
                    gamma_payload_reg <= nn_snapshot_payload[1151:1024];
                else
                    nn_payload_reg <= nn_snapshot_payload[1023:0];
                nn_fresh <= 1'b1;
                packet_index <= 8'd0;
                packet_active <= 1'b1;
                waiting_nn <= 1'b0;
                snapshot_wait_count <= 32'd0;
            end else if (waiting_nn &&
                         (snapshot_wait_count >= SNAPSHOT_TIMEOUT_CYCLES - 1)) begin
                // A zero NN payload is the explicit not-fresh/reject response.
                if (packet_kind_gamma) gamma_payload_reg <= 128'd0;
                else nn_payload_reg <= 1024'd0;
                nn_fresh <= 1'b0;
                packet_index <= 8'd0;
                packet_active <= 1'b1;
                waiting_nn <= 1'b0;
                snapshot_wait_count <= 32'd0;
            end else if (waiting_nn) begin
                snapshot_wait_count <= snapshot_wait_count + 1'b1;
            end else if (waiting_measurement && measurement_ack_transition &&
                         (measurement_ack_sync2 == measurement_req_toggle)) begin
                // The request toggle changed before this match; the core holds
                // this bundled payload stable until a later request.
                measurement_payload_reg <= measurement_snapshot_payload;
                measurement_crc <= 16'hFFFF;
                packet_index <= 8'd0;
                packet_active <= 1'b1;
                waiting_measurement <= 1'b0;
                snapshot_wait_count <= 32'd0;
            end else if (waiting_measurement &&
                         (snapshot_wait_count >= SNAPSHOT_TIMEOUT_CYCLES - 1)) begin
                // Timeout is a fresh, explicit all-zero response, never stale data.
                measurement_payload_reg <= 1024'd0;
                measurement_crc <= 16'hFFFF;
                packet_index <= 8'd0;
                packet_active <= 1'b1;
                waiting_measurement <= 1'b0;
                snapshot_wait_count <= 32'd0;
            end else if (waiting_measurement) begin
                snapshot_wait_count <= snapshot_wait_count + 1'b1;
            end else if (tx_start) begin
                if (packet_kind_measure && (packet_index >= 8'd4) && (packet_index <= 8'd131))
                    measurement_crc <= crc16_ccitt_byte(
                        measurement_crc, measurement_payload_reg[(packet_index-8'd4)*8 +: 8]);
                if ((packet_kind_measure && (packet_index == MEASUREMENT_PACKET_BYTES - 1)) ||
                    (packet_kind_gamma && (packet_index == GAMMA_PACKET_BYTES - 1)) ||
                    (packet_kind_nn && (packet_index == NN_PACKET_BYTES - 1)) ||
                    (!packet_kind_measure && !packet_kind_gamma && !packet_kind_nn &&
                     (packet_index == PACKET_BYTES - 1))) begin
                    packet_active <= 1'b0;
                    packet_index <= 8'd0;
                end else begin
                    packet_index <= packet_index + 1'b1;
                end
            end
        end
    end
endmodule
module camera_diag_uart_tx #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer BAUD_RATE   = 115_200
) (
    input       clk,
    input       rst_n,
    input       start,
    input [7:0] data,
    output reg  busy,
    output reg  txd
);
    localparam integer BAUD_DIV = (CLK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;
    reg [15:0] baud_count;
    reg [3:0] bit_index;
    reg [9:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_count <= 16'd0;
            bit_index <= 4'd0;
            shift_reg <= 10'h3ff;
            busy <= 1'b0;
            txd <= 1'b1;
        end else if (!busy) begin
            txd <= 1'b1;
            if (start) begin
                shift_reg <= {1'b1, data, 1'b0};
                baud_count <= BAUD_DIV - 1;
                bit_index <= 4'd0;
                busy <= 1'b1;
                txd <= 1'b0;
            end
        end else if (baud_count == 0) begin
            baud_count <= BAUD_DIV - 1;
            if (bit_index == 4'd9) begin
                busy <= 1'b0;
                txd <= 1'b1;
            end else begin
                bit_index <= bit_index + 1'b1;
                shift_reg <= {1'b1, shift_reg[9:1]};
                txd <= shift_reg[1];
            end
        end else begin
            baud_count <= baud_count - 1'b1;
        end
    end
endmodule

module camera_diag_uart_rx #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer BAUD_RATE   = 115_200
) (
    input       clk,
    input       rst_n,
    input       rxd,
    output reg [7:0] data,
    output reg  data_valid
);
    localparam integer BAUD_DIV = (CLK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;
    reg rx_meta;
    reg rx_sync;
    reg rx_busy;
    reg [15:0] baud_count;
    reg [3:0] bit_index;
    reg [7:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            rx_busy <= 1'b0;
            baud_count <= 16'd0;
            bit_index <= 4'd0;
            shift_reg <= 8'd0;
            data <= 8'd0;
            data_valid <= 1'b0;
        end else begin
            rx_meta <= rxd;
            rx_sync <= rx_meta;
            data_valid <= 1'b0;

            if (!rx_busy) begin
                if (!rx_sync) begin
                    rx_busy <= 1'b1;
                    bit_index <= 4'd0;
                    baud_count <= BAUD_DIV + (BAUD_DIV / 2) - 1;
                end
            end else if (baud_count == 0) begin
                baud_count <= BAUD_DIV - 1;
                if (bit_index < 4'd8) begin
                    shift_reg[bit_index] <= rx_sync;
                    bit_index <= bit_index + 1'b1;
                end else begin
                    rx_busy <= 1'b0;
                    if (rx_sync) begin
                        data <= shift_reg;
                        data_valid <= 1'b1;
                    end
                end
            end else begin
                baud_count <= baud_count - 1'b1;
            end
        end
    end
endmodule

