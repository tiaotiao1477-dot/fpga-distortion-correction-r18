`timescale 1ns/1ps
module measurement_display_observer_r16 #(
    parameter integer FIFO_DEPTH = 16
) (
    input wire core_clk,
    input wire core_rst_n,
    input wire pix_clk,
    input wire pix_rst_n,
    input wire [63:0] time_gray,
    input wire vs,
    input wire de,
    input wire source_sof,
    input wire [31:0] source_id,
    output reg event_valid,
    output reg [31:0] event_source_id,
    output reg [63:0] event_ticks,
    // errors[31:16]=16-bit overflow count; errors[15:0]=16-bit missing-ID count.
    output wire [31:0] errors
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset_release;
    (* ASYNC_REG = "TRUE" *) reg [2:0] core_reset_release;
    wire observer_reset_n = core_rst_n && pix_rst_n;
    wire pix_domain_rst_n = pix_reset_release[2];
    wire core_domain_rst_n = core_reset_release[2];

    // Either external reset asynchronously clears both FIFO domains; each domain
    // then releases only after its own three-clock synchronizer.
    always @(posedge pix_clk or negedge observer_reset_n) begin
        if (!observer_reset_n)
            pix_reset_release <= 3'b000;
        else
            pix_reset_release <= {pix_reset_release[1:0], 1'b1};
    end
    always @(posedge core_clk or negedge observer_reset_n) begin
        if (!observer_reset_n)
            core_reset_release <= 3'b000;
        else
            core_reset_release <= {core_reset_release[1:0], 1'b1};
    end

    (* ASYNC_REG = "TRUE" *) reg [63:0] time_gray_pix_meta;
    (* ASYNC_REG = "TRUE" *) reg [63:0] time_gray_pix_sync1;
    (* ASYNC_REG = "TRUE" *) reg [63:0] time_gray_pix_sync2;
    reg vs_d;
    reg frame_armed;
    reg seen_source_stream;
    reg source_id_valid;
    reg [31:0] source_id_held;
    reg [15:0] overflow_count_pix;
    reg [15:0] overflow_gray_pix;
    reg [15:0] missing_count_pix;
    reg [15:0] missing_gray_pix;

    (* ASYNC_REG = "TRUE" *) reg [15:0] overflow_gray_core_meta;
    (* ASYNC_REG = "TRUE" *) reg [15:0] overflow_gray_core_sync1;
    (* ASYNC_REG = "TRUE" *) reg [15:0] overflow_gray_core_sync2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] missing_gray_core_meta;
    (* ASYNC_REG = "TRUE" *) reg [15:0] missing_gray_core_sync1;
    (* ASYNC_REG = "TRUE" *) reg [15:0] missing_gray_core_sync2;

    function [15:0] gray_to_binary16;
        input [15:0] gray_value;
        reg [15:0] stage1, stage2, stage3, stage4;
        begin
            stage1 = gray_value ^ (gray_value >> 1);
            stage2 = stage1 ^ (stage1 >> 2);
            stage3 = stage2 ^ (stage2 >> 4);
            stage4 = stage3 ^ (stage3 >> 8);
            gray_to_binary16 = stage4;
        end
    endfunction

    function [63:0] gray_to_binary64;
        input [63:0] gray_value;
        reg [63:0] stage1, stage2, stage3, stage4, stage5, stage6;
        begin
            stage1 = gray_value ^ (gray_value >> 1);
            stage2 = stage1 ^ (stage1 >> 2);
            stage3 = stage2 ^ (stage2 >> 4);
            stage4 = stage3 ^ (stage3 >> 8);
            stage5 = stage4 ^ (stage4 >> 16);
            stage6 = stage5 ^ (stage5 >> 32);
            gray_to_binary64 = stage6;
        end
    endfunction

    wire [63:0] timestamp_pix = gray_to_binary64(time_gray_pix_sync2);
    wire [15:0] overflow_count_core = gray_to_binary16(overflow_gray_core_sync2);
    wire [15:0] missing_count_core = gray_to_binary16(missing_gray_core_sync2);
    assign errors = {overflow_count_core, missing_count_core};
    wire vs_rise = vs && !vs_d;
    wire frame_start_candidate = pix_domain_rst_n && (frame_armed || vs_rise) && de;
    // VS invalidates the previous frame's ID before a coincident first DE;
    // a same-cycle accepted SOF is current-frame metadata and takes priority.
    wire record_id_valid = source_sof || (source_id_valid && !vs_rise);
    wire [31:0] record_source_id = source_sof ? source_id : source_id_held;
    wire observer_wr_valid = frame_start_candidate && record_id_valid;
    wire observer_wr_ready;
    wire observer_rd_valid;
    wire [95:0] observer_rd_data;
    wire observer_rd_ready = 1'b1;
    wire observer_overflow = observer_wr_valid && !observer_wr_ready;
    wire observer_missing = frame_start_candidate && seen_source_stream && !record_id_valid;

    raw_async_fifo_dualreset_r1 #(.DATA_WIDTH(96), .DEPTH(FIFO_DEPTH)) measurement_t2_fifo (
        .wr_clk(pix_clk), .rd_clk(core_clk),
        .wr_rst_n(pix_domain_rst_n), .rd_rst_n(core_domain_rst_n),
        .wr_valid(observer_wr_valid), .wr_ready(observer_wr_ready),
        .wr_data({record_source_id, timestamp_pix}),
        .rd_valid(observer_rd_valid), .rd_ready(observer_rd_ready),
        .rd_data(observer_rd_data)
    );

    always @(posedge pix_clk or negedge pix_domain_rst_n) begin
        if (!pix_domain_rst_n) begin
            time_gray_pix_meta <= 64'd0;
            time_gray_pix_sync1 <= 64'd0;
            time_gray_pix_sync2 <= 64'd0;
            vs_d <= 1'b0;
            frame_armed <= 1'b0;
            seen_source_stream <= 1'b0;
            source_id_valid <= 1'b0;
            source_id_held <= 32'd0;
            overflow_count_pix <= 16'd0;
            overflow_gray_pix <= 16'd0;
            missing_count_pix <= 16'd0;
            missing_gray_pix <= 16'd0;
        end else begin
            time_gray_pix_meta <= time_gray;
            time_gray_pix_sync1 <= time_gray_pix_meta;
            time_gray_pix_sync2 <= time_gray_pix_sync1;
            vs_d <= vs;

            if (vs_rise) begin
                frame_armed <= 1'b1;
                source_id_valid <= 1'b0;
            end
            if (source_sof) begin
                seen_source_stream <= 1'b1;
                source_id_valid <= 1'b1;
                source_id_held <= source_id;
            end
            if (frame_start_candidate)
                frame_armed <= 1'b0;

            if (observer_overflow && overflow_count_pix != 16'hffff) begin
                overflow_count_pix <= overflow_count_pix + 16'd1;
                overflow_gray_pix <= ((overflow_count_pix + 16'd1) >> 1) ^
                                     (overflow_count_pix + 16'd1);
            end
            if (observer_missing && missing_count_pix != 16'hffff) begin
                missing_count_pix <= missing_count_pix + 16'd1;
                missing_gray_pix <= ((missing_count_pix + 16'd1) >> 1) ^
                                    (missing_count_pix + 16'd1);
            end
        end
    end

    always @(posedge core_clk or negedge core_domain_rst_n) begin
        if (!core_domain_rst_n) begin
            overflow_gray_core_meta <= 16'd0;
            overflow_gray_core_sync1 <= 16'd0;
            overflow_gray_core_sync2 <= 16'd0;
            missing_gray_core_meta <= 16'd0;
            missing_gray_core_sync1 <= 16'd0;
            missing_gray_core_sync2 <= 16'd0;
            event_valid <= 1'b0;
            event_ticks <= 64'd0;
            event_source_id <= 32'd0;
        end else begin
            overflow_gray_core_meta <= overflow_gray_pix;
            overflow_gray_core_sync1 <= overflow_gray_core_meta;
            overflow_gray_core_sync2 <= overflow_gray_core_sync1;
            missing_gray_core_meta <= missing_gray_pix;
            missing_gray_core_sync1 <= missing_gray_core_meta;
            missing_gray_core_sync2 <= missing_gray_core_sync1;
            event_valid <= observer_rd_valid && observer_rd_ready;
            if (observer_rd_valid && observer_rd_ready) begin
                event_source_id <= observer_rd_data[95:64];
                event_ticks <= observer_rd_data[63:0];
            end
        end
    end
endmodule
