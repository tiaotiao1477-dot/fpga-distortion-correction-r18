`timescale 1ns / 1ps

// Core-to-pixel gamma identity override with frame-boundary application and
// feedback-qualified completion.  request_identity=0 selects AUTO; 1 forces
// the tone block into its identity/bypass mode.
module measurement_gamma_control_r18 (
    input              core_clk,
    input              core_rst_n,
    input              pix_clk,
    input              pix_rst_n,
    input              request_valid,
    input      [31:0]  request_seq,
    input              request_identity,
    output             request_ready,
    output reg [31:0]  applied_seq,
    output reg         applied_identity,
    output             busy,
    input              pix_vs,
    input              pix_applied_identity,
    output reg         pix_force_identity
);
    wire shared_reset_n = core_rst_n & pix_rst_n;

    (* ASYNC_REG = "TRUE" *) reg core_reset_release_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg core_reset_release_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg core_reset_release_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    (* ASYNC_REG = "TRUE" *) reg pix_reset_release_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg pix_reset_release_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg pix_reset_release_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    (* ASYNC_REG = "TRUE" *) reg core_release_pix_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg core_release_pix_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg core_release_pix_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    (* ASYNC_REG = "TRUE" *) reg pix_release_core_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg pix_release_core_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg pix_release_core_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    (* ASYNC_REG = "TRUE" *) reg req_toggle_pix_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg req_toggle_pix_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg req_toggle_pix_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    (* ASYNC_REG = "TRUE" *) reg ack_toggle_core_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg ack_toggle_core_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg ack_toggle_core_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    reg req_toggle_core;
    reg ack_toggle_pix;
    reg source_busy;
    reg [31:0] held_request_seq;
    reg held_request_identity;

    reg pix_req_seen;
    reg pix_request_pending;
    reg pix_request_boundary_seen;
    reg [31:0] pix_request_seq_held;
    reg pix_request_identity_held;
    reg pix_vs_d;
    reg [31:0] pix_ack_seq_held;
    reg pix_ack_identity_held;

    assign busy = source_busy;
    assign request_ready = core_reset_release_sync2 &
                           pix_release_core_sync2 &
                           ~source_busy;

    // A reset on either side asynchronously asserts both local release chains.
    // Each chain releases through three local clock edges; only these small
    // synchronizers use the shared asynchronous reset.
    always @(posedge core_clk or negedge shared_reset_n) begin
        if (!shared_reset_n) begin
            core_reset_release_meta  <= 1'b0;
            core_reset_release_sync1 <= 1'b0;
            core_reset_release_sync2 <= 1'b0;
        end else begin
            core_reset_release_meta  <= 1'b1;
            core_reset_release_sync1 <= core_reset_release_meta;
            core_reset_release_sync2 <= core_reset_release_sync1;
        end
    end

    always @(posedge pix_clk or negedge shared_reset_n) begin
        if (!shared_reset_n) begin
            pix_reset_release_meta  <= 1'b0;
            pix_reset_release_sync1 <= 1'b0;
            pix_reset_release_sync2 <= 1'b0;
        end else begin
            pix_reset_release_meta  <= 1'b1;
            pix_reset_release_sync1 <= pix_reset_release_meta;
            pix_reset_release_sync2 <= pix_reset_release_sync1;
        end
    end

    // Cross-domain release indications gate operation until both reset
    // synchronizers have completed in both clock domains.
    always @(posedge pix_clk or negedge pix_reset_release_sync2) begin
        if (!pix_reset_release_sync2) begin
            core_release_pix_meta  <= 1'b0;
            core_release_pix_sync1 <= 1'b0;
            core_release_pix_sync2 <= 1'b0;
        end else begin
            core_release_pix_meta  <= core_reset_release_sync2;
            core_release_pix_sync1 <= core_release_pix_meta;
            core_release_pix_sync2 <= core_release_pix_sync1;
        end
    end

    always @(posedge core_clk or negedge core_reset_release_sync2) begin
        if (!core_reset_release_sync2) begin
            pix_release_core_meta  <= 1'b0;
            pix_release_core_sync1 <= 1'b0;
            pix_release_core_sync2 <= 1'b0;
        end else begin
            pix_release_core_meta  <= pix_reset_release_sync2;
            pix_release_core_sync1 <= pix_release_core_meta;
            pix_release_core_sync2 <= pix_release_core_sync1;
        end
    end

    // Request phase crosses through three single-bit synchronizer stages.
    always @(posedge pix_clk or negedge pix_reset_release_sync2) begin
        if (!pix_reset_release_sync2) begin
            req_toggle_pix_meta  <= 1'b0;
            req_toggle_pix_sync1 <= 1'b0;
            req_toggle_pix_sync2 <= 1'b0;
        end else begin
            req_toggle_pix_meta  <= req_toggle_core;
            req_toggle_pix_sync1 <= req_toggle_pix_meta;
            req_toggle_pix_sync2 <= req_toggle_pix_sync1;
        end
    end

    // Completion phase crosses back through three single-bit stages.
    always @(posedge core_clk or negedge core_reset_release_sync2) begin
        if (!core_reset_release_sync2) begin
            ack_toggle_core_meta  <= 1'b0;
            ack_toggle_core_sync1 <= 1'b0;
            ack_toggle_core_sync2 <= 1'b0;
        end else begin
            ack_toggle_core_meta  <= ack_toggle_pix;
            ack_toggle_core_sync1 <= ack_toggle_core_meta;
            ack_toggle_core_sync2 <= ack_toggle_core_sync1;
        end
    end

    // The held multi-bit request remains stable from acceptance until the
    // returned ACK. The pixel side samples the held sequence/mode only after
    // the synchronized request toggle arrives; the ACK mailbox remains stable
    // until source_busy clears.
    always @(posedge core_clk or negedge core_reset_release_sync2) begin
        if (!core_reset_release_sync2) begin
            req_toggle_core       <= 1'b0;
            source_busy           <= 1'b0;
            held_request_seq      <= 32'd0;
            held_request_identity <= 1'b0;
            applied_seq           <= 32'd0;
            applied_identity      <= 1'b0;
        end else if (!pix_release_core_sync2) begin
            req_toggle_core       <= 1'b0;
            source_busy           <= 1'b0;
            held_request_seq      <= 32'd0;
            held_request_identity <= 1'b0;
            applied_seq           <= 32'd0;
            applied_identity      <= 1'b0;
        end else if (!source_busy) begin
            if (request_valid && request_ready) begin
                held_request_seq      <= request_seq;
                held_request_identity <= request_identity;
                req_toggle_core       <= ~req_toggle_core;
                source_busy           <= 1'b1;
            end
        end else if (ack_toggle_core_sync2 == req_toggle_core) begin
            // ACK payload is held in the pixel domain until source_busy clears.
            applied_seq      <= pix_ack_seq_held;
            applied_identity <= pix_ack_identity_held;
            source_busy      <= 1'b0;
        end
    end

    // Pixel sink.  The output mode changes only on a VS rising edge.  ACK is
    // generated only when the real tone feedback equals the held request.
    always @(posedge pix_clk or negedge pix_reset_release_sync2) begin
        if (!pix_reset_release_sync2) begin
            pix_req_seen             <= 1'b0;
            pix_request_pending      <= 1'b0;
            pix_request_boundary_seen <= 1'b0;
            pix_request_seq_held     <= 32'd0;
            pix_request_identity_held <= 1'b0;
            pix_vs_d                 <= 1'b0;
            pix_force_identity       <= 1'b0;
            ack_toggle_pix           <= 1'b0;
            pix_ack_seq_held         <= 32'd0;
            pix_ack_identity_held    <= 1'b0;
        end else if (!core_release_pix_sync2) begin
            pix_req_seen              <= 1'b0;
            pix_request_pending       <= 1'b0;
            pix_request_boundary_seen <= 1'b0;
            pix_request_seq_held      <= 32'd0;
            pix_request_identity_held <= 1'b0;
            pix_vs_d                  <= 1'b0;
            pix_force_identity        <= 1'b0;
            ack_toggle_pix            <= 1'b0;
            pix_ack_seq_held          <= 32'd0;
            pix_ack_identity_held     <= 1'b0;
        end else begin
            pix_vs_d <= pix_vs;

            if (req_toggle_pix_sync2 != pix_req_seen) begin
                pix_req_seen              <= req_toggle_pix_sync2;
                pix_request_pending       <= 1'b1;
                pix_request_boundary_seen <= 1'b0;
                pix_request_seq_held      <= held_request_seq;
                pix_request_identity_held <= held_request_identity;
            end

            if (pix_vs && !pix_vs_d && pix_request_pending) begin
                pix_force_identity <= pix_request_identity_held;
                pix_request_boundary_seen <= 1'b1;
            end

            if (pix_request_pending &&
                pix_request_boundary_seen &&
                (pix_applied_identity == pix_request_identity_held)) begin
                pix_ack_seq_held       <= pix_request_seq_held;
                pix_ack_identity_held  <= pix_request_identity_held;
                ack_toggle_pix         <= pix_req_seen;
                pix_request_pending    <= 1'b0;
                pix_request_boundary_seen <= 1'b0;
            end
        end
    end
endmodule

