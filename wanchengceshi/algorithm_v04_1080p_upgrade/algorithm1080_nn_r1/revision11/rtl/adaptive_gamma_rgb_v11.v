// Two elastic pixel stages; independent end-of-frame statistics decision.
// Default image/threshold semantics are bit-exact with v0.5.
module adaptive_gamma_rgb_v06 #(
    parameter LUT_FILE="../revision11/data/gamma_lut.hex"
) (
    input clk, input rst_n,
    input frame_start_valid, output frame_start_ready,
    input [15:0] frame_width, input [15:0] frame_height,
    input [31:0] frame_id, input [15:0] cfg_version,
    input enable, input history_reset,
    input in_valid, output in_ready, input [23:0] in_rgb,
    input in_sample_valid, input [31:0] in_pixel_tag,
    output reg out_valid, input out_ready, output [23:0] out_rgb,
    output reg out_sample_valid, output reg [31:0] out_pixel_tag,
    output reg [31:0] out_frame_id, output reg [15:0] out_cfg_version,
    output reg out_last, output reg [2:0] active_bank,
    output reg [39:0] last_luminance_sum, output reg [31:0] last_valid_count,
    output reg stats_valid, output reg [31:0] stats_frame_id,
    output reg [15:0] stats_cfg_version,
    output reg [2:0] stats_applied_bank, output reg [2:0] stats_next_bank,
    input clean_frame_commit,
    output idle
);
    // Keep the source table in logic/distributed ROM for this candidate.
    // The three synchronous channel reads and table contents are unchanged.
    (* syn_romstyle = "lut_rom" *) reg [7:0] lut [0:1279];
    initial $readmemh(LUT_FILE,lut);
    reg running, stage_valid, finalize_pending;
    reg [31:0] remaining, saved_frame;
    reg [15:0] saved_version;
    reg [2:0] target_bank;
    reg [2:0] pending_bank;
    reg pending_bank_valid;
    reg [39:0] luminance_sum;
    reg [31:0] valid_count;
    reg [23:0] stage_rgb, mapped_rgb, passthrough_rgb;
    reg [7:0] stage_gray;
    reg stage_sample, stage_last;
    reg [31:0] stage_tag;
    wire [15:0] r_term={8'd0,in_rgb[23:16]}*16'd77;
    wire [15:0] g_term={8'd0,in_rgb[15:8]}*16'd150;
    wire [15:0] b_term={8'd0,in_rgb[7:0]}*16'd29;
    wire [15:0] gray_sum=r_term+g_term+b_term+16'd128;
    wire output_ready=!out_valid || out_ready;
    wire stage_ready=!stage_valid || output_ready;
    wire stage_move=rst_n && stage_valid && output_ready;
    assign idle=!running && !stage_valid && !out_valid && !finalize_pending;
    assign frame_start_ready=rst_n && idle && frame_width!=0 && frame_height!=0;
    assign in_ready=rst_n && running && stage_ready;
    assign out_rgb=(out_sample_valid && active_bank!=2) ? mapped_rgb : passthrough_rgb;

    function [2:0] choose_bank;
        input [2:0] bank;
        input [39:0] total;
        input [31:0] count;
        reg [39:0] n;
        begin
            n={8'd0,count}; choose_bank=bank;
            if (count!=0) begin
                case(bank)
                    0: if (total>n*40'd72) choose_bank=1;
                    1: if (total<n*40'd56) choose_bank=0;
                       else if (total>n*40'd112) choose_bank=2;
                    2: if (total<n*40'd96) choose_bank=1;
                       else if (total>n*40'd160) choose_bank=3;
                    3: if (total<n*40'd144) choose_bank=2;
                       else if (total>n*40'd200) choose_bank=4;
                    4: if (total<n*40'd184) choose_bank=3;
                    default: choose_bank=2;
                endcase
            end
        end
    endfunction

    // Synchronous ROM read, deliberately without async reset on ROM output.
    // Valid/mask/bypass registers below determine when this value is visible.
    // Mapping to device RAM (and replication for three reads) requires PDS.
    always @(posedge clk) begin
        if (stage_move && stage_sample && active_bank!=2)
            mapped_rgb <= {lut[{active_bank,stage_rgb[23:16]}],
                           lut[{active_bank,stage_rgb[15:8]}],lut[{active_bank,stage_rgb[7:0]}]};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running<=0; stage_valid<=0; out_valid<=0; finalize_pending<=0;
            remaining<=0; target_bank<=2; active_bank<=2;
            luminance_sum<=0; valid_count<=0; last_luminance_sum<=0; last_valid_count<=0;
            saved_frame<=0; saved_version<=0; passthrough_rgb<=0; out_sample_valid<=0;
            out_pixel_tag<=0; out_frame_id<=0; out_cfg_version<=0; out_last<=0;
            stats_valid<=0; stats_frame_id<=0; stats_cfg_version<=0;
            stats_applied_bank<=2; stats_next_bank<=2;
            pending_bank<=2; pending_bank_valid<=0;
        end else begin
            stats_valid<=0;
            if (frame_start_valid && frame_start_ready) begin
                running<=1;
                remaining<={16'd0,frame_width}*{16'd0,frame_height};
                saved_frame<=frame_id; saved_version<=cfg_version;
                active_bank<=(enable && !history_reset) ? target_bank : 3'd2;
                if (history_reset) target_bank<=2;
                // Any previous frame that drained dirty never asserted
                // clean_frame_commit; discard its deferred learning now.
                pending_bank_valid<=0;
                luminance_sum<=0; valid_count<=0;
            end
            if (stage_ready) begin
                stage_valid<=in_valid && in_ready;
                if (in_valid && in_ready) begin
                    stage_rgb<=in_rgb; stage_gray<=gray_sum[15:8];
                    stage_sample<=in_sample_valid; stage_tag<=in_pixel_tag;
                    stage_last<=remaining==1;
                    remaining<=remaining-32'd1;
                    if (remaining==1) running<=0;
                end
            end
            if (output_ready) begin
                out_valid<=stage_valid;
                if (stage_valid) begin
                    passthrough_rgb<=stage_rgb; out_sample_valid<=stage_sample;
                    out_pixel_tag<=stage_tag; out_frame_id<=saved_frame;
                    out_cfg_version<=saved_version; out_last<=stage_last;
                end
            end
            if (stage_move) begin
                if (stage_sample) begin
                    luminance_sum<=luminance_sum+{32'd0,stage_gray};
                    valid_count<=valid_count+32'd1;
                end
                if (stage_last) finalize_pending<=1;
            end
            // The accumulator has already included the final pixel. This
            // separate cycle breaks the grayscale -> sum -> compare chain.
            if (finalize_pending) begin
                finalize_pending<=0;
                pending_bank<=choose_bank(target_bank,luminance_sum,valid_count);
                pending_bank_valid<=1;
                last_luminance_sum<=luminance_sum; last_valid_count<=valid_count;
                stats_frame_id<=saved_frame; stats_cfg_version<=saved_version;
                stats_applied_bank<=active_bank;
                stats_next_bank<=choose_bank(target_bank,luminance_sum,valid_count);
                stats_valid<=1;
            end
            // Commit only after the containing R24 result frame has drained
            // without error. The pending bit is deliberately excluded from
            // idle, so this handshake cannot deadlock DRAIN.
            if (clean_frame_commit && !(frame_start_valid && frame_start_ready)) begin
                if (pending_bank_valid) begin
                    target_bank<=pending_bank;
                    pending_bank_valid<=0;
                end else if (finalize_pending) begin
                    // Defensive same-edge case: commit the just-finalized
                    // bank instead of losing it to nonblocking assignment order.
                    target_bank<=choose_bank(target_bank,luminance_sum,valid_count);
                    pending_bank_valid<=0;
                end
            end
        end
    end
endmodule
