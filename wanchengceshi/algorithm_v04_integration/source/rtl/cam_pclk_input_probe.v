// Observe an asynchronous camera PCLK as sampled data in the independent
// system-clock domain. This diagnoses pad activity without using cam_pclk as
// a fabric clock; it is not a substitute for a dedicated clock input.
module cam_pclk_input_probe (
    input             sys_clk,
    input             rst_n,
    input             async_pclk,
    output reg        pclk_seen_sys,
    output reg [31:0] transition_count_sys
);
    reg       pclk_meta;
    reg       pclk_sync;
    reg       pclk_sync_d;
    reg [2:0] sample_valid;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            pclk_meta          <= 1'b0;
            pclk_sync          <= 1'b0;
            pclk_sync_d        <= 1'b0;
            sample_valid       <= 3'b000;
            pclk_seen_sys      <= 1'b0;
            transition_count_sys <= 32'd0;
        end else begin
            pclk_meta    <= async_pclk;
            pclk_sync    <= pclk_meta;
            pclk_sync_d  <= pclk_sync;
            sample_valid <= {sample_valid[1:0], 1'b1};

            // Allow the synchronizer to establish its initial level before
            // counting changes, avoiding a false event if the pad starts high.
            if (sample_valid[2] && (pclk_sync != pclk_sync_d)) begin
                pclk_seen_sys <= 1'b1;
                transition_count_sys <= transition_count_sys + 1'b1;
            end
        end
    end
endmodule
