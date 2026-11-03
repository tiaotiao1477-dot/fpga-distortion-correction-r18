// Board-only replacement. Common asynchronous assertion; local third-edge release.
// Keep the existing three-bit debug/output interface and startup value.
module reset_sync(input clk,arst_n,output [2:0] sync_state);
    reg [2:0] sync_ff=3'b000;
    always @(posedge clk or negedge arst_n)
        if(!arst_n)sync_ff<=3'b000;
        else sync_ff<={sync_ff[1:0],1'b1};
    assign sync_state=sync_ff;
endmodule
