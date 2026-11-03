module camera_diag_status_sync (
    input wire pix_clk,
    input wire rst_n,
    input wire [4:0] status_async,
    output reg [4:0] status_sync
);
    reg [4:0] status_meta;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            status_meta <= 5'b0;
            status_sync <= 5'b0;
        end else begin
            status_meta <= status_async;
            status_sync <= status_meta;
        end
    end
endmodule
