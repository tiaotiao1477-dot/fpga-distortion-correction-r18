module camera_readpath_diag (
    input wire core_clk,
    input wire ddr_init_done,
    input wire rd_cmd_en,
    input wire rd_cmd_ready,
    input wire axi_arvalid,
    input wire axi_arready,
    output reg [31:0] rd_request_count,
    output reg [31:0] arvalid_cycle_count,
    output reg [31:0] ar_handshake_count,
    output reg [31:0] busy_request_count
);
    always @(posedge core_clk) begin
        if (!ddr_init_done) begin
            rd_request_count <= 32'd0;
            arvalid_cycle_count <= 32'd0;
            ar_handshake_count <= 32'd0;
            busy_request_count <= 32'd0;
        end else begin
            if (rd_cmd_en)
                rd_request_count <= rd_request_count + 1'b1;
            if (rd_cmd_en && !rd_cmd_ready)
                busy_request_count <= busy_request_count + 1'b1;
            if (axi_arvalid)
                arvalid_cycle_count <= arvalid_cycle_count + 1'b1;
            if (axi_arvalid && axi_arready)
                ar_handshake_count <= ar_handshake_count + 1'b1;
        end
    end
endmodule
