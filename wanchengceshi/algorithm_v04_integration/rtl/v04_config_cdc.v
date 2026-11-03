// Bundled-data toggle handshake. Source holds the complete snapshot until
// destination consumption; three destination synchronizer stages precede capture.
// Both resets must assert for a link restart; unacknowledged packets are dropped.
module v04_config_cdc #(parameter integer WIDTH=768)(
    input src_clk,src_rst_n,src_valid,output src_ready,input [WIDTH-1:0] src_data,
    input dst_clk,dst_rst_n,output reg dst_valid,input dst_ready,output reg [WIDTH-1:0] dst_data
);
    reg [WIDTH-1:0] snapshot;
    reg request_toggle,ack_toggle,seen;
    (* async_reg="true" *) reg [1:0] ack_sync;
    (* async_reg="true" *) reg [2:0] request_sync;
    assign src_ready=src_rst_n&&(request_toggle==ack_sync[1]);
    always @(posedge src_clk)if(src_valid&&src_ready)snapshot<=src_data;
    always @(posedge src_clk or negedge src_rst_n)begin
        if(!src_rst_n)begin request_toggle<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[0],ack_toggle};
            if(src_valid&&src_ready)request_toggle<=~request_toggle;
        end
    end
    always @(posedge dst_clk)if(!dst_valid&&request_sync[2]!=seen)dst_data<=snapshot;
    always @(posedge dst_clk or negedge dst_rst_n)begin
        if(!dst_rst_n)begin dst_valid<=0;seen<=0;ack_toggle<=0;request_sync<=0;end
        else begin
            request_sync<={request_sync[1:0],request_toggle};
            if(!dst_valid&&request_sync[2]!=seen)begin dst_valid<=1;seen<=request_sync[2];end
            if(dst_valid&&dst_ready)begin dst_valid<=0;ack_toggle<=seen;end
        end
    end
endmodule
