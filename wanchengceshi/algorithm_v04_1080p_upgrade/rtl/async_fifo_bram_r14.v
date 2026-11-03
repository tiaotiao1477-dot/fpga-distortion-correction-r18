`timescale 1ns/1ps
// FWFT async FIFO using synchronous dual-clock RAM plus two output entries.
// The writer observes CONSUMED pointers, so advertised capacity stays DEPTH.
// Fetching into the output buffer never frees a writer slot early.
module async_fifo #(
    parameter integer DATA_WIDTH=32,DEPTH=1024,
    parameter integer ADDR_WIDTH=$clog2(DEPTH),PTR_WIDTH=ADDR_WIDTH+1,
    parameter integer BRAM_MIN_DEPTH=256
)(
    input wire wr_clk,rd_clk,rst_n,
    input wire wr_valid,output wire wr_ready,input wire [DATA_WIDTH-1:0] wr_data,
    output wire rd_valid,input wire rd_ready,output wire [DATA_WIDTH-1:0] rd_data
);
    generate if(DEPTH<BRAM_MIN_DEPTH)begin:g_small
        async_fifo_small_r14 #(.DATA_WIDTH(DATA_WIDTH),.DEPTH(DEPTH),.ADDR_WIDTH(ADDR_WIDTH),.PTR_WIDTH(PTR_WIDTH)) small_fifo(.wr_clk(wr_clk),.rd_clk(rd_clk),.rst_n(rst_n),.wr_valid(wr_valid),.wr_ready(wr_ready),.wr_data(wr_data),.rd_valid(rd_valid),.rd_ready(rd_ready),.rd_data(rd_data));
    end else begin:g_bram
        reg [DATA_WIDTH-1:0] memory[0:DEPTH-1];
        reg [DATA_WIDTH-1:0] fetched_word,output_words[0:1];
        reg [PTR_WIDTH-1:0] write_binary,write_gray;
        reg [PTR_WIDTH-1:0] consumed_binary,consumed_gray;
        reg [PTR_WIDTH-1:0] fetch_binary,fetch_gray;
        reg [PTR_WIDTH-1:0] consumed_sync1,consumed_sync2,write_sync1,write_sync2;
        reg full;
        reg [1:0] output_count;
        reg output_head,output_tail,fetch_pending;
        wire write_fire=wr_valid&&wr_ready;
        wire pop=rd_valid&&rd_ready;
        wire [PTR_WIDTH-1:0] next_write=write_binary+write_fire;
        wire [PTR_WIDTH-1:0] next_write_gray=(next_write>>1)^next_write;
        wire [PTR_WIDTH-1:0] next_consumed=consumed_binary+pop;
        wire [PTR_WIDTH-1:0] next_fetch=fetch_binary+1'b1;
        wire ram_empty=fetch_gray==write_sync2;
        wire [2:0] reserved={1'b0,output_count}+{2'b0,fetch_pending};
        wire issue=rst_n&&!ram_empty&&(reserved-{2'b0,pop}<3'd2);
        assign wr_ready=rst_n&&!full;
        assign rd_valid=rst_n&&output_count!=0;
        assign rd_data=output_words[output_head];
        always @(posedge wr_clk)if(rst_n&&write_fire)memory[write_binary[ADDR_WIDTH-1:0]]<=wr_data;
        // Standard synchronous read template: no reset on RAM data.
        always @(posedge rd_clk)if(issue)fetched_word<=memory[fetch_binary[ADDR_WIDTH-1:0]];
        always @(posedge wr_clk or negedge rst_n)
            if(!rst_n)begin write_binary<=0;write_gray<=0;consumed_sync1<=0;consumed_sync2<=0;full<=0;end
            else begin
                consumed_sync1<=consumed_gray;consumed_sync2<=consumed_sync1;
                write_binary<=next_write;write_gray<=next_write_gray;
                full<=next_write_gray=={~consumed_sync2[PTR_WIDTH-1:PTR_WIDTH-2],consumed_sync2[PTR_WIDTH-3:0]};
            end
        always @(posedge rd_clk or negedge rst_n)
            if(!rst_n)begin
                consumed_binary<=0;consumed_gray<=0;fetch_binary<=0;fetch_gray<=0;
                write_sync1<=0;write_sync2<=0;output_count<=0;output_head<=0;output_tail<=0;fetch_pending<=0;
            end else begin
                write_sync1<=write_gray;write_sync2<=write_sync1;
                consumed_binary<=next_consumed;consumed_gray<=(next_consumed>>1)^next_consumed;
                fetch_pending<=issue;
                if(issue)begin fetch_binary<=next_fetch;fetch_gray<=(next_fetch>>1)^next_fetch;end
                case({fetch_pending,pop})
                    2'b10:output_count<=output_count+1'b1;
                    2'b01:output_count<=output_count-1'b1;
                    default:;
                endcase
                if(pop)output_head<=!output_head;
                if(fetch_pending)begin output_words[output_tail]<=fetched_word;output_tail<=!output_tail;end
            end
    end endgenerate
endmodule

`timescale 1ns/1ps
// Gray-pointer asynchronous FIFO for camera-pixel to DDR-clock CDC.
// DEPTH must be a power of two and at least four entries. Read data is FWFT:
// it remains stable until rd_valid && rd_ready advances the read pointer.
module async_fifo_small_r14 #(
    parameter integer DATA_WIDTH = 32,
    parameter integer DEPTH = 1024,
    parameter integer ADDR_WIDTH = $clog2(DEPTH),
    parameter integer PTR_WIDTH = ADDR_WIDTH + 1
) (
    input wire wr_clk,
    input wire rd_clk,
    input wire rst_n,

    input wire wr_valid,
    output wire wr_ready,
    input wire [DATA_WIDTH-1:0] wr_data,

    output wire rd_valid,
    input wire rd_ready,
    output wire [DATA_WIDTH-1:0] rd_data
);
    reg [DATA_WIDTH-1:0] storage [0:DEPTH-1];

    reg [PTR_WIDTH-1:0] wr_binary;
    reg [PTR_WIDTH-1:0] wr_gray;
    reg [PTR_WIDTH-1:0] rd_binary;
    reg [PTR_WIDTH-1:0] rd_gray;
    reg [PTR_WIDTH-1:0] rd_gray_sync1;
    reg [PTR_WIDTH-1:0] rd_gray_sync2;
    reg [PTR_WIDTH-1:0] wr_gray_sync1;
    reg [PTR_WIDTH-1:0] wr_gray_sync2;
    reg fifo_full;
    reg fifo_empty;

    wire wr_accept = wr_valid && wr_ready;
    wire rd_accept = rd_valid && rd_ready;
    wire [PTR_WIDTH-1:0] wr_binary_next = wr_binary + wr_accept;
    wire [PTR_WIDTH-1:0] rd_binary_next = rd_binary + rd_accept;
    wire [PTR_WIDTH-1:0] wr_gray_next = (wr_binary_next >> 1) ^ wr_binary_next;
    wire [PTR_WIDTH-1:0] rd_gray_next = (rd_binary_next >> 1) ^ rd_binary_next;
    wire fifo_full_next =
        (wr_gray_next == {~rd_gray_sync2[PTR_WIDTH-1:PTR_WIDTH-2],
                          rd_gray_sync2[PTR_WIDTH-3:0]});
    wire fifo_empty_next = (rd_gray_next == wr_gray_sync2);

    assign wr_ready = !fifo_full;
    assign rd_valid = !fifo_empty;
    assign rd_data = storage[rd_binary[ADDR_WIDTH-1:0]];

    // Read pointer synchronized into the write clock domain.
    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_gray_sync1 <= {PTR_WIDTH{1'b0}};
            rd_gray_sync2 <= {PTR_WIDTH{1'b0}};
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // Write pointer synchronized into the read clock domain.
    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_gray_sync1 <= {PTR_WIDTH{1'b0}};
            wr_gray_sync2 <= {PTR_WIDTH{1'b0}};
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_binary <= {PTR_WIDTH{1'b0}};
            wr_gray <= {PTR_WIDTH{1'b0}};
            fifo_full <= 1'b0;
        end else begin
            if (wr_accept)
                storage[wr_binary[ADDR_WIDTH-1:0]] <= wr_data;
            wr_binary <= wr_binary_next;
            wr_gray <= wr_gray_next;
            fifo_full <= fifo_full_next;
        end
    end

    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_binary <= {PTR_WIDTH{1'b0}};
            rd_gray <= {PTR_WIDTH{1'b0}};
            fifo_empty <= 1'b1;
        end else begin
            rd_binary <= rd_binary_next;
            rd_gray <= rd_gray_next;
            fifo_empty <= fifo_empty_next;
        end
    end
endmodule
