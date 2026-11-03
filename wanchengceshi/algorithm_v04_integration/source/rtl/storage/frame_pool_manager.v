`timescale 1ns/1ps
// Generic source/result frame-slot ownership manager.
// POOL_ROLE=0 is a source pool (READY -> PROCESSING); POOL_ROLE=1 is a result
// pool (RESULT_READY -> DISPLAYING). Pixel data and DDR transactions live elsewhere.
module frame_pool_manager #(
    parameter integer NUM_SLOTS = 3,
    parameter integer ADDR_WIDTH = 64,
    parameter [ADDR_WIDTH-1:0] POOL_BASE_ADDR = {ADDR_WIDTH{1'b0}},
    parameter [ADDR_WIDTH-1:0] SLOT_STRIDE_BYTES = {ADDR_WIDTH{1'b0}},
    parameter integer POOL_ROLE = 0,
    parameter integer SLOT_W = (NUM_SLOTS <= 2) ? 1 : $clog2(NUM_SLOTS)
) (
    input wire clk,
    input wire rst_n,

    // One writer may own at most one WRITING slot at a time.
    input wire writer_alloc_valid,
    output reg writer_alloc_ready,
    output reg [SLOT_W-1:0] writer_alloc_slot,
    output reg [ADDR_WIDTH-1:0] writer_alloc_base_addr,

    input wire writer_commit_valid,
    output reg writer_commit_ready,
    // Abort returns the currently owned WRITING slot to FREE without publishing it.
    input wire writer_abort_valid,
    output reg writer_abort_ready,
    input wire [SLOT_W-1:0] writer_commit_slot,
    input wire [31:0] writer_commit_frame_id,
    input wire [31:0] writer_commit_source_frame_id,
    input wire [15:0] writer_commit_cfg_version,
    input wire [15:0] writer_commit_width,
    input wire [15:0] writer_commit_height,
    input wire [31:0] writer_commit_stride_bytes,
    input wire [7:0] writer_commit_format,
    input wire [31:0] writer_commit_process_cycles,

    // A reader claims the next READY descriptor atomically with its state change.
    input wire reader_claim_valid,
    // Indicates a published descriptor even while the single reader is busy.
    output reg reader_pending,
    output reg reader_claim_ready,
    output reg [SLOT_W-1:0] reader_claim_slot,
    output reg [ADDR_WIDTH-1:0] reader_desc_base_addr,
    output reg [31:0] reader_desc_frame_id,
    output reg [31:0] reader_desc_source_frame_id,
    output reg [15:0] reader_desc_cfg_version,
    output reg [15:0] reader_desc_width,
    output reg [15:0] reader_desc_height,
    output reg [31:0] reader_desc_stride_bytes,
    output reg [7:0] reader_desc_format,
    output reg [31:0] reader_desc_process_cycles,
    output reg [2:0] reader_desc_state,
    output reg reader_desc_valid,

    input wire reader_release_valid,
    input wire [SLOT_W-1:0] reader_release_slot,
    output reg reader_release_ready,

    // Packed debug/state output: slot i is slot_state_debug[i*3 +: 3].
    output reg [NUM_SLOTS*3-1:0] slot_state_debug
);
    localparam [2:0] ST_FREE       = 3'd0;
    localparam [2:0] ST_WRITING    = 3'd1;
    localparam [2:0] ST_READY      = 3'd2;
    localparam [2:0] ST_PROCESSING = 3'd3;
    localparam [2:0] ST_DISPLAYING = 3'd4;
    localparam [2:0] ST_RESULT_READY = 3'd5;
    localparam [2:0] ST_PUBLISHED = (POOL_ROLE == 0) ? ST_READY : ST_RESULT_READY;
    localparam [2:0] ST_READER_OWNED = (POOL_ROLE == 0) ? ST_PROCESSING : ST_DISPLAYING;

    reg [2:0] slot_state [0:NUM_SLOTS-1];
    reg [31:0] slot_frame_id [0:NUM_SLOTS-1];
    reg [31:0] slot_source_frame_id [0:NUM_SLOTS-1];
    reg [15:0] slot_cfg_version [0:NUM_SLOTS-1];
    reg [15:0] slot_width [0:NUM_SLOTS-1];
    reg [15:0] slot_height [0:NUM_SLOTS-1];
    reg [31:0] slot_stride_bytes [0:NUM_SLOTS-1];
    reg [7:0] slot_format [0:NUM_SLOTS-1];
    reg [31:0] slot_process_cycles [0:NUM_SLOTS-1];

    reg writer_active;
    reg [SLOT_W-1:0] writer_active_slot;
    reg reader_active;
    reg [SLOT_W-1:0] reader_active_slot;
    reg [SLOT_W-1:0] alloc_rr_ptr;
    reg [SLOT_W-1:0] claim_rr_ptr;

    reg alloc_found;
    reg claim_found;
    reg [SLOT_W-1:0] alloc_choice;
    reg [SLOT_W-1:0] claim_choice;
    wire [47:0] writer_commit_frame_bytes =
        {16'd0, writer_commit_stride_bytes} * {32'd0, writer_commit_height};
    wire writer_commit_extent_valid = ({16'd0, writer_commit_frame_bytes} <= SLOT_STRIDE_BYTES);
    integer k;
    integer idx;
    integer dbg_i;

    always @* begin
        alloc_found = 1'b0;
        alloc_choice = {SLOT_W{1'b0}};
        for (k = 0; k < NUM_SLOTS; k = k + 1) begin
            idx = alloc_rr_ptr + k;
            if (idx >= NUM_SLOTS)
                idx = idx - NUM_SLOTS;
            if (!alloc_found && (slot_state[idx] == ST_FREE)) begin
                alloc_found = 1'b1;
                alloc_choice = idx[SLOT_W-1:0];
            end
        end
        writer_alloc_ready = !writer_active && alloc_found;
        writer_alloc_slot = writer_alloc_ready ? alloc_choice : {SLOT_W{1'b0}};
        if (writer_alloc_ready)
            writer_alloc_base_addr = POOL_BASE_ADDR + (alloc_choice * SLOT_STRIDE_BYTES);
        else
            writer_alloc_base_addr = {ADDR_WIDTH{1'b0}};

        writer_commit_ready = 1'b0;
        if (writer_active && (writer_commit_slot == writer_active_slot) &&
            (slot_state[writer_active_slot] == ST_WRITING) &&
            !writer_abort_valid &&
            (writer_commit_width != 16'd0) && (writer_commit_height != 16'd0) &&
            (writer_commit_format == 8'h01) &&
            (writer_commit_stride_bytes >= ({16'd0, writer_commit_width} << 2)) &&
            writer_commit_extent_valid
            )
            writer_commit_ready = 1'b1;

        writer_abort_ready = writer_active &&
            (slot_state[writer_active_slot] == ST_WRITING);

        claim_found = 1'b0;
        claim_choice = {SLOT_W{1'b0}};
        for (k = 0; k < NUM_SLOTS; k = k + 1) begin
            idx = claim_rr_ptr + k;
            if (idx >= NUM_SLOTS)
                idx = idx - NUM_SLOTS;
            if (!claim_found && (slot_state[idx] == ST_PUBLISHED)) begin
                claim_found = 1'b1;
                claim_choice = idx[SLOT_W-1:0];
            end
        end
        reader_pending = claim_found;
        reader_claim_ready = !reader_active && claim_found;
        reader_claim_slot = reader_claim_ready ? claim_choice : {SLOT_W{1'b0}};
        reader_desc_valid = reader_claim_ready;
        reader_desc_state = ST_READER_OWNED;
        reader_desc_base_addr = {ADDR_WIDTH{1'b0}};
        reader_desc_frame_id = 32'd0;
        reader_desc_source_frame_id = 32'd0;
        reader_desc_cfg_version = 16'd0;
        reader_desc_width = 16'd0;
        reader_desc_height = 16'd0;
        reader_desc_stride_bytes = 32'd0;
        reader_desc_format = 8'd0;
        reader_desc_process_cycles = 32'd0;
        if (reader_claim_ready) begin
            reader_desc_base_addr = POOL_BASE_ADDR + (claim_choice * SLOT_STRIDE_BYTES);
            reader_desc_frame_id = slot_frame_id[claim_choice];
            reader_desc_source_frame_id = slot_source_frame_id[claim_choice];
            reader_desc_cfg_version = slot_cfg_version[claim_choice];
            reader_desc_width = slot_width[claim_choice];
            reader_desc_height = slot_height[claim_choice];
            reader_desc_stride_bytes = slot_stride_bytes[claim_choice];
            reader_desc_format = slot_format[claim_choice];
            reader_desc_process_cycles = slot_process_cycles[claim_choice];
        end

        reader_release_ready = 1'b0;
        if (reader_active && (reader_release_slot == reader_active_slot) &&
            (slot_state[reader_active_slot] == ST_READER_OWNED))
            reader_release_ready = 1'b1;

        slot_state_debug = {(NUM_SLOTS*3){1'b0}};
        for (dbg_i = 0; dbg_i < NUM_SLOTS; dbg_i = dbg_i + 1)
            slot_state_debug = slot_state_debug |
                ({{(NUM_SLOTS*3-3){1'b0}}, slot_state[dbg_i]} << (dbg_i*3));
    end

    integer reset_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            writer_active <= 1'b0;
            writer_active_slot <= {SLOT_W{1'b0}};
            reader_active <= 1'b0;
            reader_active_slot <= {SLOT_W{1'b0}};
            alloc_rr_ptr <= {SLOT_W{1'b0}};
            claim_rr_ptr <= {SLOT_W{1'b0}};
            for (reset_i = 0; reset_i < NUM_SLOTS; reset_i = reset_i + 1) begin
                slot_state[reset_i] <= ST_FREE;
                slot_frame_id[reset_i] <= 32'd0;
                slot_source_frame_id[reset_i] <= 32'd0;
                slot_cfg_version[reset_i] <= 16'd0;
                slot_width[reset_i] <= 16'd0;
                slot_height[reset_i] <= 16'd0;
                slot_stride_bytes[reset_i] <= 32'd0;
                slot_format[reset_i] <= 8'd0;
                slot_process_cycles[reset_i] <= 32'd0;
            end
        end else begin
            if (writer_alloc_valid && writer_alloc_ready) begin
                slot_state[alloc_choice] <= ST_WRITING;
                writer_active <= 1'b1;
                writer_active_slot <= alloc_choice;
                if (alloc_choice == NUM_SLOTS-1)
                    alloc_rr_ptr <= {SLOT_W{1'b0}};
                else
                    alloc_rr_ptr <= alloc_choice + 1'b1;
            end

            if (writer_commit_valid && writer_commit_ready) begin
                slot_state[writer_active_slot] <= ST_PUBLISHED;
                slot_frame_id[writer_active_slot] <= writer_commit_frame_id;
                slot_source_frame_id[writer_active_slot] <= writer_commit_source_frame_id;
                slot_cfg_version[writer_active_slot] <= writer_commit_cfg_version;
                slot_width[writer_active_slot] <= writer_commit_width;
                slot_height[writer_active_slot] <= writer_commit_height;
                slot_stride_bytes[writer_active_slot] <= writer_commit_stride_bytes;
                slot_format[writer_active_slot] <= writer_commit_format;
                slot_process_cycles[writer_active_slot] <= writer_commit_process_cycles;
                writer_active <= 1'b0;
            end

            if (writer_abort_valid && writer_abort_ready) begin
                slot_state[writer_active_slot] <= ST_FREE;
                writer_active <= 1'b0;
            end

            if (reader_claim_valid && reader_claim_ready) begin
                slot_state[claim_choice] <= ST_READER_OWNED;
                reader_active <= 1'b1;
                reader_active_slot <= claim_choice;
                if (claim_choice == NUM_SLOTS-1)
                    claim_rr_ptr <= {SLOT_W{1'b0}};
                else
                    claim_rr_ptr <= claim_choice + 1'b1;
            end

            if (reader_release_valid && reader_release_ready) begin
                slot_state[reader_active_slot] <= ST_FREE;
                reader_active <= 1'b0;
            end
        end
    end
endmodule
