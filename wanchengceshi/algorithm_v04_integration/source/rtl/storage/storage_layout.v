`timescale 1ns/1ps
// Byte-address layout helper for RGB888 pixels stored in 32-bit words.
// The physical DDR address conversion is intentionally outside this module.
module storage_layout #(
    parameter integer NUM_SLOTS = 3,
    parameter integer MEM_BEAT_BYTES = 32,
    parameter integer SLOT_W = (NUM_SLOTS <= 2) ? 1 : $clog2(NUM_SLOTS)
) (
    input wire [15:0] image_width,
    input wire [15:0] image_height,
    input wire [63:0] usable_base_addr,
    input wire [63:0] usable_size_bytes,
    input wire [63:0] pool_base_addr,
    input wire [SLOT_W-1:0] slot_index,

    output reg [31:0] stride_bytes,
    output reg [63:0] frame_size_bytes,
    output reg [63:0] pool_size_bytes,
    output reg [63:0] pool_end_addr,
    output reg layout_valid,
    output reg arithmetic_overflow,
    output reg slot_base_valid,
    output reg [63:0] slot_base_addr
);
    reg [63:0] row_bytes_calc;
    reg [63:0] stride_calc;
    reg [63:0] frame_calc;
    reg [63:0] pool_calc;
    reg [63:0] pool_end_calc;
    reg [63:0] usable_end_calc;
    reg frame_mul_overflow;
    reg pool_mul_overflow;
    reg pool_add_overflow;
    reg usable_add_overflow;

    always @* begin
        row_bytes_calc = {48'd0, image_width} << 2;
        stride_calc = 64'd0;
        if (MEM_BEAT_BYTES > 0)
            stride_calc = ((row_bytes_calc + MEM_BEAT_BYTES - 1) / MEM_BEAT_BYTES) * MEM_BEAT_BYTES;

        frame_calc = stride_calc * image_height;
        frame_mul_overflow = 1'b0;
        if ((image_height != 16'd0) && ((frame_calc / image_height) != stride_calc))
            frame_mul_overflow = 1'b1;

        pool_calc = frame_calc * NUM_SLOTS;
        pool_mul_overflow = 1'b0;
        if ((NUM_SLOTS > 0) && ((pool_calc / NUM_SLOTS) != frame_calc))
            pool_mul_overflow = 1'b1;

        pool_end_calc = pool_base_addr + pool_calc;
        pool_add_overflow = (pool_end_calc < pool_base_addr);
        usable_end_calc = usable_base_addr + usable_size_bytes;
        usable_add_overflow = (usable_end_calc < usable_base_addr);

        stride_bytes = stride_calc[31:0];
        frame_size_bytes = frame_calc;
        pool_size_bytes = pool_calc;
        pool_end_addr = pool_end_calc;
        arithmetic_overflow = frame_mul_overflow | pool_mul_overflow |
                              pool_add_overflow | usable_add_overflow;

        layout_valid = 1'b1;
        if ((image_width == 16'd0) || (image_height == 16'd0) ||
            (MEM_BEAT_BYTES <= 0) || (NUM_SLOTS <= 0) ||
            frame_mul_overflow || pool_mul_overflow || pool_add_overflow || usable_add_overflow ||
            ((pool_base_addr % MEM_BEAT_BYTES) != 0) ||
            (pool_base_addr < usable_base_addr) || (pool_end_calc > usable_end_calc))
            layout_valid = 1'b0;

        slot_base_valid = layout_valid && (slot_index < NUM_SLOTS);
        if (slot_base_valid)
            slot_base_addr = pool_base_addr + (slot_index * frame_calc);
        else
            slot_base_addr = 64'd0;
    end
endmodule
