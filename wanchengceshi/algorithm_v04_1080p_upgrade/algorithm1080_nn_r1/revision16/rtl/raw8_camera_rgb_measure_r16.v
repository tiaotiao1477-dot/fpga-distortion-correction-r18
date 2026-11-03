`timescale 1ns/1ps
// Error-aware offline RAW candidate. A camera-domain END token follows every
// source frame and carries capture/drop status. Last RGB is held until clean
// END confirmation; frame_abort requires the downstream frame writer to cancel
// all earlier pixels of that frame. Counter clock domains match candidate r1.
module raw8_camera_rgb_candidate_r3 #(
    parameter integer WIDTH=1920,HEIGHT=1080,FIFO_DEPTH=2048,BAYER_PHASE=0,
    parameter [15:0] CONFIG_VERSION=16'd7
)(
    input wire pclk,clk,rst_n,vsync,href,input wire [7:0] data,
    output wire out_valid,input wire out_ready,output wire [23:0] out_rgb,
    output wire [15:0] out_x,out_y,output wire [31:0] out_frame_id,out_pixel_tag,
    output wire [15:0] out_cfg_version,output wire out_last,out_sof,out_eol,
    output wire [31:0] capture_errors,output reg [31:0] fifo_overflow_errors,
    output reg [31:0] protocol_errors,input_pixels,started_frames,
    output reg frame_abort,
    output reg [31:0] debug_abort_branches,debug_abort_detail,
    input wire [63:0] measurement_time_gray,
    output reg measurement_start_event,measurement_start_time_valid,
    output reg [63:0] measurement_start_time,
    output reg [31:0] measurement_start_sensor_id,
    output reg measurement_end_event,
    output reg [63:0] measurement_end_time,
    output reg [31:0] measurement_end_sensor_id,
    output reg [31:0] measurement_input_frames,measurement_errors
);
    // Asynchronous assertion is common; release is local to each clock.
    (* ASYNC_REG = "TRUE" *) reg [2:0] camera_reset_sync,core_reset_sync;
    always @(posedge pclk or negedge rst_n)
        if(!rst_n)camera_reset_sync<=0;
        else camera_reset_sync<={camera_reset_sync[1:0],1'b1};
    always @(posedge clk or negedge rst_n)
        if(!rst_n)core_reset_sync<=0;
        else core_reset_sync<={core_reset_sync[1:0],1'b1};
    wire camera_rst_n=camera_reset_sync[2];
    wire core_rst_n=core_reset_sync[2];
    wire [7:0] raw;wire raw_valid,raw_sof,raw_eol,raw_eof,frame_active;
    wire [15:0] raw_x,raw_y;
    raw8_dvp_capture_r2 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) capture(
        .pclk(pclk),.rst_n(camera_rst_n),.vsync(vsync),.href(href),.data(data),
        .raw_data(raw),.pixel_valid(raw_valid),.px_x(raw_x),.px_y(raw_y),
        .sof(raw_sof),.eol(raw_eol),.eof(raw_eof),.frame_active(frame_active),.error_count(capture_errors));
    reg previous_active,end_pending,end_dirty,frame_dropped,drop_frame;
    reg [3:0] end_diag_flags; // R8_OBSERVER: capture changed, dropped samples, drop-frame, pending END
    reg [31:0] error_baseline;
    wire frame_end_event=previous_active&&!frame_active;
    wire fifo_wr_ready,fifo_valid,fifo_ready;wire [11:0] fifo_data;
    wire write_raw=raw_valid&&!end_pending&&(!drop_frame||raw_sof);
    wire [11:0] write_data=end_pending?{1'b1,end_dirty,end_diag_flags,6'd0}:{2'd0,raw_sof,raw_eof,raw};
    raw_async_fifo_dualreset_r1 #(.DATA_WIDTH(12),.DEPTH(FIFO_DEPTH)) raw_fifo(
        .wr_clk(pclk),.rd_clk(clk),.wr_rst_n(camera_rst_n),.rd_rst_n(core_rst_n),.wr_valid(end_pending||write_raw),.wr_ready(fifo_wr_ready),
        .wr_data(write_data),.rd_valid(fifo_valid),.rd_ready(fifo_ready),.rd_data(fifo_data));
    always @(posedge pclk or negedge camera_rst_n)begin
        if(!camera_rst_n)begin
            input_pixels<=0;fifo_overflow_errors<=0;previous_active<=0;end_pending<=0;end_dirty<=0;
            frame_dropped<=0;drop_frame<=0;error_baseline<=0;
            end_diag_flags<=0; // R8_OBSERVER
        end else begin
            previous_active<=frame_active;
            if(end_pending&&fifo_wr_ready)end_pending<=0;
            if(raw_valid&&raw_sof)begin
                error_baseline<=capture_errors;frame_dropped<=0;
                // Preserve the pending terminal token before accepting another
                // frame. Dropping a colliding new SOF discards that whole frame.
                drop_frame<=end_pending||!fifo_wr_ready;
            end
            if(raw_valid)begin
                input_pixels<=input_pixels+1'b1;
                if(!fifo_wr_ready||end_pending||(drop_frame&&!raw_sof))begin
                    fifo_overflow_errors<=fifo_overflow_errors+1'b1;frame_dropped<=1;
                end
            end
            if(frame_end_event)begin
                end_pending<=1;
                end_diag_flags<=(end_pending?end_diag_flags:4'd0)|{capture_errors!=error_baseline,frame_dropped,drop_frame,end_pending}; // R8_OBSERVER
                end_dirty<=(end_pending&&end_dirty)||capture_errors!=error_baseline||frame_dropped||drop_frame;
            end
        end
    end
    localparam SEEK=0,STREAM=1,WAIT_END=2,WAIT_OUTPUT=3;
    reg [1:0] state;reg [31:0] input_tag;
    reg reset_core,frame_confirmed;
    wire core_idle,start_ready,input_ready,core_valid,core_last;
    wire end_token=fifo_data[11];
    wire start_valid=state==SEEK&&fifo_valid&&!end_token&&fifo_data[9]&&!reset_core;
    wire packet_bad=state==STREAM&&fifo_valid&&!end_token&&
        ((fifo_data[9]&&input_tag!=0)||(fifo_data[8]!=(input_tag==WIDTH*HEIGHT-1)));
    wire input_valid=state==STREAM&&fifo_valid&&!end_token&&!packet_bad;
    assign fifo_ready=end_token?(state!=WAIT_OUTPUT):
        state==SEEK?!fifo_data[9]:state==STREAM?(packet_bad?!fifo_data[9]:input_ready):1'b0;
    assign out_valid=core_valid&&(!core_last||frame_confirmed);
    assign out_last=core_last;
    raw8_to_rgb_stream #(.MAX_WIDTH(WIDTH),.BAYER_PHASE(BAYER_PHASE)) color(
        .clk(clk),.rst_n(core_rst_n&&!reset_core),.frame_start_valid(start_valid),.frame_start_ready(start_ready),
        .frame_width(WIDTH[15:0]),.frame_height(HEIGHT[15:0]),.frame_id(started_frames+32'd1),
        .cfg_version(CONFIG_VERSION),.in_valid(input_valid),.in_ready(input_ready),.in_raw(fifo_data[7:0]),
        .in_pixel_tag(input_tag),.out_valid(core_valid),.out_ready(out_ready&&(!core_last||frame_confirmed)),
        .out_rgb(out_rgb),.out_x(out_x),.out_y(out_y),.out_last(core_last),.out_frame_id(out_frame_id),
        .out_pixel_tag(out_pixel_tag),.out_cfg_version(out_cfg_version),.idle(core_idle));
    // R16_OBSERVER_DECL_BEGIN
    // Observer-only sidebands. These wires never affect the 12-bit video FIFO.
    (* ASYNC_REG = "TRUE" *) reg [63:0] measurement_time_gray_cam_meta;
    (* ASYNC_REG = "TRUE" *) reg [63:0] measurement_time_gray_cam_sync1;
    (* ASYNC_REG = "TRUE" *) reg [63:0] measurement_time_gray_cam_sync2;
    reg [31:0] measurement_sensor_frame_id;
    reg [31:0] measurement_sensor_frame_gray;
    reg [31:0] measurement_cam_overflow_count;
    reg [31:0] measurement_cam_overflow_gray;
    (* ASYNC_REG = "TRUE" *) reg [31:0] measurement_frames_gray_core_meta;
    (* ASYNC_REG = "TRUE" *) reg [31:0] measurement_frames_gray_core_sync1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] measurement_frames_gray_core_sync2;
    (* ASYNC_REG = "TRUE" *) reg [31:0] measurement_overflow_gray_core_meta;
    (* ASYNC_REG = "TRUE" *) reg [31:0] measurement_overflow_gray_core_sync1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] measurement_overflow_gray_core_sync2;
    reg [31:0] measurement_missing_sof_count;
    reg measurement_alignment_invalid;
    wire measurement_sof_wr_valid=write_raw&&raw_sof&&fifo_wr_ready;
    wire measurement_eof_wr_valid=write_raw&&raw_eof&&fifo_wr_ready;
    wire measurement_sof_wr_ready,measurement_eof_wr_ready;
    wire measurement_sof_rd_valid,measurement_eof_rd_valid;
    wire measurement_sof_rd_ready=start_valid&&start_ready;
    wire measurement_eof_rd_ready=1'b1;
    wire [95:0] measurement_sof_rd_data,measurement_eof_rd_data;
    wire measurement_camera_overflow =
        (measurement_sof_wr_valid && !measurement_sof_wr_ready) ||
        (measurement_eof_wr_valid && !measurement_eof_wr_ready);
    wire [31:0] measurement_sensor_id_for_record = raw_sof ?
        (measurement_sensor_frame_id + 32'd1) : measurement_sensor_frame_id;
    wire [63:0] measurement_time_cam_binary;
    wire [31:0] measurement_frames_core_binary;
    wire [31:0] measurement_overflow_core_binary;
    wire measurement_start_core_fire=start_valid&&start_ready;
    wire measurement_alignment_bad_now = measurement_alignment_invalid ||
        (measurement_overflow_core_binary != 32'd0) ||
        (measurement_start_core_fire && !measurement_sof_rd_valid);

    function [31:0] measurement_bin_to_gray32;
        input [31:0] binary_value;
        begin measurement_bin_to_gray32=(binary_value>>1)^binary_value; end
    endfunction
    // Parallel-prefix XOR decode keeps the wide timestamp decode to log2(width)
    // logic levels before its local event register/FIFO write.
    function [31:0] measurement_gray_to_bin32;
        input [31:0] gray_value;
        reg [31:0] stage1,stage2,stage3,stage4,stage5;
        begin
            stage1=gray_value^(gray_value>>1);
            stage2=stage1^(stage1>>2);
            stage3=stage2^(stage2>>4);
            stage4=stage3^(stage3>>8);
            stage5=stage4^(stage4>>16);
            measurement_gray_to_bin32=stage5;
        end
    endfunction
    function [63:0] measurement_gray_to_bin64;
        input [63:0] gray_value;
        reg [63:0] stage1,stage2,stage3,stage4,stage5,stage6;
        begin
            stage1=gray_value^(gray_value>>1);
            stage2=stage1^(stage1>>2);
            stage3=stage2^(stage2>>4);
            stage4=stage3^(stage3>>8);
            stage5=stage4^(stage4>>16);
            stage6=stage5^(stage5>>32);
            measurement_gray_to_bin64=stage6;
        end
    endfunction
    assign measurement_time_cam_binary=measurement_gray_to_bin64(measurement_time_gray_cam_sync2);
    assign measurement_frames_core_binary =
        measurement_gray_to_bin32(measurement_frames_gray_core_sync2);
    assign measurement_overflow_core_binary =
        measurement_gray_to_bin32(measurement_overflow_gray_core_sync2);

    raw_async_fifo_dualreset_r1 #(.DATA_WIDTH(96),.DEPTH(16)) measurement_sof_fifo(
        .wr_clk(pclk),.rd_clk(clk),.wr_rst_n(camera_rst_n),.rd_rst_n(core_rst_n),
        .wr_valid(measurement_sof_wr_valid),.wr_ready(measurement_sof_wr_ready),
        .wr_data({measurement_sensor_id_for_record,measurement_time_cam_binary}),
        .rd_valid(measurement_sof_rd_valid),.rd_ready(measurement_sof_rd_ready),
        .rd_data(measurement_sof_rd_data));
    raw_async_fifo_dualreset_r1 #(.DATA_WIDTH(96),.DEPTH(16)) measurement_eof_fifo(
        .wr_clk(pclk),.rd_clk(clk),.wr_rst_n(camera_rst_n),.rd_rst_n(core_rst_n),
        .wr_valid(measurement_eof_wr_valid),.wr_ready(measurement_eof_wr_ready),
        .wr_data({measurement_sensor_id_for_record,measurement_time_cam_binary}),
        .rd_valid(measurement_eof_rd_valid),.rd_ready(measurement_eof_rd_ready),
        .rd_data(measurement_eof_rd_data));
    // R16_OBSERVER_DECL_END
    assign out_sof=out_pixel_tag==0;assign out_eol=out_x==WIDTH-1;
    always @(posedge clk or negedge core_rst_n)begin
        if(!core_rst_n)begin
            state<=SEEK;input_tag<=0;reset_core<=0;frame_confirmed<=0;
            protocol_errors<=0;started_frames<=0;frame_abort<=0;
            debug_abort_branches<=0;debug_abort_detail<=0; // R8_OBSERVER
        end else begin
            reset_core<=0;frame_abort<=0;
            if(state==SEEK&&fifo_valid&&fifo_ready&&!end_token)protocol_errors<=protocol_errors+1'b1;
            if(start_valid&&start_ready)begin
                state<=STREAM;input_tag<=0;frame_confirmed<=0;started_frames<=started_frames+1'b1;
            end
            if(packet_bad) // R8_OBSERVER
                debug_abort_branches[7:0]<=debug_abort_branches[7:0]+8'd1; // R8_OBSERVER
            if(state==WAIT_END&&fifo_valid&&!end_token&&fifo_data[9]) // R8_OBSERVER
                debug_abort_branches[15:8]<=debug_abort_branches[15:8]+8'd1; // R8_OBSERVER
            if(packet_bad||(state==WAIT_END&&fifo_valid&&!end_token&&fifo_data[9])) // R8_OBSERVER
                debug_abort_detail<={2'b0,input_tag[21:0],4'b0,2'b0,(state==WAIT_END),packet_bad}; // R8_OBSERVER
            if(packet_bad||(state==WAIT_END&&fifo_valid&&!end_token&&fifo_data[9]))begin
                protocol_errors<=protocol_errors+1'b1;frame_abort<=1;reset_core<=1;frame_confirmed<=0;state<=SEEK;
            end else if(input_valid&&input_ready)begin
                input_tag<=input_tag+1'b1;
                if(input_tag==WIDTH*HEIGHT-1)state<=WAIT_END;
            end
            if(fifo_valid&&fifo_ready&&end_token&&(state==STREAM||state==WAIT_END))begin
                if(fifo_data[10]) // R8_OBSERVER
                    debug_abort_branches[23:16]<=debug_abort_branches[23:16]+8'd1; // R8_OBSERVER
                if(input_tag!=WIDTH*HEIGHT) // R8_OBSERVER
                    debug_abort_branches[31:24]<=debug_abort_branches[31:24]+8'd1; // R8_OBSERVER
                if(fifo_data[10]||input_tag!=WIDTH*HEIGHT) // R8_OBSERVER
                    debug_abort_detail<={2'b0,input_tag[21:0],fifo_data[9:6],(input_tag!=WIDTH*HEIGHT),fifo_data[10],2'b0}; // R8_OBSERVER
                if(fifo_data[10]||input_tag!=WIDTH*HEIGHT)begin
                    protocol_errors<=protocol_errors+1'b1;frame_abort<=1;reset_core<=1;frame_confirmed<=0;state<=SEEK;
                end else begin frame_confirmed<=1;state<=WAIT_OUTPUT;end
            end
            if(state==WAIT_OUTPUT&&core_idle)state<=SEEK;
        end
    end
    // R16_OBSERVER_LOGIC_BEGIN
    // This block only observes accepted RAW tokens and independently drains
    // metadata. It does not control main FIFO ready/valid or video state.
    always @(posedge pclk or negedge camera_rst_n)begin
        if(!camera_rst_n)begin
            measurement_time_gray_cam_meta<=0;
            measurement_time_gray_cam_sync1<=0;
            measurement_time_gray_cam_sync2<=0;
            measurement_sensor_frame_id<=0;
            measurement_sensor_frame_gray<=0;
            measurement_cam_overflow_count<=0;
            measurement_cam_overflow_gray<=0;
        end else begin
            measurement_time_gray_cam_meta<=measurement_time_gray;
            measurement_time_gray_cam_sync1<=measurement_time_gray_cam_meta;
            measurement_time_gray_cam_sync2<=measurement_time_gray_cam_sync1;
            if(raw_valid&&raw_sof)begin
                measurement_sensor_frame_id<=measurement_sensor_frame_id+32'd1;
                measurement_sensor_frame_gray <=
                    measurement_bin_to_gray32(measurement_sensor_frame_id + 32'd1);
            end
            if(measurement_camera_overflow)begin
                measurement_cam_overflow_count<=measurement_cam_overflow_count+32'd1;
                measurement_cam_overflow_gray <=
                    measurement_bin_to_gray32(measurement_cam_overflow_count + 32'd1);
            end
        end
    end

    always @(posedge clk or negedge core_rst_n)begin
        if(!core_rst_n)begin
            measurement_frames_gray_core_meta<=0;
            measurement_frames_gray_core_sync1<=0;
            measurement_frames_gray_core_sync2<=0;
            measurement_overflow_gray_core_meta<=0;
            measurement_overflow_gray_core_sync1<=0;
            measurement_overflow_gray_core_sync2<=0;
            measurement_missing_sof_count<=0;
            measurement_alignment_invalid<=0;
            measurement_start_event<=0;
            measurement_start_time_valid<=0;
            measurement_start_time<=0;
            measurement_start_sensor_id<=0;
            measurement_end_event<=0;
            measurement_end_time<=0;
            measurement_end_sensor_id<=0;
            measurement_input_frames<=0;
            measurement_errors<=0;
        end else begin
            measurement_frames_gray_core_meta<=measurement_sensor_frame_gray;
            measurement_frames_gray_core_sync1<=measurement_frames_gray_core_meta;
            measurement_frames_gray_core_sync2<=measurement_frames_gray_core_sync1;
            measurement_overflow_gray_core_meta<=measurement_cam_overflow_gray;
            measurement_overflow_gray_core_sync1<=measurement_overflow_gray_core_meta;
            measurement_overflow_gray_core_sync2<=measurement_overflow_gray_core_sync1;
            measurement_input_frames<=measurement_frames_core_binary;
            measurement_errors<=measurement_overflow_core_binary+measurement_missing_sof_count;
            measurement_start_event<=1'b0;
            measurement_start_time_valid<=1'b0;
            measurement_end_event<=1'b0;
            if(measurement_overflow_core_binary!=32'd0)
                measurement_alignment_invalid<=1'b1;
            if(measurement_start_core_fire)begin
                measurement_start_event<=1'b1;
                if(measurement_sof_rd_valid&&!measurement_alignment_bad_now)begin
                    measurement_start_time_valid<=1'b1;
                    measurement_start_sensor_id<=measurement_sof_rd_data[95:64];
                    measurement_start_time<=measurement_sof_rd_data[63:0];
                end else begin
                    measurement_start_time_valid<=1'b0;
                    measurement_start_sensor_id<=0;
                    measurement_start_time<=0;
                    if(!measurement_sof_rd_valid)begin
                        measurement_missing_sof_count<=measurement_missing_sof_count+32'd1;
                        measurement_alignment_invalid<=1'b1;
                    end
                end
            end
            if(measurement_eof_rd_valid)begin
                measurement_end_event<=1'b1;
                if(!measurement_alignment_bad_now)begin
                    measurement_end_sensor_id<=measurement_eof_rd_data[95:64];
                    measurement_end_time<=measurement_eof_rd_data[63:0];
                end else begin
                    measurement_end_sensor_id<=0;
                    measurement_end_time<=0;
                end
            end
        end
    end
    // R16_OBSERVER_LOGIC_END
endmodule
