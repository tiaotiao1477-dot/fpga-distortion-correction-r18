`timescale 1ns/1ps
// R8 side branch: never provides ready/backpressure to the camera stream.
// Initial integration exposes predictions; it cannot silently alter geometry.
module nn_video_sidecar #(
    parameter [31:0] EXPECTED_CRC32=32'h00000000
)(
    input clk,input rst_n,input frame_active,input pixel_valid,input pixel_abort,
    input [23:0] rgb,input [31:0] source_frame_id,
    input cmd_valid,output cmd_ready,input [7:0] cmd_op,
    input [31:0] cmd_seq,input [31:0] cmd_addr,input [255:0] cmd_data,
    output [1023:0] telemetry,
    output rd_req_valid,input rd_req_ready,output [63:0] rd_req_addr,
    output [31:0] rd_req_len,output [7:0] rd_req_id,
    input rd_rsp_valid,output rd_rsp_ready,input [255:0] rd_rsp_data,
    input [7:0] rd_rsp_id,input rd_rsp_last,input rd_rsp_error,
    output wr_req_valid,input wr_req_ready,output [63:0] wr_req_addr,
    output [31:0] wr_req_len,output [7:0] wr_req_id,
    output wr_data_valid,input wr_data_ready,output [255:0] wr_data,
    output wr_data_last,input wr_done_valid,input [7:0] wr_done_id,input wr_done_error
);
    wire loaded,enabled,k_valid,cache_idle,image_valid,image_ready,capture_busy;
    wire [63:0] k_packed;
    wire invalidate_valid,invalidate_ready;
    wire [31:0] last_seq,loaded_bytes,model_crc;
    wire [7:0] loader_error;
    wire loader_wr_req_valid,loader_wr_req_ready,loader_wr_data_valid,loader_wr_data_ready,loader_wr_last;
    wire [63:0] loader_wr_addr;
    wire [31:0] loader_wr_len;
    wire [7:0] loader_wr_id;
    wire [255:0] loader_wr_data;
    wire cache_wr_req_valid,cache_wr_req_ready,cache_wr_data_valid,cache_wr_data_ready,cache_wr_last;
    wire [63:0] cache_wr_addr;
    wire [31:0] cache_wr_len;
    wire [7:0] cache_wr_id;
    wire [255:0] cache_wr_data;
    wire nn_busy,nn_done,nn_saturated,nn_protocol_error;
    wire [1:0] nn_error;
    wire nn_start_ready;
    reg fault_latched; // R16 declaration order only
    wire nn_start_valid=image_valid&&loaded&&enabled&&k_valid&&!fault_latched;
    wire nn_start_fire=nn_start_valid&&nn_start_ready;
    wire [31:0] image_frame_id,nn_cycles,pred_frame;
    wire [31:0] capture_errors,skipped_frames;
    wire pred_valid;
    wire [447:0] pred_data;
    reg [447:0] last_prediction;
    reg [31:0] prediction_count,last_prediction_frame,last_prediction_cycles;
    // Disabled pending thumbnails are discarded, allowing a quiescent reload.
    assign image_ready=nn_start_fire||!enabled;
    wire capture_enable=loaded&&enabled&&k_valid&&!nn_busy&&!fault_latched;
    wire inference_idle=!nn_busy&&!capture_busy&&!image_valid;
    nn_model_loader #(.EXPECTED_CRC32(EXPECTED_CRC32)) loader(
        .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .cmd_op(cmd_op),.cmd_seq(cmd_seq),.cmd_addr(cmd_addr),.cmd_data(cmd_data),
        .inference_idle(inference_idle),.cache_idle(cache_idle),
        .model_loaded(loaded),.nn_enable(enabled),.k_packed(k_packed),.k_valid(k_valid),
        .cache_invalidate_valid(invalidate_valid),.cache_invalidate_ready(invalidate_ready),
        .wr_req_valid(loader_wr_req_valid),.wr_req_ready(loader_wr_req_ready),
        .wr_req_addr(loader_wr_addr),.wr_req_len(loader_wr_len),.wr_req_id(loader_wr_id),
        .wr_data_valid(loader_wr_data_valid),.wr_data_ready(loader_wr_data_ready),
        .wr_data(loader_wr_data),.wr_last(loader_wr_last),
        .wr_done_valid(wr_done_valid&&wr_done_id==8'he2),.wr_done_id(wr_done_id),.wr_done_error(wr_done_error),
        .last_completed_seq(last_seq),.last_error(loader_error),.loaded_bytes(loaded_bytes),.computed_crc32(model_crc));

    reg [13:0] thumb_addr;
    reg [1:0] input_state;
    reg [7:0] held_gray;
    wire thumb_read_en=input_state==1;
    wire thumb_read_valid;
    wire [7:0] thumb_read_data;
    wire nn_pixel_ready;
    nn_thumbnail_capture thumbnail(
        .clk(clk),.rst_n(rst_n),.capture_enable(capture_enable),
        .frame_active(frame_active),.pixel_valid(pixel_valid),.pixel_abort(pixel_abort),
        .rgb(rgb),.source_frame_id(source_frame_id),.image_valid(image_valid),
        .image_ready(image_ready),.image_frame_id(image_frame_id),.capture_busy(capture_busy),
        .read_en(thumb_read_en),.read_address(thumb_addr),
        .read_valid(thumb_read_valid),.read_data(thumb_read_data),
        .capture_error_count(capture_errors),.skipped_frame_count(skipped_frames));
    always @(posedge clk)begin
        if(!rst_n)begin input_state<=0;thumb_addr<=0;held_gray<=0;end
        else if(nn_done)input_state<=0;
        else case(input_state)
            0:if(nn_start_fire)begin thumb_addr<=0;input_state<=1;end
            1:input_state<=2;
            2:if(thumb_read_valid)begin held_gray<=thumb_read_data;input_state<=3;end
            3:if(nn_pixel_ready)begin
                if(thumb_addr==16383)input_state<=0;
                else begin thumb_addr<=thumb_addr+1'b1;input_state<=1;end
            end
        endcase
    end
    wire word_req_valid,word_req_ready,word_req_write,word_rsp_valid,word_rsp_ready,word_rsp_error;
    wire [63:0] word_req_addr;
    wire [15:0] word_req_wdata,word_rsp_data;
    nn_geometry_predictor_ddr predictor(
        .clk(clk),.rst_n(rst_n),.start_valid(nn_start_valid),.start_ready(nn_start_ready),
        .frame_id(image_frame_id),.k_fx(k_packed[15:0]),.k_fy(k_packed[31:16]),
        .k_cx(k_packed[47:32]),.k_cy(k_packed[63:48]),
        .pixel_valid(input_state==3),.pixel_ready(nn_pixel_ready),.pixel_gray(held_gray),
        .prediction_valid(pred_valid),.prediction_ready(1'b1),.prediction_data(pred_data),
        .prediction_frame_id(pred_frame),.busy(nn_busy),.compute_done(nn_done),
        .saturated(nn_saturated),.error_code(nn_error),.protocol_error(nn_protocol_error),.cycle_count(nn_cycles),
        .mem_req_valid(word_req_valid),.mem_req_ready(word_req_ready),.mem_req_write(word_req_write),
        .mem_req_addr(word_req_addr),.mem_req_wdata(word_req_wdata),
        .mem_rsp_valid(word_rsp_valid),.mem_rsp_ready(word_rsp_ready),
        .mem_rsp_rdata(word_rsp_data),.mem_rsp_error(word_rsp_error));
    nn_word_cache cache(
        .clk(clk),.rst_n(rst_n),.invalidate_valid(invalidate_valid),.invalidate_ready(invalidate_ready),
        .req_valid(word_req_valid),.req_ready(word_req_ready),.req_write(word_req_write),
        .req_addr(word_req_addr),.req_wdata(word_req_wdata),
        .rsp_valid(word_rsp_valid),.rsp_ready(word_rsp_ready),.rsp_rdata(word_rsp_data),.rsp_error(word_rsp_error),
        .idle(cache_idle),.rd_req_valid(rd_req_valid),.rd_req_ready(rd_req_ready),
        .rd_req_addr(rd_req_addr),.rd_req_len(rd_req_len),.rd_req_id(rd_req_id),
        .rd_rsp_valid(rd_rsp_valid),.rd_rsp_ready(rd_rsp_ready),.rd_rsp_data(rd_rsp_data),
        .rd_rsp_id(rd_rsp_id),.rd_rsp_last(rd_rsp_last),.rd_rsp_error(rd_rsp_error),
        .wr_req_valid(cache_wr_req_valid),.wr_req_ready(cache_wr_req_ready),
        .wr_req_addr(cache_wr_addr),.wr_req_len(cache_wr_len),.wr_req_id(cache_wr_id),
        .wr_data_valid(cache_wr_data_valid),.wr_data_ready(cache_wr_data_ready),
        .wr_data(cache_wr_data),.wr_data_last(cache_wr_last),
        .wr_done_valid(wr_done_valid&&wr_done_id==8'he1),.wr_done_id(wr_done_id),.wr_done_error(wr_done_error));
    // A BEGIN can be accepted only with both inference and cache idle. Loader
    // traffic therefore cannot overlap a cache transaction; select by request
    // phase and keep the write owner through done as an additional safeguard.
    reg write_active,write_owner_loader,write_held,held_owner_loader;
    wire select_loader=write_held?held_owner_loader:loader_wr_req_valid;
    assign wr_req_valid=write_active?1'b0:(select_loader?loader_wr_req_valid:cache_wr_req_valid);
    assign wr_req_addr=select_loader?loader_wr_addr:cache_wr_addr;
    assign wr_req_len=select_loader?loader_wr_len:cache_wr_len;
    assign wr_req_id=select_loader?loader_wr_id:cache_wr_id;
    assign loader_wr_req_ready=!write_active&&select_loader&&wr_req_ready;
    assign cache_wr_req_ready=!write_active&&!select_loader&&wr_req_ready;
    assign wr_data_valid=write_active&&(write_owner_loader?loader_wr_data_valid:cache_wr_data_valid);
    assign wr_data=write_owner_loader?loader_wr_data:cache_wr_data;
    assign wr_data_last=write_owner_loader?loader_wr_last:cache_wr_last;
    assign loader_wr_data_ready=write_active&&write_owner_loader&&wr_data_ready;
    assign cache_wr_data_ready=write_active&&!write_owner_loader&&wr_data_ready;
    always @(posedge clk)begin
        if(!rst_n)begin write_active<=0;write_owner_loader<=0;write_held<=0;held_owner_loader<=0;end
        else if(!write_active)begin
            if(wr_req_valid&&wr_req_ready)begin
                write_active<=1;write_owner_loader<=select_loader;write_held<=0;
            end else if(wr_req_valid&&!wr_req_ready)begin write_held<=1;held_owner_loader<=select_loader;end
        end else if(wr_done_valid)write_active<=0;
    end
    always @(posedge clk)begin
        if(!rst_n)begin
            last_prediction<=0;prediction_count<=0;last_prediction_frame<=0;
            last_prediction_cycles<=0;fault_latched<=0;
        end else begin
            if(invalidate_valid&&invalidate_ready)begin
                fault_latched<=0;last_prediction<=0;last_prediction_frame<=0;
                last_prediction_cycles<=0;prediction_count<=0;
            end else begin
                if(nn_done&&(nn_error!=0||nn_protocol_error))fault_latched<=1;
                if(pred_valid)begin
                    last_prediction<=pred_data;last_prediction_frame<=pred_frame;
                    last_prediction_cycles<=nn_cycles;prediction_count<=prediction_count+1'b1;
                end
            end
        end
    end
    assign telemetry[0*32+:32]=32'h4e4e0001;
    assign telemetry[1*32+:32]={20'd0,fault_latched,nn_protocol_error,nn_error,
        nn_saturated,capture_busy,cache_idle,image_valid,nn_busy,k_valid,enabled,loaded};
    assign telemetry[2*32+:32]=last_seq;
    assign telemetry[3*32+:32]={24'd0,loader_error};
    assign telemetry[4*32+:32]=loaded_bytes;
    assign telemetry[5*32+:32]=model_crc;
    assign telemetry[6*32+:32]=last_prediction_frame;
    assign telemetry[7*32+:32]=last_prediction_cycles;
    assign telemetry[8*32+:32]=prediction_count;
    assign telemetry[9*32+:32]=capture_errors;
    assign telemetry[10*32+:32]=skipped_frames;
    assign telemetry[11*32+:32]=image_frame_id;
    assign telemetry[12*32+:32]=nn_cycles;
    assign telemetry[13*32+:32]=k_packed[31:0];
    assign telemetry[14*32+:32]=k_packed[63:32];
    assign telemetry[15*32+:32]=32'd0; // geometry auto-apply deliberately disabled
    assign telemetry[16*32+:448]=last_prediction;
    assign telemetry[30*32+:32]=EXPECTED_CRC32;
    assign telemetry[31*32+:32]=32'h000f0003; // Bit0=length units fixed; bit1=display Gamma available.
endmodule
