// Derived, bit-exact arithmetic staging. Original delivery kept separately.
module basic_distortion_coord_pipelined #(
    parameter integer COORD_W = 26,
    parameter integer COORD_F = 10
) (
    input               coord_clk,
    input               coord_rst_n,

    input               coord_req_valid,
    output              coord_req_ready,
    input      [15:0]   coord_req_x,
    input      [15:0]   coord_req_y,
    input      [31:0]   coord_req_frame_id,
    input      [15:0]   coord_req_cfg_version,
    input      [31:0]   coord_req_pixel_tag,

    // Frame-boundary Active Camera Profile. Keep every cfg_active_* input
    // stable until coord_pipe_empty is high and no old result is stalled.
    input      [15:0]   cfg_active_source_width,
    input      [15:0]   cfg_active_source_height,
    input signed [31:0] cfg_active_fx_q16,
    input signed [31:0] cfg_active_fy_q16,
    input signed [31:0] cfg_active_cx_q16,
    input signed [31:0] cfg_active_cy_q16,
    input signed [31:0] cfg_active_inv_fx_q24,
    input signed [31:0] cfg_active_inv_fy_q24,
    input signed [31:0] cfg_active_k1_q24,
    input signed [31:0] cfg_active_k2_q24,
    input signed [31:0] cfg_active_p1_q24,
    input signed [31:0] cfg_active_p2_q24,

    output              coord_res_valid,
    input               coord_res_ready,
    output reg signed [COORD_W-1:0] src_x_fixed,
    output reg signed [COORD_W-1:0] src_y_fixed,
    output reg        [1:0]  border,
    output reg       [31:0]  coord_res_frame_id,
    output reg       [15:0]  coord_res_cfg_version,
    output reg       [31:0]  coord_res_pixel_tag,
    output              coord_pipe_empty
);

    localparam STAGES=22, DATA_W=896;
    localparam COORD_SHIFT=16-COORD_F;
    localparam [63:0] COORD_ROUND_BIAS=64'd1 << ((COORD_F==16)?0:(COORD_SHIFT-1));
    localparam signed [63:0] COORD_MAX_64=(64'sd1<<<(COORD_W-1))-64'sd1;
    localparam signed [63:0] COORD_MIN_64=-(64'sd1<<<(COORD_W-1));
    localparam X=0, Y=32, X2=64, Y2=96, XY=128, R2=160, R4=192, K1R2=224, K2R4=256, GAIN=288, RX=320, RY=352, P1XY=384, P2XY=416, TX=448, TY=480, XD=512, YD=544, SQX=576, SQY=608, P0=640,P1=704,P2=768,P3=832;

    reg [STAGES-1:0] valid_pipe;
    wire [STAGES-1:0] stage_ready;
    reg [DATA_W-1:0] data_pipe[0:STAGES-2];
    reg [15:0] width_pipe[0:STAGES-2],height_pipe[0:STAGES-2],cfg_pipe[0:STAGES-2];
    reg [31:0] frame_pipe[0:STAGES-2],tag_pipe[0:STAGES-2];
    reg [1:0] border_stage20;
    function signed [31:0] v32;
        input [DATA_W-1:0] data;
        input integer offset;
        begin v32=data[offset+:32];end
    endfunction
    function [63:0] mul32_unsigned;
        input [31:0] a,b;
        begin mul32_unsigned=a*b;end
    endfunction
    function signed [63:0] signed_product;
        input [63:0] unsigned_product;
        input signed [31:0] a,b;
        reg [31:0] corrected_high;
        begin
            // Modulo-2^64 sign correction changes only the high 32 bits.
            corrected_high=unsigned_product[63:32]
                           -(a[31]?$unsigned(b):32'd0)
                           -(b[31]?$unsigned(a):32'd0);
            signed_product=$signed({corrected_high,unsigned_product[31:0]});
        end
    endfunction
    function signed [63:0] mul32;
        input signed [31:0] a,b;
        begin mul32=signed_product(mul32_unsigned(a,b),a,b);end
    endfunction
    // Truncate floor first; increment towards +infinity when needed. Negative
    // exact halves stay at floor, matching the delivered ties-away convention.
    function signed [31:0] round_product;
        input signed [63:0] value;
        input integer shift;
        reg signed [31:0] floor_value;
        reg increment;
        begin
            floor_value=value>>>shift;
            increment=value[shift-1]&&(!value[63]||((value&((64'd1<<(shift-1))-1))!=0));
            round_product=floor_value+{31'd0,increment};
        end
    endfunction
    function signed [31:0] round_signed_product;
        input [63:0] unsigned_product;
        input signed [31:0] a,b;
        input integer shift;
        begin round_signed_product=round_product(signed_product(unsigned_product,a,b),shift);end
    endfunction
    function signed [63:0] q16_to_coord;
        input signed [31:0] value_q16;
        reg signed [63:0] value_ext;
        reg [63:0] magnitude;
        reg [63:0] rounded;
        begin
            value_ext = {{32{value_q16[31]}}, value_q16};
            if (COORD_F == 16) begin
                q16_to_coord = value_ext;
            end else if (value_ext[63]) begin
                magnitude = (~value_ext) + 64'd1;
                rounded = (magnitude + COORD_ROUND_BIAS) >> COORD_SHIFT;
                q16_to_coord = -$signed(rounded);
            end else begin
                magnitude = value_ext;
                rounded = (magnitude + COORD_ROUND_BIAS) >> COORD_SHIFT;
                q16_to_coord = $signed(rounded);
            end
        end
    endfunction
    genvar n;
    assign stage_ready[STAGES-1]=!valid_pipe[STAGES-1]||coord_res_ready;
    generate for(n=0;n<STAGES-1;n=n+1)begin:g_ready
        assign stage_ready[n]=!valid_pipe[n]||stage_ready[n+1];
    end endgenerate
    assign coord_req_ready=stage_ready[0];
    assign coord_res_valid=valid_pipe[STAGES-1];
    assign coord_pipe_empty=!(|valid_pipe);
    wire signed [31:0] qx19=v32(data_pipe[19],SQX),qy19=v32(data_pipe[19],SQY);
    wire signed [31:0] max_x19=($signed({16'd0,width_pipe[19]})-32'sd1)<<<16;
    wire signed [31:0] max_y19=($signed({16'd0,height_pipe[19]})-32'sd1)<<<16;
    wire signed [63:0] wide_x19=q16_to_coord(qx19),wide_y19=q16_to_coord(qy19);
    wire signed [63:0] wide_x20=data_pipe[20][P0+:64],wide_y20=data_pipe[20][P1+:64];
    integer i;
    // Datapath has no asynchronous reset: validity guards unknown data and
    // prevents frame-state transitions from driving DSP reset/recovery paths.
    always @(posedge coord_clk)begin
        if(stage_ready[0]&&coord_req_valid)begin
            data_pipe[0]<=0;
            data_pipe[0][X+:32]<=($signed({16'd0,coord_req_x})<<<16)-cfg_active_cx_q16;
            data_pipe[0][Y+:32]<=($signed({16'd0,coord_req_y})<<<16)-cfg_active_cy_q16;
            width_pipe[0]<=cfg_active_source_width;height_pipe[0]<=cfg_active_source_height;
            frame_pipe[0]<=coord_req_frame_id;cfg_pipe[0]<=coord_req_cfg_version;tag_pipe[0]<=coord_req_pixel_tag;
        end
        for(i=1;i<STAGES-1;i=i+1)begin
            if(stage_ready[i]&&valid_pipe[i-1])begin
                data_pipe[i]<=data_pipe[i-1];
                width_pipe[i]<=width_pipe[i-1];height_pipe[i]<=height_pipe[i-1];
                frame_pipe[i]<=frame_pipe[i-1];cfg_pipe[i]<=cfg_pipe[i-1];tag_pipe[i]<=tag_pipe[i-1];
                case(i)
                    1: begin
                        // Register the raw unsigned DSP result first. Sign
                        // correction and rounding run in the existing next stage.
                        data_pipe[i][P0+:64] <= mul32_unsigned(v32(data_pipe[i-1],X),cfg_active_inv_fx_q24);
                        data_pipe[i][P1+:64] <= mul32_unsigned(v32(data_pipe[i-1],Y),cfg_active_inv_fy_q24);
                    end
                    2: begin
                        data_pipe[i][X+:32] <= round_signed_product(data_pipe[i-1][P0+:64],v32(data_pipe[i-1],X),cfg_active_inv_fx_q24,24);
                        data_pipe[i][Y+:32] <= round_signed_product(data_pipe[i-1][P1+:64],v32(data_pipe[i-1],Y),cfg_active_inv_fy_q24,24);
                    end
                    3: begin
                        data_pipe[i][P0+:64] <= mul32_unsigned(v32(data_pipe[i-1],X),v32(data_pipe[i-1],X));
                        data_pipe[i][P1+:64] <= mul32_unsigned(v32(data_pipe[i-1],Y),v32(data_pipe[i-1],Y));
                        data_pipe[i][P2+:64] <= mul32_unsigned(v32(data_pipe[i-1],X),v32(data_pipe[i-1],Y));
                    end
                    4: begin
                        data_pipe[i][X2+:32] <= round_signed_product(data_pipe[i-1][P0+:64],v32(data_pipe[i-1],X),v32(data_pipe[i-1],X),16);
                        data_pipe[i][Y2+:32] <= round_signed_product(data_pipe[i-1][P1+:64],v32(data_pipe[i-1],Y),v32(data_pipe[i-1],Y),16);
                        data_pipe[i][XY+:32] <= round_signed_product(data_pipe[i-1][P2+:64],v32(data_pipe[i-1],X),v32(data_pipe[i-1],Y),16);
                    end
                    5: begin
                        data_pipe[i][R2+:32] <= v32(data_pipe[i-1],X2) + v32(data_pipe[i-1],Y2);
                    end
                    6: begin
                        data_pipe[i][P0+:64] <= mul32_unsigned(v32(data_pipe[i-1],R2),v32(data_pipe[i-1],R2));
                        data_pipe[i][P1+:64] <= mul32_unsigned(cfg_active_k1_q24,v32(data_pipe[i-1],R2));
                    end
                    7: begin
                        data_pipe[i][R4+:32] <= round_signed_product(data_pipe[i-1][P0+:64],v32(data_pipe[i-1],R2),v32(data_pipe[i-1],R2),16);
                        data_pipe[i][K1R2+:32] <= round_signed_product(data_pipe[i-1][P1+:64],cfg_active_k1_q24,v32(data_pipe[i-1],R2),24);
                    end
                    8: begin
                        data_pipe[i][P0+:64] <= mul32_unsigned(cfg_active_k2_q24,v32(data_pipe[i-1],R4));
                    end
                    9: begin
                        data_pipe[i][K2R4+:32] <= round_signed_product(data_pipe[i-1][P0+:64],cfg_active_k2_q24,v32(data_pipe[i-1],R4),24);
                    end
                    10: begin
                        data_pipe[i][GAIN+:32] <= 32'sd65536 + v32(data_pipe[i-1],K1R2) + v32(data_pipe[i-1],K2R4);
                    end
                    11: begin
                        data_pipe[i][P0+:64] <= mul32_unsigned(v32(data_pipe[i-1],X),v32(data_pipe[i-1],GAIN));
                        data_pipe[i][P1+:64] <= mul32_unsigned(v32(data_pipe[i-1],Y),v32(data_pipe[i-1],GAIN));
                        data_pipe[i][P2+:64] <= mul32_unsigned(cfg_active_p1_q24,v32(data_pipe[i-1],XY));
                        data_pipe[i][P3+:64] <= mul32_unsigned(cfg_active_p2_q24,v32(data_pipe[i-1],XY));
                    end
                    12: begin
                        data_pipe[i][RX+:32] <= round_signed_product(data_pipe[i-1][P0+:64],v32(data_pipe[i-1],X),v32(data_pipe[i-1],GAIN),16);
                        data_pipe[i][RY+:32] <= round_signed_product(data_pipe[i-1][P1+:64],v32(data_pipe[i-1],Y),v32(data_pipe[i-1],GAIN),16);
                        data_pipe[i][P1XY+:32] <= round_signed_product(data_pipe[i-1][P2+:64],cfg_active_p1_q24,v32(data_pipe[i-1],XY),24);
                        data_pipe[i][P2XY+:32] <= round_signed_product(data_pipe[i-1][P3+:64],cfg_active_p2_q24,v32(data_pipe[i-1],XY),24);
                    end
                    13: begin
                        data_pipe[i][TX+:32] <= v32(data_pipe[i-1],R2) + (v32(data_pipe[i-1],X2) <<< 1);
                        data_pipe[i][TY+:32] <= v32(data_pipe[i-1],R2) + (v32(data_pipe[i-1],Y2) <<< 1);
                        data_pipe[i][RX+:32] <= v32(data_pipe[i-1],RX) + (v32(data_pipe[i-1],P1XY) <<< 1);
                        data_pipe[i][RY+:32] <= v32(data_pipe[i-1],RY) + (v32(data_pipe[i-1],P2XY) <<< 1);
                    end
                    14: begin
                        data_pipe[i][P0+:64] <= mul32_unsigned(cfg_active_p2_q24,v32(data_pipe[i-1],TX));
                        data_pipe[i][P1+:64] <= mul32_unsigned(cfg_active_p1_q24,v32(data_pipe[i-1],TY));
                    end
                    15: begin
                        data_pipe[i][TX+:32] <= round_signed_product(data_pipe[i-1][P0+:64],cfg_active_p2_q24,v32(data_pipe[i-1],TX),24);
                        data_pipe[i][TY+:32] <= round_signed_product(data_pipe[i-1][P1+:64],cfg_active_p1_q24,v32(data_pipe[i-1],TY),24);
                    end
                    16: begin
                        data_pipe[i][XD+:32] <= v32(data_pipe[i-1],RX) + v32(data_pipe[i-1],TX);
                        data_pipe[i][YD+:32] <= v32(data_pipe[i-1],RY) + v32(data_pipe[i-1],TY);
                    end
                    17: begin
                        data_pipe[i][P0+:64] <= mul32_unsigned(cfg_active_fx_q16,v32(data_pipe[i-1],XD));
                        data_pipe[i][P1+:64] <= mul32_unsigned(cfg_active_fy_q16,v32(data_pipe[i-1],YD));
                    end
                    18: begin
                        data_pipe[i][SQX+:32] <= round_signed_product(data_pipe[i-1][P0+:64],cfg_active_fx_q16,v32(data_pipe[i-1],XD),16);
                        data_pipe[i][SQY+:32] <= round_signed_product(data_pipe[i-1][P1+:64],cfg_active_fy_q16,v32(data_pipe[i-1],YD),16);
                    end
                    19: begin
                        data_pipe[i][SQX+:32] <= v32(data_pipe[i-1],SQX) + cfg_active_cx_q16;
                        data_pipe[i][SQY+:32] <= v32(data_pipe[i-1],SQY) + cfg_active_cy_q16;
                    end
                    20: begin
                        data_pipe[i][P0+:64] <= q16_to_coord(v32(data_pipe[i-1],SQX));
                        data_pipe[i][P1+:64] <= q16_to_coord(v32(data_pipe[i-1],SQY));
                    end
                endcase
            end
        end
        if(stage_ready[20]&&valid_pipe[19])begin
            if(width_pipe[19]==0||height_pipe[19]==0||qx19<0||qy19<0||qx19>max_x19||qy19>max_y19||
               wide_x19>COORD_MAX_64||wide_x19<COORD_MIN_64||wide_y19>COORD_MAX_64||wide_y19<COORD_MIN_64)
                border_stage20<=2'b10;
            else if(width_pipe[19]<2||height_pipe[19]<2||qx19>=max_x19||qy19>=max_y19)border_stage20<=2'b01;
            else border_stage20<=2'b00;
        end
    end
    integer j;
    always @(posedge coord_clk or negedge coord_rst_n)begin
        if(!coord_rst_n)begin
            valid_pipe<=0;src_x_fixed<=0;src_y_fixed<=0;border<=2'b10;
            coord_res_frame_id<=0;coord_res_cfg_version<=0;coord_res_pixel_tag<=0;
        end else begin
            if(stage_ready[0])valid_pipe[0]<=coord_req_valid;
            for(j=1;j<STAGES;j=j+1)if(stage_ready[j])valid_pipe[j]<=valid_pipe[j-1];
            if(stage_ready[21]&&valid_pipe[20])begin
                if(wide_x20>COORD_MAX_64)src_x_fixed<={1'b0,{(COORD_W-1){1'b1}}};
                else if(wide_x20<COORD_MIN_64)src_x_fixed<={1'b1,{(COORD_W-1){1'b0}}};
                else src_x_fixed<=wide_x20[COORD_W-1:0];
                if(wide_y20>COORD_MAX_64)src_y_fixed<={1'b0,{(COORD_W-1){1'b1}}};
                else if(wide_y20<COORD_MIN_64)src_y_fixed<={1'b1,{(COORD_W-1){1'b0}}};
                else src_y_fixed<=wide_y20[COORD_W-1:0];
                border<=border_stage20;coord_res_frame_id<=frame_pipe[20];
                coord_res_cfg_version<=cfg_pipe[20];coord_res_pixel_tag<=tag_pipe[20];
            end
        end
    end
endmodule
