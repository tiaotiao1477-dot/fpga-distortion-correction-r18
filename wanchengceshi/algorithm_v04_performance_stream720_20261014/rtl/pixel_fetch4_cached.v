`timescale 1ns/1ps
// Four ways per hashed set with packed synchronous metadata RAM.
// A 128-byte block fills at most four DDR beats,
// clipped to the claimed source frame. Tags/masks invalidate on frame changes.
// All four required blocks are pinned during replacement, avoiding row-stride
// conflicts. One transaction outstanding; every malformed response is drained.
module pixel_fetch4_cached #(
    parameter integer COORD_W=26, COORD_F=10,
    parameter [7:0] REQ_ID=8'hA6,
    parameter integer CACHE_SET_BITS=9, CACHE_BLOCK_BITS=7,
    parameter integer DEFER_ZERO_MASK=0
)(
    input wire clk,rst_n,in_valid,
    output wire in_ready,
    input wire signed [COORD_W-1:0] src_x_fixed,src_y_fixed,
    input wire [15:0] source_width,source_height,
    input wire [63:0] source_base,
    input wire [31:0] source_stride,frame_id,pixel_tag,
    input wire [15:0] cfg_version,
    input wire border_clamp,
    output wire fetch_rd_req_valid,
    input wire fetch_rd_req_ready,
    output wire [63:0] fetch_rd_req_addr,
    output wire [31:0] fetch_rd_req_len,
    output wire [7:0] fetch_rd_req_id,
    input wire fetch_rd_rsp_valid,
    output wire fetch_rd_rsp_ready,
    input wire [255:0] fetch_rd_rsp_data,
    input wire [7:0] fetch_rd_rsp_id,
    input wire fetch_rd_rsp_last,fetch_rd_rsp_error,
    output wire out_valid,
    input wire out_ready,
    output wire [23:0] P00_rgb,P10_rgb,P01_rgb,P11_rgb,
    output reg [COORD_F-1:0] dx,dy,
    output reg [31:0] out_frame_id,out_pixel_tag,
    output reg [15:0] out_cfg_version,
    output reg out_error,out_border,
    output wire out_pixels_zero
);
    localparam IDLE=0,PREPARE=1,LOOKUP=2,REQUEST=3,RESPONSE=4,OUTPUT=5,DRAIN=6,MISS_SELECT=7,READ_TAG=8,SWEEP=9;
    localparam integer BEAT_BITS=CACHE_BLOCK_BITS-5, BLOCK_BEATS=1<<BEAT_BITS;
    localparam integer TAG_BITS=64-CACHE_BLOCK_BITS, WAY_BITS=TAG_BITS+BLOCK_BEATS;
    localparam integer META_W=4*WAY_BITS+2;
    localparam integer CACHE_SETS=1<<CACHE_SET_BITS;
    localparam integer CACHE_BANKS=4*CACHE_SETS;
    localparam integer BANK_W=CACHE_SET_BITS+2;
    localparam integer SET_W=(CACHE_SET_BITS>0)?CACHE_SET_BITS:1;
    reg [3:0] state;
    reg [3:0] sweep_resume;
    reg [SET_W-1:0] sweep_index;
    reg [15:0] x0,x1,y0,y1;
    reg [63:0] active_base,active_end,request_addr;
    reg [31:0] active_stride,request_len;
    reg [63:0] tap_addr[0:3];
    wire [23:0] sample_rgb[0:3];
    reg [255:0] sample_way0[0:3] /* synthesis syn_keep = 1 */;
    reg [255:0] sample_way1[0:3] /* synthesis syn_keep = 1 */;
    reg [255:0] sample_way2[0:3] /* synthesis syn_keep = 1 */;
    reg [255:0] sample_way3[0:3] /* synthesis syn_keep = 1 */;
    reg [1:0] sample_bank[0:3] /* synthesis syn_preserve = 1 */;
    reg [3:0] payload_read_enable /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg pixels_zero;
    reg [255:0] data_way0[0:CACHE_SETS*BLOCK_BEATS-1];
    reg [255:0] data_way1[0:CACHE_SETS*BLOCK_BEATS-1];
    reg [255:0] data_way2[0:CACHE_SETS*BLOCK_BEATS-1];
    reg [255:0] data_way3[0:CACHE_SETS*BLOCK_BEATS-1];
    reg [META_W-1:0] metadata_ram[0:CACHE_SETS-1];
    reg [META_W-1:0] metadata_words[0:3];
    reg metadata_snapshot_valid;
    reg [SET_W-1:0] metadata_sets[0:3];
    reg [META_W-1:0] fill_metadata, write_metadata;
    reg [BLOCK_BEATS-1:0] fill_mask;
    function [SET_W-1:0] cache_set;
        input [63:0] address;
        begin cache_set=(address>>CACHE_BLOCK_BITS)^(address>>(CACHE_BLOCK_BITS+CACHE_SET_BITS));end
    endfunction
    reg [BANK_W-1:0] fill_bank;
    reg [BEAT_BITS-1:0] fill_start;
    reg [4:0] fill_count;
    reg key_valid;
    reg [63:0] key_base;
    reg [31:0] key_stride,key_frame;
    reg [15:0] key_width,key_height,key_cfg;
    integer i,j;
    wire signed [63:0] input_x={{(64-COORD_W){src_x_fixed[COORD_W-1]}},src_x_fixed};
    wire signed [63:0] input_y={{(64-COORD_W){src_y_fixed[COORD_W-1]}},src_y_fixed};
    wire signed [63:0] max_x=($signed({48'd0,source_width})-64'sd1)<<<COORD_F;
    wire signed [63:0] max_y=($signed({48'd0,source_height})-64'sd1)<<<COORD_F;
    wire outside=input_x<0||input_y<0||input_x>max_x||input_y>max_y;
    wire signed [63:0] clipped_x=input_x<0?64'sd0:(input_x>max_x?max_x:input_x);
    wire signed [63:0] clipped_y=input_y<0?64'sd0:(input_y>max_y?max_y:input_y);
    wire [63:0] floor_x=clipped_x>>>COORD_F, floor_y=clipped_y>>>COORD_F;
    wire [64:0] input_end={1'b0,source_base}+({49'd0,source_height}*{33'd0,source_stride});
    wire descriptor_ok=source_width!=0&&source_height!=0&&source_base[4:0]==0&&
        source_stride[4:0]==0&&source_stride>={14'd0,source_width,2'b00}&&!input_end[64];
    wire same_key=key_valid&&key_base==source_base&&key_stride==source_stride&&
        key_frame==frame_id&&key_cfg==cfg_version&&key_width==source_width&&key_height==source_height;
    reg [3:0] hits;
    reg [3:0] way_hits[0:3];
    reg [BANK_W-1:0] hit_bank[0:3];
    reg [3:0] pinned;
    reg [1:0] missing_tap,candidate_way,selected_missing_tap;
    reg [BANK_W-1:0] replacement,lookup_bank;
    reg [SET_W-1:0] missing_set;
    reg found_missing,found_bank;
    reg [63:0] miss_block,start_addr,end_addr;
    reg [64:0] block_end;
    always @* begin
        hits=0;pinned=0;missing_tap=0;found_missing=0;replacement=0;found_bank=0;
        lookup_bank=0;candidate_way=0;missing_set=0;
        for(i=0;i<4;i=i+1)begin
            hit_bank[i]=0;
            way_hits[i]=0;
            for(j=0;j<4;j=j+1)begin
                if(metadata_words[i][j*WAY_BITS+:BLOCK_BEATS]!=0&&
                   metadata_words[i][j*WAY_BITS+BLOCK_BEATS+:TAG_BITS]==tap_addr[i][63:CACHE_BLOCK_BITS])begin
                    if(metadata_words[i][j*WAY_BITS+tap_addr[i][CACHE_BLOCK_BITS-1:5]])begin
                        hits[i]=1;hit_bank[i]=cache_set(tap_addr[i])*4+j;
                        way_hits[i][j]=1;
                    end
                end
            end
            if(!hits[i]&&!found_missing)begin missing_tap=i;found_missing=1;end
        end
        missing_set=cache_set(tap_addr[selected_missing_tap]);
        for(j=0;j<4;j=j+1)begin
            for(i=0;i<4;i=i+1)
                if(metadata_words[selected_missing_tap][j*WAY_BITS+:BLOCK_BEATS]!=0&&
                   metadata_words[selected_missing_tap][j*WAY_BITS+BLOCK_BEATS+:TAG_BITS]==tap_addr[i][63:CACHE_BLOCK_BITS])pinned[j]=1;
        end
        for(j=0;j<4;j=j+1)begin
            candidate_way=metadata_words[selected_missing_tap][META_W-1-:2]+j;
            if(!pinned[candidate_way]&&!found_bank)begin
                replacement=missing_set*4+candidate_way;found_bank=1;
            end
        end
        for(j=0;j<4;j=j+1)begin
            if(metadata_words[selected_missing_tap][j*WAY_BITS+:BLOCK_BEATS]!=0&&
               metadata_words[selected_missing_tap][j*WAY_BITS+BLOCK_BEATS+:TAG_BITS]==tap_addr[selected_missing_tap][63:CACHE_BLOCK_BITS])replacement=missing_set*4+j;
        end
        miss_block=(tap_addr[selected_missing_tap]>>CACHE_BLOCK_BITS)<<CACHE_BLOCK_BITS;
        start_addr=miss_block<active_base?active_base:miss_block;
        block_end={1'b0,miss_block}+(65'd1<<CACHE_BLOCK_BITS);
        end_addr=block_end>{1'b0,active_end}?active_end:block_end[63:0];
        write_metadata=fill_metadata;
        write_metadata[fill_bank[1:0]*WAY_BITS+:WAY_BITS]={tap_addr[selected_missing_tap][63:CACHE_BLOCK_BITS],
            fill_mask | ({{(BLOCK_BEATS-1){1'b0}},1'b1} << (fill_start+fill_count))};
        write_metadata[META_W-1-:2]=fill_bank[1:0]+2'd1;
    end
    // Metadata is unchanged between transactions. Reuse it only when all
    // four read ports still address exactly the same rows. Full tags and
    // beat masks are still checked by hits; set equality alone is no hit.
    wire reuse_metadata_hit=metadata_snapshot_valid&&
        metadata_sets[0]==cache_set(tap_addr[0])&&
        metadata_sets[1]==cache_set(tap_addr[1])&&
        metadata_sets[2]==cache_set(tap_addr[2])&&
        metadata_sets[3]==cache_set(tap_addr[3])&&(&hits);
    // READ_TAG sampled all payload ways beside synchronous metadata. On the
    // following LOOKUP hit, both are ready: publish directly rather than pay
    // a separate OUTPUT clock. A stalled hit still enters registered OUTPUT.
    wire lookup_complete=state==LOOKUP&&(&hits);
    // Unmasked data in an unsuccessful LOOKUP is never valid. Keep the
    // all-tap hit reduction off the RGB publication data path.
    wire publish_zero=pixels_zero&&state!=LOOKUP;
    // Optional internal preview. The consumer must queue this flag with all
    // four taps and apply it before interpolation; default publication masks.
    assign out_pixels_zero=publish_zero;
    assign in_ready=rst_n&&(state==IDLE||((state==OUTPUT||lookup_complete)&&out_ready));
    assign out_valid=rst_n&&(state==OUTPUT||lookup_complete);
    assign fetch_rd_req_valid=rst_n&&state==REQUEST;
    assign fetch_rd_req_addr=request_addr;
    assign fetch_rd_req_len=request_len;
    assign fetch_rd_req_id=REQ_ID;
    assign fetch_rd_rsp_ready=rst_n&&(state==RESPONSE||state==DRAIN);
    assign P00_rgb=(!DEFER_ZERO_MASK&&publish_zero)?24'd0:sample_rgb[0];
    assign P10_rgb=(!DEFER_ZERO_MASK&&publish_zero)?24'd0:sample_rgb[1];
    assign P01_rgb=(!DEFER_ZERO_MASK&&publish_zero)?24'd0:sample_rgb[2];
    assign P11_rgb=(!DEFER_ZERO_MASK&&publish_zero)?24'd0:sample_rgb[3];
    // The existing REQUEST clock precedes every RESPONSE. Capture compact
    // operands here so the response-count path no longer crosses a wide
    // 32-bit equality before RAM WEA. No new transaction state is needed.
    reg [5:0] response_len /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    reg response_len_invalid /* synthesis syn_preserve = 1 */ /* synthesis syn_keep = 1 */;
    always @(posedge clk)begin
        if(state==REQUEST)begin
            response_len<=request_len[5:0];
            response_len_invalid<=|request_len[31:6];
        end
    end
    // fill_count is unsigned five bits: count+1 is exactly in [1,32]. The
    // high-bit flag preserves the original comparison for every 32-bit len,
    // including zero and malformed/out-of-range headers; no bound is assumed.
    wire [5:0] response_next_count={1'b0,fill_count}+6'd1;
    wire expected_last=!response_len_invalid&&(response_next_count==response_len);
    wire response_bad=fetch_rd_rsp_id!=REQ_ID||fetch_rd_rsp_error||fetch_rd_rsp_last!=expected_last;
    // Address operands are payload, loaded before PREPARE. State/valid masks
    // reset coherently; avoid absorbing their clear into DSP input registers.
    always @(posedge clk)begin
        if(in_valid&&in_ready)begin
            active_stride<=source_stride;
            if(descriptor_ok&&(border_clamp||!outside))begin
                x0<=floor_x[15:0];y0<=floor_y[15:0];
                x1<=floor_x[15:0]<(source_width-16'd1)?floor_x[15:0]+16'd1:floor_x[15:0];
                y1<=floor_y[15:0]<(source_height-16'd1)?floor_y[15:0]+16'd1:floor_y[15:0];
            end
        end
    end
    integer read_tap;
    // Predict the next read states so each way gets its own registered CE.
    // The assertion in the boundary bench checks exact CE/state equivalence.
    // This removes the decoded-state net with 248 RAM loads, without a new
    // FSM state or changing which edge samples the payload.
    wire next_payload_read = !(in_valid&&in_ready)&&
        (state==PREPARE||(state==READ_TAG&&!reuse_metadata_hit)||
        (state==RESPONSE&&fetch_rd_rsp_valid&&fetch_rd_rsp_ready&&!response_bad&&expected_last));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)payload_read_enable<=0;
        else payload_read_enable<={4{next_payload_read}};
    end
    genvar data_tap;
    generate for(data_tap=0;data_tap<4;data_tap=data_tap+1)begin: g_way_select
        // Select the RGB lane before the tag-dependent way mux, keeping each
        // selection load at 24 bits rather than up to eight RGB lanes.
        wire [23:0] rgb_way0 /* synthesis syn_keep = 1 */;
        wire [23:0] rgb_way1 /* synthesis syn_keep = 1 */;
        wire [23:0] rgb_way2 /* synthesis syn_keep = 1 */;
        wire [23:0] rgb_way3 /* synthesis syn_keep = 1 */;
        assign rgb_way0=sample_way0[data_tap][tap_addr[data_tap][4:2]*32+:24];
        assign rgb_way1=sample_way1[data_tap][tap_addr[data_tap][4:2]*32+:24];
        assign rgb_way2=sample_way2[data_tap][tap_addr[data_tap][4:2]*32+:24];
        assign rgb_way3=sample_way3[data_tap][tap_addr[data_tap][4:2]*32+:24];
        wire [3:0] choose_way;
        // The original hit_bank loop gives the highest matching way priority.
        // No match defaults to way0, whose value is not published as valid.
        assign choose_way[3]=way_hits[data_tap][3];
        assign choose_way[2]=way_hits[data_tap][2]&&!way_hits[data_tap][3];
        assign choose_way[1]=way_hits[data_tap][1]&&!(|way_hits[data_tap][3:2]);
        assign choose_way[0]=!(|way_hits[data_tap][3:1]);
        wire [23:0] lookup_rgb=(rgb_way0&{24{choose_way[0]}})|
            (rgb_way1&{24{choose_way[1]}})|(rgb_way2&{24{choose_way[2]}})|
            (rgb_way3&{24{choose_way[3]}});
        wire [23:0] registered_rgb=sample_bank[data_tap]==0?rgb_way0:
            sample_bank[data_tap]==1?rgb_way1:
            sample_bank[data_tap]==2?rgb_way2:rgb_way3;
        assign sample_rgb[data_tap]=state==LOOKUP?lookup_rgb:registered_rgb;
    end endgenerate
    // Full-beat synchronous reads keep byte selection after the payload register.
    // Payload has no reset; pixels_zero masks stale/missed words.
    // Set address comes directly from the tap, even on a miss. Read enable
    // depends only on the read states; hit testing controls publication,
    // not RAM CE. READ_TAG data is published only for a valid reused hit.
    always @(posedge clk)begin
        if(payload_read_enable[0])
            for(read_tap=0;read_tap<4;read_tap=read_tap+1)begin
                // All ways use only set/beat addresses. Register the matching
                // way beside the synchronous RAM samples, then mux its output.
                // This removes the tag-comparator -> payload-RAM address path
                // without adding a transaction state or changing hit latency.
                sample_bank[read_tap]<=hit_bank[read_tap][1:0];
                sample_way0[read_tap]<=data_way0[cache_set(tap_addr[read_tap])*BLOCK_BEATS+tap_addr[read_tap][CACHE_BLOCK_BITS-1:5]];
            end
        if(payload_read_enable[1])
            for(read_tap=0;read_tap<4;read_tap=read_tap+1)begin
                sample_way1[read_tap]<=data_way1[cache_set(tap_addr[read_tap])*BLOCK_BEATS+tap_addr[read_tap][CACHE_BLOCK_BITS-1:5]];
            end
        if(payload_read_enable[2])
            for(read_tap=0;read_tap<4;read_tap=read_tap+1)begin
                sample_way2[read_tap]<=data_way2[cache_set(tap_addr[read_tap])*BLOCK_BEATS+tap_addr[read_tap][CACHE_BLOCK_BITS-1:5]];
            end
        if(payload_read_enable[3])
            for(read_tap=0;read_tap<4;read_tap=read_tap+1)begin
                sample_way3[read_tap]<=data_way3[cache_set(tap_addr[read_tap])*BLOCK_BEATS+tap_addr[read_tap][CACHE_BLOCK_BITS-1:5]];
            end
    end
    // No reset on either RAM payload. Every key change sweeps all metadata
    // rows before reading; publication remains blocked during the sweep.
    integer metadata_tap;
    always @(posedge clk)begin
        if(rst_n&&state==READ_TAG)
            for(metadata_tap=0;metadata_tap<4;metadata_tap=metadata_tap+1)
                metadata_words[metadata_tap]<=metadata_ram[cache_set(tap_addr[metadata_tap])];
        if(rst_n&&state==SWEEP)metadata_ram[sweep_index]<=0;
        else if(rst_n&&state==RESPONSE&&fetch_rd_rsp_valid&&fetch_rd_rsp_ready&&!response_bad&&expected_last)
            metadata_ram[cache_set(tap_addr[selected_missing_tap])]<=write_metadata;
    end
    integer snapshot_tap;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            metadata_snapshot_valid<=0;
            for(snapshot_tap=0;snapshot_tap<4;snapshot_tap=snapshot_tap+1)
                metadata_sets[snapshot_tap]<=0;
        end else if(state==SWEEP)metadata_snapshot_valid<=0;
        else if(state==RESPONSE&&fetch_rd_rsp_valid&&fetch_rd_rsp_ready)
            metadata_snapshot_valid<=0;
        else if(state==READ_TAG)begin
            metadata_snapshot_valid<=1;
            for(snapshot_tap=0;snapshot_tap<4;snapshot_tap=snapshot_tap+1)
                metadata_sets[snapshot_tap]<=cache_set(tap_addr[snapshot_tap]);
        end
    end
    integer k;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;
            active_base<=0;active_end<=0;request_addr<=0;request_len<=0;
            fill_bank<=0;fill_start<=0;fill_count<=0;fill_mask<=0;key_valid<=0;selected_missing_tap<=0;
            sweep_index<=0;sweep_resume<=IDLE;
            key_base<=0;key_stride<=0;key_frame<=0;key_cfg<=0;key_width<=0;key_height<=0;
            dx<=0;dy<=0;out_frame_id<=0;out_pixel_tag<=0;out_cfg_version<=0;out_error<=0;out_border<=0;
            for(k=0;k<4;k=k+1)tap_addr[k]<=0;
            pixels_zero<=1;
            // RAM payload is not reset. Valid masks protect all uninitialized words.
        end else if(in_valid&&in_ready)begin
            out_frame_id<=frame_id;out_pixel_tag<=pixel_tag;out_cfg_version<=cfg_version;
            out_error<=!descriptor_ok;out_border<=outside;
            active_base<=source_base;active_end<=input_end[63:0];
            key_valid<=1;key_base<=source_base;key_stride<=source_stride;
            key_frame<=frame_id;key_cfg<=cfg_version;key_width<=source_width;key_height<=source_height;
            pixels_zero<=1;
            if(!descriptor_ok||(!border_clamp&&outside))begin
                dx<=0;dy<=0;
                if(!same_key)begin sweep_index<=0;sweep_resume<=OUTPUT;state<=SWEEP;end
                else state<=OUTPUT;
            end
            else begin
                dx<=clipped_x[COORD_F-1:0];dy<=clipped_y[COORD_F-1:0];
                if(!same_key)begin sweep_index<=0;sweep_resume<=PREPARE;state<=SWEEP;end
                else state<=PREPARE;
            end
        end else case(state)
            PREPARE:begin
                tap_addr[0]<=active_base+({48'd0,y0}*{32'd0,active_stride})+{46'd0,x0,2'b00};
                tap_addr[1]<=active_base+({48'd0,y0}*{32'd0,active_stride})+{46'd0,x1,2'b00};
                tap_addr[2]<=active_base+({48'd0,y1}*{32'd0,active_stride})+{46'd0,x0,2'b00};
                tap_addr[3]<=active_base+({48'd0,y1}*{32'd0,active_stride})+{46'd0,x1,2'b00};
                state<=READ_TAG;
            end
            READ_TAG:if(reuse_metadata_hit)begin
                pixels_zero<=0;state<=OUTPUT;
            end else state<=LOOKUP;
            SWEEP:begin
                if(sweep_index==CACHE_SETS-1)state<=sweep_resume;
                else sweep_index<=sweep_index+1'b1;
            end
            LOOKUP:if(&hits)begin
                pixels_zero<=0;
                state<=out_ready?IDLE:OUTPUT;
            end else begin
                selected_missing_tap<=missing_tap;state<=MISS_SELECT;
            end
            // Only misses pay this extra cycle; hit throughput is unchanged.
            MISS_SELECT:begin
                request_addr<=start_addr;request_len<=(end_addr-start_addr)>>5;
                fill_bank<=replacement;fill_start<=start_addr[CACHE_BLOCK_BITS-1:5];fill_count<=0;fill_mask<=0;
                fill_metadata<=metadata_words[selected_missing_tap];
                state<=REQUEST;
            end
            REQUEST:if(fetch_rd_req_valid&&fetch_rd_req_ready)state<=RESPONSE;
            RESPONSE:if(fetch_rd_rsp_valid&&fetch_rd_rsp_ready)begin
                if(response_bad)begin
                    out_error<=1;
                    pixels_zero<=1;
                    sweep_index<=0;sweep_resume<=OUTPUT;
                    state<=fetch_rd_rsp_last?SWEEP:DRAIN;
                end else begin
                    case(fill_bank[1:0])
                        0:data_way0[(fill_bank>>2)*BLOCK_BEATS+fill_start+fill_count]<=fetch_rd_rsp_data;
                        1:data_way1[(fill_bank>>2)*BLOCK_BEATS+fill_start+fill_count]<=fetch_rd_rsp_data;
                        2:data_way2[(fill_bank>>2)*BLOCK_BEATS+fill_start+fill_count]<=fetch_rd_rsp_data;
                        3:data_way3[(fill_bank>>2)*BLOCK_BEATS+fill_start+fill_count]<=fetch_rd_rsp_data;
                    endcase
                    fill_mask[fill_start+fill_count]<=1;
                    if(expected_last)state<=READ_TAG;
                    else fill_count<=fill_count+1'b1;
                end
            end
            DRAIN:if(fetch_rd_rsp_valid&&fetch_rd_rsp_ready&&fetch_rd_rsp_last)begin sweep_index<=0;sweep_resume<=OUTPUT;state<=SWEEP;end
            OUTPUT:if(out_ready)state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
