`timescale 1ns/1ps
// Offline candidate. Two synchronous payload ports, four ways, ordered ROB.
// A quad sharing its two row beats costs one read command. Crossing a beat
// costs a second command. Misses use the preserved protocol/error controller;
// its accepted DDR beats populate this independent larger cache.
module pixel_fetch4_stream2_cached_r24 #(
    parameter integer COORD_W=26, COORD_F=10,
    parameter [7:0] REQ_ID=8'hA6,
    parameter integer CACHE_SET_BITS=10, CACHE_BLOCK_BITS=9, HASH_FOLD_OFFSET=1,
    parameter integer DEFER_ZERO_MASK=0
)(
    input wire clk,rst_n,in_valid, output wire in_ready,
    input wire signed [COORD_W-1:0] src_x_fixed,src_y_fixed,
    input wire [15:0] source_width,source_height,
    input wire [63:0] source_base,
    input wire [31:0] source_stride,frame_id,pixel_tag,
    input wire [15:0] cfg_version, input wire border_clamp,
    output wire fetch_rd_req_valid,input wire fetch_rd_req_ready,
    output wire [63:0] fetch_rd_req_addr,output wire [31:0] fetch_rd_req_len,
    output wire [7:0] fetch_rd_req_id,
    input wire fetch_rd_rsp_valid,output wire fetch_rd_rsp_ready,
    input wire [255:0] fetch_rd_rsp_data,input wire [7:0] fetch_rd_rsp_id,
    input wire fetch_rd_rsp_last,fetch_rd_rsp_error,
    output wire out_valid,input wire out_ready,
    output wire [23:0] P00_rgb,P10_rgb,P01_rgb,P11_rgb,
    output wire [COORD_F-1:0] dx,dy,
    output wire [31:0] out_frame_id,out_pixel_tag,
    output wire [15:0] out_cfg_version,
    output wire out_error,out_border,out_pixels_zero
);
    localparam BB=CACHE_BLOCK_BITS-5, NB=1<<BB, TB=64-CACHE_BLOCK_BITS;
    localparam WB=TB+NB, MW=4*WB+2, NS=1<<CACHE_SET_BITS, AW=CACHE_SET_BITS+BB;
    localparam Q=8;
    reg key_valid,sweep;
    reg [CACHE_SET_BITS-1:0] sweep_index;
    reg [63:0] key_base;
    reg [31:0] key_stride,key_frame;
    reg [15:0] key_width,key_height,key_cfg;
    reg [3:0] count,work_count;
    reg [2:0] head,tail,issue_pointer;
    reg issue_phase;
    reg [7:0] ready_bits,miss_bits,need_bits,left_hit;
    reg [63:0] a0[0:Q-1],a1[0:Q-1],a2[0:Q-1],a3[0:Q-1];
    reg signed [COORD_W-1:0] qx[0:Q-1],qy[0:Q-1];
    reg [COORD_F-1:0] qdx[0:Q-1],qdy[0:Q-1];
    reg [31:0] qtag[0:Q-1];
    reg [23:0] p0[0:Q-1],p1[0:Q-1],p2[0:Q-1],p3[0:Q-1];
    reg [7:0] qerror,qborder,qzero,qclamp,qcross;
    wire same_key=key_valid&&key_base==source_base&&key_stride==source_stride&&
        key_frame==frame_id&&key_cfg==cfg_version&&key_width==source_width&&key_height==source_height;
    wire signed [63:0] ix={{(64-COORD_W){src_x_fixed[COORD_W-1]}},src_x_fixed};
    wire signed [63:0] iy={{(64-COORD_W){src_y_fixed[COORD_W-1]}},src_y_fixed};
    wire signed [63:0] mx=($signed({48'd0,source_width})-64'sd1)<<<COORD_F;
    wire signed [63:0] my=($signed({48'd0,source_height})-64'sd1)<<<COORD_F;
    wire outside=ix<0||iy<0||ix>mx||iy>my;
    wire signed [63:0] cx=ix<0?64'sd0:(ix>mx?mx:ix),cy=iy<0?64'sd0:(iy>my?my:iy);
    wire [15:0] x0=cx>>>COORD_F,y0=cy>>>COORD_F;
    wire [15:0] x1=x0<source_width-1?x0+16'd1:x0,y1=y0<source_height-1?y0+16'd1:y0;
    wire [64:0] frame_end={1'b0,source_base}+({49'd0,source_height}*{33'd0,source_stride});
    wire desc_ok=source_width!=0&&source_height!=0&&source_base[4:0]==0&&source_stride[4:0]==0&&
        source_stride>={14'd0,source_width,2'b00}&&!frame_end[64];
    wire no_read=!desc_ok||(!border_clamp&&outside);
    // Reserve a ROB slot at acceptance. Pending slots cannot issue or retire
    // until the three address stages finish, including zero/error pixels.
    wire accept;
    reg [7:0] addr_pending;
    reg addr_v0,addr_v1;
    reg [2:0] addr_slot0,addr_slot1;
    reg [15:0] ax0,ax1,ay0,ay1;
    reg [63:0] abase;
    reg [31:0] astride;
    reg [47:0] row0_product,row1_product;
    reg [63:0] base_x0,base_x1;
    reg addr_cross0,addr_cross1;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin addr_v0<=0;addr_v1<=0;end
        else begin
            addr_v0<=accept;addr_v1<=addr_v0;
            if(accept)begin
                addr_slot0<=tail;ax0<=x0;ax1<=x1;ay0<=y0;ay1<=y1;
                abase<=source_base;astride<=source_stride;
                // Valid descriptors have 32-byte aligned base and row stride.
                addr_cross0<=x0[2:0]==3'd7&&x1!=x0;
            end
            if(addr_v0)begin
                addr_slot1<=addr_slot0;addr_cross1<=addr_cross0;
                row0_product<=ay0*astride;row1_product<=ay1*astride;
                base_x0<=abase+{46'd0,ax0,2'b00};
                base_x1<=abase+{46'd0,ax1,2'b00};
            end
        end
    end
    wire retire;
    assign in_ready=rst_n&&same_key&&!sweep&&(count<Q||retire);
    assign accept=in_valid&&in_ready;
    assign out_valid=rst_n&&count!=0&&ready_bits[head];
    assign retire=out_valid&&out_ready;
    assign P00_rgb=qzero[head]?24'd0:p0[head];assign P10_rgb=qzero[head]?24'd0:p1[head];
    assign P01_rgb=qzero[head]?24'd0:p2[head];assign P11_rgb=qzero[head]?24'd0:p3[head];
    assign dx=qdx[head];assign dy=qdy[head];assign out_pixel_tag=qtag[head];
    assign out_frame_id=key_frame;assign out_cfg_version=key_cfg;
    assign out_error=qerror[head];assign out_border=qborder[head];assign out_pixels_zero=qzero[head];
    function [CACHE_SET_BITS-1:0] set_of(input [63:0] addr);
        set_of=(addr>>CACHE_BLOCK_BITS)^(addr>>(CACHE_BLOCK_BITS+CACHE_SET_BITS+HASH_FOLD_OFFSET));
    endfunction
    function [AW-1:0] word_of(input [63:0] addr);
        word_of={set_of(addr),addr[CACHE_BLOCK_BITS-1:5]};
    endfunction
    localparam M_IDLE=0,M_START=1,M_READ=2,M_SELECT=3,M_REQUEST=4,M_RESPONSE=5,
               M_REPLAY0=6,M_REPLAY1=7,M_WAIT=8,M_DRAIN=9;
    reg [3:0] miss_state;
    reg [3:0] tap_hits[0:Q-1];
    reg [63:0] missing_addr;
    wire fallback_busy=miss_state!=M_IDLE;
    reg rd_pipe_valid,rd_pipe_phase;
    reg lookup_valid,lookup_phase;
    reg [2:0] lookup_slot;
    reg [63:0] lookup_addr0,lookup_addr1;
    reg [2:0] rd_pipe_slot;
    reg [63:0] rd_addr0,rd_addr1;
    reg [63:0] fill_addr,fill_addr2,response_addr;
    reg prefetch;
    reg [5:0] first_beats;
    reg [1:0] fill_way2;
    reg [MW-1:0] fill_meta2;
    reg [31:0] fill_len;
    reg [5:0] fill_count;
    reg [1:0] fill_way;
    reg [MW-1:0] fill_meta;
    wire response_second=prefetch&&fill_count>=first_beats;
    wire [1:0] response_way=response_second?fill_way2:fill_way;
    wire [MW-1:0] response_meta=response_second?fill_meta2:fill_meta;
    reg fill_bad;
    reg payload_commit_valid;
    wire can_issue=!payload_commit_valid&&!addr_pending[issue_pointer]&&!sweep&&!fallback_busy&&(!( |miss_bits)||issue_phase)&&work_count!=0;
    wire skip_issue=can_issue&&!need_bits[issue_pointer];
    wire normal_issue=can_issue&&need_bits[issue_pointer];
    wire replay=miss_state==M_REPLAY0||miss_state==M_REPLAY1;
    wire issue=normal_issue||replay;
    wire read_phase=replay?(miss_state==M_REPLAY1):issue_phase;
    wire [2:0] read_slot=replay?head:issue_pointer;
    wire issue_done=skip_issue||(normal_issue&&(issue_phase||!qcross[issue_pointer]));
    wire [63:0] issue_addr0=read_phase?a1[read_slot]:a0[read_slot];
    wire [63:0] issue_addr1=read_phase?a3[read_slot]:a2[read_slot];
    wire [AW-1:0] issue_word0=word_of(issue_addr0);
    wire [AW-1:0] issue_word1=word_of(issue_addr1);
    // Register the complete RAM request, including the folded word address.
    // The measured ROB selection -> distributed BRAM CE/address path ends here.
    // No request is dropped: fallback and replay wait for this stage to drain.
    reg request_valid,request_phase;
    reg [2:0] request_slot;
    reg [63:0] request_addr0,request_addr1;
    reg [AW-1:0] request_word0,request_word1;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)request_valid<=0;
        else begin
            request_valid<=issue;
            if(issue)begin
                request_slot<=read_slot;request_phase<=read_phase;
                request_addr0<=issue_addr0;request_addr1<=issue_addr1;
                request_word0<=issue_word0;request_word1<=issue_word1;
            end
        end
    reg [MW-1:0] metadata[0:NS-1];
    reg [MW-1:0] meta0,meta1;
    wire [63:0] clipped_start;wire [64:0] block_end;
    wire alloc_read=miss_state==M_START;
    wire [CACHE_SET_BITS-1:0] meta_addr0=alloc_read?set_of(clipped_start):request_word0[AW-1:BB];
    wire [CACHE_SET_BITS-1:0] meta_addr1=alloc_read?set_of(block_end[63:0]):request_word1[AW-1:BB];
    wire response_accept=fetch_rd_rsp_valid&&fetch_rd_rsp_ready;
    wire [3:0] response_taps={response_addr[63:5]==a3[head][63:5],
        response_addr[63:5]==a2[head][63:5],response_addr[63:5]==a1[head][63:5],
        response_addr[63:5]==a0[head][63:5]};
    wire response_completes_quad=&(tap_hits[head]|response_taps);
    wire expected_last={1'b0,fill_count}+7'd1=={1'b0,fill_len};
    wire response_bad=fetch_rd_rsp_id!=REQ_ID||fetch_rd_rsp_error||fetch_rd_rsp_last!=expected_last;
    wire payload_write=rst_n&&miss_state==M_RESPONSE&&response_accept&&!fill_bad&&!response_bad;
    // Keep the existing one-cycle payload data/valid stage. R7 separately
    // captures address and RAM control locally on this same accepted-response edge.
    reg [1023:0] payload_commit_data /* synthesis syn_maxfan=4 */;
    integer write_way;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)payload_commit_valid<=0;
        else begin
            payload_commit_valid<=payload_write;
            if(payload_write)begin
                for(write_way=0;write_way<4;write_way=write_way+1)
                    if(response_way==write_way)
                        payload_commit_data[write_way*256+:256]<=fetch_rd_rsp_data;
            end
        end
    reg [MW-1:0] updated_meta;
    reg [NB-1:0] updated_mask;
    always @* begin
        updated_meta=response_meta;
        updated_mask=response_meta[response_way*WB+:NB];
        updated_mask[response_addr[CACHE_BLOCK_BITS-1:5]]=1;
        updated_meta[response_way*WB+:WB]={response_addr[63:CACHE_BLOCK_BITS],updated_mask};
        updated_meta[MW-1-:2]=response_way+2'd1;
    end
    wire meta_write=sweep||payload_write;
    wire [CACHE_SET_BITS-1:0] meta_write_addr=sweep?sweep_index:set_of(response_addr);
    wire [MW-1:0] meta_write_data=sweep?{MW{1'b0}}:updated_meta;
    wire meta_a_enable=rst_n&&(meta_write||alloc_read||request_valid);
    wire [CACHE_SET_BITS-1:0] meta_a_address=meta_write?meta_write_addr:meta_addr0;
    // Same-address read/write READ_FIRST template permits true dual port RAM.
    always @(posedge clk)if(meta_a_enable)begin
        if(meta_write)metadata[meta_a_address]<=meta_write_data;
        meta0<=metadata[meta_a_address];
    end
    always @(posedge clk)if(rst_n&&(request_valid||alloc_read))meta1<=metadata[meta_addr1];
    wire ram_ready,ram_valid0,ram_valid1;
    wire [1023:0] ram_data0,ram_data1;
    payload_rgb24_store_banked_r21 #(.ADDR_W(AW)) payload(
        .clk(clk),.rst_n(rst_n),.wr_en(payload_commit_valid),.wr_data(payload_commit_data),
        .wr_capture(payload_write),.wr_capture_way(response_way),
        .wr_capture_word(word_of(response_addr)),
        .rd0_en(request_valid),.rd1_en(request_valid),.rd0_addr(request_word0),.rd1_addr(request_word1),
        .rd0_issue(issue),.rd1_issue(issue),.rd0_issue_word(issue_word0),.rd1_issue_word(issue_word1),
        .rd_ready(ram_ready),.out0_valid(ram_valid0),.out1_valid(ram_valid1),
        .out0_data(ram_data0),.out1_data(ram_data1));
    reg lookup_hit0,lookup_hit1;reg [1:0] lookup_way0,lookup_way1;
    integer j;
    always @* begin
        lookup_hit0=0;lookup_hit1=0;lookup_way0=0;lookup_way1=0;
        for(j=0;j<4;j=j+1)begin
            if(meta0[j*WB+:NB]!=0&&meta0[j*WB+NB+:TB]==lookup_addr0[63:CACHE_BLOCK_BITS]&&
               meta0[j*WB+lookup_addr0[CACHE_BLOCK_BITS-1:5]])begin lookup_hit0=1;lookup_way0=j;end
            if(meta1[j*WB+:NB]!=0&&meta1[j*WB+NB+:TB]==lookup_addr1[63:CACHE_BLOCK_BITS]&&
               meta1[j*WB+lookup_addr1[CACHE_BLOCK_BITS-1:5]])begin lookup_hit1=1;lookup_way1=j;end
        end
    end
    reg hit0,hit1;reg [1:0] way0,way1;
    reg bank_valid,bank_phase,bank_hit0,bank_hit1;
    reg [2:0] bank_slot;reg [1:0] bank_way0,bank_way1;
    reg [63:0] bank_addr0,bank_addr1;
    wire [23:0] rgb_left0=ram_data0[way0*256+rd_addr0[4:2]*32+:24];
    wire [23:0] rgb_left1=ram_data1[way1*256+rd_addr1[4:2]*32+:24];
    wire [23:0] rgb_right0=ram_data0[way0*256+a1[rd_pipe_slot][4:2]*32+:24];
    wire [23:0] rgb_right1=ram_data1[way1*256+a3[rd_pipe_slot][4:2]*32+:24];
    reg [MW-1:0] alloc_meta0,alloc_meta1;
    // Issue allocation RAM addresses in M_START; capture in M_READ.
    // M_SELECT retains its original cycle and uses only saved RAM outputs.
    always @(posedge clk)if(rst_n&&miss_state==M_READ)begin
        alloc_meta0<=meta0;alloc_meta1<=meta1;
    end
    reg [3:0] pinned;reg [1:0] chosen_way,candidate;reg found;
    reg [MW-1:0] allocated_meta;
    integer k;
    always @* begin
        pinned=0;chosen_way=0;found=0;candidate=0;allocated_meta=alloc_meta0;
        for(k=0;k<4;k=k+1)begin
            if(alloc_meta0[k*WB+:NB]!=0&&(
                alloc_meta0[k*WB+NB+:TB]==a0[head][63:CACHE_BLOCK_BITS]||
                alloc_meta0[k*WB+NB+:TB]==a1[head][63:CACHE_BLOCK_BITS]||
                alloc_meta0[k*WB+NB+:TB]==a2[head][63:CACHE_BLOCK_BITS]||
                alloc_meta0[k*WB+NB+:TB]==a3[head][63:CACHE_BLOCK_BITS]))pinned[k]=1;
        end
        for(k=0;k<4;k=k+1)begin
            candidate=alloc_meta0[MW-1-:2]+k;
            if(!found&&!pinned[candidate])begin chosen_way=candidate;found=1;end
        end
        for(k=0;k<4;k=k+1)
            if(alloc_meta0[k*WB+:NB]!=0&&alloc_meta0[k*WB+NB+:TB]==fill_addr[63:CACHE_BLOCK_BITS])chosen_way=k;
        if(alloc_meta0[chosen_way*WB+NB+:TB]!=fill_addr[63:CACHE_BLOCK_BITS])
            allocated_meta[chosen_way*WB+:WB]={fill_addr[63:CACHE_BLOCK_BITS],{NB{1'b0}}};
    end
    reg [3:0] pinned2;reg [1:0] chosen_way2,candidate2;reg found2;
    reg [MW-1:0] allocated_meta2;
    integer k2;
    always @* begin
        pinned2=0;chosen_way2=0;found2=0;candidate2=0;allocated_meta2=alloc_meta1;
        for(k2=0;k2<4;k2=k2+1)begin
            if(alloc_meta1[k2*WB+:NB]!=0&&(
                alloc_meta1[k2*WB+NB+:TB]==a0[head][63:CACHE_BLOCK_BITS]||
                alloc_meta1[k2*WB+NB+:TB]==a1[head][63:CACHE_BLOCK_BITS]||
                alloc_meta1[k2*WB+NB+:TB]==a2[head][63:CACHE_BLOCK_BITS]||
                alloc_meta1[k2*WB+NB+:TB]==a3[head][63:CACHE_BLOCK_BITS]))pinned2[k2]=1;
        end
        for(k2=0;k2<4;k2=k2+1)begin
            candidate2=alloc_meta1[MW-1-:2]+k2;
            if(!found2&&!pinned2[candidate2])begin chosen_way2=candidate2;found2=1;end
        end
        for(k2=0;k2<4;k2=k2+1)
            if(alloc_meta1[k2*WB+:NB]!=0&&alloc_meta1[k2*WB+NB+:TB]==fill_addr2[63:CACHE_BLOCK_BITS])chosen_way2=k2;
        if(alloc_meta1[chosen_way2*WB+NB+:TB]!=fill_addr2[63:CACHE_BLOCK_BITS])
            allocated_meta2[chosen_way2*WB+:WB]={fill_addr2[63:CACHE_BLOCK_BITS],{NB{1'b0}}};
    end
    reg second_full;
    integer complete_way;
    always @* begin
        second_full=0;
        for(complete_way=0;complete_way<4;complete_way=complete_way+1)
            if(alloc_meta1[complete_way*WB+:NB]=={NB{1'b1}}&&
               alloc_meta1[complete_way*WB+NB+:TB]==fill_addr2[63:CACHE_BLOCK_BITS])second_full=1;
    end
    wire start_fallback=rst_n&&!payload_commit_valid&&!sweep&&count!=0&&miss_bits[head]&&!fallback_busy&&!request_valid&&!lookup_valid&&!bank_valid&&!rd_pipe_valid&&!issue_phase;
    wire fb_in_ready=1'b1; // retained observation point for model counters
    assign fetch_rd_req_valid=rst_n&&miss_state==M_REQUEST;
    assign fetch_rd_req_addr=fill_addr;assign fetch_rd_req_len=fill_len;assign fetch_rd_req_id=REQ_ID;
    assign fetch_rd_rsp_ready=rst_n&&(miss_state==M_RESPONSE||miss_state==M_DRAIN);
    wire [63:0] block_start=(missing_addr>>CACHE_BLOCK_BITS)<<CACHE_BLOCK_BITS;
    assign block_end={1'b0,block_start}+(65'd1<<CACHE_BLOCK_BITS);
    wire [64:0] source_end={1'b0,key_base}+({49'd0,key_height}*{33'd0,key_stride});
    assign clipped_start=block_start<key_base?key_base:block_start;
    wire [63:0] clipped_end=block_end>source_end?source_end[63:0]:block_end[63:0];
    wire [64:0] paired_end=block_end+(65'd1<<CACHE_BLOCK_BITS);
    // Equal-set pairs fall back to one block, so two allocation snapshots can
    // never overwrite each other. Clipping and full tags remain exact.
    wire can_prefetch=CACHE_BLOCK_BITS==8&&!block_end[64]&&paired_end<=source_end&&
        set_of(clipped_start)!=set_of(block_end[63:0]);
    always @* begin
        if(!tap_hits[head][0])missing_addr=a0[head];
        else if(!tap_hits[head][1])missing_addr=a1[head];
        else if(!tap_hits[head][2])missing_addr=a2[head];
        else missing_addr=a3[head];
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            key_valid<=0;sweep<=0;sweep_index<=0;key_base<=0;key_stride<=0;key_frame<=0;
            key_width<=0;key_height<=0;key_cfg<=0;head<=0;tail<=0;issue_pointer<=0;
            addr_pending<=0;count<=0;work_count<=0;ready_bits<=0;miss_bits<=0;need_bits<=0;left_hit<=0;
            qerror<=0;qborder<=0;qzero<=0;qclamp<=0;qcross<=0;issue_phase<=0;
            bank_valid<=0;rd_pipe_valid<=0;rd_pipe_phase<=0;rd_pipe_slot<=0;rd_addr0<=0;rd_addr1<=0;
            lookup_valid<=0;lookup_phase<=0;lookup_slot<=0;lookup_addr0<=0;lookup_addr1<=0;
            hit0<=0;hit1<=0;way0<=0;way1<=0;
            miss_state<=M_IDLE;fill_addr2<=0;response_addr<=0;prefetch<=0;first_beats<=0;fill_way2<=0;fill_meta2<=0;fill_addr<=0;fill_len<=0;fill_count<=0;fill_way<=0;fill_meta<=0;fill_bad<=0;
        end else begin
            if(!same_key&&in_valid&&count==0&&!sweep)begin
                key_valid<=1;key_base<=source_base;key_stride<=source_stride;key_frame<=frame_id;
                key_width<=source_width;key_height<=source_height;key_cfg<=cfg_version;
                sweep<=1;sweep_index<=0;
            end
            if(sweep)begin
                if(sweep_index==NS-1)sweep<=0;else sweep_index<=sweep_index+1'b1;
            end
            case({accept,retire})2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:;endcase
            case({accept,issue_done})2'b10:work_count<=work_count+1'b1;2'b01:work_count<=work_count-1'b1;default:;endcase
            if(accept)begin
                addr_pending[tail]<=1;
                qx[tail]<=src_x_fixed;qy[tail]<=src_y_fixed;qtag[tail]<=pixel_tag;qclamp[tail]<=border_clamp;
                qdx[tail]<=no_read?0:cx[COORD_F-1:0];qdy[tail]<=no_read?0:cy[COORD_F-1:0];
                qerror[tail]<=!desc_ok;qborder[tail]<=outside;qzero[tail]<=no_read;

                ready_bits[tail]<=0;need_bits[tail]<=!no_read;miss_bits[tail]<=0;left_hit[tail]<=0;tap_hits[tail]<=0;
                tail<=tail+1'b1;
            end
            if(addr_v1)begin
                a0[addr_slot1]<=base_x0+{16'd0,row0_product};
                a1[addr_slot1]<=base_x1+{16'd0,row0_product};
                a2[addr_slot1]<=base_x0+{16'd0,row1_product};
                a3[addr_slot1]<=base_x1+{16'd0,row1_product};
                qcross[addr_slot1]<=addr_cross1;
                addr_pending[addr_slot1]<=0;
                ready_bits[addr_slot1]<=qzero[addr_slot1];
            end
            if(retire)begin ready_bits[head]<=0;miss_bits[head]<=0;head<=head+1'b1;end
            lookup_valid<=request_valid;bank_valid<=lookup_valid;rd_pipe_valid<=bank_valid;
            if(lookup_valid)begin
                bank_slot<=lookup_slot;bank_phase<=lookup_phase;
                bank_addr0<=lookup_addr0;bank_addr1<=lookup_addr1;
                bank_hit0<=lookup_hit0;bank_hit1<=lookup_hit1;bank_way0<=lookup_way0;bank_way1<=lookup_way1;
            end
            if(bank_valid)begin
                rd_pipe_slot<=bank_slot;rd_pipe_phase<=bank_phase;
                rd_addr0<=bank_addr0;rd_addr1<=bank_addr1;
                hit0<=bank_hit0;hit1<=bank_hit1;way0<=bank_way0;way1<=bank_way1;
            end
            if(request_valid)begin
                lookup_slot<=request_slot;lookup_phase<=request_phase;
                lookup_addr0<=request_addr0;lookup_addr1<=request_addr1;
            end
            if(normal_issue)issue_phase<=qcross[issue_pointer]&&!issue_phase;
            if(issue_done)begin issue_pointer<=issue_pointer+1'b1;issue_phase<=0;end
            if(rd_pipe_valid)begin
                if(!rd_pipe_phase)begin
                    p0[rd_pipe_slot]<=rgb_left0;p2[rd_pipe_slot]<=rgb_left1;
                    left_hit[rd_pipe_slot]<=hit0&&hit1;
                    tap_hits[rd_pipe_slot][0]<=hit0;tap_hits[rd_pipe_slot][2]<=hit1;
                    if(!qcross[rd_pipe_slot])begin
                        p1[rd_pipe_slot]<=rgb_right0;p3[rd_pipe_slot]<=rgb_right1;
                        tap_hits[rd_pipe_slot][1]<=hit0;tap_hits[rd_pipe_slot][3]<=hit1;
                        ready_bits[rd_pipe_slot]<=hit0&&hit1;miss_bits[rd_pipe_slot]<=!(hit0&&hit1);
                    end else if(!(hit0&&hit1))miss_bits[rd_pipe_slot]<=1;
                end else begin
                    p1[rd_pipe_slot]<=rgb_left0;p3[rd_pipe_slot]<=rgb_left1;
                    tap_hits[rd_pipe_slot][1]<=hit0;tap_hits[rd_pipe_slot][3]<=hit1;
                    ready_bits[rd_pipe_slot]<=left_hit[rd_pipe_slot]&&hit0&&hit1;
                    miss_bits[rd_pipe_slot]<=!(left_hit[rd_pipe_slot]&&hit0&&hit1);
                end
            end
            // A younger miss may have been satisfied by an older fill. Recheck before allocating.
            if(start_fallback)miss_state<=M_REPLAY0;
            case(miss_state)
                M_START:begin
                    fill_addr<=clipped_start;fill_addr2<=block_end[63:0];response_addr<=clipped_start;
                    first_beats<=(clipped_end-clipped_start)>>5;prefetch<=can_prefetch;
                    fill_len<=can_prefetch?(paired_end-{1'b0,clipped_start})>>5:(clipped_end-clipped_start)>>5;
                    fill_count<=0;fill_bad<=0;miss_state<=M_READ;
                end
                M_READ:miss_state<=M_SELECT;
                M_SELECT:begin
                    fill_way<=chosen_way;fill_meta<=allocated_meta;fill_way2<=chosen_way2;fill_meta2<=allocated_meta2;
                    if(second_full)begin prefetch<=0;fill_len<=first_beats;end
                    miss_state<=M_REQUEST;
                end
                M_REQUEST:if(fetch_rd_req_valid&&fetch_rd_req_ready)miss_state<=M_RESPONSE;
                M_RESPONSE:if(response_accept)begin
                    if(response_bad)begin
                        qerror[head]<=1;qzero[head]<=1;fill_bad<=1;
                        if(fetch_rd_rsp_last)begin
                            ready_bits[head]<=1;miss_bits[head]<=0;sweep<=1;sweep_index<=0;miss_state<=M_IDLE;
                        end else miss_state<=M_DRAIN;
                    end else begin
                        // A head miss cannot retire until the complete burst
                        // is checked. Existing head tags are pinned during
                        // replacement, so earlier valid RGB snapshots survive.
                        if(response_taps[0])p0[head]<=fetch_rd_rsp_data[a0[head][4:2]*32+:24];
                        if(response_taps[1])p1[head]<=fetch_rd_rsp_data[a1[head][4:2]*32+:24];
                        if(response_taps[2])p2[head]<=fetch_rd_rsp_data[a2[head][4:2]*32+:24];
                        if(response_taps[3])p3[head]<=fetch_rd_rsp_data[a3[head][4:2]*32+:24];
                        tap_hits[head]<=tap_hits[head]|response_taps;
                        if(response_second)fill_meta2<=updated_meta;else fill_meta<=updated_meta;
                        response_addr<=response_addr+64'd32;fill_count<=fill_count+1'b1;
                        if(expected_last)begin
                            if(response_completes_quad)begin
                                ready_bits[head]<=1;miss_bits[head]<=0;miss_state<=M_IDLE;
                            end else miss_state<=M_START;
                        end
                    end
                end
                M_DRAIN:if(response_accept&&fetch_rd_rsp_last)begin
                    ready_bits[head]<=1;miss_bits[head]<=0;sweep<=1;sweep_index<=0;miss_state<=M_IDLE;
                end
                M_REPLAY0:miss_state<=qcross[head]?M_REPLAY1:M_WAIT;
                M_REPLAY1:miss_state<=M_WAIT;
                M_WAIT:if(!request_valid&&!lookup_valid&&!bank_valid&&!rd_pipe_valid)miss_state<=ready_bits[head]?M_IDLE:M_START;
                default:;
            endcase
        end
    end
endmodule
