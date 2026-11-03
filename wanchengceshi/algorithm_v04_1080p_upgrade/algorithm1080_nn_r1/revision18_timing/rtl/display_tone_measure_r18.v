// Non-stalling HDMI final stage. Spatial banks and the global bank commit together at VS.
module display_tone_measure_r18 #(
    parameter integer WIDTH=1280, HEIGHT=720, SPLIT_COMPARE=0, SPATIAL=1,
    parameter LUT_FILE="data/display_tone_v07.hex"
)(
    input clk, input rst_n, input enable, input vs, input hs, input de,
    input [23:0] rgb, output reg vs_out, output reg hs_out, output reg de_out,
    output reg [23:0] rgb_out, output [2:0] applied_bank, output [1:0] local_axis,
    output reg [31:0] last_de_count, output reg last_complete,
    output reg [127:0] observation, output wire applied_identity
);
    localparam [31:0] FRAME_PIXELS=WIDTH*HEIGHT;
    localparam [39:0] HALF_PIXELS={8'd0,FRAME_PIXELS}>>1;
    localparam [10:0] IDENTITY={3'd2,2'd0,3'd2,3'd2};
    reg [10:0] committed, pending;
    reg trained, pending_valid, enabled;
    reg previous_vs, previous_de, seen_boundary, bad_layout;
    reg layout_fault_pending;
    reg [16:0] x, y;
    wire boundary=vs && !previous_vs;
    wire [16:0] col=boundary ? 0 : x;
    wire [16:0] row=boundary ? 0 : y;
    wire take=de && (boundary || seen_boundary);
    wire [15:0] weighted={8'd0,rgb[23:16]}*16'd77+{8'd0,rgb[15:8]}*16'd150+
                         {8'd0,rgb[7:0]}*16'd29+16'd128;
    wire [39:0] luma={32'd0,weighted[15:8]};
    reg [39:0] total_sum, left_sum, top_sum;
    wire [39:0] right_sum=total_sum-left_sum, bottom_sum=total_sum-top_sum;
    wire [2:0] global_candidate;
    wire [31:0] count;
    tone_meter_v07 meter(.clk(clk),.rst_n(rst_n),.clear(boundary),.take(take),
        .y(weighted[15:8]),.history(committed[10:8]),.count(count),.decision(global_candidate));

    function [2:0] half_bank;
        input [39:0] sum;
        begin
            if(sum<=HALF_PIXELS*40'd76) half_bank=0;
            else if(sum<=HALF_PIXELS*40'd116) half_bank=1;
            else if(sum>=HALF_PIXELS*40'd196) half_bank=4;
            else if(sum>=HALF_PIXELS*40'd170) half_bank=3;
            else half_bank=2;
        end
    endfunction
    function opposed;
        input [2:0] a,b;
        begin opposed=(a<2 && b>2) || (b<2 && a>2); end
    endfunction
    wire [2:0] left_bank=half_bank(left_sum), right_bank=half_bank(right_sum);
    wire [2:0] top_bank=half_bank(top_sum), bottom_bank=half_bank(bottom_sum);
    wire [39:0] x_delta=left_sum>right_sum ? left_sum-right_sum : right_sum-left_sum;
    wire [39:0] y_delta=top_sum>bottom_sum ? top_sum-bottom_sum : bottom_sum-top_sum;
    localparam LOCAL_OK=SPATIAL && WIDTH>=4 && HEIGHT>=4 && WIDTH%2==0 && HEIGHT%2==0;
    wire lr=LOCAL_OK && opposed(left_bank,right_bank);
    wire tb=LOCAL_OK && opposed(top_bank,bottom_bank);
    wire horizontal=lr && (!tb || x_delta>=y_delta);
    wire [10:0] candidate=horizontal ? {global_candidate,2'd1,left_bank,right_bank} :
                            tb ? {global_candidate,2'd2,top_bank,bottom_bank} :
                                 {global_candidate,2'd0,global_candidate,global_candidate};
    wire complete=seen_boundary && !bad_layout && !layout_fault_pending && count==FRAME_PIXELS && (!previous_de || x==WIDTH);
    wire accept=complete && (!trained || (pending_valid && candidate==pending));
    wire [10:0] next_plan=accept ? candidate : committed;
    wire [10:0] plan=(boundary ? enable : enabled) ? (boundary ? next_plan : committed) : IDENTITY;
    assign applied_identity=!enabled; // R16 actual registered application feedback
    assign applied_bank=enabled ? committed[10:8] : 3'd2;
    assign local_axis=enabled ? committed[7:6] : 2'd0;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            previous_vs<=0; previous_de<=0; seen_boundary<=0; enabled<=0;
            x<=0; y<=0; bad_layout<=0; total_sum<=0; top_sum<=0; left_sum<=0;
            committed<=IDENTITY; pending<=IDENTITY; pending_valid<=0; trained<=0;
            last_de_count<=0; last_complete<=0;
        end else begin
            previous_vs<=vs; previous_de<=de;
            x<=de ? (boundary ? 17'd1 : x+17'd1) : 17'd0;
            if(boundary) begin
                y<=0; seen_boundary<=1; enabled<=enable; bad_layout<=0;
                last_de_count<=count; last_complete<=complete;
                committed<=next_plan; pending<=candidate; pending_valid<=complete;
                if(complete) trained<=1;
                total_sum<=de ? luma : 0; top_sum<=de ? luma : 0; left_sum<=de ? luma : 0;
            end else begin
                if(layout_fault_pending) bad_layout<=1;
                if(previous_de && !de) begin
                    y<=y+17'd1;
                    // Layout checks retire through the dedicated fault register below.
                end
                if(take) begin
                    // Bounds are checked against registered x/y, not the VS-selected col/row.
                    total_sum<=total_sum+luma;
                    if(col<WIDTH/2) left_sum<=left_sum+luma;
                    if(row<HEIGHT/2) top_sum<=top_sum+luma;
                end
            end
        end
    end

    // One-cycle layout checker. The next boundary consumes the pending flag
    // together with sticky bad_layout, so an error on the last frame cycle is
    // never lost. VS only gates this stage after the registered-coordinate
    // comparisons; it is absent from their arithmetic/mux cone.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) layout_fault_pending<=1'b0;
        else layout_fault_pending <= !boundary &&
            ((previous_de && !de && x!=WIDTH) ||
             (de && seen_boundary && (x>=WIDTH || y>=HEIGHT || count==FRAME_PIXELS)));
    end

    // Power-of-two transition width around the centre; no pixel-rate division.
    function integer ramp_bits;
        input integer dimension;
        integer k;
        begin
            k=0;
            while ((2<<k)<=dimension/4) k=k+1;
            ramp_bits=k;
        end
    endfunction
    localparam X_BITS=ramp_bits(WIDTH), Y_BITS=ramp_bits(HEIGHT);
    localparam X_SPAN=1<<X_BITS, Y_SPAN=1<<Y_BITS;
    localparam X_START=WIDTH/2-X_SPAN/2, Y_START=HEIGHT/2-Y_SPAN/2;
    wire [22:0] x_num=({6'd0,col}-X_START)*32+X_SPAN/2;
    wire [22:0] y_num=({6'd0,row}-Y_START)*32+Y_SPAN/2;
    wire [5:0] wx=(col<=X_START) ? 0 : (col>=X_START+X_SPAN) ? 32 : x_num>>X_BITS;
    wire [5:0] wy=(row<=Y_START) ? 0 : (row>=Y_START+Y_SPAN) ? 32 : y_num>>Y_BITS;
    wire [5:0] fraction=plan[7:6]==1 ? wx : plan[7:6]==2 ? wy : 6'd0;
    wire [2:0] a_bank=plan[5:3], b_bank=plan[2:0];
    reg [7:0] lut[0:1535];
    initial $readmemh(LUT_FILE,lut);
    reg [23:0] a_rgb,b_rgb;
    reg [5:0] mix_q5;
    reg [2:0] timing;
    wire show_raw=!de || (SPLIT_COMPARE && col<WIDTH/2);
    wire marker=de && SPLIT_COMPARE && col==WIDTH/2;

    function [7:0] blend;
        input [7:0] a,b;
        input [5:0] weight;
        reg [13:0] value;
        begin
            value=a*(6'd32-weight)+b*weight+14'd16;
            blend=value[12:5];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            a_rgb<=0; b_rgb<=0; mix_q5<=0; timing<=0;
            rgb_out<=0; vs_out<=0; hs_out<=0; de_out<=0;
        end else begin
            timing<={vs,hs,de}; {vs_out,hs_out,de_out}<=timing;
            mix_q5<=show_raw || marker ? 6'd0 : fraction;
            if(marker) a_rgb<=24'hff00ff;
            else if(show_raw || a_bank==2) a_rgb<=rgb;
            else a_rgb<={lut[{a_bank,rgb[23:16]}],lut[{a_bank,rgb[15:8]}],lut[{a_bank,rgb[7:0]}]};
            if(!show_raw && !marker && fraction!=0)
                b_rgb<={lut[{b_bank,rgb[23:16]}],lut[{b_bank,rgb[15:8]}],lut[{b_bank,rgb[7:0]}]};
            rgb_out<={blend(a_rgb[23:16],b_rgb[23:16],mix_q5),
                      blend(a_rgb[15:8],b_rgb[15:8],mix_q5),
                      blend(a_rgb[7:0],b_rgb[7:0],mix_q5)};
        end
    end

    // R15_DISPLAY_OBSERVATION_BEGIN
    // Frozen A8 payload layout: sum[39:0], count[71:40], frame[103:72],
    // cfg[119:104], applied[122:120], next[125:123], enable[126], valid[127].
    // Bank 5 denotes mixed processing (global mixed or spatial); this legacy
    // payload cannot distinguish them. cfg ffff is not a geometry ACK version.
    reg [31:0] display_frame_number;
    wire [2:0] completed_applied_bank = !enabled ? 3'd2 :
        (committed[7:6] != 2'd0 ? 3'd5 : committed[10:8]);
    wire [2:0] boundary_next_bank = !enable ? 3'd2 :
        (next_plan[7:6] != 2'd0 ? 3'd5 : next_plan[10:8]);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            display_frame_number <= 32'd0;
            observation <= 128'd0;
        end else if (boundary) begin
            observation <= {complete, enabled, boundary_next_bank,
                            completed_applied_bank, 16'hffff,
                            display_frame_number, count, total_sum};
            display_frame_number <= display_frame_number + 32'd1;
        end
    end
    // R15_DISPLAY_OBSERVATION_END
endmodule
