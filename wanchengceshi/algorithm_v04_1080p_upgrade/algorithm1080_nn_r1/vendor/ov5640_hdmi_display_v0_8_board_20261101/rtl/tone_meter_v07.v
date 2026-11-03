// All comparisons use 36 bits (16 * (2^32-1) fits). Clear may coincide
// with the first accepted pixel of a new raster frame.
module tone_meter_v07(
    input clk, input rst_n, input clear, input take, input [7:0] y,
    input [2:0] history, output reg [31:0] count, output reg [2:0] decision
);
    reg [31:0] dark, dim_count, bright, glare;
    wire [35:0] n = {4'd0,count};
    wire [35:0] d = {4'd0,dark}, m = {4'd0,dim_count};
    wire [35:0] b = {4'd0,bright}, g = {4'd0,glare};
    wire [35:0] tails=d+g;
    function majority;
        input [35:0] votes, population;
        input retaining;
        begin
            majority = (votes*36'd5 >= population*36'd3) ||
                       (retaining && (votes<<1) >= population);
        end
    endfunction
    always @* begin
        if (n==0) decision=history;
        else if (b*36'd3 >= (n<<1) && (g<<1)>=n) decision=4;
        else if (m*36'd3 >= (n<<1) && (d<<1)>=n) decision=0;
        else if ((d*36'd10>=n && g*36'd10>=n && (tails<<4)>=n*36'd7) ||
                 (history==5 && d*36'd12>=n && g*36'd12>=n && (tails<<3)>=n*36'd3)) decision=5;
        else if (majority(d,n,history==0)) decision=0;
        else if (majority(g,n,history==4)) decision=4;
        else if (majority(m,n,history==1) || ((d<<2)>=n && (g<<4)<n)) decision=1;
        else if ((m<<3)<n && (majority(b,n,history==3) || (g<<2)>=n)) decision=3;
        else decision=2;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin count<=0; dark<=0; dim_count<=0; bright<=0; glare<=0; end
        else if (clear) begin
            count<={31'd0,take};
            dark<={31'd0,take && y<=80}; dim_count<={31'd0,take && y<=104};
            bright<={31'd0,take && y>=152}; glare<={31'd0,take && y>=192};
        end else if (take) begin
            count<=count+32'd1;
            if(y<=80) dark<=dark+32'd1;
            if(y<=104) dim_count<=dim_count+32'd1;
            if(y>=152) bright<=bright+32'd1;
            if(y>=192) glare<=glare+32'd1;
        end
    end
endmodule
