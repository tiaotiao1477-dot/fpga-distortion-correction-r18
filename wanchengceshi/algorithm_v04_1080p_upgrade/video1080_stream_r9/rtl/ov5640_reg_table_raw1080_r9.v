`timescale 1ns/1ps
// Candidate only. Requires actual 24MHz XCLK/PCLK/RAW phase readback.
// Common initialization is reused unchanged; streaming is the final write.
module ov5640_reg_table_raw1080_candidate #(parameter integer CFA_PHASE=0)(
    input wire [8:0] index,output reg [23:0] reg_data,output reg reg_valid);
    wire [23:0] baseline_data;wire baseline_valid;
    ov5640_reg_table baseline(.index(index),.reg_data(baseline_data),.reg_valid(baseline_valid));
    always @* begin
        reg_data=index<308?baseline_data:24'd0;reg_valid=index<308?baseline_valid:1'b0;
        case(index)
            // Explicit ISP state required by the existing strict readback.
            9'd307:begin reg_data=24'h500183;reg_valid=1'b1;end
            9'd308:begin reg_data=24'h303418;reg_valid=1'b1;end
            9'd309:begin reg_data=24'h303510;reg_valid=1'b1;end
            9'd310:begin reg_data=24'h303654;reg_valid=1'b1;end
            9'd311:begin reg_data=24'h303713;reg_valid=1'b1;end
            9'd312:begin reg_data=24'h310801;reg_valid=1'b1;end
            9'd313:begin reg_data=24'h381411;reg_valid=1'b1;end
            9'd314:begin reg_data=24'h381511;reg_valid=1'b1;end
            9'd315:begin reg_data=24'h382046;reg_valid=1'b1;end
            9'd316:begin reg_data=24'h382100;reg_valid=1'b1;end
            9'd317:begin reg_data=24'h361804;reg_valid=1'b1;end
            9'd318:begin reg_data=24'h361229;reg_valid=1'b1;end
            9'd319:begin reg_data=24'h370821;reg_valid=1'b1;end
            9'd320:begin reg_data=24'h370912;reg_valid=1'b1;end
            9'd321:begin reg_data=24'h370C00;reg_valid=1'b1;end
            9'd322:begin reg_data=24'h30021C;reg_valid=1'b1;end
            9'd323:begin reg_data=24'h3006C3;reg_valid=1'b1;end
            9'd324:begin reg_data=24'h382401;reg_valid=1'b1;end
            9'd325:begin reg_data=24'h501F03;reg_valid=1'b1;end
            9'd326:begin reg_data={16'h4300,6'd0,CFA_PHASE[1:0]};reg_valid=1'b1;end
            9'd327:begin reg_data=24'h474022;reg_valid=1'b1;end
            9'd328:begin reg_data=24'h503D00;reg_valid=1'b1;end
            9'd329:begin reg_data=24'h350303;reg_valid=1'b1;end
            9'd330:begin reg_data=24'h350000;reg_valid=1'b1;end
            9'd331:begin reg_data=24'h350140;reg_valid=1'b1;end
            9'd332:begin reg_data=24'h350200;reg_valid=1'b1;end
            9'd333:begin reg_data=24'h350A00;reg_valid=1'b1;end
            9'd334:begin reg_data=24'h350B40;reg_valid=1'b1;end
            9'd335:begin reg_data=24'h380001;reg_valid=1'b1;end
            9'd336:begin reg_data=24'h380150;reg_valid=1'b1;end
            9'd337:begin reg_data=24'h380201;reg_valid=1'b1;end
            9'd338:begin reg_data=24'h3803B2;reg_valid=1'b1;end
            9'd339:begin reg_data=24'h380408;reg_valid=1'b1;end
            9'd340:begin reg_data=24'h3805EF;reg_valid=1'b1;end
            9'd341:begin reg_data=24'h380605;reg_valid=1'b1;end
            9'd342:begin reg_data=24'h3807F1;reg_valid=1'b1;end
            9'd343:begin reg_data=24'h380807;reg_valid=1'b1;end
            9'd344:begin reg_data=24'h380980;reg_valid=1'b1;end
            9'd345:begin reg_data=24'h380A04;reg_valid=1'b1;end
            9'd346:begin reg_data=24'h380B38;reg_valid=1'b1;end
            9'd347:begin reg_data=24'h380C09;reg_valid=1'b1;end
            9'd348:begin reg_data=24'h380DC4;reg_valid=1'b1;end
            9'd349:begin reg_data=24'h380E04;reg_valid=1'b1;end
            9'd350:begin reg_data=24'h380F60;reg_valid=1'b1;end
            9'd351:begin reg_data=24'h381000;reg_valid=1'b1;end
            9'd352:begin reg_data=24'h381110;reg_valid=1'b1;end
            9'd353:begin reg_data=24'h381200;reg_valid=1'b1;end
            9'd354:begin reg_data=24'h381304;reg_valid=1'b1;end
            9'd355:begin reg_data=24'h3A0204;reg_valid=1'b1;end
            9'd356:begin reg_data=24'h3A035C;reg_valid=1'b1;end
            9'd357:begin reg_data=24'h3A1404;reg_valid=1'b1;end
            9'd358:begin reg_data=24'h3A155C;reg_valid=1'b1;end
            9'd359:begin reg_data=24'h300802;reg_valid=1'b1;end
            default:;
        endcase
    end
endmodule
