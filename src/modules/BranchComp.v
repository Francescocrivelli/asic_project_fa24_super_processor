// Module: BranchComp.v
// Desc:   Module to compare whether instruction should branch or not
// Inputs: 
//    RegData1: data from some input register
//    RegData2: data from some input register
//    BrUn: 1 bit value to specify whether data is unsigned or signed
//
// 						
// Outputs:
//    BrEq: 1 bit value signalling whether RegData's are equal.
//    BrLT: 1 bit value signalling whether RegData1 is less than RegData2
//



module BranchComp(
    input [31:0] RegData1,
    input [31:0] RegData2,
    input BrUn,

    output reg BrEq,
    output reg BrLT
);

always@(*) begin
    case (BrUn)
        1'b0: begin
            BrEq = ($signed(RegData1) == $signed(RegData2)) ? 1 : 0;
            BrLT = ($signed(RegData1) < $signed(RegData2)) ? 1 : 0;
        end
        1'b1: begin
            BrEq = (RegData1 == RegData2) ? 1 : 0;
            BrLT = (RegData1 < RegData2) ? 1 : 0;
        end
    endcase 
end

endmodule