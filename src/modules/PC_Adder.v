// Module: PC.v
// Desc:   Module to increase program counter by 4 every clock cycle
// Inputs: 
//    clk: clock signal
//    PC_Cur: Current program counter
//    BrUn: 1 bit value to specify whether data is unsigned or signed
//
// 						
// Outputs:
//    BrEq: 1 bit value signalling whether RegData's are equal.
//    BrLT: 1 bit value signalling whether RegData1 is less than RegData2
//


module PCAdder(
    input [31:0] PC_Cur,
    output [31:0] PC_Next
);

// Add 4 to the current PC value
assign PC_Next = PC_Cur + 4;

endmodule