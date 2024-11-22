module immGen (
    input [31:0] inst,
    input [4:0] imm_sel,
    output reg [31:0] imm
);

// `define R     4'd0
`define S     4'd0
`define B     4'd1
`define U     4'd2
`define J     4'd3
`define I     4'd4
`define I_star     4'd5




always @ (*) begin
    case (imm_sel)
        // `R: begin // should we ignore the case R since there is NO immediate
        //     imm = 32'b0; // just setting it to 0 as a place holder
        // end
        `I: begin
            imm = {{20{inst[31]}}, inst[31:20]}
        end
        `I_star: begin
            imm = {28{1'b0}, inst[24:20]}
        end
        `S: begin
            imm = {{20{inst[31]}}, inst[31:25], inst[11:7]}
        end
        `B: begin //LSB of B instrcution is Always    -> Therefore makes it a 13 bit immediate value before sign extending it
            imm = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
        end
        `U: begin //20 bits immediate before sign extension
            imm = {inst[31:12], {12{1'b0}}}
        end
        `J: begin //20 bits immediate before sign extension
            imm = {{12{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}
        end
        endcase 
end
endmodule