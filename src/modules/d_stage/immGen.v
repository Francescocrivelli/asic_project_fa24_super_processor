module immGen (
    input [31:0] inst,
    input [2:0] imm_sel,
    output reg [31:0] imm
);

// `define R     4'd0
`define S     3'd0
`define B     3'd1
`define U     3'd2
`define J     3'd3
`define I     3'd4
`define I_star     3'd5
`define CSR   3'd6




always @ (*) begin
    case (imm_sel)
        // `R: begin // should we ignore the case R since there is NO immediate
        //     imm = 32'b0; // just setting it to 0 as a place holder
        // end
        `I: begin
            imm = {{20{inst[31]}}, inst[31:20]};
        end
        `I_star: begin
            imm = {{27{1'b0}}, inst[24:20]};
        end
        `S: begin
            imm = {{20{inst[31]}}, inst[31:25], inst[11:7]};
        end
        `B: begin //LSB of B instrcution is Always    -> Therefore makes it a 13 bit immediate value before sign extending it
            imm = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
        end
        `U: begin //20 bits immediate before sign extension
            imm = {inst[31:12], {12{1'b0}}};
        end
        `J: begin //20 bits immediate before sign extension
            imm = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
        end
        `CSR: begin
            imm = {{27{1'b0}}, inst[19:15]};
        end
        default: imm = 32'd0;
    endcase 
end
endmodule