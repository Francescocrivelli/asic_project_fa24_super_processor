// Module: ALU.v
// Desc:   32-bit ALU for the RISC-V Processor
// Inputs: 
//    A: 32-bit value
//    B: 32-bit value
//    ALUop: Selects the ALU's operation 
// 						
// Outputs:
//    Out: The chosen function mapped to A and B.

`include "Opcode.vh"
`include "ALUop.vh"

module ALU(
    input [31:0] A,B,
    input [3:0] ALUop,
    output reg [31:0] Out
);


reg [31:0] sol;



always@(*) begin
    case (ALUop) 
        `ALU_ADD: begin
            sol = A + B;
        end
        `ALU_SUB: begin
            sol = A - B;
        end
        `ALU_AND: begin
            sol = A & B; 
        end
        `ALU_OR: begin
            sol = A | B;
        end
        `ALU_XOR: begin
            sol = A ^ B;
        end
        `ALU_SLT: begin
            if ($signed(A) < $signed(B)) begin
                sol = 1;
            end
        end
        `ALU_SLTU: begin
            if (A < B) begin
                sol = 1;
            end
        end
        `ALU_SLL: begin
            sol = A << B[4:0];
        end
        `ALU_SRA: begin
            sol = $signed(A) >>> B[4:0];
        end
        `ALU_SRL: begin
            sol = A >> B[4:0];
        end
        `ALU_COPY_B: begin
            sol = B;
        end
        `ALU_XXX: begin
            sol = 0;
        end
        default: sol = 0;
    endcase

    Out = sol;
end


endmodule
