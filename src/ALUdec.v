// Module: ALUdecoder
// Desc:   Sets the ALU operation
// Inputs: opcode: the top 6 bits of the instruction
//         funct: the funct, in the case of r-type instructions
//         add_rshift_type: selects whether an ADD vs SUB, or an SRA vs SRL
// Outputs: ALUop: Selects the ALU's operation
//

`include "Opcode.vh"
`include "ALUop.vh"

module ALUdec(
  input [6:0]       opcode,
  input [2:0]       funct,
  input             add_rshift_type,
  output reg [3:0]  ALUop
);

  // Implement your ALU decoder here, then delete this comment
  
  
  always@(*) begin
    case (opcode)
      `OPC_NOOP: begin
        ALUop = `ALU_XXX;
      end
      `OPC_LUI: begin
        ALUop = `ALU_COPY_B;
      end 
      `OPC_AUIPC: begin
        ALUop = `ALU_ADD;
      end
      `OPC_BRANCH: begin
        ALUop = `ALU_ADD;
      end
      `OPC_LOAD: begin // hey matias what the heck
        ALUop = `ALU_ADD;
      end
      `OPC_STORE: begin
        ALUop = `ALU_ADD;
      end
      `OPC_JAL: begin
        ALUop = `ALU_ADD;
      end 
      `OPC_JALR: begin
        ALUop = `ALU_ADD;
      end
      `OPC_ARI_RTYPE: begin
        if (funct == 3'b000) begin
          ALUop = (add_rshift_type == 0) ? `ALU_ADD : `ALU_SUB;
        end else if (funct == 3'b111) begin
          ALUop = `ALU_AND;
        end else if (funct == 3'b110) begin
          ALUop = `ALU_OR;
        end else if (funct == 3'b100) begin
          ALUop = `ALU_XOR;
        end else if (funct == 3'b001) begin
          ALUop = `ALU_SLL;
        end else if (funct == 3'b101) begin
          ALUop = (add_rshift_type == 0) ? `ALU_SRL : `ALU_SRA;
        end else if (funct == 3'b010) begin
          ALUop = `ALU_SLT;
        end else if (funct == 3'b011) begin
          ALUop = `ALU_SLTU;
        end 
      end 
      `OPC_ARI_ITYPE: begin
        if (funct == 3'b000) begin
          ALUop = `ALU_ADD;
        end else if (funct == 3'b111) begin
          ALUop = `ALU_AND;
        end else if (funct == 3'b110) begin
          ALUop = `ALU_OR;
        end else if (funct == 3'b100) begin
          ALUop = `ALU_XOR;
        end else if (funct == 3'b001) begin
          ALUop = `ALU_SLL;
        end else if (funct == 3'b101) begin
          ALUop = (add_rshift_type == 0) ? `ALU_SRL : `ALU_SRA;
        end else if (funct == 3'b010) begin
          ALUop = `ALU_SLT;
        end else if (funct == 3'b011) begin
          ALUop = `ALU_SLTU;
        end
      end
    endcase
  end


endmodule
