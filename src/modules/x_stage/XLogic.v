module XLogic (
    input [6:0] opcode,
    input [2:0] funct3,
    input BrEq,
    input BrLT,
    input X_inst;
    input [31:0] D_inst;
    input [31:0] Mem_WB_inst;
    input [31:0] prev_inst;


    output BrUn,
    output [1:0] ASel,
    output BSel,
    output PCSel

);

// Opcode values
reg [6:0] X_opcode;
reg [6:0] D_opcode;
reg [6:0] Mem_WB_opcode;

// rd values
reg [4:0] X_rd;
reg [4:0] D_rd;
reg [4:0] Mem_WB_rd;

always @(*) begin
  if ()
end




always@(*) begin
  case (opcode)
    `OPC_ARI_RTYPE: begin
      ASel = 0;
      BSel = 0;
    end
    `OPC_ARI_ITYPE: begin
      ASel = 0;
      BSel = 1;
    end
    `OPC_LOAD: begin
      ASel = 0;
      BSel = 1;
    end
    `OPC_STORE: begin
      ASel = 0;
      BSel = 1;
    end
    `OPC_BRANCH: begin
      ASel = 1;
      BSel = 1;
      case (funct3)
        // beq case
        3'b000: begin
          BrUn = 0;
          if (BrEq)
            PCSel = 1;
        end
        // bge case
        3'b101: begin
          BrUn = 0;
          if (!BrLT)
            PCSel = 1;
        end
        // bgeu case
        3'b111: begin
          BrUn = 1;
          if (!BrLT)
            PCSel = 1;
        end
        // blt case
        3'b100: begin
          BrUn = 0;
          if (BrLT)
            PCSel = 1;
        end
        // bltu case
        3'b110: begin
          BrUn = 1;
          if (BrLT)
            PCSel = 1;
        end
        // bne case
        3'b001: begin
          BrUn = 0;
          if (!BrEq)
            PCSel = 1;
        end
        default: begin
          PCSel = 0;
          BrUn = 0;
        end
      endcase
    end
    `OPC_JAL: begin
      ASel = 1;
      BSel = 1;
    end
    `OPC_JALR: begin
      ASel = 0;
      BSel = 1;
    end
    `OPC_AUIPC: begin
      ASel = 1;
      BSel = 1;
    end
    default: begin
        PCSel = 0;
        ASel = 0;
        BSel = 1;
    end
  endcase

end 
    
endmodule