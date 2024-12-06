module XLogic (
    input [6:0] opcode,
    input [2:0] funct3,
    input BrEq,
    input BrLT,

    output BrUn,
    output [1:0] ASel,
    output BSel,
    output PCSel

);

always@(*) begin
  if (funct3) //
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
        default: PCSel = 0;
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