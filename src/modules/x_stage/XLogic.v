module XLogic (
    input reset,
    input [6:0] opcode,
    input [2:0] funct3,
    input BrEq,
    input BrLT,
    input X_inst,
    input [31:0] D_inst,
    input [31:0] Mem_WB_inst,
    input [31:0] prev_inst,


    output BrUn,
    output [1:0] ASel,
    output BSel,
    output [1:0] PCSel,
    output DMem_re

);

// Opcode values
reg [6:0] X_opcode;
reg [6:0] D_opcode;
reg [6:0] Mem_WB_opcode;

// rd values
reg [4:0] X_rd;
reg [4:0] D_rd;
reg [4:0] Mem_WB_rd;

// @Francesco: I merged this block with below logic because we want to handle it for specific opcodes

/*always @(*) begin
  // Forward from MEM stage if rd matches rs1/rs2 and reg write is enabled
    if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]) ASel = 2'b10;
    else if (D_inst[11:7] != 0 && D_inst[11:7] == X_inst[19:15]) ASel = 2'b01;
    else ASel = 2'b00;

    if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[24:20]) BSel = 2'b10;
    else if (D_inst[11:7] != 0 && D_inst[11:7] == X_inst[24:20]) BSel = 2'b01;
    else BSel = 2'b00;
end*/




always@(*) begin
  if (reset) begin
    PCSel = 1'd2;
    ASel = 0;
    BSel = 1;
    DMem_re = 1;
  end
  case (opcode)
    `OPC_ARI_RTYPE: begin

      // Check if prev rd equal to cur rs1
      if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]) begin
        ASel = 2'b10;
      end else begin
        ASel = 0;
      end
      // Check if prev rd equal to cur rs2
      if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[24:20]) begin
        BSel = 2'b10;
      end else begin
        BSel = 0;
      end
      DMem_re = 1;
    end
    `OPC_ARI_ITYPE: begin

      // Check if prev rd equal to cur rs1
      if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]) begin
        ASel = 2'b10;
      end else begin
        ASel = 0;
      end
      BSel = 1;
      DMem_re = 1;
    end
    `OPC_LOAD: begin

      // Check if prev rd equal to cur rs1
      if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]) begin
        ASel = 2'b10;
      end else begin
        ASel = 0;
      end
      BSel = 1;
      DMem_re = 1;
    end
    `OPC_STORE: begin
      // Check if prev rd equal to cur rs1
      if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]) begin
        ASel = 2'b10;
      end else begin
        ASel = 0;
      end
      BSel = 1;
      DMem_re = 0;
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
      DMem_re = 1;
    end
    `OPC_JAL: begin
      ASel = 1;
      BSel = 1;
      DMem_re = 1;
    end
    `OPC_JALR: begin
      ASel = 0;
      BSel = 1;
      DMem_re = 1;
    end
    `OPC_AUIPC: begin
      ASel = 1;
      BSel = 1;
      DMem_re = 1;
    end
    default: begin
        PCSel = 0;
        ASel = 0;
        BSel = 1;
        DMem_re = 1;
    end
  endcase

end 
    
endmodule