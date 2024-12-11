module XLogic (
    input reset,
    input [6:0] opcode,
    input [2:0] funct3,
    input BrEq,
    input BrLT,
    input [31:0] X_inst,
    input [31:0] D_inst,
    input [31:0] Mem_WB_inst,
    input [31:0] RegReadData1, // rs1 value for csrw instruction
    input [31:0] imm,
    input [31:0] prev_prev_inst,
    input branch_prev,
	input [1:0] addr_offset,
	input [31:0] rs2_data,

    //output reg [3:0] MemRW, 
    output reg BrUn,
    output reg [2:0] ASel,
    output reg [1:0] BSel,
    output reg PCSel,
    output reg DMem_re,
    output reg [3:0] MemRW,
    output reg [1:0] branchASel,
    output reg [1:0] branchBSel,
    output reg flush,
    output reg branch_taken,
	output reg [31:0] store_data

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
    if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]) ASel = 3'b010;
    else if (D_inst[11:7] != 0 && D_inst[11:7] == X_inst[19:15]) ASel = 2'b01;
    else ASel = 2'b00;

    if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[24:20]) BSel = 2'b10;
    else if (D_inst[11:7] != 0 && D_inst[11:7] == X_inst[24:20]) BSel = 2'b01;
    else BSel = 2'b00;
end*/



always@(*) begin
  if (reset) begin
    PCSel = 1'b0;
    ASel = 3'b000;
    BSel = 1;
    DMem_re = 1;
    BrUn = 0;
    MemRW = 0;
    branchASel = 0;
    branchBSel = 0;
    flush = 0;
    branch_taken = 0;
  end else begin
	store_data = rs2_data;
	MemRW = 4'b0000;
    case (opcode)
      `OPC_ARI_RTYPE: begin
        // Check if prev rd equal to cur rs1
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15] 
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b010;
        // Check if second prev instruction rd equals rs1
        end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[19:15]
        && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b100;
        end else begin
          ASel = 3'b000;
        end
        // Check if prev rd equal to cur rs2
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[24:20]
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
          BSel = 2'b10;
        // Check if second prev instruction rd equals cur rs2
        end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[24:20]
        && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          BSel = 2'b11;
        end else begin
          BSel = 2'b00;
        end
        DMem_re = 0;
        branch_taken = 0;
        PCSel = 0;
      end
      `OPC_ARI_ITYPE: begin
        // Check if prev rd equal to cur rs1
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15] 
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst [6:0] != `OPC_STORE) begin
          ASel = 3'b010;
        // Check if second prev instruction rd equals rs1
        end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[19:15]
         && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b100;
        end else begin
          ASel = 3'b000;
        end
        BSel = 1;
        DMem_re = 0;
        branch_taken = 0;
        PCSel = 0;
      end
      `OPC_LOAD: begin
        // Check if prev rd equal to cur rs1
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b010;
        // Check if second prev instruction rd equals rs1
        end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[19:15]
         && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b100;
        end else begin
          ASel = 3'b000;
        end
        BSel = 1;
        DMem_re = 1;
        branch_taken = 0;
        PCSel = 0;
        MemRW = 4'b0000;
      end
      `OPC_STORE: begin
        // Check if prev rd equal to cur rs1
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
        	ASel = 3'b010;
		// check is prev prev rd equals to cur rs1
		end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[19:15]
        && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
        	ASel = 3'b100;
        end else begin
          ASel = 3'b000;
        end
		case (funct3)
        	`FNC_SB: begin
				// SINGLE Byte
				case (addr_offset)
					2'b00: begin
						MemRW = 4'b0001;
						store_data = rs2_data[7:0];
					end
					2'b01: begin
						MemRW = 4'b0010;
						store_data = {{16{1'b0}}, rs2_data[7:0], {8{1'b0}}};
					end
					2'b10: begin
						MemRW = 4'b0100;
						store_data = {{8{1'b0}}, rs2_data[7:0], {16{1'b0}}};
					end
					2'b11: begin
						MemRW = 4'b1000;
						store_data = {rs2_data[7:0], {24{1'b0}}};
					end
				endcase
          end
    		`FNC_SH: begin
              case (addr_offset[1])
                  1'b0: begin
                      MemRW = 4'b0011;
					  store_data = rs2_data[15:0];
                  end
                  1'b1: begin
                      MemRW = 4'b1100;
					  store_data = {rs2_data[15:0], {16{1'b0}}};
                  end
              endcase                
          end
          `FNC_SW: begin
              //HALF WORD
              MemRW = 4'b1111;
			  store_data = rs2_data;
          end
        endcase
		BSel = 1'b01;
        DMem_re = 0;
        PCSel = 0;
        branch_taken = 0;
      end
      `OPC_BRANCH: begin
        // check if prev rd equals ot cur rs1 
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
          branchASel = 2'b01;
        // Check if second prev instruction rd equals rs1
        end else if (prev_prev_inst != 0 && prev_prev_inst[11:7] == X_inst[19:15]
        && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          branchASel = 2'b10;
        end else begin
          branchASel = 2'b00;
        end
        // Check if prev rd equal to cur rs2
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[24:20]
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
          branchBSel = 2'b01;
        // Check if second prev instruction rd equals rs2
        end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[24:20]
        && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          branchBSel = 2'b10;
        end else begin
          branchBSel = 2'b00;
        end
        case (funct3)
          // beq case
          `FNC_BEQ: begin
            BrUn = 0;
            if (BrEq) begin
              PCSel = 1'b1;
              branch_taken = 1;
            end else begin
              PCSel = 1'b0;
              branch_taken = 0;
            end
          end
          // bge case
          `FNC_BGE: begin
            BrUn = 0;
            if (!BrLT) begin
              PCSel = 1'b1;
              branch_taken = 1;
            end else begin
              PCSel = 1'b0;
              branch_taken = 0;
            end
          end
          // bgeu case
          `FNC_BGEU: begin
            BrUn = 1;
            if (!BrLT) begin
              PCSel = 1'b1;
              branch_taken = 1;
            end else begin
              PCSel = 1'b0;
              branch_taken = 0;
            end
          end
          // blt case
          `FNC_BLT: begin
            BrUn = 0;
            if (BrLT) begin
              PCSel = 1'b1;
              branch_taken = 1;
            end else begin
              PCSel = 1'b0;
              branch_taken = 0;
            end
          end
          // bltu case
          `FNC_BLTU: begin
            BrUn = 1;
            if (BrLT) begin
              PCSel = 1'b1;
              branch_taken = 1;
            end else begin
              PCSel = 1'b0;
              branch_taken = 0;
            end
          end
          // bne case
          `FNC_BNE: begin
            BrUn = 0;
            if (!BrEq) begin
              PCSel = 1'b1;
              branch_taken = 1;
            end else begin
              PCSel = 1'b0;
              branch_taken = 0;
            end
          end
          default: begin
            //PCSel = 1'b0;
            BrUn = 0;
            //branch_taken = 0;
          end
        endcase
        DMem_re = 0;
        ASel = 3'b001;
        BSel = 1;
      end
      `OPC_JAL: begin
        PCSel = 1'b1;
        ASel = 3'b001;
        BSel = 1;
        DMem_re = 0;
        branch_taken = 0;
      end
      `OPC_JALR: begin
        // check if previous rd equals to cur rs1
        if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]
        && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b010;
        // check if prev prev rd equals cur rs1
        end else if (prev_prev_inst != 0 && prev_prev_inst[11:7] == X_inst[19:15]
        && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
          ASel = 3'b100;
        end else begin
          ASel = 3'b000;
        end 
        BSel = 1;
        DMem_re = 0;
        branch_taken = 0;
        PCSel = 1'b1;
      end
      `OPC_AUIPC: begin
        ASel = 3'b001;
        BSel = 1;
        DMem_re = 0;
        branch_taken = 0;
        PCSel = 1'b0;
      end
      `OPC_LUI: begin
        ASel = 3'b000;
        BSel = 1;
        DMem_re = 0;
        branch_taken = 0;
        PCSel = 0;
      end 
      `OPC_CSR: begin
        // Check if prev rd equal to cur rs1
        if (funct3 == `FNC_RW) begin
          if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[19:15]
          && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
            ASel = 3'b010;
          end else begin
            ASel = 3'b000;
          end
        end else begin
          ASel = 3'b000;
        end
        PCSel = 1'b0;
        BSel = 1;
        DMem_re = 0;
        branch_taken = 0;
      end
      default: begin
          PCSel = 1'b0;
          ASel = 3'b000;
          BSel = 1;
          DMem_re = 0;
          //branch_taken = 0;
      end
    endcase
    if (branch_taken || branch_prev) begin
      flush = 1;
    end else begin
      flush = 0;
    end

  end
end 
 
endmodule


