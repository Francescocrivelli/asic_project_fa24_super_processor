module MemWBLogic (
    input [6:0] opcode,

    output reg  [1:0] WBSel,
    output reg RegWEn
);

always@(*) begin
    case (opcode)
        `OPC_ARI_RTYPE, `OPC_ARI_ITYPE, `OPC_AUIPC: begin
            WBSel = 2'b01;
            RegWEn = 1;
        end
        `OPC_STORE: begin
            WBSel = 2'b00;
            RegWEn = 1;
        end
        `OPC_BRANCH: begin
            WBSel = 2'b00;
            RegWEn = 0;
        end
        `OPC_JAL, `OPC_JALR: begin
            WBSel = 2'b10;
            RegWEn = 1;
        end
        default: begin
            WBSel = 2'b00;
            RegWEn = 0;
        end
    endcase
end

endmodule