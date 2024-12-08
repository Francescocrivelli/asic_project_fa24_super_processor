module DLogic(
    input [6:0] opcode,
    input [2:0] funct3,

    output reg [2:0] ImmSel
);

always@(*) begin
    case (opcode)
        `OPC_STORE: ImmSel = 3'd0;
        `OPC_BRANCH: ImmSel = 3'd1;
        `OPC_AUIPC, `OPC_LUI: ImmSel = 3'd2;
        `OPC_JAL, `OPC_JALR: ImmSel = 3'd3;
        `OPC_ARI_ITYPE: begin
            if (funct3 == 3'b001 || funct3 == 3'b101) begin
                ImmSel = 3'd5;
            end else begin
                ImmSel = 3'd4;
            end
        end
        default: ImmSel = 3'd0;
    endcase
end

endmodule