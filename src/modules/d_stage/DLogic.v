module DLogic(
    input [6:0] opcode,
    input [2:0] funct3,
    input [11:0] csr_address, // Added to decode specific CSR address
    
    output reg [2:0] ImmSel  // Immediate selection
    // output reg is_csr_rw,     // Signal for csrw instruction
    // output reg is_csr_rwi,    // Signal for csrwi instruction
    // output reg is_tohost_csr  // Signal for tohost CSR
);

always @(*) begin
    // Default values
    ImmSel = 3'd0;

    // Decode immediate selection and CSR logic
    case (opcode)
        `OPC_STORE: ImmSel = 3'd0; // Store-type immediate
        `OPC_BRANCH: ImmSel = 3'd1; // Branch immediate
        `OPC_AUIPC, `OPC_LUI: ImmSel = 3'd2; // Upper immediate
        `OPC_JAL, `OPC_JALR: ImmSel = 3'd3; // Jump immediate
        `OPC_ARI_ITYPE: begin
            if (funct3 == 3'b001 || funct3 == 3'b101) begin
                ImmSel = 3'd5; // Shift immediate
            end else begin
                ImmSel = 3'd4; // Default I-type immediate
            end
        end
        `OPC_CSR: ImmSel = 3'd6; // CSR immediate
        default: ImmSel = 3'd0; // Default case
    endcase
end

endmodule
