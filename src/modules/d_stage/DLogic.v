module DLogic(
    input [31:0] D_inst,
    input [31:0] X_inst,
    input [31:0] Mem_WB_inst,
    
    output reg [2:0] ImmSel,  // Immediate selection
    output reg icache_re,
    output reg [4:0] rs1_addr,
    output reg [4:0] rs2_addr,
    output reg [31:0] next_inst
    
);

wire [6:0] opcode;
wire [2:0] funct3;

assign opcode = D_inst[6:0];
assign funct3 = D_inst[14:12];

always @(*) begin
    // Default values

    // Handle No Op Case after JAL
    if (X_inst[6:0] == `OPC_JAL || X_inst[6:0] == `OPC_JALR) begin
        ImmSel = 3'd0;
        icache_re = 0;
        rs1_addr = {5{1'b0}}; // no-op for rs1 rs1= x0
        rs2_addr = {5{1'b0}};
        next_inst = {32{1'b0}};
    end else if (Mem_WB_inst[6:0] == `OPC_JAL || Mem_WB_inst[6:0] == `OPC_JALR) begin
        ImmSel = 3'd0;
        icache_re = 1;
        rs1_addr = {5{1'b0}}; // no-op for rs1 rs1= x0
        rs2_addr = {5{1'b0}};
        next_inst = {32{1'b0}};
    end else begin
        // Decode immediate selection and CSR logic
        case (opcode)
            `OPC_STORE: begin
                ImmSel = 3'd0; // Store-type immediate
                icache_re = 1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end
            `OPC_LOAD: begin
                ImmSel = 3'd4;
                icache_re =  1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end 
            `OPC_BRANCH: begin
                ImmSel = 3'd1; // Branch immediate
                icache_re = 1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end
            `OPC_AUIPC, `OPC_LUI: begin
                ImmSel = 3'd2; // Upper immediate
                icache_re = 1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end
            `OPC_JAL: begin
                ImmSel = 3'd3; // Jump immediate
                icache_re = 0;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end
            `OPC_ARI_ITYPE, `OPC_JALR: begin
                if (funct3 == 3'b001 || funct3 == 3'b101) begin
                    ImmSel = 3'd5; // Shift immediate
                end else begin
                    ImmSel = 3'd4; // Default I-type immediate
                end
                icache_re = 1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end
            `OPC_CSR: begin
                ImmSel = 3'd6; // CSR immediate
                icache_re = 1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;
            end
            default: begin
                ImmSel = 3'd0; // Default case
                icache_re = 1;
                rs1_addr = D_inst[19:15];
                rs2_addr = D_inst[24:20];
                next_inst = D_inst;

            end
        endcase
    end 

    // Check if previous inst was jal then we need to no op
    
end

endmodule
