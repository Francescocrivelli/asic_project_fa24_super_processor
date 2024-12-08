module ForwardingLogic(
    input [31:0] X_inst;
    input [31:0] D_inst;
    input [31:0] Mem_WB_inst;


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
    // Check if previous instruction needs current register data
    // if (X stage register == Mem_wb rd)
        // (if rs1) forward path from output of ALU to input of A mux 
        // (if rs2) forward ALU out to B mux
    
    
end



endmodule