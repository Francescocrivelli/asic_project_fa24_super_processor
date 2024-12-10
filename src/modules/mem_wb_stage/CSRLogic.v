module CSRLogic (
    input [31:0] RegWriteData,
    input [6:0] Opcode,

    output reg [31:0] csr_output
);

always@(*) begin
    if (Opcode == `OPC_CSR) begin
        csr_output = RegWriteData;
    end else begin
        csr_output = {32{1'b0}};
    end
end

endmodule