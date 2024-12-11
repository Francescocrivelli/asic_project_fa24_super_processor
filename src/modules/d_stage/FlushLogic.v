module FlushLogic (
    input [31:0] icache_dout,
    input flush,

    output reg [31:0] D_inst
);


always @(*) begin
    if (flush) begin
        D_inst = `INSTR_NOP;
    end else begin
        D_inst = icache_dout;
    end 
end

endmodule