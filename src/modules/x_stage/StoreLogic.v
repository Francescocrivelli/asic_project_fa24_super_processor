

module StoreLogic (
    input [31:0] RegReadData2,
    input [31:0] X_inst,
    input [31:0] Mem_WB_inst,
    input [31:0] prev_prev_inst,
    input [31:0] reg_write_data,
    input [31:0] prev_write_data,

    output reg [31:0] rs2_data
);

always@(*) begin
    // check if prev rd equals to cur rs2
    if (Mem_WB_inst[11:7] != 0 && Mem_WB_inst[11:7] == X_inst[24:20] 
    && Mem_WB_inst[6:0] != `OPC_BRANCH && Mem_WB_inst[6:0] != `OPC_STORE) begin
        rs2_data = reg_write_data;
    // Check if second prev instruction rd equals rs2
    end else if (prev_prev_inst[11:7] != 0 && prev_prev_inst[11:7] == X_inst[24:20]
    && prev_prev_inst[6:0] != `OPC_BRANCH && prev_prev_inst[6:0] != `OPC_STORE) begin
        rs2_data = prev_write_data;
    end else begin
        rs2_data = RegReadData2;
    end
end

endmodule