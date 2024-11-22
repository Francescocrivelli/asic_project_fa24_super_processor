//=========================================================================
// REG FILE
// important to know: 
//          -   register (and so the regFile) is synchronous write and asynchornous read
//-------------------------------------------------------------------------
 
 module regFile () (
    input clk,
    input reset,

    //write enable
    input we,

    //write
    input[4:0] wb_addr,
    input[31:0] wb_data,

    // read adress input
    input[4:0] rs1_addr,
    input[4:0] rs2_addr,

    // read the data of rs1
    output[31:0] rs1_data,
    output[31:0] rs2_data,
 );

 reg [31:0] regs [31:0];
 assign rs1_data = regs[rs1_addr];
 assign rs2_data = regs[rs2_addr];

 always @(posedge clk) begin
    if (reset) begin
        we <=0;
        wb_addr <= 0;
        wb_addr <= 0;
    end
    if (we) begin
        regs[wb_addr] <= wb_data;
    end else
 end




 endmodule