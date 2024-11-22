`include "const.vh"

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output [31:0] dcache_addr,  
    output [31:0] icache_addr,
    output [3:0] dcache_we,
    output dcache_re,
    output icache_re, //
    output [31:0] dcache_din,
    input [31:0] dcache_dout, 
    input [31:0] icache_dout,// output of i_mem
    input stall,
    output [31:0] csr

);

reg [31:0] PC;

wire [6:0] opcode = ichache_dout[6:0];
wire [4:0] rd;
wire [4:0] rs1;
wire [4:0] rs2;
wire [2:0] funct3;
wire [4:0] rs1;
wire [6:0] funct7;
wire [11:0] imm;



  // Implement your core here, then delete this comment
  if (reset == 1'b1) begin 
    PC = `PC_RESET;

  end

  // // // CSR
  // reg [31:0] tohost;

  // always @ (posedge) begin
  //   tohost <= csr;
  // end
  

endmodule
