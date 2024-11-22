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

wire [6:0] opcode = icache_dout[6:0];
wire [4:0] rd;
wire [4:0] rs1;
wire [4:0] rs2;
wire [2:0] funct3;
wire [6:0] funct7;
wire [11:0] imm;

parameter R_TYPE = 7'b0110011;
parameter I_TYPE = 7'b0010011;
parameter S_TYPE = 7'b0100011;
parameter B_TYPE = 7'b1100011;
parameter J_TYPE = 7'b1101111;
parameter JR_TYPE = 7'b1100111;
parameter AUIPC_TYPE = 7'b0010111;
parameter LUI_TYPE = 7'b0110111;



  // Implement your core here, then delete this comment
  always @ (*) begin
    if (reset) begin 
      PC = `PC_RESET;

    end
    case (opcode)
      `OPC_ARI_RTYPE: begin

      end


    endcase
  end
  

  // // CSR
  reg [31:0] tohost;

  always @ (posedge clk) begin
    tohost <= csr;
  end
  

endmodule
