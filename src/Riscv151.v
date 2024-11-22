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

/* Program Counter */
reg [31:0] PC_Next;
reg [31:0] PC_Cur;

/* Various ISA Bits */
wire [6:0] opcode = icache_dout[6:0];
wire [4:0] rd;
wire [4:0] rs1;
wire [4:0] rs2;
wire [2:0] funct3;
wire [6:0] funct7;
wire [11:0] imm;

/* ISA Constants */
parameter R_TYPE = 7'b0110011;
parameter I_TYPE = 7'b0010011;
parameter S_TYPE = 7'b0100011;
parameter B_TYPE = 7'b1100011;
parameter J_TYPE = 7'b1101111;
parameter JR_TYPE = 7'b1100111;
parameter AUIPC_TYPE = 7'b0010111;
parameter LUI_TYPE = 7'b0110111;

/* Control Logic Output Signals */
wire PCSel;
wire RegWEn;
wire [4:0] ImmSel;
wire BrUn;
wire BSel;
wire [1:0] ASel;
wire ALUSel;
wire MemRW;
wire WBSel;

/* RegFile Input Signals */
wire [4:0] RegWriteIndex;
// wire [4:0] ReadIndex1;
// wire [4:0] ReadIndex2;
wire [31:0] RegWriteData;

/* RegFile Output Signals */
wire [31:0] rs1_data;
wire [31:0] rs2_data;


/* ALU Signals */
wire [31:0] ALUOut;



//**********************************************************************//
//----------------------BEGINING INSTRUCTION FETCH STAGE----------------//
//**********************************************************************//

wire [31:0] PC_mux_out;

/* Program Counter Adder */
PC pc0 (
  .PC_Cur(PC_Cur),
  .PC_Next(PC_Next)
);

/* PC Sel Mux */
mux_2_to_1 pcMux (
  .in_1(PC_Next),
  .in_2(ALUOut),
  .sel(PCSel),
  .out(PC_mux_out)
);

/* Register for PC value */
PARAM_REGISTER pc_reg (
  .clk(clk),
  .d(PC_mux_out),
  .q(PC_Cur)
);

assign icache_addr = PC_Cur;




/////////////////////////////////////////////////////////////////////////
////////////////////////END INSTRUCTION FETCH STAGE/////////////////////
///////////////////////////////////////////////////////////////////////


//--------------- REGISTER     IF -> ID     -----------------//

wire [31:0] PC_Decode_Stage;
wire [31:0] imm;

PARAM_REGISTER PC_I_to_D (
  .clk(clk),
  .d(PC_Cur),
  .q(PC_Decode_Stage)
);



//**********************************************************************//
//----------------------BEGINING       DECODE      STAGE----------------//
//**********************************************************************//



/* RegFile Instatiation */
regFile RegFile(
  .clk(clk),
  .reset(reset),

  //write enable
  .we(RegWEn),

  //write back data
  .wb_addr(icache_dout[11:7]),
  .wb_data(RegWriteData),

  // read adress input
  .rs1_addr(icache_dout[19:15]),
  .rs2_addr(icache_dout[24:20]),
  
  // read data output
  .rs1_data(rs1_data), // OUTPUT DAT rs1
  .rs2_data(rs2_data) // output data rs2

);

wire [31:0] imm;

immGen imm_gen(

  .inst(icache_dout),
  .imm_sel(ImmSel),
  .imm(imm)
);

///////////////////////////////////////////////////////////////////////
/////////////////// END DECODE STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////

//--------- Wires output for register DECODE -> (ALU) stage---------//

wire [31:0] rs1_data_ALU;
wire [31:0] rs2_data_ALU;
wire [31:0] PC_ALU;
wire [31:0] imm_ALU;
wire [31:0] instr_ALU;






///---------Registers from DECODE -> (ALU) stage---------///
/*pipelined register after regfile*/
PARAM_REGISTER#(WIDTH=32) reg_read_data_1 (
  .clk(clk),
  .reset(reset),
  .in(rs1_data),
  .out(rs1_data_ALU)
);

PARAM_REGISTER reg_read_data21 (WIDTH=32) (
  .clk(clk),
  .reset(reset),
  .in(rs2_data),
  .out(rs2_data_ALU)
);

PARAM_REGISTER pc_ID_to_ALU (WIDTH=32) (
  .clk(clk),
  .reset(reset),
  .in(PC_Decode_Stage),
  .out(PC_ALU)
);

PARAM_REGISTER imm_ID_to_ALU (WIDTH=32) (
  .clk(clk),
  .reset(reset),
  .in(imm),
  .out(imm_ALU)
);

PARAM_REGISTER instr_ID_to_ALU (WIDTH=32) (
  .clk(clk),
  .reset(reset),
  .in(icache_dout),
  .out(instr_ALU)
);


//**********************************************************************//
//----------------------BEGINING       ALU      STAGE   ----------------//
//**********************************************************************//

// #TODO  instr_ALU should go to the control logic



/* BranchComp Output Signals for CONTROL LOGIC */
wire Brun // input from control logic to branch comp
wire BrEq; // output from branch comp to control logic
wire BrLT; // output from branch comp to control logic

// ALU 4-1 control logic
wire [1:0] A_sel; // output from control logic to ALU
wire B_sel; // output from control logic to ALU
wire [31:0] A_mux_out;
wire [31:0] B_mux_out;


// ALU Control Logic
wire [3:0] ALU_sel; // output from control logic to ALU;
wire [31:0] ALUOut; // output from ALU to control logic



/* Branch Comparator Instantiation */
BranchComp branchcomp0 (
  .RegData1(rs1_data_ALU),
  .RegData2(rs2_data_ALU),
  .BrUn(Brun),
  .BrEq(BrEq),
  .BrLT(BrLT)
);


/* A_sel Sel Mux with forwarding */
mux_4_to_1 A_mux (
  .in_1(PC_ALU), 
  .in_2(rs1_data_ALU),
  .in_3(PC+4 from forwarding)          //  <--- #TODO
  .in_4( REG WRITE DATA from forwarding)  //  <---    #TODO
  .sel(ASel),
  .out(A_mux_out)
);

mux_2_to_1 B_mux (
  .in_1(rs2_data_ALU),
  .in_2(imm_ALU),
  .sel(BSel),
  .out(B_mux_out)
);

/* ALU Instatiation */
ALU alu0 (
  .A(A_mux_out),
  .B(B_mux_out),
  .ALU_sel(ALU_sel),
  .ALUOut(ALUOut)
);


///////////////////////////////////////////////////////////////////////
/////////////////// END ALU STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////

wire MemRW; // from control logic to memory stage
wire [31:0] PC_MEM_WB;
// wire [31:0] ALUOut_MEM_WB;
// wire [31:0] rs2_data_MEM_WB; // rs2 is the only thing that we can write back to the memory stage, rs1 is generally the offset
wire [31:0] inst_MEM_WB; 





///---------Registers from ALU to (MEM + WB) stage---------///
PARAM_REGISTER pc_ALU_to_MEM_WB (WIDTH=32) (
  .clk(clk),
  .reset(reset),
  .in(PC_ALU),
  .out(PC_MEM_WB)
);

// PARAM_REGISTER ALU_to_MEM_WB (WIDTH=32) (
//   .clk(clk),
//   .reset(reset),
//   .in(ALUOut),
//   .out(ALUOut_MEM_WB)
// );

// PARAM_REGISTER rs2_data_ALU_to_MEM_WB (WIDTH=32) ( // mem write data
//   .clk(clk),
//   .reset(reset),
//   .in(rs2_data_ALU),
//   .out(rs2_data_MEM_WB)
// );

PARAM_REGISTER inst_ALU_to_MEM_WB (WIDTH=32) (
  .clk(clk),
  .reset(reset),
  .in(instr_ALU),
  .out(inst_MEM_WB)
);



//**********************************************************************//
//----------------------BEGINING      ( MEM + WB)      STAGE----------------//
//**********************************************************************//

// AS of now the memory is the dcache, later we will implement our chache

    // output [31:0] dcache_addr,  
    // output [31:0] icache_addr,
    // output [3:0] dcache_we,
    // output dcache_re,
    // output icache_re, //
    // output [31:0] dcache_din,
    // input [31:0] dcache_dout, 
  
  assign dcache_addr = ALUOut_MEM_WB;
  assign dcache_we = MemRW;
  








///////////////////////////////////////////////////////////////////////
/////////////////// END ( MEM + WB) STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////


///---------Registers from (MEM + WB) to FETCH stage---------///



  always @ (*) begin
    if (reset) begin 
      PC = `PC_RESET;

    end
    case (opcode)
      `OPC_ARI_RTYPE: begin
        RegWriteIndex = icache_dout[11:7];
        ReadIndex1 = icache_dout[19:15];
        ReadIndex2 = icache_dout[24:20];
        
      end

    if ()
    endcase
  end
  

  // // CSR
  reg [31:0] tohost;

  always @ (posedge clk) begin
    tohost <= csr;
  end
  

endmodule
