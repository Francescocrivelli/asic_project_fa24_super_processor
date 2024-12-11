`include "const.vh"
`include "Opcode.vh"

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
//wire BrUn;

/* RegFile Input Signals */
//wire [4:0] RegWriteIndex;
// wire [4:0] ReadIndex1;

/* RegFile Output Signals */
wire [31:0] rs1_data;
wire [31:0] rs2_data;


/* ALU Signals */
wire [31:0] ALUOut;







//--------- Wires output for register DECODE -> (ALU) stage---------//

/* Program Counter */
reg [31:0] PC_Next;
reg [31:0] PC_Cur;

wire [31:0] PC_mux_out;

///---------Registers from MEM + WB -> IF stage---------///



/* Register for PC value */
PARAM_REGISTER_PC#(32) pc_reg (
  .reset(reset),
  .clk(clk),
  .in(PC_mux_out),
  .out(PC_Cur)
);


//**********************************************************************//
//----------------------BEGINING INSTRUCTION FETCH STAGE----------------//
//**********************************************************************//


/* Program Counter Adder */
PCAdder pc0 (
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



// comment for @matias below
assign icache_addr = PC_Cur;  



/////////////////////////////////////////////////////////////////////////
////////////////////////END INSTRUCTION FETCH STAGE/////////////////////
///////////////////////////////////////////////////////////////////////


//--------------- REGISTER     IF -> ID     -----------------//

wire [31:0] PC_Decode_Stage;
wire [31:0] imm;

PARAM_REGISTER#(32) PC_I_to_D (
  .clk(clk),
  .reset(reset),
  .in(PC_Cur),
  .out(PC_Decode_Stage)
);




//**********************************************************************//
//----------------------BEGINING       DECODE      STAGE----------------//
//**********************************************************************//

wire [31:0] instr_ALU;

wire [2:0] ImmSel;

// Mem_wb signal for regfile write
wire [31:0] inst_MEM_WB; 

wire flush; // signal to flush in case of branch or jump

wire [31:0] D_inst;

wire [31:0] reg_write_data; // data to be written to reg file
wire [4:0] reg_write_addr; // address for write back
wire [4:0] rs1_addr;
wire [4:0] rs2_addr;

wire [31:0] next_inst;

assign reg_write_addr = inst_MEM_WB[11:7];

/* Flush Block */
FlushLogic flush_inst (
  .icache_dout(icache_dout),
  .flush(flush),
  .D_inst(D_inst)
);

/* RegFile Instatiation */
regFile RegFile(
  .clk(clk),
  .reset(reset),

  //write enable
  .we(RegWEn),

  //write back data
  .wb_addr(reg_write_addr), // comes from MEM/WB stage
  .wb_data(reg_write_data),    // ^ same

  // read adress input
  .rs1_addr(D_inst[19:15]),
  .rs2_addr(D_inst[24:20]),
  
  // read data output
  .rs1_data(rs1_data), // OUTPUT DAT rs1
  .rs2_data(rs2_data) // output data rs2

);


immGen imm_gen(
  .inst(D_inst),
  .imm_sel(ImmSel),
  .imm(imm)
);

DLogic d_control(
  .D_inst(D_inst),
  .X_inst(instr_ALU),
  .Mem_WB_inst(inst_MEM_WB),
  .ImmSel(ImmSel),
  .icache_re(icache_re),
  .rs1_addr(rs1_addr),
  .rs2_addr(rs2_addr),
  .next_inst(next_inst)
);

///////////////////////////////////////////////////////////////////////
/////////////////// END DECODE STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////

//--------- Wires output for register DECODE -> (ALU) stage---------//

wire [31:0] rs1_data_ALU;
wire [31:0] rs2_data_ALU;
wire [31:0] PC_ALU;
wire [31:0] imm_ALU;






///---------Registers from DECODE -> (ALU) stage---------///
/*pipelined register after regfile*/
PARAM_REGISTER#(32) reg_read_data_1 (
  .clk(clk),
  .reset(reset),
  .in(rs1_data),
  .out(rs1_data_ALU)
);

PARAM_REGISTER#(32) reg_read_data_2 (
  .clk(clk),
  .reset(reset),
  .in(rs2_data),
  .out(rs2_data_ALU)
);

PARAM_REGISTER#(32) pc_ID_to_ALU  (
  .clk(clk),
  .reset(reset),
  .in(PC_Decode_Stage),
  .out(PC_ALU)
);

PARAM_REGISTER#(32) imm_ID_to_ALU (
  .clk(clk),
  .reset(reset),
  .in(imm),
  .out(imm_ALU)
);

PARAM_REGISTER#(32) instr_ID_to_ALU (
  .clk(clk),
  .reset(reset),
  .in(next_inst),
  .out(instr_ALU)
);


//**********************************************************************//
//----------------------BEGINING       ALU      STAGE   ----------------//
//**********************************************************************//

// #TODO  instr_ALU should go to the control logic


/* PC+4 for mux */
wire [31:0] X_PC_Next;

/* BranchComp Output Signals for CONTROL LOGIC */
wire BrUn; // input from control logic to branch comp
wire BrEq; // output from branch comp to control logic
wire BrLT; // output from branch comp to control logic

wire branch_prev; // signal for second flush after branch/jump
wire branch_taken; // signal gets set to high if we branch

// ALU 4-1 control logic
wire [2:0] ASel; // output from control logic to ALU
wire [1:0] BSel;  // output from control logic to ALU
wire [31:0] A_mux_out; 
wire [31:0] B_mux_out;


// ALU Control Logic
wire [3:0] ALUop; // output from control logic to ALU;
wire [31:0] ALUOut; // output from ALU to control logic

// DMem Signals
wire DMem_re;

assign dcache_re = DMem_re;

wire [31:0] prev_write_data;
wire [31:0] prev_Mem_WB_inst;

// wires connecting output of branch muxes to branch comp module
wire [31:0] branch_data_A;
wire [31:0] branch_data_B;

// selector wires
wire [1:0] branchASel;
wire [1:0] branchBSel;

/* Branch Comp A Mux */
mux_3_to_1 branchMuxA (
  .in_1(rs1_data_ALU),
  .in_2(reg_write_data),
  .in_3(prev_write_data),
  .sel(branchASel),
  .out(branch_data_A)
);

/* Branch Comp B Mux */
mux_3_to_1 branchMuxB (
  .in_1(rs2_data_ALU),
  .in_2(reg_write_data),
  .in_3(prev_write_data),
  .sel(branchBSel),
  .out(branch_data_B)
);

/* Branch Comparator Instantiation */
BranchComp branchcomp0 (
  .RegData1(branch_data_A),
  .RegData2(branch_data_B),
  .BrUn(BrUn),
  .BrEq(BrEq),
  .BrLT(BrLT)
);


PCAdder X_PCAdder (
  .PC_Cur(PC_ALU),
  .PC_Next(X_PC_Next)
);

/* A_sel Sel Mux with forwarding */
mux_5_to_1 A_mux (
  .in_1(rs1_data_ALU), 
  .in_2(PC_ALU),
  .in_3(reg_write_data),          //  <--- #TODO --> Done
  .in_4(X_PC_Next),  //  <---    #TODO --->Done
  .in_5(prev_write_data),
  .sel(ASel),
  .out(A_mux_out)
);


mux_4_to_1 B_mux (
  .in_1(rs2_data_ALU),
  .in_2(imm_ALU),
  .in_3(reg_write_data),
  .in_4(prev_write_data),
  .sel(BSel),
  .out(B_mux_out)
);


ALUdec ALUDec0 (
  .opcode(instr_ALU[6:0]),
  .funct(instr_ALU[14:12]),
  .add_rshift_type(instr_ALU[30]),
  .ALUop(ALUop)
);

/* ALU Instatiation */
ALU alu0 (
  .A(A_mux_out),
  .B(B_mux_out),
  .ALUop(ALUop),
  .Out(ALUOut)
);


XLogic x_control (
  .reset(reset),
  .opcode(instr_ALU[6:0]),
  .funct3(instr_ALU[14:12]),
  .BrEq(BrEq),
  .BrLT(BrLT), 
  .BrUn(BrUn),
  .ASel(ASel),
  .BSel(BSel),
  .PCSel(PCSel),
  .RegReadData1(rs1_data_ALU),
  .DMem_re(DMem_re),
  .imm(imm_ALU),
  .X_inst(instr_ALU),
  .D_inst(D_inst), 
  .Mem_WB_inst(inst_MEM_WB), 
  .MemRW(MemRW),
  .prev_prev_inst(prev_Mem_WB_inst),
  .branchASel(branchASel),
  .branchBSel(branchBSel),
  .branch_prev(branch_prev),
  .branch_taken(branch_taken),
  .flush(flush)
);




///////////////////////////////////////////////////////////////////////
/////////////////// END ALU STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////

wire MemRW; // from control logic to memory stage
wire [31:0] PC_MEM_WB;
wire [31:0] ALUOut_MEM_WB;
// wire [31:0] rs2_data_MEM_WB; // rs2 is the only thing that we can write back to the memory stage, rs1 is generally the offset




///---------Registers from ALU to (MEM + WB) stage---------///
PARAM_REGISTER#(32) pc_ALU_to_MEM_WB (
  .clk(clk),
  .reset(reset),
  .in(PC_ALU),
  .out(PC_MEM_WB)
);


PARAM_REGISTER#(32) inst_ALU_to_MEM_WB (
  .clk(clk),
  .reset(reset),
  .in(instr_ALU),
  .out(inst_MEM_WB)
);

PARAM_REGISTER#(32) ALUOut_to_WB (
  .clk(clk),
  .reset(reset),
  .in(ALUOut),
  .out(ALUOut_MEM_WB)
);

PARAM_REGISTER#(1) branch_prev_taken (
  .clk(clk),
  .reset(reset),
  .in(branch_taken),
  .out(branch_prev)
);





//**********************************************************************//
//----------------------BEGINING      ( MEM + WB)      STAGE----------------//
//**********************************************************************//

// AS of now the memory is the dcache, later we will implement our cache

    // output [31:0] dcache_addr,  
    // output [31:0] icache_addr,
    // output [3:0] dcache_we,
    // output dcache_re,
    // output icache_re, //
    // output [31:0] dcache_din,
  


  wire [31:0] pc_MEM_WB_plus_4;


  

  //Memory
  assign dcache_addr = ALUOut;
  assign dcache_din = rs2_data_ALU;
  assign dcache_we = MemRW;

    // for lsb for the bit masking
  wire [1:0] addr_lsb;
  assign addr_lsb = ALUOut[1:0];

  //control path for the mux
  wire [1:0] WBSel;

  //PC+4 MEM_WB STAGE
  wire [31:0] PC_MEM_WB_PLUS_4 ;
  PCAdder PC_mem_adder(
    .PC_Cur(PC_MEM_WB),
    .PC_Next(PC_MEM_WB_PLUS_4)
  );


  mux_3_to_1 mux_MEM_WB(
    .in_1(dcache_dout), 
    .in_2(ALUOut_MEM_WB),
    .in_3(PC_MEM_WB_PLUS_4),
    .sel(WBSel),
    .out(reg_write_data)
  );

  MemWBLogic mem_wb_control(
    .opcode(inst_MEM_WB[6:0]),
    .funct3(inst_MEM_WB[14:12]), //@matias, it wont affect  other instructions because in the WB+MEM lOGIC the func3 is only used for store instructions
    .addr_lsb(addr_lsb),
    .WBSel(WBSel),
    .RegWEn(RegWEn),
    .write_mask(dcache_we)
  );

  CSRLogic csr_control(
    .RegWriteData(reg_write_data),
    .Opcode(inst_MEM_WB[6:0]),
    .csr_output(csr)
  );

//@francesco + @matias how to do the write back???
//Also update the instruction to RegWriteAdress -> look at CS 61C full datapath

  








///////////////////////////////////////////////////////////////////////
/////////////////// END ( MEM + WB) STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////



  
///---------Registers for saving output for forwarding---------///



PARAM_REGISTER#(32) data_out_reg (
  .clk(clk),
  .reset(reset),
  .in(reg_write_data),
  .out(prev_write_data)
);

PARAM_REGISTER#(32) inst_out_reg (
  .clk(clk),
  .reset(reset),
  .in(inst_MEM_WB),
  .out(prev_Mem_WB_inst)
);


///////////////////////////////////////////////////////////////////////
/////////////////// Memory/////////////////////////////////
/////////////////////////////////////////////////////////////////////


// Memory151 memory (
//   .clk(clk),
//   .reset(reset),
//   .dcache_addr(dcache_addr),
//   .icache_addr(icache_addr),
//   .dcache_we(dcache_we),
//   .dcache_re(dcache_re),
//   .icache_re(icache_re),
//   .dcache_din(dcache_din),
//   .dcache_dout(dcache_dout),
//   .icache_dout(icache_dout),
//   .stall(stall),

//   // Main memory 
//   .mem_req_valid(),
//   .mem_req_ready(),
//   .mem_req_rw(),
//   .mem_req_addr(),
//   .mem_req_tag(),
//   .mem_req_data_valid(),
//   .mem_req_data_ready(),
//   .mem_req_data_bits(),
//   .mem_req_data_mask(),
//   .mem_resp_valid(),
//   .mem_resp_data(),
//   .mem_resp_tag()
// );

endmodule
