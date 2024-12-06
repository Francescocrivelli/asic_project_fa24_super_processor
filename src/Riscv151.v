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

/* Program Counter */
reg [31:0] PC_Next;
reg [31:0] PC_Cur;

/* Various ISA Bits */
wire [6:0] opcode = icache_dout[6:0];
wire [4:0] rd;
wire [4:0] rs1;
wire [4:0] rs2;
wire [2:0] funct3 = icache_dout[];
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
reg PCSel;
wire RegWEn;
wire [4:0] ImmSel;
//wire BrUn;
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

/* Register for PC value */
PARAM_REGISTER pc_reg (
  .clk(clk),
  .d(PC_mux_out),
  .q(PC_Cur)
);

// comment for @matias below
assign icache_addr = PC_Cur;   // @matias ichache adress is an output and you are setting it to something.
                               // Yeah that's what we're supposed to be doing I think




/////////////////////////////////////////////////////////////////////////
////////////////////////END INSTRUCTION FETCH STAGE/////////////////////
///////////////////////////////////////////////////////////////////////


//--------------- REGISTER     IF -> ID     -----------------//

wire [31:0] PC_Decode_Stage;
wire [31:0] imm;

PARAM_REGISTER#(WIDTH=32) PC_I_to_D (
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

PARAM_REGISTER#(WIDTH=32) reg_read_data_2 (
  .clk(clk),
  .reset(reset),
  .in(rs2_data),
  .out(rs2_data_ALU)
);

PARAM_REGISTER#(WIDTH=32) pc_ID_to_ALU  (
  .clk(clk),
  .reset(reset),
  .in(PC_Decode_Stage),
  .out(PC_ALU)
);

PARAM_REGISTER#(WIDTH=32) imm_ID_to_ALU (
  .clk(clk),
  .reset(reset),
  .in(imm),
  .out(imm_ALU)
);

PARAM_REGISTER#(WIDTH=32) instr_ID_to_ALU (
  .clk(clk),
  .reset(reset),
  .in(icache_dout),
  .out(instr_ALU)
);


//**********************************************************************//
//----------------------BEGINING       ALU      STAGE   ----------------//
//**********************************************************************//

// #TODO  instr_ALU should go to the control logic


/* PC+4 for mux */
wire [31:0] X_PC_Next;

/* BranchComp Output Signals for CONTROL LOGIC */
wire BrUn // input from control logic to branch comp
wire BrEq; // output from branch comp to control logic
wire BrLT; // output from branch comp to control logic

// ALU 4-1 control logic
wire [1:0] A_sel; // output from control logic to ALU
wire B_sel; // output from control logic to ALU
wire [31:0] A_mux_out;
wire [31:0] B_mux_out;


// ALU Control Logic
wire [3:0] ALUop; // output from control logic to ALU;
wire [31:0] ALUOut; // output from ALU to control logic



/* Branch Comparator Instantiation */
BranchComp branchcomp0 (
  .RegData1(rs1_data_ALU),
  .RegData2(rs2_data_ALU),
  .BrUn(BrUn),
  .BrEq(BrEq),
  .BrLT(BrLT)
);


PCAdder X_PCAdder (
  .PC_Cur(PC_ALU),
  .PC_Next(X_PC_Next)
);

/* A_sel Sel Mux with forwarding */
mux_4_to_1 A_mux (
  .in_1(rs1_data_ALU), 
  .in_2(PC_ALU),
  .in_3(X_PC_Next)          //  <--- #TODO
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



ALUdec ALUDec0 (
  .opcode(X_opcode),
  .funct(X_funct3),
  .add_rshift_type(instr_ALU[30]),
  .ALUop(ALUop)
);

/* ALU Instatiation */
ALU alu0 (
  .A(A_mux_out),
  .B(B_mux_out),
  .ALUop(ALUop),
  .ALUOut(ALUOut)
);


XLogic x_control (
  .opcode(instr_ALU[6:0]),
  .funct3(instr_ALU[14:12]),
  .BrEq(BrEq),
  .BrLT(BrLT), // @matias add branch unsign signal
  .BrUn(),
  .ASel(ASel),
  .BSel(BSel),
  .PCSel(PCSel)
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
PARAM_REGISTER#(WIDTH=32) pc_ALU_to_MEM_WB (
  .clk(clk),
  .reset(reset),
  .in(PC_ALU),
  .out(PC_MEM_WB)
);

PARAM_REGISTER#(WIDTH=32) ALU_to_MEM_WB ( // @francesco remove register and adapt wires to assign dcache address  = ......
  .clk(clk),
  .reset(reset),
  .in(ALUOut),
  .out(ALUOut_MEM_WB)
);
// PARAM_REGISTER#(WIDTH=32) ALU_to_MEM_WB (
//   .clk(clk),
//   .reset(reset),
//   .in(ALUOut),
//   .out(ALUOut_MEM_WB)
// );

PARAM_REGISTER#(WIDTH=32) rs2_data_ALU_to_MEM_WB ( //  @francesco  remove register and adapt wires to assign dcache data  = ......
  .clk(clk),
  .reset(reset),
  .in(rs2_data_ALU),
  .out(rs2_data_MEM_WB)
);

PARAM_REGISTER#(WIDTH=32) inst_ALU_to_MEM_WB (
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
  

  //from @francesco @matias
  //I am confused with that is dcache dout

  wire [31:0] rs2_data_MEM_WB ;

  wire [31:0] pc_MEM_WB_plus_4;
  

  //Memory
  assign dcache_addr = ALUOut_MEM_WB;
  assign dcache_din = rs2_data_ALU;
  assign dcache_we = MemRW;
  assign dcache_dout = what is the difference ?
  assign dcache_re = what is the difference ?

  //control path for the mux
  wire WB_sel;

  //PC+4 MEM_WB STAGE
  wire [31:0] PC_MEM_WB_PLUS_4 ;
  PCAdder PC_MEM_WB(
    .PC_Cur(PC_MEM_WB),
    .PC_Next(PC_MEM_WB_PLUS_4)
  )


  mux_3_to_1 mux_MEM_WB(
    .in_1(), // @francesco + @matias
    .in_2(ALUOut_MEM_WB),
    .in_3(PC_MEM_WB_PLUS_4),
    .sel(WB_sel),
    .out(out)
  );

//@francesco + @matias how to do the write back???
  assign RegWriteData =  

  








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
