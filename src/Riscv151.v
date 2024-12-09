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
wire [31:0] RegWriteData;

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
assign icache_addr = PC_Cur;   // @matias ichache adress is an output and you are setting it to something.
                               // Yeah that's what we're supposed to be doing I think



/////////////////////////////////////////////////////////////////////////
////////////////////////END INSTRUCTION FETCH STAGE/////////////////////
///////////////////////////////////////////////////////////////////////


//--------------- REGISTER     IF -> ID     -----------------//

wire [31:0] PC_Decode_Stage;
wire [31:0] imm;

PARAM_REGISTER#(32) PC_I_to_D (
  .clk(clk),
  .reset(reset)
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

wire [31:0] reg_write_data; // data to be written to reg file
wire [4:0] rs1_addr;
wire [4:0] rs2_addr;

wire [31:0] next_inst;

/* RegFile Instatiation */
regFile RegFile(
  .clk(clk),
  .reset(reset),

  //write enable
  .we(RegWEn),

  //write back data
  .wb_addr(inst_MEM_WB[11:7]), // comes from MEM/WB stage
  .wb_data(reg_write_data),    // ^ same

  // read adress input
  .rs1_addr(icache_dout[19:15]),
  .rs2_addr(icache_dout[24:20]),
  
  // read data output
  .rs1_data(rs1_data), // OUTPUT DAT rs1
  .rs2_data(rs2_data) // output data rs2

);


immGen imm_gen(
  .inst(icache_dout),
  .imm_sel(ImmSel),
  .imm(imm)
);

DLogic d_control(
  .D_inst(icache_dout),
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

// ALU 4-1 control logic
wire [1:0] ASel; // output from control logic to ALU
wire [1:0] BSel;  // output from control logic to ALU
wire [31:0] A_mux_out;
wire [31:0] B_mux_out;


// ALU Control Logic
wire [3:0] ALUop; // output from control logic to ALU;
wire [31:0] ALUOut; // output from ALU to control logic

// DMem Signals
wire DMem_re;

assign dcache_re = DMem_re;





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
  .in_3(reg_write_data),          //  <--- #TODO --> Done
  .in_4(X_PC_Next),  //  <---    #TODO --->Done
  .sel(ASel),
  .out(A_mux_out)
);

mux_3_to_1 B_mux (
  .in_1(rs2_data_ALU),
  .in_2(imm_ALU),
  .in_3(reg_write_data),
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
  .BrLT(BrLT), // @matias add branch unsign signal
  .BrUn(BrUn),
  .ASel(ASel),
  .BSel(BSel),
  .PCSel(PCSel),
  .prev_inst(inst_MEM_WB),
  .RegReadData1(rs1_data_ALU),
  .DMem_re(DMem_re),
  .csr_output(csr),
  .imm(imm_ALU),
  .X_inst(instr_ALU),
  .D_inst(icache_dout), 
  .Mem_WB_inst(inst_MEM_WB), 
  .MemRW(MemRW) 
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
    .in_1(dcache_dout), // @francesco + @matias DONE 
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

//@francesco + @matias how to do the write back???
//Also update the instruction to RegWriteAdress -> look at CS 61C full datapath

  








///////////////////////////////////////////////////////////////////////
/////////////////// END ( MEM + WB) STAGE/////////////////////////////////
/////////////////////////////////////////////////////////////////////


///---------Registers from (MEM + WB) to FETCH stage---------///




    // case (opcode)
    //   `OPC_ARI_RTYPE: begin
    //     RegWriteIndex = icache_dout[11:7];
    //     ReadIndex1 = icache_dout[19:15];
    //     ReadIndex2 = icache_dout[24:20];
        
    //   end

    // endcase

  

  // // CSR
  reg [31:0] tohost;

  always @ (posedge clk) begin
    tohost <= csr;
  end
  
// @matias there is a problem with MemRW
endmodule
