module MemWBLogic (
    input [6:0] opcode,
    input [2:0] funct3,
    input [1:0] addr_offset,
    input [31:0] MemReadData,

    
    output reg  [1:0] WBSel,
    output reg RegWEn,
    output reg [31:0] maskedReadData
);

reg [3:0] write_mask;
reg [31:0] load_result;


always@(*) begin
    write_mask = 4'b0000;
    WBSel = 2'b00;
    RegWEn = 0;
    maskedReadData = MemReadData;
    case (opcode)
        `OPC_ARI_RTYPE, `OPC_ARI_ITYPE, `OPC_AUIPC, `OPC_LUI: begin
            WBSel = 2'b01;
            RegWEn = 1;
        end
        `OPC_LOAD: begin
            WBSel = 2'b00;
            RegWEn = 1;
                `FNC_LB: begin
                    // SINGLE Byte
                    case (addr_offset)
                        2'b00: begin
                            maskedReadData = {MemReadData & (32'h000000FF)};
                        end
                        2'b01: begin
                            maskedReadData
                        end
                    endcase
                end
                `FNC_LBU: begin
                    maskedReadData = {MemReadData & (32'h000000FF << (2*addr_offset))};                
                end
                `FNC_LH: begin
                    //HALF WORD
                    case (addr_offset[1])
                        1'b0: write_mask = 4'b0011; // Lower half-word
                        1'b1: write_mask = 4'b1100; // Upper half-word
                    endcase
                end
                `FNC_LHU: begin

                end
                `FNC_LW: begin
                    //Write the word
                    write_mask = 4'b1111;
                end
                default: write_mask = 4'b0000; 
        end
        `OPC_STORE: begin
            WBSel = 2'b00;
            RegWEn = 1;
        end
        `OPC_BRANCH: begin
            WBSel = 2'b00;
            RegWEn = 0;
        end
        `OPC_JAL, `OPC_JALR: begin
            WBSel = 2'b10;
            RegWEn = 1;
        end
        `OPC_CSR: begin
            WBSel = 2'b01;
            RegWEn = 0;
        end
        default: begin
            WBSel = 2'b00;
            RegWEn = 0;
        end
    endcase
end
assign dcache_re = write_mask;

endmodule