module MemWBLogic (
    input [6:0] opcode,
    input funct3,
    input [1:0] addr_lsb,

    
    output reg [3:0] write_mask, 
    output reg  [1:0] WBSel,
    output reg RegWEn
);

`define FUNCT3_BYTE   3'b000
`define FUNCT3_HALF   3'b001
`define FUNCT3_WORD   3'b010

always@(*) begin
    write_mask = 4'b0000;
    WBSel = 2'b00;
    RegWEn = 0;
    case (opcode)
        `OPC_ARI_RTYPE, `OPC_ARI_ITYPE, `OPC_AUIPC: begin
            WBSel = 2'b01;
            RegWEn = 1;
        end
        `OPC_STORE: begin
            WBSel = 2'b00;
            RegWEn = 1;

            case (funct3)
                `FUNCT3_BYTE: begin
                    // SINGLE Byte
                    write_mask = (4'b0001 << addr_lsb);
                end
                `FUNCT3_HALF: begin
                    //HALF WORD
                    case (addr_lsb[1])
                        1'b0: write_mask = 4'b0011; // Lower half-word
                        1'b1: write_mask = 4'b1100; // Upper half-word
                    endcase
                end
                `FUNCT3_WORD: begin
                    //Write the word
                    write_mask = 4'b1111;
                end
                default: write_mask = 4'b0000; 
            endcase
        end
        `OPC_BRANCH: begin
            WBSel = 2'b00;
            RegWEn = 0;
        end
        `OPC_JAL, `OPC_JALR: begin
            WBSel = 2'b10;
            RegWEn = 1;
        end
        default: begin
            WBSel = 2'b00;
            RegWEn = 0;
        end
    endcase
end
assign dcache_re = write_mask

endmodule