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
reg [31:0] load_data;


always@(*) begin
    write_mask = 4'b0000;
    load_data = 0;
    WBSel = 2'b00;
    RegWEn = 0;
    case (opcode)
        `OPC_ARI_RTYPE, `OPC_ARI_ITYPE, `OPC_AUIPC, `OPC_LUI: begin
            WBSel = 2'b01;
            RegWEn = 1;
            maskedReadData = MemReadData;
        end
        `OPC_LOAD: begin
            WBSel = 2'b00;
            RegWEn = 1;
            case (funct3)
                `FNC_LB: begin
                    // SINGLE Byte
                    case (addr_offset)
                        2'b00: begin
                            load_data = MemReadData[7:0];
                            maskedReadData = {{24{load_data[7]}}, load_data[7:0]};
                        end
                        2'b01: begin
                            load_data = MemReadData[15:8];
                            maskedReadData = {{24{load_data[7]}}, load_data[7:0]};
                        end
                        2'b10: begin
                            load_data = MemReadData[23:16];
                            maskedReadData = {{24{load_data[7]}}, load_data[7:0]};
                        end
                        2'b11: begin
                            load_data = MemReadData[31:24];
                            maskedReadData = {{24{load_data[7]}}, load_data[7:0]};
                        end
                    endcase
                end
                `FNC_LBU: begin
                    case (addr_offset)
                        2'b00: begin
                            maskedReadData = MemReadData[7:0];
                        end
                        2'b01: begin
                            maskedReadData = MemReadData[15:8];
                        end
                        2'b10: begin
                            maskedReadData = MemReadData[23:16];
                        end
                        2'b11: begin
                            maskedReadData = MemReadData[31:24];
                        end
                    endcase                
                end
                `FNC_LH: begin
                    //HALF WORD
                    case (addr_offset[1])
                        1'b0: begin
                            load_data = MemReadData[15:0];
                            maskedReadData = {{16{load_data[15]}}, load_data[15:0]};
                        end
                        1'b1: begin
                            load_data = MemReadData[31:16];
                            maskedReadData = {{16{load_data[15]}}, load_data[15:0]};
                        end
                    endcase
                end
                `FNC_LHU: begin
                    case (addr_offset[1])
                        1'b0: begin
                            maskedReadData = MemReadData[15:0];
                        end
                        1'b1: begin
                            maskedReadData = MemReadData[31:16];
                        end
                    endcase        
                end
                `FNC_LW: begin
                    //Write the word
                    maskedReadData = MemReadData;
                end
                //default: 
                    //maskedReadData = MemReadData;
            endcase 
        end
        `OPC_STORE: begin
            WBSel = 2'b00;
            RegWEn = 0;
            maskedReadData = MemReadData;
        end
        `OPC_BRANCH: begin
            WBSel = 2'b00;
            RegWEn = 0;
            maskedReadData = MemReadData;
        end
        `OPC_JAL, `OPC_JALR: begin
            WBSel = 2'b10;
            RegWEn = 1;
            maskedReadData = MemReadData;
        end
        `OPC_CSR: begin
            WBSel = 2'b01;
            RegWEn = 0;
            maskedReadData = MemReadData;
        end
        default: begin
            WBSel = 2'b00;
            RegWEn = 0;
            maskedReadData = MemReadData;
        end
    endcase
end

endmodule