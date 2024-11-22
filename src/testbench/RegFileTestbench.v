
module regTestbench();

    //write enable
    wire we;

    //write
    wire [4:0] wb_addr;
    wire [31:0] wb_data;

    // read adress input
    wire [4:0] rs1_addr;
    wire [4:0] rs2_addr;

    // read the data of rs1
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;

endmodule