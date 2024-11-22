`timescale 1ns/1ps

module tb_PCAdder;

    reg [31:0] PC_Cur;
    wire [31:0] PC_Next;

    // Instantiate the DUT (Device Under Test)
    PCAdder uut (
        .PC_Cur(PC_Cur),
        .PC_Next(PC_Next)
    );

    initial begin
        // Apply test cases
        PC_Cur = 32'd0; #10; // Test case 1
        $display("PC_Cur: %d, PC_Next: %d", PC_Cur, PC_Next);

        PC_Cur = 32'd4; #10; // Test case 2
        $display("PC_Cur: %d, PC_Next: %d", PC_Cur, PC_Next);

        PC_Cur = 32'd100; #10; // Test case 3
        $display("PC_Cur: %d, PC_Next: %d", PC_Cur, PC_Next);

        // End simulation
        $stop;
    end

endmodule