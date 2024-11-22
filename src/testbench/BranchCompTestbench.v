`timescale 1ns/1ps

module BranchComp_TB;

    // Inputs
    reg [31:0] RegData1;
    reg [31:0] RegData2;
    reg BrUn;

    // Outputs
    wire BrEq;
    wire BrLT;

    // Instantiate the DUT (Design Under Test)
    BranchComp dut (
        .RegData1(RegData1),
        .RegData2(RegData2),
        .BrUn(BrUn),
        .BrEq(BrEq),
        .BrLT(BrLT)
    );

    // Test procedure
    initial begin
        // Test 1: Equal values, unsigned comparison
        RegData1 = 32'h5A5A5A5A;
        RegData2 = 32'h5A5A5A5A;
        BrUn = 1'b1; // Unsigned comparison
        #10;
        if (BrEq && !BrLT)
            $display("Test 1 Passed: Equal values, unsigned");
        else
            $display("Test 1 Failed");

        // Test 2: RegData1 < RegData2, unsigned comparison
        RegData1 = 32'h00000001;
        RegData2 = 32'h00000010; 
        BrUn = 1'b1; // Unsigned comparison
        #10;
        if (!BrEq && BrLT)
            $display("Test 2 Passed: RegData1 < RegData2, unsigned");
        else
            $display("Test 2 Failed");

        // Test 3: RegData1 > RegData2, unsigned comparison
        RegData1 = 32'hFFFFFFF0;
        RegData2 = 32'h00000010;
        BrUn = 1'b1; // Unsigned comparison
        #10;
        if (!BrEq && !BrLT)
            $display("Test 3 Passed: RegData1 > RegData2, unsigned");
        else
            $display("Test 3 Failed");

        // Test 4: RegData1 < RegData2, signed comparison
        RegData1 = 32'h80000000; // -2147483648 in signed
        RegData2 = 32'h7FFFFFFF; //  2147483647 in signed
        BrUn = 1'b0; // Signed comparison
        #10;
        if (!BrEq && BrLT)
            $display("Test 4 Passed: RegData1 < RegData2, signed");
        else
            $display("Test 4 Failed");

        // Test 5: RegData1 > RegData2, signed comparison
        RegData1 = 32'h7FFFFFFF; // 2147483647 in signed
        RegData2 = 32'hFFFFFFFF; // -1 in signed
        BrUn = 1'b0; // Signed comparison
        #10;
        if (!BrEq && !BrLT)
            $display("Test 5 Passed: RegData1 > RegData2, signed");
        else
            $display("Test 5 Failed");

        // End simulation
        #10 $stop;
    end

endmodule