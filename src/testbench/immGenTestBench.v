`timescale 1ns/1ps

module immGen_tb;

    // Inputs
    reg [31:0] inst;
    reg [4:0] imm_sel;
    
    // Outputs
    wire [31:0] imm;

    // Instantiate the Design Under Test (DUT)
    immGen dut (
        .inst(inst),
        .imm_sel(imm_sel),
        .imm(imm)
    );

    // Test stimulus
    initial begin
        // Initialize inputs
        inst = 0;
        imm_sel = 0;
        #10;

        // Test 1: I-type instruction (ADDI)
        // Example: addi x1, x0, -50
        $display("Test 1: I-type instruction");
        imm_sel = 4'd5;  // I-type
        inst = 32'b111111001110_00000_000_00001_0010011;  // -50 in immediate
        #10;
        $display("I-type: inst = %b", inst);
        $display("Expected: -50, Got: %d", $signed(imm));
        
        // Test 2: S-type instruction (SW)
        // Example: sw x1, 20(x2)
        $display("\nTest 2: S-type instruction");
        imm_sel = 4'd1;  // S-type
        inst = 32'b0000001_00001_00010_010_10100_0100011;  // 20 in immediate
        #10;
        $display("S-type: inst = %b", inst);
        $display("Expected: 20, Got: %d", $signed(imm));

        // Test 3: B-type instruction (BEQ)
        // Example: beq x1, x2, 8
        $display("\nTest 3: B-type instruction");
        imm_sel = 4'd2;  // B-type
        inst = 32'b0000000_00010_00001_000_01000_1100011;  // 8 in immediate
        #10;
        $display("B-type: inst = %b", inst);
        $display("Expected: 8, Got: %d", $signed(imm));

        // Test 4: U-type instruction (LUI)
        // Example: lui x1, 0x12345
        $display("\nTest 4: U-type instruction");
        imm_sel = 4'd3;  // U-type
        inst = 32'b00010010001101000101_00001_0110111;  // 0x12345
        #10;
        $display("U-type: inst = %b", inst);
        $display("Expected: 0x12345000, Got: %h", imm);

        // Test 5: J-type instruction (JAL)
        // Example: jal x1, 16
        $display("\nTest 5: J-type instruction");
        imm_sel = 4'd4;  // J-type
        inst = 32'b0000000000100000000000001_1101111;  // 16 in immediate
        #10;
        $display("J-type: inst = %b", inst);
        $display("Expected: 16, Got: %d", $signed(imm));

        // Test 6: I*-type instruction (SLLI)
        // Example: slli x1, x2, 4
        $display("\nTest 6: I*-type instruction");
        imm_sel = 4'd6;  // I*-type
        inst = 32'b0000000_00100_00010_001_00001_0010011;  // 4 in immediate
        #10;
        $display("I*-type: inst = %b", inst);
        $display("Expected: 4, Got: %d", $signed(imm));

        // Test 7: R-type instruction (should output 0)
        $display("\nTest 7: R-type instruction");
        imm_sel = 4'd0;  // R-type
        inst = 32'b0000000_00010_00011_000_00001_0110011;
        #10;
        $display("R-type: inst = %b", inst);
        $display("Expected: 0, Got: %d", $signed(imm));

        $display("\nAll tests completed!");
        #10 $finish;
    end

endmodule