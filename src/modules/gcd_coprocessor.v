//=========================================================================
// Template for GCD coprocessor
//-------------------------------------------------------------------------
//

module gcd_coprocessor #( parameter W = 32 ) (
    input clk,
    input reset,

    input operands_val,  // Same valid signal for both A and B inputs
    input [W-1:0] operands_bits_A,
    input [W-1:0] operands_bits_B,
    output operands_rdy, // Both FIFOs must be ready for operands to be ready

    output result_val,
    output [W-1:0] result_bits,
    input result_rdy
);

// You should be able to build this with mostly structural verilog!
// TODO: Define wires
// //FIFO request and FIFO response



//request FIFO A
    wire req_deq_val_A, req_deq_val_B;
    wire [W-1:0] req_deq_data_A, req_deq_data_B;
    wire req_enq_rdy_A, req_enq_rdy_B;
    wire req_deq_rdy_A, req_deq_rdy_B;
    // Response FIFO wires
    wire res_enq_rdy;

    // // GCD unit wires
    // wire gcd_operands_rdy;
    // wire gcd_result_val;
    // wire [W-1:0] gcd_result_bits;


     // TODO: Instantiate gcd_datapath -> only need to modify the external
    // Combine readiness of both FIFO queues for the ready signal
    // assign operands_rdy = req_deq_rdy_A && req_deq_rdy_B;

    // wirrs for gcd units 1 and 2 their inouts will be determined by the arbiter
    wire gcd_operands_val_0, gcd_operands_val_1;
    wire gcd_operands_rdy_0, gcd_operands_rdy_1;
    wire [W-1:0] gcd_operands_bits_A_0, gcd_operands_bits_A_1;
    wire [W-1:0] gcd_operands_bits_B_0, gcd_operands_bits_B_1;
    wire gcd_result_val_0, gcd_result_val_1;
    wire gcd_result_rdy_0, gcd_result_rdy_1;
    wire [W-1:0] gcd_result_bits_0, gcd_result_bits_1;
    
    // arbiter wires
    wire arbiter_operands_val;
    wire arbiter_operands_rdy;
    wire [W-1:0] arbiter_operands_bits_A;
    wire [W-1:0] arbiter_operands_bits_B;
    wire arbiter_result_val;
    wire [W-1:0] arbiter_result_bits_data;


fifo #(.WIDTH(W)) request_fifo_A (
    .clk(clk),
    .reset(reset),
    .enq_val(operands_val),         // Operand A valid signal
    .enq_data(operands_bits_A),     // Operand A data
    .enq_rdy(req_enq_rdy_A),        // FIFO ready for operand A
    .deq_val(req_deq_val_A),        // FIFO has valid operand A to dequeue
    .deq_data(req_deq_data_A),      // Dequeued operand A data
    .deq_rdy(arbiter_operands_rdy)  // GCD datapath ready to accept operand A
);

// Instantiate Request FIFO for operands B
fifo #(.WIDTH(W)) request_fifo_B (
    .clk(clk),
    .reset(reset),
    .enq_val(operands_val),         // Operand B valid signal
    .enq_data(operands_bits_B),     // Operand B data
    .enq_rdy(req_enq_rdy_B),        // FIFO ready for operand B
    .deq_val(req_deq_val_B),        // FIFO has valid operand B to dequeue
    .deq_data(req_deq_data_B),      // Dequeued operand B data
    .deq_rdy(arbiter_operands_rdy)  // GCD datapath ready to accept operand B
);



    gcd_unit #(W) gcd_0 (
      .clk(clk),
      .reset(reset),
      .operands_val(gcd_operands_val_0),
      .operands_bits_A(gcd_operands_bits_A_0),
      .operands_bits_B(gcd_operands_bits_B_0),
      .operands_rdy(gcd_operands_rdy_0),
      .result_val(gcd_result_val_0),
      .result_bits_data(gcd_result_bits_0),
      .result_rdy(gcd_result_rdy_0)
    );
    gcd_unit #(W) gcd_1 (
      .clk(clk),
      .reset(reset),
      .operands_val(gcd_operands_val_1),
      .operands_bits_A(gcd_operands_bits_A_1),
      .operands_bits_B(gcd_operands_bits_B_1),
      .operands_rdy(gcd_operands_rdy_1),
      .result_val(gcd_result_val_1),
      .result_bits_data(gcd_result_bits_1),
      .result_rdy(gcd_result_rdy_1)
    );

        // Assign ready when both FIFOs can accept data
    assign operands_rdy = req_enq_rdy_A && req_enq_rdy_B;

    // Operands are valid when both FIFOs have data
    assign arbiter_operands_val = req_deq_val_A && req_deq_val_B;
    assign arbiter_operands_bits_A = req_deq_data_A;
    assign arbiter_operands_bits_B = req_deq_data_B;



    //instantiating the arbiter:
    gcd_arbiter #(W) arbiter (
    .clk(clk),
    .reset(reset),

    // Inputs from the input FIFO (operands)
    .operands_val(arbiter_operands_val),                    // Input valid signal from FIFO
    .operands_bits_A(arbiter_operands_bits_A),              // Operand A from FIFO
    .operands_bits_B(arbiter_operands_bits_B),              // Operand B from FIFO
    .operands_rdy(arbiter_operands_rdy),                    // Output ready signal to FIFO

    // Outputs to GCD Unit 0
    .request0_val(gcd_operands_val_0),              // Send operands to GCD Unit 0
    .request0_operands_bits_A(gcd_operands_bits_A_0), // Operand A for GCD Unit 0
    .request0_operands_bits_B(gcd_operands_bits_B_0), // Operand B for GCD Unit 0
    .request0_rdy(gcd_operands_rdy_0),              // Ready signal from GCD Unit 0

    // Outputs to GCD Unit 1
    .request1_val(gcd_operands_val_1),              // Send operands to GCD Unit 1
    .request1_operands_bits_A(gcd_operands_bits_A_1), // Operand A for GCD Unit 1
    .request1_operands_bits_B(gcd_operands_bits_B_1), // Operand B for GCD Unit 1
    .request1_rdy(gcd_operands_rdy_1),              // Ready signal from GCD Unit 1

    // Outputs to the output FIFO (results)
    .result_val(arbiter_result_val),                        // Final result valid signal (for output FIFO)
    .result_bits_data(arbiter_result_bits_data),                 // Final result data (for output FIFO)
    .result_rdy(res_enq_rdy),                        // Ready signal from output FIFO

    // Inputs from GCD Unit 0 (result)
    .response0_val(gcd_result_val_0),               // GCD Unit 0 result valid
    .response0_result_bits_data(gcd_result_bits_0), // GCD Unit 0 result data
    .response0_rdy(gcd_result_rdy_0),               // Ready signal for GCD Unit 0 result

    // Inputs from GCD Unit 1 (result)
    .response1_val(gcd_result_val_1),               // GCD Unit 1 result valid
    .response1_result_bits_data(gcd_result_bits_1), // GCD Unit 1 result data
    .response1_rdy(gcd_result_rdy_1)                // Ready signal for GCD Unit 1 result
);


    
    // TODO: Instantiate response FIFO
fifo #(.WIDTH(W)) response_fifo (
    .clk(clk),
    .reset(reset),
    .enq_val(arbiter_result_val),
    .enq_data(arbiter_result_bits_data),
    .enq_rdy(res_enq_rdy),
    .deq_val(result_val),
    .deq_data(result_bits),
    .deq_rdy(result_rdy)
);
endmodule