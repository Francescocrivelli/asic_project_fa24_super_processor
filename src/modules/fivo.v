//=========================================================================
// FIFO Template
//-------------------------------------------------------------------------
//
//`include "EECS151.v"

module fifo #(parameter WIDTH = 8, parameter LOGDEPTH = 3) (
    input clk,
    input reset,

    input enq_val,
    input [WIDTH-1:0] enq_data,
    output enq_rdy,

    output deq_val,
    output [WIDTH-1:0] deq_data,
    input deq_rdy

);

localparam DEPTH = (1 << LOGDEPTH);

// the buffer itself. Take note of the 2D syntax.
reg [WIDTH-1:0] buffer [DEPTH-1:0];
// read pointer, write pointer
reg [LOGDEPTH-1:0] rptr, wptr; // wp-> enq   rp_> for deq
// is the buffer full? This is needed for when rptr == wptr
reg full;

// Define any additional regs or wires you need (if any) here
reg empty;// we create it as a reg type becuae is in an always at block

// use "fire" to indicate when a valid transaction has been made
wire enq_fire;
wire deq_fire;

assign enq_fire = enq_val & enq_rdy;
assign deq_fire = deq_val & deq_rdy;
always @(posedge clk) begin
    // can not assigned a reg to an output signal in n always at block
    if (reset) begin
        // Reset all pointers and flags
        rptr <= 0;
        wptr <= 0;
        full <= 0;
        empty <= 1;
    end else begin
        if(enq_fire) begin //enq and deq fire tell us when we are ready to do a transaction
            buffer[wptr] <= enq_data;
            wptr <= (wptr + 1'b1) % DEPTH;
            empty <= 1'b0;
            if((wptr + 1'b1)% DEPTH == rptr) begin
                full <= 1'b1;
            end
        end
        if (deq_fire)begin
            rptr <= (rptr + 1)% DEPTH;
            full <= 1'b0;
            if ((rptr + 1)% DEPTH == wptr) begin
                empty <= 1'b1;
            end
        end
        
    end


end
assign deq_data = buffer[rptr];
assign deq_val = !empty;
assign enq_rdy = !full;

endmodule
