module mux_2_to_1(
    input wire [31:0] in_1,
    input wire [31:0] in_2,
    input wire sel,
    output wire[31:0] out
);
assign out = (sel) ? in_2 : in_1

endmodule
