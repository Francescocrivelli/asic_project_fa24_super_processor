module mux_2_to_1(
    input wire [31:0] in_1,
    input wire [31:0] in_2,
    input wire select,
    output wire[31:0] out
);
assign out = (select) ? in_2 : in_1

endmodule