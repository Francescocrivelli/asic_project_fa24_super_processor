module mux_3_to_1(
    input [31:0] in_1,
    input [31:0] in_2,
    input [31:0] in_3,
    input [1:0] sel,

    output reg [31:0] out
);

always@(*) begin
    case (sel)
        2'b00: out = in_1;
        2'b01: out = in_2;
        2'b10: out = in_3;
        default: out = 32'b0;
    endcase
end


endmodule