module mux_5_to_1( // input are defoult to wire
    input [31:0] in_1,
    input [31:0] in_2,
    input [31:0] in_3,
    input [31:0] in_4,
    input [31:0] in_5,
    output reg [31:0] out,
    input [2:0] sel
);

always@(*) begin
    case (sel)
        3'b000: out = in_1;
        3'b001: out = in_2;
        3'b010: out = in_3;
        3'b011: out = in_4;
        3'b100: out = in_5;

        default: out = 32'b0;
    endcase
end
endmodule


// when using always at star block the output should be a reg
