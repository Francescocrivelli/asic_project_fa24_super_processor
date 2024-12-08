module mux_4_to_1( // input are defoult to wire
    input [31:0] in_1,
    input [31:0] in_2,
    input [31:0] in_3,
    input [31:0] in_4,
    output reg [31:0] out,
    input [1:0] sel
);

always@(*) begin
    case (sel)
        2'b00: out = in_1;
        2'b01: out = in_2;
        2'b10: out = in_3;
        2'b11: out = in_4;
        default: out = 32'b0;
    endcase
end
endmodule


// when using always at star block the output should be a reg
