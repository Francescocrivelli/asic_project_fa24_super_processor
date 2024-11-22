module mux_2_to_1(
    input [31:0] in_1;
    input [31:0] in_2;
    input [31:0] in_3;
    input [1:0] sel;

    output [31:0] out;
);

always@(*) begin
    if (sel == 1'b0) begin

    end else if (sel == 1'b1) begin

    end else begin

    end
end


endmodule