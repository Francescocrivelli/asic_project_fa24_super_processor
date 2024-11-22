module mux_3_to_1(
    input [31:0] in_1,
    input [31:0] in_2,
    input [31:0] in_3,
    input [1:0] sel,

    output reg [31:0] out
);

always@(*) begin

    
    if (sel == 2'b00) begin
        out = in_1;
    end else if (sel == 2'b01) begin
        out = in_2;
    end else begin
        out = in_3;
    end
end


endmodule