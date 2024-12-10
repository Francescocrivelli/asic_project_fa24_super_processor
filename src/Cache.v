`include "util.vh"
`include "const.vh"

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8)
)
(
  input clk,
  input reset,

  input                       cpu_req_valid,
  output                      cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output                      cpu_resp_valid,
  output [CPU_WIDTH-1:0]      cpu_resp_data,

  output                      mem_req_valid,
  input                       mem_req_ready,
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr,
  output                           mem_req_rw,
  output                           mem_req_data_valid,
  input                            mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // byte level masking
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);

  // Implement your cache here, then delete this comment

  // Address partitions
  localparam OFFSET_BITS = 4; // log2(16 words per line)
  localparam INDEX_BITS = 6;  // log2(64 lines)
  localparam TAG_BITS = CPU_WIDTH - OFFSET_BITS - INDEX_BITS;

  wire [TAG_BITS-1:0] tag = cpu_req_addr[WORD_ADDR_BITS-1:WORD_ADDR_BITS-TAG_BITS];
  wire [INDEX_BITS-1:0] index = cpu_req_addr[INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
  wire [OFFSET_BITS-1:0] offset = cpu_req_addr[OFFSET_BITS-1:0];

  // Data and metadata SRAMs
  wire [31:0] data_out;
  wire [31:0] metadata_out;
  reg [31:0] data_in;
  reg [31:0] metadata_in;
  reg data_we, metadata_we;

  // SRAM instances
  sram22_256x32m4w8 data_sram[3:0] (
    .clk(clk), .rstb(reset), .ce(1'b1),
    .we(data_we), .wmask(4'b1111), .addr(index),
    .din(data_in), .dout(data_out)
  );

  sram22_64x32m4w8 metadata_sram (
    .clk(clk), .rstb(reset), .ce(1'b1),
    .we(metadata_we), .wmask(4'b1111), .addr(index),
    .din(metadata_in), .dout(metadata_out)
  );

  // State machine states
  reg [1:0] state, next_state;
  localparam IDLE = 2'b00,
             CHECK = 2'b01,
             FETCH = 2'b10,
             WRITE_BACK = 2'b11;


  always @(posedge clk or negedge reset) begin
    if (!reset)
      state <= IDLE;
    else
      state <= next_state;
  end


  always @(*) begin
    case (state)
      IDLE: begin
        if (cpu_req_valid)
          next_state = CHECK;
        else
          next_state = IDLE;
      end
      CHECK: begin
        if (metadata_out[31]) begin //to set the valid bit @ matias, I am still really cinfused ont he metadata thingy
          if (metadata_out[TAG_BITS-1:0] == tag)
            next_state = IDLE; // Hit
          else
            next_state = FETCH; // Miss
        end else
          next_state = FETCH; // Miss
      end
      FETCH: begin
        if (mem_resp_valid)
          next_state = IDLE;
        else
          next_state = FETCH;
      end
      WRITE_BACK: begin
        if (mem_req_data_ready)
          next_state = FETCH; //WB COmpleted
        else
          next_state = WRITE_BACK;
      end
      default: next_state = IDLE;
    endcase
  end

  // Output logic
  assign cpu_resp_valid = (state == CHECK && metadata_out[31] && metadata_out[TAG_BITS-1:0] == tag);
  assign cpu_resp_data = data_out;

  assign mem_req_valid = (state == FETCH || state == WRITE_BACK);
  assign mem_req_addr = {tag, index};
  assign mem_req_rw = (state == WRITE_BACK);

endmodule
