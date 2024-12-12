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

  reg [1:0] mem_count;
  wire mem_done = (mem_count == 2'b11);

  // Data and metadata SRAMs
  // SRAM control signals
  wire [31:0] data_out0, data_out1, data_out2, data_out3;
  reg [31:0] data_in;
  reg data_we0, data_we1, data_we2, data_we3;
  wire [31:0] metadata_out;
  reg [31:0] metadata_in;
  reg metadata_we;

  //states:
  localparam IDLE = 2'b00;
  localparam COMPARE = 2'b01;
  localparam WRITEBACK = 2'b10;
  localparam FETCH = 2'b11;

  reg [1:0] state, next_state;



  // Individual SRAM instances for each word
  sram22_256x32m4w8 data_sram0 (
    .clk(clk),
    .rstb(!reset),
    .ce(1'b1),
    .we(data_we0),
    .wmask(cpu_req_write),
    .addr(index),
    .din(data_in),
    .dout(data_out0)
  );
  
  sram22_256x32m4w8 data_sram1 (
    .clk(clk),
    .rstb(!reset),
    .ce(1'b1),
    .we(data_we1),
    .wmask(cpu_req_write),
    .addr(index),
    .din(data_in),
    .dout(data_out1)
  );
  
  sram22_256x32m4w8 data_sram2 (
    .clk(clk),
    .rstb(!reset),
    .ce(1'b1),
    .we(data_we2),
    .wmask(cpu_req_write),
    .addr(index),
    .din(data_in),
    .dout(data_out2)
  );
  
  sram22_256x32m4w8 data_sram3 (
    .clk(clk),
    .rstb(!reset),
    .ce(1'b1),
    .we(data_we3),
    .wmask(cpu_req_write),
    .addr(index),
    .din(data_in),
    .dout(data_out3)
  );
  


  sram22_64x32m4w8 metadata_sram (
    .clk(clk),
    .rstb(!reset),
    .ce(1'b1),
    .we(metadata_we),
    .wmask(cpu_req_write),
    .addr(index),
    .din(metadata_in),
    .dout(metadata_out)
  );
    // State machine
    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            mem_count <= 2'b00;
        end else begin
            state <= next_state;
            if (state == FETCH || state == WRITEBACK) begin
                if (mem_resp_valid || mem_req_data_ready)
                    mem_count <= mem_count + 1;
            end else begin
                mem_count <= 2'b00;
            end
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (cpu_req_valid)
                    next_state = COMPARE;
            end
            COMPARE: begin
                if (!metadata_out[31]) // Invalid
                    next_state = FETCH;
                else if (metadata_out[TAG_BITS-1:0] != tag) begin // Tag mismatch
                    if (metadata_out[30]) // Dirty
                        next_state = WRITEBACK;
                    else
                        next_state = FETCH;
                end else // Hit
                    next_state = IDLE;
            end
            WRITEBACK: begin
                if (mem_done && mem_req_data_ready)
                    next_state = FETCH;
            end
            FETCH: begin
                if (mem_done && mem_resp_valid)
                    next_state = IDLE;
            end
        endcase
    end


    // Write enable logic
    always @(*) begin
        data_we0 = 1'b0;
        data_we1 = 1'b0;
        data_we2 = 1'b0;
        data_we3 = 1'b0;
        
        if (state == COMPARE && metadata_out[31] && metadata_out[TAG_BITS-1:0] == tag 
            && cpu_req_write != 4'b0) begin
            case (offset[3:2])
                2'b00: data_we0 = 1'b1;
                2'b01: data_we1 = 1'b1;
                2'b10: data_we2 = 1'b1;
                2'b11: data_we3 = 1'b1;
            endcase
        end
    end

    // Output multiplexing
    reg [31:0] cpu_resp_data_reg;
    always @(*) begin
        case (offset[3:2])
            2'b00: cpu_resp_data_reg = data_out0;
            2'b01: cpu_resp_data_reg = data_out1;
            2'b10: cpu_resp_data_reg = data_out2;
            2'b11: cpu_resp_data_reg = data_out3;
        endcase
    end
    assign cpu_resp_data = cpu_resp_data_reg;

    // Interface signals
    assign cpu_req_ready = (state == IDLE);
    assign cpu_resp_valid = (state == COMPARE && metadata_out[31] && 
                           metadata_out[TAG_BITS-1:0] == tag);
    assign mem_req_valid = (state == WRITEBACK || state == FETCH);
    assign mem_req_rw = (state == WRITEBACK);
    assign mem_req_data_valid = (state == WRITEBACK);
    assign mem_req_addr = {tag, index, mem_count};
    assign mem_req_data_mask = {16{1'b1}};

endmodule
