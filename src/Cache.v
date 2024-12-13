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
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);

  localparam OFFSET_BITS = 6; // log2(16 words per line = 512 bits/32 bits per word)
  localparam INDEX_BITS = 6;  // log2(64 lines)
  localparam TAG_BITS = (WORD_ADDR_BITS) - OFFSET_BITS - INDEX_BITS;

  wire [TAG_BITS-1:0]   tag = cpu_req_addr[WORD_ADDR_BITS-1:WORD_ADDR_BITS-TAG_BITS];
  wire [INDEX_BITS-1:0] index = cpu_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
  wire [OFFSET_BITS-1:0] offset = cpu_req_addr[OFFSET_BITS-1:0];

  reg [1:0] mem_count;
  wire mem_done = (mem_count == 2'b11);

  // Data and metadata SRAM I/O
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

  reg [511:0] cache_line;  // Full cache line buffer (optional usage)

  // Instantiate data srams
  // The SRAM expects: .addr(8-bit), .we(1-bit), .wmask(4-bit)
    sram22_256x32m4w8 data_sram0 (
    .clk(clk),
    .we(data_we0),
    .wmask(cpu_req_write),
    .addr({2'b00, index}),
    .din(data_in),
    .dout(data_out0)
    );


  sram22_256x32m4w8 data_sram1 (
    .clk(clk),
    .we(data_we1),
    .wmask(cpu_req_write),
    .addr({2'b00, index}),
    .din(data_in),
    .dout(data_out1)
  );

  sram22_256x32m4w8 data_sram2 (
    .clk(clk),
    .we(data_we2),
    .wmask(cpu_req_write),
    .addr({2'b00, index}),
    .din(data_in),
    .dout(data_out2)
  );

  sram22_256x32m4w8 data_sram3 (
    .clk(clk),
    .we(data_we3),
    .wmask(cpu_req_write),
    .addr({2'b00, index}),
    .din(data_in),
    .dout(data_out3)
  );

  // Metadata SRAM (64x32)
  sram22_64x32m4w8 metadata_sram (
    .clk(clk),
    .we(metadata_we),
    .wmask(4'b1111),
    .addr(index),  // 6-bit index matches directly
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

      if ((state == FETCH && mem_resp_valid) || (state == WRITEBACK && mem_req_data_ready)) begin
        mem_count <= mem_count + 2'b01; 
      end else if (state != FETCH && state != WRITEBACK) begin
        mem_count <= 2'b00;
      end
    end
  end

  always @(*) begin
    // Default
    data_in = 32'b0;
    if (state == FETCH && mem_resp_valid) begin
      case (mem_count)
        2'b00: data_in = mem_resp_data[31:0];
        2'b01: data_in = mem_resp_data[63:32];
        2'b10: data_in = mem_resp_data[95:64];
        2'b11: data_in = mem_resp_data[127:96];
      endcase
    end else if (state == COMPARE && (cpu_req_write != 4'b0) &&
                 metadata_out[31] && (metadata_out[TAG_BITS-1:0] == tag)) begin
      data_in = cpu_req_data;
    end
  end

  always @(*) begin
    metadata_we = 1'b0;
    metadata_in = 32'b0;

    // Metadata format: [Valid(31), Dirty(30), Unused(29:TAG_BITS), Tag(TAG_BITS-1:0)]
    if (state == FETCH && mem_done && mem_resp_valid) begin
      // After fetching new line: Valid=1, Dirty=0, Tag = current tag
      metadata_we = 1'b1;
      metadata_in = {1'b1, 1'b0, {(30 - TAG_BITS){1'b0}}, tag};
    end else if (state == COMPARE && cpu_req_write != 4'b0 &&
                 metadata_out[31] && (metadata_out[TAG_BITS-1:0] == tag)) begin
      // On a hit write, set dirty bit
      metadata_we = 1'b1;
      metadata_in = {1'b1, 1'b1, {(30 - TAG_BITS){1'b0}}, tag};
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
        // Check valid bit
        if (!metadata_out[31]) begin // Invalid line
          next_state = FETCH;
        end else if (metadata_out[TAG_BITS-1:0] != tag) begin // Tag mismatch
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

  always @(posedge clk) begin
    if (state == FETCH && mem_resp_valid) begin
      case (mem_count)
        2'b00: cache_line[127:0]   <= mem_resp_data;
        2'b01: cache_line[255:128] <= mem_resp_data;
        2'b10: cache_line[383:256] <= mem_resp_data;
        2'b11: cache_line[511:384] <= mem_resp_data;
      endcase
    end else if (state == COMPARE && metadata_out[31] &&
                 (metadata_out[TAG_BITS-1:0] == tag) && cpu_req_write != 4'b0) begin
      // Update cache_line copy if needed (not strictly necessary if we rely on SRAM)
      cache_line[offset*32 +: 32] <= cpu_req_data;
    end
  end

  // Write enable logic for data SRAMs
  always @(*) begin
    data_we0 = 1'b0;
    data_we1 = 1'b0;
    data_we2 = 1'b0;
    data_we3 = 1'b0;

    if (state == COMPARE && metadata_out[31] && (metadata_out[TAG_BITS-1:0] == tag) 
        && cpu_req_write != 4'b0) begin
      case (offset[3:2])
        2'b00: data_we0 = 1'b1;
        2'b01: data_we1 = 1'b1;
        2'b10: data_we2 = 1'b1;
        2'b11: data_we3 = 1'b1;
      endcase
    end else if (state == FETCH && mem_resp_valid) begin
      // Writing fetched data into the SRAM line
      case (mem_count)
        2'b00: data_we0 = 1'b1;
        2'b01: data_we1 = 1'b1;
        2'b10: data_we2 = 1'b1;
        2'b11: data_we3 = 1'b1;
      endcase
    end
  end

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
  assign cpu_req_ready = (state == IDLE);
  assign cpu_resp_valid = (state == COMPARE && metadata_out[31] && 
                           (metadata_out[TAG_BITS-1:0] == tag));

  // Memory interface
  assign mem_req_valid     = (state == WRITEBACK || state == FETCH);
  assign mem_req_rw        = (state == WRITEBACK);
  assign mem_req_data_valid= (state == WRITEBACK);
  assign mem_req_addr      = {tag, index, mem_count};
  assign mem_req_data_mask = {16{1'b1}};
  assign mem_req_data_bits = (state == WRITEBACK) ? cache_line[mem_count*128 +: 128] : 128'b0;

endmodule

