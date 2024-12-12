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


















// `include "util.vh"
// `include "const.vh"

// module cache #
// (
//   parameter LINES = 64,
//   parameter CPU_WIDTH = `CPU_INST_BITS,
//   parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8)
// )
// (
//   input clk,
//   input reset,

//   input                       cpu_req_valid,
//   output                      cpu_req_ready,
//   input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
//   input [CPU_WIDTH-1:0]       cpu_req_data,
//   input [3:0]                 cpu_req_write,

//   output                      cpu_resp_valid,
//   output [CPU_WIDTH-1:0]      cpu_resp_data,

//   output                      mem_req_valid,
//   input                       mem_req_ready,
//   output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr,
//   output                      mem_req_rw,
//   output                      mem_req_data_valid,
//   input                       mem_req_data_ready,
//   output [`MEM_DATA_BITS-1:0] mem_req_data_bits,
//   output [(`MEM_DATA_BITS/8)-1:0] mem_req_data_mask,

//   input                       mem_resp_valid,
//   input [`MEM_DATA_BITS-1:0]  mem_resp_data
// );

//   localparam CACHE_LINE_SIZE = 512;
//   localparam INDEX_WIDTH = `ceilLog2(LINES);
//   localparam CACHE_ADDR_BITS = 4;
//   localparam TAG_WIDTH = WORD_ADDR_BITS - INDEX_WIDTH - CACHE_ADDR_BITS; 

//   reg [31:0] prev_resp;

//   wire cpu_req_is_write;
//   wire in_hit, in_miss, next_state_is_miss, in_write;
//   wire line_is_dirty, saving_line;
//   wire [1:0] current_dirty_block;

//   reg line_present, previously_in_miss;
//   reg [3:0] line_dirty_blocks;
//   reg [TAG_WIDTH-1:0] line_tag;

//   reg [63:0] meta_present;

//   wire [WORD_ADDR_BITS-1:CACHE_ADDR_BITS+INDEX_WIDTH] tag, prev_tag;
//   wire [WORD_ADDR_BITS-TAG_WIDTH-1:CACHE_ADDR_BITS] index, prev_index;
//   wire [CACHE_ADDR_BITS-1:0] word, prev_word;
//   wire [1:0] sram_lower, wordselect;

//   reg meta_dout_present;
//   wire [3:0] meta_dout_dirty, meta_din_dirty;
//   wire [TAG_WIDTH-1:0] meta_dout_tag, meta_din_tag;

//   reg [2:0] state, next_state;
//   localparam IDLE = 3'b000;
//   localparam READ_QUERY = 3'b001;
//   localparam WRITE_QUERY = 3'b011;
//   localparam CACHE_READ_MISS = 3'b100;
//   localparam CACHE_WRITE_MISS = 3'b110;

//   wire meta_we; 
//   wire [3:0] meta_wmask;
//   wire data_we [4];
//   reg [7:0] data_addr;
//   wire [5:0] meta_addr;
//   wire [CPU_WIDTH-1:0] meta_din;
//   wire [CPU_WIDTH-1:0] data_din [4];
//   wire [CPU_WIDTH-1:0] meta_dout;
//   wire [CPU_WIDTH-1:0] data_dout [4];

//   reg [1:0] current_cache_block;
//   reg [CPU_WIDTH-1:0] async_cache;
//   reg [WORD_ADDR_BITS-1:0] previous_address;
//   reg [31:0] prev_req_data;
//   reg [3:0] prev_req_write;

//   assign cpu_req_is_write = |cpu_req_write;
//   assign {sram_lower, wordselect} = word;
//   assign {tag, index, word} = cpu_req_addr;
//   assign {prev_tag, prev_index, prev_word} = previous_address;
//   assign {meta_dout_dirty, meta_dout_tag} = meta_dout[0+:4+TAG_WIDTH];
//   assign meta_din = {{(32-(4+TAG_WIDTH)){1'b0}}, meta_din_dirty, meta_din_tag};

//   assign data_addr = {
//     in_miss || state == WRITE_QUERY ? prev_index : index,
//     saving_line ? current_dirty_block : in_miss ? current_cache_block : state == WRITE_QUERY ? prev_word[3:2] : sram_lower
//   };
//   assign meta_addr = in_miss || state == WRITE_QUERY ? prev_index : index;

//   assign in_hit = (previously_in_miss && !in_miss) 
//     || (meta_dout_present && meta_dout_tag == prev_tag && (state == READ_QUERY || state == WRITE_QUERY));
//   assign in_miss = state == CACHE_WRITE_MISS || state == CACHE_READ_MISS;
//   assign next_state_is_miss = next_state == CACHE_WRITE_MISS || next_state == CACHE_READ_MISS;
//   assign line_is_dirty = |line_dirty_blocks;
//   assign in_write = state == WRITE_QUERY || state == CACHE_WRITE_MISS;
//   assign current_dirty_block = line_dirty_blocks[0] ? 2'd0 : line_dirty_blocks[1] ? 2'd1 : line_dirty_blocks[2] ? 2'd2 : 2'd3;
//   assign saving_line = line_is_dirty && in_miss;

//   assign cpu_resp_valid = (state == READ_QUERY || state == IDLE) && in_hit;
//   assign cpu_req_ready = state == IDLE || (state == READ_QUERY && in_hit);
//   assign cpu_resp_data = previously_in_miss ? async_cache : state == IDLE ? prev_resp : data_dout[prev_word[1:0]];

//   assign mem_req_rw = saving_line;
//   assign mem_req_data_valid = mem_req_valid;
//   assign mem_req_valid = saving_line || (in_miss && current_cache_block == 2'd0);
//   assign mem_req_addr = saving_line ? {line_tag, prev_index, current_dirty_block} : {prev_tag, prev_index, 2'b00};
//   assign mem_req_data_bits = {data_dout[3], data_dout[2], data_dout[1], data_dout[0]};
//   assign mem_req_data_mask = 16'hFFFF;

//   assign meta_wmask = 4'hF;
//   assign meta_we = (state == WRITE_QUERY && in_hit) || (in_miss && !next_state_is_miss);
//   assign meta_din_tag = prev_tag;

//   genvar i;
//   generate
//     for (i = 0; i < 4; i = i + 1) begin
//       assign data_we[i] = (state == CACHE_READ_MISS && mem_resp_valid)
//         || (state == WRITE_QUERY && in_hit && prev_word[1:0] == i[1:0])
//         || (state == CACHE_WRITE_MISS && (!line_present || !saving_line) && mem_resp_valid);
//       assign data_din[i] = (state == WRITE_QUERY && in_hit) ? prev_req_data : mem_resp_data[CPU_WIDTH*i+:CPU_WIDTH];
//       assign meta_din_dirty[i] = (state == WRITE_QUERY && in_hit && prev_word[3:2] == i[1:0]) 
//         || (state == WRITE_QUERY && in_hit && !previously_in_miss ? meta_dout_dirty[i] : line_dirty_blocks[i]);
//     end
//   endgenerate

//   sram22_256x32m4w8 sramData0 (
//     .clk(clk),
//     .we(data_we[0]),
//     .wmask(state == WRITE_QUERY ? prev_req_write : 4'hF),
//     .addr(data_addr),
//     .din(data_din[0]),
//     .dout(data_dout[0])
//   );    

//   sram22_256x32m4w8 sramData1 (
//     .clk(clk),
//     .we(data_we[1]),
//     .wmask(state == WRITE_QUERY ? prev_req_write : 4'hF),
//     .addr(data_addr),
//     .din(data_din[1]),
//     .dout(data_dout[1])
//   );    

//   sram22_256x32m4w8 sramData2 (
//     .clk(clk),
//     .we(data_we[2]),
//     .wmask(state == WRITE_QUERY ? prev_req_write : 4'hF),
//     .addr(data_addr),
//     .din(data_din[2]),
//     .dout(data_dout[2])
//   );    

//   sram22_256x32m4w8 sramData3 (
//     .clk(clk),
//     .we(data_we[3]),
//     .wmask(state == WRITE_QUERY ? prev_req_write : 4'hF),
//     .addr(data_addr),
//     .din(data_din[3]),
//     .dout(data_dout[3])
//   );        

//   sram22_64x32m4w8 sramMeta (
//     .clk(clk),
//     .we(meta_we),
//     .wmask(meta_wmask),
//     .addr(meta_addr),
//     .din(meta_din),
//     .dout(meta_dout)
//   );

//   always @(*) begin
//     next_state = state;
//     case (state)
//       IDLE: begin
//         if (cpu_req_valid && cpu_req_ready) next_state = cpu_req_is_write ? WRITE_QUERY : READ_QUERY;
//       end
//       READ_QUERY: begin
//         if (!in_hit) next_state = CACHE_READ_MISS;
//         else if (!cpu_req_valid) next_state = IDLE;
//       end
//       WRITE_QUERY: begin
//         if (!in_hit) next_state = CACHE_WRITE_MISS;
//         else next_state = IDLE;
//       end
//       CACHE_READ_MISS: begin
//         if (mem_resp_valid && current_cache_block == 2'b11) next_state = IDLE;
//       end
//       CACHE_WRITE_MISS: begin
//         if (mem_resp_valid && current_cache_block == 2'b11) next_state = WRITE_QUERY;
//       end 
//     endcase
//   end

//   always @(posedge clk) begin
//     if (reset) begin
//       state <= IDLE;
//       current_cache_block <= 2'd0;
//       line_dirty_blocks <= 4'd0;
//       previously_in_miss <= 1'b0;
//       meta_present <= 64'd0;
//     end else begin
//       state <= next_state;
//       previously_in_miss <= in_miss;
//       if (in_miss && next_state == WRITE_QUERY) meta_dout_present <= 1'b1;
//       else meta_dout_present <= meta_present[meta_addr];
//       if (!in_miss && (next_state == READ_QUERY || next_state == WRITE_QUERY)) previous_address <= cpu_req_addr;
//       if (next_state == WRITE_QUERY && !in_miss) begin
//         prev_req_data <= cpu_req_data;
//         prev_req_write <= cpu_req_write;
//       end
//       if ((state == READ_QUERY || state == WRITE_QUERY) && next_state_is_miss) begin
//         current_cache_block <= 2'd0;
//         line_present <= meta_dout_present;
//         line_dirty_blocks <= meta_dout_dirty;
//         line_tag <= meta_dout_tag;
//       end
//       if (state == READ_QUERY) prev_resp <= cpu_resp_data;
//       if (saving_line && mem_req_data_ready) line_dirty_blocks[current_dirty_block] <= 1'b0;
//       if (in_miss) begin
//         if (mem_resp_valid && !line_is_dirty) begin
//           current_cache_block <= current_cache_block + 2'd1;
//           if (state == CACHE_READ_MISS && current_cache_block == prev_word[3:2]) async_cache <= data_din[prev_word[1:0]];
//         end
//         if (!next_state_is_miss) meta_present[prev_index] <= 1'b1;
//       end
//     end
//   end

// endmodule
