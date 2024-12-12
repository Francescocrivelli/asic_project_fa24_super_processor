
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
  output                      cpu_req_ready, // assigned properly -> cpu is not ready os a stall
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output                      cpu_resp_valid, // check
  output [CPU_WIDTH-1:0]      cpu_resp_data, // check

  output                      mem_req_valid, // check
  input                       mem_req_ready,
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr, // check
  output                           mem_req_rw, // check
  output                           mem_req_data_valid, // check
  input                            mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits, // check
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask, 

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);




localparam SLICE_BITS = 2; // log2(4 slices)
localparam OFFSET_BITS = 4; // log2(16 words per line = 512 bits/32 bits per word)
localparam INDEX_BITS = 6;  // log2(64 lines)
localparam TAG_BITS = WORD_ADDR_BITS - OFFSET_BITS - INDEX_BITS;

// Extract fields from the CPU address
wire [TAG_BITS-1:0]   tag    = cpu_req_addr[WORD_ADDR_BITS-1:WORD_ADDR_BITS-TAG_BITS];
wire [INDEX_BITS-1:0] index  = cpu_req_addr[OFFSET_BITS + INDEX_BITS - 1:OFFSET_BITS];
wire [1:0]            slice  = cpu_req_addr[1:0];      // Select which SRAM (2 bits)
wire [1:0]            offset = cpu_req_addr[3:2];      // Select word within a block (remaining 2 bits)


localparam IDLE = 2'b00;
localparam CACHE_LOAD = 2'b01;
localparam MEM_LOAD = 2'b10;
localparam WRITE = 2'b11;

reg [1:0] next_state;
reg [1:0] state;

reg data_we [4];
reg meta_we;
reg [3:0] meta_wmask;
reg [3:0] data_wmask [4];

reg [CPU_WIDTH-1:0] meta_in;
reg [CPU_WIDTH-1:0] meta_out;


reg [CPU_WIDTH-1:0] data_in [4];
reg [CPU_WIDTH-1:0] data_out [4];

assign wmask = {{8{cpu_req_write[3]}},
                  {8{cpu_req_write[2]}},
                  {8{cpu_req_write[1]}},
                  {8{cpu_req_write[0]}}};

reg [7:0] sram_addr0;
reg [7:0] sram_addr1;
reg [7:0] sram_addr2;
reg [7:0] sram_addr3;

reg [5:0] meta_addr;

// Instantiate data srams
  // The SRAM expects: .addr(8-bit), .we(1-bit), .wmask(4-bit)
sram22_256x32m4w8 data_sram0 (
  .clk(clk),
  .we(data_we[0]),
  .wmask(data_wmask[0]),
  .addr(sram_addr0),
  .din(data_in[0]),
  .dout(data_out[0])
);

sram22_256x32m4w8 data_sram1 (
  .clk(clk),
  .we(data_we[1]),
  .wmask(data_wmask[1]),
  .addr(sram_addr1),
  .din(data_in[1]),
  .dout(data_out[1])
);

sram22_256x32m4w8 data_sram2 (
  .clk(clk),
  .we(data_we[2]),
  .wmask(data_wmask[2]),
  .addr(sram_addr2),
  .din(data_in[2]),
  .dout(data_out[2])
);

sram22_256x32m4w8 data_sram3 (
  .clk(clk),
  .we(data_we[3]),
  .wmask(data_wmask[3]),
  .addr(sram_addr3),
  .din(data_in[3]),
  .dout(data_out[3])
);

// Metadata SRAM (64x32)
sram22_64x32m4w8 metadata_sram (
  .clk(clk),
  .we(meta_we),
  .wmask(meta_wmask),
  .addr(meta_addr),
  .din(meta_in),
  .dout(meta_out)
);


integer i;

reg [(`MEM_DATA_BITS/8)-1:0] mem_data_mask;
reg [`MEM_DATA_BITS-1:0] mem_data_bits;
reg mem_valid;
reg [CPU_WIDTH-1:0] cpu_data;
reg cpu_valid; // signal goes high if data is ready to be sent to cpu
reg cpu_rdy; // signal if state=IDLE to receive addr
reg [1:0] load_counter; // counter to spend 4 cycles in load
reg mem_write;
reg mem_addr;
reg mem_data_valid;

// Extract valid and tag from metadata
wire valid_bit = meta_out[TAG_BITS];
wire [TAG_BITS-1:0] stored_tag = meta_out[TAG_BITS-1:0];

always@(*) begin
  if (reset) begin
    for (i=0; i<4; i=i+1) begin
      data_we[i] = 0;
      data_wmask[i] = 4'b0000;
      data_in[i] = 32'b0;
    end
    cpu_valid = 1'b0;
    cpu_rdy = 1'b0;
    meta_we = 0;
    cpu_data = 0;
    meta_wmask = 4'b0000;
    meta_in = 32'b0;
    mem_valid = 0;
    mem_addr = {WORD_ADDR_BITS-`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH){1'b0}};
    mem_write = 1'b0;
    mem_data_valid = 1'b0;
    mem_data_bits = {`MEM_DATA_BITS{1'b0}};
    mem_data_mask = {(`MEM_DATA_BITS/8){1'b0}};

    next_state = IDLE;

  end else begin
    case (state)
      IDLE: begin
        cpu_rdy = 1'b1;
        if (cpu_req_valid) begin // check if cput request memory and if its a read operation 
          meta_addr = index;
          case (slice) 
            2'b00: begin
              sram_addr0 = {offset, index};
            end
            2'b01: begin
              sram_addr1 = {offset, index};
            end
            2'b10: begin
              sram_addr2 = {offset, index};
            end
            2'b11: begin
              sram_addr3 = {offset, index};
            end
          endcase
          if (!cpu_req_write) begin
            next_state = CACHE_LOAD; 
          end else if (cpu_req_valid && cpu_req_write) begin
            next_state = WRITE;
          end
        end else begin
          next_state = IDLE;
        end
      end
      CACHE_LOAD: begin
        //meta_out is the stored tag
        if (meta_out[TAG_BITS-1:0] == tag) begin // cache hit b
        // load immediate from cache
          cpu_valid = (cpu_req_write == 4'b0000); // On reads, data is ready
          case (slice)
            2'b00: cpu_data = data_out[0];
            2'b01: cpu_data = data_out[1];
            2'b10: cpu_data = data_out[2];
            2'b11: cpu_data = data_out[3];
          endcase
          next_state = IDLE;
      end else begin // cache miss
          next_state = MEM_LOAD;
          load_counter = 1'b0;
        end 
      end
      MEM_LOAD: begin
          mem_valid = 1'b1;
        //mem_req_addr = {}; 
        mem_write = 1'b0; // read from memory
        if (mem_req_ready) begin
        
          // Loop 4 times to get data from memory
          if (load_counter < 4) begin
            next_state = MEM_LOAD;
            load_counter = load_counter + 1;
          end else begin
            next_state = IDLE;
          end

        end
        
      end
      WRITE: begin
        if (mem_req_ready) begin
          cpu_valid = 1;
          next_state = IDLE;
        end else begin
          mem_write = 1;
          next_state = WRITE;
        end
      end
    endcase
  end
end

assign mem_req_data_mask = mem_data_mask;
assign mem_req_data_valid = mem_data_valid;
assign mem_req_data_bits = mem_data_bits;
assign mem_req_addr = mem_addr;
assign mem_req_valid = mem_valid;
assign mem_req_rw = mem_write;
assign cpu_resp_valid = cpu_valid;
assign cpu_resp_data = cpu_data;
assign cpu_req_ready = cpu_rdy;

// always@(posedge clk) begin
//   case (state)
//     IDLE: begin

//     end
//     CACHE_LOAD: begin

//     end
//     MEM_LOAD: begin

//     end
//     WRITE: begin

//     end
//   endcase
// end


// PARAM_REGISTER#(.N(2)) state_machine (
//   .out(state), 
//   .in(next_state), 
//   .clk(clk),
//   .reset(reset)
//   );

    
endmodule
