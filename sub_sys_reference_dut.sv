`ifndef SUB_SYS_REFERENCE_DUT_SV
`define SUB_SYS_REFERENCE_DUT_SV

// Behavioral subsystem wrapper used for Layer2 verification.
// The wrapper accepts packed FIFO/bus beats and reuses the systolic-array module internally.
module sub_sys_reference_dut #(
  parameter int DATA_WIDTH = 8,
  parameter int ARRAY_SIZE = 4,
  parameter int BUS_WIDTH  = 2 * ARRAY_SIZE * DATA_WIDTH,
  parameter int ACC_WIDTH  = (2 * DATA_WIDTH) + $clog2(ARRAY_SIZE),
  parameter int INPUT_FIFO_DEPTH = 8
) (
  input  logic sys_clk,
  input  logic array_clk,
  input  logic sys_rst_n,
  input  logic array_rst_n,
  input  logic in_valid,
  input  logic wr_fifo,
  input  logic rd_fifo,
  input  logic out_ready,
  input  logic [BUS_WIDTH-1:0] bus_din,
  input  logic [$clog2(ARRAY_SIZE+1)-1:0] m_minus_one,
  output logic fifo_full,
  output logic fifo_empty,
  output logic [BUS_WIDTH-1:0] bus_dout,
  output logic out_valid,
  output logic signed [ACC_WIDTH-1:0] c_data [ARRAY_SIZE][ARRAY_SIZE]
);

  typedef logic signed [DATA_WIDTH-1:0] data_t;

  logic [BUS_WIDTH-1:0] input_fifo [INPUT_FIFO_DEPTH];
  int unsigned fifo_wr_ptr;
  int unsigned fifo_rd_ptr;
  int unsigned fifo_count;

  logic systolic_in_valid;
  logic systolic_in_ready;
  data_t systolic_a_data [ARRAY_SIZE];
  data_t systolic_b_data [ARRAY_SIZE];
  logic systolic_busy;
  logic systolic_done;

  assign fifo_full = (fifo_count == INPUT_FIFO_DEPTH);
  assign fifo_empty = 1'b1;
  assign bus_dout = '0;
  assign systolic_in_valid = (fifo_count != 0);

  // The reference subsystem model keeps the two clock ports visible, but uses sys_clk for
  // the packed-bus adapter and the internal array to keep this behavioral wrapper deterministic.
  systolic_array_reference_dut #(
    .DATA_WIDTH(DATA_WIDTH),
    .ARRAY_SIZE(ARRAY_SIZE),
    .ACC_WIDTH(ACC_WIDTH)
  ) systolic_core (
    .clk(sys_clk),
    .rst_n(sys_rst_n && array_rst_n),
    .in_valid(systolic_in_valid),
    .in_ready(systolic_in_ready),
    .a_data(systolic_a_data),
    .b_data(systolic_b_data),
    .out_valid(out_valid),
    .out_ready(out_ready),
    .c_data(c_data),
    .busy(systolic_busy),
    .done(systolic_done)
  );

  always_comb begin
    for (int lane = 0; lane < ARRAY_SIZE; lane++) begin
      systolic_a_data[lane] = data_t'(input_fifo[fifo_rd_ptr][(lane * DATA_WIDTH) +: DATA_WIDTH]);
      systolic_b_data[lane] = data_t'(input_fifo[fifo_rd_ptr][((ARRAY_SIZE + lane) * DATA_WIDTH) +: DATA_WIDTH]);
    end
  end

  always_ff @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      input_fifo <= '{default:'0};
      fifo_wr_ptr <= 0;
      fifo_rd_ptr <= 0;
      fifo_count <= 0;
    end else begin
      int unsigned fifo_wr_ptr_next;
      int unsigned fifo_rd_ptr_next;
      int unsigned fifo_count_next;

      fifo_wr_ptr_next = fifo_wr_ptr;
      fifo_rd_ptr_next = fifo_rd_ptr;
      fifo_count_next = fifo_count;

      if (systolic_in_valid && systolic_in_ready) begin
        fifo_rd_ptr_next = (fifo_rd_ptr_next + 1) % INPUT_FIFO_DEPTH;
        fifo_count_next--;
      end

      if (in_valid && wr_fifo && !fifo_full) begin
        input_fifo[fifo_wr_ptr_next] <= bus_din;
        fifo_wr_ptr_next = (fifo_wr_ptr_next + 1) % INPUT_FIFO_DEPTH;
        fifo_count_next++;
      end

      fifo_wr_ptr <= fifo_wr_ptr_next;
      fifo_rd_ptr <= fifo_rd_ptr_next;
      fifo_count <= fifo_count_next;
    end
  end

endmodule

`endif