`ifndef SYSTOLIC_ARRAY_REFERENCE_DUT_SV
`define SYSTOLIC_ARRAY_REFERENCE_DUT_SV

// Behavioral reference DUT used to smoke-test the verification environment.
// Replace this module with the real RTL while keeping the same wrapper ports.
module systolic_array_reference_dut #(
  parameter int DATA_WIDTH = 8,
  parameter int ARRAY_SIZE = 4,
  parameter int ACC_WIDTH  = (2 * DATA_WIDTH) + $clog2(ARRAY_SIZE),
  parameter int RESULT_FIFO_DEPTH = 32
) (
  input  logic clk,
  input  logic rst_n,
  input  logic in_valid,
  output logic in_ready,
  input  logic signed [DATA_WIDTH-1:0] a_data [ARRAY_SIZE],
  input  logic signed [DATA_WIDTH-1:0] b_data [ARRAY_SIZE],
  output logic out_valid,
  input  logic out_ready,
  output logic signed [ACC_WIDTH-1:0] c_data [ARRAY_SIZE][ARRAY_SIZE],
  output logic busy,
  output logic done
);

  typedef logic signed [DATA_WIDTH-1:0] data_t;
  typedef logic signed [ACC_WIDTH-1:0]  acc_t;

  // Operand pipelines model the systolic data movement: A moves right and B moves down.
  data_t a_pipe [ARRAY_SIZE][ARRAY_SIZE];
  data_t b_pipe [ARRAY_SIZE][ARRAY_SIZE];

  // Valid pipelines travel with the operands so each PE only accumulates real samples.
  logic  a_valid_pipe [ARRAY_SIZE][ARRAY_SIZE];
  logic  b_valid_pipe [ARRAY_SIZE][ARRAY_SIZE];

  // Each PE keeps its own output-stationary partial sum and MAC count.
  acc_t  psum [ARRAY_SIZE][ARRAY_SIZE];
  int unsigned mac_count [ARRAY_SIZE][ARRAY_SIZE];

  // Holds one completed C matrix while individual PE results are collected.
  acc_t completed_matrix [ARRAY_SIZE][ARRAY_SIZE];
  int unsigned completed_cell_count;

  // Small output FIFO decouples matrix completion from downstream out_ready backpressure.
  acc_t result_fifo [RESULT_FIFO_DEPTH][ARRAY_SIZE][ARRAY_SIZE];
  int unsigned fifo_wr_ptr;
  int unsigned fifo_rd_ptr;
  int unsigned fifo_count;

  // Counts accepted input beats within the 2*N-1 diagonal wavefront transaction.
  int unsigned sample_index;

  assign in_ready = (fifo_count < RESULT_FIFO_DEPTH - 2);
  assign busy = (sample_index != 0) || (completed_cell_count != 0) || (fifo_count != 0) || out_valid;
  assign done = out_valid && out_ready;

  function automatic bit boundary_lane_active(input int beat, input int lane);
    int operand_index;

    operand_index = beat - lane;
    return (operand_index >= 0) && (operand_index < ARRAY_SIZE);
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sample_index <= 0;
      out_valid <= 1'b0;
      c_data <= '{default:'0};
      a_pipe <= '{default:'0};
      b_pipe <= '{default:'0};
      a_valid_pipe <= '{default:'0};
      b_valid_pipe <= '{default:'0};
      psum <= '{default:'0};
      mac_count <= '{default:0};
      completed_matrix <= '{default:'0};
      completed_cell_count <= 0;
      result_fifo <= '{default:'0};
      fifo_wr_ptr <= 0;
      fifo_rd_ptr <= 0;
      fifo_count <= 0;
    end else begin
      int unsigned completed_cell_count_next;
      int unsigned fifo_count_next;
      int unsigned fifo_wr_ptr_next;
      int unsigned fifo_rd_ptr_next;
      int unsigned completed_this_cycle;

      completed_cell_count_next = completed_cell_count;
      fifo_count_next = fifo_count;
      fifo_wr_ptr_next = fifo_wr_ptr;
      fifo_rd_ptr_next = fifo_rd_ptr;
      completed_this_cycle = 0;

      if (out_valid && out_ready) begin
        if (fifo_count_next != 0) begin
          c_data <= result_fifo[fifo_rd_ptr_next];
          out_valid <= 1'b1;
          fifo_rd_ptr_next = (fifo_rd_ptr_next + 1) % RESULT_FIFO_DEPTH;
          fifo_count_next--;
        end else begin
          out_valid <= 1'b0;
        end
      end else if (!out_valid && (fifo_count_next != 0)) begin
        c_data <= result_fifo[fifo_rd_ptr_next];
        out_valid <= 1'b1;
        fifo_rd_ptr_next = (fifo_rd_ptr_next + 1) % RESULT_FIFO_DEPTH;
        fifo_count_next--;
      end

      if (completed_cell_count_next == ARRAY_SIZE * ARRAY_SIZE) begin
        result_fifo[fifo_wr_ptr_next] <= completed_matrix;
        fifo_wr_ptr_next = (fifo_wr_ptr_next + 1) % RESULT_FIFO_DEPTH;
        fifo_count_next++;
        completed_cell_count_next = 0;
      end

      if (in_valid && in_ready) begin
        if (sample_index == (2 * ARRAY_SIZE) - 2) begin
          sample_index   <= 0;
        end else begin
          sample_index <= sample_index + 1;
        end
      end

      for (int row = 0; row < ARRAY_SIZE; row++) begin
        for (int col = 0; col < ARRAY_SIZE; col++) begin
          data_t a_in;
          data_t b_in;
          logic a_valid_in;
          logic b_valid_in;
          acc_t next_psum;

          if (col == 0) begin
            a_in = a_data[row];
            a_valid_in = in_valid && in_ready && boundary_lane_active(sample_index, row);
          end else begin
            a_in = a_pipe[row][col - 1];
            a_valid_in = a_valid_pipe[row][col - 1];
          end

          if (row == 0) begin
            b_in = b_data[col];
            b_valid_in = in_valid && in_ready && boundary_lane_active(sample_index, col);
          end else begin
            b_in = b_pipe[row - 1][col];
            b_valid_in = b_valid_pipe[row - 1][col];
          end

          a_pipe[row][col] <= a_in;
          b_pipe[row][col] <= b_in;
          a_valid_pipe[row][col] <= a_valid_in;
          b_valid_pipe[row][col] <= b_valid_in;

          if (a_valid_in && b_valid_in) begin
            next_psum = psum[row][col] + (acc_t'(a_in) * acc_t'(b_in));
            if (mac_count[row][col] == ARRAY_SIZE - 1) begin
              completed_matrix[row][col] <= next_psum;
              psum[row][col] <= '0;
              mac_count[row][col] <= 0;
              completed_this_cycle++;
            end else begin
              psum[row][col] <= next_psum;
              mac_count[row][col] <= mac_count[row][col] + 1;
            end
          end
        end
      end

      if (completed_this_cycle != 0) begin
        completed_cell_count_next += completed_this_cycle;
      end

      if (completed_cell_count_next == ARRAY_SIZE * ARRAY_SIZE) begin
        bit defer_push;

        defer_push = 1'b0;
        for (int row = 0; row < ARRAY_SIZE; row++) begin
          for (int col = 0; col < ARRAY_SIZE; col++) begin
            if (mac_count[row][col] == ARRAY_SIZE - 1) begin
              defer_push = 1'b1;
            end
          end
        end

        if (!defer_push) begin
          result_fifo[fifo_wr_ptr_next] <= completed_matrix;
          fifo_wr_ptr_next = (fifo_wr_ptr_next + 1) % RESULT_FIFO_DEPTH;
          fifo_count_next++;
          completed_cell_count_next = 0;
        end
      end

      fifo_count <= fifo_count_next;
      fifo_wr_ptr <= fifo_wr_ptr_next;
      fifo_rd_ptr <= fifo_rd_ptr_next;
      completed_cell_count <= completed_cell_count_next;
    end
  end

endmodule

`endif