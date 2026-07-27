`timescale 1ns/1ps

module accel_top #(
    parameter int unsigned AXI_ADDR_W = 32,
    parameter int unsigned AXI_DATA_SIZE = 64,
    parameter int FIFO_DEPTH = 4,
    parameter int FIFO_THRESHOLD = 2,
    parameter int unsigned NumWords_idx = 128,
    parameter int unsigned ByteWidth_idx = 16,
    parameter int unsigned NumWords = 128,
    parameter int unsigned ByteWidth = 32,
    parameter int unsigned NumPorts = 1,
    parameter int unsigned Latency = 1,
    parameter int N_BLOCKS = 8,
    parameter int BUFFER_BIT_ADDR = 3,
    parameter int PTR_FIFO_DEPTH = 4,
    parameter int RESP_DEPTH = Latency + 2
) (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic is_spmm,
    output logic done,

    // AXI4 read.
    input  logic [2:0] ar_size_in,
    output logic [AXI_ADDR_W-1:0] ar_addr_out,
    output logic [7:0] ar_len_out,
    output logic [2:0] ar_size_out,
    input  logic [AXI_DATA_SIZE-1:0] data_in,
    output logic ar_valid,
    input  logic ar_ready,
    output logic [1:0] ar_burst,
    input  logic r_valid,
    output logic r_ready,
    input  logic r_last,

    // AXI4 write.
    input  logic [2:0] axi_wr_aw_size_in,
    input  logic [AXI_ADDR_W-1:0] axi_wr_aw_addr_in,
    output logic [AXI_ADDR_W-1:0] aw_addr,
    output logic [7:0] aw_len,
    output logic [2:0] aw_size,
    output logic [1:0] aw_burst,
    output logic aw_valid,
    input  logic aw_ready,
    output logic [AXI_DATA_SIZE-1:0] w_data,
    output logic [AXI_DATA_SIZE/8-1:0] w_strb,
    output logic w_valid,
    input  logic w_ready,
    output logic w_last,
    input  logic [1:0] b_resp,
    input  logic b_valid,
    output logic b_ready,

    // Status registers.
    input  logic [AXI_ADDR_W-1:0] ar_addr_dense_matrix,
    input  logic [AXI_ADDR_W-1:0] total_len_dense_matrix,
    input  logic [AXI_ADDR_W-1:0] ar_addr_comp_idx,
    input  logic [AXI_ADDR_W-1:0] total_len_comp_idx,
    input  logic [AXI_ADDR_W-1:0] ar_addr_idx,
    input  logic [AXI_ADDR_W-1:0] total_len_idx,
    input  logic [AXI_ADDR_W-1:0] status_reg_n_row
);

logic axi_rd_rst_n;
logic axi_rd_start;
logic axi_rd_available;
logic axi_rd_done;
logic axi_rd_done_rst_n;
logic rst_n_multi_burst;
logic tc_wr_mem;
logic wr_index;
logic tc_rd_index;

logic fifo_r_en;
logic fifo_rst_n;
logic fifo_empty;


/* verilator lint_off SYNCASYNCNET */
logic mem_rst_n;
logic mem_wr_en;
logic mem_idx_wr_en;
logic dense_rd_req;
logic dense_req_ready;
logic dense_pipe_empty;

logic buffer_rst_n;
logic buffer_data_in_last;
logic buffer_data_out_valid;
logic unused_buffer_wait_wr;

logic axi_wr_rst_n;
logic axi_wr_done;
logic axi_wr_done_rst_n;

logic cnt_row_en;
logic cnt_row_rst_n;
logic rd_b_mem;

logic piso_rst_n;
logic piso_load;
logic piso_shift_en;
logic piso_empty;

logic ptr_stream_enable;
logic ptr_stream_rst_n;
logic segment_advance;
logic segment_valid;
logic segment_done;
logic stream_end;
logic nnz_limit_reached;
logic end_rd_index;
/* verilator lint_on SYNCASYNCNET */
logic unused_next_row_empty;

logic cnt_stage_en;
logic cnt_stage_rst_n;
logic cnt_comp_idx_en;
logic cnt_comp_idx_rst_n;

accel_cu i_accel_cu (
    .clk,
    .rst_n,
    .start,
    .done,
    .axi_rd_rst_n,
    .axi_rd_start,
    .axi_rd_available,
    .axi_rd_done,
    .axi_rd_done_rst_n,
    .fifo_r_en,
    .fifo_rst_n,
    .fifo_empty,
    .mem_rst_n,
    .mem_wr_en,
    .mem_idx_wr_en,
    .dense_rd_req,
    .dense_req_ready,
    .dense_pipe_empty,
    .buffer_rst_n,
    .buffer_data_in_last,
    .buffer_data_out_valid,
    .axi_wr_rst_n,
    .axi_wr_done,
    .axi_wr_done_rst_n,
    .rst_n_multi_burst,
    .tc_wr_mem,
    .wr_index,
    .tc_rd_index,
    .cnt_row_en,
    .cnt_row_rst_n,
    .rd_b_mem,
    .piso_rst_n,
    .piso_load,
    .piso_shift_en,
    .piso_empty,
    .ptr_stream_enable,
    .ptr_stream_rst_n,
    .segment_advance,
    .segment_valid,
    .segment_done,
    .stream_end,
    .nnz_limit_reached,
    .end_rd_index,
    .cnt_stage_en,
    .cnt_stage_rst_n,
    .cnt_comp_idx_en,
    .cnt_comp_idx_rst_n
);

accel_dp #(
    .AXI_ADDR_W(AXI_ADDR_W),
    .AXI_DATA_SIZE(AXI_DATA_SIZE),
    .FIFO_DEPTH(FIFO_DEPTH),
    .FIFO_THRESHOLD(FIFO_THRESHOLD),
    .NumWords_idx(NumWords_idx),
    .ByteWidth_idx(ByteWidth_idx),
    .NumWords(NumWords),
    .ByteWidth(ByteWidth),
    .NumPorts(NumPorts),
    .Latency(Latency),
    .N_BLOCKS(N_BLOCKS),
    .BUFFER_BIT_ADDR(BUFFER_BIT_ADDR),
    .PTR_FIFO_DEPTH(PTR_FIFO_DEPTH),
    .RESP_DEPTH(RESP_DEPTH)
) i_accel_dp (
    .clk,
    .rst_n,
    .is_spmm,
    .axi_rd_rst_n,
    .axi_rd_start,
    .ar_size_in,
    .ar_addr_dense_matrix,
    .total_len_dense_matrix,
    .ar_addr_comp_idx,
    .total_len_comp_idx,
    .ar_addr_idx,
    .total_len_idx,
    .rst_n_multi_burst,
    .axi_rd_available,
    .axi_rd_done,
    .axi_rd_done_rst_n,
    .ar_addr_out,
    .ar_len_out,
    .ar_size_out,
    .data_in,
    .ar_valid,
    .ar_ready,
    .ar_burst,
    .r_valid,
    .r_ready,
    .r_last,
    .fifo_r_en,
    .fifo_rst_n,
    .fifo_empty,
    .mem_rst_n,
    .mem_wr_en,
    .mem_idx_wr_en,
    .dense_rd_req,
    .dense_req_ready,
    .dense_pipe_empty,
    .buffer_rst_n,
    .buffer_data_in_last,
    .buffer_wait_wr(unused_buffer_wait_wr),
    .buffer_data_out_valid,
    .axi_wr_rst_n,
    .axi_wr_done,
    .axi_wr_done_rst_n,
    .axi_wr_aw_size_in,
    .axi_wr_aw_addr_in,
    .aw_addr,
    .aw_len,
    .aw_size,
    .aw_burst,
    .aw_valid,
    .aw_ready,
    .w_data,
    .w_strb,
    .w_valid,
    .w_ready,
    .w_last,
    .b_resp,
    .b_valid,
    .b_ready,
    .tc_wr_mem,
    .wr_index,
    .tc_rd_index,
    .status_reg_n_row,
    .cnt_row_en,
    .cnt_row_rst_n,
    .rd_b_mem,
    .piso_rst_n,
    .piso_load,
    .piso_shift_en,
    .piso_empty,
    .ptr_stream_enable,
    .ptr_stream_rst_n,
    .segment_advance,
    .segment_valid,
    .segment_done,
    .stream_end,
    .nnz_limit_reached,
    .end_rd_index,
    .next_row_empty(unused_next_row_empty),
    .cnt_stage_en,
    .cnt_stage_rst_n,
    .cnt_comp_idx_en,
    .cnt_comp_idx_rst_n
);

endmodule
