`timescale 1ns/1ps
module accel_top #(
    parameter int unsigned AXI_ADDR_W        = 32,
    parameter int unsigned AXI_DATA_SIZE     = 64,
    parameter int FIFO_DEPTH                 = 4,
    parameter int FIFO_THRESHOLD             = 2,
    parameter int unsigned NumWords_idx      = 128,
    parameter int unsigned ByteWidth_idx     = 16,
    parameter int unsigned NumWords          = 128,
    parameter int unsigned ByteWidth         = 32,
    parameter int unsigned NumPorts          = 1,
    parameter int unsigned Latency           = 1,
    parameter int N_BLOCKS                   = 8,
    parameter int BUFFER_BIT_ADDR            = 3
)(
    input  logic                                clk,
    input  logic                                rst_n,
    input  logic                                start,
    input  logic                                is_spmm,
	output logic								done,
    
    // Interfaccia AXI4 Read (Verso l'esterno)
    input  logic [2:0]                          ar_size_in,
    output logic [AXI_ADDR_W-1:0]               ar_addr_out,
    output logic [7:0]                          ar_len_out,
    output logic [2:0]                          ar_size_out,
    input  logic [AXI_DATA_SIZE-1:0]            data_in,
    output logic                                ar_valid,
    input  logic                                ar_ready,
    output logic [1:0]                          ar_burst,
    input  logic                                r_valid,
    output logic                                r_ready,
    input  logic                                r_last,

    // Interfaccia AXI4 Write (Verso l'esterno)
    input  logic [2:0]                          axi_wr_aw_size_in,
    input  logic [AXI_ADDR_W-1:0]               axi_wr_aw_addr_in,
    output logic [AXI_ADDR_W-1:0]               aw_addr,
    output logic [7:0]                          aw_len,
    output logic [2:0]                          aw_size,
    output logic [1:0]                          aw_burst,
    output logic                                aw_valid,
    input  logic                                aw_ready,
    output logic [AXI_DATA_SIZE-1:0]            w_data,
    output logic [AXI_DATA_SIZE/8-1:0]          w_strb,
    output logic                                w_valid,
    input  logic                                w_ready,
    output logic                                w_last,
    input  logic [1:0]                          b_resp,
    input  logic                                b_valid,
    output logic                                b_ready,

    // Parametri e Impostazioni (Status Registers)
	input 	logic [AXI_ADDR_W-1:0] 				ar_addr_dense_matrix,
	input 	logic [AXI_ADDR_W-1:0] 				total_len_dense_matrix,
	input 	logic [AXI_ADDR_W-1:0] 				ar_addr_comp_idx,
	input 	logic [AXI_ADDR_W-1:0] 				total_len_comp_idx,
	input 	logic [AXI_ADDR_W-1:0] 				ar_addr_idx,
	input 	logic [AXI_ADDR_W-1:0] 				total_len_idx,
    input   logic [AXI_ADDR_W-1:0] 				status_reg_n_row
);

// Segnali Interni di Collegamento

// Segnali AXI Read
logic axi_rd_rst_n, axi_rd_start, axi_rd_available, axi_rd_done, axi_rd_done_rst_n;
logic rst_n_multi_burst, tc_wr_mem, wr_index, tc_rd_index;

// Segnali FIFO 
logic fifo_r_en, fifo_rst_n, fifo_empty;

// Segnali Memory 
logic mem_rst_n, mem_wr_en, mem_idx_wr_en, mem_idx_mem_en;

// Segnali Buffer 
logic buffer_rst_n, buffer_data_in_valid, buffer_data_in_last, buffer_wait_wr, buffer_data_out_valid;

// Segnali AXI Write 
logic axi_wr_rst_n, axi_wr_done, axi_wr_done_rst_n;

// Segnali Address Manager 
logic cnt_row_en, cnt_row_rst_n, rd_b_mem;
logic piso_rst_n, piso_load, piso_shift_en, piso_empty;
logic comp_idx_ptr_old_en, comp_idx_ptr_old_rst_n, mux_comp_idx_sel;
logic cnt_nnzs_en, cnt_nnzs_rst_n, last_index, end_rd_index, tc_nnzs;
logic next_row_empty;
logic cnt_stage_en, cnt_stage_rst_n;
logic cnt_comp_idx_en, cnt_comp_idx_rst_n;

// --- Istanza della CU ---
accel_cu i_accel_cu (
	.clk(clk), .rst_n(rst_n), .start(start),
	.* 
);

// --- Istanza del DP ---
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
	.BUFFER_BIT_ADDR(BUFFER_BIT_ADDR)
) i_accel_dp (
	.clk(clk),
	.rst_n(rst_n),
	.is_spmm(is_spmm),
	// AXI4 Read
	.axi_rd_rst_n(axi_rd_rst_n),
	.axi_rd_start(axi_rd_start),
	.ar_size_in(ar_size_in),
	.ar_addr_dense_matrix(ar_addr_dense_matrix),
	.total_len_dense_matrix(total_len_dense_matrix),
	.ar_addr_comp_idx(ar_addr_comp_idx),
	.total_len_comp_idx(total_len_comp_idx),
	.ar_addr_idx(ar_addr_idx),
	.total_len_idx(total_len_idx),
	.rst_n_multi_burst(rst_n_multi_burst),
	.tc_wr_mem(tc_wr_mem),   
	.wr_index(wr_index),	 
	.tc_rd_index(tc_rd_index), 
	.axi_rd_available(axi_rd_available),
	.axi_rd_done(axi_rd_done),
	.axi_rd_done_rst_n(axi_rd_done_rst_n),
	.ar_addr_out(ar_addr_out),
	.ar_len_out(ar_len_out),
	.ar_size_out(ar_size_out),
	.data_in(data_in),
	.ar_valid(ar_valid),
	.ar_ready(ar_ready),
	.ar_burst(ar_burst),
	.r_valid(r_valid),
	.r_ready(r_ready),
	.r_last(r_last),
	// FIFO
	.fifo_r_en(fifo_r_en),
	.fifo_rst_n(fifo_rst_n),
	.fifo_empty(fifo_empty),
	// Memory
	.mem_rst_n(mem_rst_n),
	.mem_wr_en(mem_wr_en),
	.mem_idx_wr_en(mem_idx_wr_en),
	.mem_idx_mem_en(mem_idx_mem_en), 
	// Buffer
	.buffer_rst_n(buffer_rst_n),
	.buffer_data_in_valid(buffer_data_in_valid),
	.buffer_data_in_last(buffer_data_in_last),
	.buffer_wait_wr(buffer_wait_wr),
	.buffer_data_out_valid(buffer_data_out_valid),
	// AXI4 Write
	.axi_wr_rst_n(axi_wr_rst_n),
	.axi_wr_done(axi_wr_done),
	.axi_wr_done_rst_n(axi_wr_done_rst_n),
	.axi_wr_aw_size_in(axi_wr_aw_size_in),
	.axi_wr_aw_addr_in(axi_wr_aw_addr_in),
	.aw_addr(aw_addr),
	.aw_len(aw_len),
	.aw_size(aw_size),
	.aw_burst(aw_burst),
	.aw_valid(aw_valid),
	.aw_ready(aw_ready),
	.w_data(w_data),
	.w_strb(w_strb),
	.w_valid(w_valid),
	.w_ready(w_ready),
	.w_last(w_last),
	.b_resp(b_resp),
	.b_valid(b_valid),
	.b_ready(b_ready),
	// Address Manager
	.status_reg_n_row(status_reg_n_row),
	.cnt_row_en(cnt_row_en),
	.cnt_row_rst_n(cnt_row_rst_n),
	.rd_b_mem(rd_b_mem),
	.piso_rst_n(piso_rst_n),
	.piso_load(piso_load),
	.piso_shift_en(piso_shift_en),
	.piso_empty(piso_empty),
	.comp_idx_ptr_old_en(comp_idx_ptr_old_en),
	.comp_idx_ptr_old_rst_n(comp_idx_ptr_old_rst_n),
	.mux_comp_idx_sel(mux_comp_idx_sel),
	.cnt_nnzs_en(cnt_nnzs_en),
	.cnt_nnzs_rst_n(cnt_nnzs_rst_n),
	.last_index(last_index),
	.end_rd_index(end_rd_index),
	.tc_nnzs(tc_nnzs),
	.next_row_empty(next_row_empty),
	.cnt_stage_en(cnt_stage_en),
	.cnt_stage_rst_n(cnt_stage_rst_n),
	.cnt_comp_idx_en(cnt_comp_idx_en),
	.cnt_comp_idx_rst_n(cnt_comp_idx_rst_n)
);

endmodule
