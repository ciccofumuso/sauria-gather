`timescale 1ns/1ps

module accel_tlul #(
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
	output logic 								start,
	output logic 								done,
	
	input  tlul_pkg::tl_h2d_t 					tl_i,
	output tlul_pkg::tl_d2h_t 					tl_o,
    
    // Interfaccia AXI4 Read (Verso l'esterno)
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
    output logic                                intg_err_o
);

;

status_register_reg_pkg::status_register_reg2hw_t reg2hw; 

status_register_reg_pkg::status_register_hw2reg_t hw2reg;

assign hw2reg.done.d  = 1'b1;
assign hw2reg.done.de = done;
assign start = reg2hw.start.q;

status_register_reg_top i_status_reg_top (
    .clk_i      (clk),
    .rst_ni     (rst_n),
    .tl_i       (tl_i),
    .tl_o       (tl_o),
    .reg2hw     (reg2hw),
    .hw2reg     (hw2reg),
    .intg_err_o (intg_err_o)
);

accel_top #(
	.AXI_ADDR_W(AXI_ADDR_W),
	.AXI_DATA_SIZE(AXI_DATA_SIZE),
	.NumWords(NumWords),
	.ByteWidth(ByteWidth),
	.FIFO_DEPTH(FIFO_DEPTH),
	.FIFO_THRESHOLD(FIFO_THRESHOLD),
	.NumWords_idx(NumWords_idx),
	.ByteWidth_idx(ByteWidth_idx),
	.NumPorts(NumPorts),
	.Latency(Latency),
	.N_BLOCKS(N_BLOCKS),
	.BUFFER_BIT_ADDR(BUFFER_BIT_ADDR)
) i_accel_top (
	.clk(clk),
	.rst_n(rst_n),
	.start(reg2hw.start.q),
	.is_spmm(reg2hw.is_spmm.q),
	.done(done),
	// Interfaccia AXI4 Read
	.ar_size_in(reg2hw.ar_size.q), 
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
	// Interfaccia AXI4 Write 
	.axi_wr_aw_size_in(reg2hw.aw_size.q),
	.axi_wr_aw_addr_in(reg2hw.axi_wr_aw_addr_in.q), // Esempio indirizzo output
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
	// Registri di stato 
	.ar_addr_dense_matrix(reg2hw.ar_addr_dense_matrix.q),
	.total_len_dense_matrix(reg2hw.total_len_dense_matrix.q),
	.ar_addr_comp_idx(reg2hw.ar_addr_comp_idx.q),
	.total_len_comp_idx(reg2hw.total_len_comp_idx.q),
	.ar_addr_idx(reg2hw.ar_addr_idx.q),
	.total_len_idx(reg2hw.total_len_idx.q),
	.status_reg_n_row(reg2hw.status_reg_n_row.q)
);


endmodule
