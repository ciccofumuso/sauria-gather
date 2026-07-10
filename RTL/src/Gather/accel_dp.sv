module accel_dp #(
	parameter int unsigned AXI_ADDR_W 		= 32,
	parameter int unsigned AXI_DATA_SIZE 	= 64,
	parameter int FIFO_DEPTH 				= 16,    // Numero di locazioni
	parameter int FIFO_THRESHOLD 			= 8,	// Soglia impostata a metà
	parameter int unsigned NumWords_idx		= 16,   // Numero di word totali nella SRAM degli indici compressi
	parameter int unsigned ByteWidth_idx 	= 32,    // Larghezza del singolo byte di indici compressi 
    parameter int unsigned NumWords 		= 16,   // Numero di word totali nella SRAM singola
	parameter int unsigned ByteWidth 		= 32,    // Larghezza del singolo byte 
	parameter int unsigned NumPorts			= 1,    // Numero di porte
	parameter int unsigned Latency 			= 1,    // Latenza di lettura 
	parameter int N_BLOCKS 					= 8,
	parameter int BUFFER_BIT_ADDR 			= 4
	// PARAMETRI DIPENDENTI
)(
	input  	logic 				clk, rst_n,
	input 	logic				is_spmm,
	// INTERFACCIA AXI4 READ
	input  	logic 								axi_rd_rst_n, // From CU
	input  	logic              					axi_rd_start,  		// Richiesta di lettura valida (From CU)
	input	logic [2:0]        					ar_size_in,
	input 	logic [AXI_ADDR_W-1:0] 				ar_addr_dense_matrix,
	input 	logic [AXI_ADDR_W-1:0] 				total_len_dense_matrix,
	input 	logic [AXI_ADDR_W-1:0] 				ar_addr_comp_idx,
	input 	logic [AXI_ADDR_W-1:0] 				total_len_comp_idx,
	input 	logic [AXI_ADDR_W-1:0] 				ar_addr_idx,
	input 	logic [AXI_ADDR_W-1:0] 				total_len_idx,
	input	logic								rst_n_multi_burst,
    output 	logic              					axi_rd_available,  	// Disponibile ad accettare una nuova richiesta
	output 	logic              					axi_rd_done,		// Ultimo dato arrivato
	input	logic								axi_rd_done_rst_n, // From CU
	// Interfaccia con l'esterno		
	output 	logic [AXI_ADDR_W-1:0] 				ar_addr_out,
	output 	logic [7:0]        					ar_len_out,
	output 	logic [2:0]        					ar_size_out,
	input 	logic [AXI_DATA_SIZE-1:0] 			data_in,
	// Canale AR		
    output 	logic              					ar_valid,
    input  	logic              					ar_ready,
    output 	logic [1:0]        					ar_burst,
    // Canale R		
    input  	logic              					r_valid,
    output 	logic              					r_ready,
    input  	logic              					r_last,
			
	// INTERFACCIA FIFO		
	input  	logic                   			fifo_r_en, // From CU
	input	logic                   			fifo_rst_n,// From CU     
    output 	logic                   			fifo_empty,     
			
	// INTERFACCIA MEMORIA		
	input	logic                   			mem_rst_n, // From CU
	input	logic 								mem_wr_en, // From CU
	input	logic 								mem_idx_wr_en, // From CU
	input	logic 								mem_idx_mem_en, // From CU
			
	// INTERFACCIA BUFFER 		
	input	logic                   			buffer_rst_n, // From CU
	input  	logic                				buffer_data_in_valid, // From CU
    input  	logic                				buffer_data_in_last, // From CU
	output 	logic                				buffer_wait_wr,
	output	logic								buffer_data_out_valid,
			
	// INTERFACCIA AXI4 WRITE		
	input  	logic 								axi_wr_rst_n,      // From CU
	output 	logic              					axi_wr_done,		
	input	logic								axi_wr_done_rst_n, // From CU
	input	logic [2:0]        					axi_wr_aw_size_in, // From CU
	input	logic [AXI_ADDR_W-1:0] 				axi_wr_aw_addr_in, // From CU
	// Interfaccia con l'esterno		
    output 	logic [AXI_ADDR_W-1:0] 				aw_addr,
    output 	logic [7:0]        					aw_len,
    output 	logic [2:0]        					aw_size,
    output 	logic [1:0]        					aw_burst,
    output 	logic              					aw_valid,
    input  	logic              					aw_ready,
	// Canale W		
    output 	logic [AXI_DATA_SIZE-1:0] 			w_data,
    output 	logic [AXI_DATA_SIZE/8-1:0] 		w_strb,
    output 	logic              					w_valid,
    input  	logic              					w_ready,
    output 	logic              					w_last,
	// Canale B		
    input  	logic [1:0]        					b_resp,
    input  	logic              					b_valid,
    output 	logic              					b_ready,
	
	// INTERFACCIA ADDRESS MANAGER
	// Segnali di stato delle impostazioni del driver
	output 	logic 								tc_wr_mem,   // Fine scrittura della memoria densa
	output 	logic 								wr_index,	 // Scritura memoria degli indici compressi
	output 	logic 								tc_rd_index, // Lettura degli indici
	// Gestione indirizzi per la memoria a blocchi
	input	logic[AXI_ADDR_W-1:0]							status_reg_n_row,
	input 	logic											cnt_row_en,cnt_row_rst_n, // From CU
	input   logic											rd_b_mem, // From CU
	// Segnali PISO
	input	logic										piso_rst_n,piso_load,piso_shift_en, // From CU
	output	logic										piso_empty,
	input	logic										comp_idx_ptr_old_en,comp_idx_ptr_old_rst_n, // From CU
	input	logic										mux_comp_idx_sel, // From CU
	input 	logic										cnt_nnzs_en,cnt_nnzs_rst_n, // From CU
	output 	logic 										last_index, end_rd_index,
	output	logic										tc_nnzs,
	output  logic                               		next_row_empty,
	// Gestione blocco di SRAM per spMM
	input 	logic										cnt_stage_en,cnt_stage_rst_n,	// Si incrementa quando tc_nnzs = 1
	// Indicizzazione memoria degli indici
	input 	logic										cnt_comp_idx_en,cnt_comp_idx_rst_n  // From CU
);


// INTERFACCIA AXI4 READ


logic [AXI_DATA_SIZE-1:0] 	axi_rd_data_out;
logic						rd_done;
logic              			axi_rd_is_valid_data;  			// Un dato del burst è pronto per l'acceleratore

// SEGNALI FIFO
logic 						fifo_rd;
logic [AXI_DATA_SIZE-1:0]  	fifo_r_data;
logic 						fifo_threshold;

logic	piso_to_fifo_r_en;

// SEGNALI MEMORIA
// Memoria matrice densa
logic [$clog2(NumWords)-1:0]				mem_addr;
logic [$clog2(AXI_DATA_SIZE/ByteWidth)-1:0]	mem_byte_sel;
logic [$clog2(N_BLOCKS)-1:0]				mem_mem_en;
logic [ByteWidth-1:0]						mem_data_out;
// Memoria indici compressi
logic [$clog2(NumWords_idx)-1:0]			mem_idx_addr;
logic [$clog2(AXI_DATA_SIZE/ByteWidth_idx)-1:0]	mem_idx_byte_sel;
logic [ByteWidth_idx-1:0]					mem_idx_data_out;

// SEGNALI DOUBLE BUFFER
logic                		buffer_out_ready;
logic [AXI_DATA_SIZE-1:0]	buffer_data_out;
logic                		buffer_out_valid;
logic [7:0] 				buffer_rd_len;
logic [AXI_DATA_SIZE/8-1:0] buffer_wstrb_last;

// INTERFACCIA AXI4 WRITE
logic						wr_done;



address_manager #(
	.AXI_ADDR_W(AXI_ADDR_W),
	.NumWords(NumWords),
	.N_BLOCK_SRAM(N_BLOCKS),
	.WORD_SIZE(ByteWidth),
	.AXI_DATA_SIZE(AXI_DATA_SIZE),
	.NumWords_idx(NumWords_idx),   // Numero di word totali nella SRAM degli indici compressi
	.WORD_SIZE_IDX(ByteWidth_idx)   // Larghezza del singolo byte di indici compressi 
)i_address_manager(
	.clk(clk),
	.status_reg_n_row,
	.cnt_row_en,.cnt_row_rst_n,
	.rd_b_mem,
	.piso_rst_n,.load(piso_load),.shift_en(piso_shift_en),
	.empty(piso_empty),
	.is_spmm,
	.piso_data_in(fifo_r_data),
	.fifo_empty(fifo_empty),   
    .fifo_r_en(piso_to_fifo_r_en),    
	.byte_b_mem_out(mem_byte_sel),
	.addr_b_mem_out(mem_addr),
	.en_b_mem_out(mem_mem_en),			
	.comp_idx_in(mem_idx_data_out),                                 
	.comp_idx_ptr_old_en,.comp_idx_ptr_old_rst_n,  
	.mux_comp_idx_sel,
	.cnt_nnzs_en,.cnt_nnzs_rst_n,
	.last_index,.end_rd_index,
	.tc_nnzs,
	.next_row_empty(next_row_empty),
	.cnt_stage_en,.cnt_stage_rst_n,	
	.cnt_comp_idx_en,.cnt_comp_idx_rst_n,
	.cnt_comp_idx_out({mem_idx_addr,mem_idx_byte_sel})
);

assign fifo_rd = (rd_b_mem == 1'b1) ? piso_to_fifo_r_en : fifo_r_en;


// IMPLEMENTAZIONE AXI4 WRITE
axi4_master_write #(
    .ADDR_W(AXI_ADDR_W),
    .DATA_W(AXI_DATA_SIZE)
)axi4_write(
    .clk_i(clk),
    .rst_ni(axi_wr_rst_n),
    .start(buffer_out_valid),
    .addr_in(axi_wr_aw_addr_in),
    .len_in(buffer_rd_len),
    .size_in(axi_wr_aw_size_in),
    .data_in(buffer_data_out),     // Dato da scrivere (es. da una FIFO)
    .w_strb_last(buffer_wstrb_last),
	.next_data_out(buffer_out_ready), // Richiesta nuovo dato per il burst
    .done(wr_done),
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
    .b_ready
);

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		axi_wr_done <= 1'b0;
	end else begin
		if (!axi_wr_done_rst_n) begin // Diventa un clear sincrono controllato dalla CU
			axi_wr_done <= 1'b0;
		end else if (wr_done) begin
			axi_wr_done <= 1'b1;
		end
	end
end

assign buffer_data_out_valid = buffer_out_valid;

// IMPLEMENTAZIONE BUFFER PING-PONG
double_buffer_top #(
    .N_in(ByteWidth),
    .N_out(AXI_DATA_SIZE),
    .BIT_ADDR(BUFFER_BIT_ADDR)
)i_double_buffer_top(
    .clk(clk),
    .rst_n(buffer_rst_n),
    .data_in(mem_data_out),
    .data_in_valid(buffer_data_in_valid),
    .data_in_last(buffer_data_in_last),
    .wait_wr(buffer_wait_wr),
    .out_ready(buffer_out_ready),
    .data_out(buffer_data_out),
    .out_valid(buffer_out_valid),
	.wstrb_last(buffer_wstrb_last),
    .rd_len(buffer_rd_len)
);


// IMPLEMENTAZIONE MEMORIA
memory_unit #(
	.NumWords_idx(NumWords_idx),   	// Numero di word totali nella SRAM degli indici compressi
	.ByteWidth_idx(ByteWidth_idx),  // Larghezza del singolo byte di indici compressi 
    .NumWords(NumWords),   			// Numero di word totali nella SRAM singola
	.DataWidth(AXI_DATA_SIZE),   	// Larghezza del bus dati (64 bit)
	.ByteWidth(ByteWidth),    		// Larghezza del singolo byte 
	.NumPorts(NumPorts),    		// Numero di porte
	.Latency(Latency),    			// Latenza di lettura 
	.N_BLOCKS(N_BLOCKS)
)i_memory_unit(
    .clk_i(clk),
    .rst_ni(mem_rst_n),
    .addr(mem_addr),
	.byte_sel(mem_byte_sel),
	.data_in(fifo_r_data),
	.mem_en(mem_mem_en),
	.wr_en(mem_wr_en),
	.data_out(mem_data_out),
	.idx_addr(mem_idx_addr),
	.idx_byte_sel(mem_idx_byte_sel),
	.idx_data_in(fifo_r_data),
	.idx_mem_en(mem_idx_mem_en),
	.idx_wr_en(mem_idx_wr_en),
	.idx_data_out(mem_idx_data_out)
);


// IMPLEMENTAZIONE FIFO
fifo #(
    .DATA_WIDTH(AXI_DATA_SIZE),      
    .DEPTH(FIFO_DEPTH),      
	.THRESHOLD(FIFO_THRESHOLD)
)i_fifo(
    .clk(clk),      
    .rst_n(fifo_rst_n),    
    .w_en(axi_rd_is_valid_data),     
    .w_data(axi_rd_data_out), 
    .r_en(fifo_rd),
    .r_data(fifo_r_data),     
    .empty(fifo_empty),    
    .threshold(fifo_threshold)
);


// IMPLEMENTAZIONE AXI4 MASTER READ
axi4_master_read #(
    .ADDR_W(AXI_ADDR_W),
	.DATA_W(AXI_DATA_SIZE)
)axi4_read(
    .clk_i(clk),
    .rst_ni(axi_rd_rst_n),
    .start(axi_rd_start),
    .available(axi_rd_available),
	.ar_size_in,
	.ar_addr_dense_matrix(ar_addr_dense_matrix),
	.total_len_dense_matrix(total_len_dense_matrix),
	.ar_addr_comp_idx(ar_addr_comp_idx),
	.total_len_comp_idx(total_len_comp_idx),
	.ar_addr_idx(ar_addr_idx),
	.total_len_idx(total_len_idx),
	.rst_n_multi_burst(rst_n_multi_burst),
	.fifo_empty(fifo_empty),
	.tc_wr_mem(tc_wr_mem),   // Fine scrittura della memoria densa
	.wr_index(wr_index),	 // Scritura memoria degli indici compressi
	.tc_rd_index(tc_rd_index), // Lettura degli indici
	.ar_addr_out,
	.ar_len_out,
	.ar_size_out,
	.data_in,
	.data_out(axi_rd_data_out),
    .is_valid_data(axi_rd_is_valid_data),  		
    .done(rd_done),       	
	.fifo_full(fifo_threshold),	
    .ar_valid,
    .ar_ready,
    .ar_burst,
    .r_valid,
    .r_ready,
    .r_last
);

always_ff @(posedge clk or negedge rst_n) begin // Reset hardware generale asincrono
	if (!rst_n) begin
		axi_rd_done <= 1'b0;
	end else begin
		if (!axi_rd_done_rst_n) begin // Diventa un clear sincrono perfetto gestito dalla CU
			axi_rd_done <= 1'b0;
		end else if (rd_done) begin
			axi_rd_done <= 1'b1;
		end
	end
end

endmodule
