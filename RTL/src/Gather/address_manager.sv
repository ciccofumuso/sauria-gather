`timescale 1ns/1ps
module address_manager #(
	parameter int AXI_ADDR_W				= 32,
	parameter int NumWords					= 16,	// Numero di word totali nella SRAM della matrice complessa
	parameter int N_BLOCK_SRAM				= 8,	// Numero di blocchi di memoria
	parameter int WORD_SIZE 				= 16,	// Dimensione della word
	parameter int AXI_DATA_SIZE  			= 64,	// Dimensione dei dati dall'esterno
	parameter int unsigned NumWords_idx		= 16,   // Numero di word totali nella SRAM degli indici compressi
	parameter int unsigned WORD_SIZE_IDX	= 32,   // Larghezza del singolo byte di indici compressi 
	// PARAMETRI DIPENDENTI
	parameter int unsigned WORDS_PER_LINE 		= $clog2(AXI_DATA_SIZE / WORD_SIZE),
	parameter int unsigned WORDS_PER_LINE_IDX	= $clog2(AXI_DATA_SIZE / WORD_SIZE_IDX)
)(
	input 	logic							clk,
	
	// Gestione indirizzi per la memoria a blocchi
	input	logic[AXI_ADDR_W-1:0]							status_reg_n_row,			// Registro di stato per il numero di word in un blocco (n_word-1)
	input 	logic											cnt_row_en,cnt_row_rst_n,	// Segnali Counter del numero di word per blocco
	input   logic											rd_b_mem,					// Switch tra lettura=1 e scrittura=0
	output	logic[($clog2(NumWords)-1):0] 					addr_b_mem_out,
	output	logic[WORDS_PER_LINE-1:0]						byte_b_mem_out,
	output	logic[($clog2(N_BLOCK_SRAM)-1):0]				en_b_mem_out,
	// Segnali PISO
	input	logic											piso_rst_n,load,shift_en,
	output	logic											empty,
	input   logic                    						fifo_empty,   // Segnale di FIFO vuota
    output  logic                    						fifo_r_en,    // Segnale di lettura per la FIFO
	
	input 	logic											is_spmm,					// Selettore dell'operazione
	
	input	logic[(AXI_DATA_SIZE-1):0]						piso_data_in,
	
	// Gestione valori non zeri per indici compressi(CSR -> riga, CSC -> colonna)
	input	logic [WORD_SIZE_IDX-1:0]		comp_idx_in,								// word degli indici in ingresso
	input	logic							comp_idx_ptr_old_en,comp_idx_ptr_old_rst_n, // Registro per il salvataggio del primo indice
	input	logic							mux_comp_idx_sel,							// Switch tra lettura nuovo intervallo oppure decremento
	input 	logic							cnt_nnzs_en,cnt_nnzs_rst_n,					// Counter dei valori non zero
	output 	logic 							last_index, end_rd_index,					// Segnali di stato dell'operazione di calcolo di valori non zero
	output	logic							tc_nnzs,									// Terminal Counter del Counter dei valori non zero
	output  logic                           next_row_empty,
	// Gestione blocco di SRAM per spMM
	input 	logic							cnt_stage_en,cnt_stage_rst_n,				// Aggiornamento del blocco memoria(solo spMM), si incrementa quando tc_nnzs = 1
	
	// Indicizzazione memoria degli indici
	input 	logic												cnt_comp_idx_en,cnt_comp_idx_rst_n, // Counter scrittura indirizzi della memoria indici
	output	logic [($clog2(NumWords_idx) + WORDS_PER_LINE_IDX)-1:0]	cnt_comp_idx_out				// Uscita indirizzo della memoria indici

);

localparam int ADDR_IDX_WIDTH = $clog2(NumWords_idx) + WORDS_PER_LINE_IDX;
localparam int MAX_VAL_IDX    = (NumWords_idx * (AXI_DATA_SIZE / WORD_SIZE_IDX)) - 1;

logic [WORD_SIZE_IDX-1:0]		comp_idx_ptr_old_out;
logic [1:0][WORD_SIZE_IDX-1:0]	mux_comp_idx_ptr;
logic [WORD_SIZE_IDX-1:0]		cnt_nnzs_out;


logic [($clog2(NumWords_idx) + WORDS_PER_LINE_IDX)-1:0]	cnt_addr_idx_out;
logic [1:0][($clog2(NumWords_idx) + WORDS_PER_LINE_IDX)-1:0]	mux_comp_idx_addr;

logic [$clog2(N_BLOCK_SRAM)-1:0]cnt_stage_out; 					// Puntatore al blocco di SRAM

logic update_n_col;
logic [$clog2(NumWords)-1:0] cnt_row_out;
logic [($clog2(N_BLOCK_SRAM)-1):0] cnt_col_out;



//========================================================
//	CALCOLO QUANTITà DI NON-ZERI
//========================================================

always_ff @(posedge clk or negedge comp_idx_ptr_old_rst_n) begin
	if (!comp_idx_ptr_old_rst_n) begin
		comp_idx_ptr_old_out <= '0;
	end else if (comp_idx_ptr_old_en) begin
		comp_idx_ptr_old_out <= comp_idx_in;
	end
end

assign next_row_empty = (comp_idx_in == comp_idx_ptr_old_out);

assign mux_comp_idx_ptr[0] = comp_idx_in - comp_idx_ptr_old_out;
assign mux_comp_idx_ptr[1] = cnt_nnzs_out - 1'b1;

always_ff @(posedge clk or negedge cnt_nnzs_rst_n) begin
	if (!cnt_nnzs_rst_n) begin
		cnt_nnzs_out <= '0;
	end else if (cnt_nnzs_en) begin
		cnt_nnzs_out <= mux_comp_idx_ptr[mux_comp_idx_sel];
	end
end

assign tc_nnzs = (cnt_nnzs_out == '0);
// flag se sono finiti i puntatori
assign last_index = (comp_idx_in == '0);

//========================================================
//	COUNTER DEGLI INDIRIZZI DEGLI INDICI COMPRESSI
//========================================================
always_ff @(posedge clk or negedge cnt_comp_idx_rst_n) begin
	if (!cnt_comp_idx_rst_n) begin
		cnt_addr_idx_out <= '0;
	end else if (cnt_comp_idx_en) begin
		cnt_addr_idx_out <= cnt_addr_idx_out + 1'b1;
	end
end

assign mux_comp_idx_addr[0] = {cnt_addr_idx_out[0 +: $clog2(NumWords_idx)], {WORDS_PER_LINE_IDX{1'b0}}};
assign mux_comp_idx_addr[1] = cnt_addr_idx_out;

assign cnt_comp_idx_out = mux_comp_idx_addr[rd_b_mem];

assign end_rd_index = (cnt_addr_idx_out == ADDR_IDX_WIDTH'(MAX_VAL_IDX));

//========================================================
//	COUNTER STAGE
//======================================================== 
always_ff @(posedge clk or negedge cnt_stage_rst_n) begin
	if (!cnt_stage_rst_n) begin
		cnt_stage_out <= '0;
	end else if (cnt_stage_en) begin
		cnt_stage_out <= cnt_stage_out + 1'b1;
	end
end

//========================================================
//	COUNTER INDIRIZZAMENTO MATRICE DENSA
//======================================================== 
//counter row
always_ff @(posedge clk or negedge cnt_row_rst_n) begin
	if (!cnt_row_rst_n) begin
		cnt_row_out <= '0;
	end else if (update_n_col) begin
		cnt_row_out <= '0;
	end else if (cnt_row_en) begin
		cnt_row_out <= cnt_row_out + 1'b1;
	end
end
//counter col
always_ff @(posedge clk or negedge cnt_row_rst_n) begin
	if (!cnt_row_rst_n) begin
		cnt_col_out <= '0;
	end else if (update_n_col) begin
		cnt_col_out <= cnt_col_out + 1'b1;
	end
end

assign update_n_col =(is_spmm == 1'b0) ? (cnt_row_en && (cnt_row_out == '1)) : (cnt_row_en && (cnt_row_out == status_reg_n_row[WORDS_PER_LINE +: $clog2(NumWords)]));

//========================================================
//	SEGNALI ED ISTANZA DELLA PISO
//========================================================

logic [WORD_SIZE_IDX-1:0] 				piso_data_out;
logic[($clog2(NumWords)-1):0] 			piso_addr_b_mem_out;
logic[WORDS_PER_LINE-1:0]				piso_byte_b_mem_out;
logic[($clog2(N_BLOCK_SRAM)-1):0]		piso_en_b_mem_out;
logic [1:0][($clog2(N_BLOCK_SRAM)-1):0]	mux_en_b_mem_spmm;
logic [1:0][($clog2(N_BLOCK_SRAM)-1):0]	mux_en_b_mem_rd_wr;
logic [1:0][($clog2(NumWords)-1):0]		mux_addr_b_mem;
// Istanza della PISO
piso #(
    .N_DATA_IN(AXI_DATA_SIZE),
	.WORD_SIZE(WORD_SIZE_IDX)
)i_piso(
    .clk(clk),
    .rst_n(piso_rst_n),
    .data_in(piso_data_in),    // Dato a 64 bit dal buffer 
    .load(load),       // Segnale per caricare il dato
    .shift_en(shift_en),   // Richiede la parola successiva
	.fifo_empty(fifo_empty),   // Segnale di FIFO vuota
    .fifo_r_en(fifo_r_en),    // Segnale di lettura per la FIFO
    .data_out(piso_data_out),   // Parola in uscita (max 32bit)
    .empty(empty)       // 1 quando tutti i dati sono stati inviati
);

// Controllo della lunghezza di piso_data_out_extended
localparam int REQ_WIDTH = WORDS_PER_LINE + $clog2(NumWords) + $clog2(N_BLOCK_SRAM);
localparam int EXT_WIDTH = (WORD_SIZE_IDX > REQ_WIDTH) ? WORD_SIZE_IDX : REQ_WIDTH;

logic [EXT_WIDTH-1:0] piso_data_out_extended;
assign piso_data_out_extended = EXT_WIDTH'(piso_data_out);

// Divisione uscita dalla piso in (blocco,indirizzo,byte)
assign piso_byte_b_mem_out = piso_data_out_extended[0 +: WORDS_PER_LINE];
assign piso_addr_b_mem_out = piso_data_out_extended[WORDS_PER_LINE +: $clog2(NumWords)];
assign piso_en_b_mem_out = piso_data_out_extended[(WORDS_PER_LINE + $clog2(NumWords)) +: $clog2(N_BLOCK_SRAM)];

//========================================================
//	MUX INDIRIZZAMENTO BLOCCO DI MEMORIA
//======================================================== 
// Mux selezione del blocco per l'operazione spMM
assign mux_en_b_mem_spmm[0] = piso_en_b_mem_out;
assign mux_en_b_mem_spmm[1] = cnt_stage_out;

// Mux selezione del blocco per Read/Write
assign mux_en_b_mem_rd_wr[0] = cnt_col_out;
assign mux_en_b_mem_rd_wr[1] = mux_en_b_mem_spmm[is_spmm];

// Mux selezione dell'indirizzo per Read/Write
assign mux_addr_b_mem[0] = cnt_row_out;
assign mux_addr_b_mem[1] = piso_addr_b_mem_out;

// Assegnazione uscite per l'indirizzamento della memoria densa
assign byte_b_mem_out = piso_byte_b_mem_out;
assign addr_b_mem_out = mux_addr_b_mem[rd_b_mem];
assign en_b_mem_out = mux_en_b_mem_rd_wr[rd_b_mem];




endmodule
