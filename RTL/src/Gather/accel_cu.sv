`timescale 1ns/1ps
module accel_cu(
	input  	logic 		clk,rst_n,start,
	output	logic		done,
	// INTERFACCIA AXI4 READ
	output  logic 		axi_rd_rst_n, 
	output  logic       axi_rd_start,  		// Richiesta di lettura valida 
    input 	logic       axi_rd_available,  	// Disponibile ad accettare una nuova richiesta
	input 	logic       axi_rd_done,		// Ultimo dato arrivato
	output	logic		axi_rd_done_rst_n, 

	// INTERFACCIA FIFO		
	output  logic       fifo_r_en, 
	output	logic       fifo_rst_n,    
    input 	logic       fifo_empty,    
			
	// INTERFACCIA MEMORIA		
	output	logic       mem_rst_n, 
	output	logic 		mem_wr_en, 
	output	logic 		mem_idx_wr_en, 
	output	logic 		mem_idx_mem_en, 
			
	// INTERFACCIA BUFFER 		
	output	logic       buffer_rst_n, 
	output  logic       buffer_data_in_valid, 
    output  logic       buffer_data_in_last, 
	input 	logic       buffer_wait_wr,
	input	logic		buffer_data_out_valid,
			
	// INTERFACCIA AXI4 WRITE		
	output  logic 		axi_wr_rst_n,      
	input 	logic       axi_wr_done,		
	output	logic		axi_wr_done_rst_n, 
	output	logic		rst_n_multi_burst,
	
	// INTERFACCIA ADDRESS MANAGER
	// Segnali di stato delle impostazioni del driver
	input 	logic 		tc_wr_mem,   // Fine scrittura della memoria densa
	input 	logic 		wr_index,	 // Scritura memoria degli indici compressi
	input 	logic 		tc_rd_index, // Lettura degli indici
	// Gestione indirizzi per la memoria a blocchi
	output 	logic		cnt_row_en,cnt_row_rst_n, 
	output  logic		rd_b_mem, 
	// Segnali PISO
	output	logic		piso_rst_n,piso_load,piso_shift_en, 
	input	logic		piso_empty,
	output	logic		comp_idx_ptr_old_en,comp_idx_ptr_old_rst_n, 
	output	logic		mux_comp_idx_sel, 
	output 	logic		cnt_nnzs_en,cnt_nnzs_rst_n, 
	input 	logic 		last_index, end_rd_index,
	input	logic		tc_nnzs,
	// Gestione blocco di SRAM per spMM
	output 	logic		cnt_stage_en,cnt_stage_rst_n,	// Si incrementa quando tc_nnzs = 1
	// Indicizzazione memoria degli indici
	output 	logic		cnt_comp_idx_en,cnt_comp_idx_rst_n
);

typedef enum logic [5:0] {
	IDLE,       			// Attesa comando
	START,      			// Campionamento parametri
	WAIT_AXI_RD,			// Attesa memoria per leggere i dati da salvare internamente
	RD_FIFO_WR_MEM,			// Prelevamento dati dalla FIFO per scrivere in memoria
	WR_B,					// Scrittura in entrambe le memorie
	WR_LAST_DATA,			// Svuotamento FIFO per evitare di scrivere in locazioni sbagliate
	START_NEXT_AXI_ADDR,	// Invio richiesta per ricevere altri dati dalla memoria esterna
	REQ_MEM_1_PTR,			// Invio l'indirizzo per il 1° indice compresso
	RD_1_PTR,				// Lettura 1° indice compresso
	RD_2_PTR,				// Lettura 2° indice compresso
	REQ_MEM_NEW_PTR,		// Richiesta nuovo indice compresso
	START_OP,				// Check iniziale condizioni riga
	RD_FIFO,				// Estrazione parallela word indici colonna da FIFO
	LOAD_PISO,				// Caricamento parallelo nel SIPO/PISO
	REQ_MEM_DATA,			// Fase di inserimento indirizzo (PISO bloccata, segnali stabili)
	WR_BUFFER,				// Fase di lettura sincrona e validazione dato (PISO avanza)
	CLR_AXI_RD_DONE,		// Reset del flag done precedente
	AXI_RD_NEXT_ADDR,		// Invio del nuovo comando di avvio burst su canale AXI
	UPDATE_BLOCK,			// Aggiornamento del blocco di memoria selezionato
	FLUSH_BUFFER,			// Svuotamento buffer quando ho finito le operazioni di ricerca valori
	WAIT_AXI_WR,			// Attesa fine invio dati all'ALU
	DONE					// Fine operazioni
} state_e;

state_e current_state, next_state;

logic fifo_data_vld_q;
logic nnzs_check;

// Ritardo di 1 colpo di clock del segnale di lettura della FIFO per la fase di scrittura iniziale
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_data_vld_q <= 1'b0;
	end else begin
        fifo_data_vld_q <= fifo_r_en;
	end
end

// Logica Sequenziale FSM
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		current_state <= IDLE;
	end else begin        
		current_state <= next_state;
	end
end

// Registro di protezione per evitare falsi positivi di last_index su righe vuote iniziali
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		nnzs_check <= 1'b0;
	end else begin
		if (current_state == IDLE || current_state == START) begin
			nnzs_check <= 1'b0;
		end else if ((current_state == RD_2_PTR || current_state == REQ_MEM_NEW_PTR) && !last_index) begin
			nnzs_check <= 1'b1;
		end
	end
end

// Definizione Stati e Logica Combinatoria delle Uscite
always_comb begin
	case(current_state)
	
		IDLE : begin
			if (start) next_state = START;
			else next_state = IDLE;
			axi_rd_rst_n			= 1'b0; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b0;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b0; 
			mem_rst_n				= 1'b0;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b0; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b0; 
			axi_wr_done_rst_n       = 1'b0; 
			rst_n_multi_burst		= 1'b0;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b0; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b0; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b0; 
			comp_idx_ptr_old_rst_n  = 1'b0; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b0; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b0;
			done					= 1'b0;
		end
		
		START : begin
			next_state = WAIT_AXI_RD;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b1; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		WAIT_AXI_RD : begin
			if (fifo_empty) next_state = WAIT_AXI_RD;
			else next_state = RD_FIFO_WR_MEM;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		RD_FIFO_WR_MEM : begin
			next_state = WR_B;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b1; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		WR_B : begin
			if (axi_rd_done && axi_rd_available && fifo_empty && !fifo_data_vld_q) begin
				next_state = WR_LAST_DATA;
			end else begin
				next_state = WR_B;
			end
			fifo_r_en = !fifo_empty;
			if (wr_index) begin
				mem_wr_en				= 1'b0;
				cnt_row_en              = 1'b0;
				cnt_comp_idx_en         = fifo_data_vld_q;
				mem_idx_wr_en			= fifo_data_vld_q;
				mem_idx_mem_en			= fifo_data_vld_q;
			end else begin
				mem_wr_en				= fifo_data_vld_q;
				cnt_row_en              = fifo_data_vld_q;
				cnt_comp_idx_en         = 1'b0;
				mem_idx_wr_en			= 1'b0;
				mem_idx_mem_en			= 1'b0;
			end
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1; 
			buffer_rst_n			= 1'b1;
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0; 
			axi_wr_rst_n            = 1'b1;
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1; 
			cnt_row_rst_n           = 1'b1;
			rd_b_mem           		= 1'b0; 
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0;
			piso_shift_en           = 1'b0; 
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0;
			cnt_nnzs_en             = 1'b0; 
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1;
			cnt_stage_en            = 1'b0; 
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		WR_LAST_DATA : begin
			if (tc_wr_mem) next_state = REQ_MEM_1_PTR;
			else next_state = START_NEXT_AXI_ADDR;
			if (wr_index) begin
				mem_wr_en       = 1'b0; 
				cnt_row_en      = 1'b0;
				mem_idx_wr_en   = fifo_data_vld_q; 
				mem_idx_mem_en  = fifo_data_vld_q;
				cnt_comp_idx_en = fifo_data_vld_q;
			end else begin
				mem_wr_en       = fifo_data_vld_q; 
				cnt_row_en      = fifo_data_vld_q;
				mem_idx_wr_en   = 1'b0; 
				mem_idx_mem_en  = 1'b0; 
				cnt_comp_idx_en = 1'b0;
			end
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b0; 
			piso_rst_n              = 1'b1;
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0; 
			comp_idx_ptr_old_en     = 1'b0;
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0; 
			cnt_nnzs_rst_n          = 1'b1;
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0; 
			cnt_stage_rst_n         = 1'b1;
			cnt_comp_idx_rst_n      = 1'b1; 
			done					= 1'b0;
		end
		
		START_NEXT_AXI_ADDR : begin
			next_state = WAIT_AXI_RD;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b1; 
			axi_rd_done_rst_n		= 1'b0;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		REQ_MEM_1_PTR : begin
			next_state = RD_1_PTR;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b1; 
			axi_rd_done_rst_n		= 1'b0; 
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b0;
			done					= 1'b0;
		end
		
		RD_1_PTR : begin
			next_state = RD_2_PTR;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b1; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b1; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		RD_2_PTR : begin
			next_state = START_OP; 
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b1; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b1;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		START_OP : begin
			// Last_index è valido solo se abbiamo già incontrato dati validi
			if (last_index && nnzs_check) begin 
				next_state = FLUSH_BUFFER;
			end else if (tc_nnzs) begin
				if (end_rd_index) next_state = FLUSH_BUFFER;
				else next_state = UPDATE_BLOCK;
			end else begin
				if (piso_empty) begin
					if (!fifo_empty) next_state = RD_FIFO;
					else next_state = START_OP;
				end else begin
					next_state = REQ_MEM_DATA; 
				end
			end
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		RD_FIFO : begin
			next_state = LOAD_PISO;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b1; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		LOAD_PISO : begin
			next_state = REQ_MEM_DATA; 
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b1; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		REQ_MEM_DATA : begin
			if (axi_rd_done && !tc_rd_index) begin
				next_state = CLR_AXI_RD_DONE;
			end else if (tc_nnzs) begin
				if (end_rd_index) next_state = FLUSH_BUFFER;
				else              next_state = UPDATE_BLOCK;
			end else if (piso_empty) begin
				if (!fifo_empty)  next_state = RD_FIFO;
				else              next_state = REQ_MEM_DATA;
			end else begin
				next_state = WR_BUFFER;
			end

			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		WR_BUFFER : begin
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1; 
			mem_wr_en				= 1'b0;
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1; 
			buffer_rst_n			= 1'b1;
			buffer_data_in_last     = 1'b0; 
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1;
			rst_n_multi_burst		= 1'b1; 
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1;
			rd_b_mem           		= 1'b1; 
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_nnzs_rst_n          = 1'b1;
			mux_comp_idx_sel        = 1'b1; 
			cnt_stage_en            = 1'b0; 
			cnt_stage_rst_n         = 1'b1;
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1; 
			done					= 1'b0; 
			
			if (buffer_wait_wr) begin
				next_state           = WR_BUFFER;
				fifo_r_en            = 1'b0;
				piso_shift_en        = 1'b0;
				cnt_nnzs_en          = 1'b0;
				buffer_data_in_valid = 1'b0;
			end else begin
				buffer_data_in_valid = 1'b1;
				piso_shift_en        = 1'b1;
				cnt_nnzs_en          = 1'b1;
				fifo_r_en            = 1'b0;
				next_state           = REQ_MEM_DATA;
			end
		end

		CLR_AXI_RD_DONE : begin
			next_state = AXI_RD_NEXT_ADDR;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b0; 
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		AXI_RD_NEXT_ADDR : begin
			next_state = REQ_MEM_DATA;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b1; 
			axi_rd_done_rst_n		= 1'b1; 
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end

		UPDATE_BLOCK : begin
			next_state = REQ_MEM_NEW_PTR;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b1;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b1; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		REQ_MEM_NEW_PTR : begin
			next_state = RD_2_PTR;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		FLUSH_BUFFER : begin
			next_state = WAIT_AXI_WR;
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b1;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b1;
			axi_wr_rst_n            = 1'b1; 
			axi_wr_done_rst_n       = 1'b0; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		WAIT_AXI_WR : begin
			if (axi_wr_done) begin
				if (!buffer_data_out_valid) begin
					axi_wr_done_rst_n	= 1'b1; 
					next_state = DONE;
				end else begin
					axi_wr_done_rst_n	= 1'b0; 
					next_state = WAIT_AXI_WR;
				end
			end else begin
				next_state = WAIT_AXI_WR; 
				axi_wr_done_rst_n	= 1'b1;
			end
			axi_rd_rst_n			= 1'b1; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b1;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b1; 
			mem_rst_n				= 1'b1;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b1; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b1; 
			rst_n_multi_burst		= 1'b1;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b1; 
			rd_b_mem           		= 1'b1;
			piso_rst_n              = 1'b1; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b1; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b1; 
			comp_idx_ptr_old_rst_n  = 1'b1; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b1; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b1;
			done					= 1'b0;
		end
		
		DONE : begin
			next_state = IDLE;
			axi_rd_rst_n			= 1'b0; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b0;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b0; 
			mem_rst_n				= 1'b0;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b0; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b0; 
			axi_wr_done_rst_n       = 1'b0; 
			rst_n_multi_burst		= 1'b0;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b0; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b0; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b0; 
			comp_idx_ptr_old_rst_n  = 1'b0; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b0; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b0;
			done					= 1'b1;
		end
		
		default : begin
			next_state = IDLE;
			axi_rd_rst_n			= 1'b0; 
			axi_rd_start			= 1'b0; 
			axi_rd_done_rst_n		= 1'b0;
			fifo_r_en				= 1'b0; 
			fifo_rst_n				= 1'b0; 
			mem_rst_n				= 1'b0;
			mem_wr_en				= 1'b0; 
			mem_idx_wr_en			= 1'b0; 
			mem_idx_mem_en			= 1'b0;
			buffer_rst_n			= 1'b0; 
			buffer_data_in_valid	= 1'b0; 
			buffer_data_in_last     = 1'b0;
			axi_wr_rst_n            = 1'b0; 
			axi_wr_done_rst_n       = 1'b0; 
			rst_n_multi_burst		= 1'b0;
			cnt_row_en              = 1'b0; 
			cnt_row_rst_n           = 1'b0; 
			rd_b_mem           		= 1'b0;
			piso_rst_n              = 1'b0; 
			piso_load               = 1'b0; 
			piso_shift_en           = 1'b0;
			comp_idx_ptr_old_en     = 1'b0; 
			mux_comp_idx_sel        = 1'b0; 
			cnt_nnzs_en             = 1'b0;
			cnt_nnzs_rst_n          = 1'b0; 
			comp_idx_ptr_old_rst_n  = 1'b0; 
			cnt_stage_en            = 1'b0;
			cnt_stage_rst_n         = 1'b0; 
			cnt_comp_idx_en         = 1'b0; 
			cnt_comp_idx_rst_n      = 1'b0;
			done					= 1'b0;
		end
		
	endcase
end

endmodule
