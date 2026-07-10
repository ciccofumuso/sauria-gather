`timescale 1ns/1ps
module axi4_master_read #(
    parameter int unsigned ADDR_W 			= 32,
	parameter int unsigned DATA_W 			= 64
)(
    input  logic clk_i,
    input  logic rst_ni,

    // ==========================================
    // INTERFACCIA ACCELERATORE
    // ==========================================
    // Comandi di avvio transazione
    input  	logic              	start,  					// Richiesta di lettura valida
    output 	logic              	available,  				// Disponibile ad accettare una nuova richiesta
	
	// Reset e segnali di stato
	input 	logic 		rst_n_multi_burst,
	input	logic		fifo_empty,
	output 	logic 		tc_wr_mem,   // Fine scrittura della memoria densa
	output 	logic 		wr_index,	 // Scritura memoria degli indici compressi
	output 	logic 		tc_rd_index, // Lettura degli indici
	// Paramentri in ingresso
	input  	logic [2:0]        	ar_size_in,
	input 	logic [ADDR_W-1:0] 	ar_addr_dense_matrix,
	input 	logic [ADDR_W-1:0] 	total_len_dense_matrix,
	input 	logic [ADDR_W-1:0] 	ar_addr_comp_idx,
	input 	logic [ADDR_W-1:0] 	total_len_comp_idx,
	input 	logic [ADDR_W-1:0] 	ar_addr_idx,
	input 	logic [ADDR_W-1:0] 	total_len_idx,
	// Parametri in uscita verso lo slave
	output 	logic [ADDR_W-1:0] ar_addr_out,
	output 	logic [7:0]        ar_len_out,
	output 	logic [2:0]        ar_size_out,
	
	// ==========================================
    // DATI DELLA TRANSAZIONE
    // ==========================================
    // Dati in ingresso provenienti dallo slave
	input 	logic [DATA_W-1:0] data_in,
	// Dati verso l'Acceleratore
	output 	logic [DATA_W-1:0] data_out,
	
	
    // Segnali verso l'Acceleratore
    output logic              	is_valid_data,  			// Un dato del burst è pronto per l'acceleratore
    output logic              	done,       				// Ultimo dato del burst e transazione finita
	input  logic				fifo_full,

    // ==========================================
    // INTERFACCIA VERSO L'ESTERNO
    // ==========================================
    // Canale AR
    output logic              	ar_valid,
    input  logic              	ar_ready,
    output logic [1:0]        	ar_burst,

    // Canale R
    input  logic              	r_valid,
    output logic              	r_ready,
    input  logic              	r_last
);

// segnali dei registri del datapath
logic		ar_size_en,ar_size_rst_n; 	// Controlli registro per la dimensione
logic       reg_data_en,reg_data_rst_n; // Controlli registro dei dati
logic		valid_data, waiting_for_fifo;


// ==========================================
// CONTROL UNIT AXI4 MASTER READ
// ==========================================
// Stati FSM
typedef enum logic [2:0] {
	IDLE,    // Attesa di un comando
	START,   // Campionamento parametri
	READ_REQ,  // Invio parametri sul bus AXI
	SET_RREADY, // Dico allo slave che sono pronto per leggere
	READ_BURST,  // Ricezione dati all'utente
	DONE	  // Invio dell'ultimo dato all'acceleratore
} state_e;

state_e current_state, next_state;


always_ff @(posedge clk_i  or negedge rst_ni) begin
	if (!rst_ni) begin
		current_state <= IDLE;
	end else begin
		current_state <= next_state;
	end
end


always_comb begin
	case (current_state)

		IDLE: begin
			available 		= !waiting_for_fifo; 
			done	      	= 1'b0;
			valid_data 		= 1'b0;
			ar_valid 		= 1'b0;
			ar_size_en  	= 1'b0;
			ar_size_rst_n	= 1'b0;
			reg_data_en		= 1'b0;
			reg_data_rst_n	= 1'b0;
			ar_burst	 	= 2'b01;  // INCR
			r_ready  		= 1'b0;
			
			if (start) begin
				next_state = START;
			end else begin
				next_state = IDLE;
			end
		end
		
		START: begin
			next_state = READ_REQ;
			available 		= 1'b0; 
			done	      	= 1'b0;
			valid_data	 	= 1'b0;
			ar_valid 		= 1'b0;
			ar_size_en  	= 1'b1;	// Attivo il registro size
			ar_size_rst_n	= 1'b1;
			reg_data_en		= 1'b0;
			reg_data_rst_n	= 1'b0;
			ar_burst	 	= 2'b01;
			r_ready  		= 1'b0;
			
		end
		
		READ_REQ: begin
			// Quando la memoria accetta i parametri, passiamo alla lettura
			if (ar_ready) begin
				next_state = READ_BURST;
			end else begin
				next_state = READ_REQ;
			end
			available 		= 1'b0; 
			done	      	= 1'b0;
			valid_data	 	= 1'b0;
			ar_valid 		= 1'b1;	// Alziamo la richiesta sul bus
			ar_size_en  	= 1'b0;
			ar_size_rst_n	= 1'b1;
			reg_data_en		= 1'b0;
			reg_data_rst_n	= 1'b0;
			ar_burst	 	= 2'b01;  
			r_ready  		= 1'b0;
		end

		READ_BURST: begin
			if (r_valid && r_last && r_ready) begin
				// Se è l'ultimo dato del burst
				next_state = DONE;
			end else begin
				next_state = READ_BURST;
			end
			available 		= 1'b0; 
			done	      	= 1'b0;
			ar_valid 		= 1'b0;
			ar_size_en  	= 1'b0;
			ar_size_rst_n	= 1'b1;
			reg_data_en		= 1'b1; // Abilito il registro dei dati
			reg_data_rst_n	= 1'b1;
			ar_burst	 	= 2'b01;  
			r_ready 		= !fifo_full; 
			if (r_valid && r_ready) begin
				valid_data	 	= 1'b1;	// Il dato è valido per il Campionamento
			end else begin
				valid_data	 	= 1'b0;	// Il dato non è pronto
			end
		end
		
		DONE: begin
			next_state = IDLE;
			available 		= 1'b0; 
			done	      	= 1'b1;
			valid_data	 	= 1'b0;	
			ar_valid 		= 1'b0;
			ar_size_en  	= 1'b0;
			ar_size_rst_n	= 1'b1;
			reg_data_en		= 1'b0; // Disabilito il registro dei dati
			reg_data_rst_n	= 1'b1;
			ar_burst	 	= 2'b01;  
			r_ready 		= 1'b0; 
		end

		default: begin
			next_state = IDLE;
			available 		= !waiting_for_fifo; 
			done	      	= 1'b0;
			valid_data	 	= 1'b0;
			ar_valid 		= 1'b0;
			ar_size_en  	= 1'b0;
			ar_size_rst_n	= 1'b0;
			reg_data_en		= 1'b0;
			reg_data_rst_n	= 1'b0;
			ar_burst	 	= 2'b01;
			r_ready  		= 1'b0;
		end
	endcase
end

// ==========================================   
// DATAPATH AXI4 MASTER READ                    
// ==========================================   
  
// Registro dei dati
always_ff @(posedge clk_i  or negedge reg_data_rst_n) begin
	if (!reg_data_rst_n) begin
		data_out <= '0;
	end
    else if (reg_data_en) begin
		data_out <= data_in;
    end
end

// Registro per la dimensione
always_ff @(posedge clk_i  or negedge ar_size_rst_n) begin
	if (!ar_size_rst_n) begin
		ar_size_out <= '0;
	end
    else if (ar_size_en) begin
		ar_size_out <= ar_size_in;
    end
end

//Registro di pipe per il segnale di validità del dato
always_ff @(posedge clk_i  or negedge rst_ni) begin
	if (!rst_ni) begin
		is_valid_data <= '0;
	end
    else begin
		is_valid_data <= valid_data;
    end
end

// ==========================================   
// LOGICA MULTI BURST                    
// ==========================================   
logic[ADDR_W-1:0] reg_len_out, sub_len_out;
logic first_run, end_burst;
logic[ADDR_W-1:0] reg_addr_out, add_addr_out;


always_comb begin
	if (reg_len_out > 256) begin
		sub_len_out = reg_len_out - 32'd256;
		ar_len_out = 8'hFF;
	end else begin
		sub_len_out = reg_len_out - {23'd0,reg_len_out[8:0]};
		ar_len_out = reg_len_out[7:0] - 8'd1;
	end
end

always_ff @(posedge clk_i, negedge rst_n_multi_burst) begin
	if (!rst_n_multi_burst) begin
		first_run <= 1'b1;
	end else begin
		if (start) begin
			first_run <= 1'b0;
		end
	end
	
end

assign end_burst = (sub_len_out == '0);


assign add_addr_out = reg_addr_out + (32'(ar_len_out+8'd1) << ar_size_out);

always_ff @(posedge clk_i  or negedge rst_n_multi_burst) begin
	if (!rst_n_multi_burst) begin
		reg_len_out      <= '0;
		reg_addr_out     <= '0;
		wr_index         <= 1'b0;
		tc_wr_mem        <= 1'b0;
		tc_rd_index      <= 1'b0;
		waiting_for_fifo <= 1'b0;
	end else begin
		if (first_run) begin
			if (start) begin
				reg_len_out	 <= total_len_dense_matrix;
				reg_addr_out <= ar_addr_dense_matrix;
			end
		end else begin
			// Analisi fine burst normale
			if (done && end_burst) begin
				if (wr_index == 1'b0) begin
					if (fifo_empty) begin
						// Se la FIFO è già vuota al momento del done
						reg_len_out  <= total_len_comp_idx;
						reg_addr_out <= ar_addr_comp_idx;
						wr_index     <= 1'b1;
						waiting_for_fifo <= 1'b0;
					end else begin
						// Se la FIFO è ancora occupata
						waiting_for_fifo <= 1'b1;
					end
				end else if (wr_index && !tc_wr_mem) begin
					tc_wr_mem    <= 1'b1;
					reg_len_out  <= total_len_idx;
					reg_addr_out <= ar_addr_idx;
				end else if (wr_index && tc_wr_mem)
					tc_rd_index  <= 1'b1;
			end else if (done && !end_burst) begin
				reg_len_out  <= sub_len_out;
				reg_addr_out <= add_addr_out;
			end
			
			// Controllo di uscita dal waiting_for_fifo
			if (waiting_for_fifo && fifo_empty) begin
				reg_len_out      <= total_len_comp_idx;
				reg_addr_out     <= ar_addr_comp_idx;
				wr_index         <= 1'b1;   
				waiting_for_fifo <= 1'b0;  
			end
		end
	end
end

assign ar_addr_out = reg_addr_out;

endmodule
