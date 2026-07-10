`timescale 1ns/1ps

module piso #(
    parameter int N_DATA_IN = 64,       // Dimensione del dato in ingresso dalla FIFO
    parameter int WORD_SIZE = 16        // Dimensione della singola word in uscita
)(
    input  logic                    clk, 
    input  logic                    rst_n, 
 
    // Interfaccia di Controllo
    input  logic                    load,         // Segnale di load (sticky) 
    input  logic                    shift_en,     // Richiede la parola successiva dalla PISO attiva
    
    // Interfaccia FIFO
    input  logic [N_DATA_IN-1:0]    data_in,      // Collegato a fifo_r_data del DP 
    input  logic                    fifo_empty,   // Segnale di FIFO vuota 
    output logic                    fifo_r_en,    // Segnale di lettura per la FIFO 
    
    // Uscite
    output logic [WORD_SIZE-1:0]    data_out,     // Word in uscita 
    output logic                    empty         // Entrambe le PISO vuote
);

localparam int N_WORD = N_DATA_IN / WORD_SIZE; 
localparam int CNT_W  = $clog2(N_WORD) + 1; 

// --- Registri Dati delle due PISO ---
logic [N_DATA_IN-1:0]   piso0_reg, piso1_reg; 
logic [CNT_W-1:0]       piso0_words_left, piso1_words_left; 

// --- Flag di Controllo Flusso ---
logic                   piso0_full, piso1_full;
logic                   piso0_req, piso1_req;
logic                   piso0_data_ready, piso1_data_ready;

logic                   sticky_load;
logic                   active_piso;      // PISO preferenziale al ciclo precedente 
logic                   current_piso_sel; // Selezione combinatoria

// Il sistema globale è vuoto se nessuna delle due PISO ha dati validi pronti
assign empty = !piso0_full && !piso1_full;

// Ultimo shift in questo ciclo, la PISO sarà vuota al prossimo ciclo
logic piso0_empty, piso1_empty;
assign piso0_empty = (shift_en && (current_piso_sel == 1'b0) && piso0_full && (piso0_words_left == 1));
assign piso1_empty = (shift_en && (current_piso_sel == 1'b1) && piso1_full && (piso1_words_left == 1));

// Flag se la PISO è già stata allocata
logic piso0_alloc, piso1_alloc;
assign piso0_alloc = (piso0_full && !piso0_empty) || piso0_req || piso0_data_ready;
assign piso1_alloc = (piso1_full && !piso1_empty) || piso1_req || piso1_data_ready;

// ========================================================
// LOGICA STICKY LOAD
// ========================================================
always_ff @(posedge clk or negedge rst_n) begin 
	if (!rst_n) begin 
		sticky_load <= 1'b0; 
	end else if (load) begin 
		sticky_load <= 1'b1; 
	end 
end 

// ========================================================
// LOGICA DI CONTROLLO INTERNA, SHIFT E LETTURA FIFO
// ========================================================
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		fifo_r_en        	<= 1'b0;
		piso0_req  			<= 1'b0;
		piso0_data_ready 	<= 1'b0;
		piso0_full       	<= 1'b0;
		piso0_reg        	<= '0;
		piso0_words_left 	<= '0;
		
		piso1_req  			<= 1'b0;
		piso1_data_ready 	<= 1'b0;
		piso1_full       	<= 1'b0;
		piso1_reg        	<= '0;
		piso1_words_left 	<= '0;
	end else begin
		fifo_r_en <= 1'b0;

		// ----------------------------------------------------
		// GESTIONE AVANZAMENTO DATI PISO 0
		// ----------------------------------------------------
		if (piso0_req) begin
			piso0_req  			<= 1'b0;
			piso0_data_ready 	<= 1'b1; // Al prossimo clock il dato esce dalla FIFO
		end
		
		if (piso0_data_ready) begin
			piso0_data_ready <= 1'b0;
			piso0_reg        <= data_in; // Campionamento sicuro del dato valido
			piso0_words_left <= CNT_W'(N_WORD);
			piso0_full       <= 1'b1;
		end

		// ----------------------------------------------------
		// GESTIONE AVANZAMENTO DATI PISO 1
		// ----------------------------------------------------
		if (piso1_req) begin
			piso1_req  			<= 1'b0;
			piso1_data_ready 	<= 1'b1;
		end
		
		if (piso1_data_ready) begin
			piso1_data_ready <= 1'b0;
			piso1_reg        <= data_in;
			piso1_words_left <= CNT_W'(N_WORD);
			piso1_full       <= 1'b1;
		end

		// ----------------------------------------------------
		// LOGICA DI SHIFT 
		// ----------------------------------------------------
		if (shift_en && (current_piso_sel == 1'b0) && piso0_full) begin
			if (piso0_words_left == 1) begin
				piso0_words_left <= '0;
				piso0_full       <= 1'b0; 
			end else begin
				piso0_reg        <= piso0_reg >> WORD_SIZE;
				piso0_words_left <= piso0_words_left - 1'b1;
			end
		end

		if (shift_en && (current_piso_sel == 1'b1) && piso1_full) begin
			if (piso1_words_left == 1) begin
				piso1_words_left <= '0;
				piso1_full       <= 1'b0; 
			end else begin
				piso1_reg        <= piso1_reg >> WORD_SIZE;
				piso1_words_left <= piso1_words_left - 1'b1;
			end
		end

		// ----------------------------------------------------
		// LOGICA DI RICHIESTA ALLA FIFO
		// ----------------------------------------------------
		if (sticky_load && !fifo_empty) begin
			// Priorità di riempimento alla PISO 0 se non è allocata
			if (!piso0_alloc) begin
				fifo_r_en       <= 1'b1;
				piso0_req 		<= 1'b1;
			// Altrimenti, se libera, riempie in background la PISO 1
			end else if (!piso1_alloc) begin
				fifo_r_en       <= 1'b1;
				piso1_req 		<= 1'b1;
			end
		end
	end
end

// ========================================================
// LOGICA COMBINATORIA DI SELEZIONE
// ========================================================
always_comb begin
	if (active_piso == 1'b0) begin
		// Se la PISO 0 si svuota ed è pronta la PISO 1
		if (!piso0_full && piso1_full)
			current_piso_sel = 1'b1;
		else
			current_piso_sel = 1'b0;
	end else begin
		// Se la PISO 1 si svuota ed è pronta la PISO 0
		if (!piso1_full && piso0_full)
			current_piso_sel = 1'b0;
		else
			current_piso_sel = 1'b1;
	end
end

// Aggiornamento dello stato sequenziale di controllo
always_ff @(posedge clk or negedge rst_n) begin 
	if (!rst_n) begin 
		active_piso <= 1'b0; 
	end else begin 
		active_piso <= current_piso_sel;
	end 
end 

// ========================================================
// ASSEGNAZIONE DATA OUT (MUX)
// ========================================================
assign data_out = (current_piso_sel == 1'b0) ? piso0_reg[WORD_SIZE-1:0] : piso1_reg[WORD_SIZE-1:0];

endmodule
