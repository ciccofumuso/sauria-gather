module double_buffer #(
    parameter int M = 4,         // Bit Address (profondità buffer 2**M) 
    parameter int N = 64         // Data width (64 bit) 
)(
    input  logic             clk,
    input  logic             rst_n,
    
    // Produttore (Acceleratore)
    input  logic             	data_in_valid,
    input  logic             	data_in_last,    // Segnale di fine dati 
    input  logic [N-1:0]     	data_in,
	input  logic [(N/8)-1:0]    data_in_strb,
    output logic             	wait_wr,    
    
    // Consumatore (Uscita)
    input  logic             	out_ready,
    output logic [N-1:0] 	    data_out,
    output logic             	out_valid,
	output logic [(N/8)-1:0]    wstrb_last,
	output logic [7:0]			rd_len           // Numero di righe valide da leggere 
);

// Memoria interna (Buffer A e B)
logic [N-1:0] mem_A [0:(2**M)-1];
logic [N-1:0] mem_B [0:(2**M)-1];

// Flag di stato e registri per il numero di righe valide 
logic buf_A_full, buf_B_full;
logic [M-1:0] count_A, count_B;
logic [(N/8)-1:0] wstrb_last_A, wstrb_last_B;

// Puntatori di scrittura/lettura e selettori dei buffer 
logic [M-1:0] wr_ptr, rd_ptr;
logic         wr_sel, rd_sel; // 0 -> Buffer A, 1 -> Buffer B 

// --- LOGICA DI SCRITTURA ---	
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		count_A			<= '0;
		count_B         <= '0;
		wr_ptr			<= '0;
		wr_sel 			<= '0;
		buf_A_full		<= '0;
		buf_B_full 		<= '0;
		wstrb_last_A    <= '1;
		wstrb_last_B    <= '1;
		for(int i = 0; i < $size(mem_A); i++) begin
			mem_A[i] <= '0;
			mem_B[i] <= '0;
		end
	end else begin
        // Scrittura diretta se il dato è valido e il buffer non è in wait 
		if (data_in_valid && !wait_wr) begin
			if (wr_sel == 0) begin
				mem_A[wr_ptr] <= data_in;
			end else begin
				mem_B[wr_ptr] <= data_in;
			end

			// Chiusura Buffer se arriva l'ultimo dato o se la memoria è piena 
			if (data_in_last || wr_ptr == (2**M)-1) begin
				if (wr_sel == 0) begin
					buf_A_full <= 1;
					count_A    <= wr_ptr;
					wstrb_last_A <= data_in_strb;
				end else begin
					buf_B_full <= 1;
					count_B    <= wr_ptr;
					wstrb_last_B <= data_in_strb;
				end
				wr_ptr <= 0;
				wr_sel <= !wr_sel; // Cambio del buffer di scrittura 
			end else begin
				wr_ptr <= wr_ptr + 1;
			end
		end else if (data_in_last && !wait_wr) begin
            // Caso di chiusura forzata del buffer senza un nuovo dato valido in ingresso ma con dati conservati:
            if (wr_ptr != 0) begin
                if (wr_sel == 0) begin
                    buf_A_full <= 1;
                    count_A    <= wr_ptr - 1;
					wstrb_last_A <= data_in_strb;
                end else begin
                    buf_B_full <= 1;
                    count_B    <= wr_ptr - 1;
					wstrb_last_B <= data_in_strb;
                end
                wr_ptr <= 0;
                wr_sel <= !wr_sel; // Cambiamo buffer solo se abbiamo effettuato il flush di dati reali
            end
        end

		// Liberazione del buffer dopo che il driver ha terminato la lettura 
		if (buf_A_full && rd_sel == 0 && out_ready && out_valid && rd_ptr == count_A) begin
			buf_A_full <= 0;
		end
		if (buf_B_full && rd_sel == 1 && out_ready && out_valid && rd_ptr == count_B) begin
			buf_B_full <= 0;
		end
	end
end

// --- LOGICA DI LETTURA ---
always_ff @(posedge clk  or negedge rst_n) begin
	if (!rst_n) begin
		rd_ptr <= 0;
		rd_sel <= 0; 
	end else if (out_valid && out_ready) begin
        // Verifica se è stata raggiunta l'ultima riga valida del buffer corrente 
		if ((rd_sel && rd_ptr == count_B) || (!rd_sel && rd_ptr == count_A)) begin
			rd_ptr <= 0;
			rd_sel <= !rd_sel; // Passaggio al buffer successivo 
		end else begin
			rd_ptr <= rd_ptr + 1; 
		end
	end
end
		
// --- LOGICA COMBINATORIA ---
always_comb begin
	// Gestione del segnale di wait basata sul buffer attualmente selezionato 
	if (wr_sel) begin
		wait_wr = buf_B_full;
	end else begin
		wait_wr = buf_A_full;
	end

	// Selezione dei dati e dei segnali di stato per l'uscita 
	if (rd_sel) begin
		out_valid = buf_B_full;
		data_out  = mem_B[rd_ptr];
		rd_len 	  = 8'(count_B);
		wstrb_last = wstrb_last_B;
	end else begin
		out_valid = buf_A_full;
		data_out  = mem_A[rd_ptr];
		rd_len 	  = 8'(count_A);
		wstrb_last = wstrb_last_A;		
	end
end

endmodule
