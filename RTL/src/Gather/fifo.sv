`timescale 1ns/1ps
module fifo #(
    parameter int DATA_WIDTH = 64,      // Larghezza del dato (64 bit)
    parameter int DEPTH      = 16,      // Numero di linee
	parameter int THRESHOLD  = 8		// Soglia impostata a metà
)(
    input  logic                   clk,      // Clock di sistema
    input  logic                   rst_n,    // Reset asincrono attivo basso
    
    // Interfaccia di Scrittura
    input  logic                   w_en,     // Write Enable
    input  logic [DATA_WIDTH-1:0]  w_data,   // Dati in ingresso
    
    // Interfaccia di Lettura
    input  logic                   r_en,     // Read Enable
    output logic [DATA_WIDTH-1:0]  r_data,   // Dati in uscita
    
    // Segnali di Stato
    output logic                   empty,    // FIFO vuota
    output logic                   threshold // FIFO che supera una soglia
);

// Calcolo dei bit necessari per gli indirizzi
localparam int ADDR_W = $clog2(DEPTH);

// Memoria interna (Array)
logic [DATA_WIDTH-1:0] mem [DEPTH];

// Puntatori e Contatore
logic [ADDR_W-1:0] w_ptr;   // Puntatore di scrittura
logic [ADDR_W-1:0] r_ptr;   // Puntatore di lettura
logic [ADDR_W:0]   count;   // Contatore elementi

// Controllo FIFO piena
logic	full;

// --- Gestione Scrittura e Lettura ---
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		w_ptr   <= '0;
		r_ptr   <= '0;
		count   <= '0;
		r_data  <= '0;
		for(int i = 0; i < $size(mem); i++) mem[i] <= '0;
	end else begin
		// Logica di Scrittura
		if (w_en && !full) begin
			mem[w_ptr] <= w_data;
			if (w_ptr == ADDR_W'(DEPTH - 1)) begin
				w_ptr <= '0;
			end else begin
				w_ptr <= w_ptr + 1'b1;
			end
		end

		// Logica di Lettura
		if (r_en && !empty) begin
			r_data <= mem[r_ptr];
			if (r_ptr == ADDR_W'(DEPTH - 1)) begin
				r_ptr <= '0;
			end else begin
				r_ptr <= r_ptr + 1'b1;
			end
		end

		// Aggiornamento del contatore elementi
		if ((w_en && !full) && !(r_en && !empty)) begin
			count <= count + 1'b1;
		end else if (!(w_en && !full) && (r_en && !empty)) begin
			count <= count - 1'b1;
		end
	end
end

// --- Logica degli stati ---
assign full      = (count == ((ADDR_W+1)'(DEPTH)));
assign empty     = (count == ((ADDR_W+1)'(0)));
assign threshold = (count >= ((ADDR_W+1)'(THRESHOLD)));

endmodule
