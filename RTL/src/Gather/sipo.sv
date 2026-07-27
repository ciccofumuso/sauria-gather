`timescale 1ns/1ps

module sipo #(
    parameter int N_in  = 16,       // Larghezza della singola word
    parameter int N_out = 64        // Larghezza totale in uscita
)(
    input  logic                 clk,
    input  logic                 rst_n,
    
    // Interfaccia Produttore
    input  logic [N_in-1:0]    data_in,
    input  logic                 data_in_valid,
    input  logic                 data_in_last,
    input  logic                 wait_wr,   // Se 1, blocca l'avanzamento
    
    // Interfaccia Uscita
    output logic [N_out-1:0]   data_out,
	output logic [(N_out/8)-1:0] wr_strb,
    output logic                 wr_en,      
	output logic                 last_out
);

    localparam int NUM_STAGES = N_out / N_in; // Do per scontato che le word siano più piccole di 64bit
	localparam int COUNT_WIDTH = $clog2(NUM_STAGES);
    
    logic [N_in-1:0] pipe_regs [0:NUM_STAGES-1];
    logic [COUNT_WIDTH-1:0] count;

    // --- LOGICA DI SALVATAGGIO E CONTROLLO ---
    always_ff @(posedge clk or negedge rst_n) begin
	        if (!rst_n) begin
	            for (int i = 0; i < NUM_STAGES; i++) pipe_regs[i] <= '0;
	            count <= '0;
	            wr_en <= 1'b0; // Reset del segnale di scrittura
				wr_strb <= '0;
				last_out <= 1'b0;
	        end else begin
            if (!wait_wr) begin
                // Gestione dei dati
                if (data_in_valid) begin
                    pipe_regs[count] <= data_in;
                    
                    // Incremento o reset del contatore
                    if (count == COUNT_WIDTH'(NUM_STAGES-1) || data_in_last) begin
                        count <= '0;
                    end else begin
                        count <= count + 1;
                    end
                end else if (data_in_last && count > 0) begin
                    count <= '0;
                end

                // Generazione di wr_en
                // Questo segnale diventa 1 il ciclo DOPO che l'ultima word è stata salvata
                wr_en <= (data_in_valid && (count == COUNT_WIDTH'(NUM_STAGES-1) || data_in_last)) || (!data_in_valid && data_in_last && count > 0);
                last_out <= data_in_last;
				
				if (data_in_valid && (count == COUNT_WIDTH'(NUM_STAGES-1) || data_in_last)) begin
                    for (int i = 0; i < NUM_STAGES; i++) begin
                        if (i <= count) begin
                            wr_strb[i*(N_in/8) +: (N_in/8)] <= { (N_in/8){1'b1} };
                        end else begin
                            wr_strb[i*(N_in/8) +: (N_in/8)] <= '0;
                        end
                    end
                end else if (!data_in_valid && data_in_last && count > 0) begin
                    for (int i = 0; i < NUM_STAGES; i++) begin
                        if (i < count) begin
                            wr_strb[i*(N_in/8) +: (N_in/8)] <= { (N_in/8){1'b1} };
                        end else begin
                            wr_strb[i*(N_in/8) +: (N_in/8)] <= '0;
                        end
                    end
                end else begin
                    wr_strb <= '1; // Default a buffer pieno (tutti i byte validi)
                end
            end
        end
    end

    // --- USCITA DATI ---
    // data_out cambia insieme a wr_en
    always_comb begin
        for (int k = 0; k < NUM_STAGES; k++) begin
            data_out[k*N_in +: N_in] = pipe_regs[k];
        end
    end

endmodule
