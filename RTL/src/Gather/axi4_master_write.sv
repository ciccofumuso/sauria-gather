`timescale 1ns/1ps
module axi4_master_write #(
    parameter int unsigned ADDR_W = 32,
    parameter int unsigned DATA_W = 64
)(
    input  logic clk_i,
    input  logic rst_ni,

    // Interfaccia Acceleratore
    input  logic              start,
    input  logic [ADDR_W-1:0] addr_in,
    input  logic [7:0]        len_in,
    input  logic [2:0]        size_in,
    input  logic [DATA_W-1:0] data_in,     // Dato da scrivere 
	input  logic [DATA_W/8-1:0] w_strb_last,
    output logic              next_data_out, // Richiesta nuovo dato per il burst
    output logic              done,

    // Interfaccia verso l'esterno
    output logic [ADDR_W-1:0] aw_addr,
    output logic [7:0]        aw_len,
    output logic [2:0]        aw_size,
    output logic [1:0]        aw_burst,
    output logic              aw_valid,
    input  logic              aw_ready,

    output logic [DATA_W-1:0] w_data,
    output logic [DATA_W/8-1:0] w_strb,
    output logic              w_valid,
    input  logic              w_ready,
    output logic              w_last,

    input  logic [1:0]        b_resp,
    input  logic              b_valid,
    output logic              b_ready
);

logic addr_en, addr_rst_n, len_en, len_rst_n, size_en, size_rst_n;
logic cnt_en, cnt_rst_n;
logic [7:0] addr_rd_cnt;
logic is_last_beat, data_req;
logic w_fire;

assign aw_burst = 2'b01; // INCR
assign next_data_out = data_req;


// Control Unit
typedef enum logic [2:0] {
	IDLE,       // Attesa dello Start
	START,      // Campionamento parametri
	AW_REQ,     // Fase Address Write
	WR_DATA,     // Fase invio dati (Burst)
	B_WAIT,     // Attesa di risposta dallo Slave
	DONE        // Fine transazione
} state_e;

state_e current_state, next_state;

assign w_strb = (current_state == WR_DATA && is_last_beat) ? w_strb_last : '1;
assign w_fire = (current_state == WR_DATA) && start && w_ready;

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
			addr_en     = 1'b0;
			addr_rst_n 	= 1'b0;
			len_en      = 1'b0;
			len_rst_n  	= 1'b0;
			size_en     = 1'b0; 
			size_rst_n 	= 1'b0;
			cnt_en		= 1'b0;
			cnt_rst_n	= 1'b0;
			w_data		= 64'd0;
			data_req    = 1'b0;
			done        = 1'b0;
			aw_valid    = 1'b0;
			w_valid     = 1'b0;
			w_last      = is_last_beat;
			b_ready     = 1'b0;
			if (start) begin
				next_state = START;
			end else begin
				next_state = IDLE;
			end
		end
		
		START: begin
			next_state = AW_REQ;
			addr_en     = 1'b1;
			addr_rst_n 	= 1'b1;
			len_en      = 1'b1;
			len_rst_n  	= 1'b1;
			size_en     = 1'b1; 
			size_rst_n 	= 1'b1;
			cnt_en		= 1'b0;
			cnt_rst_n	= 1'b1;
			w_data		= 64'd0;
			data_req    = 1'b0;
			done        = 1'b0;
			aw_valid    = 1'b0;
			w_valid     = 1'b0;
			w_last      = is_last_beat;
			b_ready     = 1'b0;
		end

		AW_REQ: begin
			if (aw_ready) begin
				next_state = WR_DATA;
			end else begin
				next_state = AW_REQ;
			end
			addr_en     = 1'b0;
			addr_rst_n 	= 1'b1;
			len_en      = 1'b0;
			len_rst_n  	= 1'b1;
			size_en     = 1'b0; 
			size_rst_n 	= 1'b1;
			cnt_en		= 1'b0;
			cnt_rst_n	= 1'b1;
			w_data		= 64'd0;
			data_req    = 1'b0;
			done        = 1'b0;
			aw_valid    = 1'b1;
			w_valid     = 1'b0;
			w_last      = is_last_beat;
			b_ready     = 1'b0;
		end
		
		WR_DATA: begin
			if (w_fire) begin
				if (is_last_beat) begin
					cnt_en		= 1'b0;
					next_state = B_WAIT;
				end else begin
					cnt_en		= 1'b1;
					next_state = WR_DATA;
				end
			end else begin
				next_state = WR_DATA;
				cnt_en		= 1'b0;
			end
			addr_en     = 1'b0;
			addr_rst_n 	= 1'b1;
			len_en      = 1'b0;
			len_rst_n  	= 1'b1;
			size_en     = 1'b0; 
			size_rst_n 	= 1'b1;
			cnt_rst_n	= 1'b1;
			w_data   	= data_in;
			data_req    = w_fire; // Richiedi/consuma un dato solo su handshake reale
			done        = 1'b0;
			aw_valid    = 1'b0;
			w_valid     = start;
			w_last      = is_last_beat;
			b_ready     = 1'b0;
		end
		
		B_WAIT: begin
			if (b_valid) begin
				next_state = DONE;
			end else begin
				next_state = B_WAIT;
			end
			addr_en     = 1'b0;
			addr_rst_n 	= 1'b1;
			len_en      = 1'b0;
			len_rst_n  	= 1'b1;
			size_en     = 1'b0; 
			size_rst_n 	= 1'b1;
			cnt_en		= 1'b0;
			cnt_rst_n	= 1'b1;
			w_data		= 64'd0;
			data_req    = 1'b0;
			done        = 1'b0;
			aw_valid    = 1'b0;
			w_valid     = 1'b0;
			w_last      = is_last_beat;
			b_ready     = 1'b1;
		end
		
		DONE: begin
			addr_en     = 1'b0;
			addr_rst_n 	= 1'b0;
			len_en      = 1'b0;
			len_rst_n  	= 1'b0;
			size_en     = 1'b0; 
			size_rst_n 	= 1'b0;
			cnt_en		= 1'b0;
			cnt_rst_n	= 1'b0;
			w_data		= 64'd0;
			data_req    = 1'b0;
			done        = 1'b1;
			aw_valid    = 1'b0;
			w_valid     = 1'b0;
			w_last      = is_last_beat;
			b_ready     = 1'b0;
			next_state	= IDLE;
		end

		default: begin
			next_state = IDLE;
			addr_en     = 1'b0;
			addr_rst_n 	= 1'b0;
			len_en      = 1'b0;
			len_rst_n  	= 1'b0;
			size_en     = 1'b0; 
			size_rst_n 	= 1'b0;
			cnt_en		= 1'b0;
			cnt_rst_n	= 1'b0;
			w_data		= 64'd0;
			data_req    = 1'b0;
			done        = 1'b0;
			aw_valid    = 1'b0;
			w_valid     = 1'b0;
			w_last      = is_last_beat;
			b_ready     = 1'b0;
		end
	endcase
end


// Registri Parametri
logic [ADDR_W-1:0] addr_offset;

always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        addr_offset <= '0;
    end else begin
        if (current_state == B_WAIT && b_valid && b_ready) begin
			addr_offset <= addr_offset + ((32'(aw_len) + 32'd1) << 3);
		end
    end
end

// Registri Parametri modificati con applicazione dell'offset
always_ff @(posedge clk_i or negedge addr_rst_n) begin
	if (!addr_rst_n) begin
		aw_addr <= '0;
	end else if (addr_en) begin
		aw_addr <= addr_in + addr_offset; // Somma l'offset progressivo alla base statica!
	end
end

always_ff @(posedge clk_i  or negedge len_rst_n) begin
	if (!len_rst_n) begin
		aw_len <= '0;
	end else if (len_en) begin
		aw_len <= len_in;
	end
end


always_ff @(posedge clk_i  or negedge size_rst_n) begin
	if (!size_rst_n) begin
		aw_size <= '0;
	end else if (size_en) begin
		aw_size <= size_in;
	end
end

// Contatore per generare w_last
always_ff @(posedge clk_i  or negedge cnt_rst_n) begin
	if (!cnt_rst_n) begin
		addr_rd_cnt <= '0;
	end else if (cnt_en) begin
		addr_rd_cnt <= addr_rd_cnt + 1;
	end
end
always_comb begin
	if (current_state == WR_DATA) begin
		is_last_beat = (addr_rd_cnt == aw_len);
	end else begin
		is_last_beat = '0;
	end
end 

endmodule
