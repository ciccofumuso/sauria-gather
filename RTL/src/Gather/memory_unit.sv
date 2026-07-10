module memory_unit #(
	parameter int unsigned NumWords_idx	= 16,   // Numero di word totali nella SRAM degli indici compressi
	parameter int unsigned ByteWidth_idx= 32,    // Larghezza del singolo byte di indici compressi 
    parameter int unsigned NumWords  	= 16,   // Numero di word totali nella SRAM singola
	parameter int unsigned DataWidth 	= 64,   // Larghezza del bus dati (64 bit)
	parameter int unsigned ByteWidth 	= 32,    // Larghezza del singolo byte 
	parameter int unsigned NumPorts  	= 1,    // Numero di porte
	parameter int unsigned Latency   	= 1,    // Latenza di lettura 
	parameter int N_BLOCKS = 8
)(
    input  	logic                 	clk_i,
    input  	logic                 	rst_ni,
    // Memoria matrice densa
    input 	logic [$clog2(NumWords)-1:0]			addr,
	input	logic [$clog2(DataWidth/ByteWidth)-1:0]	byte_sel,
	input	logic [DataWidth-1:0]					data_in,
	input	logic [$clog2(N_BLOCKS)-1:0]			mem_en,
	input	logic 									wr_en,
	output	logic [ByteWidth-1:0]					data_out,
	// Memoria indici compressi
	input 	logic [$clog2(NumWords_idx)-1:0]			idx_addr,
	input	logic [$clog2(DataWidth/ByteWidth_idx)-1:0]	idx_byte_sel,
	input	logic [DataWidth-1:0]						idx_data_in,
	input	logic 										idx_mem_en,
	input	logic 										idx_wr_en,
	output	logic [ByteWidth_idx-1:0]					idx_data_out
);
localparam int unsigned WORDS_PER_LINE = DataWidth / ByteWidth;
localparam int unsigned BeWidth   = (DataWidth + ByteWidth - 32'd1) / ByteWidth; 

localparam int unsigned INDEX_PER_LINE = DataWidth / ByteWidth_idx;
localparam int unsigned BeWidth_idx   = (DataWidth + ByteWidth_idx - 32'd1) / ByteWidth_idx;

// Segnali interni della memoria della matrice densa
logic [BeWidth-1:0]				be_i;
logic [N_BLOCKS-1:0]			decoded_en;
logic [N_BLOCKS-1:0][DataWidth-1:0]						single_mem_data_out;
logic [N_BLOCKS-1:0][WORDS_PER_LINE-1:0][ByteWidth-1:0]	words_array;
logic [N_BLOCKS-1:0][ByteWidth-1:0]						single_mem_byte_out;

// Segnali interni della memoria della matrice densa
logic [BeWidth_idx-1:0]						idx_be_i;
logic [INDEX_PER_LINE-1:0][ByteWidth_idx-1:0]	idx_mem_data_out;


//====================================================
//Decoder degli enable della memoria
//====================================================
always_comb begin
	decoded_en			= '0;
	decoded_en[mem_en] 	= 1'b1;
end

//====================================================
// Istanza dei blocchi di memoria per la matrice densa
//====================================================
assign be_i = '1;
genvar i ;
generate
	for (i = 0; i < N_BLOCKS; i++) begin : gen_memory_blocks
		tc_sram #(
		.NumWords   (NumWords),
		.DataWidth  (DataWidth),
		.ByteWidth  (ByteWidth),
		.NumPorts   (NumPorts),
		.Latency    (Latency)
		) block_sram (
		.clk_i      (clk_i),
		.rst_ni     (rst_ni),
		.req_i      (decoded_en[i]),
		.we_i       (wr_en),
		.addr_i     (addr),
		.wdata_i    (data_in),
		.be_i       (be_i),
		.rdata_o    (single_mem_data_out[i][DataWidth-1:0])
		);
	end
endgenerate

//==================================================================
// Istanza dei mux per la selezione della word dalla sram singola
//==================================================================
assign words_array = single_mem_data_out;
always_comb begin
    for (int j = 0; j < N_BLOCKS; j++) begin
        // Per ogni blocco i, seleziona la word indicata da byte_sel
        single_mem_byte_out[j] = words_array[j][byte_sel];
    end
end

//==================================================================
// Istanza dei mux per la selezione della word dai blocchi di sram
//==================================================================
assign data_out = single_mem_byte_out[mem_en];


//==================================================================
// Istanza della memoria per gli indici compressi
//==================================================================
assign idx_be_i = '1;

tc_sram #(
.NumWords   (NumWords_idx),
.DataWidth  (DataWidth),
.ByteWidth  (ByteWidth_idx),
.NumPorts   (NumPorts),
.Latency    (Latency)
) comp_idx_sram (
.clk_i      (clk_i),
.rst_ni     (rst_ni),
.req_i      (idx_mem_en),
.we_i       (idx_wr_en),
.addr_i     (idx_addr),
.wdata_i    (idx_data_in),
.be_i       (idx_be_i),
.rdata_o    (idx_mem_data_out)
);

//==================================================================
// Istanza del mux per la selezione dell'indice compresso
//==================================================================
assign idx_data_out = idx_mem_data_out[idx_byte_sel];

endmodule
