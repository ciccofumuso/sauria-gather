module double_buffer_top #(
    parameter int N_in  	= 16,
    parameter int N_out 	= 64,
    parameter int BIT_ADDR 	= 4
)(
    input  logic                 clk,
    input  logic                 rst_n,
    
    // Interfaccia Acceleratore
    input  logic [N_in-1:0]    	data_in,
    input  logic                data_in_valid,
    input  logic                data_in_last,
    output logic                wait_wr,
    
    // Interfaccia AXI Write
    input  logic                out_ready,
    output logic [N_out-1:0]   	data_out,
    output logic                out_valid,
	output logic [(N_out/8)-1:0] wstrb_last,
    output logic [7:0] 			rd_len
);

    // Segnali interni di interconnessione
    logic [N_out-1:0] 	sipo_to_buf_data;
	logic [(N_out/8)-1:0] sipo_to_buf_strb;
    logic               sipo_to_buf_wr;
    logic               buf_to_sipo_wait;
	logic 				sipo_to_buf_last;

    // Istanza del modulo SIPO
    sipo #(
        .N_in(N_in),
        .N_out(N_out)
    ) sipo_in (
        .clk           (clk),
        .rst_n         (rst_n),
        .data_in       (data_in),
        .data_in_valid (data_in_valid),
        .data_in_last  (data_in_last),
        .wait_wr       (buf_to_sipo_wait), // Riceve il wait dal buffer
        .data_out      (sipo_to_buf_data),
		.wr_strb       (sipo_to_buf_strb),
        .wr_en         (sipo_to_buf_wr),    // Trigger per la scrittura
		.last_out	   (sipo_to_buf_last)
	);

    // Istanza del modulo Double Buffer
    double_buffer #(
        .M(BIT_ADDR),
        .N(N_out)
    ) buffer (
        .clk           (clk),
        .rst_n         (rst_n),
        .data_in_valid (sipo_to_buf_wr),    // Validato dal SIPO
        .data_in_last  (sipo_to_buf_last),    // Indica se la riga corrente è l'ultima
        .data_in       (sipo_to_buf_data),
		.data_in_strb  (sipo_to_buf_strb),
        .wait_wr       (buf_to_sipo_wait), // Genera il wait se pieno
        .out_ready     (out_ready),
        .data_out      (data_out),
        .out_valid     (out_valid),
		.wstrb_last    (wstrb_last),
        .rd_len        (rd_len)
    );

    // Output di wait verso l'acceleratore
    assign wait_wr = buf_to_sipo_wait;

endmodule
