`timescale 1ns/1ps

module gather_frontend_axi #(
    parameter int unsigned CFG_AXI_DATA_WIDTH  = 32,
    parameter int unsigned CFG_AXI_ADDR_WIDTH  = 32,

    parameter int unsigned DATA_AXI_DATA_WIDTH = 64,
    parameter int unsigned DATA_AXI_ADDR_WIDTH = 32,
    parameter int unsigned DATA_AXI_ID_WIDTH   = 2,

    parameter int FIFO_DEPTH       = 16,
    parameter int FIFO_THRESHOLD   = 14,
    parameter int unsigned NumWords_idx  = 64,
    parameter int unsigned ByteWidth_idx = 32,
    parameter int unsigned NumWords      = 128,
    parameter int unsigned ByteWidth     = 16,
    parameter int unsigned NumPorts      = 1,
    parameter int unsigned Latency       = 1,
    parameter int N_BLOCKS               = 16,
    parameter int BUFFER_BIT_ADDR        = 3
)(
    input  logic i_clk,
    input  logic i_rstn,

    AXI_LITE.Slave cfg_slv,
    AXI_BUS.Master ext_mst,
	AXI_BUS.Master sauria_mst,

    output logic o_doneintr, o_busy
);

localparam int unsigned DATA_AXI_BYTE_NUM = DATA_AXI_DATA_WIDTH / 8;
localparam logic [2:0] DEFAULT_AXI_SIZE   = $clog2(DATA_AXI_BYTE_NUM);

// ---------------------------------------------------------------------
// Register map locale del gather
// ---------------------------------------------------------------------

localparam logic [7:0] REG_START                 = 8'h00;
localparam logic [7:0] REG_IS_SPMM               = 8'h04;
localparam logic [7:0] REG_DONE                  = 8'h08;
localparam logic [7:0] REG_AR_SIZE               = 8'h0C;
localparam logic [7:0] REG_AW_SIZE               = 8'h10;
localparam logic [7:0] REG_AR_ADDR_DENSE_MATRIX  = 8'h14;
localparam logic [7:0] REG_TOTAL_LEN_DENSE_MATRIX= 8'h18;
localparam logic [7:0] REG_AR_ADDR_COMP_IDX      = 8'h1C;
localparam logic [7:0] REG_TOTAL_LEN_COMP_IDX    = 8'h20;
localparam logic [7:0] REG_AR_ADDR_IDX           = 8'h24;
localparam logic [7:0] REG_TOTAL_LEN_IDX         = 8'h28;
localparam logic [7:0] REG_AXI_WR_AW_ADDR_IN     = 8'h2C;
localparam logic [7:0] REG_STATUS_REG_N_ROW      = 8'h30;

localparam logic [7:0] REG_PERF_TOTAL_CYCLES = 8'h34;
localparam logic [7:0] REG_PERF_RD_CYCLES    = 8'h38;
localparam logic [7:0] REG_PERF_WR_CYCLES    = 8'h3C;
localparam logic [7:0] REG_PERF_R_STALL      = 8'h40;
localparam logic [7:0] REG_PERF_W_STALL      = 8'h44;
localparam logic [7:0] REG_PERF_R_BEATS      = 8'h48;
localparam logic [7:0] REG_PERF_W_BEATS      = 8'h4C;

logic        is_spmm_q;
logic [2:0]  ar_size_q;
logic [2:0]  aw_size_q;

logic [DATA_AXI_ADDR_WIDTH-1:0] ar_addr_dense_matrix_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] total_len_dense_matrix_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] ar_addr_comp_idx_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] total_len_comp_idx_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] ar_addr_idx_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] total_len_idx_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] axi_wr_aw_addr_in_q;
logic [DATA_AXI_ADDR_WIDTH-1:0] status_reg_n_row_q;

logic perf_active;

logic [31:0] perf_total_cycles;
logic [31:0] perf_rd_cycles;
logic [31:0] perf_wr_cycles;
logic [31:0] perf_r_stall;
logic [31:0] perf_w_stall;
logic [31:0] perf_r_beats;
logic [31:0] perf_w_beats;

wire gather_start_fire = start_pulse;
wire gather_done_fire  = done;

wire r_fire = ext_mst.r_valid && ext_mst.r_ready;
wire w_fire = sauria_mst.w_valid && sauria_mst.w_ready;

wire read_active  = ext_mst.ar_valid || ext_mst.r_valid;
wire write_active = sauria_mst.aw_valid || sauria_mst.w_valid || sauria_mst.b_valid;

logic start_pulse;
logic done;
logic done_sticky;
logic busy_q;
// ---------------------------------------------------------------------
// AXI-Lite slave minimale
// ---------------------------------------------------------------------

logic        wr_fire;
logic        rd_fire;
logic [31:0] rd_data_q;
logic [31:0] rd_data_d;

// AXI-Lite write channels AW and W are independent.  After the SAURIA
// AXI-Lite demux, AW and W may reach this slave in different cycles, so
// we must buffer them separately.  The previous implementation required
// AW and W to be valid in the same cycle, therefore register writes could
// be silently lost and START never reached accel_top.
logic aw_buf_valid;
logic w_buf_valid;
logic [CFG_AXI_ADDR_WIDTH-1:0] aw_addr_q;
logic [CFG_AXI_DATA_WIDTH-1:0] w_data_q;

logic aw_accept;
logic w_accept;
logic [CFG_AXI_ADDR_WIDTH-1:0] wr_addr;
logic [CFG_AXI_DATA_WIDTH-1:0] wr_data;

assign cfg_slv.aw_ready = !aw_buf_valid && !cfg_slv.b_valid;
assign cfg_slv.w_ready  = !w_buf_valid  && !cfg_slv.b_valid;

assign aw_accept = cfg_slv.aw_valid && cfg_slv.aw_ready;
assign w_accept  = cfg_slv.w_valid  && cfg_slv.w_ready;

assign wr_fire = (aw_buf_valid || aw_accept) &&
                 (w_buf_valid  || w_accept)  &&
                 !cfg_slv.b_valid;

assign wr_addr = aw_accept ? cfg_slv.aw_addr : aw_addr_q;
assign wr_data = w_accept  ? cfg_slv.w_data  : w_data_q;

assign rd_fire = cfg_slv.ar_valid && cfg_slv.ar_ready;
assign cfg_slv.ar_ready = !cfg_slv.r_valid;

assign cfg_slv.b_resp = axi_pkg::RESP_OKAY;
assign cfg_slv.r_resp = axi_pkg::RESP_OKAY;
assign cfg_slv.r_data = rd_data_q;

always_ff @(posedge i_clk or negedge i_rstn) begin
	if (!i_rstn) begin
		cfg_slv.b_valid <= 1'b0;
		aw_buf_valid    <= 1'b0;
		w_buf_valid     <= 1'b0;
		aw_addr_q       <= '0;
		w_data_q        <= '0;
	end else begin

		// Store AW if it arrives before W.
		if (aw_accept && !wr_fire) begin
			aw_addr_q    <= cfg_slv.aw_addr;
			aw_buf_valid <= 1'b1;
		end

		// Store W if it arrives before AW.
		if (w_accept && !wr_fire) begin
			w_data_q     <= cfg_slv.w_data;
			w_buf_valid  <= 1'b1;
		end

		// Once both AW and W are available, perform exactly one register write
		// and return the AXI-Lite write response.
		if (wr_fire) begin
			cfg_slv.b_valid <= 1'b1;
			aw_buf_valid    <= 1'b0;
			w_buf_valid     <= 1'b0;
		end else if (cfg_slv.b_ready) begin
			cfg_slv.b_valid <= 1'b0;
		end
	end
end

always_ff @(posedge i_clk or negedge i_rstn) begin
	if (!i_rstn) begin
		cfg_slv.r_valid <= 1'b0;
		rd_data_q       <= 32'h0;
	end else begin
		if (rd_fire) begin
			cfg_slv.r_valid <= 1'b1;
			rd_data_q       <= rd_data_d;
		end else if (cfg_slv.r_ready) begin
			cfg_slv.r_valid <= 1'b0;
		end
	end
end

// ---------------------------------------------------------------------
// Scrittura registri
// ---------------------------------------------------------------------

always_ff @(posedge i_clk or negedge i_rstn) begin
	if (!i_rstn) begin
		is_spmm_q                <= 1'b0;
		ar_size_q                <= DEFAULT_AXI_SIZE;
		aw_size_q                <= DEFAULT_AXI_SIZE;

		ar_addr_dense_matrix_q   <= '0;
		total_len_dense_matrix_q <= '0;
		ar_addr_comp_idx_q       <= '0;
		total_len_comp_idx_q     <= '0;
		ar_addr_idx_q            <= '0;
		total_len_idx_q          <= '0;
		axi_wr_aw_addr_in_q      <= '0;
		status_reg_n_row_q       <= '0;

		start_pulse              <= 1'b0;
		done_sticky              <= 1'b0;
	end else begin
		start_pulse <= 1'b0;

		if (done) begin
			done_sticky <= 1'b1;
		end

		if (wr_fire) begin
			unique case (wr_addr[7:0])

				REG_START: begin
					if (wr_data[0]) begin
						start_pulse <= 1'b1;
						done_sticky <= 1'b0;
					end
				end

				REG_IS_SPMM: begin
					is_spmm_q <= wr_data[0];
				end

				REG_DONE: begin
					// write-1-to-clear del done sticky
					if (wr_data[0]) begin
						done_sticky <= 1'b0;
					end
				end

				REG_AR_SIZE: begin
					ar_size_q <= wr_data[2:0];
				end

				REG_AW_SIZE: begin
					aw_size_q <= wr_data[2:0];
				end

				REG_AR_ADDR_DENSE_MATRIX: begin
					ar_addr_dense_matrix_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_TOTAL_LEN_DENSE_MATRIX: begin
					total_len_dense_matrix_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_AR_ADDR_COMP_IDX: begin
					ar_addr_comp_idx_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_TOTAL_LEN_COMP_IDX: begin
					total_len_comp_idx_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_AR_ADDR_IDX: begin
					ar_addr_idx_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_TOTAL_LEN_IDX: begin
					total_len_idx_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_AXI_WR_AW_ADDR_IN: begin
					axi_wr_aw_addr_in_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				REG_STATUS_REG_N_ROW: begin
					status_reg_n_row_q <= wr_data[DATA_AXI_ADDR_WIDTH-1:0];
				end

				default: begin
					// Per ora ignoro scritture fuori mappa.
				end
			endcase
		end
	end
end

// ---------------------------------------------------------------------
// Lettura registri
// ---------------------------------------------------------------------

always_comb begin
	rd_data_d = 32'h0;

	unique case (cfg_slv.ar_addr[7:0])

		REG_START: begin
			rd_data_d = 32'h0;
		end

		REG_IS_SPMM: begin
			rd_data_d = {31'h0, is_spmm_q};
		end

		REG_DONE: begin
			rd_data_d = {31'h0, done_sticky};
		end

		REG_AR_SIZE: begin
			rd_data_d = {29'h0, ar_size_q};
		end

		REG_AW_SIZE: begin
			rd_data_d = {29'h0, aw_size_q};
		end

		REG_AR_ADDR_DENSE_MATRIX: begin
			rd_data_d = ar_addr_dense_matrix_q;
		end

		REG_TOTAL_LEN_DENSE_MATRIX: begin
			rd_data_d = total_len_dense_matrix_q;
		end

		REG_AR_ADDR_COMP_IDX: begin
			rd_data_d = ar_addr_comp_idx_q;
		end

		REG_TOTAL_LEN_COMP_IDX: begin
			rd_data_d = total_len_comp_idx_q;
		end

		REG_AR_ADDR_IDX: begin
			rd_data_d = ar_addr_idx_q;
		end

		REG_TOTAL_LEN_IDX: begin
			rd_data_d = total_len_idx_q;
		end

		REG_AXI_WR_AW_ADDR_IN: begin
			rd_data_d = axi_wr_aw_addr_in_q;
		end

		REG_STATUS_REG_N_ROW: begin
			rd_data_d = status_reg_n_row_q;
		end
		
		REG_PERF_TOTAL_CYCLES: begin
			rd_data_d = perf_total_cycles;
		end
		
		REG_PERF_RD_CYCLES: begin
			rd_data_d = perf_rd_cycles;
		end
		
		REG_PERF_WR_CYCLES: begin
			rd_data_d = perf_wr_cycles;
		end
		
		REG_PERF_R_STALL: begin
			rd_data_d = perf_r_stall;
		end
		
		REG_PERF_W_STALL: begin
			rd_data_d = perf_w_stall;
		end
		
		REG_PERF_R_BEATS: begin
			rd_data_d = perf_r_beats;
		end
		
		REG_PERF_W_BEATS: begin
			rd_data_d = perf_w_beats;
		end
		
		default: begin
			rd_data_d = 32'h0BAD_ADD2;
		end
	endcase
end

// ---------------------------------------------------------------------
// Segnali AXI raw verso accel_top
// ---------------------------------------------------------------------

logic [DATA_AXI_ADDR_WIDTH-1:0] ar_addr_out;
logic [7:0]                     ar_len_out;
logic [2:0]                     ar_size_out;
logic [1:0]                     ar_burst;
logic                           ar_valid;
logic                           ar_ready;

logic [DATA_AXI_DATA_WIDTH-1:0] data_in;
logic                           r_valid;
logic                           r_ready;
logic                           r_last;

logic [DATA_AXI_ADDR_WIDTH-1:0] aw_addr;
logic [7:0]                     aw_len;
logic [2:0]                     aw_size;
logic [1:0]                     aw_burst;
logic                           aw_valid;
logic                           aw_ready;

logic [DATA_AXI_DATA_WIDTH-1:0]   w_data;
logic [DATA_AXI_BYTE_NUM-1:0]     w_strb;
logic                             w_valid;
logic                             w_ready;
logic                             w_last;

logic [1:0] b_resp;
logic       b_valid;
logic       b_ready;

// ---------------------------------------------------------------------
// DRAM read and local-SRAM write are independent paths
// ---------------------------------------------------------------------
// In the SRAM-write integration, ext_mst serves only AR/R towards DRAM,
// while sauria_mst serves only AW/W/B towards SAURIA local memories.
// Therefore the old read-vs-write interlock is unnecessary and would only
// serialize two independent interfaces.

// ---------------------------------------------------------------------
// Istanza del tuo acceleratore
// ---------------------------------------------------------------------

accel_top #(
	.AXI_ADDR_W       (DATA_AXI_ADDR_WIDTH),
	.AXI_DATA_SIZE    (DATA_AXI_DATA_WIDTH),
	.FIFO_DEPTH       (FIFO_DEPTH),
	.FIFO_THRESHOLD   (FIFO_THRESHOLD),
	.NumWords_idx     (NumWords_idx),
	.ByteWidth_idx    (ByteWidth_idx),
	.NumWords         (NumWords),
	.ByteWidth        (ByteWidth),
	.NumPorts         (NumPorts),
	.Latency          (Latency),
	.N_BLOCKS         (N_BLOCKS),
	.BUFFER_BIT_ADDR  (BUFFER_BIT_ADDR)
) i_accel_top (
	.clk                    (i_clk),
	.rst_n                  (i_rstn),

	.start                  (start_pulse),
	.is_spmm                (is_spmm_q),
	.done                   (done),

	.ar_size_in             (ar_size_q),
	.ar_addr_out            (ar_addr_out),
	.ar_len_out             (ar_len_out),
	.ar_size_out            (ar_size_out),
	.data_in                (data_in),
	.ar_valid               (ar_valid),
	.ar_ready               (ar_ready),
	.ar_burst               (ar_burst),
	.r_valid                (r_valid),
	.r_ready                (r_ready),
	.r_last                 (r_last),

	.axi_wr_aw_size_in      (aw_size_q),
	.axi_wr_aw_addr_in      (axi_wr_aw_addr_in_q),
	.aw_addr                (aw_addr),
	.aw_len                 (aw_len),
	.aw_size                (aw_size),
	.aw_burst               (aw_burst),
	.aw_valid               (aw_valid),
	.aw_ready               (aw_ready),
	.w_data                 (w_data),
	.w_strb                 (w_strb),
	.w_valid                (w_valid),
	.w_ready                (w_ready),
	.w_last                 (w_last),
	.b_resp                 (b_resp),
	.b_valid                (b_valid),
	.b_ready                (b_ready),

	.ar_addr_dense_matrix   (ar_addr_dense_matrix_q),
	.total_len_dense_matrix (total_len_dense_matrix_q),
	.ar_addr_comp_idx       (ar_addr_comp_idx_q),
	.total_len_comp_idx     (total_len_comp_idx_q),
	.ar_addr_idx            (ar_addr_idx_q),
	.total_len_idx          (total_len_idx_q),
	.status_reg_n_row       (status_reg_n_row_q)
);

// ---------------------------------------------------------------------
// Conversione segnali raw AXI -> AXI_BUS.Master SAURIA
// ---------------------------------------------------------------------

// Read address channel
assign ext_mst.ar_id     = '0;
assign ext_mst.ar_addr   = ar_addr_out;
assign ext_mst.ar_len    = ar_len_out;
assign ext_mst.ar_size   = ar_size_out;
assign ext_mst.ar_burst  = axi_pkg::burst_t'(ar_burst);
assign ext_mst.ar_valid  = ar_valid;
assign ar_ready          = ext_mst.ar_ready;

assign ext_mst.ar_prot   = '0;
assign ext_mst.ar_lock   = '0;
assign ext_mst.ar_cache  = '0;
assign ext_mst.ar_qos    = '0;
assign ext_mst.ar_region = '0;
assign ext_mst.ar_user   = '0;

// Read data channel
assign data_in           = ext_mst.r_data;
assign r_valid           = ext_mst.r_valid;
assign r_last            = ext_mst.r_last;
assign ext_mst.r_ready   = r_ready;

// Write address channel
assign ext_mst.aw_id     = '0;
assign ext_mst.aw_addr   = '0;
assign ext_mst.aw_len    = '0;
assign ext_mst.aw_size   = '0;
assign ext_mst.aw_burst  = axi_pkg::BURST_INCR;
assign ext_mst.aw_valid  = 1'b0;
assign ext_mst.aw_prot   = '0;
assign ext_mst.aw_lock   = '0;
assign ext_mst.aw_cache  = '0;
assign ext_mst.aw_qos    = '0;
assign ext_mst.aw_region = '0;
assign ext_mst.aw_atop   = '0;
assign ext_mst.aw_user   = '0;

assign ext_mst.w_data    = '0;
assign ext_mst.w_strb    = '0;
assign ext_mst.w_last    = 1'b0;
assign ext_mst.w_valid   = 1'b0;
assign ext_mst.w_user    = '0;

assign ext_mst.b_ready   = 1'b1;

// -------------------------
// SAURIA internal SRAM write path
// -------------------------

assign sauria_mst.aw_id     = '0;
assign sauria_mst.aw_addr   = aw_addr;
assign sauria_mst.aw_len    = aw_len;
assign sauria_mst.aw_size   = aw_size;
assign sauria_mst.aw_burst  = axi_pkg::burst_t'(aw_burst);
assign sauria_mst.aw_valid  = aw_valid;
assign aw_ready             = sauria_mst.aw_ready;

assign sauria_mst.aw_prot   = '0;
assign sauria_mst.aw_lock   = '0;
assign sauria_mst.aw_cache  = '0;
assign sauria_mst.aw_qos    = '0;
assign sauria_mst.aw_region = '0;
assign sauria_mst.aw_atop   = '0;
assign sauria_mst.aw_user   = '0;

assign sauria_mst.w_data    = w_data;
assign sauria_mst.w_strb    = w_strb;
assign sauria_mst.w_last    = w_last;
assign sauria_mst.w_valid   = w_valid;
assign sauria_mst.w_user    = '0;
assign w_ready              = sauria_mst.w_ready;

assign b_resp               = sauria_mst.b_resp;
assign b_valid              = sauria_mst.b_valid;
assign sauria_mst.b_ready   = b_ready;

assign sauria_mst.ar_id     = '0;
assign sauria_mst.ar_addr   = '0;
assign sauria_mst.ar_len    = '0;
assign sauria_mst.ar_size   = '0;
assign sauria_mst.ar_burst  = axi_pkg::BURST_INCR;
assign sauria_mst.ar_valid  = 1'b0;
assign sauria_mst.ar_prot   = '0;
assign sauria_mst.ar_lock   = '0;
assign sauria_mst.ar_cache  = '0;
assign sauria_mst.ar_qos    = '0;
assign sauria_mst.ar_region = '0;
assign sauria_mst.ar_user   = '0;

assign sauria_mst.r_ready   = 1'b1;

assign o_doneintr        = done_sticky;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
	busy_q <= 1'b0;
  end else begin
	if (start_pulse)
	  busy_q <= 1'b1;
	else if (done)
	  busy_q <= 1'b0;
  end
end

assign o_busy = busy_q;

always_ff @(posedge i_clk or negedge i_rstn) begin
  if (!i_rstn) begin
    perf_active       <= 1'b0;
    perf_total_cycles <= '0;
    perf_rd_cycles    <= '0;
    perf_wr_cycles    <= '0;
    perf_r_stall      <= '0;
    perf_w_stall      <= '0;
    perf_r_beats      <= '0;
    perf_w_beats      <= '0;
  end else begin

    if (gather_start_fire) begin
      perf_active       <= 1'b1;
      perf_total_cycles <= '0;
      perf_rd_cycles    <= '0;
      perf_wr_cycles    <= '0;
      perf_r_stall      <= '0;
      perf_w_stall      <= '0;
      perf_r_beats      <= '0;
      perf_w_beats      <= '0;
    end

    if (perf_active) begin
      perf_total_cycles <= perf_total_cycles + 1;

      if (read_active)
        perf_rd_cycles <= perf_rd_cycles + 1;

      if (write_active)
        perf_wr_cycles <= perf_wr_cycles + 1;

      if (ext_mst.r_valid && !ext_mst.r_ready)
        perf_r_stall <= perf_r_stall + 1;

      if (sauria_mst.w_valid && !sauria_mst.w_ready)
        perf_w_stall <= perf_w_stall + 1;

      if (r_fire)
        perf_r_beats <= perf_r_beats + 1;

      if (w_fire)
        perf_w_beats <= perf_w_beats + 1;
    end

    if (gather_done_fire) begin
      perf_active <= 1'b0;
    end

  end
end

endmodule
