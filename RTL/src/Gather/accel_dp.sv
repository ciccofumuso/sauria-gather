`timescale 1ns/1ps

module accel_dp #(
    parameter int unsigned AXI_ADDR_W = 32,
    parameter int unsigned AXI_DATA_SIZE = 64,
    parameter int FIFO_DEPTH = 16,
    parameter int FIFO_THRESHOLD = 8,
    parameter int unsigned NumWords_idx = 16,
    parameter int unsigned ByteWidth_idx = 32,
    parameter int unsigned NumWords = 16,
    parameter int unsigned ByteWidth = 32,
    parameter int unsigned NumPorts = 1,
    parameter int unsigned Latency = 1,
    parameter int N_BLOCKS = 8,
    parameter int BUFFER_BIT_ADDR = 4,
    parameter int PTR_FIFO_DEPTH = 4,
    parameter int RESP_DEPTH = Latency + 2
) (
    input  logic clk,
    input  logic rst_n,
    input  logic is_spmm,

    // AXI4 read control and external interface.
    input  logic axi_rd_rst_n,
    input  logic axi_rd_start,
    input  logic [2:0] ar_size_in,
    input  logic [AXI_ADDR_W-1:0] ar_addr_dense_matrix,
    input  logic [AXI_ADDR_W-1:0] total_len_dense_matrix,
    input  logic [AXI_ADDR_W-1:0] ar_addr_comp_idx,
    input  logic [AXI_ADDR_W-1:0] total_len_comp_idx,
    input  logic [AXI_ADDR_W-1:0] ar_addr_idx,
    input  logic [AXI_ADDR_W-1:0] total_len_idx,
    input  logic rst_n_multi_burst,
    output logic axi_rd_available,
    output logic axi_rd_done,
    input  logic axi_rd_done_rst_n,
    output logic [AXI_ADDR_W-1:0] ar_addr_out,
    output logic [7:0] ar_len_out,
    output logic [2:0] ar_size_out,
    input  logic [AXI_DATA_SIZE-1:0] data_in,
    output logic ar_valid,
    input  logic ar_ready,
    output logic [1:0] ar_burst,
    input  logic r_valid,
    output logic r_ready,
    input  logic r_last,

    // Shared AXI-read FIFO.
    input  logic fifo_r_en,
    input  logic fifo_rst_n,
    output logic fifo_empty,

    // Internal memories and pipelined dense read.
    input  logic mem_rst_n,
    input  logic mem_wr_en,
    input  logic mem_idx_wr_en,
    input  logic dense_rd_req,
    output logic dense_req_ready,
    output logic dense_pipe_empty,

    // Output packing and double buffer.
    input  logic buffer_rst_n,
    input  logic buffer_data_in_last,
    output logic buffer_wait_wr,
    output logic buffer_data_out_valid,

    // AXI4 write control and external interface.
    input  logic axi_wr_rst_n,
    output logic axi_wr_done,
    input  logic axi_wr_done_rst_n,
    input  logic [2:0] axi_wr_aw_size_in,
    input  logic [AXI_ADDR_W-1:0] axi_wr_aw_addr_in,
    output logic [AXI_ADDR_W-1:0] aw_addr,
    output logic [7:0] aw_len,
    output logic [2:0] aw_size,
    output logic [1:0] aw_burst,
    output logic aw_valid,
    input  logic aw_ready,
    output logic [AXI_DATA_SIZE-1:0] w_data,
    output logic [AXI_DATA_SIZE/8-1:0] w_strb,
    output logic w_valid,
    input  logic w_ready,
    output logic w_last,
    input  logic [1:0] b_resp,
    input  logic b_valid,
    output logic b_ready,

    // Read-master phase status.
    output logic tc_wr_mem,
    output logic wr_index,
    output logic tc_rd_index,

    // Dense preload address manager.
    input  logic [AXI_ADDR_W-1:0] status_reg_n_row,
    input  logic cnt_row_en,
    input  logic cnt_row_rst_n,
    input  logic rd_b_mem,

    // PISO and pointer-look-ahead controls.
    input  logic piso_rst_n,
    input  logic piso_load,
    input  logic piso_shift_en,
    output logic piso_empty,
    input  logic ptr_stream_enable,
    input  logic ptr_stream_rst_n,
    input  logic segment_advance,
    output logic segment_valid,
    output logic segment_done,
    output logic stream_end,
    output logic nnz_limit_reached,
    output logic end_rd_index,
    output logic next_row_empty,

    // SpMM dense-bank stage.
    input  logic cnt_stage_en,
    input  logic cnt_stage_rst_n,

    // Compressed-pointer preload address.
    input  logic cnt_comp_idx_en,
    input  logic cnt_comp_idx_rst_n
);

localparam int RESP_PTR_W = $clog2(RESP_DEPTH);
localparam int RESP_COUNT_W = $clog2(RESP_DEPTH + 1);
localparam int unsigned INDICES_PER_BEAT = AXI_DATA_SIZE / ByteWidth_idx;
localparam int unsigned NNZ_COUNT_W =
    AXI_ADDR_W + ((INDICES_PER_BEAT > 1) ? $clog2(INDICES_PER_BEAT) : 0);

logic [AXI_DATA_SIZE-1:0] axi_rd_data_out;
logic rd_done;
logic axi_rd_is_valid_data;

logic fifo_rd;
logic [AXI_DATA_SIZE-1:0] fifo_r_data;
logic fifo_threshold;
logic piso_to_fifo_r_en;

logic [$clog2(NumWords)-1:0] mem_addr;
logic [$clog2(AXI_DATA_SIZE/ByteWidth)-1:0] mem_byte_sel;
logic [$clog2(N_BLOCKS)-1:0] mem_mem_en;
logic [ByteWidth-1:0] mem_data_out;
logic dense_rsp_valid;

logic [$clog2(NumWords_idx)-1:0] mem_idx_addr;
logic [$clog2(AXI_DATA_SIZE/ByteWidth_idx)-1:0] mem_idx_byte_sel;
logic [ByteWidth_idx-1:0] mem_idx_data_out;
logic ptr_rd_req;
logic ptr_rsp_valid;

logic buffer_out_ready;
logic [AXI_DATA_SIZE-1:0] buffer_data_out;
logic buffer_out_valid;
logic [7:0] buffer_rd_len;
logic [AXI_DATA_SIZE/8-1:0] buffer_wstrb_last;
logic buffer_value_valid;
logic [ByteWidth-1:0] buffer_value_data;

logic wr_done;

logic [ByteWidth_idx-1:0] ptr_load_prev_q;
logic [ByteWidth_idx-1:0] ptr_load_prev_d;
logic [ByteWidth_idx-1:0] ptr_load_final_q;
logic [ByteWidth_idx-1:0] ptr_load_final_d;
logic ptr_load_seen_q;
logic ptr_load_seen_d;
logic ptr_load_end_q;
logic ptr_load_end_d;
logic [ByteWidth_idx-1:0] ptr_final_nnz;
logic [AXI_ADDR_W:0] ptr_final_nnz_ext;
logic [AXI_ADDR_W:0] ptr_required_beats_ext;
logic [AXI_ADDR_W:0] total_len_idx_ext;
logic [AXI_ADDR_W-1:0] ptr_required_beats;
logic [AXI_ADDR_W-1:0] effective_total_len_idx;
logic [AXI_ADDR_W-1:0] effective_total_len_idx_nonzero;

always_comb begin
    ptr_load_prev_d = ptr_load_prev_q;
    ptr_load_final_d = ptr_load_final_q;
    ptr_load_seen_d = ptr_load_seen_q;
    ptr_load_end_d = ptr_load_end_q;

    if (mem_idx_wr_en && !ptr_load_end_d) begin
        for (int unsigned lane = 0; lane < INDICES_PER_BEAT; lane++) begin
            if (!ptr_load_end_d) begin
                if (ptr_load_seen_d && (((fifo_r_data[lane*ByteWidth_idx +: ByteWidth_idx] == '0) && (ptr_load_prev_d != '0)) || 
					(fifo_r_data[lane*ByteWidth_idx +: ByteWidth_idx] < ptr_load_prev_d))) begin
                    ptr_load_final_d = ptr_load_prev_d;
                    ptr_load_end_d = 1'b1;
                end else begin
                    ptr_load_prev_d =
                        fifo_r_data[lane*ByteWidth_idx +: ByteWidth_idx];
                    ptr_load_seen_d = 1'b1;
                end
            end
        end
    end
end

always_ff @(posedge clk or negedge mem_rst_n) begin
    if (!mem_rst_n) begin
        ptr_load_prev_q <= '0;
        ptr_load_final_q <= '0;
        ptr_load_seen_q <= 1'b0;
        ptr_load_end_q <= 1'b0;
    end else begin
        ptr_load_prev_q <= ptr_load_prev_d;
        ptr_load_final_q <= ptr_load_final_d;
        ptr_load_seen_q <= ptr_load_seen_d;
        ptr_load_end_q <= ptr_load_end_d;
    end
end


assign ptr_final_nnz = ptr_load_end_q ? ptr_load_final_q : ptr_load_prev_q;
assign ptr_final_nnz_ext = (AXI_ADDR_W+1)'(ptr_final_nnz);
assign ptr_required_beats_ext = (ptr_final_nnz_ext + (AXI_ADDR_W+1)'(INDICES_PER_BEAT - 1)) / (AXI_ADDR_W+1)'(INDICES_PER_BEAT);
assign ptr_required_beats = ptr_required_beats_ext[AXI_ADDR_W-1:0];
assign total_len_idx_ext = (AXI_ADDR_W+1)'(total_len_idx);

assign effective_total_len_idx = ptr_load_seen_q && (ptr_required_beats_ext < total_len_idx_ext) ? ptr_required_beats : total_len_idx;


assign effective_total_len_idx_nonzero = (effective_total_len_idx == '0) ? AXI_ADDR_W'(1) : effective_total_len_idx;


logic [NNZ_COUNT_W-1:0] nnz_issue_count_q;
logic [NNZ_COUNT_W-1:0] nnz_issue_limit;

assign nnz_issue_limit = NNZ_COUNT_W'(total_len_idx) * NNZ_COUNT_W'(INDICES_PER_BEAT);
assign nnz_limit_reached = (nnz_issue_count_q >= nnz_issue_limit);

always_ff @(posedge clk or negedge ptr_stream_rst_n) begin
    if (!ptr_stream_rst_n) begin
        nnz_issue_count_q <= '0;
    end else if (dense_rd_req && !nnz_limit_reached) begin
        nnz_issue_count_q <= nnz_issue_count_q + 1'b1;
    end
end

// ========================================================
// Address generation, PISO, and pointer look-ahead
// ========================================================

address_manager #(
    .AXI_ADDR_W(AXI_ADDR_W),
    .NumWords(NumWords),
    .N_BLOCK_SRAM(N_BLOCKS),
    .WORD_SIZE(ByteWidth),
    .AXI_DATA_SIZE(AXI_DATA_SIZE),
    .NumWords_idx(NumWords_idx),
    .WORD_SIZE_IDX(ByteWidth_idx),
    .PTR_FIFO_DEPTH(PTR_FIFO_DEPTH)
) i_address_manager (
    .clk,
    .status_reg_n_row,
    .cnt_row_en,
    .cnt_row_rst_n,
    .rd_b_mem,
    .addr_b_mem_out(mem_addr),
    .byte_b_mem_out(mem_byte_sel),
    .en_b_mem_out(mem_mem_en),
    .piso_rst_n,
    .load(piso_load),
    .shift_en(piso_shift_en),
    .empty(piso_empty),
    .fifo_empty,
    .fifo_r_en(piso_to_fifo_r_en),
    .is_spmm,
    .piso_data_in(fifo_r_data),
    .ptr_stream_enable,
    .ptr_stream_rst_n,
    .ptr_rd_req,
    .ptr_rsp_valid,
    .comp_idx_in(mem_idx_data_out),
    .total_len_comp_idx,
    .segment_advance,
    .nnz_issue(dense_rd_req),
    .segment_valid,
    .segment_done,
    .stream_end,
    .end_rd_index,
    .next_row_empty,
    .cnt_stage_en,
    .cnt_stage_rst_n,
    .cnt_comp_idx_en,
    .cnt_comp_idx_rst_n,
    .cnt_comp_idx_out({mem_idx_addr, mem_idx_byte_sel})
);

assign fifo_rd = rd_b_mem ? piso_to_fifo_r_en : fifo_r_en;

// ========================================================
// Dense SRAM response queue
// ========================================================

logic [ByteWidth-1:0] rsp_mem [0:RESP_DEPTH-1];
logic [RESP_PTR_W-1:0] rsp_wr_ptr_q;
logic [RESP_PTR_W-1:0] rsp_rd_ptr_q;
logic [RESP_COUNT_W-1:0] rsp_count_q;
logic [RESP_COUNT_W-1:0] reserved_count_q;
logic rsp_push;
logic rsp_pop;

assign rsp_push = dense_rsp_valid;
assign rsp_pop = (rsp_count_q != '0) && !buffer_wait_wr;


assign dense_req_ready = (reserved_count_q < RESP_COUNT_W'(RESP_DEPTH)) || rsp_pop;
assign dense_pipe_empty = (reserved_count_q == '0);

assign buffer_value_valid = rsp_pop;
assign buffer_value_data = rsp_mem[rsp_rd_ptr_q];

always_ff @(posedge clk or negedge buffer_rst_n) begin
    if (!buffer_rst_n) begin
        rsp_wr_ptr_q <= '0;
        rsp_rd_ptr_q <= '0;
        rsp_count_q <= '0;
        reserved_count_q <= '0;
        for (int unsigned i = 0; i < RESP_DEPTH; i++) begin
            rsp_mem[i] <= '0;
        end
    end else begin
        if (rsp_push) begin
            rsp_mem[rsp_wr_ptr_q] <= mem_data_out;
            if (rsp_wr_ptr_q == RESP_PTR_W'(RESP_DEPTH-1)) begin
                rsp_wr_ptr_q <= '0;
            end else begin
                rsp_wr_ptr_q <= rsp_wr_ptr_q + 1'b1;
            end
        end

        if (rsp_pop) begin
            if (rsp_rd_ptr_q == RESP_PTR_W'(RESP_DEPTH-1)) begin
                rsp_rd_ptr_q <= '0;
            end else begin
                rsp_rd_ptr_q <= rsp_rd_ptr_q + 1'b1;
            end
        end

        unique case ({rsp_push, rsp_pop})
            2'b10: rsp_count_q <= rsp_count_q + 1'b1;
            2'b01: rsp_count_q <= rsp_count_q - 1'b1;
            default: rsp_count_q <= rsp_count_q;
        endcase

        unique case ({dense_rd_req, rsp_pop})
            2'b10: reserved_count_q <= reserved_count_q + 1'b1;
            2'b01: reserved_count_q <= reserved_count_q - 1'b1;
            default: reserved_count_q <= reserved_count_q;
        endcase
    end
end

// ========================================================
// AXI4 write and double buffer
// ========================================================

axi4_master_write #(
    .ADDR_W(AXI_ADDR_W),
    .DATA_W(AXI_DATA_SIZE)
) axi4_write (
    .clk_i(clk),
    .rst_ni(axi_wr_rst_n),
    .start(buffer_out_valid),
    .addr_in(axi_wr_aw_addr_in),
    .len_in(buffer_rd_len),
    .size_in(axi_wr_aw_size_in),
    .data_in(buffer_data_out),
    .w_strb_last(buffer_wstrb_last),
    .next_data_out(buffer_out_ready),
    .done(wr_done),
    .aw_addr,
    .aw_len,
    .aw_size,
    .aw_burst,
    .aw_valid,
    .aw_ready,
    .w_data,
    .w_strb,
    .w_valid,
    .w_ready,
    .w_last,
    .b_resp,
    .b_valid,
    .b_ready
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        axi_wr_done <= 1'b0;
    end else if (!axi_wr_done_rst_n) begin
        axi_wr_done <= 1'b0;
    end else if (wr_done) begin
        axi_wr_done <= 1'b1;
    end
end

assign buffer_data_out_valid = buffer_out_valid;

double_buffer_top #(
    .N_in(ByteWidth),
    .N_out(AXI_DATA_SIZE),
    .BIT_ADDR(BUFFER_BIT_ADDR)
) i_double_buffer_top (
    .clk,
    .rst_n(buffer_rst_n),
    .data_in(buffer_value_data),
    .data_in_valid(buffer_value_valid),
    .data_in_last(buffer_data_in_last),
    .wait_wr(buffer_wait_wr),
    .out_ready(buffer_out_ready),
    .data_out(buffer_data_out),
    .out_valid(buffer_out_valid),
    .wstrb_last(buffer_wstrb_last),
    .rd_len(buffer_rd_len)
);

// ========================================================
// Dense and pointer SRAMs
// ========================================================

memory_unit #(
    .NumWords_idx(NumWords_idx),
    .ByteWidth_idx(ByteWidth_idx),
    .NumWords(NumWords),
    .DataWidth(AXI_DATA_SIZE),
    .ByteWidth(ByteWidth),
    .NumPorts(NumPorts),
    .Latency(Latency),
    .N_BLOCKS(N_BLOCKS)
) i_memory_unit (
    .clk_i(clk),
    .rst_ni(mem_rst_n),
    .addr(mem_addr),
    .byte_sel(mem_byte_sel),
    .data_in(fifo_r_data),
    .mem_en(mem_mem_en),
    .wr_en(mem_wr_en),
    .dense_rd_req,
    .dense_rsp_valid,
    .data_out(mem_data_out),
    .idx_addr(mem_idx_addr),
    .idx_byte_sel(mem_idx_byte_sel),
    .idx_data_in(fifo_r_data),
    .idx_wr_en(mem_idx_wr_en),
    .ptr_rd_req,
    .ptr_rsp_valid,
    .idx_data_out(mem_idx_data_out)
);

// ========================================================
// Shared FIFO and AXI4 read master
// ========================================================

fifo #(
    .DATA_WIDTH(AXI_DATA_SIZE),
    .DEPTH(FIFO_DEPTH),
    .THRESHOLD(FIFO_THRESHOLD)
) i_fifo (
    .clk,
    .rst_n(fifo_rst_n),
    .w_en(axi_rd_is_valid_data),
    .w_data(axi_rd_data_out),
    .r_en(fifo_rd),
    .r_data(fifo_r_data),
    .empty(fifo_empty),
    .threshold(fifo_threshold)
);

axi4_master_read #(
    .ADDR_W(AXI_ADDR_W),
    .DATA_W(AXI_DATA_SIZE)
) axi4_read (
    .clk_i(clk),
    .rst_ni(axi_rd_rst_n),
    .start(axi_rd_start),
    .available(axi_rd_available),
    .ar_size_in,
    .ar_addr_dense_matrix,
    .total_len_dense_matrix,
    .ar_addr_comp_idx,
    .total_len_comp_idx,
    .ar_addr_idx,
    .total_len_idx,
    .effective_total_len_idx(effective_total_len_idx_nonzero),
    .rst_n_multi_burst,
    .fifo_empty,
    .tc_wr_mem,
    .wr_index,
    .tc_rd_index,
    .ar_addr_out,
    .ar_len_out,
    .ar_size_out,
    .data_in,
    .data_out(axi_rd_data_out),
    .is_valid_data(axi_rd_is_valid_data),
    .done(rd_done),
    .fifo_full(fifo_threshold),
    .ar_valid,
    .ar_ready,
    .ar_burst,
    .r_valid,
    .r_ready,
    .r_last
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        axi_rd_done <= 1'b0;
    end else if (!axi_rd_done_rst_n) begin
        axi_rd_done <= 1'b0;
    end else if (rd_done) begin
        axi_rd_done <= 1'b1;
    end
end


endmodule
