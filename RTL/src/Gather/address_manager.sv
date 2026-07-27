`timescale 1ns/1ps

module address_manager #(
    parameter int AXI_ADDR_W = 32,
    parameter int NumWords = 16,
    parameter int N_BLOCK_SRAM = 8,
    parameter int WORD_SIZE = 16,
    parameter int AXI_DATA_SIZE = 64,
    parameter int unsigned NumWords_idx = 16,
    parameter int unsigned WORD_SIZE_IDX = 32,
    parameter int unsigned PTR_FIFO_DEPTH = 4,
    // Dependent parameters.
    parameter int unsigned WORDS_PER_LINE = $clog2(AXI_DATA_SIZE / WORD_SIZE),
    parameter int unsigned WORDS_PER_LINE_IDX = $clog2(AXI_DATA_SIZE / WORD_SIZE_IDX)
) (
    input  logic clk,

    // Dense SRAM write/read addressing.
    input  logic [AXI_ADDR_W-1:0] status_reg_n_row,
    input  logic cnt_row_en,
    input  logic cnt_row_rst_n,
    input  logic rd_b_mem,
    output logic [$clog2(NumWords)-1:0] addr_b_mem_out,
    output logic [WORDS_PER_LINE-1:0] byte_b_mem_out,
    output logic [$clog2(N_BLOCK_SRAM)-1:0] en_b_mem_out,

    // Column/row-index PISO.
    input  logic piso_rst_n,
    input  logic load,
    input  logic shift_en,
    output logic empty,
    input  logic fifo_empty,
    output logic fifo_r_en,
    input  logic is_spmm,
    input  logic [AXI_DATA_SIZE-1:0] piso_data_in,

    // Compressed-pointer look-ahead stream.
    input  logic ptr_stream_enable,
    input  logic ptr_stream_rst_n,
    output logic ptr_rd_req,
    input  logic ptr_rsp_valid,
    input  logic [WORD_SIZE_IDX-1:0] comp_idx_in,
    input  logic [AXI_ADDR_W-1:0] total_len_comp_idx,
    input  logic segment_advance,
    input  logic nnz_issue,
    output logic segment_valid,
    output logic segment_done,
    output logic stream_end,
    output logic end_rd_index,
    output logic next_row_empty,

    // Dense-bank selector for SpMM.
    input  logic cnt_stage_en,
    input  logic cnt_stage_rst_n,

    // Pointer-SRAM write address during preload.
    input  logic cnt_comp_idx_en,
    input  logic cnt_comp_idx_rst_n,
    output logic [($clog2(NumWords_idx) + WORDS_PER_LINE_IDX)-1:0] cnt_comp_idx_out
);

localparam int ADDR_IDX_WIDTH = $clog2(NumWords_idx) + WORDS_PER_LINE_IDX;
localparam int MAX_VAL_IDX = (NumWords_idx * (AXI_DATA_SIZE / WORD_SIZE_IDX)) - 1;
localparam int PTR_COUNT_W = $clog2(PTR_FIFO_DEPTH + 1);
localparam int PTR_INDEX_W = $clog2(PTR_FIFO_DEPTH);

logic [ADDR_IDX_WIDTH-1:0] cnt_addr_idx_out;
logic [ADDR_IDX_WIDTH-1:0] ptr_req_index_q;
logic [ADDR_IDX_WIDTH-1:0] load_ptr_addr;

logic [WORD_SIZE_IDX-1:0] ptr_fifo [0:PTR_FIFO_DEPTH-1];
logic [PTR_COUNT_W-1:0] ptr_count_q;
logic [PTR_COUNT_W-1:0] ptr_inflight_q;
logic [PTR_COUNT_W:0] ptr_reserved;
logic ptr_limit_reached_q;
logic ptr_issue;
logic ptr_synth_issue;
logic ptr_push;
logic ptr_pop;
logic ptr_pair_valid;
logic [WORD_SIZE_IDX-1:0] ptr_push_data;
logic ptr_push_synth;
logic ptr_synth_fifo [0:PTR_FIFO_DEPTH-1];
logic [AXI_ADDR_W-1:0] ptr_req_word_index;
logic ptr_loaded_range_exhausted;
logic stream_end_zero;
logic stream_end_decrease;

logic [WORD_SIZE_IDX-1:0] segment_remaining_q;
logic segment_valid_q;

logic [$clog2(N_BLOCK_SRAM)-1:0] cnt_stage_out;
logic update_n_col;
logic [$clog2(NumWords)-1:0] cnt_row_out;
logic [$clog2(N_BLOCK_SRAM)-1:0] cnt_col_out;

// ========================================================
// Pointer preload address
// ========================================================

always_ff @(posedge clk or negedge cnt_comp_idx_rst_n) begin
    if (!cnt_comp_idx_rst_n) begin
        cnt_addr_idx_out <= '0;
    end else if (cnt_comp_idx_en) begin
        cnt_addr_idx_out <= cnt_addr_idx_out + 1'b1;
    end
end


assign load_ptr_addr = {cnt_addr_idx_out[0 +: $clog2(NumWords_idx)],{WORDS_PER_LINE_IDX{1'b0}}};
assign cnt_comp_idx_out = rd_b_mem ? ptr_req_index_q : load_ptr_addr;

// ========================================================
// Pointer look-ahead FIFO
// ========================================================

assign ptr_reserved = (PTR_COUNT_W+1)'(ptr_count_q) + (PTR_COUNT_W+1)'(ptr_inflight_q);
assign ptr_pop = segment_advance && segment_valid_q;
assign ptr_req_word_index = AXI_ADDR_W'(ptr_req_index_q >> WORDS_PER_LINE_IDX);
assign ptr_loaded_range_exhausted = (ptr_req_word_index >= total_len_comp_idx);

assign ptr_issue = ptr_stream_enable && !ptr_limit_reached_q && ((ptr_reserved < (PTR_COUNT_W+1)'(PTR_FIFO_DEPTH)) || ptr_pop);
assign ptr_synth_issue = ptr_issue && ptr_loaded_range_exhausted && (ptr_inflight_q == '0);
assign ptr_rd_req = ptr_issue && !ptr_loaded_range_exhausted;
assign ptr_push = ptr_rsp_valid || ptr_synth_issue;
assign ptr_push_data = ptr_synth_issue ? '0 : comp_idx_in;
assign ptr_push_synth = ptr_synth_issue;

always_ff @(posedge clk or negedge ptr_stream_rst_n) begin
    if (!ptr_stream_rst_n) begin
        ptr_req_index_q <= '0;
        ptr_count_q <= '0;
        ptr_inflight_q <= '0;
        ptr_limit_reached_q <= 1'b0;
        segment_remaining_q <= '0;
        segment_valid_q <= 1'b0;
        for (int unsigned i = 0; i < PTR_FIFO_DEPTH; i++) begin
            ptr_fifo[i] <= '0;
            ptr_synth_fifo[i] <= 1'b0;
        end
    end else begin
        if (ptr_rd_req || ptr_synth_issue) begin
            if (ptr_synth_issue
                    || (ptr_req_index_q == ADDR_IDX_WIDTH'(MAX_VAL_IDX))) begin
                ptr_limit_reached_q <= 1'b1;
            end else begin
                ptr_req_index_q <= ptr_req_index_q + 1'b1;
            end
        end

        unique case ({ptr_rd_req, ptr_rsp_valid})
            2'b10: ptr_inflight_q <= ptr_inflight_q + 1'b1;
            2'b01: ptr_inflight_q <= ptr_inflight_q - 1'b1;
            default: ptr_inflight_q <= ptr_inflight_q;
        endcase

        unique case ({ptr_pop, ptr_push})
            2'b01: begin
                ptr_fifo[PTR_INDEX_W'(ptr_count_q)] <= ptr_push_data;
                ptr_synth_fifo[PTR_INDEX_W'(ptr_count_q)] <= ptr_push_synth;
                ptr_count_q <= ptr_count_q + 1'b1;
            end

            2'b10: begin
                for (int unsigned i = 0; i < PTR_FIFO_DEPTH-1; i++) begin
                    ptr_fifo[i] <= ptr_fifo[i+1];
                    ptr_synth_fifo[i] <= ptr_synth_fifo[i+1];
                end
                ptr_fifo[PTR_FIFO_DEPTH-1] <= '0;
                ptr_synth_fifo[PTR_FIFO_DEPTH-1] <= 1'b0;
                ptr_count_q <= ptr_count_q - 1'b1;
            end

            2'b11: begin
                for (int unsigned i = 0; i < PTR_FIFO_DEPTH-1; i++) begin
                    ptr_fifo[i] <= ptr_fifo[i+1];
                    ptr_synth_fifo[i] <= ptr_synth_fifo[i+1];
                end
                ptr_fifo[PTR_INDEX_W'(ptr_count_q-1'b1)] <= ptr_push_data;
                ptr_synth_fifo[PTR_INDEX_W'(ptr_count_q-1'b1)]
                    <= ptr_push_synth;
                ptr_count_q <= ptr_count_q;
            end

            default: ptr_count_q <= ptr_count_q;
        endcase

        if (ptr_pop) begin
            if (ptr_count_q >= PTR_COUNT_W'(3)) begin
                segment_remaining_q <= (ptr_fifo[2] >= ptr_fifo[1]) ? (ptr_fifo[2] - ptr_fifo[1]) : '0;
                segment_valid_q <= 1'b1;
            end else if ((ptr_count_q == PTR_COUNT_W'(2)) && ptr_push) begin
                segment_remaining_q <= (comp_idx_in >= ptr_fifo[1]) ? (comp_idx_in - ptr_fifo[1]) : '0;
                segment_valid_q <= 1'b1;
            end else begin
                segment_remaining_q <= '0;
                segment_valid_q <= 1'b0;
            end
        end else if (!segment_valid_q) begin
            if ((ptr_count_q == PTR_COUNT_W'(1)) && ptr_push) begin
                segment_remaining_q <= (comp_idx_in >= ptr_fifo[0]) ? (comp_idx_in - ptr_fifo[0]) : '0;
                segment_valid_q <= 1'b1;
            end else if (ptr_count_q >= PTR_COUNT_W'(2)) begin
                segment_remaining_q <= (ptr_fifo[1] >= ptr_fifo[0]) ? (ptr_fifo[1] - ptr_fifo[0]) : '0;
                segment_valid_q <= 1'b1;
            end
        end else if (nnz_issue) begin
            segment_remaining_q <= segment_remaining_q - 1'b1;
        end
    end
end

assign segment_valid = segment_valid_q;
assign segment_done = segment_valid_q && (segment_remaining_q == '0);

assign ptr_pair_valid = (ptr_count_q >= PTR_COUNT_W'(2));
assign stream_end_zero = ptr_pair_valid && (ptr_fifo[1] == '0) && ((ptr_fifo[0] != '0) || ptr_synth_fifo[1]);
assign stream_end_decrease = ptr_pair_valid && (ptr_fifo[1] != '0) && (ptr_fifo[1] < ptr_fifo[0]);
assign stream_end = stream_end_zero || stream_end_decrease;


assign end_rd_index = ptr_limit_reached_q && (ptr_inflight_q == '0) && (ptr_count_q < PTR_COUNT_W'(2));

assign next_row_empty = (ptr_count_q >= PTR_COUNT_W'(3)) && (ptr_fifo[2] == ptr_fifo[1]);

// ========================================================
// Dense-bank stage counter
// ========================================================

always_ff @(posedge clk or negedge cnt_stage_rst_n) begin
    if (!cnt_stage_rst_n) begin
        cnt_stage_out <= '0;
    end else if (cnt_stage_en) begin
        cnt_stage_out <= cnt_stage_out + 1'b1;
    end
end

// ========================================================
// Dense SRAM preload address
// ========================================================

always_ff @(posedge clk or negedge cnt_row_rst_n) begin
    if (!cnt_row_rst_n) begin
        cnt_row_out <= '0;
    end else if (update_n_col) begin
        cnt_row_out <= '0;
    end else if (cnt_row_en) begin
        cnt_row_out <= cnt_row_out + 1'b1;
    end
end

always_ff @(posedge clk or negedge cnt_row_rst_n) begin
    if (!cnt_row_rst_n) begin
        cnt_col_out <= '0;
    end else if (update_n_col) begin
        cnt_col_out <= cnt_col_out + 1'b1;
    end
end

assign update_n_col = (is_spmm == 1'b0) ? (cnt_row_en && (cnt_row_out == '1)) : (cnt_row_en && (cnt_row_out == status_reg_n_row[WORDS_PER_LINE +: $clog2(NumWords)]));

// ========================================================
// Column/row-index PISO and dense SRAM address decode
// ========================================================

logic [WORD_SIZE_IDX-1:0] piso_data_out;
logic [$clog2(NumWords)-1:0] piso_addr_b_mem_out;
logic [WORDS_PER_LINE-1:0] piso_byte_b_mem_out;
logic [$clog2(N_BLOCK_SRAM)-1:0] piso_en_b_mem_out;
logic [1:0][$clog2(N_BLOCK_SRAM)-1:0] mux_en_b_mem_spmm;
logic [1:0][$clog2(N_BLOCK_SRAM)-1:0] mux_en_b_mem_rd_wr;
logic [1:0][$clog2(NumWords)-1:0] mux_addr_b_mem;

piso #(
    .N_DATA_IN(AXI_DATA_SIZE),
    .WORD_SIZE(WORD_SIZE_IDX)
) i_piso (
    .clk,
    .rst_n(piso_rst_n),
    .data_in(piso_data_in),
    .load,
    .shift_en,
    .fifo_empty,
    .fifo_r_en,
    .data_out(piso_data_out),
    .empty
);

localparam int REQ_WIDTH = WORDS_PER_LINE + $clog2(NumWords) + $clog2(N_BLOCK_SRAM);
localparam int EXT_WIDTH = (WORD_SIZE_IDX > REQ_WIDTH) ? WORD_SIZE_IDX : REQ_WIDTH;

logic [EXT_WIDTH-1:0] piso_data_out_extended;
assign piso_data_out_extended = EXT_WIDTH'(piso_data_out);

assign piso_byte_b_mem_out = piso_data_out_extended[0 +: WORDS_PER_LINE];
assign piso_addr_b_mem_out = piso_data_out_extended[WORDS_PER_LINE +: $clog2(NumWords)];
assign piso_en_b_mem_out = piso_data_out_extended[(WORDS_PER_LINE + $clog2(NumWords)) +: $clog2(N_BLOCK_SRAM)];

assign mux_en_b_mem_spmm[0] = piso_en_b_mem_out;
assign mux_en_b_mem_spmm[1] = cnt_stage_out;
assign mux_en_b_mem_rd_wr[0] = cnt_col_out;
assign mux_en_b_mem_rd_wr[1] = mux_en_b_mem_spmm[is_spmm];

assign mux_addr_b_mem[0] = cnt_row_out;
assign mux_addr_b_mem[1] = piso_addr_b_mem_out;

assign byte_b_mem_out = piso_byte_b_mem_out;
assign addr_b_mem_out = mux_addr_b_mem[rd_b_mem];
assign en_b_mem_out = mux_en_b_mem_rd_wr[rd_b_mem];


endmodule
