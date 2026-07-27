`timescale 1ns/1ps

module memory_unit #(
    parameter int unsigned NumWords_idx = 16,
    parameter int unsigned ByteWidth_idx = 32,
    parameter int unsigned NumWords = 16,
    parameter int unsigned DataWidth = 64,
    parameter int unsigned ByteWidth = 32,
    parameter int unsigned NumPorts = 1,
    parameter int unsigned Latency = 1,
    parameter int unsigned N_BLOCKS = 8
) (
    input  logic clk_i,
    input  logic rst_ni,

    // Dense SRAM banks.
    input  logic [$clog2(NumWords)-1:0] addr,
    input  logic [$clog2(DataWidth/ByteWidth)-1:0] byte_sel,
    input  logic [DataWidth-1:0] data_in,
    input  logic [$clog2(N_BLOCKS)-1:0] mem_en,
    input  logic wr_en,
    input  logic dense_rd_req,
    output logic dense_rsp_valid,
    output logic [ByteWidth-1:0] data_out,

    // Compressed-pointer SRAM.
    input  logic [$clog2(NumWords_idx)-1:0] idx_addr,
    input  logic [$clog2(DataWidth/ByteWidth_idx)-1:0] idx_byte_sel,
    input  logic [DataWidth-1:0] idx_data_in,
    input  logic idx_wr_en,
    input  logic ptr_rd_req,
    output logic ptr_rsp_valid,
    output logic [ByteWidth_idx-1:0] idx_data_out
);

localparam int unsigned WORDS_PER_LINE = DataWidth / ByteWidth;
localparam int unsigned INDEX_PER_LINE = DataWidth / ByteWidth_idx;
localparam int unsigned BeWidth = (DataWidth + ByteWidth - 1) / ByteWidth;
localparam int unsigned BeWidth_idx = (DataWidth + ByteWidth_idx - 1) / ByteWidth_idx;
localparam int unsigned BANK_W = $clog2(N_BLOCKS);
localparam int unsigned LANE_W = $clog2(WORDS_PER_LINE);
localparam int unsigned IDX_LANE_W = $clog2(INDEX_PER_LINE);

logic [BeWidth-1:0] be_i;
logic [N_BLOCKS-1:0] decoded_en;
logic [N_BLOCKS-1:0][NumPorts-1:0][DataWidth-1:0] single_mem_data_out;
logic [N_BLOCKS-1:0][ByteWidth-1:0] single_mem_byte_out;

logic [BeWidth_idx-1:0] idx_be_i;
logic [NumPorts-1:0][DataWidth-1:0] idx_mem_data_out;

logic [BANK_W-1:0] dense_bank_sel;
logic [LANE_W-1:0] dense_lane_sel;
logic [IDX_LANE_W-1:0] ptr_lane_sel;
logic [BANK_W-1:0] dense_bank_q;
logic [LANE_W-1:0] dense_lane_q;
logic [IDX_LANE_W-1:0] ptr_lane_q;
logic dense_valid_q;
logic ptr_valid_q;

always_comb begin
    decoded_en = '0;
    if (wr_en || dense_rd_req) begin
        decoded_en[mem_en] = 1'b1;
    end
end

assign be_i = '1;

for (genvar i = 0; i < N_BLOCKS; i++) begin : gen_memory_blocks
    tc_sram #(
        .NumWords(NumWords),
        .DataWidth(DataWidth),
        .ByteWidth(ByteWidth),
        .NumPorts(NumPorts),
        .Latency(1)
    ) i_tc_sram (
        .clk_i,
        .rst_ni,
        .req_i({NumPorts{decoded_en[i]}}),
        .we_i({NumPorts{wr_en && decoded_en[i]}}),
        .addr_i({NumPorts{addr}}),
        .wdata_i({NumPorts{data_in}}),
        .be_i({NumPorts{be_i}}),
        .rdata_o(single_mem_data_out[i])
    );
end

always_comb begin
    for (int unsigned j = 0; j < N_BLOCKS; j++) begin
        single_mem_byte_out[j] = single_mem_data_out[j][0][dense_lane_sel*ByteWidth +: ByteWidth];
    end
end

assign data_out = single_mem_byte_out[dense_bank_sel];

assign idx_be_i = '1;

tc_sram #(
    .NumWords(NumWords_idx),
    .DataWidth(DataWidth),
    .ByteWidth(ByteWidth_idx),
    .NumPorts(NumPorts),
    .Latency(1)
) i_comp_idx_sram (
    .clk_i,
    .rst_ni,
    .req_i({NumPorts{idx_wr_en || ptr_rd_req}}),
    .we_i({NumPorts{idx_wr_en}}),
    .addr_i({NumPorts{idx_addr}}),
    .wdata_i({NumPorts{idx_data_in}}),
    .be_i({NumPorts{idx_be_i}}),
    .rdata_o(idx_mem_data_out)
);

assign idx_data_out = idx_mem_data_out[0][ptr_lane_sel*ByteWidth_idx +: ByteWidth_idx];

always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        dense_bank_q <= '0;
        dense_lane_q <= '0;
        ptr_lane_q <= '0;
        dense_valid_q <= 1'b0;
        ptr_valid_q <= 1'b0;
    end else begin
        dense_bank_q <= mem_en;
        dense_lane_q <= byte_sel;
        ptr_lane_q <= idx_byte_sel;
        dense_valid_q <= dense_rd_req;
        ptr_valid_q <= ptr_rd_req;
    end
end

assign dense_bank_sel = dense_bank_q;
assign dense_lane_sel = dense_lane_q;
assign ptr_lane_sel = ptr_lane_q;
assign dense_rsp_valid = dense_valid_q;
assign ptr_rsp_valid = ptr_valid_q;


endmodule
