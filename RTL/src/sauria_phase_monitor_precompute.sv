`timescale 1ns/1ps

// Simulation-only phase monitor for SAURIA + Gather benchmarks.
//
// The monitor observes the DMA write address toward the local SAURIA SRAMs
// and associates the next writer-completion interrupt with that transfer.
// It also measures the complete Gather interval.
//
// Only transfers that start before the first SAURIA completion interrupt are
// classified. Therefore the final C writeback is not counted as a preload.
//
// All reported cycles belong to i_system_clk.
module sauria_phase_monitor #(
    parameter int unsigned ADDR_W = 32
) (
    input  logic                  clk_i,
    input  logic                  rst_ni,

    input  logic                  gather_busy_i,
    input  logic                  gather_done_i,
    input  logic                  sauria_done_i,

    input  logic [ADDR_W-1:0]     dma_aw_addr_i,
    input  logic                  dma_aw_valid_i,
    input  logic                  dma_aw_ready_i,
    input  logic                  dma_writer_done_i
);

    localparam logic [ADDR_W-1:0] SRAM_REGION_MASK =
        ADDR_W'(32'h003C_0000);
    localparam logic [ADDR_W-1:0] SRAMA_REGION =
        ADDR_W'(32'h0004_0000);
    localparam logic [ADDR_W-1:0] SRAMB_REGION =
        ADDR_W'(32'h0008_0000);
    localparam logic [ADDR_W-1:0] SRAMC_REGION =
        ADDR_W'(32'h000C_0000);

    typedef enum logic [1:0] {
        PHASE_NONE,
        PHASE_SRAMA,
        PHASE_SRAMB,
        PHASE_SRAMC
    } dma_phase_e;

    logic [63:0] cycle_q;

    logic gather_busy_q;
    logic gather_done_q;
    logic gather_started_q;
    logic gather_finished_q;

    logic sauria_done_q;
    logic compute_done_seen_q;

    logic dma_writer_done_q;
    dma_phase_e dma_phase_q;

    logic srama_seen_q;
    logic sramb_seen_q;
    logic sramc_seen_q;

    wire dma_aw_fire = dma_aw_valid_i && dma_aw_ready_i;
    wire [ADDR_W-1:0] dma_aw_region =
        dma_aw_addr_i & SRAM_REGION_MASK;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            cycle_q            <= 64'd0;

            gather_busy_q      <= 1'b0;
            gather_done_q      <= 1'b0;
            gather_started_q   <= 1'b0;
            gather_finished_q  <= 1'b0;

            sauria_done_q      <= 1'b0;
            compute_done_seen_q <= 1'b0;

            dma_writer_done_q  <= 1'b0;
            dma_phase_q        <= PHASE_NONE;

            srama_seen_q       <= 1'b0;
            sramb_seen_q       <= 1'b0;
            sramc_seen_q       <= 1'b0;
        end else begin
            cycle_q           <= cycle_q + 64'd1;
            gather_busy_q     <= gather_busy_i;
            gather_done_q     <= gather_done_i;
            sauria_done_q     <= sauria_done_i;
            dma_writer_done_q <= dma_writer_done_i;

            if (sauria_done_i && !sauria_done_q) begin
                compute_done_seen_q <= 1'b1;
            end

            // Complete Gather interval, including dense/metadata reads,
            // local gather processing and the write to SRAM A.
            if (
                gather_busy_i
                && !gather_busy_q
                && !gather_started_q
            ) begin
                gather_started_q <= 1'b1;
                $display(
                    "[PERF_PHASE] phase=GATHER event=START cycle=%0d",
                    cycle_q
                );
            end

            if (
                gather_done_i
                && !gather_done_q
                && gather_started_q
                && !gather_finished_q
            ) begin
                gather_finished_q <= 1'b1;
                $display(
                    "[PERF_PHASE] phase=GATHER event=DONE cycle=%0d",
                    cycle_q
                );
            end

            // The first accepted AW identifies the SRAM transfer. Only AWs
            // observed before the core completion are eligible, so output
            // writeback traffic cannot be mistaken for a preload.
            if (
                dma_aw_fire
                && dma_phase_q == PHASE_NONE
                && !compute_done_seen_q
                && !sauria_done_i
            ) begin
                unique case (dma_aw_region)
                    SRAMA_REGION: begin
                        if (!srama_seen_q) begin
                            srama_seen_q <= 1'b1;
                            dma_phase_q  <= PHASE_SRAMA;
                            $display(
                                "[PERF_PHASE] phase=SRAM_A_LOAD event=START cycle=%0d",
                                cycle_q
                            );
                        end
                    end

                    SRAMB_REGION: begin
                        if (!sramb_seen_q) begin
                            sramb_seen_q <= 1'b1;
                            dma_phase_q  <= PHASE_SRAMB;
                            $display(
                                "[PERF_PHASE] phase=SRAM_B_LOAD event=START cycle=%0d",
                                cycle_q
                            );
                        end
                    end

                    SRAMC_REGION: begin
                        if (!sramc_seen_q) begin
                            sramc_seen_q <= 1'b1;
                            dma_phase_q  <= PHASE_SRAMC;
                            $display(
                                "[PERF_PHASE] phase=SRAM_C_LOAD event=START cycle=%0d",
                                cycle_q
                            );
                        end
                    end

                    default: begin
                        // Ignore writes outside the local A/B/C SRAM regions.
                    end
                endcase
            end

            // The DMA writer interrupt marks completion of the currently
            // classified local-memory transfer.
            if (
                dma_writer_done_i
                && !dma_writer_done_q
                && dma_phase_q != PHASE_NONE
            ) begin
                unique case (dma_phase_q)
                    PHASE_SRAMA: begin
                        $display(
                            "[PERF_PHASE] phase=SRAM_A_LOAD event=DONE cycle=%0d",
                            cycle_q
                        );
                    end

                    PHASE_SRAMB: begin
                        $display(
                            "[PERF_PHASE] phase=SRAM_B_LOAD event=DONE cycle=%0d",
                            cycle_q
                        );
                    end

                    PHASE_SRAMC: begin
                        $display(
                            "[PERF_PHASE] phase=SRAM_C_LOAD event=DONE cycle=%0d",
                            cycle_q
                        );
                    end

                    default: begin
                    end
                endcase

                dma_phase_q <= PHASE_NONE;
            end
        end
    end

endmodule
