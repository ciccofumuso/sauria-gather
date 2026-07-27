`timescale 1ns/1ps

module accel_cu (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    output logic done,

    // AXI4 read.
    output logic axi_rd_rst_n,
    output logic axi_rd_start,
    input  logic axi_rd_available,
    input  logic axi_rd_done,
    output logic axi_rd_done_rst_n,

    // Shared FIFO during preload.
    output logic fifo_r_en,
    output logic fifo_rst_n,
    input  logic fifo_empty,

    // Internal memories.
    output logic mem_rst_n,
    output logic mem_wr_en,
    output logic mem_idx_wr_en,
    output logic dense_rd_req,
    input  logic dense_req_ready,
    input  logic dense_pipe_empty,

    // Output buffer.
    output logic buffer_rst_n,
    output logic buffer_data_in_last,
    input  logic buffer_data_out_valid,

    // AXI4 write.
    output logic axi_wr_rst_n,
    input  logic axi_wr_done,
    output logic axi_wr_done_rst_n,
    output logic rst_n_multi_burst,

    // AXI-read phase status.
    input  logic tc_wr_mem,
    input  logic wr_index,
    input  logic tc_rd_index,

    // Dense preload address.
    output logic cnt_row_en,
    output logic cnt_row_rst_n,
    output logic rd_b_mem,

    // Index PISO.
    output logic piso_rst_n,
    output logic piso_load,
    output logic piso_shift_en,
    input  logic piso_empty,

    // Pointer look-ahead and segment scheduler.
    output logic ptr_stream_enable,
    output logic ptr_stream_rst_n,
    output logic segment_advance,
    input  logic segment_valid,
    input  logic segment_done,
    input  logic stream_end,
    input  logic nnz_limit_reached,
    input  logic end_rd_index,

    // SpMM bank stage.
    output logic cnt_stage_en,
    output logic cnt_stage_rst_n,

    // Pointer-SRAM preload address.
    output logic cnt_comp_idx_en,
    output logic cnt_comp_idx_rst_n
);

typedef enum logic [4:0] {
    IDLE,                   //Reset locali, attesa richiesta
    START,                  //Impulso axi_rd_start
    WAIT_AXI_RD,            //Attesa primo beat FIFO
    RD_FIFO_WR_MEM,         //Prime della FIFO registrata
    WR_B,                   //Preload streaming denso/puntatori
    WR_LAST_DATA,           //Commit ultimo beat ritardato
    START_NEXT_AXI_ADDR,    //Clear done e nuovo burst
    PTR_STREAM_RESET,       //Reset scheduler e start indici
    PTR_STREAM_FILL,        //Riempimento pointer FIFO
    RUN,                    //Issue densi e cambio segmento
    CLR_AXI_RD_DONE,		//Clear burst intermedio
    AXI_RD_NEXT_ADDR,		//Avvio burst successivo
    DRAIN,					//Svuotamento AXI e pipeline
    CLR_AXI_RD_DONE_DRAIN,	//Clear burst durante drain
    AXI_RD_NEXT_ADDR_DRAIN,	//Continua burst senza nuovi issue
    FLUSH_BUFFER,           //Chiude SIPO/buffer parziale
    WAIT_AXI_WR,            //Attende tutti gli AW/W/B
    DONE					//Impulso done
} state_e;

state_e current_state;
state_e next_state;

logic fifo_data_vld_q;
logic issued_any_q;
logic terminate_event;
logic terminate_pending_q;
logic terminate_compute;

assign terminate_event = nnz_limit_reached || stream_end || end_rd_index;
assign terminate_compute = terminate_event || terminate_pending_q;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_data_vld_q <= 1'b0;
    end else begin
        fifo_data_vld_q <= fifo_r_en;
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        issued_any_q <= 1'b0;
    end else if ((current_state == IDLE) || (current_state == START)) begin
        issued_any_q <= 1'b0;
    end else if (dense_rd_req) begin
        issued_any_q <= 1'b1;
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        terminate_pending_q <= 1'b0;
    end else if ((current_state == IDLE) || (current_state == START) || (current_state == DONE)) begin
        terminate_pending_q <= 1'b0;
    end else if (((current_state == PTR_STREAM_FILL) || (current_state == RUN)) && terminate_event) begin
        terminate_pending_q <= 1'b1;
    end
end

always_comb begin
    // Active defaults.  Individual states only override the controls they
    // own, avoiding the duplicated thousand-line output table of the legacy
    // FSM.
    next_state = current_state;

    axi_rd_rst_n = 1'b1;
    axi_rd_start = 1'b0;
    axi_rd_done_rst_n = 1'b1;

    fifo_r_en = 1'b0;
    fifo_rst_n = 1'b1;

    mem_rst_n = 1'b1;
    mem_wr_en = 1'b0;
    mem_idx_wr_en = 1'b0;
    dense_rd_req = 1'b0;

    buffer_rst_n = 1'b1;
    buffer_data_in_last = 1'b0;

    axi_wr_rst_n = 1'b1;
    axi_wr_done_rst_n = 1'b1;
    rst_n_multi_burst = 1'b1;

    cnt_row_en = 1'b0;
    cnt_row_rst_n = 1'b1;
    rd_b_mem = 1'b0;

    piso_rst_n = 1'b1;
    piso_load = 1'b0;
    piso_shift_en = 1'b0;

    ptr_stream_enable = 1'b0;
    ptr_stream_rst_n = 1'b0;
    segment_advance = 1'b0;

    cnt_stage_en = 1'b0;
    cnt_stage_rst_n = 1'b1;

    cnt_comp_idx_en = 1'b0;
    cnt_comp_idx_rst_n = 1'b1;

    done = 1'b0;

    unique case (current_state)
        IDLE: begin
            next_state = start ? START : IDLE;

            axi_rd_rst_n = 1'b0;
            axi_rd_done_rst_n = 1'b0;
            fifo_rst_n = 1'b0;
            mem_rst_n = 1'b0;
            buffer_rst_n = 1'b0;
            axi_wr_rst_n = 1'b0;
            axi_wr_done_rst_n = 1'b0;
            rst_n_multi_burst = 1'b0;
            cnt_row_rst_n = 1'b0;
            piso_rst_n = 1'b0;
            ptr_stream_rst_n = 1'b0;
            cnt_stage_rst_n = 1'b0;
            cnt_comp_idx_rst_n = 1'b0;
        end

        START: begin
            next_state = WAIT_AXI_RD;
            axi_rd_start = 1'b1;
        end

        WAIT_AXI_RD: begin
            if (!fifo_empty) begin
                next_state = RD_FIFO_WR_MEM;
            end
        end

        RD_FIFO_WR_MEM: begin
            fifo_r_en = 1'b1;
            next_state = WR_B;
        end

        WR_B: begin
            fifo_r_en = !fifo_empty;

            if (wr_index) begin
                mem_idx_wr_en = fifo_data_vld_q;
                cnt_comp_idx_en = fifo_data_vld_q;
            end else begin
                mem_wr_en = fifo_data_vld_q;
                cnt_row_en = fifo_data_vld_q;
            end

            if (axi_rd_done && axi_rd_available && fifo_empty && !fifo_data_vld_q) begin
                next_state = WR_LAST_DATA;
            end
        end

        WR_LAST_DATA: begin
            if (wr_index) begin
                mem_idx_wr_en = fifo_data_vld_q;
                cnt_comp_idx_en = fifo_data_vld_q;
            end else begin
                mem_wr_en = fifo_data_vld_q;
                cnt_row_en = fifo_data_vld_q;
            end

            next_state = tc_wr_mem ? PTR_STREAM_RESET : START_NEXT_AXI_ADDR;
        end

        START_NEXT_AXI_ADDR: begin
            axi_rd_start = 1'b1;
            axi_rd_done_rst_n = 1'b0;
            next_state = WAIT_AXI_RD;
        end

        PTR_STREAM_RESET: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            ptr_stream_rst_n = 1'b0;
            cnt_comp_idx_rst_n = 1'b0;
            axi_rd_start = 1'b1;
            axi_rd_done_rst_n = 1'b0;
            next_state = PTR_STREAM_FILL;
        end

        PTR_STREAM_FILL: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            ptr_stream_rst_n = 1'b1;
            ptr_stream_enable = 1'b1;

            if (terminate_compute) begin
                next_state = DRAIN;
            end else if (axi_rd_done && !tc_rd_index) begin
                next_state = CLR_AXI_RD_DONE;
            end else if (segment_valid) begin
                next_state = RUN;
            end
        end

        RUN: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            ptr_stream_rst_n = 1'b1;
            ptr_stream_enable = 1'b1;

            if (terminate_compute) begin
                next_state = DRAIN;
            end else if (axi_rd_done && !tc_rd_index) begin
                next_state = CLR_AXI_RD_DONE;
            end else if (!segment_valid) begin
                next_state = PTR_STREAM_FILL;
            end else if (segment_done) begin
                segment_advance = 1'b1;
                cnt_stage_en = 1'b1;
            end else if (!piso_empty && dense_req_ready) begin
                dense_rd_req = 1'b1;
                piso_shift_en = 1'b1;
            end
        end

        CLR_AXI_RD_DONE: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            ptr_stream_rst_n = 1'b1;
            ptr_stream_enable = 1'b1;
            axi_rd_done_rst_n = 1'b0;
            next_state = AXI_RD_NEXT_ADDR;
        end

        AXI_RD_NEXT_ADDR: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            ptr_stream_rst_n = 1'b1;
            ptr_stream_enable = 1'b1;
            axi_rd_start = 1'b1;
            next_state = RUN;
        end

        DRAIN: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            piso_shift_en = !piso_empty;
            ptr_stream_rst_n = 1'b1;

            if (axi_rd_done && !tc_rd_index) begin
                next_state = CLR_AXI_RD_DONE_DRAIN;
            end else if (dense_pipe_empty && tc_rd_index) begin
                next_state = FLUSH_BUFFER;
            end
        end

        CLR_AXI_RD_DONE_DRAIN: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            piso_shift_en = !piso_empty;
            ptr_stream_rst_n = 1'b1;
            axi_rd_done_rst_n = 1'b0;
            next_state = AXI_RD_NEXT_ADDR_DRAIN;
        end

        AXI_RD_NEXT_ADDR_DRAIN: begin
            rd_b_mem = 1'b1;
            piso_load = 1'b1;
            piso_shift_en = !piso_empty;
            ptr_stream_rst_n = 1'b1;
            axi_rd_start = 1'b1;
            next_state = DRAIN;
        end

        FLUSH_BUFFER: begin
            rd_b_mem = 1'b1;
            ptr_stream_rst_n = 1'b1;
            buffer_data_in_last = 1'b1;
            axi_wr_done_rst_n = 1'b0;
            next_state = WAIT_AXI_WR;
        end

        WAIT_AXI_WR: begin
            rd_b_mem = 1'b1;
            ptr_stream_rst_n = 1'b1;

            if (!issued_any_q) begin
                next_state = DONE;
            end else if (axi_wr_done) begin
                if (!buffer_data_out_valid) begin
                    next_state = DONE;
                end else begin
                    axi_wr_done_rst_n = 1'b0;
                end
            end
        end

        DONE: begin
            next_state = IDLE;
            done = 1'b1;

            axi_rd_rst_n = 1'b0;
            axi_rd_done_rst_n = 1'b0;
            fifo_rst_n = 1'b0;
            mem_rst_n = 1'b0;
            buffer_rst_n = 1'b0;
            axi_wr_rst_n = 1'b0;
            axi_wr_done_rst_n = 1'b0;
            rst_n_multi_burst = 1'b0;
            cnt_row_rst_n = 1'b0;
            piso_rst_n = 1'b0;
            ptr_stream_rst_n = 1'b0;
            cnt_stage_rst_n = 1'b0;
            cnt_comp_idx_rst_n = 1'b0;
        end

        default: begin
            next_state = IDLE;
        end
    endcase
end


endmodule
