`timescale 1ns/1ps

// AXI4 two-master priority mux.
//
// - priority_i = 1: only the high-priority master may start a new transaction.
// - priority_i = 0: only the low-priority master may start a new transaction.
// - A transaction owner is locked from AW to B, or from AR to RLAST.
// - At most one AXI transaction is outstanding through this mux.  This is
//   intentional: it makes a busy-driven ownership change protocol-safe.
module axi_busy_priority_mux #(
    parameter int unsigned AXI_ADDR_WIDTH = 32,
    parameter int unsigned AXI_DATA_WIDTH = 64,
    parameter int unsigned AXI_ID_WIDTH   = 2,
    parameter int unsigned AXI_USER_WIDTH = 1
) (
    input logic clk_i,
    input logic rst_ni,

    input logic priority_i,

    // Connect AXI masters here.
    AXI_BUS.Slave low_prio_slv,
    AXI_BUS.Slave high_prio_slv,

    // Connect the shared AXI slave path here.
    AXI_BUS.Master mst
);

    typedef enum logic [2:0] {
        ST_IDLE,
        ST_AW,
        ST_W,
        ST_B,
        ST_AR,
        ST_R
    } state_e;

    state_e state_q, state_d;
    logic   owner_q, owner_d; // 0 = low priority, 1 = high priority

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= ST_IDLE;
            owner_q <= 1'b0;
        end else begin
            state_q <= state_d;
            owner_q <= owner_d;
        end
    end

    always_comb begin
        state_d = state_q;
        owner_d = owner_q;

        // ------------------------------------------------------------
        // Default request values towards the shared slave
        // ------------------------------------------------------------
        mst.aw_id     = '0;
        mst.aw_addr   = '0;
        mst.aw_len    = '0;
        mst.aw_size   = '0;
        mst.aw_burst  = axi_pkg::BURST_INCR;
        mst.aw_lock   = '0;
        mst.aw_cache  = '0;
        mst.aw_prot   = '0;
        mst.aw_qos    = '0;
        mst.aw_region = '0;
        mst.aw_atop   = '0;
        mst.aw_user   = '0;
        mst.aw_valid  = 1'b0;

        mst.w_data    = '0;
        mst.w_strb    = '0;
        mst.w_last    = 1'b0;
        mst.w_user    = '0;
        mst.w_valid   = 1'b0;

        mst.b_ready   = 1'b0;

        mst.ar_id     = '0;
        mst.ar_addr   = '0;
        mst.ar_len    = '0;
        mst.ar_size   = '0;
        mst.ar_burst  = axi_pkg::BURST_INCR;
        mst.ar_lock   = '0;
        mst.ar_cache  = '0;
        mst.ar_prot   = '0;
        mst.ar_qos    = '0;
        mst.ar_region = '0;
        mst.ar_user   = '0;
        mst.ar_valid  = 1'b0;

        mst.r_ready   = 1'b0;

        // ------------------------------------------------------------
        // Default responses towards both masters
        // ------------------------------------------------------------
        low_prio_slv.aw_ready  = 1'b0;
        low_prio_slv.w_ready   = 1'b0;
        low_prio_slv.ar_ready  = 1'b0;
        low_prio_slv.b_id      = '0;
        low_prio_slv.b_resp    = axi_pkg::RESP_OKAY;
        low_prio_slv.b_user    = '0;
        low_prio_slv.b_valid   = 1'b0;
        low_prio_slv.r_id      = '0;
        low_prio_slv.r_data    = '0;
        low_prio_slv.r_resp    = axi_pkg::RESP_OKAY;
        low_prio_slv.r_last    = 1'b0;
        low_prio_slv.r_user    = '0;
        low_prio_slv.r_valid   = 1'b0;

        high_prio_slv.aw_ready = 1'b0;
        high_prio_slv.w_ready  = 1'b0;
        high_prio_slv.ar_ready = 1'b0;
        high_prio_slv.b_id     = '0;
        high_prio_slv.b_resp   = axi_pkg::RESP_OKAY;
        high_prio_slv.b_user   = '0;
        high_prio_slv.b_valid  = 1'b0;
        high_prio_slv.r_id     = '0;
        high_prio_slv.r_data   = '0;
        high_prio_slv.r_resp   = axi_pkg::RESP_OKAY;
        high_prio_slv.r_last   = 1'b0;
        high_prio_slv.r_user   = '0;
        high_prio_slv.r_valid  = 1'b0;

        unique case (state_q)
            ST_IDLE: begin
                // Strict ownership: while priority_i is high, the low-priority
                // master cannot start a new transaction even if the high-priority
                // master has not asserted VALID yet.
                owner_d = priority_i;

                if (priority_i) begin
                    // Fixed AW-before-AR ordering if both are presented together.
                    if (high_prio_slv.aw_valid) begin
                        state_d = ST_AW;
                    end else if (high_prio_slv.ar_valid) begin
                        state_d = ST_AR;
                    end
                end else begin
                    if (low_prio_slv.aw_valid) begin
                        state_d = ST_AW;
                    end else if (low_prio_slv.ar_valid) begin
                        state_d = ST_AR;
                    end
                end
            end

            ST_AW: begin
                if (owner_q) begin
                    mst.aw_id     = high_prio_slv.aw_id;
                    mst.aw_addr   = high_prio_slv.aw_addr;
                    mst.aw_len    = high_prio_slv.aw_len;
                    mst.aw_size   = high_prio_slv.aw_size;
                    mst.aw_burst  = high_prio_slv.aw_burst;
                    mst.aw_lock   = high_prio_slv.aw_lock;
                    mst.aw_cache  = high_prio_slv.aw_cache;
                    mst.aw_prot   = high_prio_slv.aw_prot;
                    mst.aw_qos    = high_prio_slv.aw_qos;
                    mst.aw_region = high_prio_slv.aw_region;
                    mst.aw_atop   = high_prio_slv.aw_atop;
                    mst.aw_user   = high_prio_slv.aw_user;
                    mst.aw_valid  = high_prio_slv.aw_valid;
                    high_prio_slv.aw_ready = mst.aw_ready;
                end else begin
                    mst.aw_id     = low_prio_slv.aw_id;
                    mst.aw_addr   = low_prio_slv.aw_addr;
                    mst.aw_len    = low_prio_slv.aw_len;
                    mst.aw_size   = low_prio_slv.aw_size;
                    mst.aw_burst  = low_prio_slv.aw_burst;
                    mst.aw_lock   = low_prio_slv.aw_lock;
                    mst.aw_cache  = low_prio_slv.aw_cache;
                    mst.aw_prot   = low_prio_slv.aw_prot;
                    mst.aw_qos    = low_prio_slv.aw_qos;
                    mst.aw_region = low_prio_slv.aw_region;
                    mst.aw_atop   = low_prio_slv.aw_atop;
                    mst.aw_user   = low_prio_slv.aw_user;
                    mst.aw_valid  = low_prio_slv.aw_valid;
                    low_prio_slv.aw_ready = mst.aw_ready;
                end

                if (mst.aw_valid && mst.aw_ready) begin
                    state_d = ST_W;
                end
            end

            ST_W: begin
                if (owner_q) begin
                    mst.w_data   = high_prio_slv.w_data;
                    mst.w_strb   = high_prio_slv.w_strb;
                    mst.w_last   = high_prio_slv.w_last;
                    mst.w_user   = high_prio_slv.w_user;
                    mst.w_valid  = high_prio_slv.w_valid;
                    high_prio_slv.w_ready = mst.w_ready;
                end else begin
                    mst.w_data   = low_prio_slv.w_data;
                    mst.w_strb   = low_prio_slv.w_strb;
                    mst.w_last   = low_prio_slv.w_last;
                    mst.w_user   = low_prio_slv.w_user;
                    mst.w_valid  = low_prio_slv.w_valid;
                    low_prio_slv.w_ready = mst.w_ready;
                end

                if (mst.w_valid && mst.w_ready && mst.w_last) begin
                    state_d = ST_B;
                end
            end

            ST_B: begin
                if (owner_q) begin
                    high_prio_slv.b_id    = mst.b_id;
                    high_prio_slv.b_resp  = mst.b_resp;
                    high_prio_slv.b_user  = mst.b_user;
                    high_prio_slv.b_valid = mst.b_valid;
                    mst.b_ready           = high_prio_slv.b_ready;
                end else begin
                    low_prio_slv.b_id      = mst.b_id;
                    low_prio_slv.b_resp    = mst.b_resp;
                    low_prio_slv.b_user    = mst.b_user;
                    low_prio_slv.b_valid   = mst.b_valid;
                    mst.b_ready            = low_prio_slv.b_ready;
                end

                if (mst.b_valid && mst.b_ready) begin
                    state_d = ST_IDLE;
                end
            end

            ST_AR: begin
                if (owner_q) begin
                    mst.ar_id     = high_prio_slv.ar_id;
                    mst.ar_addr   = high_prio_slv.ar_addr;
                    mst.ar_len    = high_prio_slv.ar_len;
                    mst.ar_size   = high_prio_slv.ar_size;
                    mst.ar_burst  = high_prio_slv.ar_burst;
                    mst.ar_lock   = high_prio_slv.ar_lock;
                    mst.ar_cache  = high_prio_slv.ar_cache;
                    mst.ar_prot   = high_prio_slv.ar_prot;
                    mst.ar_qos    = high_prio_slv.ar_qos;
                    mst.ar_region = high_prio_slv.ar_region;
                    mst.ar_user   = high_prio_slv.ar_user;
                    mst.ar_valid  = high_prio_slv.ar_valid;
                    high_prio_slv.ar_ready = mst.ar_ready;
                end else begin
                    mst.ar_id     = low_prio_slv.ar_id;
                    mst.ar_addr   = low_prio_slv.ar_addr;
                    mst.ar_len    = low_prio_slv.ar_len;
                    mst.ar_size   = low_prio_slv.ar_size;
                    mst.ar_burst  = low_prio_slv.ar_burst;
                    mst.ar_lock   = low_prio_slv.ar_lock;
                    mst.ar_cache  = low_prio_slv.ar_cache;
                    mst.ar_prot   = low_prio_slv.ar_prot;
                    mst.ar_qos    = low_prio_slv.ar_qos;
                    mst.ar_region = low_prio_slv.ar_region;
                    mst.ar_user   = low_prio_slv.ar_user;
                    mst.ar_valid  = low_prio_slv.ar_valid;
                    low_prio_slv.ar_ready = mst.ar_ready;
                end

                if (mst.ar_valid && mst.ar_ready) begin
                    state_d = ST_R;
                end
            end

            ST_R: begin
                if (owner_q) begin
                    high_prio_slv.r_id    = mst.r_id;
                    high_prio_slv.r_data  = mst.r_data;
                    high_prio_slv.r_resp  = mst.r_resp;
                    high_prio_slv.r_last  = mst.r_last;
                    high_prio_slv.r_user  = mst.r_user;
                    high_prio_slv.r_valid = mst.r_valid;
                    mst.r_ready           = high_prio_slv.r_ready;
                end else begin
                    low_prio_slv.r_id      = mst.r_id;
                    low_prio_slv.r_data    = mst.r_data;
                    low_prio_slv.r_resp    = mst.r_resp;
                    low_prio_slv.r_last    = mst.r_last;
                    low_prio_slv.r_user    = mst.r_user;
                    low_prio_slv.r_valid   = mst.r_valid;
                    mst.r_ready            = low_prio_slv.r_ready;
                end

                if (mst.r_valid && mst.r_ready && mst.r_last) begin
                    state_d = ST_IDLE;
                end
            end

            default: begin
                state_d = ST_IDLE;
                owner_d = 1'b0;
            end
        endcase
    end

endmodule


// AXI request gate.
//
// While enable_i is low, no new transaction can start. Once an AW or AR
// handshake has been accepted, the complete W/B or R transaction is allowed
// to drain even if enable_i changes. At most one transaction is outstanding.
module axi_request_gate #(
    parameter int unsigned AXI_ADDR_WIDTH = 32,
    parameter int unsigned AXI_DATA_WIDTH = 64,
    parameter int unsigned AXI_ID_WIDTH   = 2,
    parameter int unsigned AXI_USER_WIDTH = 1
) (
    input logic clk_i,
    input logic rst_ni,
    input logic enable_i,
    AXI_BUS.Slave slv,
    AXI_BUS.Master mst
);

    typedef enum logic [1:0] {
        G_IDLE,
        G_W,
        G_B,
        G_R
    } gate_state_e;

    gate_state_e state_q, state_d;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) state_q <= G_IDLE;
        else         state_q <= state_d;
    end

    always_comb begin
        state_d = state_q;

        mst.aw_id     = slv.aw_id;
        mst.aw_addr   = slv.aw_addr;
        mst.aw_len    = slv.aw_len;
        mst.aw_size   = slv.aw_size;
        mst.aw_burst  = slv.aw_burst;
        mst.aw_lock   = slv.aw_lock;
        mst.aw_cache  = slv.aw_cache;
        mst.aw_prot   = slv.aw_prot;
        mst.aw_qos    = slv.aw_qos;
        mst.aw_region = slv.aw_region;
        mst.aw_atop   = slv.aw_atop;
        mst.aw_user   = slv.aw_user;
        mst.aw_valid  = 1'b0;
        slv.aw_ready  = 1'b0;

        mst.w_data    = slv.w_data;
        mst.w_strb    = slv.w_strb;
        mst.w_last    = slv.w_last;
        mst.w_user    = slv.w_user;
        mst.w_valid   = 1'b0;
        slv.w_ready   = 1'b0;

        slv.b_id      = mst.b_id;
        slv.b_resp    = mst.b_resp;
        slv.b_user    = mst.b_user;
        slv.b_valid   = 1'b0;
        mst.b_ready   = 1'b0;

        mst.ar_id     = slv.ar_id;
        mst.ar_addr   = slv.ar_addr;
        mst.ar_len    = slv.ar_len;
        mst.ar_size   = slv.ar_size;
        mst.ar_burst  = slv.ar_burst;
        mst.ar_lock   = slv.ar_lock;
        mst.ar_cache  = slv.ar_cache;
        mst.ar_prot   = slv.ar_prot;
        mst.ar_qos    = slv.ar_qos;
        mst.ar_region = slv.ar_region;
        mst.ar_user   = slv.ar_user;
        mst.ar_valid  = 1'b0;
        slv.ar_ready  = 1'b0;

        slv.r_id      = mst.r_id;
        slv.r_data    = mst.r_data;
        slv.r_resp    = mst.r_resp;
        slv.r_last    = mst.r_last;
        slv.r_user    = mst.r_user;
        slv.r_valid   = 1'b0;
        mst.r_ready   = 1'b0;

        unique case (state_q)
            G_IDLE: begin
                if (enable_i) begin
                    // Fixed AW-before-AR ordering when both are valid.
                    if (slv.aw_valid) begin
                        mst.aw_valid = slv.aw_valid;
                        slv.aw_ready = mst.aw_ready;
                        if (mst.aw_valid && mst.aw_ready) state_d = G_W;
                    end else if (slv.ar_valid) begin
                        mst.ar_valid = slv.ar_valid;
                        slv.ar_ready = mst.ar_ready;
                        if (mst.ar_valid && mst.ar_ready) state_d = G_R;
                    end
                end
            end

            G_W: begin
                mst.w_valid = slv.w_valid;
                slv.w_ready = mst.w_ready;
                if (mst.w_valid && mst.w_ready && mst.w_last) state_d = G_B;
            end

            G_B: begin
                slv.b_valid = mst.b_valid;
                mst.b_ready = slv.b_ready;
                if (mst.b_valid && mst.b_ready) state_d = G_IDLE;
            end

            G_R: begin
                slv.r_valid = mst.r_valid;
                mst.r_ready = slv.r_ready;
                if (mst.r_valid && mst.r_ready && mst.r_last) state_d = G_IDLE;
            end

            default: state_d = G_IDLE;
        endcase
    end

endmodule
