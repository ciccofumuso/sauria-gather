`timescale 1ns/1ps

module axi_lite_to_tlul #(
    parameter int unsigned AW = 32,
    parameter int unsigned DW = 32
) (
    input  logic             clk_i,
    input  logic             rst_ni,

    // Interfaccia Slave AXI-Lite
    input  logic [AW-1:0]    axi_awaddr,
    input  logic             axi_awvalid,
    output logic             axi_awready,
    input  logic [DW-1:0]    axi_wdata,
    input  logic [DW/8-1:0]  axi_wstrb,
    input  logic             axi_wvalid,
    output logic             axi_wready,
    output logic [1:0]       axi_bresp,
    output logic             axi_bvalid,
    input  logic             axi_bready,

    input  logic [AW-1:0]    axi_araddr,
    input  logic             axi_arvalid,
    output logic             axi_arready,
    output logic [DW-1:0]    axi_rdata,
    output logic [1:0]       axi_rresp,
    output logic             axi_rvalid,
    input  logic             axi_rready,

    // Interfaccia Master TL-UL
    output tlul_pkg::tl_h2d_t tl_o,
    input  tlul_pkg::tl_d2h_t tl_i
);

    // Stati della FSM
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_TL_WRITE,
        ST_AXI_BVALID,
        ST_TL_READ,
        ST_AXI_RVALID
    } state_e;

    state_e state_q, state_d;

    logic [AW-1:0] addr_reg;
    logic [DW-1:0] data_reg;
    logic [DW/8-1:0] mask_reg;

    // Blocco Sequenziale: Cattura i dati in modo stabile sul fronte di clock
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q  <= ST_IDLE;
            addr_reg <= '0;
            data_reg <= '0;
            mask_reg <= '0;
        end else begin
            state_q <= state_d;
            if (state_q == ST_IDLE) begin
                if (axi_awvalid && axi_wvalid) begin
                    addr_reg <= axi_awaddr;
                    data_reg <= axi_wdata;
                    mask_reg <= axi_wstrb;
                end else if (axi_arvalid) begin
                    addr_reg <= axi_araddr;
                end
            end
        end
    end

    // Blocco Combinatorio Corretto
    // =======================================================
    // Blocco Combinatorio con Mascheramento di Indirizzo Base
    // =======================================================
    always_comb begin
        state_d     = state_q;
        axi_awready = 1'b0;
        axi_wready  = 1'b0;
        axi_bvalid  = 1'b0;
        axi_bresp   = 2'b00;
        axi_arready = 1'b0;
        axi_rvalid  = 1'b0;
        axi_rresp   = 2'b00;
        axi_rdata   = tl_i.d_data;

        // Reset dei segnali TL-UL di default
        tl_o.a_valid   = 1'b0;
        tl_o.a_address = '0;
        tl_o.a_data    = '0;
        tl_o.a_mask    = '0;
        tl_o.a_opcode  = tlul_pkg::Get;
        tl_o.a_size    = 2'd2;
        tl_o.a_param   = 3'd0;
        tl_o.a_source  = '0;
        tl_o.a_user    = '0;
        tl_o.d_ready   = 1'b0;

        case (state_q)
            ST_IDLE: begin
                if (axi_awvalid && axi_wvalid) begin
                    state_d = ST_TL_WRITE;
                end else if (axi_arvalid) begin
                    state_d = ST_TL_READ;
                end
            end

            ST_TL_WRITE: begin
                tl_o.a_valid   = 1'b1;
                tl_o.a_opcode  = tlul_pkg::PutFullData;
                // CRITICO: Mascheriamo l'indirizzo per rimuovere il prefisso 32'h4000_0000 di SAURIA
                tl_o.a_address = addr_reg & 32'h0000_FFFF; 
                tl_o.a_data    = data_reg;
                tl_o.a_mask    = mask_reg;
                
                if (tl_i.a_ready) begin
                    state_d = ST_AXI_BVALID;
                end
            end

            ST_AXI_BVALID: begin
                tl_o.d_ready = 1'b1;
                if (tl_i.d_valid) begin
                    axi_awready = 1'b1;
                    axi_wready  = 1'b1;
                    axi_bvalid  = 1'b1;
                    state_d     = ST_IDLE;
                end
            end

            ST_TL_READ: begin
                tl_o.a_valid   = 1'b1;
                tl_o.a_opcode  = tlul_pkg::Get;
                // CRITICO: Mascheriamo l'indirizzo anche per le letture CSR
                tl_o.a_address = addr_reg & 32'h0000_FFFF; 
                tl_o.a_mask    = '1;
                
                if (tl_i.a_ready) begin
                    state_d = ST_AXI_RVALID;
                end
            end

            ST_AXI_RVALID: begin
                tl_o.d_ready = 1'b1;
                if (tl_i.d_valid) begin
                    axi_arready = 1'b1;
                    axi_rvalid  = 1'b1;
                    state_d     = ST_IDLE;
                end
            end
            
            default: state_d = ST_IDLE;
        endcase
    end
endmodule
