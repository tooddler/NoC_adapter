// - NOC SLAVE UNIT -
// TODO: ∑¥—π–≈∫≈…Ë÷√
module noc2axi4_slave #(
    parameter   DATA_WIDTH      =   128                ,
    parameter   FLIT_NUM_MAX    =   16                 ,
    parameter   ID_WIDTH        =   4                  ,
    parameter   VIRTUAL_CH_NUM  =   16                 ,
    parameter   FIFO_ADDR_WIDTH =   16                 ,
    parameter   Source_ID       =   {ID_WIDTH{1'b1}}   ,

    parameter   AXI_ID_WIDTH    =   4                  ,
    parameter   AXI_ADDR_WIDTH  =   32                 ,
    parameter   AXI_DATA_WIDTH  =   DATA_WIDTH         ,
    parameter   FREQ_DDR_CLK    =   200                ,
    parameter   FREQ_NOC_CLK    =   100
)(
    // noc domain clk
    input                                   noc_clk       ,
    input                                   noc_rst_n     ,
    // user domain clk
    input                                   ddr_clk       ,
    input                                   ddr_rst_n     ,
    
    // - interact with NOC -
    input     [DATA_WIDTH : 0]              noc2axi_data  ,  //noc2axi data  
    input                                   s_is_head     ,
    input                                   s_is_tail     ,    
    output                                  nsu_busy      ,

    output    [DATA_WIDTH : 0]              nocdata       ,  //axi2noc data
    output                                  m_is_head     ,
    output                                  m_is_tail     ,
    input                                   noc_ready     ,

    // - interact with DDR -
     // Master Interface Write Address Ports
    output     [AXI_ID_WIDTH - 1:0]         m_axi_awid    ,
    output     [AXI_ADDR_WIDTH - 1:0]       m_axi_awaddr  ,
    output     [7:0]                        m_axi_awlen   ,
    output     [2:0]                        m_axi_awsize  ,
    output     [1:0]                        m_axi_awburst ,
    output     [0:0]                        m_axi_awlock  ,
    output     [3:0]                        m_axi_awcache ,
    output     [2:0]                        m_axi_awprot  ,
    output     [3:0]                        m_axi_awqos   ,
    output                                  m_axi_awvalid ,
    input                                   m_axi_awready ,
     // Master Interface Write Data Port
    output     [AXI_DATA_WIDTH - 1:0]       m_axi_wdata   ,
    output     [AXI_DATA_WIDTH/8 - 1:0]     m_axi_wstrb   ,
    output                                  m_axi_wlast   ,
    output                                  m_axi_wvalid  ,
    input                                   m_axi_wready  ,
     // Master Interface Write Response Port
    input      [AXI_ID_WIDTH - 1:0]         m_axi_bid     ,
    input      [1:0]                        m_axi_bresp   ,
    input                                   m_axi_bvalid  ,
    output                                  m_axi_bready  ,
     // Master Interface Read Address Ports 
    output     [AXI_ID_WIDTH - 1:0]         m_axi_arid    ,
    output     [AXI_ADDR_WIDTH - 1:0]       m_axi_araddr  ,
    output     [7:0]                        m_axi_arlen   ,
    output     [2:0]                        m_axi_arsize  ,
    output     [1:0]                        m_axi_arburst ,
    output     [0:0]                        m_axi_arlock  ,
    output     [3:0]                        m_axi_arcache ,
    output     [2:0]                        m_axi_arprot  ,
    output     [3:0]                        m_axi_arqos   ,
    output                                  m_axi_arvalid ,
    input                                   m_axi_arready ,
     // Master Interface Read Data Ports
    input      [AXI_ID_WIDTH - 1:0]         m_axi_rid     ,
    input      [AXI_DATA_WIDTH - 1:0]       m_axi_rdata   ,
    input      [1:0]                        m_axi_rresp   ,
    input                                   m_axi_rlast   ,
    input                                   m_axi_rvalid  ,
    output                                  m_axi_rready  
);

wire  [AXI_ADDR_WIDTH - 1 : 0]              axi_addr            ;
wire  [AXI_ADDR_WIDTH - 1 : 0]              re_pack             ; // {basepack_order, total_lens}
wire  [AXI_ADDR_WIDTH - 1 : 0]              re_pack_for_pack    ;
wire  [7:0]                                 axi_len             ;
wire  [2:0]                                 axi_type            ;
wire  [ID_WIDTH - 1 : 0]                    source_id           ;
wire  [VIRTUAL_CH_NUM - 1 : 0]              rd_en               ;
wire  [DATA_WIDTH - 1 : 0]                  data_out            ;

wire  [VIRTUAL_CH_NUM - 1 : 0]              empty_vc            ;
wire  [VIRTUAL_CH_NUM - 1 : 0]              pack_num            ;
wire                                        nocpack_done        ;

wire  [DATA_WIDTH - 1 : 0]                  data2ddr            ;
wire                                        data2ddr_last       ;
wire                                        en_data2ddr         ;
wire                                        data_fifo_full      ;
wire  [2*AXI_ADDR_WIDTH+ID_WIDTH+10:0]      cmd2ddr             ;
wire                                        en_cmd2ddr          ;
wire                                        cmd_fifo_full       ;

wire                                        type_wr_done        ;
wire                                        type_rd_req_done    ;

wire [DATA_WIDTH : 0]                       axi2nocdata         ;
wire                                        head                ;
wire                                        tail                ;
wire                                        noc_buf_full        ;
wire                                        noc_buf_empty       ;

wire                                        TypeRdReqFinish     ;

// - reorder buffer -
nsu_reorder_module #(
    .DATA_WIDTH       ( DATA_WIDTH      ),
    .AXI_ADDR_WIDTH   ( AXI_ADDR_WIDTH  ),
    .FLIT_NUM_MAX     ( FLIT_NUM_MAX    ),
    .ID_WIDTH         ( ID_WIDTH        ),
    .VIRTUAL_CH_NUM   ( VIRTUAL_CH_NUM  )
) u_nsu_reorder_module(
    .noc_clk          ( noc_clk          ),
    .noc_rst_n        ( noc_rst_n        ),
    .noc2axi_data     ( noc2axi_data     ),
    .s_is_head        ( s_is_head        ),
    .s_is_tail        ( s_is_tail        ),
    .nsu_busy         ( nsu_busy         ),

    .re_pack          ( re_pack          ),

    .axi_addr         ( axi_addr         ),
    .axi_len          ( axi_len          ),
    .axi_type         ( axi_type         ),
    .source_id        ( source_id        ),
    .rd_en            ( rd_en            ),
    .data_out         ( data_out         ),
    .empty_vc         ( empty_vc         ),
    .pack_num         ( pack_num         ),
    .nocpack_done     ( nocpack_done     )
);

// - Depacketizing -
nsu_depacket_proc #(
    .DATA_WIDTH        ( DATA_WIDTH      ),
    .AXI_ADDR_WIDTH    ( AXI_ADDR_WIDTH  ),
    .FIFO_ADDR_WIDTH   ( FIFO_ADDR_WIDTH ),
    .ID_WIDTH          ( ID_WIDTH        ),
    .VIRTUAL_CH_NUM    ( VIRTUAL_CH_NUM  ),
    .AXI_ID_WIDTH      ( AXI_ID_WIDTH    )
) u_nsu_depacket_proc(
    .noc_clk           ( noc_clk           ),
    .noc_rst_n         ( noc_rst_n         ),
    .axi_addr          ( axi_addr          ),
    .axi_len           ( axi_len           ),
    .axi_type          ( axi_type          ),
    .source_id         ( source_id         ),
    .re_pack           ( re_pack           ),
    .rd_en             ( rd_en             ),
    .data_in           ( data_out          ),
    .empty_vc          ( empty_vc          ),
    .pack_num          ( pack_num          ),
    .nocpack_done      ( nocpack_done      ),

    .nsu_busy          ( nsu_busy          ),
    .type_wr_done      ( type_wr_done      ),
    .type_rd_req_done  ( type_rd_req_done  ),

    .data2ddr          ( data2ddr          ),
    .data2ddr_last     ( data2ddr_last     ),
    .en_data2ddr       ( en_data2ddr       ),
    .data_fifo_full    ( data_fifo_full    ),
    .cmd2ddr           ( cmd2ddr           ),
    .en_cmd2ddr        ( en_cmd2ddr        ),
    .cmd_fifo_full     ( cmd_fifo_full     )
);

// - rate matching and asynchronous data boundary crossing -
nsu_asyn_cross_noc2axi #(
    .DATA_WIDTH       ( DATA_WIDTH      ),
    .FIFO_ADDR_WIDTH  ( FIFO_ADDR_WIDTH ),
    .ID_WIDTH         ( ID_WIDTH        ),
    .VIRTUAL_CH_NUM   ( VIRTUAL_CH_NUM  ),
    .AXI_ID_WIDTH     ( AXI_ID_WIDTH    ),
    .AXI_ADDR_WIDTH   ( AXI_ADDR_WIDTH  ),

    .FREQ_DDR_CLK     ( FREQ_DDR_CLK    ),
    .FREQ_NOC_CLK     ( FREQ_NOC_CLK    )
) u_nsu_asyn_cross_noc2axi(
    .noc_clk          ( noc_clk          ),
    .noc_rst_n        ( noc_rst_n        ),
    .ddr_clk          ( ddr_clk          ),
    .ddr_rst_n        ( ddr_rst_n        ),
    .data2ddr         ( data2ddr         ),
    .data2ddr_last    ( data2ddr_last    ),
    .en_data2ddr      ( en_data2ddr      ),
    .data_fifo_full   ( data_fifo_full   ),
    .cmd2ddr          ( cmd2ddr          ),
    .en_cmd2ddr       ( en_cmd2ddr       ),
    .cmd_fifo_full    ( cmd_fifo_full    ),

    .re_pack          ( re_pack_for_pack ),
    
    .m_axi_awid       ( m_axi_awid       ),
    .m_axi_awaddr     ( m_axi_awaddr     ),
    .m_axi_awlen      ( m_axi_awlen      ),
    .m_axi_awsize     ( m_axi_awsize     ),
    .m_axi_awburst    ( m_axi_awburst    ),
    .m_axi_awlock     ( m_axi_awlock     ),
    .m_axi_awcache    ( m_axi_awcache    ),
    .m_axi_awprot     ( m_axi_awprot     ),
    .m_axi_awqos      ( m_axi_awqos      ),
    .m_axi_awvalid    ( m_axi_awvalid    ),
    .m_axi_awready    ( m_axi_awready    ),

    .m_axi_wdata      ( m_axi_wdata      ),
    .m_axi_wstrb      ( m_axi_wstrb      ),
    .m_axi_wlast      ( m_axi_wlast      ),
    .m_axi_wvalid     ( m_axi_wvalid     ),
    .m_axi_wready     ( m_axi_wready     ),

    .m_axi_arid       ( m_axi_arid       ),
    .m_axi_araddr     ( m_axi_araddr     ),
    .m_axi_arlen      ( m_axi_arlen      ),
    .m_axi_arsize     ( m_axi_arsize     ),
    .m_axi_arburst    ( m_axi_arburst    ),
    .m_axi_arlock     ( m_axi_arlock     ),
    .m_axi_arcache    ( m_axi_arcache    ),
    .m_axi_arprot     ( m_axi_arprot     ),
    .m_axi_arqos      ( m_axi_arqos      ),
    .m_axi_arvalid    ( m_axi_arvalid    ),
    .m_axi_arready    ( m_axi_arready    ),

    .m_axi_bvalid     ( m_axi_bvalid     ),
    .m_axi_bready     ( m_axi_bready     ),
    .m_axi_rlast      ( m_axi_rlast      ),
    .m_axi_rvalid     ( m_axi_rvalid     ),
    .m_axi_rready     ( m_axi_rready     ),

    .TypeRdReqFinish  ( TypeRdReqFinish  ),
    .type_wr_done     ( type_wr_done     ),
    .type_rd_req_done ( type_rd_req_done )
);

nsu_packet_proc #(
    .DATA_WIDTH         ( DATA_WIDTH        ),
    .ID_WIDTH           ( ID_WIDTH          ),
    .VIRTUAL_CH_NUM     ( VIRTUAL_CH_NUM    ),
    .AXI_ID_WIDTH       ( AXI_ID_WIDTH      ),
    .AXI_ADDR_WIDTH     ( AXI_ADDR_WIDTH    ),
    .FLIT_NUM_MAX       ( FLIT_NUM_MAX      ),
    .Source_ID          ( Source_ID         )
) u_nsu_packet_proc(
    .ddr_clk            ( ddr_clk           ),
    .ddr_rst_n          ( ddr_rst_n         ),

    .m_axi_awvalid      ( m_axi_awvalid     ),
    .m_axi_awready      ( m_axi_awready     ),
    .m_axi_awlen        ( m_axi_awlen       ),
    .m_axi_arid         ( m_axi_arid        ),
    .m_axi_arvalid      ( m_axi_arvalid     ),
    .m_axi_arready      ( m_axi_arready     ),
    .m_axi_arlen        ( m_axi_arlen       ),
    .m_axi_wvalid       ( m_axi_wvalid      ),
    .m_axi_wready       ( m_axi_wready      ),

    .re_pack            ( re_pack_for_pack  ),

    .noc_outdata        ( axi2nocdata       ),
    .m_is_head          ( head              ),
    .m_is_tail          ( tail              ),

    .m_axi_bid          ( m_axi_bid         ),
    .m_axi_bresp        ( m_axi_bresp       ),
    .m_axi_bvalid       ( m_axi_bvalid      ),
    .m_axi_bready       ( m_axi_bready      ),
    .m_axi_rid          ( m_axi_rid         ),
    .m_axi_rdata        ( m_axi_rdata       ),
    .m_axi_rresp        ( m_axi_rresp       ),
    .m_axi_rlast        ( m_axi_rlast       ),
    .m_axi_rvalid       ( m_axi_rvalid      ),
    .m_axi_rready       ( m_axi_rready      ),
    .TypeRdReqFinish    ( TypeRdReqFinish   )
);

nsu_asyn_cross_axi2noc #(
    .DATA_WIDTH       ( DATA_WIDTH      ),
    .FIFO_ADDR_WIDTH  ( FIFO_ADDR_WIDTH ),
    .ID_WIDTH         ( ID_WIDTH        ),
    .VIRTUAL_CH_NUM   ( VIRTUAL_CH_NUM  ),
    .AXI_ADDR_WIDTH   ( AXI_ADDR_WIDTH  )
)u_nsu_asyn_cross_axi2noc(
    .noc_clk          ( noc_clk          ),
    .noc_rst_n        ( noc_rst_n        ),
    .ddr_clk          ( ddr_clk          ),
    .ddr_rst_n        ( ddr_rst_n        ),

    .axi2nocdata      ( axi2nocdata      ),
    .head             ( head             ),
    .tail             ( tail             ),

    .nocdata          ( nocdata          ),
    .m_is_head        ( m_is_head        ),
    .m_is_tail        ( m_is_tail        ),
    .nsu_ready        (nsu_ready         ),
    .noc_buf_full     ( noc_buf_full     ),
    .noc_buf_empty    ( noc_buf_empty    )
);

endmodule //noc2axi4_slave
