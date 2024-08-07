/* 
    - NOC MASTER UNIT -
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn
*/

module noc2axi4_master #(
    parameter   VIRTUAL_CH_NUM  =      8              ,
    parameter   DATA_WIDTH      =      128            ,
    parameter   ID_WIDTH        =      4              ,
    parameter   Source_ID       =  {ID_WIDTH{1'b0}}   ,
    parameter   BUFFER_DEPTH    =      8              ,
    parameter   FLIT_NUM_MAX    =      16             ,
    parameter   FIFO_ADDR_WIDTH =      16             ,
    parameter   AXI_ID_WIDTH    =      4              ,
    parameter   AXI_ADDR_WIDTH  =      32             ,
    parameter   PS_INTERLEAVING =      32'h0000_1000  ,
    parameter   PL_INTERLEAVING =  2*PS_INTERLEAVING  ,
    parameter   AXI_DATA_WIDTH  =      DATA_WIDTH
)(
    // noc domain clk
    input                                   noc_clk       ,
    input                                   noc_rst_n     ,
    // user domain clk
    input                                   axi_clk       ,
    input                                   axi_rst_n     ,

    // noc to master unit interface
    input     [DATA_WIDTH : 0]              noc2axi_data  , //noc2axi data  
    input                                   s_is_head     ,
    input                                   s_is_tail     ,             
    output                                  buffer_busy   ,
    
    output    [DATA_WIDTH : 0]              nocdata       , //axi2noc data
    output                                  m_is_head     ,
    output                                  m_is_tail     ,    

    // - user port -
     // Slave Interface Write Address Ports
    input     [AXI_ID_WIDTH - 1:0]          s_axi_awid    ,
    input     [AXI_ADDR_WIDTH - 1:0]        s_axi_awaddr  ,
    input     [7:0]                         s_axi_awlen   ,
    input     [2:0]                         s_axi_awsize  ,
    input     [1:0]                         s_axi_awburst ,
    input     [0:0]                         s_axi_awlock  ,
    input     [3:0]                         s_axi_awcache ,
    input     [2:0]                         s_axi_awprot  ,
    input     [3:0]                         s_axi_awqos   ,
    input                                   s_axi_awvalid ,
    output                                  s_axi_awready ,
     // Slave Interface Write Data Port
    input     [AXI_DATA_WIDTH - 1:0]        s_axi_wdata   ,
    input     [AXI_DATA_WIDTH/8 - 1:0]      s_axi_wstrb   ,
    input                                   s_axi_wlast   ,
    input                                   s_axi_wvalid  ,
    output                                  s_axi_wready  ,
     // Slave Interface Write Response Port
    output    [AXI_ID_WIDTH - 1:0]          s_axi_bid     ,
    output    [1:0]                         s_axi_bresp   ,
    output                                  s_axi_bvalid  ,
    input                                   s_axi_bready  ,
     // Slave Interface Read Address Ports 
    input     [AXI_ID_WIDTH - 1:0]          s_axi_arid    ,
    input     [AXI_ADDR_WIDTH - 1:0]        s_axi_araddr  ,
    input     [7:0]                         s_axi_arlen   ,
    input     [2:0]                         s_axi_arsize  ,
    input     [1:0]                         s_axi_arburst ,
    input     [0:0]                         s_axi_arlock  ,
    input     [3:0]                         s_axi_arcache ,
    input     [2:0]                         s_axi_arprot  ,
    input     [3:0]                         s_axi_arqos   ,
    input                                   s_axi_arvalid ,
    output                                  s_axi_arready ,
     // Slave Interface Read Data Ports
    output    [AXI_ID_WIDTH - 1:0]          s_axi_rid     ,
    output    [AXI_DATA_WIDTH - 1:0]        s_axi_rdata   ,
    output    [1:0]                         s_axi_rresp   ,
    output                                  s_axi_rlast   ,
    output                                  s_axi_rvalid  ,
    input                                   s_axi_rready  
);

// noc2axi signals
wire                                buffer_empty    ;
wire                                rd_data_en      ;
wire [DATA_WIDTH - 1: 0]            rd_data         ;

wire [2:0]                          axi_type        ; 
wire [ID_WIDTH - 1 : 0]             dest_id         ; 
wire [VIRTUAL_CH_NUM - 1 : 0]       rd_en           ; 
wire [DATA_WIDTH - 1 : 0]           data_out        ; 
wire [VIRTUAL_CH_NUM - 1 : 0]       empty_vc        ; 
wire [VIRTUAL_CH_NUM - 1 : 0]       pack_num        ; 
wire                                nocpack_done    ; 

// axi2noc signals
wire [DATA_WIDTH : 0]               noc_outdata     ;
wire                                noc_buf_full    ;
wire                                noc_buf_empty   ;

wire                                head            ;
wire                                tail            ;
wire                                rd_head         ;
wire                                rd_tail         ;

// - rate matching and asynchronous data boundary crossing -s
asyn_data_cross #(
    .DATA_WIDTH      (DATA_WIDTH       ),
    .FIFO_ADDR_WIDTH (FIFO_ADDR_WIDTH  ),
    .ID_WIDTH        (ID_WIDTH         ),
    .VIRTUAL_CH_NUM  ( VIRTUAL_CH_NUM  )
) u_asyn_data_cross(
    .noc_clk          ( noc_clk          ),
    .noc_rst_n        ( noc_rst_n        ),
    .axi_clk          ( axi_clk          ),
    .axi_rst_n        ( axi_rst_n        ),
    // AXI 2 NOC
    .head             ( head             ),// INPUT
    .tail             ( tail             ),// INPUT
    .axi2nocdata      ( noc_outdata      ),// INPUT
    .nocdata          ( nocdata          ),
    .m_is_head        ( m_is_head        ),
    .m_is_tail        ( m_is_tail        ),
    .noc_buf_full     ( noc_buf_full     ),
    .noc_buf_empty    ( noc_buf_empty    ),

    //NOC 2 AXI
    .indata           ( noc2axi_data     ),// INPUT
    .s_is_head        ( s_is_head        ),// INPUT
    .s_is_tail        ( s_is_tail        ),// INPUT
    .buffer_full      ( buffer_busy      ),
    .buffer_empty     ( buffer_empty     ),
    .rd_data_en       ( rd_data_en       ),
    .rd_data          ( rd_data          ),
    .rd_head          ( rd_head          ),
    .rd_tail          ( rd_tail          ) 
);

// - read reorder buffer -
rd_reorder_buff #(
    .DATA_WIDTH       ( DATA_WIDTH      ),
    .AXI_ADDR_WIDTH   ( AXI_ADDR_WIDTH  ),
    .FLIT_NUM_MAX     ( FLIT_NUM_MAX    ),
    .ID_WIDTH         ( ID_WIDTH        ),
    .VIRTUAL_CH_NUM   ( VIRTUAL_CH_NUM  )
) u_rd_reorder_buff(
    .axi_clk          ( axi_clk        ),
    .axi_rst_n        ( axi_rst_n      ),
    .buffer_empty     ( buffer_empty   ),
    .rd_data_en       ( rd_data_en     ),    
    .rd_data          ( rd_data        ),
    .rd_head          ( rd_head        ),
    .rd_tail          ( rd_tail        ), 
    
    .axi_type         ( axi_type       ),
    .dest_id          ( dest_id        ),
    .rd_en            ( rd_en          ),
    .data_out         ( data_out       ),
    .empty_vc         ( empty_vc       ),
    .pack_num         ( pack_num       ),
    .nocpack_done     ( nocpack_done   )
);

// - Packetizing and Depacketizing include Address Map -
depacket_proc   #(
    .DATA_WIDTH      ( DATA_WIDTH      ),
    .FIFO_ADDR_WIDTH ( FIFO_ADDR_WIDTH ),
    .ID_WIDTH        ( ID_WIDTH        ),
    .VIRTUAL_CH_NUM  ( VIRTUAL_CH_NUM  ),
    .AXI_ID_WIDTH    ( AXI_ID_WIDTH    )
) u_depacket_proc(
    .axi_clk         ( axi_clk         ),
    .axi_rst_n       ( axi_rst_n       ),
    .axi_type        ( axi_type        ),
    .dest_id         ( dest_id         ),
    .rd_en           ( rd_en           ),
    .data_in         ( data_out        ),
    .empty_vc        ( empty_vc        ),
    .pack_num        ( pack_num        ),
    .nocpack_done    ( nocpack_done    ),
    .s_axi_arlen     ( s_axi_arlen     ),
    .s_axi_arvalid   ( s_axi_arvalid   ),
    .s_axi_arready   ( s_axi_arready   ),

    .s_axi_bid       ( s_axi_bid       ),
    .s_axi_bresp     ( s_axi_bresp     ),
    .s_axi_bvalid    ( s_axi_bvalid    ),
    .s_axi_bready    ( s_axi_bready    ),
    .s_axi_rid       ( s_axi_rid       ),
    .s_axi_rdata     ( s_axi_rdata     ),
    .s_axi_rresp     ( s_axi_rresp     ),
    .s_axi_rlast     ( s_axi_rlast     ),
    .s_axi_rvalid    ( s_axi_rvalid    ),
    .s_axi_rready    ( s_axi_rready    )
);

packet_proc #(
    .DATA_WIDTH     ( DATA_WIDTH       ),
    .ID_WIDTH       ( ID_WIDTH         ),
    .VIRTUAL_CH_NUM ( VIRTUAL_CH_NUM   ),
    .AXI_ID_WIDTH   ( AXI_ID_WIDTH     ),
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH   ),
    .FLIT_NUM_MAX   ( FLIT_NUM_MAX     ),
    .Source_ID      ( Source_ID        )
) u_packet_proc(
    .axi_clk        ( axi_clk        ),
    .axi_rst_n      ( axi_rst_n      ),
    
    .s_axi_bresp    ( s_axi_bresp    ),
    .s_axi_bvalid   ( s_axi_bvalid   ),
    .s_axi_bready   ( s_axi_bready   ),
    
    .noc_outdata    ( noc_outdata    ), //todo
    .m_is_head      ( head           ),
    .m_is_tail      ( tail           ),
     
    .s_axi_awid     ( s_axi_awid     ),
    .s_axi_awaddr   ( s_axi_awaddr   ),
    .s_axi_awlen    ( s_axi_awlen    ),
    .s_axi_awsize   ( s_axi_awsize   ),
    .s_axi_awburst  ( s_axi_awburst  ),
    .s_axi_awlock   ( s_axi_awlock   ),
    .s_axi_awcache  ( s_axi_awcache  ),
    .s_axi_awprot   ( s_axi_awprot   ),
    .s_axi_awqos    ( s_axi_awqos    ),
    .s_axi_awvalid  ( s_axi_awvalid  ),
    .s_axi_awready  ( s_axi_awready  ),
    .s_axi_wdata    ( s_axi_wdata    ),
    .s_axi_wstrb    ( s_axi_wstrb    ),
    .s_axi_wlast    ( s_axi_wlast    ),
    .s_axi_wvalid   ( s_axi_wvalid   ),
    .s_axi_wready   ( s_axi_wready   ),
    .s_axi_arid     ( s_axi_arid     ),
    .s_axi_araddr   ( s_axi_araddr   ),
    .s_axi_arlen    ( s_axi_arlen    ),
    .s_axi_arsize   ( s_axi_arsize   ),
    .s_axi_arburst  ( s_axi_arburst  ),
    .s_axi_arlock   ( s_axi_arlock   ),
    .s_axi_arcache  ( s_axi_arcache  ),
    .s_axi_arprot   ( s_axi_arprot   ),
    .s_axi_arqos    ( s_axi_arqos    ),
    .s_axi_arvalid  ( s_axi_arvalid  ),
    .s_axi_arready  ( s_axi_arready  )
);

endmodule //noc2axi4_master
