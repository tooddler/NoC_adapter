`timescale 1ns / 1ps

module nsu_tb_top #(
    parameter   DATA_WIDTH          =   128                ,
    parameter   AXI_ID_WIDTH        =   4                  ,
    parameter   AXI_ADDR_WIDTH      =   32                 ,
    parameter   AXI_DATA_WIDTH      =   DATA_WIDTH         
)(
    input   wire                       clkin_50m           ,

    inout   [15:0]                     ddr3_dq             ,
    inout   [1:0]                      ddr3_dqs_n          ,
    inout   [1:0]                      ddr3_dqs_p          ,
    // Outputs
    output  [13:0]                     ddr3_addr           ,
    output  [2:0]                      ddr3_ba             ,
    output                             ddr3_ras_n          ,
    output                             ddr3_cas_n          ,
    output                             ddr3_we_n           ,
    output                             ddr3_reset_n        ,
    output  [0:0]                      ddr3_ck_p           ,
    output  [0:0]                      ddr3_ck_n           ,
    output  [0:0]                      ddr3_cke            ,
    output  [0:0]                      ddr3_cs_n           ,
    output  [1:0]                      ddr3_dm             ,
    output  [0:0]                      ddr3_odt            ,
    output                             init_calib_complete 
);

wire     [AXI_ID_WIDTH - 1:0]         m_axi_awid    ;
wire     [AXI_ADDR_WIDTH - 1:0]       m_axi_awaddr  ;
wire     [7:0]                        m_axi_awlen   ;
wire     [2:0]                        m_axi_awsize  ;
wire     [1:0]                        m_axi_awburst ;
wire     [0:0]                        m_axi_awlock  ;
wire     [3:0]                        m_axi_awcache ;
wire     [2:0]                        m_axi_awprot  ;
wire     [3:0]                        m_axi_awqos   ;
wire                                  m_axi_awvalid ;
wire                                  m_axi_awready ;

wire     [AXI_DATA_WIDTH - 1:0]       m_axi_wdata   ;
wire     [AXI_DATA_WIDTH/8 - 1:0]     m_axi_wstrb   ;
wire                                  m_axi_wlast   ;
wire                                  m_axi_wvalid  ;
wire                                  m_axi_wready  ;

wire     [AXI_ID_WIDTH - 1:0]         m_axi_bid     ;
wire     [1:0]                        m_axi_bresp   ;
wire                                  m_axi_bvalid  ;
wire                                  m_axi_bready  ;

wire    [AXI_ID_WIDTH - 1:0]          m_axi_arid    ;
wire    [AXI_ADDR_WIDTH - 1:0]        m_axi_araddr  ;
wire    [7:0]                         m_axi_arlen   ;
wire    [2:0]                         m_axi_arsize  ;
wire    [1:0]                         m_axi_arburst ;
wire    [0:0]                         m_axi_arlock  ;
wire    [3:0]                         m_axi_arcache ;
wire    [2:0]                         m_axi_arprot  ;
wire    [3:0]                         m_axi_arqos   ;
wire                                  m_axi_arvalid ;
wire                                  m_axi_arready ;

wire    [AXI_ID_WIDTH - 1:0]          m_axi_rid     ;
wire    [AXI_DATA_WIDTH - 1:0]        m_axi_rdata   ;
wire    [1:0]                         m_axi_rresp   ;
wire                                  m_axi_rlast   ;
wire                                  m_axi_rvalid  ;
wire                                  m_axi_rready  ;

wire    [DATA_WIDTH:0]                nocdata       ;
wire                                  m_is_head     ;
wire                                  m_is_tail     ;

wire    [DATA_WIDTH:0]                noc2axi_data  ; 

// ---------------------------- combinational logic -------------------------
assign init_calib_complete = ddr_init_done;

// ---------------------------- sequential logic ----------------------------

// ----------------------------  instantiation  ----------------------------
clock_and_reset clock_and_reset(
    .clkin_50m   (clkin_50m     ),

    .clkout_50m  (axi_clk       ),
    .clkout_200m (noc_clk       ),
    .clkout_ddr  (ddr_sys_clk   ),
    .reset       (reset         )
);

noc_gen #(
    .DATA_WIDTH     ( 128   ),
    .ID_WIDTH       ( 4     ),
    .VIRTUAL_CH_NUM ( 16    ),
    .AXI_ADDR_WIDTH ( 32    )
) u_noc_gen(
    .noc_clk        ( noc_clk        ),
    .noc_rst        ( reset          ),
    .noc2axi_data   ( noc2axi_data   ),
    .s_is_head      ( s_is_head      ),
    .s_is_tail      ( s_is_tail      ),
    .nsu_busy       ( nsu_busy       ),
    .ddr_init_done  ( ddr_init_done  )
);


noc2axi4_slave #(
    .DATA_WIDTH      ( 128  ),
    .FLIT_NUM_MAX    ( 16   ),
    .ID_WIDTH        ( 4    ),
    .VIRTUAL_CH_NUM  ( 16   ),
    .FIFO_ADDR_WIDTH ( 16   ),
    .AXI_ID_WIDTH    ( 4    ),
    .AXI_ADDR_WIDTH  ( 32   ),
    .FREQ_DDR_CLK    ( 100  ),
    .FREQ_NOC_CLK    ( 200  )
)u_noc2axi4_slave(
    .noc_clk         ( noc_clk          ), // pll 提供
    .noc_rst_n       ( ~reset           ),
    .ddr_clk         ( ui_clk           ), // mig 提供
    .ddr_rst_n       ( ~ui_clk_sync_rst ),

    .noc2axi_data    ( noc2axi_data     ), // in nsu
    .s_is_head       ( s_is_head        ),
    .s_is_tail       ( s_is_tail        ),
    .nsu_busy        ( nsu_busy         ),

    .nocdata         ( nocdata          ), // out nsu
    .m_is_head       ( m_is_head        ),
    .m_is_tail       ( m_is_tail        ),

    .m_axi_awid      ( m_axi_awid       ),
    .m_axi_awaddr    ( m_axi_awaddr     ),
    .m_axi_awlen     ( m_axi_awlen      ),
    .m_axi_awsize    ( m_axi_awsize     ),
    .m_axi_awburst   ( m_axi_awburst    ),
    .m_axi_awlock    ( m_axi_awlock     ),
    .m_axi_awcache   ( m_axi_awcache    ),
    .m_axi_awprot    ( m_axi_awprot     ),
    .m_axi_awqos     ( m_axi_awqos      ),
    .m_axi_awvalid   ( m_axi_awvalid    ),
    .m_axi_awready   ( m_axi_awready    ),

    .m_axi_wdata     ( m_axi_wdata      ),
    .m_axi_wstrb     ( m_axi_wstrb      ),
    .m_axi_wlast     ( m_axi_wlast      ),
    .m_axi_wvalid    ( m_axi_wvalid     ),
    .m_axi_wready    ( m_axi_wready     ),

    .m_axi_bid       ( m_axi_bid        ),
    .m_axi_bresp     ( m_axi_bresp      ),
    .m_axi_bvalid    ( m_axi_bvalid     ),
    .m_axi_bready    ( m_axi_bready     ),

    .m_axi_arid      ( m_axi_arid       ),
    .m_axi_araddr    ( m_axi_araddr     ),
    .m_axi_arlen     ( m_axi_arlen      ),
    .m_axi_arsize    ( m_axi_arsize     ),
    .m_axi_arburst   ( m_axi_arburst    ),
    .m_axi_arlock    ( m_axi_arlock     ),
    .m_axi_arcache   ( m_axi_arcache    ),
    .m_axi_arprot    ( m_axi_arprot     ),
    .m_axi_arqos     ( m_axi_arqos      ),
    .m_axi_arvalid   ( m_axi_arvalid    ),
    .m_axi_arready   ( m_axi_arready    ),
    
    .m_axi_rid       ( m_axi_rid        ),
    .m_axi_rdata     ( m_axi_rdata      ),
    .m_axi_rresp     ( m_axi_rresp      ),
    .m_axi_rlast     ( m_axi_rlast      ),
    .m_axi_rvalid    ( m_axi_rvalid     ),
    .m_axi_rready    ( m_axi_rready     )
);


mig_7series_0 u_mig_7series_0 (
    // Memory interface ports
    .ddr3_addr                      (ddr3_addr          ),  // output [13:0]        ddr3_addr
    .ddr3_ba                        (ddr3_ba            ),  // output [2:0]         ddr3_ba
    .ddr3_cas_n                     (ddr3_cas_n         ),  // output               ddr3_cas_n
    .ddr3_ck_n                      (ddr3_ck_n          ),  // output [0:0]         ddr3_ck_n
    .ddr3_ck_p                      (ddr3_ck_p          ),  // output [0:0]         ddr3_ck_p
    .ddr3_cke                       (ddr3_cke           ),  // output [0:0]         ddr3_cke
    .ddr3_ras_n                     (ddr3_ras_n         ),  // output               ddr3_ras_n
    .ddr3_reset_n                   (ddr3_reset_n       ),  // output               ddr3_reset_n
    .ddr3_we_n                      (ddr3_we_n          ),  // output               ddr3_we_n
    .ddr3_dq                        (ddr3_dq            ),  // inout [15:0]         ddr3_dq
    .ddr3_dqs_n                     (ddr3_dqs_n         ),  // inout [1:0]          ddr3_dqs_n
    .ddr3_dqs_p                     (ddr3_dqs_p         ),  // inout [1:0]          ddr3_dqs_p
    .ddr3_cs_n                      (ddr3_cs_n          ),  // output [0:0]         ddr3_cs_n
    .ddr3_dm                        (ddr3_dm            ),  // output [1:0]         ddr3_dm
    .ddr3_odt                       (ddr3_odt           ),  // output [0:0]         ddr3_odt

    .init_calib_complete            (ddr_init_done      ),  // output               init_calib_complete
    
    .ui_clk                         (ui_clk             ),  // output               ui_clk
    .ui_clk_sync_rst                (ui_clk_sync_rst    ),  // output               ui_clk_sync_rst

    .mmcm_locked                    (),  // output          mmcm_locked

    .app_sr_req                     (0),  // input          app_sr_req
    .app_ref_req                    (0),  // input          app_ref_req
    .app_zq_req                     (0),  // input          app_zq_req
    .app_sr_active                  (),   // output         app_sr_active
    .app_ref_ack                    (),   // output         app_ref_ack
    .app_zq_ack                     (),   // output         app_zq_ack

    // Slave Interface Write Address Ports
    .aresetn                        (~ui_clk_sync_rst   ),  // input aresetn  ui_clk_sync_rst复位翻转得到

    .s_axi_awid                     (m_axi_awid         ),  // input [3:0]	    s_axi_awid
    .s_axi_awaddr                   (m_axi_awaddr       ),  // input [27:0]	    s_axi_awaddr
    .s_axi_awlen                    (m_axi_awlen        ),  // input [7:0]	    s_axi_awlen
    .s_axi_awsize                   (m_axi_awsize       ),  // input [2:0]	    s_axi_awsize
    .s_axi_awburst                  (m_axi_awburst      ),  // input [1:0]	    s_axi_awburst
    .s_axi_awlock                   (m_axi_awlock       ),  // input [0:0]	    s_axi_awlock
    .s_axi_awcache                  (m_axi_awcache      ),  // input [3:0]	    s_axi_awcache
    .s_axi_awprot                   (m_axi_awprot       ),  // input [2:0]	    s_axi_awprot
    .s_axi_awqos                    (m_axi_awqos        ),  // input [3:0]	    s_axi_awqos
    .s_axi_awvalid                  (m_axi_awvalid      ),  // input		    s_axi_awvalid
    .s_axi_awready                  (m_axi_awready      ),  // output		    s_axi_awready

    // Slave Interface Write Data Ports
    .s_axi_wdata                    (m_axi_wdata        ),  // input [127:0]	s_axi_wdata
    .s_axi_wstrb                    (m_axi_wstrb        ),  // input [15:0]		s_axi_wstrb
    .s_axi_wlast                    (m_axi_wlast        ),  // input			s_axi_wlast
    .s_axi_wvalid                   (m_axi_wvalid       ),  // input			s_axi_wvalid
    .s_axi_wready                   (m_axi_wready       ),  // output			s_axi_wready

    // Slave Interface Write Response Ports
    .s_axi_bid                      (m_axi_bid          ),  // output [3:0]     s_axi_bid
    .s_axi_bresp                    (m_axi_bresp        ),  // output [1:0]     s_axi_bresp
    .s_axi_bvalid                   (m_axi_bvalid       ),  // output           s_axi_bvalid
    .s_axi_bready                   (m_axi_bready       ),  // input            s_axi_bready
    // Slave Interface Read Address Ports
    .s_axi_arid                     (m_axi_arid         ),  // input [3:0]      s_axi_arid
    .s_axi_araddr                   (m_axi_araddr       ),  // input [27:0]     s_axi_araddr
    .s_axi_arlen                    (m_axi_arlen        ),  // input [7:0]      s_axi_arlen
    .s_axi_arsize                   (m_axi_arsize       ),  // input [2:0]      s_axi_arsize
    .s_axi_arburst                  (m_axi_arburst      ),  // input [1:0]      s_axi_arburst
    .s_axi_arlock                   (m_axi_arlock       ),  // input [0:0]      s_axi_arlock
    .s_axi_arcache                  (m_axi_arcache      ),  // input [3:0]      s_axi_arcache
    .s_axi_arprot                   (m_axi_arprot       ),  // input [2:0]      s_axi_arprot
    .s_axi_arqos                    (m_axi_arqos        ),  // input [3:0]      s_axi_arqos
    .s_axi_arvalid                  (m_axi_arvalid      ),  // input            s_axi_arvalid
    .s_axi_arready                  (m_axi_arready      ),  // output           s_axi_arready
    // Slave Interface Read Data Ports
    .s_axi_rid                      (m_axi_rid          ),  // output [3:0]     s_axi_rid
    .s_axi_rdata                    (m_axi_rdata        ),  // output [127:0]   s_axi_rdata
    .s_axi_rresp                    (m_axi_rresp        ),  // output [1:0]     s_axi_rresp
    .s_axi_rlast                    (m_axi_rlast        ),  // output           s_axi_rlast
    .s_axi_rvalid                   (m_axi_rvalid       ),  // output           s_axi_rvalid
    .s_axi_rready                   (m_axi_rready       ),  // input            s_axi_rready

    // System Clock Port
    .sys_clk_i                      (ddr_sys_clk        ),
    .sys_rst                        (~reset             )   // input sys_rst    复位信号低有效
);


endmodule
