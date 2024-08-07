/*
    NMU问题： ① 收端 若下一包数据到来较快，怎么做流量控制？
              ② 需要额外的数据线区分 data 和 head/tail
*/
`timescale 1ns / 1ps
`define CLK_PERIOD_50M  20
`define CLK_PERIOD_100M 10

module axi2noc_tb();
// noc domain clk
reg                     noc_clk     ;
reg                     noc_rst_n   ;
// user domain clk
reg                     axi_clk     ;
reg                     axi_rst_n   ;

reg  [31:0]             awaddr      ;
reg  [7:0]              awlen       ;

reg  [128:0]            noc2axi_data;
reg                     s_axi_bready;
//reg                     s_axi_rready;

reg  [127:0]            wdata       ;
reg                     wdata_last  ;
reg                     wdata_vld   ;

reg                     awvalid     ;
reg  [3:0]              cnt         ;

wire                    buffer_busy  ;
wire                    s_axi_awready;

wire [3:0]              s_axi_bid   ;
wire [1:0]              s_axi_bresp ;
wire                    s_axi_bvalid;

wire                    s_axi_wready;
wire [128:0]            nocdata     ;

wire                    m_is_head   ;
wire                    m_is_tail   ;
//wire                    s_axi_arready;
initial noc_clk = 1'b1;
always #(`CLK_PERIOD_100M/2) noc_clk = ~noc_clk;

initial axi_clk = 1'b1;
always #(`CLK_PERIOD_50M/2) axi_clk = ~axi_clk;


initial begin
s_axi_bready = 1'b1 ;
noc_rst_n    = 1'b0 ;
axi_rst_n    = 1'b0 ;
noc2axi_data = 'd0  ;
awaddr       = 'd0  ;
awlen        = 'd0  ;
wdata        = 'd0  ;
wdata_last   = 'd0  ;
wdata_vld    = 'd0  ;
awvalid      = 1'b0 ;
# 201;
noc_rst_n = 1'b1;
axi_rst_n = 1'b1;
# 200;
// // ------ write data ------
// awaddr       = 32'h0000_2aa0 ;
// awlen        = 8'b0111_1111  ; // 128
// //awlen        = 8'b0011_1100  ; // 60
// awvalid      = 1'b1 ;
// #(`CLK_PERIOD_50M); 
// cnt = 'd0;
// #(`CLK_PERIOD_50M); 
// #(`CLK_PERIOD_50M); 
// #(`CLK_PERIOD_50M); 
// repeat(128) begin
//     wdata     = wdata + 'd1;
// //    wdata = $random;
//     wdata_vld = 1'b1;
//     cnt = cnt + 1'b1;
//     if (cnt == 128) wdata_last=1'b1;
//     #(`CLK_PERIOD_50M); 
//     wdata_last=1'b0; 
// //    wdata     = 'd0;
//     wdata_vld = 'd0;
//     #(`CLK_PERIOD_50M); 
// end
// // ------ end write data ------
// ------ read req ------
awaddr       = 32'h0000_2aa0 ;
awlen        = 8'b0111_1111  ; // 128
//awlen        = 8'b0011_1100  ; // 60
awvalid      = 1'b1 ;
// ------ end read req ------
# 2000;
awvalid      = 1'b0 ;
// ------ bresp ------
noc2axi_data = {1'b1,4'h5,4'h1,4'h0,3'b011,16'h01,   8'b0,4'hA,  82'b0,3'b111}; // head
#(`CLK_PERIOD_100M); 
noc2axi_data = {1'b1,4'h5,4'h0,4'h1,3'b011,8'h01,8'b0,4'hA,90'b0,3'b100}; // data
#(`CLK_PERIOD_100M); 
noc2axi_data = {1'b1,4'h0,4'h1,4'h0,3'b011,16'h01,   8'b0,4'hF,  82'b0,3'b111}; // tail
#(`CLK_PERIOD_100M); 
// ------ end bresp ------
# 4000;
$stop;
end

// --- use for write data ---
// noc2axi4_master #(
//     .VIRTUAL_CH_NUM  ( 16 ),
//     .DATA_WIDTH      ( 128 ),
//     .ID_WIDTH        ( 4 ),
//     .BUFFER_DEPTH    ( 8 ),
//     .FIFO_ADDR_WIDTH ( 16 ),
//     .AXI_ID_WIDTH    ( 4 ),
//     .AXI_ADDR_WIDTH  ( 32 )
// ) u_noc2axi4_master(
//     .noc_clk         ( noc_clk         ),
//     .noc_rst_n       ( noc_rst_n       ),
//     .axi_clk         ( axi_clk         ),
//     .axi_rst_n       ( axi_rst_n       ),
//     .noc2axi_data    ( noc2axi_data    ),
//     .buffer_busy     ( buffer_busy     ),
//     .nocdata         ( nocdata         ),
//     .m_is_head       ( m_is_head       ),
//     .m_is_tail       ( m_is_tail       ), 
//     .s_axi_awid      ( 4'b0000         ),
//     .s_axi_awaddr    ( awaddr          ),
//     .s_axi_awlen     ( awlen           ),
//     .s_axi_awsize    (3'b100           ),
//     .s_axi_awburst   (2'b01            ),
//     .s_axi_awlock    (1'b0             ),
//     .s_axi_awcache   (4'b0000          ),
//     .s_axi_awprot    (3'b000           ),
//     .s_axi_awqos     (4'b0000          ),
//     .s_axi_awvalid   (awvalid          ),
//     .s_axi_awready   (s_axi_awready    ),
    
//     .s_axi_wdata     (wdata            ),
//     .s_axi_wstrb     (16'hffff         ),
//     .s_axi_wlast     (wdata_last       ),
//     .s_axi_wvalid    (wdata_vld        ),
//     .s_axi_wready    (s_axi_wready     ),

//     .s_axi_bid       ( s_axi_bid       ),
//     .s_axi_bresp     ( s_axi_bresp     ),
//     .s_axi_bvalid    ( s_axi_bvalid    ),
//     .s_axi_bready    ( s_axi_bready    ),

//     .s_axi_arid      (                 ),
//     .s_axi_araddr    (                 ),
//     .s_axi_arlen     (                 ),
//     .s_axi_arsize    (                 ),
//     .s_axi_arburst   (                 ),
//     .s_axi_arlock    (                 ),
//     .s_axi_arcache   (                 ),
//     .s_axi_arprot    (                 ),
//     .s_axi_arqos     (                 ),
//     .s_axi_arvalid   (                 ),
//     .s_axi_arready   (                 ),

//     .s_axi_rid       (    ),
//     .s_axi_rdata     (    ),
//     .s_axi_rresp     (    ),
//     .s_axi_rlast     (    ),
//     .s_axi_rvalid    (    ),
//     .s_axi_rready    (    )
// );

// --- use for read req ---
noc2axi4_master #(
    .VIRTUAL_CH_NUM  ( 16 ),
    .DATA_WIDTH      ( 128 ),
    .ID_WIDTH        ( 4 ),
    .BUFFER_DEPTH    ( 8 ),
    .FIFO_ADDR_WIDTH ( 16 ),
    .AXI_ID_WIDTH    ( 4 ),
    .AXI_ADDR_WIDTH  ( 32 )
) u_noc2axi4_master(
    .noc_clk         ( noc_clk         ),
    .noc_rst_n       ( noc_rst_n       ),
    .axi_clk         ( axi_clk         ),
    .axi_rst_n       ( axi_rst_n       ),
    .noc2axi_data    ( noc2axi_data    ),
    .buffer_busy     ( buffer_busy     ),
    .nocdata         ( nocdata         ),
    .m_is_head       ( m_is_head       ),
    .m_is_tail       ( m_is_tail       ), 
    .s_axi_awid      (),
    .s_axi_awaddr    (),
    .s_axi_awlen     (),
    .s_axi_awsize    (),
    .s_axi_awburst   (),
    .s_axi_awlock    (),
    .s_axi_awcache   (),
    .s_axi_awprot    (),
    .s_axi_awqos     (),
    .s_axi_awvalid   (),
    .s_axi_awready   (),
    
    .s_axi_wdata     (wdata            ),
    .s_axi_wstrb     (16'hffff         ),
    .s_axi_wlast     (wdata_last       ),
    .s_axi_wvalid    (wdata_vld        ),
    .s_axi_wready    (s_axi_wready     ),

    .s_axi_bid       ( s_axi_bid       ),
    .s_axi_bresp     ( s_axi_bresp     ),
    .s_axi_bvalid    ( s_axi_bvalid    ),
    .s_axi_bready    ( s_axi_bready    ),

    .s_axi_arid      ( 4'b0000         ),
    .s_axi_araddr    ( awaddr          ),
    .s_axi_arlen     ( awlen           ),
    .s_axi_arsize    (3'b100           ),
    .s_axi_arburst   (2'b01            ),
    .s_axi_arlock    (1'b0             ),
    .s_axi_arcache   (4'b0000          ),
    .s_axi_arprot    (3'b000           ),
    .s_axi_arqos     (4'b0000          ),
    .s_axi_arvalid   (awvalid          ),
    .s_axi_arready   (s_axi_awready    ),

    .s_axi_rid       (    ),
    .s_axi_rdata     (    ),
    .s_axi_rresp     (    ),
    .s_axi_rlast     (    ),
    .s_axi_rvalid    (    ),
    .s_axi_rready    (    )
);
endmodule
