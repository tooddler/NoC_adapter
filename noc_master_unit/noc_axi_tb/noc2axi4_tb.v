// 问题： ① 收端 若下一包数据到来较快，怎么做流量控制？

`timescale 1ns / 1ps
`define CLK_PERIOD_50M  20
`define CLK_PERIOD_100M 10

module noc2axi4_tb();
// noc domain clk
reg                     noc_clk     ;
reg                     noc_rst_n   ;
// user domain clk
reg                     axi_clk     ;
reg                     axi_rst_n   ;

reg  [128:0]            noc2axi_data;
reg                     s_axi_bready;
reg                     s_axi_rready;
reg                     head        ;
reg                     tail        ;
reg  [127:0]            data        ;

wire                    buffer_busy;
//wire                    s_axi_awready;

wire [3:0]              s_axi_bid   ;
wire [1:0]              s_axi_bresp ;
wire                    s_axi_bvalid;

wire [3:0]              s_axi_rid   ;
wire [127:0]            s_axi_rdata ;
wire [1:0]              s_axi_rresp ;
wire                    s_axi_rlast ;
wire                    s_axi_rvalid;

//wire                    s_axi_wready;
//wire                    s_axi_arready;
initial noc_clk = 1'b1;
always #(`CLK_PERIOD_100M/2) noc_clk = ~noc_clk;

initial axi_clk = 1'b1;
always #(`CLK_PERIOD_50M/2) axi_clk = ~axi_clk;


initial begin
s_axi_bready = 1'b1;
s_axi_rready = 1'b1;
noc_rst_n    = 1'b0;
axi_rst_n    = 1'b0;
noc2axi_data = 'd0 ;
head         = 1'b0;
tail         = 1'b0;
data         = 'd0;
# 201;
noc_rst_n = 1'b1;
axi_rst_n = 1'b1;
# 200;
// ------ bresp ------
// 位置 1 到 0
noc2axi_data = {1'b1,4'h5,4'h1,4'h0,3'b011,16'h0001,   40'h04,4'hA,50'b0,3'b111}; // head
head         = 1'b1;
#(`CLK_PERIOD_100M);

head         = 1'b0;
noc2axi_data = {1'b1, data}; // data
#(`CLK_PERIOD_100M); 

tail         = 1'b1;
noc2axi_data = {1'b1,4'h0,4'h1,4'h0,3'b011,16'h0002,   40'h04,4'hF,50'b0,3'b111}; // tail
#(`CLK_PERIOD_100M); 
tail         = 1'b0;

// error
data = data + 'd1;

// 位置 3 到 0
noc2axi_data = {1'b1,4'h5,4'h3,4'h0,3'b011,16'h0002,   40'h04,4'hA,50'b0,3'b111}; // head
head         = 1'b1;
#(`CLK_PERIOD_100M);
head         = 1'b0;

noc2axi_data = {1'b1, data}; // data
#(`CLK_PERIOD_100M); 

tail         = 1'b1;
noc2axi_data = {1'b1,4'h0,4'h3,4'h0,3'b011,16'h0002,   40'h04,4'hF,50'b0,3'b111}; // tail
#(`CLK_PERIOD_100M);
tail         = 1'b0;
// ------ end bresp ------

noc2axi_data = 'd0 ;
# 1000
$stop;
end

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
    .s_is_head       ( head            ),
    .s_is_tail       ( tail            ),  
    .buffer_busy     ( buffer_busy     ),

    .s_axi_awid      (                 ),
    .s_axi_awaddr    (                 ),
    .s_axi_awlen     (                 ),
    .s_axi_awsize    (                 ),
    .s_axi_awburst   (                 ),
    .s_axi_awlock    (                 ),
    .s_axi_awcache   (                 ),
    .s_axi_awprot    (                 ),
    .s_axi_awqos     (                 ),
    .s_axi_awvalid   (                 ),
    .s_axi_awready   (    ),
    
    .s_axi_wdata     (                 ),
    .s_axi_wstrb     (                 ),
    .s_axi_wlast     (                 ),
    .s_axi_wvalid    (                 ),
    .s_axi_wready    (     ),

    .s_axi_bid       ( s_axi_bid       ),
    .s_axi_bresp     ( s_axi_bresp     ),
    .s_axi_bvalid    ( s_axi_bvalid    ),
    .s_axi_bready    ( s_axi_bready    ),

    .s_axi_arid      (                 ),
    .s_axi_araddr    (                 ),
    .s_axi_arlen     (                 ),
    .s_axi_arsize    (                 ),
    .s_axi_arburst   (                 ),
    .s_axi_arlock    (                 ),
    .s_axi_arcache   (                 ),
    .s_axi_arprot    (                 ),
    .s_axi_arqos     (                 ),
    .s_axi_arvalid   (                 ),
    .s_axi_arready   (    ),

    .s_axi_rid       ( s_axi_rid       ),
    .s_axi_rdata     ( s_axi_rdata     ),
    .s_axi_rresp     ( s_axi_rresp     ),
    .s_axi_rlast     ( s_axi_rlast     ),
    .s_axi_rvalid    ( s_axi_rvalid    ),
    .s_axi_rready    ( s_axi_rready    )
);


endmodule
