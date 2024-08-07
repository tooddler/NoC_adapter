// Author : ToOddler 
// Email  : 23011211185@stu.xidian.edu.cn
// - rate matching and asynchronous data boundary crossing -
// - gen axi -

module nsu_asyn_cross_noc2axi #(
    parameter   DATA_WIDTH                  =            128            ,
    parameter   FIFO_ADDR_WIDTH             =            16             ,
    parameter   ID_WIDTH                    =            4              ,
    parameter   VIRTUAL_CH_NUM              =            16             ,
    parameter   PACK_ORDER_WIDTH            =            VIRTUAL_CH_NUM ,
    parameter   AXI_ID_WIDTH                =            4              ,
    parameter   AXI_ADDR_WIDTH              =            32             ,
    parameter   AXI_DATA_WIDTH              =            DATA_WIDTH     ,
    parameter   TYPE_WRITE                  =            3'b100         ,
    parameter   TYPE_RD_REQ                 =            3'b010         ,

    parameter   HEAD_CODE_BIT               =            4              ,  // Head flit校验字符长度
    parameter   HEAD_CODE_H                 =            4'hA           ,  // Head flit头部校验字符
    parameter   HEAD_CODE_E                 =            4'hB           ,  // Head flit尾部校验字符
    parameter   TAIL_CODE_BIT               =            4              ,  // Tail flit校验字符长度
    parameter   TAIL_CODE_H                 =            4'hC           ,  // Tail flit头部校验字符
    parameter   TAIL_CODE_E                 =            4'hD           ,   // Tail flit尾部校验字符  
    
    parameter   FREQ_DDR_CLK                =            200            ,
    parameter   FREQ_NOC_CLK                =            100
)(
    // noc domain clk
    input                                           noc_clk             ,
    input                                           noc_rst_n           ,
    // ddr domain clk      
    input                                           ddr_clk             ,
    input                                           ddr_rst_n           ,
    // - noc2ddr interact with noc -      
    input       [DATA_WIDTH - 1 : 0]                data2ddr            , // data channel
    input                                           data2ddr_last       ,
    input                                           en_data2ddr         ,
    output wire                                     data_fifo_full      , 
    input       [2*AXI_ADDR_WIDTH+ID_WIDTH+10:0]    cmd2ddr             , // cmd channel    
    input                                           en_cmd2ddr          ,
    output wire                                     cmd_fifo_full       , 
    // pack_proc
    output reg [AXI_ADDR_WIDTH - 1:0]               re_pack             ,
    // - axi interface -
    output     [AXI_ID_WIDTH - 1:0]                 m_axi_awid          , // TYPE_WRITE
    output     [AXI_ADDR_WIDTH - 1:0]               m_axi_awaddr        ,
    output     [7:0]                                m_axi_awlen         ,
    output     [2:0]                                m_axi_awsize        ,
    output     [1:0]                                m_axi_awburst       ,
    output     [0:0]                                m_axi_awlock        ,
    output     [3:0]                                m_axi_awcache       ,
    output     [2:0]                                m_axi_awprot        ,
    output     [3:0]                                m_axi_awqos         ,
    output reg                                      m_axi_awvalid       ,
    input                                           m_axi_awready       ,
     // Master Interface Write Data Port
    output     [AXI_DATA_WIDTH - 1:0]               m_axi_wdata         ,
    output     [AXI_DATA_WIDTH/8 - 1:0]             m_axi_wstrb         ,
    output wire                                     m_axi_wlast         ,
    output reg                                      m_axi_wvalid        ,
    input                                           m_axi_wready        ,
     // Master Interface Read Address Ports 
    output     [AXI_ID_WIDTH - 1:0]                 m_axi_arid          ,  // TYPE_RD_REQ
    output     [AXI_ADDR_WIDTH - 1:0]               m_axi_araddr        ,
    output     [7:0]                                m_axi_arlen         ,
    output     [2:0]                                m_axi_arsize        ,
    output     [1:0]                                m_axi_arburst       ,
    output     [0:0]                                m_axi_arlock        ,
    output     [3:0]                                m_axi_arcache       ,
    output     [2:0]                                m_axi_arprot        ,
    output     [3:0]                                m_axi_arqos         ,
    output reg                                      m_axi_arvalid       ,
    input                                           m_axi_arready       ,
     // - vld&rdy cdc : to generate nsu_busy signal - 
    input                                           m_axi_bvalid        , // TYPE_WRITE
    input                                           m_axi_bready        ,
    input                                           m_axi_rlast         , // TYPE_RD_REQ
    input                                           m_axi_rvalid        ,
    input                                           m_axi_rready        ,

    input                                           TypeRdReqFinish     ,
    output wire                                     type_wr_done        ,
    output wire                                     type_rd_req_done
);

// ----- localparam -----
localparam  S_IDLE      = 4'b0001,
            READ_CMD    = 4'b0010,
            SEND_ADDR   = 4'b0100,
            SEND_DATA   = 4'b1000;

// ----- wire -----
wire                                    cmd_fifo_empty          ;
wire                                    data_fifo_empty         ;
wire [2*AXI_ADDR_WIDTH+ID_WIDTH+10:0]   cmd_fifo_dout           ;   // {axi_addr, axi_len, axi_type, source_id, re_pack}

wire [DATA_WIDTH : 0]                   data_fifo_dout          ;
wire                                    data_fifo_rd_en         ;
wire                                    type_wr_vld_rdy         ;
wire                                    type_rd_req_vld_rdy     ;

// ----- reg -----
reg  [3:0]                              curr_state              ;
reg  [3:0]                              next_state              ;

reg                                     cmd_fifo_rd_en          ;

reg  [AXI_ADDR_WIDTH - 1 : 0]           axi_addr                ;
reg  [7:0]                              axi_len                 ;
reg  [2:0]                              axi_type                ;
reg  [ID_WIDTH - 1 : 0]                 source_id               ;

reg                                     type_wr_vld_rdy_r0      ;
reg                                     type_rd_req_vld_rdy_r0  ;

reg                                     type_wr_vld_rdy_r1      ;
reg                                     type_rd_req_vld_rdy_r1  ;

reg                                     read_cmd_fifo_dly       ;

// ---------------------------- combinational logic -------------------------
assign data_fifo_rd_en              =    m_axi_wvalid & m_axi_wready                   ;

// - TYPE_WRITE -
assign m_axi_awaddr                 =    axi_addr                                      ;
assign m_axi_awid                   =    source_id                                     ;
assign m_axi_awlen                  =    axi_len                                       ;

assign m_axi_awburst                =    2'b01                                         ;
assign m_axi_awlock                 =    1'b0                                          ;
assign m_axi_awcache                =    4'b0000                                       ;
assign m_axi_awprot                 =    3'b000                                        ;
assign m_axi_awqos                  =    4'b0000                                       ;  

assign m_axi_wstrb                  =    16'hffff                                      ;
// assign {m_axi_wlast, m_axi_wdata}   =    data_fifo_dout                                ;
assign m_axi_wlast                  =    data_fifo_dout[DATA_WIDTH]                    ;
assign m_axi_wdata                  =    data_fifo_dout[DATA_WIDTH - 1 : 0]            ;

// - TYPE_RD_REQ -
assign m_axi_araddr                 =    axi_addr                                      ;
assign m_axi_arid                   =    source_id                                     ;
assign m_axi_arlen                  =    axi_len                                       ;
    
assign m_axi_arburst                =    2'b01                                         ;
assign m_axi_arlock                 =    1'b0                                          ;
assign m_axi_arcache                =    4'b0000                                       ;
assign m_axi_arprot                 =    3'b000                                        ;
assign m_axi_arqos                  =    4'b0000                                       ;

// - gen nsu_busy -
assign type_wr_vld_rdy              =    m_axi_bvalid & m_axi_bready                   ;
assign type_rd_req_vld_rdy          =    TypeRdReqFinish                               ;
// assign type_rd_req_vld_rdy          =    m_axi_rlast && m_axi_rvalid && m_axi_rready   ;

assign m_axi_awsize = 3'b011; // TODO: 主要查看slave端的数据位宽
assign m_axi_arsize = 3'b011;

// m_axi_wvalid
always@(*) begin
    if(ddr_rst_n == 1'b0)
        m_axi_wvalid <= 1'b0;
    else if (curr_state == SEND_DATA && ~data_fifo_empty)
        m_axi_wvalid <= 1'b1;
    else
        m_axi_wvalid <= 1'b0;
end
// ---------------------------- sequential logic ----------------------------
//  ====================== noc_clk domain ======================
generate 
if (FREQ_DDR_CLK > FREQ_NOC_CLK) begin : cdc1
    reg                             type_wr_ext           ;
    reg                             type_rd_req_ext       ;
    reg                             type_wr_sync2_dff1    ;
    reg                             type_rd_req_sync2_dff1;
    //脉冲同步器
    // step1: 将脉冲作为 ddr_clk domain 中一个反转寄存器的使能端
    always@(posedge ddr_clk or negedge ddr_rst_n) begin
        if(ddr_rst_n == 1'b0) begin
            type_wr_ext     <= 1'b0;
            type_rd_req_ext <= 1'b0;
        end
        else if (type_wr_vld_rdy) begin
            type_wr_ext     <= ~type_wr_ext    ;
            type_rd_req_ext <= type_rd_req_ext ;
        end
        else if (type_rd_req_vld_rdy) begin
            type_wr_ext     <= type_wr_ext      ;
            type_rd_req_ext <= ~type_rd_req_ext ;            
        end
    end
    // step2:打两拍
    always@(posedge noc_clk or negedge noc_rst_n) begin
        if (noc_rst_n == 1'b0) begin
            type_wr_vld_rdy_r0     <= 1'b0;
            type_wr_vld_rdy_r1     <= 1'b0;
            type_rd_req_vld_rdy_r0 <= 1'b0;
            type_rd_req_vld_rdy_r1 <= 1'b0;
        end
        else begin
            type_wr_vld_rdy_r0     <= type_wr_ext            ;
            type_wr_vld_rdy_r1     <= type_wr_vld_rdy_r0     ;
            type_rd_req_vld_rdy_r0 <= type_rd_req_ext        ;
            type_rd_req_vld_rdy_r1 <= type_rd_req_vld_rdy_r0 ;        
        end
    end
    // step3:产生输出脉冲
    always @(posedge noc_clk or negedge noc_rst_n) begin
        if(noc_rst_n == 1'b0) begin
            type_wr_sync2_dff1      <= 1'b0;
            type_rd_req_sync2_dff1  <= 1'b0;
        end 
        else begin        
            type_wr_sync2_dff1      <= type_wr_vld_rdy_r1    ;
            type_rd_req_sync2_dff1  <= type_rd_req_vld_rdy_r1;
        end
    end
    assign type_wr_done     = type_wr_vld_rdy_r1 ^ type_wr_sync2_dff1           ;
    assign type_rd_req_done = type_rd_req_vld_rdy_r1 ^ type_rd_req_sync2_dff1   ;
end
else begin : cdc2
    // low clk to fast clk
    always@(posedge noc_clk) begin
        type_wr_vld_rdy_r0     <= type_wr_vld_rdy       ;
        type_wr_vld_rdy_r1     <= type_wr_vld_rdy_r0    ;

        type_rd_req_vld_rdy_r0 <= type_rd_req_vld_rdy   ;
        type_rd_req_vld_rdy_r1 <= type_rd_req_vld_rdy_r0;
    end
    assign type_wr_done        =    type_wr_vld_rdy_r1      ;
    assign type_rd_req_done    =    type_rd_req_vld_rdy_r1  ;
end
endgenerate

//  ====================== ddr_clk domain ======================
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// cmd_fifo_rd_en
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        cmd_fifo_rd_en <= 1'b0;
    else if (curr_state == READ_CMD)
        cmd_fifo_rd_en <= 1'b1;
    else
        cmd_fifo_rd_en <= 1'b0;
end

// m_axi_awid m_axi_awaddr m_axi_awlen axi_type re_pack
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0) 
        {axi_addr, axi_len, axi_type, source_id, re_pack} <= 'd0;
    else if (cmd_fifo_rd_en)
        {axi_addr, axi_len, axi_type, source_id, re_pack} <= cmd_fifo_dout;
end

// m_axi_awvalid
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        m_axi_awvalid <= 1'b0;
    else if (m_axi_awvalid && m_axi_awready)
        m_axi_awvalid <= 1'b0;
    else if (curr_state == SEND_ADDR && axi_type == TYPE_WRITE)
        m_axi_awvalid <= 1'b1;
end

// m_axi_arvalid
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        m_axi_arvalid <= 1'b0;
    else if (m_axi_arvalid && m_axi_arready)
        m_axi_arvalid <= 1'b0;
    else if (curr_state == SEND_ADDR && axi_type == TYPE_RD_REQ)
        m_axi_arvalid <= 1'b1;
end

// // m_axi_wvalid
// always@(posedge ddr_clk or negedge ddr_rst_n) begin
//     if(ddr_rst_n == 1'b0)
//         m_axi_wvalid <= 1'b0;
//     else if (m_axi_wvalid && m_axi_wready && m_axi_wlast)
//         m_axi_wvalid <= 1'b0;
//     else if (curr_state == SEND_DATA && ~data_fifo_empty)
//         m_axi_wvalid <= 1'b1;
//     else
//         m_axi_wvalid <= 1'b0;
// end

// read_cmd_fifo_dly
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        read_cmd_fifo_dly <= 1'b0;
    else if (curr_state == READ_CMD)
        read_cmd_fifo_dly <= 1'b1;
    else 
        read_cmd_fifo_dly <= 1'b0;
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE: begin
            if (~cmd_fifo_empty)    // 重fifo读命令需要 2clk
                next_state = READ_CMD; 
            else                    
                next_state = S_IDLE;
        end
        READ_CMD : begin
            if (read_cmd_fifo_dly)
                next_state = SEND_ADDR;
            else 
                next_state = READ_CMD;
        end
        SEND_ADDR: begin
            if ((axi_type == TYPE_WRITE) && m_axi_awvalid && m_axi_awready)
                next_state = SEND_DATA; 
            else if ((axi_type == TYPE_RD_REQ) && m_axi_arvalid && m_axi_arready)
                next_state = S_IDLE; 
            else 
                next_state = SEND_ADDR;
        end
        SEND_DATA: begin
            if (m_axi_wvalid && m_axi_wready && m_axi_wlast)
                next_state = S_IDLE; 
            else
                next_state = SEND_DATA;
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
nsu_cmd_fifo u_nsu_cmd_fifo (
    .rst                (~noc_rst_n                ), 
    .wr_clk             (noc_clk                   ), 
    .rd_clk             (ddr_clk                   ), 
    .din                (cmd2ddr                   ), 
    .wr_en              (en_cmd2ddr                ), 
    .rd_en              (cmd_fifo_rd_en            ),
    .dout               (cmd_fifo_dout             ),
    .full               (cmd_fifo_full             ), 
    .empty              (cmd_fifo_empty            )  
);

nsu_data_fifo u_nsu_data_fifo (
    .rst                (~noc_rst_n                ), 
    .wr_clk             (noc_clk                   ), 
    .rd_clk             (ddr_clk                   ), 
    .din                ({data2ddr_last, data2ddr} ), // {data2ddr_last,data2ddr}  width = DATA_WIDTH + 1
    .wr_en              (en_data2ddr               ), 
    .rd_en              (data_fifo_rd_en           ),
    .dout               (data_fifo_dout            ), 
    .full               (data_fifo_full            ), 
    .empty              (data_fifo_empty           ),
    .wr_rst_busy        (),
    .rd_rst_busy        () 
);


endmodule //nsu_asyn_cross_noc2axi
