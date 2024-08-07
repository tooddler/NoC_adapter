/*
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn
   
    - Packetizing NoC Packet Protocol(NPP) -
    type :
        读数据 写响应：3'b100 ;  读返回数据 ： 3'b001 ; 写请求 ：3'b010 ;  
    
    write data:
        addr 先于 data

    数据格式如下：
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, AXI_LEN(8bit), AXI_ADDR, HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , AXI_LEN(8bit), AXI_ADDR, TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]
        
        <pack_order> : use one-hot encoding   
*/

module packet_proc #(
    parameter   DATA_WIDTH          =      128            ,
    parameter   ID_WIDTH            =      4              ,
    parameter   VIRTUAL_CH_NUM      =      16             ,
    parameter   AXI_ID_WIDTH        =      4              ,
    parameter   AXI_ADDR_WIDTH      =      32             ,
    parameter   AXI_DATA_WIDTH      =      DATA_WIDTH     ,
    parameter   TYPE_WRITE          =      3'b100         ,
    parameter   TYPE_RD_REQ         =      3'b010         ,
    parameter   FLIT_NUM_MAX        =      16             ,  // {head + 16*body + tail}
    parameter   Source_ID           = {ID_WIDTH{1'b0}}    ,

    parameter   HEAD_CODE_BIT       =      4              ,  // Head flit校验字符长度
    parameter   HEAD_CODE_H         =      4'h5           ,  // Head flit头部校验字符
    parameter   HEAD_CODE_E         =      4'hA           ,  // Head flit尾部校验字符
    parameter   TAIL_CODE_BIT       =      4              ,  // Tail flit校验字符长度
    parameter   TAIL_CODE_H         =      4'h0           ,  // Tail flit头部校验字符
    parameter   TAIL_CODE_E         =      4'hF              // Tail flit尾部校验字符
)(
    // user domain clk
    input                                     axi_clk       ,
    input                                     axi_rst_n     ,
    // Bresp
    input       [1:0]                         s_axi_bresp   ,
    input                                     s_axi_bvalid  ,
    input                                     s_axi_bready  ,
    // noc packetized
    output wire [DATA_WIDTH:0]                noc_outdata   ,        
    output wire                               m_is_head     ,
    output wire                               m_is_tail     ,  
    // Packetizing port :  interact with axi module
     // Slave Interface Write Address Ports
    input       [AXI_ID_WIDTH - 1:0]          s_axi_awid    ,
    input       [AXI_ADDR_WIDTH - 1:0]        s_axi_awaddr  ,
    input       [7:0]                         s_axi_awlen   ,
    input       [2:0]                         s_axi_awsize  ,
    input       [1:0]                         s_axi_awburst ,
    input       [0:0]                         s_axi_awlock  ,
    input       [3:0]                         s_axi_awcache ,
    input       [2:0]                         s_axi_awprot  ,
    input       [3:0]                         s_axi_awqos   ,
    input                                     s_axi_awvalid ,
    output reg                                s_axi_awready ,
     // Slave Interface Write Data Port
    input       [AXI_DATA_WIDTH - 1:0]        s_axi_wdata   ,
    input       [AXI_DATA_WIDTH/8 - 1:0]      s_axi_wstrb   ,
    input                                     s_axi_wlast   ,
    input                                     s_axi_wvalid  ,
    output reg                                s_axi_wready  ,
     // Slave Interface Read Address Ports 
    input       [AXI_ID_WIDTH - 1:0]          s_axi_arid    ,
    input       [AXI_ADDR_WIDTH - 1:0]        s_axi_araddr  ,
    input       [7:0]                         s_axi_arlen   ,
    input       [2:0]                         s_axi_arsize  ,
    input       [1:0]                         s_axi_arburst ,
    input       [0:0]                         s_axi_arlock  ,
    input       [3:0]                         s_axi_arcache ,
    input       [2:0]                         s_axi_arprot  ,
    input       [3:0]                         s_axi_arqos   ,
    input                                     s_axi_arvalid ,
    output reg                                s_axi_arready 
);

// ----- localparam -----
localparam  S_IDLE           = 6'b000_001,
            WADDR_LEN_DATA   = 6'b000_010,
            DDR_SEND         = 6'b000_100,
            WAIT_BRESP       = 6'b001_000,
            RADDR_LEN        = 6'b010_000,
            RADDR_REQ_SEND   = 6'b100_000;

// ----- wire -----
wire                                    data_fifo_empty ;
wire [DATA_WIDTH - 1:0]                 fifo_dout       ;
wire [DATA_WIDTH - 1:0]                 flit_head       ;
wire [DATA_WIDTH - 1:0]                 flit_tail       ;

wire [VIRTUAL_CH_NUM - 1:0]             pack_num_onehot ;


wire                                    dest_en         ;
wire  [AXI_ADDR_WIDTH - 1:0]            dest_addr       ;
wire  [7:0]                             dest_len        ;
wire  [ID_WIDTH - 1:0]                  dest_id         ;          
wire                                    lookup_done     ;
wire  [AXI_ADDR_WIDTH - 1:0]            re_pack         ;
wire                                    fifo_wr_en      ;

// ----- reg -----
reg  [5:0]                              curr_state      ;
reg  [5:0]                              next_state      ;
reg  [DATA_WIDTH:0]                     noc_outdata_wr  ;
reg  [DATA_WIDTH:0]                     noc_outdata_rd  ;

reg  [3:0]                              rst_busy_cnt    ;
reg                                     fifo_rst_busy   ;
reg                                     fifo_rst        ;

reg  [AXI_ADDR_WIDTH - 1:0]             aaddr_r0        ;
reg  [7:0]                              alen_r0         ;
reg                                     lookup_en       ;
reg                                     next_req        ; 

reg                                     fifodata_rd_en  ; 
reg  [3:0]                              pack_num        ;

reg  [2:0]                              type            ;
reg  [VIRTUAL_CH_NUM - 1:0]             pack_order      ;
reg  [VIRTUAL_CH_NUM - 1:0]             basepack_order  ;
reg                                     flit_data_last  ;
reg                                     flit_req_last   ;  

reg                                     dest_en_r0      ;
reg                                     dest_en_r1      ;
reg  [$clog2(FLIT_NUM_MAX+2):0]         flit_cnt        ;
reg  [1:0]                              flit_rd_cnt     ;                 
reg                                     lookup_flag     ;
reg  [3:0]                              lookup_wait_cnt ;
reg                                     ddr_send_done   ;

reg                                     m_is_head_wr    ;
reg                                     m_is_tail_wr    ; 
reg                                     m_is_head_rd    ;
reg                                     m_is_tail_rd    ; 

// ---------------------------- combinational logic -------------------------
assign fifo_wr_en = s_axi_wvalid & s_axi_wready;
// [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, AXI_LEN(8bit), AXI_ADDR(32bit)/re_pack, HEAD_CODE_E]
assign flit_head = {HEAD_CODE_H, Source_ID, dest_id, type, pack_order, dest_len, dest_addr, HEAD_CODE_E 
                   ,{(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
assign flit_tail= (curr_state == RADDR_REQ_SEND)
                ?{TAIL_CODE_H, Source_ID, dest_id, type, {{(VIRTUAL_CH_NUM-1){1'b0}},1'b1}, dest_len, re_pack, TAIL_CODE_E,{(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}}
                :{TAIL_CODE_H, Source_ID, dest_id, type, pack_num_onehot, dest_len, re_pack, TAIL_CODE_E,{(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};

assign re_pack[AXI_ADDR_WIDTH - 1:0] = {basepack_order, alen_r0}; // 低位补齐 高位添 0

assign noc_outdata = (curr_state == RADDR_REQ_SEND) ? noc_outdata_rd : noc_outdata_wr;

assign m_is_head = (curr_state == RADDR_REQ_SEND) ? m_is_head_rd : m_is_head_wr;
assign m_is_tail = (curr_state == RADDR_REQ_SEND) ? m_is_tail_rd : m_is_tail_wr;

// fifodata_rd_en
always@(*) begin
    if(axi_rst_n == 1'b0)
        fifodata_rd_en <= 1'b0; 
    else if ((curr_state == DDR_SEND) && ~data_fifo_empty && ~(|lookup_wait_cnt) && ~(flit_cnt == 'd0 || flit_data_last == 1'b1))
        fifodata_rd_en <= 1'b1;
    else
        fifodata_rd_en <= 1'b0;  
end

// ---------------------------- sequential logic ----------------------------
// fsm_state
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// dest_en_r
always@(posedge axi_clk) begin
    dest_en_r0 <= dest_en;
    dest_en_r1 <= dest_en_r0;
end

// aaddr_r0
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        aaddr_r0 <= 'd0;
    else if (s_axi_awvalid & s_axi_awready)
        aaddr_r0 <= s_axi_awaddr;
    else if (s_axi_arvalid & s_axi_arready)
        aaddr_r0 <= s_axi_araddr;
end

// alen_r0
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        alen_r0 <= 'd0;
    else if (s_axi_awvalid & s_axi_awready)
        alen_r0 <= s_axi_awlen;
    else if (s_axi_arvalid & s_axi_arready)
        alen_r0 <= s_axi_arlen;
end

// lookup_en
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        lookup_en <= 1'b0;
    else if ((s_axi_awvalid & s_axi_awready) || (s_axi_arvalid & s_axi_arready))
        lookup_en <= 1'b1;
    else
        lookup_en <= 1'b0;     
end

// next_req
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        next_req <= 1'b0;
    else if (lookup_flag == 1'b0 && flit_data_last && (pack_order == pack_num_onehot))
        next_req <= 1'b1;
    else if (lookup_flag == 1'b0 && flit_req_last)
        next_req <= 1'b1;
    else
        next_req <= 1'b0;     
end

// ddr_send_done
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        ddr_send_done <= 1'b0;
    else if (lookup_flag == 1'b1 && flit_data_last && (pack_order == pack_num_onehot))
        ddr_send_done <= 1'b1;
    else if (lookup_flag == 1'b1 && flit_req_last)
        ddr_send_done <= 1'b1;
    else
        ddr_send_done <= 1'b0;     
end

// lookup_flag
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        lookup_flag <= 1'b0;
    else if (lookup_done && (curr_state == DDR_SEND || curr_state == RADDR_REQ_SEND))
        lookup_flag <= 1'b1;
    else 
        lookup_flag <= 1'b0;
end

// lookup_wait_cnt
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        lookup_wait_cnt <= 1'b0;
    else if (lookup_wait_cnt == 'd4)
        lookup_wait_cnt <= 1'b0;
    else if (next_req || (|lookup_wait_cnt))
        lookup_wait_cnt <= lookup_wait_cnt + 1'b1;
end

// s_axi_awready
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        s_axi_awready <= 1'b0;
    else if ((~fifo_rst_busy) && (curr_state == S_IDLE) && ~fifo_rst)
        s_axi_awready <= 1'b1;
    else
        s_axi_awready <= 1'b0;    
end

// s_axi_arready
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        s_axi_arready <= 1'b0;
    else if (curr_state == S_IDLE)
        s_axi_arready <= 1'b1;
    else
        s_axi_arready <= 1'b0;    
end

// s_axi_wready
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        s_axi_wready <= 1'b0;
    else if (s_axi_wready && s_axi_wlast && s_axi_wvalid)
        s_axi_wready <= 1'b0;
    else if (curr_state == WADDR_LEN_DATA)
        s_axi_wready <= 1'b1;   
end

// rst_busy_cnt
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        rst_busy_cnt <= 'd0;
    else if (rst_busy_cnt == 'd11)
        rst_busy_cnt <= 'd0;
    else if (fifo_rst || (|rst_busy_cnt))
        rst_busy_cnt <= rst_busy_cnt + 1'b1;
end

// fifo_rst_busy
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        fifo_rst_busy <= 1'b0;
    else if (fifo_rst_busy && (rst_busy_cnt >= 'd10))
        fifo_rst_busy <= 1'b0;
    else if (fifo_rst)
        fifo_rst_busy <= 1'b1;
end

// fifo_rst
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        fifo_rst <= 1'b1;
    else if ((s_axi_bresp == 2'b00) & s_axi_bvalid & s_axi_bready)
        fifo_rst <= 1'b1;
    else
        fifo_rst <= 1'b0;
end

// pack_num
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        pack_num <= 'd0;
    else
        pack_num <= dest_len[7:4] + 1'b1;
end

// type : TYPE_WRITE  TYPE_RD_REQ
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        type <= 3'b111;
    else if (curr_state == WADDR_LEN_DATA)
        type <= TYPE_WRITE;
    else if (curr_state == RADDR_LEN)
        type <= TYPE_RD_REQ;
end

// pack_order
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        pack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (flit_data_last && (pack_order == pack_num_onehot))
        pack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (flit_data_last)
        pack_order <= {pack_order[VIRTUAL_CH_NUM-2:0], pack_order[VIRTUAL_CH_NUM-1]};
end

// basepack_order
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        basepack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (curr_state == S_IDLE)
        basepack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if ((curr_state == DDR_SEND) && flit_data_last && (pack_order == pack_num_onehot))
        basepack_order <= {pack_order[VIRTUAL_CH_NUM-2:0], pack_order[VIRTUAL_CH_NUM-1]};
    else if (curr_state == RADDR_REQ_SEND && flit_req_last)
        basepack_order <= {pack_num_onehot[VIRTUAL_CH_NUM-2:0], pack_num_onehot[VIRTUAL_CH_NUM-1]};
end

// noc_outdata_wr flit_cnt  flit_data_last  m_is_head_wr m_is_tail_wr
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0) begin
        noc_outdata_wr <= 'd0;
        flit_cnt       <= 'd0;
        flit_data_last <= 1'b0;
        m_is_head_wr   <= 1'b0;
        m_is_tail_wr   <= 1'b0;
    end
    else if (~ddr_send_done && ~next_req && ~(|lookup_wait_cnt) && (curr_state == DDR_SEND) && ((pack_order != pack_num_onehot) || (dest_len[3:0] == 4'b1111)))begin
        case(flit_cnt)
            0: begin
                noc_outdata_wr <= {1'b1, flit_head};
                flit_cnt       <= flit_cnt + 1'b1;
                flit_data_last <= 1'b0;
                m_is_head_wr   <= 1'b1;
                m_is_tail_wr   <= 1'b0;
            end
            FLIT_NUM_MAX + 'd1: begin
                noc_outdata_wr <= {1'b1, flit_tail};
                flit_cnt       <= 'd0;
                flit_data_last <= 1'b0;
                m_is_head_wr   <= 1'b0;
                m_is_tail_wr   <= 1'b1;
            end
            default: begin
                if (fifodata_rd_en) begin
                    noc_outdata_wr <= {1'b1, fifo_dout};
                    flit_cnt       <= flit_cnt + 1'b1;
                    m_is_head_wr   <= 1'b0;
                    m_is_tail_wr   <= 1'b0;
                    if (flit_cnt == FLIT_NUM_MAX) begin
                        flit_data_last <= 1'b1;
                    end
                end
                else begin
                    noc_outdata_wr <= {1'b0, fifo_dout};
                    flit_cnt       <= flit_cnt;
                    flit_data_last <= flit_data_last;
                    m_is_head_wr   <= 1'b0;
                    m_is_tail_wr   <= 1'b0;     
                end
            end
        endcase
    end
    else if (~ddr_send_done && ~next_req && ~(|lookup_wait_cnt) && (curr_state == DDR_SEND) && (pack_order == pack_num_onehot))begin
        if (flit_cnt == 'd0) begin
            noc_outdata_wr <= {1'b1, flit_head};
            flit_cnt       <= flit_cnt + 1'b1;
            flit_data_last <= 1'b0;
            m_is_head_wr   <= 1'b1;
            m_is_tail_wr   <= 1'b0;
        end
        else if (flit_cnt == (dest_len[3:0] + 'd2)) begin
            noc_outdata_wr <= {1'b1, flit_tail};
            flit_cnt       <= 'd0;
            flit_data_last <= 1'b0;
            m_is_head_wr   <= 1'b0;
            m_is_tail_wr   <= 1'b1;
        end
        else begin
            if (fifodata_rd_en) begin
                noc_outdata_wr <= {1'b1, fifo_dout};
                flit_cnt       <= flit_cnt + 1'b1;
                m_is_head_wr   <= 1'b0;
                m_is_tail_wr   <= 1'b0;
                if (flit_cnt == (dest_len[3:0] + 1'b1)) begin
                    flit_data_last <= 1'b1;
                end
            end
            else begin
                noc_outdata_wr <= {1'b0, fifo_dout};
                flit_cnt       <= flit_cnt;
                flit_data_last <= flit_data_last;   
                m_is_head_wr   <= 1'b0;
                m_is_tail_wr   <= 1'b0;   
            end
        end
    end
    else begin
        noc_outdata_wr <= 'd0;
        flit_cnt       <= 'd0;
        flit_data_last <= 1'b0; 
        m_is_head_wr   <= 1'b0;
        m_is_tail_wr   <= 1'b0; 
    end
end

// noc_outdata_rd  flit_rd_cnt  flit_req_last 
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0) begin
        noc_outdata_rd <= 'd0;
        flit_rd_cnt    <= 'd0;
        flit_req_last  <= 1'b0;
        m_is_head_rd   <= 1'b0;
        m_is_tail_rd   <= 1'b0;
    end
    else if (~ddr_send_done && ~next_req && (curr_state == RADDR_REQ_SEND) && ~(|lookup_wait_cnt)) begin
        case(flit_rd_cnt)
            0: begin
                noc_outdata_rd <= {1'b1, flit_head};
                flit_rd_cnt    <= flit_rd_cnt + 1'b1;
                flit_req_last  <= 1'b0;
                m_is_head_rd   <= 1'b1;
                m_is_tail_rd   <= 1'b0;
            end
            1: begin
                noc_outdata_rd <= {1'b1, {(DATA_WIDTH){1'b0}}};  // blank data
                flit_rd_cnt    <= flit_rd_cnt + 1'b1; 
                flit_req_last  <= 1'b1;
                m_is_head_rd   <= 1'b0;
                m_is_tail_rd   <= 1'b0;               
            end
            2: begin
                noc_outdata_rd <= {1'b1, flit_tail};
                flit_rd_cnt    <= 'd0; 
                flit_req_last  <= 1'b0;
                m_is_head_rd   <= 1'b0;
                m_is_tail_rd   <= 1'b1;                        
            end
            default : begin
                noc_outdata_rd <= 'd0;
                flit_rd_cnt    <= 'd0;
                flit_req_last  <= 1'b0;
                m_is_head_rd   <= 1'b0;
                m_is_tail_rd   <= 1'b0; 
            end
        endcase
    end
    else begin
        noc_outdata_rd <= 'd0;
        flit_rd_cnt    <= 'd0;
        flit_req_last  <= 1'b0;
        m_is_head_rd   <= 1'b0;
        m_is_tail_rd   <= 1'b0; 
    end
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE: begin  // write first
            if (s_axi_awvalid & s_axi_awready)
                next_state = WADDR_LEN_DATA;
            else if (s_axi_arvalid & s_axi_arready)
                next_state = RADDR_LEN;
            else
                next_state = S_IDLE;   
        end
    
        // write port
        WADDR_LEN_DATA: begin
            if (dest_en_r1 == 1'b1)
                next_state = DDR_SEND; 
            else
                next_state = WADDR_LEN_DATA; 
        end

        DDR_SEND : begin
            if (ddr_send_done)
                next_state = WAIT_BRESP;
            else 
                next_state = DDR_SEND;
        end

        WAIT_BRESP: begin
            if (s_axi_bvalid & s_axi_bready)
                next_state = S_IDLE; 
            else
                next_state = WAIT_BRESP;
        end

        // read port
        RADDR_LEN: begin
            if (dest_en_r1 == 1'b1)
                next_state = RADDR_REQ_SEND;  
            else 
                next_state = RADDR_LEN; 
        end

        RADDR_REQ_SEND: begin
            if (ddr_send_done)
                next_state = S_IDLE; 
            else
                next_state = RADDR_REQ_SEND; 
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
nmu_send_data_fifo u_nmu_send_data_fifo (
    .clk             (axi_clk           ),
    .srst            (fifo_rst          ),
    .din             (s_axi_wdata       ),
    .wr_en           (fifo_wr_en        ), 
    .rd_en           (fifodata_rd_en    ),
    .dout            (fifo_dout         ),
    .full            (                  ),
    .empty           (data_fifo_empty   ),
    .wr_rst_busy     (),
    .rd_rst_busy     () 
);

Address_Map #(
    .AXI_ADDR_WIDTH  ( AXI_ADDR_WIDTH ),
    .DATA_WIDTH      ( DATA_WIDTH     ),
    .ID_WIDTH        ( ID_WIDTH       )
) u_Address_Map(
    .axi_clk         ( axi_clk         ),
    .axi_rst_n       ( axi_rst_n       ),
    .lookup_en       ( lookup_en       ),
    .next_req        ( next_req        ),
    .axi_addr        ( aaddr_r0        ),
    .axi_len         ( alen_r0         ),
    // output
    .dest_en         ( dest_en         ),
    .dest_addr       ( dest_addr       ),
    .dest_len        ( dest_len        ),
    .dest_id         ( dest_id         ),
    .lookup_done     ( lookup_done     )
);

bin2onehot  #(
    .VIRTUAL_CH_NUM ( VIRTUAL_CH_NUM )
) u_bin2onehot(
    .sclk           ( axi_clk          ),
    .s_rst_n        ( axi_rst_n        ),
    .bin_data       ( pack_num         ),
    .onehot_data    ( pack_num_onehot  )
);


endmodule //packet_proc
