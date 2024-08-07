/*
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn
    
    - Packetizing NoC Packet Protocol(NPP) -

    数据格式如下：
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, reserved(8 + 32 bit), HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , reserved(8 + 32 bit), TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]
        
        <pack_order> : use one-hot encoding   
*/

module nsu_packet_proc #(
    parameter   DATA_WIDTH          =      128            ,
    parameter   ID_WIDTH            =      4              ,
    parameter   VIRTUAL_CH_NUM      =      16             ,
    parameter   AXI_ID_WIDTH        =      4              ,
    parameter   AXI_ADDR_WIDTH      =      32             ,
    parameter   AXI_DATA_WIDTH      =      DATA_WIDTH     ,
    parameter   TYPE_WRITE          =      3'b100         ,
    parameter   TYPE_RD_REQ         =      3'b010         ,
    parameter   TYPE_BRESP          =      3'b011         ,
    parameter   TYPE_RD_DATA        =      3'b001         ,

    parameter   FLIT_NUM_MAX        =      16             ,  // {head + 16*body + tail}
    parameter   Source_ID           = {ID_WIDTH{1'b1}}    ,

    parameter   HEAD_CODE_BIT       =      4              ,  // Head flit校验字符长度
    parameter   HEAD_CODE_H         =      4'hA           ,  // Head flit头部校验字符
    parameter   HEAD_CODE_E         =      4'hB           ,  // Head flit尾部校验字符
    parameter   TAIL_CODE_BIT       =      4              ,  // Tail flit校验字符长度
    parameter   TAIL_CODE_H         =      4'hC           ,  // Tail flit头部校验字符
    parameter   TAIL_CODE_E         =      4'hD              // Tail flit尾部校验字符  
)(
    // ddr domain clk      
    input                                   ddr_clk             ,
    input                                   ddr_rst_n           ,
    // - STATE JUMP CONDITION -
    input                                   m_axi_awvalid       ,
    input                                   m_axi_awready       ,
    input       [7:0]                       m_axi_awlen         ,

    input                                   m_axi_arid          ,
    input                                   m_axi_arvalid       ,
    input                                   m_axi_arready       ,
    input       [7:0]                       m_axi_arlen         ,

    input                                   m_axi_wvalid        ,
    input                                   m_axi_wready        ,

    input       [AXI_ADDR_WIDTH - 1:0]      re_pack             ,
    // noc packetized
    output wire [DATA_WIDTH:0]              noc_outdata         ,        
    output wire                             m_is_head           ,
    output wire                             m_is_tail           , 
     // Master Interface Write Response Port
    input      [AXI_ID_WIDTH - 1:0]         m_axi_bid           ,
    input      [1:0]                        m_axi_bresp         ,
    input                                   m_axi_bvalid        ,
    output reg                              m_axi_bready        ,
     // Master Interface Read Data Ports
    input      [AXI_ID_WIDTH - 1:0]         m_axi_rid           ,
    input      [AXI_DATA_WIDTH - 1:0]       m_axi_rdata         ,
    input      [1:0]                        m_axi_rresp         ,
    input                                   m_axi_rlast         ,
    input                                   m_axi_rvalid        ,
    output reg                              m_axi_rready        ,

    output wire                             TypeRdReqFinish
);

// ----- localparam -----
localparam  S_IDLE       = 4'b0001,
            SELECT       = 4'b0010,
            WAIT_BRESP   = 4'b0100,
            WAIT_RD_DATA = 4'b1000;

// ----- wire -----
wire [ID_WIDTH - 1:0]                   dest_id             ; 
wire                                    fifo_wr_en          ;
wire [DATA_WIDTH - 1:0]                 fifo_dout           ;

wire [VIRTUAL_CH_NUM - 1:0]             fc_basepack_order   ; // for check
wire [7:0]                              fc_total_lens       ;

wire [VIRTUAL_CH_NUM - 1 : 0]           pack_num_onehot     ;
wire [VIRTUAL_CH_NUM - 1 : 0]           pack_num_axi_onehot ;
wire                                    init_flag           ;

wire [DATA_WIDTH - 1:0]                 flit_head           ;
wire [DATA_WIDTH - 1:0]                 flit_tail           ;

// ----- reg -----
reg  [3:0]                              pack_num_binary     ;
reg  [7:0]                              pack_num_lens_s1    ;

reg  [3:0]                              curr_state          ;
reg  [3:0]                              next_state          ;
reg  [2:0]                              type                ;

reg  [7:0]                              axi_len             ;

reg  [3:0]                              pack_num_axi        ;
reg  [VIRTUAL_CH_NUM - 1:0]             pack_num            ;
// reg  [3:0]                              total_pack_num      ;
reg  [VIRTUAL_CH_NUM - 1:0]             pack_order          ;
reg  [VIRTUAL_CH_NUM - 1:0]             pack_num_for_compare;

reg  [1:0]                              axi_bresp_r0        ;
reg  [DATA_WIDTH:0]                     noc_outdata_bp      ;
reg  [$clog2(FLIT_NUM_MAX+2):0]         flit_cnt_bp         ;
reg                                     m_is_head_bp        ;
reg                                     m_is_tail_bp        ;
reg                                     fifodata_rd_en      ;

reg  [DATA_WIDTH:0]                     noc_outdata_rd      ;
reg  [$clog2(FLIT_NUM_MAX+2):0]         flit_cnt_rd         ;
reg                                     m_is_head_rd        ;
reg                                     m_is_tail_rd        ;

reg                                     bresp_done          ;
reg                                     rd_send_last        ;
reg  [4:0]                              init_flag_shreg     ;

reg  [3:0]                              rst_busy_cnt        ;
reg                                     fifo_rst_busy       ;
reg                                     fifo_rst            ;

reg  [AXI_ID_WIDTH - 1:0]               m_axi_bid_r0        ;
reg  [AXI_ID_WIDTH - 1:0]               m_axi_arid_r0       ;

reg                                     send_bresp_vld      ;
// ---------------------------- combinational logic -------------------------
assign fifo_wr_en = m_axi_rvalid & m_axi_rready;
// head --> [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, reserved(8 + 32 bit), HEAD_CODE_E]
assign flit_head = {HEAD_CODE_H, Source_ID, dest_id, type, pack_order, {(8 + AXI_ADDR_WIDTH){1'b0}}, HEAD_CODE_E 
                ,{(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
// tail --> [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , reserved(8 + 32 bit), TAIL_CODE_E]
assign flit_tail = {TAIL_CODE_H, Source_ID, dest_id, type, pack_num, {(8 + AXI_ADDR_WIDTH){1'b0}}, TAIL_CODE_E
                ,{(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};

assign {fc_basepack_order, fc_total_lens} = re_pack[AXI_ADDR_WIDTH - 1:0];
assign dest_id = (type == TYPE_BRESP) ? m_axi_bid_r0 : m_axi_arid_r0;

assign init_flag = ~(|init_flag_shreg);

assign noc_outdata = (type == TYPE_BRESP) ? noc_outdata_bp : noc_outdata_rd;

assign m_is_head = (type == TYPE_BRESP) ? m_is_head_bp : m_is_head_rd;
assign m_is_tail = (type == TYPE_BRESP) ? m_is_tail_bp : m_is_tail_rd;

assign TypeRdReqFinish = curr_state == WAIT_RD_DATA && next_state == S_IDLE;

// fifodata_rd_en
always@(*) begin
    if(ddr_rst_n == 1'b0)
        fifodata_rd_en <= 1'b0; 
    else if (~data_fifo_empty && ~(flit_cnt_rd == 'd0 || rd_send_last == 1'b1))
        fifodata_rd_en <= 1'b1;
    else
        fifodata_rd_en <= 1'b0;  
end

// ---------------------------- sequential logic ----------------------------
// fsm_state
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// m_axi_bid_r0
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        m_axi_bid_r0 <= 'd0;
    else if (m_axi_bvalid & m_axi_bready)
        m_axi_bid_r0 <= m_axi_bid;
end

// m_axi_arid_r0
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        m_axi_arid_r0 <= 'd0;
    else if (m_axi_arready & m_axi_arvalid)
        m_axi_arid_r0 <= m_axi_arid;
end

// send_bresp_vld
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        send_bresp_vld <= 'd0;
    else if (bresp_done)
        send_bresp_vld <= 'd0;
    else if (m_axi_bvalid && m_axi_bready)
        send_bresp_vld <= 1'b1;
end

// pack_num_binary
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        pack_num_binary  <= 'd0;
    else if (curr_state == WAIT_RD_DATA && fc_total_lens != axi_len) begin
        pack_num_lens_s1 <= fc_total_lens - axi_len - 1'b1;
        pack_num_binary  <= pack_num_lens_s1[7:4] + axi_len[7:4] + 'd2;
    end
    else if (curr_state == WAIT_RD_DATA)
        pack_num_binary <= axi_len[7:4] + 'd1;
end

// m_axi_bready
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        m_axi_bready <= 1'b0;
    else if (m_axi_bready & m_axi_bvalid)
        m_axi_bready <= 1'b0;    
    else if (type == TYPE_BRESP && curr_state == WAIT_BRESP)
        m_axi_bready <= 1'b1;
end

// init_flag_shreg
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        init_flag_shreg <= 'd0;
    else if (curr_state == SELECT && m_axi_arready && m_axi_arvalid)
        init_flag_shreg <= {init_flag_shreg[4:1], 1'b1};
    else
        init_flag_shreg <= {init_flag_shreg[3:0], 1'b0};
end

// m_axi_rready
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        m_axi_rready <= 1'b0;
    else if (m_axi_rready && m_axi_rvalid && m_axi_rlast)
        m_axi_rready <= 1'b0;  
    else if (type == TYPE_RD_DATA && curr_state == WAIT_RD_DATA && init_flag && ~fifo_rst_busy)
        m_axi_rready <= 1'b1;
end

// axi_len type : TYPE_BRESP  TYPE_RD_DATA
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0) begin
        axi_len <= 'd0;
        type    <= 3'b111;
    end
    else if (m_axi_awvalid & m_axi_awready) begin
        axi_len <= m_axi_awlen;
        type    <= TYPE_BRESP;
    end
    else if (m_axi_arvalid & m_axi_arready) begin
        axi_len <= m_axi_arlen;
        type    <= TYPE_RD_DATA;
    end
end

// pack_num_axi
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        pack_num_axi <= 'd0;
    else
        pack_num_axi <= axi_len[7:4] + 1'b1;
end

// pack_order
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        pack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (type == TYPE_BRESP) begin
        case(fc_basepack_order[0])
            0: pack_order <= {{(VIRTUAL_CH_NUM-2){1'b0}}, 1'b1, 1'b0};
            default: pack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
        endcase
    end
    else if (type == TYPE_RD_DATA) begin
        if (~init_flag)
            pack_order <= fc_basepack_order;
        else if (rd_send_last)
            pack_order <= {pack_order[VIRTUAL_CH_NUM-2:0], pack_order[VIRTUAL_CH_NUM-1]};
    end
end

// pack_num_for_compare
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        pack_num_for_compare <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (fc_basepack_order[0])
        pack_num_for_compare <= pack_num_axi_onehot;
    else
        pack_num_for_compare <= pack_num_onehot;
end

// pack_num
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        pack_num <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (type == TYPE_BRESP) begin
        if (fc_total_lens != axi_len)
            pack_num <= {{(VIRTUAL_CH_NUM-2){1'b0}}, 1'b1, 1'b0};
        else 
            pack_num <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    end
    else if (type == TYPE_RD_DATA) begin
        pack_num <= pack_num_onehot;
    end
end

// axi_bresp_r0
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        axi_bresp_r0 <= 2'b11;
    else if (m_axi_bvalid & m_axi_bready)
        axi_bresp_r0 <= m_axi_bresp;
end

// TYPE_BRESP
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0) begin
        noc_outdata_bp <= 'd0;
        flit_cnt_bp    <= 'd0;
        m_is_head_bp   <= 1'b0;
        m_is_tail_bp   <= 1'b0;
        bresp_done     <= 1'b0;
    end
    else if (curr_state == WAIT_BRESP && send_bresp_vld) begin
        case(flit_cnt_bp)
            0: begin
                noc_outdata_bp <= {1'b1, flit_head};
                flit_cnt_bp    <= flit_cnt_bp + 1'b1;
                m_is_head_bp   <= 1'b1;
                m_is_tail_bp   <= 1'b0;
                bresp_done     <= 1'b0;
            end
            1: begin
                noc_outdata_bp <= {1'b1, {(DATA_WIDTH-2){1'b0}}, axi_bresp_r0};
                flit_cnt_bp    <= flit_cnt_bp + 1'b1;
                m_is_head_bp   <= 1'b0;
                m_is_tail_bp   <= 1'b0;
                bresp_done     <= 1'b1;
            end
            2: begin
                noc_outdata_bp <= {1'b1, flit_tail};
                flit_cnt_bp    <= flit_cnt_bp + 1'b1;
                m_is_head_bp   <= 1'b0;
                m_is_tail_bp   <= 1'b1;
                bresp_done     <= 1'b0;
            end
            default: begin
                noc_outdata_bp <= 'd0;
                flit_cnt_bp    <= flit_cnt_bp;
                m_is_head_bp   <= 1'b0;
                m_is_tail_bp   <= 1'b0;
                bresp_done     <= 1'b0;
            end
        endcase
    end
    else begin
        noc_outdata_bp <= 'd0;
        flit_cnt_bp    <= 'd0;
        m_is_head_bp   <= 1'b0;
        m_is_tail_bp   <= 1'b0;
        bresp_done     <= 1'b0;
    end
end

// TYPE_RD_DATA
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0) begin
        noc_outdata_rd <= 'd0;
        flit_cnt_rd    <= 'd0;
        m_is_head_rd   <= 1'b0;
        m_is_tail_rd   <= 1'b0;
        rd_send_last   <= 1'b0;
    end
    else if (curr_state == WAIT_RD_DATA && init_flag && (type == TYPE_RD_DATA) && ((pack_order != pack_num_for_compare) || (axi_len[3:0] == 4'b1111))) begin  
        case(flit_cnt_rd) 
            0: begin
                noc_outdata_rd <= {1'b1, flit_head};
                flit_cnt_rd    <= flit_cnt_rd + 1'b1;
                m_is_head_rd   <= 1'b1;
                m_is_tail_rd   <= 1'b0;
                rd_send_last   <= 1'b0;
            end
            FLIT_NUM_MAX + 'd1: begin
                noc_outdata_rd <= {1'b1, flit_tail};
                flit_cnt_rd    <= 'd0;
                m_is_head_rd   <= 1'b0;
                m_is_tail_rd   <= 1'b1;
                rd_send_last   <= 1'b0;
            end
            default: begin
                if (fifodata_rd_en) begin
                    noc_outdata_rd <= {1'b1, fifo_dout};
                    flit_cnt_rd    <= flit_cnt_rd + 1'b1;
                    m_is_head_rd   <= 1'b0;
                    m_is_tail_rd   <= 1'b0;
                    if (flit_cnt_rd == FLIT_NUM_MAX) begin
                        rd_send_last <= 1'b1;
                    end
                end
                else begin
                    noc_outdata_rd <= {1'b0, fifo_dout};
                    flit_cnt_rd    <= flit_cnt_rd;
                    m_is_head_rd   <= 1'b0;
                    m_is_tail_rd   <= 1'b0;
                    rd_send_last   <= 1'b0;     
                end
            end
        endcase
    end
    else if (curr_state == WAIT_RD_DATA && init_flag && type == TYPE_RD_DATA && pack_order == pack_num_for_compare) begin
        if (flit_cnt_rd == 'd0) begin
            noc_outdata_rd <= {1'b1, flit_head};
            flit_cnt_rd    <= flit_cnt_rd + 1'b1;
            m_is_head_rd   <= 1'b1;
            m_is_tail_rd   <= 1'b0;
            rd_send_last   <= 1'b0;
        end
        else if (flit_cnt_rd == axi_len[3:0] + 'd2) begin
            noc_outdata_rd <= {1'b1, flit_tail};
            flit_cnt_rd    <= 'd0;
            m_is_head_rd   <= 1'b0;
            m_is_tail_rd   <= 1'b1;
            rd_send_last   <= 1'b0;
        end
        else begin
            if (fifodata_rd_en) begin
                noc_outdata_rd <= {1'b1, fifo_dout};
                flit_cnt_rd    <= flit_cnt_rd + 1'b1;
                m_is_head_rd   <= 1'b0;
                m_is_tail_rd   <= 1'b0;
                if (flit_cnt_rd == axi_len[3:0] + 'd1) begin
                    rd_send_last <= 1'b1;
                end
            end
            else begin
                noc_outdata_rd <= {1'b0, fifo_dout};
                flit_cnt_rd    <= flit_cnt_rd;
                m_is_head_rd   <= 1'b0;
                m_is_tail_rd   <= 1'b0;
                rd_send_last   <= 1'b0;     
            end
        end
    end
    else begin
        noc_outdata_rd <= 'd0;
        flit_cnt_rd    <= 'd0;
        m_is_head_rd   <= 1'b0; 
        m_is_tail_rd   <= 1'b0;
        rd_send_last   <= 1'b0; 
    end
end

// rst_busy_cnt
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        rst_busy_cnt <= 'd0;
    else if (rst_busy_cnt == 'd11)
        rst_busy_cnt <= 'd0;
    else if (fifo_rst || (|rst_busy_cnt))
        rst_busy_cnt <= rst_busy_cnt + 1'b1;
end

// fifo_rst_busy
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        fifo_rst_busy <= 1'b0;
    else if (fifo_rst_busy && (rst_busy_cnt >= 'd10))
        fifo_rst_busy <= 1'b0;
    else if (fifo_rst)
        fifo_rst_busy <= 1'b1;
end

// fifo_rst
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0)
        fifo_rst <= 1'b1;
    else if (curr_state == S_IDLE)
        fifo_rst <= 1'b1;
    else
        fifo_rst <= 1'b0;
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE: begin
            next_state = SELECT;   
        end
        SELECT: begin
            if (m_axi_wready & m_axi_wvalid)
                next_state = WAIT_BRESP;
            else if (m_axi_arready & m_axi_arvalid)
                next_state = WAIT_RD_DATA;
            else
                next_state = SELECT;
        end
        WAIT_BRESP: begin
            if (bresp_done)
                next_state = S_IDLE;
            else 
                next_state = WAIT_BRESP;
        end
        WAIT_RD_DATA: begin
            if (rd_send_last && pack_order == pack_num_for_compare)
                next_state = S_IDLE;
            else 
                next_state = WAIT_RD_DATA;
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
nsu_send_data_fifo u_nsu_send_data_fifo (
    .clk             (ddr_clk           ),
    .srst            (fifo_rst          ),
    .din             (m_axi_rdata       ),
    .wr_en           (fifo_wr_en        ), 
    .rd_en           (fifodata_rd_en    ),
    .dout            (fifo_dout         ),
    .full            (                  ),
    .empty           (data_fifo_empty   )
);

bin2onehot  #(
    .VIRTUAL_CH_NUM ( VIRTUAL_CH_NUM )
) u_bin2onehot_nsu_packet(
    .sclk           (ddr_clk            ),
    .s_rst_n        (ddr_rst_n          ),
    .bin_data       (pack_num_binary    ),
    .onehot_data    (pack_num_onehot    )
);

bin2onehot  #(
    .VIRTUAL_CH_NUM ( VIRTUAL_CH_NUM )
) u_bin2onehot_axi(
    .sclk           (ddr_clk             ),
    .s_rst_n        (ddr_rst_n           ),
    .bin_data       (pack_num_axi        ),
    .onehot_data    (pack_num_axi_onehot )
);

endmodule //nsu_packet_proc
