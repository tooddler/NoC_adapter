/*
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn
    
    - Depacketizing NoC Packet Protocol(NPP) -
    
    type :
        写数据：3'b100 ;  读返回数据 ： 3'b001 ; 读请求 ：3'b010 ; 写响应: 3'b011
    
    NMU传来数据格式如下：
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, AXI_LEN(8bit), AXI_ADDR, HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , AXI_LEN(8bit), RE_PACK, TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]
    
    NSU发出数据格式如下:
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, reserved(8 + 32 bit), HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM,   reserved(8 + 32 bit), TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]      

        <pack_order> : use one-hot encoding   
*/

module nsu_depacket_proc #(
    parameter   DATA_WIDTH          =      128            ,
    parameter   AXI_ADDR_WIDTH      =      32             ,
    parameter   FIFO_ADDR_WIDTH     =      16             ,
    parameter   ID_WIDTH            =      4              ,
    parameter   VIRTUAL_CH_NUM      =      8              ,
    parameter   AXI_ID_WIDTH        =      4              ,
    parameter   AXI_DATA_WIDTH      =      DATA_WIDTH     ,
    parameter   TYPE_WRITE          =      3'b100         ,
    parameter   TYPE_RD_REQ         =      3'b010         
)(
    // user domain clk
    input                                       noc_clk             ,
    input                                       noc_rst_n           ,
    // Depacketizing port :  interact with reorder_module       
    input       [AXI_ADDR_WIDTH - 1 : 0]        axi_addr            ,
    input       [7:0]                           axi_len             ,
    input       [2:0]                           axi_type            ,
    input       [ID_WIDTH - 1 : 0]              source_id           ,
    input       [AXI_ADDR_WIDTH - 1 : 0]        re_pack             ,
    output reg  [VIRTUAL_CH_NUM - 1 : 0]        rd_en               ,            
    input       [DATA_WIDTH - 1 : 0]            data_in             ,
    input       [VIRTUAL_CH_NUM - 1 : 0]        empty_vc            ,
    input       [VIRTUAL_CH_NUM - 1 : 0]        pack_num            ,  
    input                                       nocpack_done        ,
    // - NSU busy_signal -
    output reg                                  nsu_busy            ,
    input                                       type_wr_done        ,
    input                                       type_rd_req_done    ,
    // - clk domain cross part noc_clk to ddr_clk -  
    output wire[DATA_WIDTH - 1 : 0]             data2ddr            , // data channel
    output reg                                  data2ddr_last       ,
    output wire                                 en_data2ddr         ,
    input                                       data_fifo_full      ,  

    output reg [2*AXI_ADDR_WIDTH+ID_WIDTH+10:0] cmd2ddr             , // cmd channel    
    output reg                                  en_cmd2ddr          ,
    input                                       cmd_fifo_full   
);

// ----- localparam -----
localparam  S_IDLE           = 3'b001,
            RD_NUM_CHK_EMPTY = 3'b010,
            RD_FIFO          = 3'b100;

// ----- wire -----
wire [$clog2(VIRTUAL_CH_NUM) - 1 : 0]   pack_num_binary ; 

// ----- reg -----
reg  [2:0]                              curr_state      ;
reg  [2:0]                              next_state      ;
reg  [VIRTUAL_CH_NUM - 1 : 0]           pack_cnt        ;
reg  [VIRTUAL_CH_NUM - 1 : 0]           pack_num_r0     ;
reg  [$clog2(VIRTUAL_CH_NUM) - 1 : 0]   rd_count        ;
reg  [$clog2(VIRTUAL_CH_NUM) - 1 : 0]   rd_count_r0     ;
reg  [DATA_WIDTH - 1 : 0]               data_in_r0      ;
reg  [DATA_WIDTH - 1 : 0]               data_in_r1      ;
reg                                     rd_en_r0        ;
reg                                     rd_en_r1        ;                               

// reg  [VIRTUAL_CH_NUM - 1 : 0]           empty_vc_r0     ;

reg                                     cmd_en_flag     ;
reg                                     nocpack_done_r0 ;
reg                                     nocpack_done_r1 ;

// ---------------------------- combinational logic -------------------------
assign data2ddr = (axi_type == TYPE_WRITE) ? data_in_r1 : 'd0;  //当请求数据时，需要将 vc_fifo 中的数据丢弃
assign en_data2ddr = (axi_type == TYPE_WRITE) ? rd_en_r1 : 1'b0;

// rd_en
genvar k;
generate for (k=0; k<VIRTUAL_CH_NUM; k=k+1) begin
    always@(*) begin
        if(noc_rst_n == 1'b0)
            rd_en[k] <= 1'b0;
        else if ((curr_state == RD_FIFO) && ~data_fifo_full && (rd_count == k) && ~empty_vc[rd_count])
            rd_en[k] <= 1'b1;
        else
            rd_en[k] <= 1'b0;
    end
end
endgenerate
// ---------------------------- sequential logic ----------------------------
// fsm_state
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// nocpack_done delay
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0) begin
        nocpack_done_r0 <= 1'b0;
        nocpack_done_r1 <= 1'b0;
    end
    else begin
        nocpack_done_r0 <= nocpack_done;
        nocpack_done_r1 <= nocpack_done_r0;       
    end
end

// cmd2ddr
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        cmd2ddr <= 'd0;
    else if (nocpack_done_r1)
        cmd2ddr <= {axi_addr, axi_len, axi_type, source_id, re_pack};
end

// cmd_en_flag
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        cmd_en_flag <= 1'b1;
    else if (en_cmd2ddr)
        cmd_en_flag <= 1'b0;
    else if (pack_cnt == pack_num_r0 || axi_type == TYPE_RD_REQ)
        cmd_en_flag <= 1'b1;
end

// en_cmd2ddr
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        en_cmd2ddr <= 1'b0;
    else if (nocpack_done_r1 && cmd_en_flag && ~cmd_fifo_full)
        en_cmd2ddr <= 1'b1;
    else
        en_cmd2ddr <= 1'b0;
end

// nsu_busy
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        nsu_busy <= 1'b0;
    else if (pack_cnt == pack_num_r0) // TODO: busy 信号误拉高
        nsu_busy <= 1'b1;
    else if ((axi_type == TYPE_WRITE) && type_wr_done)
        nsu_busy <= 1'b0;
    else if ((axi_type == TYPE_RD_REQ) && type_rd_req_done)
        nsu_busy <= 1'b0;
end

// pack_num_r0
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        pack_num_r0 <= {VIRTUAL_CH_NUM{1'b1}};
    else if (nocpack_done_r1 == 1'b1)
        pack_num_r0 <= pack_num;
end

// rd_count
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        rd_count <= 'd0;
    else if ((curr_state == RD_FIFO) && (next_state == S_IDLE) && (rd_count == pack_num_binary))
        rd_count <= 'd0;
    else if ((curr_state == RD_FIFO) && (next_state == S_IDLE))
        rd_count <= rd_count + 1'b1;
end

// rd_count_r0
always@(posedge noc_clk) begin
    rd_count_r0 <= rd_count;
end

// pack_cnt
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        pack_cnt <= {VIRTUAL_CH_NUM{1'b0}};
    else if (((rd_count == 'd0) && (|rd_count_r0)) || ((pack_num_binary == 'd0) && (|pack_cnt)))
        pack_cnt <= {VIRTUAL_CH_NUM{1'b0}};
    else if ((nocpack_done_r1 == 1'b1) && ~(|pack_cnt))
        pack_cnt <= {pack_cnt[VIRTUAL_CH_NUM - 1 : 1], 1'b1};
    else if (nocpack_done_r1 == 1'b1)
        pack_cnt <= {pack_cnt[VIRTUAL_CH_NUM - 2 : 0], pack_cnt[VIRTUAL_CH_NUM - 1]};
end

// data_in_r
always@(posedge noc_clk) begin
    data_in_r0 <= data_in   ;
    data_in_r1 <= data_in_r0;
end

// rd_en_r
always@(posedge noc_clk) begin
    rd_en_r0 <= (|rd_en);
    rd_en_r1 <= rd_en_r0;
end

// data2ddr_last 
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        data2ddr_last <= 1'b0;
    else if (rd_en_r0 && ~(|rd_en) && (&empty_vc))
        data2ddr_last <= 1'b1;
    else
        data2ddr_last <= 1'b0;
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE: begin
            if ((nocpack_done_r1 == 1'b1) || (pack_cnt == pack_num_r0))
                next_state = RD_NUM_CHK_EMPTY;
            else
                next_state = S_IDLE; 
        end
        RD_NUM_CHK_EMPTY: begin
            if (empty_vc[rd_count] == 1'b0)
                next_state = RD_FIFO; 
            else
                next_state = S_IDLE;
        end
        RD_FIFO: begin
            if (empty_vc[rd_count] == 1'b1)
                next_state = S_IDLE; 
            else
                next_state = RD_FIFO; 
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
onehot2bin #(
    .ONE_HOT_WIDTH (VIRTUAL_CH_NUM   )
) u_onehot2bin_nsu_depacket(
    .one_hot_code  (pack_num_r0      ),
    .bin_code      (pack_num_binary  )
);

endmodule //nsu_depacket_proc
