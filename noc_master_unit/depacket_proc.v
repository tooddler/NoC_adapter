/*  
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn
    
    - Depacketizing NoC Packet Protocol(NPP) -
    
    type :
        读数据：3'b100 ;  读返回数据 ： 3'b001 ; 写请求 ：3'b010 ; 写响应: 3'b011
    
    s_axi_bresp :
        放在 noc_data 中的低两位 即 noc_data[1:0]

    这个主机的 ready 信号要等待你先拉高 valid 信号 保持一个时钟才拉高 离谱了
*/

module depacket_proc #(
    parameter   DATA_WIDTH          =      128            ,
    parameter   FIFO_ADDR_WIDTH     =      16             ,
    parameter   ID_WIDTH            =      4              ,
    parameter   VIRTUAL_CH_NUM      =      16             ,
    parameter   AXI_ID_WIDTH        =      4              ,
    parameter   AXI_DATA_WIDTH      =      DATA_WIDTH     ,
    parameter   TYPE_BRESP          =      3'b011         ,
    parameter   TYPE_RD_DATA        =      3'b001  
)(
    // user domain clk
    input                                       axi_clk       ,
    input                                       axi_rst_n     ,
    // Depacketizing port :  interact with rd_reorder_buff module
    input       [2:0]                           axi_type      ,
    input       [ID_WIDTH - 1 : 0]              dest_id       ,
    output reg  [VIRTUAL_CH_NUM - 1 : 0]        rd_en         ,            
    input       [DATA_WIDTH - 1 : 0]            data_in       ,
    input       [VIRTUAL_CH_NUM - 1 : 0]        empty_vc      ,
    input       [VIRTUAL_CH_NUM - 1 : 0]        pack_num      ,  
    input                                       nocpack_done  ,
    //generate rlast
    input       [7:0]                           s_axi_arlen   ,
    input                                       s_axi_arvalid ,
    input                                       s_axi_arready ,
    // Packetizing port :  interact with axi module
     // Slave Interface Write Response Port
    output wire [AXI_ID_WIDTH - 1:0]            s_axi_bid     ,
    output wire [1:0]                           s_axi_bresp   ,
    output reg                                  s_axi_bvalid  ,
    input                                       s_axi_bready  ,
     // Slave Interface Read Data Ports
    output wire [AXI_ID_WIDTH - 1:0]            s_axi_rid     ,
    output      [AXI_DATA_WIDTH - 1:0]          s_axi_rdata   ,
    output      [1:0]                           s_axi_rresp   ,
    output reg                                  s_axi_rlast   ,
    output wire                                 s_axi_rvalid  ,
    input                                       s_axi_rready  
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

reg                                     en_r0           ;
reg                                     en_r1           ;
reg                                     nocpack_done_r0 ;
reg                                     nocpack_done_r1 ;

reg  [7:0]                              total_lens      ;
reg  [7:0]                              len_cnt         ;

// ---------------------------- combinational logic -------------------------
// - axi brsp -
assign s_axi_bid   = dest_id;
assign s_axi_bresp = (axi_type == TYPE_BRESP) ? data_in_r1[1:0] : 3'b0;
// - axi rd -
assign s_axi_rid   = dest_id;
assign s_axi_rdata = (axi_type == TYPE_BRESP) ? 'd0 : data_in;
assign s_axi_rresp = 2'b00;

// rd_en 
genvar k;
generate for (k=0;k<VIRTUAL_CH_NUM;k=k+1) begin
    always@(*) begin
        if(axi_rst_n == 1'b0)
            rd_en[k] <= 1'b0;
        else if ((curr_state == RD_NUM_CHK_EMPTY) && (next_state == RD_FIFO) && (rd_count == k) && (axi_type == TYPE_BRESP))
            rd_en[k] <= 1'b1;
        else if ((curr_state == RD_FIFO) && (rd_count == k) && ~empty_vc[rd_count])
            rd_en[k] <= 1'b1;
        else
            rd_en[k] <= 1'b0;
    end
end
endgenerate

assign s_axi_rvalid = (axi_type == TYPE_BRESP) ? 1'b0 : |rd_en;

// ---------------------------- sequential logic ----------------------------
// fsm_state
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// nocpack_done delay
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0) begin
        nocpack_done_r0 <= 1'b0;
        nocpack_done_r1 <= 1'b0;
    end
    else begin
        nocpack_done_r0 <= nocpack_done;
        nocpack_done_r1 <= nocpack_done_r0;       
    end
end

// total_lens
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        total_lens <= 'd0;
    else if (s_axi_arvalid & s_axi_arready)
        total_lens <= s_axi_arlen;
end

// len_cnt
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        len_cnt <= 'd0;
    else if (s_axi_rlast && s_axi_rready && s_axi_rvalid)
        len_cnt <= 'd0;
    else if (|rd_en && (axi_type == TYPE_RD_DATA))
        len_cnt <= len_cnt + 1'b1;
end

// s_axi_rlast
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        s_axi_rlast <= 'd0;
    else if (s_axi_rlast && s_axi_rready && s_axi_rvalid)
        s_axi_rlast <= 'd0;
    else if (len_cnt == total_lens - 1'b1 && axi_type == TYPE_RD_DATA)
        s_axi_rlast <= 1'b1;
end

// pack_num_r0
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        pack_num_r0 <= {VIRTUAL_CH_NUM{1'b1}};
    else if (nocpack_done_r1 == 1'b1)
        pack_num_r0 <= pack_num;
end

// rd_count
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        rd_count <= 'd0;
    else if ((curr_state == RD_FIFO) && (next_state == S_IDLE) && (rd_count == pack_num_binary))
        rd_count <= 'd0;  //todo 此时一次 noc 传输完成
    else if ((curr_state == RD_FIFO) && (next_state == S_IDLE))
        rd_count <= rd_count + 1'b1;
end

// rd_count_r0
always@(posedge axi_clk) begin
    rd_count_r0 <= rd_count;
end

// pack_cnt
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        pack_cnt <= {VIRTUAL_CH_NUM{1'b0}};
    else if (((rd_count == 'd0) && (|rd_count_r0)) || ((pack_num_binary == 'd0) && (|pack_cnt)))
        pack_cnt <= {VIRTUAL_CH_NUM{1'b0}};
    else if ((nocpack_done_r1 == 1'b1) && ~(|pack_cnt))
        pack_cnt <= {pack_cnt[VIRTUAL_CH_NUM - 1 : 1], 1'b1};
    else if (nocpack_done_r1 == 1'b1)
        pack_cnt <= {pack_cnt[VIRTUAL_CH_NUM - 2 : 0], pack_cnt[VIRTUAL_CH_NUM - 1]};
end

// en_r0
always@(posedge axi_clk) begin    
    en_r0 <= |rd_en;
    en_r1 <= en_r0 ;
end

// data_in_r0
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0) begin
        data_in_r0 <= 'd0;
        data_in_r1 <= 'd0;
    end
    else if ((|rd_en) && (|data_in[1:0]) && axi_type == TYPE_BRESP) begin
        data_in_r0 <= 'd0;   
        data_in_r1 <= 2'b11;
    end
    // else if (axi_type == TYPE_RD_DATA) begin
    //     data_in_r0 <= data_in;
    //     data_in_r1 <= data_in_r0;
    // end
end
 
// s_axi_bvalid --> only one clk
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        s_axi_bvalid <= 1'b0;
    else if (s_axi_bready && s_axi_bvalid)
        s_axi_bvalid <= 1'b0;
    else if (en_r0 && axi_type == TYPE_BRESP && (&empty_vc) && (pack_num_r0 == pack_cnt || pack_num_r0[0] == 1'b1))
        s_axi_bvalid <= 1'b1;
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
) u_onehot2bin(
    .one_hot_code  (pack_num_r0      ),
    .bin_code      (pack_num_binary  )
);


endmodule //packet_proc
