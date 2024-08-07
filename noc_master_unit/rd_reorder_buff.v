/* 
    - read reorder buffer -
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn

    数据格式如下:
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, reserved(32 + 8 bit), HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM,   reserved(32 + 8 bit), TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]                                                                             DATA_WIDTH Bits
    
    PACK_ORDER setting :
        e.g. axi_master 请求 brust 256 个数据, 对于 Psoc PL 端 DDR 即需要 256*128 数据量, 
             noc包中 filt 为 DATA_WIDTH, 最多含 16 个 filt, 即 noc一包可以有 DATA_WIDTH*16.
             需要 16 个 noc 包
             利用 PACK_ORDER 来判断, 当前事务总数 PACK_NUM
             在该模块进行重新排序 方便组包
             <pack_order> : use one-hot encoding
    
    [2:0] axi_type :
        write_req and data  ;  rd data ; bresponse 
*/

module rd_reorder_buff #(
    parameter   DATA_WIDTH          =      128              ,
    parameter   AXI_ADDR_WIDTH      =      32               ,
    parameter   FLIT_NUM_MAX        =      16               ,  // {head + 16*body + tail}
    parameter   BUFFER_DEPTH        =      FLIT_NUM_MAX+2   ,
    parameter   ID_WIDTH            =      4                ,
    parameter   VIRTUAL_CH_NUM      =      16               ,
    parameter   PACK_ORDER_WIDTH    =      VIRTUAL_CH_NUM   ,
    parameter   STAGES_NUM          = $clog2(VIRTUAL_CH_NUM),

    parameter   HEAD_CODE_BIT       =      4                ,  // Head flit校验字符长度
    parameter   HEAD_CODE_H         =      4'hA             ,  // Head flit头部校验字符
    parameter   HEAD_CODE_E         =      4'hB             ,  // Head flit尾部校验字符
    parameter   TAIL_CODE_BIT       =      4                ,  // Tail flit校验字符长度
    parameter   TAIL_CODE_H         =      4'hC             ,  // Tail flit头部校验字符
    parameter   TAIL_CODE_E         =      4'hD                // Tail flit尾部校验字符  
)(
    // user domain clk
    input                                   axi_clk       ,
    input                                   axi_rst_n     ,
    // from cdc module recive part
    input                                   buffer_empty  ,
    output wire                             rd_data_en    , 
    input      [DATA_WIDTH -1 : 0]          rd_data       ,
    input                                   rd_head       ,
    input                                   rd_tail       ,   
    // interact with packet_proc module
    output reg [2:0]                        axi_type      ,
    output reg [ID_WIDTH - 1 : 0]           dest_id       ,
    input      [VIRTUAL_CH_NUM - 1 : 0]     rd_en         ,            
    output wire[DATA_WIDTH - 1 : 0]         data_out      ,
    output wire[VIRTUAL_CH_NUM - 1 : 0]     empty_vc      ,
    output reg [VIRTUAL_CH_NUM - 1 : 0]     pack_num      ,
    output reg                              nocpack_done  
);

// ----- localparam -----
localparam  S_IDLE      = 4'b0001,
            CHECK_HEAD  = 4'b0010,
            SELECT_VC   = 4'b0100,
            CHECK_TAIL  = 4'b1000;

// ----- wire -----
wire [VIRTUAL_CH_NUM - 1 : 0]           wr_en           ;
wire [VIRTUAL_CH_NUM - 1 : 0]           full            ;

wire [3:0]                              code_h          ;
wire [3:0]                              code_e          ;

wire [DATA_WIDTH - 1 : 0]               data_fifo_out [VIRTUAL_CH_NUM - 1 : 0] ;
wire [DATA_WIDTH - 1 : 0]               data_tree     [VIRTUAL_CH_NUM - 1 : 0] ;
// ----- reg -----
reg  [3:0]                              curr_state      ;
reg  [3:0]                              next_state      ;

reg  [DATA_WIDTH - 1 : 0]               data_in         ;
reg  [VIRTUAL_CH_NUM - 1 : 0]           pack_order      ;

// ---------------------------- combinational logic -------------------------
genvar stage;
generate 
    for(stage = 0; stage < VIRTUAL_CH_NUM; stage=stage+1) begin: stage_gen
        if (stage == 0) begin
            assign data_tree[stage] = (rd_en[stage]== 1'b1) ? data_fifo_out[stage] : 'd0 ;
        end 
        else begin
            assign data_tree[stage] = (rd_en[stage]== 1'b1) ? data_fifo_out[stage] : data_tree[stage-1];
        end
    end
endgenerate
assign data_out = data_tree[VIRTUAL_CH_NUM - 1];

// - flit head or tail get - 
assign code_h  = (rd_data_en == 1'b1) ? rd_data[DATA_WIDTH - 1 : DATA_WIDTH - HEAD_CODE_BIT] : 'd0;
assign code_e  = (rd_data_en == 1'b1) ? rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 - PACK_ORDER_WIDTH - 8 - AXI_ADDR_WIDTH : DATA_WIDTH - 3 - 2*HEAD_CODE_BIT - 2*ID_WIDTH - PACK_ORDER_WIDTH - 8 - AXI_ADDR_WIDTH] : 'd0;
assign wr_en   = (curr_state == CHECK_TAIL) ? pack_order & {VIRTUAL_CH_NUM{rd_data_en}} : 1'b0 ;
assign rd_data_en = ((curr_state != S_IDLE) && (buffer_empty == 1'b0)) ? 1'b1 : 1'b0 ;

// ---------------------------- sequential logic ----------------------------
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// data_in
always@(posedge axi_clk or negedge axi_rst_n) begin
    if (axi_rst_n == 1'b0)
        data_in <= {DATA_WIDTH{1'b0}};
    else if (rd_data_en == 1'b1)
        data_in <= rd_data[DATA_WIDTH - 1 : 0];
end

// - latch dest ID : dest_id - 
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        dest_id <= {ID_WIDTH{1'b0}};
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && rd_head)
        dest_id <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - ID_WIDTH : DATA_WIDTH - HEAD_CODE_BIT - 2*ID_WIDTH];
end

// - latch axi_pack order : pack_order - 
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        pack_order <= {VIRTUAL_CH_NUM{1'b0}};
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && rd_head)
        pack_order <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM];
end

// - latch order num : pack_num - 
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        pack_num <= {VIRTUAL_CH_NUM{1'b0}};
    else if ((curr_state == CHECK_TAIL) && (code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && rd_tail)
        pack_num <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM];
end

// nocpack_done
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        nocpack_done <= 1'b0;
    else if ((curr_state == CHECK_TAIL) && (code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && rd_tail)
        nocpack_done <= 1'b1;
    else
        nocpack_done <= 1'b0;
end

// axi_type
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        axi_type <= 'd0;
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && rd_head)
        axi_type <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH : DATA_WIDTH - HEAD_CODE_BIT - 2*ID_WIDTH - 3];
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE     : begin
            if (buffer_empty == 1'b0)
                next_state = CHECK_HEAD ;
            else
                next_state = S_IDLE; 
        end
        CHECK_HEAD : begin
            if ((code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && rd_head)
                next_state = SELECT_VC  ; 
            else
                next_state = CHECK_HEAD ;
        end
        SELECT_VC  :begin
            next_state = CHECK_TAIL ; 
        end
        CHECK_TAIL:begin
            if ((code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && rd_tail)
                next_state = S_IDLE ; 
            else
                next_state = CHECK_TAIL ;
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
// - virtual_channel -
genvar i;
generate for (i=0; i<VIRTUAL_CH_NUM;i=i+1) begin : virtual_channel
    syn_fifo u_syn_fifo (
        .clk        (axi_clk         ),  // input wire clk
        .srst       (~axi_rst_n      ),  // input wire srst
        .din        (data_in         ),  // input wire [127 : 0] din
        .wr_en      (wr_en[i]        ),  // input wire wr_en
        .rd_en      (rd_en[i]        ),  // input wire rd_en
        .dout       (data_fifo_out[i]),  // output wire [127 : 0] dout
        .full       (full[i]         ),  // output wire full
        .empty      (empty_vc[i]     )   // output wire empty
    );
end
endgenerate 


endmodule //rd_reorder_buff
