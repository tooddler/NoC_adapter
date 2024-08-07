/*  
    - NOC SLAVE UNIT REORDER MODULE -  
    
    对于不同路由算法，可能会导致由主机发送 noc包 顺序颠倒，设计了重新排序模块。同发送端。
    
    NMU传来数据格式如下：
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, AXI_LEN(8bit), AXI_ADDR, HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , AXI_LEN(8bit), RE_PACK, TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]
*/

module nsu_reorder_module #(
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
    // noc domain clk
    input                                   noc_clk       ,
    input                                   noc_rst_n     ,
    // - interact with NOC -
    input      [DATA_WIDTH : 0]             noc2axi_data  ,
    input                                   s_is_head     ,
    input                                   s_is_tail     ,
    input  wire                             nsu_busy      ,
    // use for Packetizing
    output reg [AXI_ADDR_WIDTH - 1 : 0]     re_pack       , // {basepack_order, dest_pack_num_onehot}
    
    // interact with depacket_proc module
    output reg [AXI_ADDR_WIDTH - 1 : 0]     axi_addr      ,
    output reg [7:0]                        axi_len       ,
    output reg [2:0]                        axi_type      ,
    output reg [ID_WIDTH - 1 : 0]           source_id     ,
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
wire [DATA_WIDTH - 1 : 0]               rd_data         ;

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
assign rd_data = noc2axi_data[DATA_WIDTH - 1 : 0];
assign rd_data_en = noc2axi_data[DATA_WIDTH] && ~nsu_busy;

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

// ---------------------------- sequential logic ----------------------------
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// data_in
always@(posedge noc_clk or negedge noc_rst_n) begin
    if (noc_rst_n == 1'b0)
        data_in <= {DATA_WIDTH{1'b0}};
    else if (rd_data_en == 1'b1)
        data_in <= rd_data[DATA_WIDTH - 1 : 0];
end

// - latch axi_addr : axi_addr - 
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        axi_addr <= 'd0;
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && s_is_head)
        axi_addr <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 - VIRTUAL_CH_NUM - 8 : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 8 - AXI_ADDR_WIDTH];
end

// - latch axi_len : axi_len - 
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        axi_len <= 'd0;
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && s_is_head)
        axi_len <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 - VIRTUAL_CH_NUM : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 8];
end

// - latch source ID : source_id - 
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        source_id <= {ID_WIDTH{1'b0}};
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && s_is_head)
        source_id <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT : DATA_WIDTH - HEAD_CODE_BIT - ID_WIDTH];
end

// - latch axi_pack order : pack_order - 
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        pack_order <= {VIRTUAL_CH_NUM{1'b0}};
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && s_is_head)
        pack_order <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM];
end

// - latch order num : pack_num - 
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        pack_num <= {VIRTUAL_CH_NUM{1'b0}};
    else if ((curr_state == CHECK_TAIL) && (code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && s_is_tail)
        pack_num <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM];
end

// nocpack_done
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        nocpack_done <= 1'b0;
    else if ((curr_state == CHECK_TAIL) && (code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && s_is_tail)
        nocpack_done <= 1'b1;
    else
        nocpack_done <= 1'b0;
end

// axi_type
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        axi_type <= 'd0;
    else if ((curr_state == CHECK_HEAD) && (code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && s_is_head)
        axi_type <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH : DATA_WIDTH - HEAD_CODE_BIT - 2*ID_WIDTH - 3];
end

// re_pack
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        re_pack <= {AXI_ADDR_WIDTH{1'b0}};
    else if ((curr_state == CHECK_TAIL) && (code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && s_is_tail)
        re_pack <= rd_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 - VIRTUAL_CH_NUM - 8 : DATA_WIDTH - 3 - HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 8 - AXI_ADDR_WIDTH];
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE     : begin
            next_state = CHECK_HEAD; 
        end
        CHECK_HEAD : begin
            if ((code_h == HEAD_CODE_H) && (code_e == HEAD_CODE_E) && s_is_head)
                next_state = SELECT_VC  ; 
            else
                next_state = CHECK_HEAD ;
        end
        SELECT_VC  :begin
            next_state = CHECK_TAIL ; 
        end
        CHECK_TAIL:begin
            if ((code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && s_is_tail)
                next_state = CHECK_HEAD; 
            else
                next_state = CHECK_TAIL;
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
// - virtual_channel -
genvar i;
generate for (i=0; i<VIRTUAL_CH_NUM; i=i+1) begin : NSU_VC
    syn_fifo u_syn_fifo (
        .clk        (noc_clk         ),  // input wire clk
        .srst       (~noc_rst_n      ),  // input wire srst
        .din        (data_in         ),  // input wire [127 : 0] din
        .wr_en      (wr_en[i]        ),  // input wire wr_en
        .rd_en      (rd_en[i]        ),  // input wire rd_en
        .dout       (data_fifo_out[i]),  // output wire [127 : 0] dout
        .full       (full[i]         ),  // output wire full
        .empty      (empty_vc[i]     )   // output wire empty
    );
end
endgenerate 

endmodule //nsu_reorder_module
