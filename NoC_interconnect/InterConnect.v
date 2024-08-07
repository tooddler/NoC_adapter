/*
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn

    简易多主机读写 DDR 无撞包
*/

module InterConnect #(
    parameter   DATA_WIDTH          =      128              ,
    parameter   AXI_ADDR_WIDTH      =      32               ,
    parameter   ID_WIDTH            =      4                ,
    parameter   VIRTUAL_CH_NUM      =      16               ,
    parameter   PACK_ORDER_WIDTH    =      VIRTUAL_CH_NUM   ,
    parameter   NOC_MASTER_UNIT_NUM =      2                ,

    parameter   HEAD_CODE_BIT       =      4                ,  // Head flit校验字符长度
    parameter   HEAD_CODE_H         =      4'hA             ,  // Head flit头部校验字符
    parameter   HEAD_CODE_E         =      4'hB             ,  // Head flit尾部校验字符
    parameter   TAIL_CODE_BIT       =      4                ,  // Tail flit校验字符长度
    parameter   TAIL_CODE_H         =      4'hC             ,  // Tail flit头部校验字符
    parameter   TAIL_CODE_E         =      4'hD                // Tail flit尾部校验字符  
)(
    // noc domain clk
    input                                   noc_clk                 ,
    input                                   noc_rst_n               ,  

    // - interact with NOC -
    input     [DATA_WIDTH-1: 0]             noc_receive_flit        ,    
    input                                   noc_receive_valid       ,
    input                                   noc_receive_is_header   ,
    input                                   noc_receive_is_tail     ,
    output                                  noc_receive_ready       ,

    output     [DATA_WIDTH-1: 0]            noc_sender_flit         , 
    output                                  noc_sender_valid        ,
    output                                  noc_sender_is_header    ,
    output                                  noc_sender_is_tail      ,
    input                                   noc_sender_ready        ,

     // - interact with NSU -
    input     [DATA_WIDTH-1: 0]             nsu_receive_flit        ,    
    input                                   nsu_receive_valid       ,
    input                                   nsu_receive_is_header   ,
    input                                   nsu_receive_is_tail     ,
    output                                  nsu_receive_ready       ,

    output wire[DATA_WIDTH-1: 0]            nsu_sender_flit         , 
    output wire                             nsu_sender_valid        ,
    output wire                             nsu_sender_is_header    ,
    output wire                             nsu_sender_is_tail      ,
    input                                   nsu_sender_ready          
);

// ----- localparam -----
localparam  S_IDLE      = 3'b001,
            SELECT_FIFO = 3'b010,
            READ_FIFO   = 3'b100;

// ----- wire -----
wire    [ID_WIDTH - 1 : 0]                       SourceID               ;
wire                                             fifo1_full             ;
wire                                             fifo2_full             ;
wire                                             fifo1_empty            ;
wire                                             fifo2_empty            ;
wire    [NOC_MASTER_UNIT_NUM -1 : 0]             Empty_fifo             ;
wire                                             PosedgeNsuReady        ;
wire    [DATA_WIDTH+1: 0]                        Fifo_out1              ;  
wire    [DATA_WIDTH+1: 0]                        Fifo_out2              ;  
wire    [3:0]                                    code_h                 ;
wire    [3:0]                                    code_e                 ;
wire                                             s_valid                ;

// ----- reg -----
reg     [DATA_WIDTH-1: 0]                        ReceiveFlit            ;
reg                                              ReceiveHead            ;
reg                                              ReceiveTail            ;
reg                                              ReceiveValid           ;

reg     [NOC_MASTER_UNIT_NUM -1 : 0]             WrEnFlag               ;
reg     [NOC_MASTER_UNIT_NUM -1 : 0]             RdEnFlag               ;         
reg                                              ArbFlag                ;

reg     [2:0]                                    curr_state             ;
reg     [2:0]                                    next_state             ;
reg                                              nsu_sender_ready_r0    ;


// ---------------------------- combinational logic -------------------------
// 输出直连
assign noc_sender_flit            =        nsu_receive_flit      ;
assign noc_sender_valid           =        nsu_receive_valid     ;
assign noc_sender_is_header       =        nsu_receive_is_header ;
assign noc_sender_is_tail         =        nsu_receive_is_tail   ;
assign nsu_receive_ready          =        noc_sender_ready      ;

// - flit info get - 
assign code_h                     =        noc_receive_valid ? noc_receive_flit[DATA_WIDTH - 1 : DATA_WIDTH - HEAD_CODE_BIT] : 'd0;
assign code_e                     =        noc_receive_valid ? noc_receive_flit[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 3 - PACK_ORDER_WIDTH - 8 - AXI_ADDR_WIDTH : DATA_WIDTH - 3 - 2*HEAD_CODE_BIT - 2*ID_WIDTH - PACK_ORDER_WIDTH - 8 - AXI_ADDR_WIDTH] : 'd0;
assign SourceID                   =        noc_receive_flit[DATA_WIDTH - 1 - HEAD_CODE_BIT : DATA_WIDTH - HEAD_CODE_BIT - ID_WIDTH];
assign noc_receive_ready          =        ~(fifo1_full & fifo2_full);
assign Empty_fifo                 =        {fifo2_empty, fifo1_empty};

assign PosedgeNsuReady            =        nsu_sender_ready & (~nsu_sender_ready_r0);

assign s_valid                    =        ~Empty_fifo[ArbFlag] && curr_state == READ_FIFO;
assign nsu_sender_valid           =        RdEnFlag[ArbFlag] & s_valid;
assign nsu_sender_flit            =        ArbFlag ? Fifo_out2[DATA_WIDTH + 1: 2] : Fifo_out1[DATA_WIDTH + 1: 2];
assign nsu_sender_is_header       =        ArbFlag ? Fifo_out2[1] : Fifo_out1[1];
assign nsu_sender_is_tail         =        ArbFlag ? Fifo_out2[0] : Fifo_out1[0];

// ---------------------------- sequential logic ----------------------------
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// ArbFlag
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        ArbFlag <= 1'b0;
    else if (curr_state[0] == 1'b0 && next_state == S_IDLE)
        ArbFlag <= ~ArbFlag;
end

// ReceiveFlit
always@(posedge noc_clk) begin
    ReceiveFlit  <= noc_receive_flit     ;
    ReceiveHead  <= noc_receive_is_header;
    ReceiveTail  <= noc_receive_is_tail  ;
    ReceiveValid <= noc_receive_valid    ;
end

// nsu_sender_ready_r0
always@(posedge noc_clk) begin
    nsu_sender_ready_r0 <= nsu_sender_ready;
end

// WrEnFlag
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        WrEnFlag <= 2'b00;
    else if (code_h == HEAD_CODE_H && code_e == HEAD_CODE_E && noc_receive_is_header)
        case(SourceID)
            2'b00:      WrEnFlag <= 2'b01;
            2'b01:      WrEnFlag <= 2'b10;
            default:    WrEnFlag <= 2'b00;
        endcase
end

// RdEnFlag
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        RdEnFlag <= 'd0;
    else if (curr_state == READ_FIFO)
        case(ArbFlag)
            1'b0:       RdEnFlag <= 2'b01;
            1'b1:       RdEnFlag <= 2'b10;
            default:    RdEnFlag <= 2'b00;
        endcase
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE     : begin
            if (~(&Empty_fifo) && nsu_sender_ready)
                next_state = SELECT_FIFO; 
            else
                next_state = S_IDLE; 
        end
        SELECT_FIFO : begin
            if (Empty_fifo[ArbFlag] == 1'b0)
                next_state =  READ_FIFO;
            else
                next_state =  S_IDLE;
        end
        READ_FIFO: begin
            if (PosedgeNsuReady)
                next_state =  S_IDLE;
            else
                next_state =  READ_FIFO;
        end
        default: next_state = S_IDLE;
    endcase
end

// ----------------------------  instantiation  ----------------------------
NoC_InterConnect1 u_NoC_InterConnect1 (
  .clk          (noc_clk        ), 
  .srst         (~noc_rst_n     ), 
  .din          ({ReceiveFlit, ReceiveHead, ReceiveTail} ), 
  .wr_en        (WrEnFlag[0] & ReceiveValid              ), 
  .rd_en        (RdEnFlag[0] & s_valid                   ), 
  .dout         (Fifo_out1                               ), 
  .full         (fifo1_full     ), 
  .empty        (fifo1_empty    )  
);

NoC_InterConnect2 u_NoC_InterConnect2 (
  .clk          (noc_clk        ), 
  .srst         (~noc_rst_n     ), 
  .din          ({ReceiveFlit, ReceiveHead, ReceiveTail} ), 
  .wr_en        (WrEnFlag[0] & ReceiveValid              ), 
  .rd_en        (RdEnFlag[1] & s_valid                   ), 
  .dout         (Fifo_out2                               ), 
  .full         (fifo2_full     ), 
  .empty        (fifo2_empty    )  
);

endmodule //InterConnect
