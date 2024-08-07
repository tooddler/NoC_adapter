/* 
    NMU传来数据格式如下：
        Head flit: [HEAD_CODE_H, Source_ID, Dest_ID, TYPE, PACK_ORDER, AXI_LEN(8bit), AXI_ADDR, HEAD_CODE_E]    DATA_WIDTH Bits
        Tail flit: [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , AXI_LEN(8bit), RE_PACK, TAIL_CODE_E]    DATA_WIDTH Bits
        Data flit: [DATA]
*/

module noc_gen #(
    parameter   DATA_WIDTH          =      128              ,
    parameter   ID_WIDTH            =      4                ,
    parameter   VIRTUAL_CH_NUM      =      16               ,
    parameter   AXI_ADDR_WIDTH      =      32               ,

    parameter   TYPE_WRITE          =      3'b100           ,
    parameter   TYPE_RD_REQ         =      3'b010           ,
    parameter   TYPE_BRESP          =      3'b011           ,
    parameter   TYPE_RD_DATA        =      3'b001           ,

    parameter   HEAD_CODE_BIT       =      4                ,
    parameter   HEAD_CODE_H         =      4'h5             ,
    parameter   HEAD_CODE_E         =      4'hA             ,
    parameter   TAIL_CODE_BIT       =      4                ,
    parameter   TAIL_CODE_H         =      4'h0             ,
    parameter   TAIL_CODE_E         =      4'hF               
)(
    input                                     noc_clk       ,
    input                                     noc_rst       ,

    output reg [DATA_WIDTH : 0]               noc2axi_data  ,
    output reg                                s_is_head     ,
    output reg                                s_is_tail     ,
    input                                     nsu_busy      ,

    input                                     ddr_init_done 
);

localparam  cnt_max     =   4;

wire        [7:0]                            axi_lens      ;
wire        [AXI_ADDR_WIDTH - 1 : 0]         axi_addr      ;
wire        [AXI_ADDR_WIDTH - 1 : 0]         re_pack       ;

reg         [3:0]                            reset_timer   ;
reg                                          reset_sync    ;
reg         [3:0]                            cnt           ;

assign      axi_lens            =     8'h29                ;  // 另一个 8'h55
assign      axi_addr            =     32'h0000_2000        ;  // 起始地址为 32'h0000_2aa0 这是第二包
assign      re_pack             =     {16'h0040, 8'h7f}    ;  // {basepack_order, total_lens} = {16'h0040, 8'h7f}

always@(posedge noc_clk) begin
    if (noc_rst) begin
        reset_timer <= 12'h0;
        reset_sync  <= 1'b1;
    end
    else if (reset_timer <= 'd10) begin
        reset_timer <= reset_timer + 1;
        reset_sync  <= 1'b1;
    end
    else if (ddr_init_done) begin
        reset_timer <= reset_timer;
        reset_sync  <= 1'b0;
    end
end

always@(posedge noc_clk) begin
    if (reset_sync)
        cnt <= 'd0;
    else if (cnt == cnt_max)
        cnt <= cnt;
    else
        cnt <= cnt + 1'b1;
end

// noc2axi_data s_is_tail s_is_head
always@(posedge noc_clk) begin
    if (reset_sync) begin
        noc2axi_data <= 'd0 ;
        s_is_head    <= 1'b0;
        s_is_tail    <= 1'b0;   
    end
    else if (~nsu_busy) begin
        case(cnt)
            1: begin
                noc2axi_data <= {1'b1
                                , HEAD_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_RD_REQ, {{(VIRTUAL_CH_NUM-1){1'b0}},1'b1}, axi_lens, axi_addr, HEAD_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b1;
                s_is_tail    <= 1'b0; 
            end
            2: begin
                noc2axi_data <= {1'b1, {(128){1'b1}}};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b0; 
            end
            3: begin
                noc2axi_data <= {1'b1
                                , TAIL_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_RD_REQ, {{(VIRTUAL_CH_NUM-1){1'b0}},1'b1}, axi_lens, re_pack, TAIL_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b1; 
            end 
            default: begin 
                noc2axi_data <= 'd0 ;
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b0; 
            end
        endcase
    end
end

endmodule
