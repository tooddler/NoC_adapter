/* 
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn

    - for Debug -
    
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
    parameter   FLIT_NUM_MAX        =      16               ,  // {head + 16*body + tail}

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

localparam  cnt_max             =   18    ,
            flag_init_cnt_max   =   30    ;

wire        [7:0]                            axi_lens        ;
wire        [AXI_ADDR_WIDTH - 1 : 0]         axi_addr        ;
wire        [AXI_ADDR_WIDTH - 1 : 0]         re_pack         ;

wire        [VIRTUAL_CH_NUM - 1 : 0]         pack_num        ;
wire                                         busy_negedge    ;

reg         [VIRTUAL_CH_NUM - 1 : 0]         pack_order      ;

reg         [3:0]                            reset_timer     ;
reg                                          reset_sync      ;
reg         [4:0]                            cnt             ;
reg         [4:0]                            cnt_init        ;

reg                                          wr_send_done    ;
reg         [DATA_WIDTH - 1 : 0]             data            ;
reg                                          nsu_busy_r0     ;
reg         [4:0]                            send_rd_req_cnt ;
reg                                          flag            ;
reg         [4:0]                            flag_init_cnt   ;

assign      axi_lens            =     8'h29                  ;  // 另一个 8'h55
assign      axi_addr            =     32'h0000_2000          ;  // 起始地址为 32'h0000_2aa0 这是第二包
assign      re_pack             =     {16'h0040, 8'h7f}      ;  // {basepack_order, total_lens} = {16'h0040, 8'h7f}
assign      pack_num            =     16'h0004               ;  // 当前需要三包
assign      busy_negedge        =     nsu_busy_r0 & nsu_busy ;

always@(posedge noc_clk) begin
    nsu_busy_r0 <= nsu_busy;
end 

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

// flag_init_cnt
always@(posedge noc_clk) begin
    if (reset_sync)
        flag_init_cnt <= 'd0;
    else if (flag_init_cnt == flag_init_cnt_max)
        flag_init_cnt <= flag_init_cnt;
    else if (flag)
        flag_init_cnt <= flag_init_cnt + 1'b1;
end

always@(posedge noc_clk) begin
    if (reset_sync)
        cnt_init <= 'd0;
    else if (cnt_init == cnt_max)
        cnt_init <= cnt_init;
    else
        cnt_init <= cnt_init + 1'b1;
end

always@(posedge noc_clk) begin
    if (reset_sync)
        pack_order <= {{(VIRTUAL_CH_NUM-1){1'b0}}, 1'b1};
    else if (wr_send_done && pack_order != pack_num)
        pack_order <= {pack_order[VIRTUAL_CH_NUM-2:0], pack_order[VIRTUAL_CH_NUM-1]};
end

// noc2axi_data s_is_tail s_is_head wr_send_done
always@(posedge noc_clk) begin
    if (reset_sync) begin
        data         <= 'd1;
        noc2axi_data <= 'd0 ;
        s_is_head    <= 1'b0;
        s_is_tail    <= 1'b0;
        wr_send_done <= 1'b0; 
        cnt          <= 'd0;
        flag         <= 1'b0;
        send_rd_req_cnt <= 'd0;
    end
    else if (flag && ~nsu_busy && flag_init_cnt == flag_init_cnt_max) begin
        case(send_rd_req_cnt)
            0: begin
                noc2axi_data <= {1'b1
                                , HEAD_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_RD_REQ, {{(VIRTUAL_CH_NUM-1){1'b0}},1'b1}, axi_lens, axi_addr, HEAD_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b1;
                s_is_tail    <= 1'b0;
                send_rd_req_cnt <=  send_rd_req_cnt + 1'b1;
            end
            1: begin
                noc2axi_data <= {1'b1, {(128){1'b1}}};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b0;
                send_rd_req_cnt <=  send_rd_req_cnt + 1'b1;
            end
            2: begin
                noc2axi_data <= {1'b1
                                , TAIL_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_RD_REQ, {{(VIRTUAL_CH_NUM-1){1'b0}},1'b1}, axi_lens, re_pack, TAIL_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b1;
                send_rd_req_cnt <=  send_rd_req_cnt + 1'b1;
            end 
            default: begin 
                noc2axi_data <= 'd0 ;
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b0;
                send_rd_req_cnt <= send_rd_req_cnt; 
            end
        endcase
    end
    else if (~nsu_busy && pack_order != pack_num && cnt_init == cnt_max) begin
        case(cnt)
            0: begin
                noc2axi_data <= {1'b1
                                , HEAD_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_WRITE, pack_order, axi_lens, axi_addr, HEAD_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b1;
                s_is_tail    <= 1'b0;
                wr_send_done <= 1'b0;
                cnt          <= cnt + 'd1; 
            end
            FLIT_NUM_MAX + 'd1: begin
                noc2axi_data <= {1'b1
                                , TAIL_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_WRITE, pack_num, axi_lens, re_pack, TAIL_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b1;
                wr_send_done <= 1'b0;
                cnt          <= 'd0; 
            end 
            default: begin 
                if (cnt <= FLIT_NUM_MAX) begin
                    data         <= data + 'd1;
                    cnt          <= cnt + 'd1; 
                    noc2axi_data <= {1'b1, data};
                    s_is_head    <= 1'b0;
                    s_is_tail    <= 1'b0;
                    if (cnt == FLIT_NUM_MAX)
                        wr_send_done <= 1'b1; 
                end
                else begin
                    cnt          <= cnt; 
                    data         <= data;
                    noc2axi_data <= 'd0;
                    s_is_head    <= 1'b0;
                    s_is_tail    <= 1'b0;
                    wr_send_done <= 1'b0;              
                end
            end
        endcase
    end
    else if (~nsu_busy && pack_order == pack_num && cnt_init == cnt_max) begin
        if (cnt == 'd0) begin
            noc2axi_data <= {1'b1
                            , HEAD_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                            , TYPE_WRITE, pack_order, axi_lens, axi_addr, HEAD_CODE_E
                            , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
            s_is_head    <= 1'b1;
            s_is_tail    <= 1'b0;
            wr_send_done <= 1'b0;
            cnt          <= cnt + 'd1; 
        end
        else if (cnt == axi_lens[3:0] + 'd2) begin
                noc2axi_data <= {1'b1
                                , TAIL_CODE_H, {(ID_WIDTH){1'b0}}, {(ID_WIDTH){1'b1}}
                                , TYPE_WRITE, pack_num, axi_lens, re_pack, TAIL_CODE_E
                                , {(DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - VIRTUAL_CH_NUM - 11 - AXI_ADDR_WIDTH){1'b0}}};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b1;
                wr_send_done <= 1'b0;
                cnt          <= cnt + 'd1; 
                flag         <= 1'b1;   
        end
        else begin
            if (cnt <= axi_lens[3:0] + 'd1 && cnt_init == cnt_max) begin
                data         <= data + 'd1;
                cnt          <= cnt + 'd1; 
                noc2axi_data <= {1'b1, data};
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b0;
                if (cnt == FLIT_NUM_MAX)
                    wr_send_done <= 1'b1; 
            end
            else begin
                cnt          <= cnt; 
                data         <= data;
                noc2axi_data <= 'd0;
                s_is_head    <= 1'b0;
                s_is_tail    <= 1'b0;
                wr_send_done <= 1'b0;              
            end
        end    
    end
end

endmodule
