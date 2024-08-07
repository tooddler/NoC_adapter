// Author : ToOddler 
// Email  : 23011211185@stu.xidian.edu.cn
// - rate matching and asynchronous data boundary crossing -
// - gen noc pack -

// TODO: NSU_READY?
module nsu_asyn_cross_axi2noc #(
    parameter   DATA_WIDTH          =      128            ,
    parameter   FIFO_ADDR_WIDTH     =      16             ,
    parameter   ID_WIDTH            =      4              ,
    parameter   VIRTUAL_CH_NUM      =      16             ,
    parameter   PACK_ORDER_WIDTH    =      VIRTUAL_CH_NUM ,
    parameter   AXI_ADDR_WIDTH      =      32             ,
    
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
    // ddr domain clk
    input                                   ddr_clk       ,
    input                                   ddr_rst_n     ,
    // to noc part
    input       [DATA_WIDTH : 0]            axi2nocdata   , 
    input                                   head          ,
    input                                   tail          , 

    output wire [DATA_WIDTH : 0]            nocdata       ,
    output wire                             m_is_head     ,
    output wire                             m_is_tail     ,
    input                                   nsu_ready     ,
    output wire                             noc_buf_full  ,
    output wire                             noc_buf_empty  
);

// ----- wire -----
wire [DATA_WIDTH - 1 : 0]           noc_data                ;

wire [HEAD_CODE_BIT - 1 : 0]        code_h                  ;
wire [TAIL_CODE_BIT - 1 : 0]        code_e                  ;
wire [HEAD_CODE_BIT - 1 : 0]        rd_code_h               ;
wire [TAIL_CODE_BIT - 1 : 0]        rd_code_e               ;

wire [3:0]                          flit_ptr_binary_next    ;
wire [3:0]                          Gray_flit_ptr           ;

wire [3:0]                          rd_flit_ptr_binary_next ;
wire [3:0]                          rd_Gray_flit_ptr        ;

wire                                Empty_Value             ;
wire                                head_flag               ;
wire                                tail_flag               ;

wire                                negedge_empty           ;

// ----- reg -----
reg                                 out_data_en             ;

reg  [3:0]                          flit_ptr_binary         ;
reg  [3:0]                          flit_ptr                ;
reg  [3:0]                          flit_ptr_r0             ;
reg  [3:0]                          flit_ptr_r1             ;

reg  [3:0]                          rd_flit_ptr_binary      ;
reg  [3:0]                          rd_flit_ptr             ;

reg  [3:0]                          empty_count             ;
reg                                 noc_buf_empty_r0        ;

// ---------------------------- combinational logic -------------------------
assign nocdata = {out_data_en, noc_data};
// [TAIL_CODE_H, Source_ID, Dest_ID, TYPE, PACK_NUM  , AXI_LEN(8bit), AXI_ADDR, TAIL_CODE_E]
// - ddr_clk domain -
assign code_h  = (axi2nocdata[DATA_WIDTH] == 1'b1) ? axi2nocdata[DATA_WIDTH - 1 : DATA_WIDTH - HEAD_CODE_BIT] : 'd0;
assign code_e  = (axi2nocdata[DATA_WIDTH] == 1'b1) ? axi2nocdata[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 11 - VIRTUAL_CH_NUM - AXI_ADDR_WIDTH : DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - 11 - VIRTUAL_CH_NUM - AXI_ADDR_WIDTH] : 'd0;

assign flit_ptr_binary_next = ((code_h == TAIL_CODE_H) && (code_e == TAIL_CODE_E) && tail) ? flit_ptr_binary + 1'b1 : flit_ptr_binary;					
assign Gray_flit_ptr 	    = (flit_ptr_binary_next >> 1) ^ flit_ptr_binary_next;

// - noc_clk domain -
assign rd_code_h = (out_data_en == 1'b1) ? noc_data[DATA_WIDTH - 1 : DATA_WIDTH - HEAD_CODE_BIT] : 'd0;
assign rd_code_e = (out_data_en == 1'b1) ? noc_data[DATA_WIDTH - 1 - HEAD_CODE_BIT - 2*ID_WIDTH - 11 - VIRTUAL_CH_NUM - AXI_ADDR_WIDTH : DATA_WIDTH - 2*HEAD_CODE_BIT - 2*ID_WIDTH - 11 - VIRTUAL_CH_NUM - AXI_ADDR_WIDTH] : 'd0;

assign Empty_Value             = (rd_Gray_flit_ptr == flit_ptr_r1);
assign rd_Gray_flit_ptr 	   = (rd_flit_ptr_binary_next >> 1) ^ rd_flit_ptr_binary_next;
assign rd_flit_ptr_binary_next = ((rd_code_h == TAIL_CODE_H) && (rd_code_e == TAIL_CODE_E) && m_is_tail) ? rd_flit_ptr_binary + 1'b1 : rd_flit_ptr_binary;			

assign m_is_head = head_flag && out_data_en;
assign m_is_tail = tail_flag && out_data_en;

assign negedge_empty = ~noc_buf_empty && noc_buf_empty_r0;

// // out_data_en
// always@(*) begin
//     if(noc_rst_n == 1'b0)
//         out_data_en <= 1'b0;
//     else if (~Empty_Value && ~noc_buf_empty && ~(|empty_count) && ~negedge_empty && nsu_ready)
//         out_data_en <= 1'b1; 
//     else
//         out_data_en <= 1'b0;   
// end
// ---------------------------- sequential logic ----------------------------
// - ddr_clk domain -
always@(posedge ddr_clk or negedge ddr_rst_n) begin
    if(ddr_rst_n == 1'b0) begin
        flit_ptr_binary <= 4'b0000;
        flit_ptr        <= 4'b0000;
    end
    else begin
        flit_ptr_binary <= flit_ptr_binary_next;
        flit_ptr        <= Gray_flit_ptr;
    end
end

// - CDC -
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0) begin
        flit_ptr_r0 <= 4'b0000;
        flit_ptr_r1 <= 4'b0000;
    end
    else begin
        flit_ptr_r0 <= flit_ptr;
        flit_ptr_r1 <= flit_ptr_r0;
    end
end

// - noc_clk domain -
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0) begin
        rd_flit_ptr_binary <= 4'b0000;
        rd_flit_ptr        <= 4'b0000;
    end
    else begin
        rd_flit_ptr_binary <= rd_flit_ptr_binary_next;
        rd_flit_ptr        <= rd_Gray_flit_ptr;        
    end
end

// noc_buf_empty_r0
always@(posedge noc_clk) begin
    noc_buf_empty_r0 <= noc_buf_empty;
end

// empty_count
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        empty_count <= 1'b0;
    else if (empty_count == 'd4)
        empty_count <= 1'b0; 
    else if (negedge_empty || (|empty_count))
        empty_count <= empty_count + 1'b1;   
end

// out_data_en
always@(posedge noc_clk or negedge noc_rst_n) begin
    if(noc_rst_n == 1'b0)
        out_data_en <= 1'b0;
    else if (~Empty_Value && ~noc_buf_empty && ~(|empty_count) && ~negedge_empty && nsu_ready)
        out_data_en <= 1'b1; 
    else
        out_data_en <= 1'b0;   
end

// ----------------------------  instantiation  ----------------------------
async_fifo_noc nsu_async_fifo_out_noc (
    .rst            (~ddr_rst_n                     ),
    .wr_clk         (ddr_clk                        ),
    .rd_clk         (noc_clk                        ),
    .din            ({axi2nocdata[DATA_WIDTH - 1 : 0], head, tail}          ),
    .wr_en          (axi2nocdata[DATA_WIDTH]        ),
    .rd_en          (out_data_en                    ),
    .dout           ({noc_data[DATA_WIDTH - 1 : 0], head_flag, tail_flag}   ),
    .full           (noc_buf_full                   ),
    .empty          (noc_buf_empty                  ),
    .wr_rst_busy    (),
    .rd_rst_busy    () 
);

endmodule //nsu_asyn_cross_axi2noc
