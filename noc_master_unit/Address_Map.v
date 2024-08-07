/* 
    - NMU lookup is performed to determine the destination ID -
    Author : ToOddler 
    Email  : 23011211185@stu.xidian.edu.cn

    保证 PL_DDR 与 PS_DDR 全部交织 ,  PL_INTERLEAVING = 2*PS_INTERLEAVING
*/

module Address_Map #(
    parameter   AXI_ADDR_WIDTH      =      32             ,
    parameter   DATA_WIDTH          =      128            ,
    parameter   ID_WIDTH            =      4              ,
    parameter   PS_INTERLEAVING     =     32'h0000_1000   ,
    parameter   PL_INTERLEAVING     =  2*PS_INTERLEAVING  ,

    parameter   DEST_ID_PS          =     4'b0001         , //TODO: 地址修改
    parameter   DEST_ID_PL          =     4'b0011             
)(
    // user domain clk
    input                                   axi_clk       ,
    input                                   axi_rst_n     ,
    // addr lens
    input                                   lookup_en     ,
    input                                   next_req      ,
    input       [AXI_ADDR_WIDTH - 1:0]      axi_addr      ,
    input       [7:0]                       axi_len       ,
    // output
    output reg                              dest_en       ,
    output reg  [AXI_ADDR_WIDTH - 1:0]      dest_addr     ,
    output reg  [7:0]                       dest_len      ,
    output reg  [ID_WIDTH - 1:0]            dest_id       ,            
    output reg                              lookup_done
);

// ----- localparam -----
localparam  S_IDLE           = 3'b001,
            CHECK_BASE_ADDR  = 3'b010,
            CHECK_HIGH_ADDR  = 3'b100;

// ----- wire -----
wire [19:0]                           quotient       ;
wire [1:0]                            reminder       ;
wire                                  vld_out        ;

// ----- reg -----
reg  [AXI_ADDR_WIDTH - 1:0]           base_addr      ;
reg  [2:0]                            curr_state     ;
reg  [2:0]                            next_state     ;

reg  [AXI_ADDR_WIDTH - 1:0]           temp_addr      ;
reg  [AXI_ADDR_WIDTH - 1:0]           delta_addr     ;
reg  [AXI_ADDR_WIDTH - 1:0]           higher_addr    ;

reg                                   vld_out_r0     ;
reg                                   vld_out_r1     ;
reg  [7:0]                            axi_len_r0     ;

// ---------------------------- combinational logic -------------------------
always@(*) begin
    base_addr <= axi_addr;
end

// ---------------------------- sequential logic ----------------------------
// fsm_state
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        curr_state <= S_IDLE;
    else
        curr_state <= next_state;
end

// vld_out_r0
always@(posedge axi_clk) begin
    vld_out_r0 <= vld_out;
    vld_out_r1 <= vld_out_r0;
end

// axi_len_r0
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        axi_len_r0 <= 'd0;
    else if (lookup_en && curr_state == S_IDLE)
        axi_len_r0 <= axi_len;
end

// delta_addr
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        delta_addr <= 'd0;
    else
        delta_addr <= (axi_len_r0 + 1'b1)<<($clog2(DATA_WIDTH/8));
end

// dest_id
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        dest_id <= 'd0;
    else if (curr_state == CHECK_BASE_ADDR) begin
        if (vld_out && (reminder < 'd2))
            dest_id <= DEST_ID_PL;
        else if (vld_out && (reminder == 'd2))
            dest_id <= DEST_ID_PS;
    end
    else if (curr_state == CHECK_HIGH_ADDR) begin
        if (dest_id == DEST_ID_PL)   
            dest_id <= DEST_ID_PS;
        else                         
            dest_id <= DEST_ID_PL;
    end
end

// dest_addr temp_addr
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0) begin
        dest_addr   <= 'd0;
        temp_addr   <= 'd0;
    end
    else if (curr_state == CHECK_BASE_ADDR) begin
        if (vld_out && (reminder < 'd2)) begin
            dest_addr   <= (quotient<<($clog2(PL_INTERLEAVING))) + base_addr[$clog2(PS_INTERLEAVING) - 1:0] + (reminder<<($clog2(PS_INTERLEAVING)));
            temp_addr   <= quotient<<($clog2(PS_INTERLEAVING));
        end
        else if (vld_out && (reminder == 'd2)) begin
            dest_addr   <= (quotient<<($clog2(PS_INTERLEAVING))) + base_addr[$clog2(PS_INTERLEAVING) - 1:0];
            temp_addr   <= (quotient<<($clog2(PL_INTERLEAVING))) + PL_INTERLEAVING;
        end
    end
    else if (curr_state == CHECK_HIGH_ADDR) begin
        dest_addr <= temp_addr;
    end
end

// higher_addr
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        higher_addr <= 'd0;
    else if (vld_out_r0 && dest_id == DEST_ID_PS) 
        higher_addr <= dest_addr[$clog2(PS_INTERLEAVING) - 1:0] + delta_addr;
    else if (vld_out_r0 && dest_id == DEST_ID_PL) 
        higher_addr <= dest_addr[$clog2(PL_INTERLEAVING) - 1:0] + delta_addr;   
end

// dest_len lookup_done
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0) begin
        dest_len    <= 'd0;
        lookup_done <= 1'b0;
    end
    else if (curr_state == CHECK_BASE_ADDR) begin
        if (vld_out_r1) begin
            if ((higher_addr < PL_INTERLEAVING && dest_id == DEST_ID_PL) || (higher_addr < PS_INTERLEAVING && dest_id == DEST_ID_PS)) begin 
                dest_len    <= axi_len_r0;
                lookup_done <= 1'b1;
            end
            else if (higher_addr >= PS_INTERLEAVING && dest_id == DEST_ID_PS) begin 
                dest_len    <= (((PS_INTERLEAVING - dest_addr[$clog2(PS_INTERLEAVING) - 1:0])>>($clog2(DATA_WIDTH/8))) - 1'b1);
                lookup_done <= 1'b0;               
            end
            else if (higher_addr >= PL_INTERLEAVING && dest_id == DEST_ID_PL) begin
                dest_len    <= (((PL_INTERLEAVING - dest_addr[$clog2(PL_INTERLEAVING) - 1:0])>>($clog2(DATA_WIDTH/8))) - 1'b1);
                lookup_done <= 1'b0;
            end
        end
    end
    else if (curr_state == CHECK_HIGH_ADDR) begin
        dest_len    <= axi_len_r0 - dest_len - 1'b1;
        lookup_done <= 1'b1;
    end
end

// dest_en
always@(posedge axi_clk or negedge axi_rst_n) begin
    if(axi_rst_n == 1'b0)
        dest_en <= 1'b0;
    else if (dest_en == 1'b1)
        dest_en <= 1'b0;
    else if ((curr_state == CHECK_BASE_ADDR) && vld_out_r1) begin
        dest_en <= 1'b1;
    end
    else if (curr_state == CHECK_HIGH_ADDR) begin
        dest_en <= 1'b1;
    end
end

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(curr_state)
        S_IDLE: begin
            if (lookup_en)
                next_state = CHECK_BASE_ADDR;
            else
                next_state = S_IDLE;
        end

        CHECK_BASE_ADDR: begin
            if (vld_out_r1 
                && ((higher_addr < PL_INTERLEAVING && dest_id == DEST_ID_PL) 
                ||  (higher_addr < PS_INTERLEAVING && dest_id == DEST_ID_PS)))   next_state = S_IDLE;
            else if (next_req)                                                   next_state = CHECK_HIGH_ADDR;
            else                                                                 next_state = CHECK_BASE_ADDR; 
        end
        
        CHECK_HIGH_ADDR: begin
            next_state = S_IDLE;
        end
        
        default: next_state = S_IDLE;
    endcase
end

// 20位无符号除法 需要 21 个 clk
divide_by_three #(
    .ADDR_WIDTH     ( AXI_ADDR_WIDTH - $clog2(PS_INTERLEAVING))
) u_divide_by_three(
    .clk            ( axi_clk   ),
    .rst_n          ( axi_rst_n ),
    .vld_in         ( lookup_en ),
    .data_in        ( base_addr>>($clog2(PS_INTERLEAVING))),
    .quotient       ( quotient  ),
    .reminder       ( reminder  ),
    .vld_out        ( vld_out   )
);


endmodule //Address_Map
