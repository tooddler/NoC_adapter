module AXI_Master_Unit #(
    parameter           C_M_TARGET_SLAVE_BASE_ADDR	= 32'h0000_2aa0 ,
    parameter integer   C_M_AXI_BURST_LEN	        = 8'h7f         ,
    parameter integer   C_M_AXI_ID_WIDTH	        = 4             ,
    parameter integer   C_M_AXI_ADDR_WIDTH	        = 32            ,
    parameter integer   C_M_AXI_DATA_WIDTH	        = 128           ,
    parameter integer   C_M_AXI_AWUSER_WIDTH	    = 0             ,
    parameter integer   C_M_AXI_ARUSER_WIDTH	    = 0             ,
    parameter integer   C_M_AXI_WUSER_WIDTH	        = 0             ,
    parameter integer   C_M_AXI_RUSER_WIDTH	        = 0             ,
    parameter integer   C_M_AXI_BUSER_WIDTH	        = 0             
)(
    input  wire                                 M_AXI_ACLK          ,
    input  wire                                 M_AXI_ARESETN       ,

    input  wire                                 DDR_INIT_DONE       ,

    output wire [C_M_AXI_ID_WIDTH-1 : 0]        M_AXI_AWID          ,
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0]      M_AXI_AWADDR        ,
    output wire [7 : 0]                         M_AXI_AWLEN         ,
    output wire [2 : 0]                         M_AXI_AWSIZE        ,
    output wire [1 : 0]                         M_AXI_AWBURST       ,
    output wire                                 M_AXI_AWLOCK        ,
    output wire [3 : 0]                         M_AXI_AWCACHE       ,
    output wire [2 : 0]                         M_AXI_AWPROT        ,
    output wire [3 : 0]                         M_AXI_AWQOS         ,
    output wire [C_M_AXI_AWUSER_WIDTH-1 : 0]    M_AXI_AWUSER        ,
    output wire                                 M_AXI_AWVALID       ,
    input  wire                                 M_AXI_AWREADY       ,

    output wire [C_M_AXI_DATA_WIDTH-1 : 0]      M_AXI_WDATA         ,
    output wire [C_M_AXI_DATA_WIDTH/8-1 : 0]    M_AXI_WSTRB         ,
    output wire                                 M_AXI_WLAST         ,
    output wire [C_M_AXI_WUSER_WIDTH-1 : 0]     M_AXI_WUSER         ,
    output wire                                 M_AXI_WVALID        ,
    input  wire                                 M_AXI_WREADY        ,

    input  wire [C_M_AXI_ID_WIDTH-1 : 0]        M_AXI_BID           ,
    input  wire [1 : 0]                         M_AXI_BRESP         ,
    input  wire [C_M_AXI_BUSER_WIDTH-1 : 0]     M_AXI_BUSER         ,
    input  wire                                 M_AXI_BVALID        ,
    output wire                                 M_AXI_BREADY        ,

    output wire [C_M_AXI_ID_WIDTH-1 : 0]        M_AXI_ARID          ,
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0]      M_AXI_ARADDR        ,
    output wire [7 : 0]                         M_AXI_ARLEN         ,
    output wire [2 : 0]                         M_AXI_ARSIZE        ,
    output wire [1 : 0]                         M_AXI_ARBURST       ,
    output wire                                 M_AXI_ARLOCK        ,   
    output wire [3 : 0]                         M_AXI_ARCACHE       ,
    output wire [2 : 0]                         M_AXI_ARPROT        ,
    output wire [3 : 0]                         M_AXI_ARQOS         ,
    output wire [C_M_AXI_ARUSER_WIDTH-1 : 0]    M_AXI_ARUSER        ,
    output wire                                 M_AXI_ARVALID       ,
    input  wire                                 M_AXI_ARREADY       ,   

    input  wire [C_M_AXI_ID_WIDTH-1 : 0]        M_AXI_RID           ,
    input  wire [C_M_AXI_DATA_WIDTH-1 : 0]      M_AXI_RDATA         ,
    input  wire [1 : 0]                         M_AXI_RRESP         ,
    input  wire                                 M_AXI_RLAST         ,
    input  wire [C_M_AXI_RUSER_WIDTH-1 : 0]     M_AXI_RUSER         ,
    input  wire                                 M_AXI_RVALID        ,
    output wire                                 M_AXI_RREADY    
);

function integer clogb2 (input integer bit_depth);              
    begin                                                           
        for(clogb2=0; bit_depth>0; clogb2=clogb2+1)                   
            bit_depth = bit_depth >> 1;                                 
    end                                                           
endfunction 

// ----- localparam -----
localparam P_DATA_BYTE  = C_M_AXI_DATA_WIDTH/8                 ;
localparam P_M_AXI_SIZE = clogb2((C_M_AXI_DATA_WIDTH/8) - 1)   ;

localparam  P_ST_IDLE    =  0   ,
            P_ST_WRITE   =  1   ,
            P_ST_READ    =  2   ,
            P_ST_CHECK   =  3   ,
            P_ST_ERR     =  4   ;

// ----- reg -----
reg  [7 :0]                             r_st_current        ;
reg  [7 :0]                             r_st_next           ;
reg  [15:0]                             r_st_cnt            ;

reg  [C_M_AXI_ID_WIDTH-1 : 0]           ro_M_AXI_AWID       ;
reg  [C_M_AXI_ADDR_WIDTH-1 : 0]         ro_M_AXI_AWADDR     ;
reg  [7 : 0]                            ro_M_AXI_AWLEN      ;
reg  [2 : 0]                            ro_M_AXI_AWSIZE     ;
reg  [1 : 0]                            ro_M_AXI_AWBURST    ;
reg                                     ro_M_AXI_AWLOCK     ;
reg  [3 : 0]                            ro_M_AXI_AWCACHE    ;
reg  [2 : 0]                            ro_M_AXI_AWPROT     ;
reg  [3 : 0]                            ro_M_AXI_AWQOS      ;
reg  [C_M_AXI_AWUSER_WIDTH-1 : 0]       ro_M_AXI_AWUSER     ;
reg                                     ro_M_AXI_AWVALID    ;
reg  [C_M_AXI_DATA_WIDTH-1 : 0]         ro_M_AXI_WDATA      ;
reg  [C_M_AXI_DATA_WIDTH/8-1 : 0]       ro_M_AXI_WSTRB      ;
reg                                     ro_M_AXI_WLAST      ;
reg  [C_M_AXI_WUSER_WIDTH-1 : 0]        ro_M_AXI_WUSER      ;
reg                                     ro_M_AXI_WVALID     ;
reg                                     ro_M_AXI_BREADY     ;
reg [C_M_AXI_ID_WIDTH-1 : 0]            ro_M_AXI_ARID       ;
reg [C_M_AXI_ADDR_WIDTH-1 : 0]          ro_M_AXI_ARADDR     ;
reg [7 : 0]                             ro_M_AXI_ARLEN      ;
reg [2 : 0]                             ro_M_AXI_ARSIZE     ;
reg [1 : 0]                             ro_M_AXI_ARBURST    ;
reg                                     ro_M_AXI_ARLOCK     ;
reg [3 : 0]                             ro_M_AXI_ARCACHE    ;
reg [2 : 0]                             ro_M_AXI_ARPROT     ;
reg [3 : 0]                             ro_M_AXI_ARQOS      ;
reg [C_M_AXI_ARUSER_WIDTH-1 : 0]        ro_M_AXI_ARUSER     ;
reg                                     ro_M_AXI_ARVALID    ;
reg                                     ro_M_AXI_RREADY     ;
reg [C_M_AXI_DATA_WIDTH-1 : 0]          ri_M_AXI_RDATA      ;
reg                                     ri_M_AXI_RLAST      ;
reg                                     ri_M_AXI_RVALID     ;
reg                                     r_check_err         ;
reg [15:0]                              r_write_cnt         ;
reg [15:0]                              r_read_cnt          ;

// ----- wire -----
wire                                    w_AW_active         ;
wire                                    w_W_active          ;
wire                                    w_B_active          ;
wire                                    w_AR_active         ;
wire                                    w_R_active          ;

// ---------------------------- combinational logic -------------------------
assign M_AXI_AWID     = ro_M_AXI_AWID    ;
assign M_AXI_AWADDR   = ro_M_AXI_AWADDR  ;
assign M_AXI_AWLEN    = ro_M_AXI_AWLEN   ;
assign M_AXI_AWSIZE   = ro_M_AXI_AWSIZE  ;
assign M_AXI_AWBURST  = ro_M_AXI_AWBURST ;
assign M_AXI_AWLOCK   = ro_M_AXI_AWLOCK  ;
assign M_AXI_AWCACHE  = ro_M_AXI_AWCACHE ;
assign M_AXI_AWPROT   = ro_M_AXI_AWPROT  ;
assign M_AXI_AWQOS    = ro_M_AXI_AWQOS   ;
assign M_AXI_AWUSER   = ro_M_AXI_AWUSER  ;
assign M_AXI_AWVALID  = ro_M_AXI_AWVALID ;
assign M_AXI_WDATA    = ro_M_AXI_WDATA   ;
assign M_AXI_WSTRB    = ro_M_AXI_WSTRB   ;
assign M_AXI_WLAST    = ro_M_AXI_WLAST   ;
assign M_AXI_WUSER    = ro_M_AXI_WUSER   ;
assign M_AXI_WVALID   = ro_M_AXI_WVALID  ;
assign M_AXI_BREADY   = ro_M_AXI_BREADY  ;
assign M_AXI_ARID     = ro_M_AXI_ARID    ;
assign M_AXI_ARADDR   = ro_M_AXI_ARADDR  ;
assign M_AXI_ARLEN    = ro_M_AXI_ARLEN   ;
assign M_AXI_ARSIZE   = ro_M_AXI_ARSIZE  ;
assign M_AXI_ARBURST  = ro_M_AXI_ARBURST ;
assign M_AXI_ARLOCK   = ro_M_AXI_ARLOCK  ;
assign M_AXI_ARCACHE  = ro_M_AXI_ARCACHE ;
assign M_AXI_ARPROT   = ro_M_AXI_ARPROT  ;
assign M_AXI_ARQOS    = ro_M_AXI_ARQOS   ;
assign M_AXI_ARUSER   = ro_M_AXI_ARUSER  ;
assign M_AXI_ARVALID  = ro_M_AXI_ARVALID ;
assign M_AXI_RREADY   = ro_M_AXI_RREADY  ;
assign w_AW_active    = M_AXI_AWVALID & M_AXI_AWREADY;
assign w_W_active     = M_AXI_WVALID  & M_AXI_WREADY ;
assign w_B_active     = M_AXI_BVALID  & M_AXI_BREADY ;
assign w_AR_active    = M_AXI_ARVALID & M_AXI_ARREADY;
assign w_R_active     = M_AXI_RVALID  & M_AXI_RREADY ;

// ---------------------------- sequential logic ---------------------------- 
always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        r_st_current <= P_ST_IDLE;
    else 
        r_st_current <= r_st_next;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        r_st_cnt <= 'd0;
    else if(r_st_current != r_st_next)
        r_st_cnt <= 'd0;
    else 
        r_st_cnt <= r_st_cnt + 1;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN) begin
        ro_M_AXI_AWID    <= 'd0;
        ro_M_AXI_AWADDR  <= 'd0;
        ro_M_AXI_AWLEN   <= 'd0;
        ro_M_AXI_AWSIZE  <= 'd0;
        ro_M_AXI_AWBURST <= 'd0;
        ro_M_AXI_AWLOCK  <= 'd0;
        ro_M_AXI_AWCACHE <= 'd0;
        ro_M_AXI_AWPROT  <= 'd0;
        ro_M_AXI_AWQOS   <= 'd0;
        ro_M_AXI_AWUSER  <= 'd0;
        ro_M_AXI_AWVALID <= 'd0;
    end else if(w_AW_active) begin
        ro_M_AXI_AWID    <= 'd0;
        ro_M_AXI_AWADDR  <= 'd0;
        ro_M_AXI_AWLEN   <= 'd0;
        ro_M_AXI_AWSIZE  <= 'd0;
        ro_M_AXI_AWBURST <= 'd0;
        ro_M_AXI_AWLOCK  <= 'd0;
        ro_M_AXI_AWCACHE <= 'd0;
        ro_M_AXI_AWPROT  <= 'd0;
        ro_M_AXI_AWQOS   <= 'd0;
        ro_M_AXI_AWUSER  <= 'd0;
        ro_M_AXI_AWVALID <= 'd0;
    end else if(r_st_current == P_ST_WRITE && r_st_cnt == 0) begin
        ro_M_AXI_AWID    <= 'd0;
        ro_M_AXI_AWADDR  <= C_M_TARGET_SLAVE_BASE_ADDR;
        ro_M_AXI_AWLEN   <= C_M_AXI_BURST_LEN;
        ro_M_AXI_AWSIZE  <= P_M_AXI_SIZE;
        ro_M_AXI_AWBURST <= 2'b01;
        ro_M_AXI_AWLOCK  <= 'd0;
        ro_M_AXI_AWCACHE <= 4'b0010;
        ro_M_AXI_AWPROT  <= 'd0;
        ro_M_AXI_AWQOS   <= 'd0;
        ro_M_AXI_AWUSER  <= 'd0;
        ro_M_AXI_AWVALID <= 'd1;
    end else begin  
        ro_M_AXI_AWID    <= ro_M_AXI_AWID   ;
        ro_M_AXI_AWADDR  <= ro_M_AXI_AWADDR ;
        ro_M_AXI_AWLEN   <= ro_M_AXI_AWLEN  ;
        ro_M_AXI_AWSIZE  <= ro_M_AXI_AWSIZE ;
        ro_M_AXI_AWBURST <= ro_M_AXI_AWBURST;
        ro_M_AXI_AWLOCK  <= ro_M_AXI_AWLOCK ;
        ro_M_AXI_AWCACHE <= ro_M_AXI_AWCACHE;
        ro_M_AXI_AWPROT  <= ro_M_AXI_AWPROT ;
        ro_M_AXI_AWQOS   <= ro_M_AXI_AWQOS  ;
        ro_M_AXI_AWUSER  <= ro_M_AXI_AWUSER ;
        ro_M_AXI_AWVALID <= ro_M_AXI_AWVALID;
    end
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN) begin
        ro_M_AXI_WDATA <= 'd0;
        ro_M_AXI_WSTRB <= {P_DATA_BYTE{1'b1}};
        ro_M_AXI_WUSER <= 'd0;
    end else if(ro_M_AXI_WLAST) begin
        ro_M_AXI_WDATA <= 'd0;
        ro_M_AXI_WSTRB <= 'd0;
        ro_M_AXI_WUSER <= 'd0;
    end else if(w_W_active) begin
        ro_M_AXI_WDATA <= ro_M_AXI_WDATA + 1;
        ro_M_AXI_WSTRB <= {P_DATA_BYTE{1'b1}};
        ro_M_AXI_WUSER <= 'd0;
    end else begin
        ro_M_AXI_WDATA <= ro_M_AXI_WDATA;
        ro_M_AXI_WSTRB <= ro_M_AXI_WSTRB;
        ro_M_AXI_WUSER <= ro_M_AXI_WUSER;
    end

end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        ro_M_AXI_WVALID <= 'd0;
    else if(ro_M_AXI_WLAST)
        ro_M_AXI_WVALID <= 'd0;
    else if(w_AW_active)   
        ro_M_AXI_WVALID <= 'd1;
    else 
        ro_M_AXI_WVALID <= ro_M_AXI_WVALID;
end

 
always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        r_write_cnt <= 'd0;
    else if(ro_M_AXI_WLAST)
        r_write_cnt <= 'd0;
    else if(w_W_active)
        r_write_cnt <= r_write_cnt + 1;
    else 
        r_write_cnt <= r_write_cnt;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        ro_M_AXI_WLAST <= 'd0;
    else if(r_write_cnt == C_M_AXI_BURST_LEN && w_W_active)
        ro_M_AXI_WLAST <= 'd0;
    else if(r_write_cnt == C_M_AXI_BURST_LEN - 1)   
        ro_M_AXI_WLAST <= 'd1;
    else        
        ro_M_AXI_WLAST <= ro_M_AXI_WLAST;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        ro_M_AXI_BREADY <= 'd0;
    else if(w_B_active)
        ro_M_AXI_BREADY <= 'd0;
    else if(ro_M_AXI_WLAST)
        ro_M_AXI_BREADY <= 'd1;
    else 
        ro_M_AXI_BREADY <= ro_M_AXI_BREADY;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN) begin
        ro_M_AXI_ARID    <= 'd0;
        ro_M_AXI_ARADDR  <= 'd0;
        ro_M_AXI_ARLEN   <= 'd0;
        ro_M_AXI_ARSIZE  <= 'd0;
        ro_M_AXI_ARBURST <= 'd0;
        ro_M_AXI_ARLOCK  <= 'd0;
        ro_M_AXI_ARCACHE <= 'd0;
        ro_M_AXI_ARPROT  <= 'd0;
        ro_M_AXI_ARQOS   <= 'd0;
        ro_M_AXI_ARUSER  <= 'd0;
        ro_M_AXI_ARVALID <= 'd0;
    end else if(w_AR_active) begin
        ro_M_AXI_ARID    <= 'd0;
        ro_M_AXI_ARADDR  <= 'd0;
        ro_M_AXI_ARLEN   <= 'd0;
        ro_M_AXI_ARSIZE  <= 'd0;
        ro_M_AXI_ARBURST <= 'd0;
        ro_M_AXI_ARLOCK  <= 'd0;
        ro_M_AXI_ARCACHE <= 'd0;
        ro_M_AXI_ARPROT  <= 'd0;
        ro_M_AXI_ARQOS   <= 'd0;
        ro_M_AXI_ARUSER  <= 'd0;
        ro_M_AXI_ARVALID <= 'd0;
    end else if(r_st_current == P_ST_READ && r_st_cnt == 0) begin
        ro_M_AXI_ARID    <= 'd0;
        ro_M_AXI_ARADDR  <= C_M_TARGET_SLAVE_BASE_ADDR;
        ro_M_AXI_ARLEN   <= C_M_AXI_BURST_LEN;
        ro_M_AXI_ARSIZE  <= P_M_AXI_SIZE;
        ro_M_AXI_ARBURST <= 2'b01;
        ro_M_AXI_ARLOCK  <= 'd0;
        ro_M_AXI_ARCACHE <= 4'b0010;
        ro_M_AXI_ARPROT  <= 'd0;
        ro_M_AXI_ARQOS   <= 'd0;
        ro_M_AXI_ARUSER  <= 'd0;
        ro_M_AXI_ARVALID <= 'd1;
    end else begin  
        ro_M_AXI_ARID    <= ro_M_AXI_ARID   ;
        ro_M_AXI_ARADDR  <= ro_M_AXI_ARADDR ;
        ro_M_AXI_ARLEN   <= ro_M_AXI_ARLEN  ;
        ro_M_AXI_ARSIZE  <= ro_M_AXI_ARSIZE ;
        ro_M_AXI_ARBURST <= ro_M_AXI_ARBURST;
        ro_M_AXI_ARLOCK  <= ro_M_AXI_ARLOCK ;
        ro_M_AXI_ARCACHE <= ro_M_AXI_ARCACHE;
        ro_M_AXI_ARPROT  <= ro_M_AXI_ARPROT ;
        ro_M_AXI_ARQOS   <= ro_M_AXI_ARQOS  ;
        ro_M_AXI_ARUSER  <= ro_M_AXI_ARUSER ;
        ro_M_AXI_ARVALID <= ro_M_AXI_ARVALID;
    end
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        ro_M_AXI_RREADY <= 'd0;
    else if(M_AXI_RLAST)
        ro_M_AXI_RREADY <= 'd0;
    else if(w_AR_active)
        ro_M_AXI_RREADY <= 'd1;
    else 
        ro_M_AXI_RREADY <= ro_M_AXI_RREADY;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        r_read_cnt <= 'd0;
    else if(ri_M_AXI_RLAST)
        r_read_cnt <= 'd0;
    else if(ri_M_AXI_RVALID)
        r_read_cnt <= r_read_cnt + 1;
    else 
        r_read_cnt <= r_read_cnt;
end

always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN) begin
        ri_M_AXI_RDATA  <= 'd0;
        ri_M_AXI_RLAST  <= 'd0;
        ri_M_AXI_RVALID <= 'd0;
    end else if(w_R_active) begin
        ri_M_AXI_RDATA  <= M_AXI_RDATA;
        ri_M_AXI_RLAST  <= M_AXI_RLAST;
        ri_M_AXI_RVALID <= 'd1;
    end else begin
        ri_M_AXI_RDATA  <= 'd0;
        ri_M_AXI_RLAST  <= 'd0;
        ri_M_AXI_RVALID <= 'd0;
    end
end


always@(posedge M_AXI_ACLK,negedge M_AXI_ARESETN) begin
    if(!M_AXI_ARESETN)
        r_check_err <= 'd0;
    else if(ri_M_AXI_RVALID && ri_M_AXI_RDATA != r_read_cnt)
        r_check_err <= 'd1;
    else        
        r_check_err <= r_check_err;
end 

// ---------------------------------- FSM ----------------------------------
always@(*) begin
    case(r_st_current)
        P_ST_IDLE  : r_st_next = DDR_INIT_DONE  ? P_ST_WRITE : P_ST_IDLE  ;
        P_ST_WRITE : r_st_next = w_B_active     ? P_ST_READ  : P_ST_WRITE ;
        P_ST_READ  : r_st_next = ri_M_AXI_RLAST ? P_ST_CHECK : P_ST_READ  ;
        P_ST_CHECK : r_st_next = r_check_err    ? P_ST_ERR   : P_ST_IDLE  ;
        P_ST_ERR   : r_st_next = P_ST_ERR;
        default    : r_st_next = P_ST_IDLE;
    endcase
end

endmodule // AXI_Master_Unit
