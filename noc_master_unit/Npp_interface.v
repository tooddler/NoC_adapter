module Npp_interface #(
    parameter   DATA_WIDTH      =      128            
)(
    input     [DATA_WIDTH : 0]              noc_data      , //noc2axi data  
    input                                   s_is_head     ,
    input                                   s_is_tail     ,  

    output                                  npp_valid     ,
    output    [DATA_WIDTH - 1 : 0]          npp_data      ,
    output                                  npp_head      ,
    output                                  npp_tail            
);

assign  npp_valid               =                   noc_data[DATA_WIDTH]             ;
assign  npp_data                =                   npp_data[DATA_WIDTH - 1 : 0]     ;
assign  npp_head                =                   s_is_head                        ;
assign  npp_tail                =                   s_is_tail                        ;


endmodule
