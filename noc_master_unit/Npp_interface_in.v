module Npp_interface_in #(
    parameter   DATA_WIDTH      =      128            
)(
    input                                  npp_valid     ,
    input    [DATA_WIDTH - 1 : 0]          npp_data      ,
    input                                  npp_head      ,
    input                                  npp_tail      ,
    
    output   [DATA_WIDTH : 0]              noc_data      , //noc2axi data  
    output                                 head          ,
    output                                 tail       
);

assign  noc_data            =                   {npp_valid, npp_data}           ;
assign  head                =                   npp_head                        ;
assign  tail                =                   npp_tail                        ; 


endmodule //Npp_interface_in
