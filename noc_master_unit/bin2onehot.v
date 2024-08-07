module bin2onehot #(
    parameter   VIRTUAL_CH_NUM      =      8        
)(
    input                               sclk                    ,       
    input                               s_rst_n                 ,
    input      [3:0]                    bin_data                ,

    output     [VIRTUAL_CH_NUM - 1:0]   onehot_data
);

reg [15:0]    onehot_data_16bit;

assign  onehot_data = onehot_data_16bit[VIRTUAL_CH_NUM - 1:0];

always@(posedge sclk or negedge s_rst_n) begin
    if(s_rst_n == 1'b0)
        onehot_data_16bit <= 'd0;
    else begin
        case (bin_data)
            1 :   onehot_data_16bit <= 16'h0001;
            2 :   onehot_data_16bit <= 16'h0002;
            3 :   onehot_data_16bit <= 16'h0004;
            4 :   onehot_data_16bit <= 16'h0008;
            5 :   onehot_data_16bit <= 16'h0010;
            6 :   onehot_data_16bit <= 16'h0020;
            7 :   onehot_data_16bit <= 16'h0040;
            8 :   onehot_data_16bit <= 16'h0080;
            9 :   onehot_data_16bit <= 16'h0100;
            10:   onehot_data_16bit <= 16'h0200;
            11:   onehot_data_16bit <= 16'h0400;
            12:   onehot_data_16bit <= 16'h0800;
            13:   onehot_data_16bit <= 16'h1000;
            14:   onehot_data_16bit <= 16'h2000;
            15:   onehot_data_16bit <= 16'h4000;                                   
            default:  onehot_data_16bit <= 'd0;
        endcase
    end
end

endmodule //bin2onehot
