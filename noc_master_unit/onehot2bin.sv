// onehot2bin
`timescale 1ns / 1ns
module onehot2bin #(
    parameter  ONE_HOT_WIDTH    = 8
)(
    input [ONE_HOT_WIDTH-1 : 0]                     one_hot_code,
    output logic [$clog2(ONE_HOT_WIDTH)-1 : 0]      bin_code
);
	logic [$clog2(ONE_HOT_WIDTH)-1 : 0]     temp1 [ONE_HOT_WIDTH-1 : 0];
	logic [ONE_HOT_WIDTH-1 : 0] 			temp2 [$clog2(ONE_HOT_WIDTH)-1 : 0];
		
	genvar i,j,k;
	generate
		for(i = 0; i < ONE_HOT_WIDTH; i = i+1)begin : temp1_loop
			assign temp1[i] = one_hot_code[i]? i:'b0;
		end
	endgenerate
	generate
		for(i = 0; i < ONE_HOT_WIDTH; i = i+1)begin : temp_ch1
			for(j = 0; j < $clog2(ONE_HOT_WIDTH); j = j+1)begin  : temp_ch2
				assign temp2[j][i] = temp1[i][j];
			end
		end
	endgenerate
	generate
		for(j = 0; j < $clog2(ONE_HOT_WIDTH); j = j+1)begin : temp2_loop
			assign bin_code[j] = |temp2[j];
		end
	endgenerate
	
endmodule
