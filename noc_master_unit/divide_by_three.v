
module divide_by_three #(
    parameter   ADDR_WIDTH     =       20
)(
    input                                       clk             ,
    input                                       rst_n           ,
    input                                       vld_in          ,
    input         [ADDR_WIDTH-1:0]              data_in         ,
    output  reg   [ADDR_WIDTH-1:0]              quotient        , 
    output  reg   [1:0]                         reminder        ,
    output  reg                                 vld_out
);

localparam IDLE = 2'b11;

reg [1:0]                    current_state   ;
reg [1:0]                    next_state      ;

reg [$clog2(ADDR_WIDTH):0]   cnt             ;
reg [ADDR_WIDTH-1:0]         data_reg        ;



always@(posedge clk or negedge rst_n) begin
    if(!rst_n) 
        current_state <= IDLE;
    else 
        current_state <= next_state;
end

always@(*) begin
    case(current_state)
        IDLE:  
            if(vld_in)                      next_state = 2'b0;
            else                            next_state = IDLE;

        2'b00: 
            if (cnt == ADDR_WIDTH)           next_state = IDLE; // cnt = 16 not 15, for the calc of remainder
            else if(data_reg[ADDR_WIDTH-1])  next_state = 2'b1;
            else                            next_state = 2'b0;

        2'b01: 
            if (cnt == ADDR_WIDTH)           next_state = IDLE;
            else if(data_reg[ADDR_WIDTH-1])  next_state = 2'b0;
            else                            next_state = 2'b10;

        2'b10: 
            if (cnt == ADDR_WIDTH)           next_state = IDLE;
            else if(data_reg[ADDR_WIDTH-1])  next_state = 2'b10;
            else                            next_state = 2'b1;
        default: next_state = IDLE;
    endcase
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        {cnt,data_reg,reminder,quotient,vld_out} <= 0;
    end else begin
        case(current_state)
            IDLE: begin
                {vld_out,cnt} <= 0;
                if(vld_in) data_reg <= data_in;
                else data_reg <= data_reg;
            end
            2'b00,2'b01,2'b10: begin
                if(cnt == ADDR_WIDTH-1) begin
                    cnt <= cnt + 1;        // without this,remainder will be next_state=IDLE=2'b11'
                    reminder <= next_state;
                    vld_out <= 1;
                end else begin
                    cnt <= cnt + 1; 
                    vld_out <= 0;
                    data_reg <= {data_reg[ADDR_WIDTH-2:0],1'b0};
                end
                if(data_reg[ADDR_WIDTH-1]) 
                    quotient <= {quotient[ADDR_WIDTH-2:0],current_state[1]|current_state[0]};
                else 
                    quotient <= {quotient[ADDR_WIDTH-2:0],current_state[1]};
            end
        endcase
    end
end

endmodule // divide_by_three
