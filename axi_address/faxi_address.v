`default_nettype none  //input和output需要自己定义类型
    

module faxi_address #(
    parameter AW = 32
) (
    input  wire [AW - 1 : 0] i_last_address,
    input  wire [2 : 0]      i_size,    // 2^i_size ，表示每一拍要传输的Byte数。1B,2B,4B,8B,16B,etc
    input  wire [1 : 0]      i_burst,   // FIXED , INCR , WRAP
    input  wire [7 : 0]      i_len,    // 表示一次发多少拍，实际为1~256，因为i_len只能0~255，所以实际为i_len + 1
                                   // INCR（0~255） FIXED（0~15） WRAP（ 1 3 7 15），
                                   //因为Cache的最小缓存单元是二进制的整数（i_len * i_size），所以WRAP取值只能是2^n
    output reg  [AW - 1 : 0] o_next_address
);
    localparam FIXED = 2'b00;
    localparam INCR  = 2'b01;
    localparam WRAP  = 2'b11;

    reg [AW - 1 : 0] wrap_mask,increment; //increment是每次增加的地址数

    always @(*) begin
        increment = 0;  //FIXED
        if(i_burst != FIXED) begin
            case(i_size) 
                3'b000:  increment = 1;
                3'b001:  increment = 2;
                3'b010:  increment = 4;
                3'b011:  increment = 8;
                3'b100:  increment = 16;
                3'b101:  increment = 32;
                3'b110:  increment = 64;
                3'b111:  increment = 128;
                default: increment = 0;
            endcase
        end
    end

    always @(*) begin
        wrap_mask = 'b0;   //FIXED & INCR
        if(i_burst == WRAP) begin
            if(i_len == 1) begin
                wrap_mask = 1 << (i_size + 1);
            end
            if(i_len == 3) begin
                wrap_mask = 1 << (i_size + 2);
            end
            if(i_len == 7) begin
                wrap_mask = 1 << (i_size + 3);
            end
            if(i_len == 15) begin
                wrap_mask = 1 << (i_size + 4);
            end
            wrap_mask = wrap_mask - 1'b1;
        end 
    end

    always @(*) begin
        o_next_address = i_last_address + increment;
        if(i_burst != FIXED) begin
            // if(i_size == 0) begin
            //     o
            // end               当i_size为0时，一拍只发1byte，默认一直以1对齐
            if(i_size == 1) begin
                o_next_address[0] = 0; 
            end
            if(i_size == 2) begin
                o_next_address[1 : 0] = 0;
            end
            if(i_size == 3) begin
                o_next_address[2 : 0] = 0;
            end
            if(i_size == 4) begin
                o_next_address[3 : 0] = 0;
            end
            if(i_size == 5) begin
                o_next_address[4 : 0] = 0;
            end
            if(i_size == 6) begin
                o_next_address[5 : 0] = 0;
            end
            if(i_size == 7) begin
                o_next_address[6 : 0] = 0;
            end
        end
        if(i_burst == WRAP) begin
            o_next_address = (i_last_address & ~wrap_mask) |       //保留高位，例如len=1,i_size=3，会在8时回转，若地址给到25，则下一次应该回到17
                             (o_next_address & wrap_mask);         //保留低位
        end
    end

endmodule