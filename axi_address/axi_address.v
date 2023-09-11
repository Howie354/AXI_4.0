`default_nettype none

module axi_address #(
    parameter DW = 32,
    parameter AW = 8
) (
    input wire [AW - 1 : 0] i_last_address,
    input wire [1 : 0]      i_burst,
    input wire [7 : 0]      i_len,
    input wire [2 : 0]      i_size,
    output reg [AW - 1 : 0] o_next_address
);
    localparam DSZ = $clog2(DW / 8);

    localparam FIXED = 2'b00;
    localparam INCR  = 2'b01;
    localparam WRAP  = 2'b11;
 
    reg [AW - 1 : 0] increment,wrap_mask;

    always @(*) begin
        increment = 'b0;
        if(i_burst != FIXED) begin
            if(DSZ == 0) begin //8 bits
                increment = 1;
            end
            else if(DSZ == 1) begin //16 bits
                increment = i_size[0] ? 2 : 1;
            end
            else if(DSZ == 2) begin //32 bits
                increment = i_size[1] ? 4 : i_size[0] ? 2 : 1;
            end
            else if(DSZ == 3) begin //64 bits
                case (i_size[1 : 0]) //2^4 = 16byte = 128 bits > 64 bits ，所以不考虑i_size[2];
                    2'b00:   increment = 1;
                    2'b01:   increment = 2;
                    2'b10:   increment = 4;
                    2'b11:   increment = 8;
                    default: increment = 0;
                endcase
            end
            else begin
                increment = 1 << i_size;
            end
        end
    end

    always @(*) begin
        wrap_mask = (1 << i_size) - 1;
        if(DSZ < 2) begin //8bits 16bits
            //wrap_mask = 1; 
            wrap_mask = wrap_mask | ({{(AW - 4){1'b0}},i_len[3 : 0]} << (i_size[0]));
        end  //注意AW-4与1'b0的拼合也要用拼接符相连
        else if(DSZ < 4) begin //32bits 64bits
            //wrap_mask = 7;
            wrap_mask = wrap_mask | ({{(AW - 4){1'b0}},i_len[3 : 0]} << (i_size[1 : 0]));
        end
        else begin
            wrap_mask = wrap_mask | ({{(AW - 4){1'b0}},i_len[3 : 0]} << (i_size)); 
        end
        
        if(AW > 12) begin
            wrap_mask[AW - 1 : ((AW > 12) ? 12 : AW - 1)] = 0;
        end
    end

    always @(*) begin
        o_next_address = i_last_address + increment;
        if(i_burst != FIXED) begin
            if(DSZ < 2) begin
                o_next_address[0] = i_size[0] ? 0 : o_next_address[0]; 
            end
            else if(DSZ < 4) begin
                case (i_size[1 : 0]) 
                    2'b00: o_next_address                              = o_next_address;
                    2'b01: o_next_address[0]                           = 0;
                    2'b10: o_next_address[(AW - 1) > 1 ? 1 : (AW - 1) : 0] = 0;
                    2'b11: o_next_address[(AW - 1) > 2 ? 2 : (AW - 1) : 0] = 0;
                endcase
            end
            else begin
                case (i_size[2 : 0])
                    3'b000: o_next_address                              = o_next_address;
                    3'b001: o_next_address[0]                           = 0;
                    3'b010: o_next_address[(AW - 1) > 1 ? 1 : (AW - 1) : 0] = 0;
                    3'b011: o_next_address[(AW - 1) > 2 ? 2 : (AW - 1) : 0] = 0;
                    3'b100: o_next_address[(AW - 1) > 3 ? 3 : (AW - 1) : 0] = 0;
                    3'b101: o_next_address[(AW - 1) > 4 ? 4 : (AW - 1) : 0] = 0;
                    3'b110: o_next_address[(AW - 1) > 5 ? 5 : (AW - 1) : 0] = 0;
                    3'b111: o_next_address[(AW - 1) > 6 ? 6 : (AW - 1) : 0] = 0;
                endcase
            end

            if(i_burst[1]) begin
                o_next_address = (i_last_address & ~wrap_mask) |       //保留高位，例如len=1,i_size=3，会在8时回转，若地址给到25，则下一次应该回到16
                                 (o_next_address & wrap_mask);         //保留低位   
            end

            if (AW > 12) begin
                o_next_address[AW - 1 : ((AW > 12) ? 12 : (AW - 1))] = i_last_address[AW - 1 : ((AW > 12) ? 12 : (AW - 1))];
            end
        end
    end
    
endmodule