module stream_remove_header_tb #(
    parameter DATA_WD = 32,
    parameter DATA_BYTE_WD = DATA_WD / 8,
    parameter BYTE_CNT_WD = $clog2(DATA_BYTE_WD)
) ( );
    reg                       clk;
    reg                       rstn;
    reg                       valid_in;
    reg  [DATA_WD-1 : 0]      data_in;
    wire [DATA_BYTE_WD-1 : 0] keep_in;
    wire                      last_in;
    wire                      ready_in;
    wire                      valid_out;
    wire [DATA_WD-1 : 0]      data_out;
    wire [DATA_BYTE_WD-1 : 0] keep_out;
    wire                      last_out;
    reg                       ready_out;
    reg                       valid_remove;
    reg  [BYTE_CNT_WD-1 : 0]  byte_remove_cnt;
    wire                      ready_remove;

wire fire_in;
wire fire_out;
wire fire_remove;
assign fire_in = valid_in && ready_in;
assign fire_out = valid_out && ready_out;
assign fire_remove = valid_remove && ready_remove;

initial begin
    clk = 0;
    forever begin
        #5 clk = ~clk;
    end
end

initial begin
    rstn = 0;
    #100 
    rstn = 1;
    #10000
    $finish;
end

initial begin
    $dumpfile("test.vcd");
    $dumpvars;
end

reg [BYTE_CNT_WD-1 : 0] last_beat_byte_cnt; //最后一拍有多少字节数据无效
reg [1:0] data_input_beat_cnt; //记录第几拍数据
assign last_in = &data_input_beat_cnt;
assign keep_in = last_in ? ~((1 << last_beat_byte_cnt) - 1) : ((1 << DATA_BYTE_WD) - 1);

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        byte_remove_cnt <= 'b0;
    end
    else if (fire_remove) begin
        byte_remove_cnt <= byte_remove_cnt + 1'b1;
    end    
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        data_in <= $random;
    end
    else begin
        if (fire_in) begin
            data_in <= $random;
        end 
    end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        last_beat_byte_cnt <= 'b0;
    end
    else begin
        if (fire_out && last_out && byte_remove_cnt == 2'b11) begin
            last_beat_byte_cnt <= last_beat_byte_cnt + 1'b1;
        end
    end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        data_input_beat_cnt <= 'b0;
    end
    else begin
        if (fire_in) begin
            data_input_beat_cnt <= data_input_beat_cnt + 1'b1;
        end
    end
end

localparam HAS_RANDOM = 1'b0;

generate if (HAS_RANDOM) begin
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            valid_in <= 'b0;
        end
        else if (fire_in || !valid_in) begin
            valid_in <= $random;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ready_out <= 1'b1;
            valid_remove <= 1'b0;
        end
        else begin
            ready_out <= $random;

            if (fire_remove || !valid_remove) begin
                valid_remove <= $random;
            end
        end
    end
end
else begin
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            valid_in <= 'b0;
        end
        else if (fire_in || !valid_in) begin
            valid_in <= 1'b1;
        end
    end
    
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ready_out <= 1'b1;
            valid_remove <= 1'b0;
        end
        else begin
            ready_out <= 1'b1;
    
            if (fire_remove || !valid_remove) begin
                valid_remove <= 1'b1;
            end
        end
    end
end
endgenerate

stream_remove_header # (.DATA_WD(DATA_WD),
                        .DATA_BYTE_WD(DATA_BYTE_WD),
                        .BYTE_CNT_WD(BYTE_CNT_WD)
) stream_remove_header_tb (.clk(clk),
                           .rstn(rstn),
                           .valid_in(valid_in),
                           .data_in(data_in),
                           .keep_in(keep_in),
                           .last_in(last_in),
                           .ready_in(ready_in),
                           .valid_out(valid_out),
                           .data_out(data_out),
                           .keep_out(keep_out),
                           .last_out(last_out),
                           .ready_out(ready_out),
                           .valid_remove(valid_remove),
                           .byte_remove_cnt(byte_remove_cnt),
                           .ready_remove(ready_remove)
);
endmodule