module stream_insert_tb #(
    parameter DATA_WD = 32,
    parameter DATA_BYTE_WD = DATA_WD / 8,
    parameter BYTE_CNT_WD = $clog2(DATA_BYTE_WD)
)();

    reg                           clk;
    reg                         rst_n;

    reg                         valid_in;
    wire [DATA_BYTE_WD - 1 : 0] keep_in;
    reg [DATA_WD - 1 : 0]       data_in;
    wire                        last_in;
    wire                        ready_in;

    wire                        valid_out;
    wire [DATA_BYTE_WD - 1 : 0] keep_out;
    wire [DATA_WD - 1 : 0]      data_out;
    wire                        last_out;
    reg                         ready_out;

    reg                         valid_insert;
    reg [BYTE_CNT_WD - 1 : 0]   byte_insert_cnt;
    reg [DATA_WD - 1 : 0]       data_insert;
    wire [BYTE_CNT_WD - 1 : 0]  keep_insert;        
    wire                        ready_insert;

    initial begin
        clk = 0;
        forever begin
            #5 clk = ~clk;
        end
    end

    initial begin
        rst_n = 0;
        #100
        rst_n = 1;
        #10000
        $finish;
    end

    initial begin
        $dumpfile("test.vcd");
        $dumpvars;
    end

    reg [BYTE_CNT_WD - 1 : 0] data_input_beat_cnt;
    reg [BYTE_CNT_WD - 1 : 0] last_beat_invalid_byte;

    assign keep_in = last_in ? ~((1 << last_beat_invalid_byte) - 1) : ((1 << DATA_BYTE_WD) - 1);
    assign last_in = &data_input_beat_cnt;
    assign keep_insert = (1 << byte_insert_cnt) - 1;

    wire fire_in     = valid_in && ready_in;
    wire fire_out    = valid_out && ready_out;
    wire fire_insert = valid_insert && ready_insert;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            data_insert            <= 'b0;
            byte_insert_cnt        <= 'b0;
            last_beat_invalid_byte <= 'b0;    
        end
        else if(fire_insert) begin // fire_out && last_out是ready_insert，而不一定valid
            data_insert <= $random;
            byte_insert_cnt <= byte_insert_cnt + 1'b1;
            if (&byte_insert_cnt) begin
                last_beat_invalid_byte <= last_beat_invalid_byte + 1'b1;
                if(last_beat_invalid_byte == DATA_BYTE_WD - 1) begin
                    last_beat_invalid_byte <= 'b0;                    //注意清零，但是感觉不清零也可以？因为有位数限制
                end
            end
        end
    end
//也可以data_insert和data_in一起处理，byte_insert_cnt和last_beat一起处理

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            data_input_beat_cnt <= 'b0;
        end
        else if (fire_in) begin
            data_input_beat_cnt <= data_input_beat_cnt + 1'b1;  //注意是input的beat,所以fire_in触发
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            data_in <= 'b0;
        end
        else if(fire_in) begin
            data_in <= $random;
        end
    end

    localparam RANDOM_FIRE = 0;
    generate if(!RANDOM_FIRE) begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                valid_in     <= 1'b0;
                ready_out    <= 1'b1;
                valid_insert <= 1'b0;
            end
            else begin
                valid_in     <= 1'b1;
                ready_out    <= 1'b1;
                valid_insert <= 1'b1;
            end
        end
    end
    else begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                valid_in     <= 1'b0;
                ready_out    <= 1'b1;
                valid_insert <= 1'b0;
            end
            else begin     //注意只有在has_not_pending的时候才可以改变valid数据!!!
                ready_out <= $random;
                if(!valid_in || ready_in) begin
                    valid_in <= $random;
                end
                if(!valid_insert || ready_insert) begin
                    valid_insert <= $random;
                end
            end
            end
        end
    endgenerate
    
//valid和ready信号在最后一个always块里集中处理

    stream_insert #(
        .DATA_WD(DATA_WD),
        .DATA_BYTE_WD(DATA_BYTE_WD),
        .BYTE_CNT_WD(BYTE_CNT_WD)
    ) u_stream_insert (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .keep_in(keep_in),
        .data_in(data_in),
        .last_in(last_in),
        .ready_in(ready_in),
        .valid_out(valid_out),
        .keep_out(keep_out),
        .data_out(data_out),
        .last_out(last_out),
        .ready_out(ready_out),
        .valid_insert(valid_insert),
        .keep_insert(keep_insert),
        .data_insert(data_insert),
        .byte_insert_cnt(byte_insert_cnt),
        .ready_insert(ready_insert)
    );

endmodule