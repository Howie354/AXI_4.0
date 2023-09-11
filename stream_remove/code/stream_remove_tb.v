module stream_remove_tb #(
    parameter DATA_WD = 32,
    parameter DATA_BYTE_WD = DATA_WD / 8,
    parameter BYTE_CNT_WD = $clog2(DATA_BYTE_WD)
) ();

    reg                         clk;
    reg                         rst_n;

    reg                         valid_in;
    reg [DATA_WD - 1 : 0]       data_in;
    wire [DATA_BYTE_WD - 1 : 0] keep_in;
    wire                        last_in;
    wire                        ready_in;

    wire                        valid_out;
    wire [DATA_WD - 1 : 0]      data_out;
    wire [DATA_BYTE_WD - 1 : 0] keep_out;
    wire                        last_out;
    reg                         ready_out;

    reg                         valid_remove;
    reg [BYTE_CNT_WD - 1 : 0]   byte_remove_cnt;
    wire                        ready_remove;

    initial begin
        $dumpfile("test.vcd");
        $dumpvars;
    end
    
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

    wire fire_in = valid_in && ready_in;
    wire fire_out = valid_out && ready_out;
    wire fire_remove = valid_remove && ready_remove;
    wire [BYTE_CNT_WD : 0] data_byte_cnt = DATA_BYTE_WD;
    reg first_beat; //首拍，用fire_out && fire_remove的下一拍来赋值

    reg [BYTE_CNT_WD - 1 : 0] last_beat_byte_cnt; //最后一拍无效数据位数
    assign keep_in = last_in ? ~((1 << last_beat_byte_cnt) - 1) : (first_beat ? ((1 << (data_byte_cnt - byte_remove_cnt)) - 1) : ((1 << DATA_BYTE_WD) - 1));

    reg [1 : 0] data_beat_cnt; //第几拍数据
    assign last_in = &data_beat_cnt;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            first_beat <= 1'b0;
        end
        else if (fire_out && fire_remove) begin
            first_beat <= 1'b1;
        end
        else begin
            first_beat <= 1'b0;
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

    always @(posedge clk or negedge rst_n) begin
       if(!rst_n) begin
            byte_remove_cnt <= 'b0;
       end
       else if (fire_remove) begin
            byte_remove_cnt <= byte_remove_cnt + 1'b1;
       end 
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            data_beat_cnt <= 'b0;
        end
        else begin
            data_beat_cnt <= data_beat_cnt + fire_in;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            last_beat_byte_cnt <= 'b0;      
        end
        else if(fire_out && fire_remove && data_beat_cnt == 2'b11)begin
            last_beat_byte_cnt <= last_beat_byte_cnt + 1'b1;
        end
    end

    localparam RANDOM_FIRE = 0;

    generate if(!RANDOM_FIRE) begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                valid_in <= 1'b0;  
            end
            else if(!valid_in || ready_in) begin
                valid_in <= 1'b1;
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                ready_out <= 1'b1;
                valid_remove <= 1'b0;
            end
            else begin
                ready_out <= 1'b1;
                if(!valid_remove || ready_remove) begin
                    valid_remove <= 1'b1;
                end
            end
        end       
    end
    else begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                valid_in <= 1'b0;  
            end
            else if(!valid_in || ready_in) begin
                valid_in <= $random;
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if(rst_n) begin
                ready_out <= 1'b1;
                valid_remove <= 1'b0;
            end
            else begin
                ready_out <= $random;
                if(!valid_remove || ready_remove) begin
                    valid_remove <= $random;
                end
            end
        end
    end
        
    endgenerate

    stream_remove #(
        .DATA_WD(DATA_WD),
        .DATA_BYTE_WD(DATA_BYTE_WD),
        .BYTE_CNT_WD(BYTE_CNT_WD)
    ) u_stream_remove (
        .clk(clk),
        .rst_n(rst_n),
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