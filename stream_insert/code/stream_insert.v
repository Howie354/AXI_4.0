module stream_insert #(
    parameter DATA_WD = 32,
    parameter DATA_BYTE_WD = DATA_WD / 8,
    parameter BYTE_CNT_WD = $clog2(DATA_BYTE_WD)
) (
    input                         clk,
    input                         rst_n,

    input                         valid_in,
    input [DATA_BYTE_WD - 1 : 0]  keep_in,
    input [DATA_WD - 1 : 0]       data_in,
    input                         last_in,
    output                        ready_in,

    output                        valid_out,
    output [DATA_BYTE_WD - 1 : 0] keep_out,
    output [DATA_WD - 1 : 0]      data_out,
    output                        last_out,
    input                         ready_out,

    input                         valid_insert,
    input [BYTE_CNT_WD - 1 : 0]   byte_insert_cnt,
    input [DATA_WD - 1 : 0]       data_insert,
    input [BYTE_CNT_WD - 1 : 0]   keep_insert,        
    output                        ready_insert
);

    localparam DATA_BIT_CNT_WD = $clog2(DATA_WD); //用于给double_data右移数据量提供宽度

    wire fire_in     = valid_in && ready_in;
    wire fire_out    = valid_out && ready_out;
    wire fire_insert = valid_insert && ready_insert;

    reg                        valid_in_r;
    reg [DATA_WD - 1 : 0]      data_in_r;
    reg [DATA_BYTE_WD - 1 : 0] keep_in_r;

    reg first_beat_r;
    reg extra_last_beat_r;

    wire [DATA_BYTE_WD - 1 : 0] keep_nothing = {DATA_BYTE_WD{1'b0}};

    wire [2*DATA_WD - 1 : 0]      double_data = first_beat_r ? {data_insert , data_in} : {data_in_r , data_in};
    wire [2*DATA_BYTE_WD - 1 : 0] double_keep = first_beat_r ?
                                                {keep_insert , keep_in} : 
                                                (extra_last_beat_r ?
                                                {keep_in_r , keep_nothing} : {keep_in_r , keep_in}
                                                );

    wire [BYTE_CNT_WD - 1     : 0] right_shift_byte_cnt = byte_insert_cnt;
    wire [DATA_BIT_CNT_WD - 1 : 0] right_shift_bit_cnt  = right_shift_byte_cnt << 3;
    
    assign data_out = double_data >> right_shift_bit_cnt;
    assign keep_out = double_keep >> right_shift_byte_cnt;

    wire [BYTE_CNT_WD : 0]      data_byte_cnt       = DATA_BYTE_WD;
    wire [BYTE_CNT_WD - 1 : 0]  lift_shift_byte_cnt = data_byte_cnt - byte_insert_cnt;
    wire [DATA_BYTE_WD - 1 : 0] next_byte_to_keep   = keep_in << lift_shift_byte_cnt;
    wire                        has_extra_beat      = !extra_last_beat_r && |next_byte_to_keep;
    
    assign ready_insert = fire_out && last_out;
    assign ready_in     = first_beat_r || (ready_out && valid_insert && !extra_last_beat_r);
    assign valid_out    = first_beat_r ? 
                        valid_in : 
                        (extra_last_beat_r ?
                         valid_in_r :
                         (valid_in && valid_in_r)
                        )&& valid_insert; //特别注意首拍和最后一拍                        
    assign last_out     = extra_last_beat_r || (last_in && !has_extra_beat);  //不是fire_out && !has_extra_beat，是last_in，想象有一条线直通

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            first_beat_r <= 1'b1;            
        end
        else if(last_out && fire_out ) begin
            first_beat_r <= 1'b1;                 //stream_remove是用输入fire_in来控制，而stream_insert不行
        end                                       //为什么？
        else if(fire_out) begin
            first_beat_r <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            extra_last_beat_r <= 1'b0;
        end
        else if (has_extra_beat && last_in && fire_in) begin
            extra_last_beat_r <= 1'b1;
        end
        else if (fire_out) begin
            extra_last_beat_r <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            valid_in_r <= 1'b0;
            data_in_r  <= 'b0;
            keep_in_r  <= 'b0;
        end
        else begin
            if(fire_out) begin
                valid_in_r <= 1'b0;
            end
            if (fire_in) begin      //不能&&fire_insert，因为规定insert数据在一组数据传输完才发生变化，然而fire_in会fire多次
                valid_in_r <= valid_in;
                data_in_r  <= data_in;
                keep_in_r  <= keep_in;
        end
        end
    end

endmodule