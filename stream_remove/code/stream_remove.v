module stream_remove #(
    parameter DATA_WD = 32,           //data数据有多少位
    parameter DATA_BYTE_WD = DATA_WD / 8,   //keep有效位的宽度
    parameter BYTE_CNT_WD = $clog2(DATA_BYTE_WD)   //remove的宽度
) (
    input                         clk,
    input                         rst_n,

    input                         valid_in,
    input [DATA_WD - 1 : 0]       data_in,
    input [DATA_BYTE_WD - 1 : 0]  keep_in, 
    input                         last_in,
    output                        ready_in,

    output                        valid_out,
    output [DATA_WD - 1 : 0]      data_out,  
    output [DATA_BYTE_WD - 1 : 0] keep_out,
    output                        last_out, 
    input                         ready_out,

    input                         valid_remove,
    input [BYTE_CNT_WD - 1 : 0]   byte_remove_cnt,
    output                        ready_remove
);

    localparam DATA_BIT_WD = $clog2(DATA_WD); //data位的宽度

    reg                        valid_in_r;
    reg [DATA_WD - 1 : 0]      data_in_r;
    reg [DATA_BYTE_WD - 1 : 0] keep_in_r;

    wire fire_in = valid_in && ready_in;
    wire fire_out = valid_out && ready_out;
    wire fire_remove = valid_remove && ready_remove;

    reg first_beat_r;
    reg extra_last_beat_r;

    wire [DATA_BYTE_WD - 1 : 0]   keep_nothing = {DATA_BYTE_WD{1'b0}};
    wire [2*DATA_WD - 1 : 0]      double_data = {data_in_r, data_in};
    wire [2*DATA_BYTE_WD - 1 : 0] double_keep = extra_last_beat_r ? 
                                                {keep_in_r , keep_nothing} : 
                                                {keep_in_r, keep_in};

    wire [BYTE_CNT_WD : 0] data_byte_cnt = DATA_BYTE_WD;
    wire [BYTE_CNT_WD : 0] right_shift_byte_cnt = data_byte_cnt - byte_remove_cnt;
    wire [DATA_BIT_WD : 0] right_shift_bit_cnt = right_shift_byte_cnt << 3;

    wire [DATA_BYTE_WD - 1 : 0] next_beat_keep = keep_in << byte_remove_cnt;
    wire                        has_extra_beat = |next_beat_keep;

    assign  data_out = double_data >> right_shift_bit_cnt;
    assign  keep_out = double_keep >> right_shift_byte_cnt;

    assign ready_in = !valid_in_r || (ready_out && valid_remove); // 第一拍的时候/当输出ready并且remove的位数已经valid
    assign valid_out = extra_last_beat_r ? valid_in_r : (valid_in_r && valid_in && valid_remove); // 当有额外最后一拍的时候给到上一拍剩下的valid_r
    assign last_out = extra_last_beat_r || (last_in && !has_extra_beat); // 有额外最后一拍/输入最后一拍并且没用额外下一拍
    assign ready_remove = last_out && fire_out; // 输出最后一拍并且fire时可以更改需要remove的位数，迎接下一个data数据

    //always块再深入理解写一遍
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            first_beat_r <= 1'b1;
        end
        else if(fire_in && last_in) begin          //first_beat由输入决定
            first_beat_r <= 1'b1;
        end
        else if(fire_in) begin
            first_beat_r <= 1'b0;
        end
    end                                 //其在fire_in时发生变化，在一个fire周期内保持不变
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            extra_last_beat_r <= 1'b0;
        end
        else if(fire_in && last_in) begin
            extra_last_beat_r <= has_extra_beat;
        end
        else if(fire_in) begin
            extra_last_beat_r <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            valid_in_r <= 1'b0;
            data_in_r <= 'b0;
            keep_in_r <= 'b0;
        end
        else begin
            if(fire_out) begin
                valid_in_r <= 1'b0;
            end
            if(fire_in) begin
                valid_in_r <= has_extra_beat;  // valid_in_r要不要拉高取决于下一拍要不要用到这一拍的数据，所以由has_extra_beat决定
                data_in_r <= data_in;
                keep_in_r <= keep_in;
            end
        end
    end

endmodule