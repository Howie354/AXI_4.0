module axi_master_w #(
    parameter ADDR_WD = 32,
    parameter DATA_WD = 32,
    parameter STRB_WD = DATA_WD / 8
) (
    input clk,
    input rst_n,
    
    input                    w_cmd_valid,
    input  [1 : 0]           w_cmd_burst, //Burst类型选择
    input  [2 : 0]           w_cmd_size,  //每一拍发多少byte（一般为总线位宽）
    input  [ADDR_WD - 1 : 0] w_cmd_addr,  //起始地址
    input  [ADDR_WD - 1 : 0] w_cmd_len,   //表示要传输的字节数（不是拍数），real_len = w_cmd_len + 1
    output                   w_cmd_ready,

    output                   M_AXI_AWVALID,
    output [1 : 0]           M_AXI_AWBURST,
    output [2 : 0]           M_AXI_AWSIZE,
    output [ADDR_WD - 1 : 0] M_AXI_AWADDR,
    output [7 : 0]           M_AXI_AWLEN, //考虑INCR和FIXED情况，INCR（0~255） FIXED（0~15） WRAP（ 1 3 7 15）
    input                    M_AXI_AWREADY,

    
    output                   M_AXI_WVALID,
    output [STRB_WD - 1 : 0] M_AXI_WSTRB,
    output                   M_AXI_WLAST, //写数据通道的一个burst发送完毕结束信号
    output [DATA_WD - 1 : 0] M_AXI_WDATA, //读通道的返回数据
    input                    M_AXI_WREADY,

    input                    M_AXI_BVALID, //一组burst信号发送完才会收到BVALID
    input [1 : 0]            M_AXI_BRESP, //反馈信号，00（OK） ； 01（exclusive OK） ；10、11 （error）
    output                   M_AXI_BREADY
);
    localparam OPT_UNALIGNED = 1'b0;

    localparam FIXED = 2'b00;
    localparam INCR  = 2'b10;
    localparam WRAP  = 2'b11;

    localparam ADDRLSB         = $clog2(DATA_WD / 8);
    //判断每一拍发多少byte的数据，因为需要做地址对齐，所以ADDRLSB为不需要的后n位
    //DATA_WD = 32，ADDRLSB = 2，每一拍发4byte数据，则ADDR的后两位为least significant bits，后面做清零操作
    localparam TEMP_LGMAXBURST = $clog2(4096 * 8 / DATA_WD);
    //判断在4K边界问题中，最大的burst能发送的拍数位宽（即发送4k byte需要的拍数），用log空间可以节省面积
    //若DATA_WD = 32，TEMP_LGMAXBURST = 10，即最多一次BURST可以发2^10即1024拍
    //若DATA_WD = 256,那么$clog2(4096/32) = $clog2(128) = 7，则LGMAXBURST只能取7，否则一个burst就超4k边界
    localparam LGMAXBURST      = (TEMP_LGMAXBURST > 8) ? 8 : TEMP_LGMAXBURST;
    //BURST在INCR模式下一次BURST最多传输256 = 2^8拍，所以TEMP_LGMAXBURST需要和8进行比较，如果小于256拍，则取TEMP
    localparam LGMAX_FIXED_BURST = (LGMAXBURST > 4) ? 4 : LGMAXBURST;
    //FIXED模式下一次BURST最多传输16 = 2^4拍，不用TEMP_LGMAXBURST进行比较的原因可能是TEMP已经在LGMAXBURST中与8比较过了，
    //接下来只需要用LGMAXBURST和4比较就够了，比较省面积（TEMP的位数较多）
    localparam MAX_FIXED_BURST   = 1 << LGMAX_FIXED_BURST;
    //FIXED模式下最多可以传输的BURST拍数
    localparam LGLEN             = ADDR_WD;
    //CMD给到的最大字节数（log空间）
    localparam LGLENT            = ADDR_WD - ADDRLSB;
    //剪除掉最不重要的位数，节省面积，即CMD给到的最大拍数（的位宽）

    //对M_AXI_ARLEN做三拍处理
    //第一拍：确定是否要对4k问题进行特殊处理
    //第二拍：确定下一个burst要发多少拍，以及outstanding问题
    //第三拍：对得到的aw_len做-1操作

    //---Update when w_cmd_fire（第一拍）---
    reg [ADDR_WD - 1 : 0]    aw_addr; //CMD传输过来的地址
    reg [1 : 0]              aw_burst; //CMD传输过来的Burst类型
    reg [LGLENT - 1 : 0]     aw_len_t; //不是一次的BURST，是要总共发的拍数，可能是多个BURST进行，real_len（byte） = aw_len_t << ADDRLSB
    reg [2 : 0]              aw_size; //一拍发多少多少字节，一般为总线的数据位宽

    //控制信号
    reg                      w_busy; //w_cmd_fire后，busy拉高，进入繁忙状态
    reg                      aw_incr_burst; //BURST是否为INCR类型，若是置1
    reg                      aw_zero_len; //是否0字节要发，若是置1
    reg                      aw_multi_full_bursts; //INCR是否有多个满拍的BURST传输
    reg                      aw_multi_fixed_bursts; //FIXED是否有多个满拍的BURST传输
    reg                      aw_need_4K_design; //是否要进行4K问题的特殊处理

    reg [LGMAXBURST : 0]     initial_burst_len_comb; //第一个burst需要发多少拍的计算，comb代表是组合逻辑生成
    reg [LGMAXBURST - 1 : 0] addr_align_burst_comb;  //如果要处理4K问题，第一个burst需要补多少拍，才可以让地址符合4K边界的要求

    //---update when the first beat of w_busy and phantom_start（第二拍）---
    reg                      w_pre_start; //当r_busy拉高后，下一拍r_pre_start拉低，用来显示r_busy的第一拍，并形成w_phantom_start_comb

    reg [LGLENT - 1 : 0]     aw_requests_remaining; //剩下多少拍要发
    reg [LGLENT - 1 : 0]     aw_next_requests_remaining_comb; //组合逻辑计算下一次的剩余拍数，在phantom_start或r_busy的第一拍时赋给寄存器

    reg                      aw_multi_full_bursts_remaining; //INCR模式中是否还有剩余满拍BURST
    reg                      aw_next_multi_full_bursts_remaining_comb;
    reg                      aw_none_incr_burst; //INCR模式中是否没有数据要发了

    reg                      aw_multi_fixed_bursts_remaining; //FIXED模式中是否还有剩余满拍BURST
    reg                      aw_next_multi_fixed_bursts_remaining_comb;
    reg                      aw_none_fixed_burst; //FIXED模式中是否没有数据要发了

    reg [LGMAXBURST : 0]     aw_next_burst; //下一个burst要发多少拍

    //---update when w_phantom_start_comb（第三拍）---
    reg [LGMAXBURST - 1 : 0] axi_awlen;   //aw_next_burst做减一操作后赋值给axi_awlen
   
    //update when WFIRE
    reg [ADDR_WD - 1 : 0] wr_beat_awaddr; //记录要发的起始地址
    
    //update when phantom_start or WFIRE
    reg [LGMAXBURST : 0] wr_beats_pending; //写数据通道计数器，记录本次burst发送了多少拍信号
    reg                  wr_none_pending;  //若无信号剩余，拉高

    //输出信号和辅助信号
    reg                   phantom_start; //后续发送过程中第一个处理信号
    reg                   w_complete_comb; //所有burst信号发送完毕
    reg                   w_phantom_start_comb;
    reg                   w_cmd_fire;
    
    reg                   axi_awvalid; //axi开头为输出信号的reg类型。
    reg [1 : 0]           axi_awburst;
    reg [2 : 0]           axi_awsize;
    reg [ADDR_WD - 1 : 0] axi_awaddr;
    reg                   axi_abort_pending; //中止等待（发生错误的情况）

    reg                   axi_wvalid;
    reg [DATA_WD - 1 : 0] axi_wdata;
    reg [STRB_WD - 1 : 0] axi_wstrb;
    reg                   axi_wlast;
    reg                   axi_bready;
    
    //------------------------------------------------------------------------
    //处理打断组合逻辑的标识信号

    //w_cmd_fire
    always @(*) begin
        w_cmd_fire = w_cmd_valid && w_cmd_ready;
    end

    //w_busy
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            w_busy <= 1'b0;
        end
        else if(w_cmd_fire) begin
            w_busy <= 1'b1;
        end
        else if(w_complete_comb) begin
            w_busy <= 1'b0;
        end
    end

    //w_pre_start
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            w_pre_start <= 1'b1;
        end
        else if (w_busy) begin
            w_pre_start <= 1'b0;
        end
        else if(!w_busy) begin
            w_pre_start <= 1'b1;
        end
    end

    //w_complete_comb
    always @(*) begin
        w_complete_comb = (M_AXI_WVALID && M_AXI_WREADY && M_AXI_WLAST) && (aw_incr_burst ? aw_none_incr_burst : aw_none_fixed_burst);
    end

    //w_phantom_start_comb
    always @(*) begin
        w_phantom_start_comb = ~(aw_incr_burst ? aw_none_incr_burst : aw_none_fixed_burst); //当剩余有数据要发的情况下，置1，代表可以下一拍拉高valid
        if(!w_busy || axi_abort_pending) begin //不busy和出错的情况下，置0
            w_phantom_start_comb = 1'b0;
        end
        if(w_pre_start || phantom_start) begin //w_pre_start和axi_awvalid拉高的当拍，置0
            w_phantom_start_comb = 1'b0;
        end
        if(M_AXI_AWVALID && !M_AXI_AWREADY) begin //写地址通道阻塞时，置0
            w_phantom_start_comb = 1'b0;
        end
        if(M_AXI_WVALID && (!M_AXI_WLAST || !M_AXI_WREADY)) begin //当写数据没发完或者写数据通道阻塞时，置0
            w_phantom_start_comb = 1'b0;
        end
    end

    //phantom_start
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            phantom_start <= 1'b0;
        end
        else begin
            phantom_start <= w_phantom_start_comb; //相当于将w_phantom_start_comb延迟一拍
        end
    end

    //---Update when w_cmd_fire（第一拍）---
    // reg [ADDR_WD - 1 : 0]    aw_addr; //CMD传输过来的地址
    // reg [1 : 0]              aw_burst; //CMD传输过来的Burst类型
    // reg [LGLENT - 1 : 0]     aw_len_t; //不是一次的BURST，是要总共发的拍数，可能是多个BURST进行，real_len（byte） = aw_len_t << ADDRLSB
    // reg [2 : 0]              aw_size; //一拍发多少多少字节，一般为总线的数据位宽

    //控制信号
    // reg                      w_busy; //w_cmd_fire后，busy拉高，进入繁忙状态
    // reg                      aw_incr_burst; //BURST是否为INCR类型，若是置1
    // reg                      aw_zero_len; //是否0字节要发，若是置1
    // reg                      aw_multi_full_bursts; //INCR是否有多个满拍的BURST传输
    // reg                      aw_multi_fixed_bursts; //FIXED是否有多个满拍的BURST传输
    // reg                      aw_need_4K_design; //是否要进行4K问题的特殊处理

    // reg [LGMAXBURST : 0]     initial_burst_len_comb; //第一个burst需要发多少拍的计算，comb代表是组合逻辑生成
    // reg [LGMAXBURST - 1 : 0] addr_align_burst_comb;  //如果要处理4K问题，第一个burst需要补多少拍，才可以让地址符合4K边界的要求

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            aw_addr               <= 'b0;
            aw_burst              <= FIXED;
            aw_len_t              <= 'b0;
            aw_size               <= 'b0;
            aw_incr_burst         <= 1'b0;
            aw_zero_len           <= 1'b1;
            aw_multi_full_bursts  <= 1'b0;
            aw_multi_fixed_bursts <= 1'b0;
        end
        else if(!w_busy) begin //不用w_cmd_fire，避免毛刺
            aw_addr               <= w_cmd_addr;
            aw_burst              <= w_cmd_burst;
            aw_len_t              <= w_cmd_len[ADDR_WD - 1 : ADDRLSB] + 1;//考虑如果r_cmd_len = 0110，实际为7字节，应该发两拍，而不是一拍
            aw_size               <= w_cmd_size;
            aw_incr_burst         <= (w_cmd_burst == INCR);
            aw_zero_len           <= (w_cmd_len[ADDR_WD - 1 : ADDRLSB] == 0);
            aw_multi_full_bursts  <= |w_cmd_len[ADDR_WD - 1 : LGMAXBURST];
            aw_multi_fixed_bursts <= |w_cmd_len[ADDR_WD - 1 : LGMAX_FIXED_BURST];
        end
        //如果出现错误，将剩余的数据记录下来
        else if(axi_abort_pending && (M_AXI_BVALID && M_AXI_BRESP[1])) begin
            aw_len_t              <= aw_requests_remaining;
            aw_zero_len           <= (aw_requests_remaining == 0);
            aw_multi_full_bursts  <= |aw_requests_remaining[LGLENT - 1 : LGMAXBURST];
            aw_multi_fixed_bursts <= |aw_requests_remaining[LGLENT - 1 : LGMAX_FIXED_BURST];
        end
    end

    //aw_need_4K_design
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            aw_need_4K_design <= 1'b0;
        end
        else if(!w_busy) begin
            aw_need_4K_design <= 1'b0; //初始地址与1k已经对齐的情况
            if(|w_cmd_addr[ADDRLSB +: LGMAXBURST]) begin //初始地址与1k不对齐
                if(|w_cmd_len[ADDR_WD - 1 : LGMAXBURST + ADDRLSB]) begin //要发送的数据字节数大于1k
                    aw_need_4K_design <= 1'b1;
                end
                //要发送的数据字节数小于1k，但是与地址加起来后与1k不对齐
                else if(w_cmd_addr[ADDRLSB +: LGMAXBURST] > ~w_cmd_len[ADDRLSB +: LGMAXBURST]) begin 
                    aw_need_4K_design <= 1'b1;
                end
            end
        end
    end

    //initial_burst_len_comb
    //addr_align_burst_comb 
    //计算如何处理第一次burst发多少拍
    always @(*) begin
        addr_align_burst_comb = ~w_cmd_addr[ADDRLSB +: LGMAXBURST] + 1;
        //r_cmd_addr[ADDRLSB +: LGMAXBURST] + (~r_cmd_addr[ADDRLSB +: LGMAXBURST] + 1) = 0 表示达到1K边界，这是因为补码的特性
        //如a = 011 补码'a = 101，a + 'a = (1)000，1进位上去了，所以在原位就表现为清零
        //addr_align_burst_comb代表发多少拍达到1K边界，即1K边界的余数
        initial_burst_len_comb = 1 << LGMAXBURST;
        if(!aw_incr_burst) begin //FIXED
            if(!aw_multi_fixed_bursts) begin
                initial_burst_len_comb = {1'b0,w_cmd_len[ADDRLSB +: LGMAX_FIXED_BURST]}; //此时aw_len_t还未被赋值，所以继续用w_cmd_len
            end
            else begin
                initial_burst_len_comb = MAX_FIXED_BURST;
            end
        end
        else begin //INCR
            if(!aw_multi_full_bursts) begin //没有满拍的情况
                initial_burst_len_comb = {1'b0,w_cmd_len[ADDRLSB +: LGMAXBURST]};
            end
            else if(aw_need_4K_design) begin //需要4k特殊处理
                initial_burst_len_comb = {1'b0,addr_align_burst_comb};
            end
            else begin
                initial_burst_len_comb = 1 << LGMAXBURST;
            end
        end
    end

    //---update when the first beat of w_busy and phantom_start（第二拍）---
    // reg                      w_pre_start; //当r_busy拉高后，下一拍r_pre_start拉低，用来显示r_busy的第一拍，并形成w_phantom_start_comb

    // reg [LGLENT - 1 : 0]     aw_requests_remaining; //剩下多少拍要发
    // reg [LGLENT - 1 : 0]     aw_next_requests_remaining_comb; //组合逻辑计算下一次的剩余拍数，在phantom_start或w_busy的第一拍时赋给寄存器

    // reg                      aw_multi_full_bursts_remaining; //INCR模式中是否还有剩余满拍BURST
    // reg                      aw_next_multi_full_bursts_remaining_comb;
    // reg                      aw_none_incr_burst; //INCR模式中是否没有数据要发了

    // reg                      aw_multi_fixed_bursts_remaining; //FIXED模式中是否还有剩余满拍BURST
    // reg                      aw_next_multi_fixed_bursts_remaining_comb;
    // reg                      aw_none_fixed_burst; //FIXED模式中是否没有数据要发了

    // reg [LGMAXBURST : 0]     aw_next_burst; //下一个burst要发多少拍

    //aw_next_requests_remaining_comb
    //aw_next_multi_full_bursts_remaining_comb
    //aw_next_multi_fixed_bursts_remaining_comb
    always @(*) begin
        aw_next_requests_remaining_comb           = aw_requests_remaining + {{(LGLENT - 8){phantom_start}},(phantom_start ? (~M_AXI_AWLEN + 1'b1) : 8'h00)};
        //aw_requests_remaining - M_AXI_AWLEN = aw_requests_remaining + ~M_AXI_AWLEN + 1'b1
        //要注意的是，因为对M_AXI_AWLEN求反，所以前面补全的0位也要取反，所以当phantom_start拉高时，前面LGLENT-8位都为1
        aw_next_multi_full_bursts_remaining_comb  = |aw_next_requests_remaining_comb[LGLENT - 1 : LGMAXBURST];
        aw_next_multi_fixed_bursts_remaining_comb = |aw_next_requests_remaining_comb[LGLENT - 1 : LGMAX_FIXED_BURST];
    end

    //aw_requests_remaining等
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            aw_requests_remaining           <= 'b0;
            aw_multi_full_bursts_remaining  <= 1'b0;
            aw_none_incr_burst              <= 1'b1;
            aw_multi_fixed_bursts_remaining <= 1'b0;
            aw_none_fixed_burst             <= 1'b1;
        end
        else if(w_pre_start) begin
            aw_requests_remaining           <= aw_len_t;
            aw_multi_full_bursts_remaining  <= |aw_len_t[LGLENT - 1 : LGMAXBURST];
            aw_none_incr_burst              <= aw_zero_len;
            aw_multi_fixed_bursts_remaining <= |aw_len_t[LGLENT - 1 : LGMAX_FIXED_BURST];
            aw_none_fixed_burst             <= aw_zero_len;
        end
        else if(phantom_start) begin
            aw_requests_remaining           <= aw_next_requests_remaining_comb;
            aw_multi_full_bursts_remaining  <= aw_next_multi_full_bursts_remaining_comb;
            aw_none_incr_burst              <= !aw_next_multi_full_bursts_remaining_comb && !(|aw_next_requests_remaining_comb[LGMAXBURST - 1 : 0]);
            aw_multi_fixed_bursts_remaining <= aw_next_multi_fixed_bursts_remaining_comb;
            aw_none_fixed_burst             <= !aw_next_multi_fixed_bursts_remaining_comb && !(|aw_next_requests_remaining_comb[LGMAX_FIXED_BURST - 1 : 0]);
        end
        else if(axi_abort_pending) begin
            aw_requests_remaining           <= 'b0;
            aw_multi_full_bursts_remaining  <= 1'b0;
            aw_none_incr_burst              <= 1'b1;
            aw_multi_fixed_bursts_remaining <= 1'b0;
            aw_none_fixed_burst             <= 1'b1;
        end
    end

    //aw_next_burst
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            aw_next_burst <= 'b0;
        end
        else if(w_pre_start) begin
            aw_next_burst <= initial_burst_len_comb;
        end
        else if(phantom_start) begin
            if(!aw_incr_burst) begin
                //判断条件都为next的，因为要赋值的为下一次burst的拍数
                if(!aw_next_multi_fixed_bursts_remaining_comb && (aw_next_requests_remaining_comb[LGMAX_FIXED_BURST - 1 : 0]) < MAX_FIXED_BURST) begin
                    aw_next_burst <= aw_next_requests_remaining_comb;
                end
                else begin
                    aw_next_burst <= MAX_FIXED_BURST;
                end
            end
            else begin
                if(!aw_next_multi_full_bursts_remaining_comb && (aw_next_requests_remaining_comb[LGMAXBURST - 1 : 0] < (1 << LGMAXBURST))) begin
                    aw_next_burst <= aw_next_requests_remaining_comb;
                end
                else begin
                    aw_next_burst <= (1 << LGMAXBURST);
                end
            end
        end
    end

    //---update when w_phantom_start_comb（第三拍）---
    // reg [LGMAXBURST - 1 : 0] axi_awlen;   //aw_next_burst做减一操作后赋值给axi_awlen

    //axi_awlen
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awlen <= 'b0;
        end
        else if (w_phantom_start_comb) begin // (! M_AXI_AWVALID || M_AXI_AWREADY) 
            axi_awlen <= aw_next_burst - 1'b1;
        end
    end

    //update when WFIRE
    // reg [ADDR_WD - 1 : 0] wr_beat_awaddr; //记录要发的起始地址
    
    //update when phantom_start or WFIRE
    // reg [LGMAXBURST : 0] wr_beats_pending; //写数据通道计数器，记录本次burst发送了多少拍信号
    // reg                      wr_none_pending;  //若无信号剩余，拉高

    //wr_beat_awaddr
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            wr_beat_awaddr <= 'b0;
        end
        else if(!axi_abort_pending || !aw_incr_burst) begin
            wr_beat_awaddr <= wr_beat_awaddr;
        end
        else if(w_pre_start) begin
            wr_beat_awaddr <= aw_addr;
        end
        else if(M_AXI_WVALID && M_AXI_WREADY) begin
            wr_beat_awaddr <= wr_beat_awaddr + (1 << ADDRLSB); //每一次地址+4，因为一拍发4个字节
        end
        else if(!OPT_UNALIGNED) begin
            wr_beat_awaddr[ADDRLSB - 1 : 0] <= 'b0;
        end
    end

    //wr_beats_pending
    //wr_none_pending
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            wr_beats_pending <= 'b0;
            wr_none_pending  <= 1'b1;
        end
        else begin
            case ({phantom_start , M_AXI_WVALID && M_AXI_WREADY})
                2'b00: begin
                    wr_beats_pending <= wr_beats_pending;
                    wr_none_pending  <= wr_none_pending;
                end
                2'b10: begin //写地址通道握手，写数据通道没有握手
                    wr_beats_pending <= wr_beats_pending + (M_AXI_AWLEN + 1'b1); //不能用aw_next_burst因为这个是计算的下一次burst的拍数，不是本次
                    wr_none_pending  <= 1'b0;
                end
                2'b01: begin //写数据通道握手，写地址通道没有握手
                    wr_beats_pending <= wr_beats_pending - 1'b1;
                    wr_none_pending  <= (wr_beats_pending == 1);
                end
                2'b11: begin //写地址和写数据通道同时握手，第一拍就发了数据
                    wr_beats_pending <= wr_beats_pending + M_AXI_AWLEN;
                    wr_none_pending  <= M_AXI_WLAST;
                end
            endcase
        end
    end

    //输出信号和辅助信号
    // reg                   phantom_start; //后续发送过程中第一个处理信号
    // reg                   w_complete_comb; //所有burst信号发送完毕
    // reg                   w_phantom_start_comb;
    // reg                   w_cmd_fire;
    
    // reg                   axi_awvalid; //axi开头为输出信号的reg类型。
    // reg [1 : 0]           axi_awburst;
    // reg [2 : 0]           axi_awsize;
    // reg [ADDR_WD - 1 : 0] axi_awaddr;
    // reg                   axi_abort_pending; //中止等待（发生错误的情况）

    // reg                   axi_wvalid;
    // reg [DATA_WD - 1 : 0] axi_wdata;
    // reg [STRB_WD - 1 : 0] axi_wstrb;
    // reg                   axi_wlast;
    // reg                   axi_bready;

    assign M_AXI_AWVALID = axi_awvalid;
    assign M_AXI_AWBURST = axi_awburst;
    assign M_AXI_AWSIZE  = axi_awsize;
    assign M_AXI_AWADDR  = axi_awaddr;
    assign M_AXI_AWLEN   = axi_awlen;
    
    assign M_AXI_WVALID  = axi_wvalid;
    assign M_AXI_WSTRB   = axi_wstrb;
    assign M_AXI_WDATA   = axi_wdata;
    assign M_AXI_WLAST   = axi_wlast;
    assign M_AXI_BREADY  = axi_bready;

    assign w_cmd_ready   = !w_busy && !axi_abort_pending;

    //axi_awvalid
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awvalid <= 1'b0;
        end
        else if(!M_AXI_AWVALID || M_AXI_AWREADY) begin
            axi_awvalid <= w_phantom_start_comb;
        end
    end

    //axi_awburst
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awburst <= FIXED;
        end
        else if(!M_AXI_AWVALID || M_AXI_AWREADY) begin //写地址通道不阻塞的情况，实际也是第三拍，即w_phantom_start_comb的情况
            axi_awburst <= aw_burst;
        end
    end

    //axi_awsize
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awsize <= 1'b0;
        end
        else if(!M_AXI_AWVALID || M_AXI_AWREADY) begin
            axi_awsize <= aw_size;
        end
    end

    //axi_awaddr
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awaddr <= 'b0;
        end
        else begin
            if(M_AXI_AWVALID && M_AXI_AWREADY) begin //每一次发送新的burst时
                axi_awaddr[ADDRLSB - 1 : 0] <= 'b0;
                if(aw_incr_burst) begin //INCR
                    axi_awaddr <= axi_awaddr + (M_AXI_AWLEN + 1'b1);
                end
                else begin
                    axi_awaddr <= axi_awaddr;
                end
            end
            if(w_pre_start) begin
                axi_awaddr <= aw_addr;
            end
            if(!OPT_UNALIGNED) begin
                axi_awaddr[ADDRLSB - 1 : 0] <= 'b0;
            end
        end
    end

    //axi_wvalid
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_wvalid <= 1'b0;
        end
        else if(!M_AXI_WVALID || M_AXI_WREADY) begin //不堵塞时才可以更改
            if(M_AXI_WVALID && !M_AXI_WLAST) begin //保证连续发送的情况
                axi_wvalid <= 1'b1;
            end
            else begin
                axi_wvalid <= w_phantom_start_comb;
            end
        end
    end

    //axi_wdata 一个简单的发送例子
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_wdata <= 'b0;
        end
        else if(M_AXI_WVALID && M_AXI_WREADY) begin //握手时，数据+1
            axi_wdata <= axi_wdata + 1'b1;
            if(M_AXI_WLAST) begin //到一次burst结束的时候，数据清零
                axi_wdata <= 'b0;
            end
        end
    end

    //axi_wstrb
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_wstrb <= 'b0;
        end
        else if(!M_AXI_WVALID || M_AXI_WREADY) begin
            axi_wstrb <= (axi_abort_pending ? 'b0 : -1);
            //4'b0001 + 4'b1111 = 0
            // 1 + -1 = 0
            // -1 = 4'b1111  
        end
    end

    //axi_wlast
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_wlast <= 'b0;
        end
        else if(!M_AXI_WVALID || M_AXI_WREADY) begin //要分第一个burst只发一拍和两拍情况讨论，
                                                     //因为wr_beats_pending只在phantom情况下变化，如果用wr_beats_pending判断就没法判断第一个burst的情况了
            if(w_phantom_start_comb) begin //假设写数据通道从第一拍就直接握手
                axi_wlast <= (aw_next_burst == 1'b1); //只有一拍数据要发的情况，这里可以用aw_next_burst因为其在pre_start就被赋值
            end
            else if(phantom_start) begin
                axi_wlast <= (M_AXI_AWLEN == 1'b1); //这里也可以用aw_next_burst，因为下一拍aw_next_burst才被赋新值
            end
            else begin //其余情况，写数据通道计数器为2即可
                axi_wlast <= wr_beats_pending == 2; 
            end
        end
    end

    //axi_bready
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_bready <= 1'b0;
        end
        else  begin //if并列放置是因为存在写数据通道和写反馈通道同时握手的情况，axi_bready置1放在下面是为了效率最大化
            if(M_AXI_BVALID && M_AXI_BREADY) begin
                axi_bready <= 1'b0;
            end
            if (M_AXI_WVALID && M_AXI_WREADY && M_AXI_WLAST) begin
                axi_bready <= 1'b1;
            end
        end
    end

    
endmodule