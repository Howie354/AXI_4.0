module axi_master_r #(
    parameter ADDR_WD = 32,
    parameter DATA_WD = 32,
    parameter STRB_WD = ADDR_WD / 8
    )(
    input clk,
    input rst_n,
    
    input                    r_cmd_valid,
    input  [1 : 0]           r_cmd_burst, //Burst类型选择
    input  [2 : 0]           r_cmd_size,  //每一拍发多少byte（一般为总线位宽）
    input  [ADDR_WD - 1 : 0] r_cmd_addr,  //起始地址
    input  [ADDR_WD - 1 : 0] r_cmd_len,   //表示要传输的字节数（不是拍数），real_len = r_cmd_len + 1
    output                   r_cmd_ready,

    output                   M_AXI_ARVALID,
    output [1 : 0]           M_AXI_ARBURST,
    output [2 : 0]           M_AXI_ARSIZE,
    output [ADDR_WD - 1 : 0] M_AXI_ARADDR,
    output [7 : 0]           M_AXI_ARLEN, //考虑INCR和FIXED情况，INCR（0~255） FIXED（0~15） WRAP（ 1 3 7 15）
    input                    M_AXI_ARREADY,

    input                    M_AXI_RVALID,
    input                    M_AXI_RLAST, //读通道的一个burst读取完毕结束信号
    input [DATA_WD - 1 : 0]  M_AXI_RDATA, //读通道的返回数据
    input [1 : 0]            M_AXI_RRESP, //反馈信号，00（OK） ； 01（exclusive OK） ；10、11 （error）
    output                   M_AXI_RREADY

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
    //CMD给到的要发的字节数（位宽）
    localparam LGLENT            = ADDR_WD - ADDRLSB;
    //剪除掉最不重要的位数，节省面积，即CMD给到的能发的最大拍数（的位宽）
    //不是一次的BURST，是要总共发的，可能是多个BURST进行，并且可以采用read_outstanding

    //对M_AXI_ARLEN做三拍处理
    //第一拍：确定是否要对4k问题进行特殊处理
    //第二拍：确定下一个burst要发多少拍，以及outstanding问题
    //第三拍：对得到的ar_len做-1操作

    //定义参数
    //---Update when r_cmd_fire（第一拍）---
    reg [ADDR_WD - 1 : 0]    ar_addr; //CMD传输过来的地址
    reg [1 : 0]              ar_burst; //CMD传输过来的Burst类型
    reg [LGLENT - 1 : 0]     ar_len_t; //不是一次的BURST，是要总共发的拍数，可能是多个BURST进行，real_len（byte） = ar_len_t << ADDRLSB
    reg [2 : 0]              ar_size; //一拍发多少多少字节，一般为总线的数据位宽

    //控制信号
    reg                      r_busy; //r_cmd_fire后，busy拉高，进入繁忙状态
    reg                      ar_incr_burst; //BURST是否为INCR类型，若是置1
    reg                      ar_zero_len; //是否0字节要发，若是置1
    reg                      ar_multi_full_bursts; //INCR是否有多个满拍的BURST传输
    reg                      ar_multi_fixed_bursts; //FIXED是否有多个满拍的BURST传输
    reg                      ar_need_4K_design; //是否要进行4K问题的特殊处理

    reg [LGMAXBURST : 0]     initial_burst_len_comb; //第一个burst需要发多少拍的计算，comb代表是组合逻辑生成
    reg [LGMAXBURST - 1 : 0] addr_align_burst_comb;  //如果要处理4K问题，第一个burst需要补多少拍，才可以让地址符合4K边界的要求

    //---update when the first beat of r_busy and phantom_start（第二拍）---
    reg                      r_pre_start; //当r_busy拉高后，下一拍r_pre_start拉低，用来显示r_busy的第一拍，并形成start_burst

    reg [LGLENT - 1 : 0]     ar_requests_remaining; //剩下多少拍要发
    reg [LGLENT - 1 : 0]     ar_next_requests_remaining_comb; //组合逻辑计算下一次的剩余拍数，在phantom_start或r_busy的第一拍时赋给寄存器

    reg                      ar_multi_full_bursts_remaining; //INCR模式中是否还有剩余满拍BURST
    reg                      ar_next_multi_full_bursts_remaining_comb;
    reg                      ar_none_incr_burst; //INCR模式中是否没有数据要发了

    reg                      ar_multi_fixed_bursts_remaining; //FIXED模式中是否还有剩余满拍BURST
    reg                      ar_next_multi_fixed_bursts_remaining_comb;
    reg                      ar_none_fixed_burst; //FIXED模式中是否没有数据要发了

    reg [LGMAXBURST : 0]     ar_next_burst; //下一个burst要发多少拍

    //---update when start_burst_comb（第三拍）--
    reg                      axi_arvalid; //axi开头为输出信号的reg类型
    reg [LGMAXBURST - 1 : 0] axi_arlen;   //ar_next_burst做减一操作后赋值给axi_arlen
    reg                      phantom_start; //第二拍的指示信号

    reg [1 : 0]           axi_arburst;
    reg [2 : 0]           axi_arsize;
    reg [ADDR_WD - 1 : 0] axi_araddr;
    reg                   axi_abort_pending; //中止等待（发生错误的情况）
    reg                   axi_rready;

    //only update when phantom start or M_AXI_RLAST
    reg [LGLENT - 1 : 0] ar_burst_outstanding; //下一拍还剩几个burst信号没有传输
    reg                  ar_last_outstanding;  //下一拍为最后一个burst信号
    reg                  ar_none_outstanding;  //下一拍没有burst信号了

    reg                  r_complete_comb; //一次数据（所有BURST发送完毕）传输结束
    
    //辅助信号
    reg                   r_cmd_fire_comb;
    reg                   start_burst_comb; //第三拍的表示信号

    //------------------------------------------------------------------------
    //处理打断组合逻辑的标识信号

    //r_cmd_fire_comb
    always @(*) begin
        r_cmd_fire_comb = r_cmd_valid && r_cmd_ready;
    end

    //r_busy
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_busy <= 1'b0;
        end
        else if (r_cmd_fire_comb) begin
            r_busy <= 1'b1;
        end
        else if (r_complete_comb) begin
            r_busy <= 1'b0;
        end
    end

    //r_complete_comb
    always @(*) begin
        if(!r_busy) begin
            r_complete_comb = 1'b0;
        end
        else begin
            r_complete_comb = (M_AXI_RVALID && M_AXI_RREADY && M_AXI_RLAST) && ar_none_outstanding && r_busy;
        end
    end
    
    //r_pre_start
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_pre_start <= 1'b1;
        end
        else if(r_busy) begin
            r_pre_start <= 1'b0;
        end
        else if (!r_busy) begin
            r_pre_start <= 1'b1;
        end
    end

    //start_burst_comb
    always @(*) begin
        start_burst_comb = ~(ar_incr_burst ? ar_none_incr_burst : ar_none_fixed_burst);
        if(!r_busy || r_pre_start) begin
            start_burst_comb = 1'b0;
        end
        if(phantom_start || axi_abort_pending) begin
            start_burst_comb = 1'b0;
        end
        // if(M_AXI_ARVALID && !M_AXI_ARREADY) begin
        //     start_burst_comb = 1'b0;
        // end
        //当阻塞时，应该让start_burst_comb持续拉高，从而维持axi_arvalid持续拉高的状态，等待握手
    end

    //phantom_start
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            phantom_start <= 1'b0;
        end
        else begin
            phantom_start <= start_burst_comb;
        end
    end

    //------------------------------------------------------------------------
    //开始处理ar_len_t数据，即每一拍需要处理的信号们

    //---Update when r_cmd_fire（第一拍）---
    // reg [ADDR_WD - 1 : 0]    ar_addr; //CMD传输过来的地址
    // reg [1 : 0]              ar_burst; //CMD传输过来的Burst类型
    // reg [LGLENT - 1 : 0]     ar_len_t; //不是一次的BURST，是要总共发的，可能是多个BURST进行，real_len（byte） = ar_len_t << ADDRLSB
    // reg [2 : 0]              ar_size; //一拍发多少多少字节，一般为总线的数据位宽

    //控制信号
    // reg                      r_busy; //r_cmd_fire后，busy拉高，进入繁忙状态
    // reg                      ar_incr_burst; //BURST是否为INCR类型，若是置1
    // reg                      ar_zero_len; //是否0字节要发，若是置1
    // reg                      ar_multi_full_bursts; //INCR是否有多个满拍的BURST传输
    // reg                      ar_multi_fixed_bursts; //FIXED是否有多个满拍的BURST传输
    // reg                      ar_need_4K_design; //是否要进行4K问题的特殊处理
    
    // reg [LGMAXBURST : 0]     initial_burst_len_comb; //第一个burst需要发多少拍的计算，comb代表是组合逻辑生成
    // reg [LGMAXBURST - 1 : 0] addr_align_burst_comb;  //如果要处理4K问题，第一个burst需要补多少拍，才可以让地址符合4K边界的要求
    
    //ar_need_4K_design 是否要进行4K问题的特殊处理
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ar_need_4K_design <= 1'b0;
        end
        else begin
            if(!r_busy) begin
                ar_need_4K_design <= 1'b0;    
            end
            if(|r_cmd_addr[ADDRLSB +: LGMAXBURST]) begin //确定起始地址是否以1k对齐
                if(|r_cmd_len[ADDR_WD - 1 : (LGMAXBURST + ADDRLSB)]) begin
                    ar_need_4K_design <= 1'b1;
                end
                //r_cmd_addr[ADDRLSB +: LGMAXBURST] + (r_cmd_len[ADDRLSB +: LGMAXBURST] + 1) > 0 
                //↑表示地址经过所有的burst之后的新地址不与1k对齐，类似于3.5k + 600这种情况，第一个if条件无法判断
                //根据补码知识可知 r_cmd_len[ADDRLSB +: LGMAXBURST] + （~r_cmd_len[ADDRLSB +: LGMAXBURST] + 1） = 0
                //则r_cmd_len[ADDRLSB +: LGMAXBURST] + 1 = -~r_cmd_len[ADDRLSB +: LGMAXBURST]
                //故化简为r_cmd_addr[ADDRLSB +: LGMAXBURST] > ~r_cmd_len[ADDRLSB +: LGMAXBURST]
                else if (r_cmd_addr[ADDRLSB +: LGMAXBURST] > ~r_cmd_len[ADDRLSB +: LGMAXBURST]) begin
                    ar_need_4K_design <= 1'b1;
                end
            end
        end
    end

    //initial_burst_len_comb[LGMAXBURST : 0] 第一个burst需要发多少拍的计算
    //addr_align_burst_comb[LGMAXBURST - 1 : 0]  如果要处理4K问题，第一个burst需要补多少拍
    always @(*) begin
        addr_align_burst_comb = ~r_cmd_addr[ADDRLSB +: LGMAXBURST] + 1;
        //r_cmd_addr[ADDRLSB +: LGMAXBURST] + (~r_cmd_addr[ADDRLSB +: LGMAXBURST] + 1) = 0 表示达到1K边界，这是因为补码的特性
        //如a = 011 补码'a = 101，a + 'a = (1)000，1进位上去了，所以在原位就表现为清零
        //addr_align_burst_comb代表发多少拍达到1K边界，即1K边界的余数
        initial_burst_len_comb = 1 << LGMAXBURST;
        if(!ar_incr_burst) begin
            if(!ar_multi_fixed_bursts) begin
                initial_burst_len_comb = {1'b0,ar_len_t[LGMAX_FIXED_BURST - 1 : 0]}; //FIXED模式下，总共的字节数不够发一个满拍的情况    
            end
            else begin
                initial_burst_len_comb = MAX_FIXED_BURST; 
            end
        end
        else if(ar_need_4K_design) begin
            initial_burst_len_comb = {1'b0,addr_align_burst_comb};
        end
        else if(!ar_multi_full_bursts) begin
            initial_burst_len_comb = {1'b0,ar_len_t[LGMAXBURST - 1 : 0]};//INCR模式下，总共的字节数不够发一个满拍的情况
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ar_addr               <= 'b0;
            ar_burst              <= FIXED;
            ar_len_t              <= 'b0;
            ar_size               <= 'b0;
            ar_incr_burst         <= 'b0;
            ar_zero_len           <= 'b0;
            ar_multi_full_bursts  <= 'b0;
            ar_multi_fixed_bursts <= 'b0;
        end
        else if(!r_busy) begin //r_cmd_fire_comb是组合逻辑，避免毛刺，尽量使用寄存器的值
                               //r_cmd_fire_comb那一拍赋值进去之后在busy拉高的期间就不再改变了
            ar_addr               <= r_cmd_addr;
            ar_burst              <= r_cmd_burst;
            ar_len_t              <= r_cmd_len[ADDR_WD - 1 : ADDRLSB] + 1; //考虑如果r_cmd_len = 0110，实际为7字节，应该发两拍，而不是一拍
            ar_size               <= r_cmd_size;
            ar_incr_burst         <= (r_cmd_burst == INCR);
            ar_zero_len           <= (r_cmd_len[ADDR_WD - 1 : ADDRLSB] == 0); 
            //因为一拍是4个字节数据，所以当[31 ：2]为0时，哪怕低两位是11，但是由于会做对齐处理，所以也会清零，说明一拍都不够发，也就是没有数据
            ar_multi_full_bursts  <= (|r_cmd_len[ADDR_WD - 1 : LGMAXBURST + ADDRLSB]);
            ar_multi_fixed_bursts <= (|r_cmd_len[ADDR_WD - 1 : LGMAX_FIXED_BURST + ADDRLSB]);
        end
        else if(!axi_abort_pending && (M_AXI_RVALID && M_AXI_RRESP[1])) begin
            //代表着在一个burst传输结束后出错，这时需要将没处理完的数据先存储起来
            ar_zero_len           <= (ar_requests_remaining == 0);
            ar_len_t              <= ar_requests_remaining;
            ar_multi_full_bursts  <= (|ar_requests_remaining[LGLENT - 1 : LGMAXBURST]);
            ar_multi_fixed_bursts <= (|ar_requests_remaining[LGLENT - 1 : LGMAX_FIXED_BURST]);
        end
    end

    //---update when the first beat of r_busy and phantom_start（第二拍）---
    // reg [LGLENT - 1 : 0]     ar_requests_remaining; //剩下多少拍要发
    // reg [LGLENT - 1 : 0]     ar_next_requests_remaining_comb; //组合逻辑计算下一次的剩余拍数，在phantom_start或r_busy的第一拍时赋给寄存器

    // reg                      ar_multi_full_bursts_remaining; //INCR模式中是否还有剩余满拍BURST
    // reg                      ar_next_multi_full_bursts_remaining_comb;
    // reg                      ar_none_incr_burst; //INCR模式中是否没有数据要发了

    // reg                      ar_multi_fixed_bursts_remaining; //FIXED模式中是否还有剩余满拍BURST
    // reg                      ar_next_multi_fixed_bursts_remaining_comb;
    // reg                      ar_none_fixed_burst; //FIXED模式中是否没有数据要发了

    // reg [LGMAXBURST : 0]     ar_next_burst; //下一个burst要发多少拍
    

    //ar_next_requests_remaining_comb
    //ar_next_multi_full_bursts_remaining_comb
    //ar_next_multi_fixed_bursts_remaining_comb
    always @(*) begin
        ar_next_requests_remaining_comb           = ar_requests_remaining + {{(LGLENT - 8){phantom_start}},(phantom_start ? (~M_AXI_ARLEN + 1'b1) : 8'h00)};
        //ar_requests_remaining - M_AXI_ARLEN = ar_requests_remaining + ~M_AXI_ARLEN + 1'b1
        //要注意的是，因为对M_AXI_ARLEN求反，所以前面补全的0位也要取反，所以当phantom_start拉高时，前面LGLENT-8位都为1
        ar_next_multi_full_bursts_remaining_comb  = |ar_next_requests_remaining_comb[LGLENT - 1 : LGMAXBURST];
        ar_next_multi_fixed_bursts_remaining_comb = |ar_next_requests_remaining_comb[LGLENT - 1 : LGMAX_FIXED_BURST];
    end

    //ar_requests_remaining及控制信号
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ar_requests_remaining           <= 'b0;
            ar_multi_full_bursts_remaining  <= 'b0;
            ar_none_incr_burst              <= 'b0;
            ar_multi_fixed_bursts_remaining <= 'b0;
            ar_none_fixed_burst             <= 'b0;
        end
        else if(r_pre_start) begin //第一个burst的情况，用ar_len_t进行判断
            ar_requests_remaining           <= ar_len_t;
            ar_multi_full_bursts_remaining  <= |ar_len_t[LGLENT - 1 : LGMAXBURST];
            ar_none_incr_burst              <= ar_zero_len;
            ar_multi_fixed_bursts_remaining <= |ar_len_t[LGLENT - 1 : LGMAX_FIXED_BURST];
            ar_none_fixed_burst             <= ar_zero_len;
        end
        else if(phantom_start) begin //剩余burst的情况，用ar_requests_remaining进行判断
            ar_requests_remaining           <= ar_next_requests_remaining_comb;
            ar_multi_full_bursts_remaining  <= ar_next_multi_full_bursts_remaining_comb;
            ar_none_incr_burst              <= !ar_next_multi_full_bursts_remaining_comb && (ar_requests_remaining[LGMAXBURST - 1 : 0] == 0);
            ar_multi_fixed_bursts_remaining <= ar_next_multi_fixed_bursts_remaining_comb;
            ar_none_fixed_burst             <= !ar_next_multi_fixed_bursts_remaining_comb && (ar_requests_remaining[LGMAX_FIXED_BURST - 1 : 0] == 0);
            // 表示在fixed模式下之后没有满拍burst要发，并且剩余要发的拍数的低位也为0，表明也没有小于满拍的残余拍要发，即FIXED模式中没有数据要发了
        end
        else if(axi_abort_pending) begin //中断
            ar_requests_remaining           <= 'b0;
            ar_multi_full_bursts_remaining  <= 1'b0;
            ar_none_incr_burst              <= 1'b1;
            ar_multi_fixed_bursts_remaining <= 1'b0;
            ar_none_fixed_burst             <= 1'b1;
        end
    end

    //ar_next_burst 下一个burst要发多少拍
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ar_next_burst <= 'b0;
        end
        else if(r_pre_start) begin //第一拍的情况
            ar_next_burst <= initial_burst_len_comb;
        end
        else if(phantom_start) begin
                if(!ar_incr_burst) begin //FIXED模式
                    if(!ar_next_multi_fixed_bursts_remaining_comb && (ar_requests_remaining[LGMAX_FIXED_BURST - 1 : 0] < MAX_FIXED_BURST)) begin
                        ar_next_burst <= ar_next_requests_remaining_comb; //ar_next_busrt是真实发的拍数,不用补0，因为ar_next..的宽度比ar_next_burst更大
                    end
                    else begin
                        ar_next_burst <= MAX_FIXED_BURST; //满拍的情况
                    end                                                          
                end
                else begin //INCR模式
                    if(!ar_next_multi_full_bursts_remaining_comb && (ar_requests_remaining[LGMAXBURST - 1 : 0] < (1 << LGMAXBURST))) begin
                        ar_next_burst <= ar_next_requests_remaining_comb; //ar_next_busrt是真实发的拍数,不用补0，因为ar_next..的宽度比ar_next_burst更大
                    end
                    else begin
                        ar_next_burst <= 1 << LGMAXBURST; //满拍的情况
                    end 
                end
        end
    end

    //---update when start_burst_comb（第三拍）--
    // reg [LGMAXBURST - 1 : 0] axi_arlen;   //ar_next_burst做减一操作后赋值给axi_arlen


    //输出信号
    // reg                   axi_arvalid; //axi开头为输出信号的reg类型
    // reg [1 : 0]           axi_arburst;
    // reg [2 : 0]           axi_arsize;
    // reg [ADDR_WD - 1 : 0] axi_araddr;
    // reg                   axi_abort_pending; //中止等待（发生错误的情况）
    // reg                   axi_rready;

    //axi_arvalid
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_arvalid <= 'b0;
        end
        else if (!M_AXI_ARVALID || M_AXI_ARREADY) begin //即start_burst_comb当拍时赋值，下一拍拉高valid使数据保持
            axi_arvalid <= start_burst_comb;            
        end
    end

    //axi_arlen
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_arlen <= 'b0;
        end
        else if (!M_AXI_ARVALID || M_AXI_ARREADY) begin //start_burst_comb下一拍arvalid拉高，axi_arlen被锁定，所以之前的非阻塞状态不用关心
            axi_arlen <= ar_next_burst - 1'b1;          //r_cmd_fire时数据已经锁定，时序满足
        end
    end

    //axi_arburst
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_arburst <= FIXED;
        end
        else if(!M_AXI_ARVALID || M_AXI_ARREADY) begin
            axi_arburst <= ar_burst;
        end
    end

    //axi_arsize
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_arsize <= 'b0;
        end
        else if(!M_AXI_ARVALID || M_AXI_ARREADY) begin
            axi_arsize <= ar_size;
        end
    end

    //axi_araddr 下一个burst的地址
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_araddr <= 'b0;
        end
        else begin
            if(r_pre_start) begin //在busy的第一拍时将ar_addr赋值给axi_araddr，因为ar_addr是在！busy时发生变化的
                axi_araddr <= ar_addr;
            end
            if(M_AXI_ARVALID && M_AXI_ARREADY) begin //握手时，有新的burst要发
                axi_araddr[ADDRLSB - 1 : 0] <= 'b0; //对ADDRLSB位做清零对齐操作
                if(ar_incr_burst) begin //FIXED模式不用地址变化
                    axi_araddr[ADDR_WD - 1 : ADDRLSB] <= axi_araddr[ADDR_WD - 1 : ADDRLSB] + (M_AXI_ARLEN + 1'b1); //M_AXI_ARLEN = axi_arlen = (ar_next_burst - 1)
                end
            end
            if(!OPT_UNALIGNED) begin //强制对齐OPT
                axi_araddr[ADDRLSB - 1 : 0] <= 'b0;
            end
        end
    end

    //axi_abort_pending
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_abort_pending <= 0;
        end
        else begin
            if(M_AXI_RVALID && M_AXI_RREADY && M_AXI_RRESP[1]) begin //返回数据出错时拉高，拉高后后面不再发剩余数据
                axi_abort_pending <= 1'b1;
            end
            if (!r_busy) begin //r_busy状态结束后，拉低错误信号，等待CMD重新发信号
                axi_abort_pending <= 1'b0;
            end
        end
    end

    //axi_rready
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_rready <= 1'b0;
        end
        else begin
            if(r_complete_comb) begin
                axi_rready <= 1'b0;
            end
            if(M_AXI_ARVALID && M_AXI_ARREADY) begin
                axi_rready <= 1'b1; //置1放在下面，提高效率
            end
        end 
    end

    //read_outstanding问题
    // reg [LGLENT - 1 : 0] ar_burst_outstanding; //下一拍还剩几个burst信号没有传输
    // reg                  ar_last_outstanding;  //下一拍为最后一个burst信号
    // reg                  ar_none_outstanding;  //下一拍没有burst信号了

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_burst_outstanding <= 'b0;
            ar_last_outstanding  <= 1'b0;
            ar_none_outstanding  <= 1'b1;
        end
        else begin
            case ({(M_AXI_ARVALID && M_AXI_ARREADY) , (M_AXI_RLAST && M_AXI_RVALID && M_AXI_RREADY)}) //{新发一个burst，一个burst刚结束}
                2'b01: begin
                    ar_burst_outstanding <= (ar_burst_outstanding - 1);
                    ar_last_outstanding  <= (ar_burst_outstanding == 2);
                    ar_none_outstanding  <= (ar_burst_outstanding == 1);
                end
                2'b10: begin
                    ar_burst_outstanding <= (ar_burst_outstanding + 1);
                    ar_last_outstanding  <= (ar_burst_outstanding == 0);
                    ar_none_outstanding  <= 1'b0;
                end
                2'b11: begin
                    ar_burst_outstanding <= ar_burst_outstanding;
                    ar_last_outstanding  <= (ar_burst_outstanding == 1);
                    ar_none_outstanding  <= 1'b0;
                end
            endcase
        end
    end
   
    //连线
    assign M_AXI_ARVALID = axi_arvalid;
    assign M_AXI_ARADDR  = axi_araddr;
    assign M_AXI_ARBURST = axi_arburst;
    assign M_AXI_ARSIZE  = axi_arsize;
    assign M_AXI_ARLEN   = axi_arlen;
    assign M_AXI_RREADY  = axi_rready;
    assign r_cmd_ready   = !r_busy && !axi_abort_pending; //不忙并且没有错误发生时，可以接受cmd新信号


endmodule