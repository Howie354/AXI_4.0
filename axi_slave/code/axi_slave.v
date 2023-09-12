`default_nettype none

module axi_slave #(
    parameter C_S_ADDR_WIDTH = 8,
    parameter C_S_DATA_WIDTH = 32,
    parameter C_S_ID_WIDTH   = 2,
    parameter OPT_LOWPOWER   = 0
) (
    input  wire                                  S_AXI_ACLK,
    input  wire                                  S_AXI_ARSTN,

    input  wire                                  S_AXI_AWVALID,
    output wire                                  S_AXI_AWREADY,
    input  wire [C_S_ID_WIDTH - 1 : 0]           S_AXI_AWID,
    input  wire [1 : 0]                          S_AXI_AWBURST,
    input  wire [2 : 0]                          S_AXI_AWSIZE,
    input  wire [7 : 0]                          S_AXI_AWLEN,
    input  wire [C_S_ADDR_WIDTH - 1 : 0]         S_AXI_AWADDR, //write address channel

    input  wire                                  S_AXI_WVALID,
    output wire                                  S_AXI_WREADY,
    input  wire [C_S_DATA_WIDTH - 1 : 0]         S_AXI_WDATA,
    input  wire [((C_S_DATA_WIDTH / 8) - 1) : 0] S_AXI_WSTRB,
    input  wire                                  S_AXI_WLAST, //write data channel

    output wire                                  S_AXI_BVALID,
    input  wire                                  S_AXI_BREADY,
    output wire [1 : 0]                          S_AXI_BRESP,
    output wire [C_S_ID_WIDTH - 1 : 0]           S_AXI_BID, //write response channel
    
    input  wire                                  S_AXI_ARVALID,
    output wire                                  S_AXI_ARREADY,
    input  wire [C_S_ID_WIDTH - 1 : 0]           S_AXI_ARID,
    input  wire [C_S_ADDR_WIDTH - 1 : 0]         S_AXI_ARADDR,
    input  wire [7 : 0]                          S_AXI_ARLEN,
    input  wire [1 : 0]                          S_AXI_ARBURST,
    input  wire [2 : 0]                          S_AXI_ARSIZE, //read address channel

    output wire                                  S_AXI_RVALID,
    input  wire                                  S_AXI_RREADY,
    output wire [C_S_DATA_WIDTH - 1 : 0]         S_AXI_RDATA,
    output wire                                  S_AXI_RRESP,
    output wire [C_S_ID_WIDTH - 1 : 0]           S_AXI_RID,
    output wire                                  S_AXI_RLAST //read response channel
);
    
    localparam AW      = C_S_ADDR_WIDTH;
    localparam DW      = C_S_DATA_WIDTH;
    localparam IW      = C_S_ID_WIDTH;
    localparam DW_BYTE = C_S_DATA_WIDTH / 8;
    localparam DEPTH   = 1 << AW;

    reg  [7 : 0] mem [DEPTH - 1 : 0];

    reg  [IW - 1 : 0] r_bid;       //为了实现高效率传输，所以在第一次B通道阻塞后，允许地址通道继续发数据
    reg               r_bvalid;    //r_bid与r_bvalid存储阻塞时的信号，待BREADY后释放
    reg  [IW - 1 : 0] axi_bid;
    reg               axi_bvalid;  //write resp channel中间数据与输出数据
                                   //输出的寄存器数据用 axi_XXX表示
    reg               axi_awready;
    reg               axi_wready;
    reg  [AW - 1 : 0] r_waddr;
 // reg  [DW - 1 : 0] r_wdata;
    reg  [1 : 0]      r_wburst;
    reg  [2 : 0]      r_wsize;
    reg  [7 : 0]      r_wlen;
    wire [AW - 1 : 0] next_wr_addr; //写的中间变量

    reg [IW - 1 : 0]  axi_rid;
    reg               axi_rvalid;
    reg               axi_arready;
    reg               axi_rlast;
    reg [DW - 1 : 0]  axi_rdata;
    reg [7 : 0]       r_rlen;
    reg [1 : 0]       r_rburst;
    reg [2 : 0]       r_rsize;
    reg [AW - 1 : 0]  r_raddr;
    wire [AW - 1 : 0] next_rd_addr;
    reg [7 : 0]       cur_rlen; //记录现在读到第几拍，写的时候不用记录，因为这是主机需要做的
                                //读的中间变量

    //outstanding fifo
    reg [(AW << 1) - 1 : 0] fifo_mem [15 : 0];
    reg [4 : 0] wr_ptr;
    reg [4 : 0] rd_ptr;
    
    wire fifo_full  = {~rd_ptr[4],rd_ptr[3 : 0]} == wr_ptr;
    wire fifo_empty = rd_ptr == wr_ptr;

/*-------------------------------------write logic-----------------------------------*/

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
        end
        else if(S_AXI_AWVALID && S_AXI_AWREADY) begin
            axi_wready  <= 1'b1;
            axi_awready <= 1'b0;
        end
        else if(S_AXI_WVALID && S_AXI_WREADY) begin
            axi_wready  <= (!S_AXI_WLAST);
            axi_awready <= S_AXI_WLAST && (!S_AXI_BVALID || S_AXI_BREADY); //没有阻塞，连续传输的情况
        end
        else if (!axi_awready) begin //一组数据传输过程中，或者被阻塞的情况 ****
            if(S_AXI_WREADY) begin //数据传输过程中，awready始终置0
                axi_awready <= 1'b0;
            end
            else if(r_bvalid && !S_AXI_BREADY) begin //第二次写数据接受完成了，但是上一拍还有写反馈没有被接收，不能接收新的写地址
                axi_awready <= 1'b0;                 
            end
            else begin 
                axi_awready <= 1'b1; //第一次数据传输结束，B通道阻塞时，r_bvalid还没来的及拉高，此时将awready拉高迎接下一次数据
            end
        end
    end

    //代表上一个数据的反馈信号有没有被接收
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            r_bvalid <= 1'b0;
        end
        else if (S_AXI_WLAST && S_AXI_WVALID && S_AXI_WREADY) begin 
            r_bvalid <= S_AXI_BVALID && !S_AXI_BREADY; //第一组数据被阻塞时，将阻塞信号存储起来
        end
        else if(S_AXI_BREADY) begin //BREADY来临时，释放
            r_bvalid <= 1'b0;
        end
    end

    //代表当次数据的反馈信号
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            axi_bvalid <= 1'b0;
        end
        else if(S_AXI_WLAST && S_AXI_WVALID && S_AXI_WREADY) begin
            axi_bvalid <= 1'b1;
        end
        //当次数据反馈请求有效
        else if(S_AXI_BREADY) begin //BREADY来临时，将上一组被阻塞的信号赋值过来
            axi_bvalid <= r_bvalid; //注意，如果是在r_bvalid拉高时，本次数据传输也结束了，axi_bvalid也在拉高，在ready到来时，先握手一次上拍的
                                    //bvalid又会被r_bvalid赋值而拉高，相当于又要握手一次本拍的
        end
    end

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN && OPT_LOWPOWER) begin
            r_bid   <= 'b0;
            axi_bid <= 'b0;
        end
        if(S_AXI_AWVALID && S_AXI_AWREADY) begin //OPT_LOWPOWER什么意思？
            r_bid <= S_AXI_AWID;                                    //地址通道握手时，就把值存到r_bid
        end                                             
        if (!S_AXI_BVALID || S_AXI_BREADY) begin //不阻塞时，将r_bid存储的值给到输出
            axi_bid <= r_bid;                    //数据传输过程中反馈通道不会阻塞
        end
    end

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            r_waddr  <= 'b0;
            r_wburst <= 'b0;
            r_wlen   <= 'b0;
            r_wsize  <= 'b0;
        end
        else if(S_AXI_AWVALID && S_AXI_AWREADY) begin
            r_waddr  <= S_AXI_AWADDR;
            r_wburst <= S_AXI_AWBURST;
            r_wlen   <= S_AXI_AWLEN;
            r_wsize  <= S_AXI_AWSIZE;
        end
        else if(S_AXI_WVALID && S_AXI_WREADY) begin
            r_waddr <= next_wr_addr;
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if(S_AXI_WVALID && S_AXI_WREADY) begin
            for(integer idx = 0; idx < DW_BYTE; idx = idx + 1'b1) begin
                mem[r_waddr + idx] <= S_AXI_WDATA[idx * 8 +: 8];
            end
        end
    end

    axi_address #(
        .DW(DW),
        .AW(AW)
        )u_wr_axi_address(
        .i_last_address(r_waddr),
        .i_burst(r_wburst),
        .i_len(r_wlen),
        .i_size(r_wsize),
        .o_next_address(next_wr_addr)
        ); //得到下一拍的写地址

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_BRESP   = 2'b00;
    assign S_AXI_BID     = axi_bid;

/*-------------------------------------read logic-----------------------------------*/
    // always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
    //     if(!S_AXI_ARSTN) begin
    //         axi_arready <= 1'b0;
    //     end
    //     else if(S_AXI_ARVALID && S_AXI_ARREADY) begin
    //         axi_arready <= (S_AXI_ARLEN == 0);
    //     end
    //     else if(!S_AXI_RVALID || S_AXI_RREADY) begin
    //         axi_arready <= (cur_rlen <= 1); //小于等于1的意思是空闲时或者当读数据读到最后一拍时，就可以在下一拍继续接收读请求
    //     end
    // end
    
    //wr_ptr
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if (!S_AXI_ARSTN) begin
            wr_ptr <= 'b0;
        end
        else if (S_AXI_ARVALID && S_AXI_ARREADY) begin
            wr_ptr <= wr_ptr + 1'b1;
            fifo_mem[wr_ptr[3:0]] <= {S_AXI_ARADDR, S_AXI_ARLEN};
        end
    end

    //rd_ptr
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if (!S_AXI_ARSTN) begin
            rd_ptr <= 'b0;
        end
        else if (cur_rlen == 1'b1) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // araddr, arlen, cur_rlen
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if (!S_AXI_ARSTN) begin
            cur_rlen <= 'b0;
        end
        else if ((cur_rlen == 'b0) && !fifo_empty) begin
            r_raddr   <= fifo_mem[rd_ptr[3:0]][(AW << 1)-1 : AW];
            r_rlen    <= fifo_mem[rd_ptr[3:0]][AW-1 : 0];
            cur_rlen <= fifo_mem[rd_ptr[3:0]][AW-1 : 0];
        end
        else if (S_AXI_RVALID && S_AXI_RREADY) begin
            cur_rlen <= (cur_rlen != 0) ? (cur_rlen - 1'b1) : 'b0;
            r_raddr   <= next_rd_addr;
        end
    end

    //axi_arready
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            axi_arready <= 1'b1;
        end
        else if(!fifo_full) begin
            axi_arready <= 1'b1;
        end
        else begin
            axi_arready <= 1'b0;
        end
    end

    //cur_rlen
    // always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
    //     if(!S_AXI_ARSTN) begin
    //         cur_rlen <= 'b0;
    //     end
    //     else if(S_AXI_ARVALID && S_AXI_ARREADY) begin
    //         cur_rlen <= S_AXI_ARLEN;
    //     end
    //     else if(S_AXI_RVALID && S_AXI_RREADY) begin
    //         cur_rlen <= cur_rlen - 1'b1;
    //     end
    // end
    
    // always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
    //     if(!S_AXI_ARSTN) begin
    //         axi_rvalid <= 1'b0;
    //     end
    //     else if (S_AXI_ARVALID && S_AXI_ARREADY) begin
    //         axi_rvalid <= 1'b1;
    //     end
    //     else if(S_AXI_RVALID && S_AXI_RREADY) begin
    //         axi_rvalid <= (cur_rlen > 1'b0);
    //     end
    // end

    // axi_rvalid
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if (!S_AXI_ARSTN) begin
            axi_rvalid <= 1'b0;
        end
        else if (S_AXI_ARVALID && S_AXI_ARREADY && !fifo_empty) begin
            axi_rvalid <= 1'b1;
        end
        else if (S_AXI_RVALID && S_AXI_RREADY) begin
            axi_rvalid <= (cur_rlen > 0);
        end
    end
    
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            axi_rlast <= 1'b0;
        end
        else if(S_AXI_ARVALID && S_AXI_ARREADY) begin //当只有一拍数据时，下一拍直接r_last拉高，
            axi_rlast <= (S_AXI_ARLEN == 1'b0);       //避免了cur_rlen接收到值的下一拍再拉高的过程，省了一拍的时间
        end
        else if (S_AXI_RVALID && S_AXI_RREADY) begin
            axi_rlast <= (cur_rlen == 1'b1);
        end
    end

    // always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
    //     if(!S_AXI_ARSTN) begin
    //         r_raddr <= 'b0;
    //     end
    //     else if(OPT_LOWPOWER && !S_AXI_ARVALID) begin
    //         r_raddr <= 'b0;
    //     end
    //     else if(S_AXI_ARVALID && S_AXI_ARREADY) begin
    //         r_raddr <= S_AXI_ARADDR;
    //     end
    //     else if(S_AXI_RVALID && S_AXI_RREADY) begin
    //         r_raddr <= next_rd_addr;
    //     end
    // end

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARSTN) begin
        if(!S_AXI_ARSTN) begin
            axi_rid  <= 'b0;
            r_rburst <= 'b0;
            r_rsize  <= 'b0;
        end
        if(S_AXI_ARREADY) begin
            axi_rid  <= S_AXI_ARID;
            r_rburst <= S_AXI_ARBURST;
            r_rsize  <= S_AXI_ARSIZE; 
            if(OPT_LOWPOWER && !S_AXI_ARVALID) begin
                axi_rid  <= 'b0;
                r_rburst <= 'b0;
                r_rsize  <= 'b0;
            end
        end
    end

    axi_address #(
        .DW(DW),
        .AW(AW)
        )u_rd_axi_address(
        .i_last_address( r_raddr),//S_AXI_RREADY ? S_AXI_ARADDR :
        .i_burst( r_rburst),//S_AXI_RREADY ? S_AXI_ARBURST :
        .i_len( r_rlen),//S_AXI_RREADY ? S_AXI_ARLEN :
        .i_size( r_rsize),//S_AXI_RREADY ? S_AXI_ARSIZE :
        .o_next_address(next_rd_addr)
        ); //得到下一拍的读地址，通过判断来提升效率
           //对于读来说，不用判断来提高效率，因为读的时候必须先收到地址的下一拍才传数据，无法优化

    always @(posedge S_AXI_ACLK) begin
        if(S_AXI_RVALID && S_AXI_RREADY) begin
            for(integer idx = 0; idx < DW_BYTE; idx = idx + 1'b1) begin
                axi_rdata[idx * 8 +: 8] <= mem[r_raddr + idx];
            end
        end
    end

    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RVALID  = axi_rvalid;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = 2'b00;
    assign S_AXI_RID     = axi_rid;
    assign S_AXI_RLAST   = axi_rlast;

endmodule

/*-------------------------------------address caculate logic-----------------------------------*/

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
        if(DSZ < 2) begin //8bits 16bits
            wrap_mask = 1; 
            wrap_mask = wrap_mask | ({{(AW - 4){1'b0}},i_len[3 : 0]} << (i_size[0]));
        end  //注意AW-4与1'b0的拼合也要用拼接符相连
        else if(DSZ < 4) begin //32bits 64bits
            wrap_mask = 3;
            wrap_mask = wrap_mask | ({{(AW - 4){1'b0}},i_len[3 : 0]} << (i_size[1 : 0]));
        end
        else begin
            wrap_mask = {i_size{1'b1}};
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
                o_next_address =  (wrap_mask & o_next_address)   //保留低位
                                | (~wrap_mask & i_last_address); //保留高位
            end

            if (AW > 12) begin
                o_next_address[AW - 1 : ((AW > 12) ? 12 : (AW - 1))] = i_last_address[AW - 1 : ((AW > 12) ? 12 : (AW - 1))];
            end
        end
    end
    
endmodule

