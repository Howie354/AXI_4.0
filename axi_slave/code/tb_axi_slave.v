`default_nettype 

module tb_axi_slave#(
    parameter DW      = 32,
    parameter AW      = 8,
    parameter DW_BYTE = DW / 8,
    parameter ID_WD   = 2
) ();

localparam STRB_WD = DW / 8;
localparam CNTR_WD = AW - $clog2(DW_BYTE);  //记录最多会有多少拍的计数器的宽度，一拍4byte,一个AW数据存储1byte
localparam DEPTH   = 1 << AW;
localparam INCR    = 2'b01;

reg clk;
reg rst_n;

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    rst_n = 0;
    #100 
    rst_n = 1;
    #50000
    $finish;
end

initial begin
    $dumpfile("test.vcd");
    $dumpvars;
    for (integer idx = 0;idx < DEPTH; idx = idx + 1'b1) begin
       $dumpvars(0,u_axi_slave.mem[idx]); 
    end //将mem中的数据也打出来
end

reg                    axi_awvalid;
reg                    axi_wvalid;
reg                    axi_arvalid;
reg                    axi_wlast;
reg                    axi_bready;
reg                    axi_rready;
reg [ID_WD - 1 : 0]    axi_awid;
reg [ID_WD - 1 : 0]    axi_arid;
reg [CNTR_WD - 1 : 0]  cntr; //计数器

wire                   M_AXI_AWVALID;
wire [ID_WD - 1 : 0]   M_AXI_AWID;
wire [AW - 1 : 0]      M_AXI_AWADDR;
wire [7 : 0]           M_AXI_AWLEN;
wire [2 : 0]           M_AXI_AWSIZE;
wire [1 : 0]           M_AXI_AWBURST;
wire                   M_AXI_AWREADY;
wire                   M_AXI_WVALID;
wire [DW - 1: 0]       M_AXI_WDATA;
wire [STRB_WD - 1 : 0] M_AXI_WSTRB;
wire                   M_AXI_WLAST;
wire                   M_AXI_WREADY;
wire                   M_AXI_BVALID;
wire [ID_WD - 1 : 0]   M_AXI_BID;
wire [1 : 0]           M_AXI_BRESP;
wire                   M_AXI_BREADY;

wire                   M_AXI_ARVALID;
wire [ID_WD - 1 : 0]   M_AXI_ARID;
wire [AW - 1 : 0]      M_AXI_ARADDR;
wire [7 : 0]           M_AXI_ARLEN;
wire [2 : 0]           M_AXI_ARSIZE;
wire [1 : 0]           M_AXI_ARBURST;
wire                   M_AXI_ARREADY;
wire                   M_AXI_RVALID;
wire [ID_WD - 1 : 0]   M_AXI_RID;
wire [DW - 1 : 0]      M_AXI_RDATA;
wire [1 : 0]           M_AXI_RRESP;
wire                   M_AXI_RLAST;
wire                   M_AXI_RREADY;

assign M_AXI_AWVALID = axi_awvalid;
assign M_AXI_AWID    = axi_awid;
assign M_AXI_AWBURST = INCR;
assign M_AXI_AWSIZE  = $clog2(DW_BYTE);
assign M_AXI_AWLEN   = (1 << CNTR_WD) - 1; //代表一次burst发多少拍，这样赋值相当于一次burst写满一次AW
assign M_AXI_AWADDR  = 'b0;
assign M_AXI_WVALID  = axi_wvalid;
assign M_AXI_WDATA   = {{(DW - CNTR_WD){1'b0}},cntr[CNTR_WD - 1 : 0]}; //让DATA和计数器当前值相同，方便debug
assign M_AXI_WSTRB   = (1 << DW_BYTE) - 1;
assign M_AXI_WLAST   = axi_wlast;
assign M_AXI_BREADY  = axi_bready;

assign M_AXI_ARVALID = axi_arvalid;
assign M_AXI_ARID    = axi_arid;
assign M_AXI_ARBURST = INCR;
assign M_AXI_ARSIZE  = $clog2(DW_BYTE);
assign M_AXI_ARLEN   = (1 << CNTR_WD) - 1;
assign M_AXI_ARADDR  = 'b0;
assign M_AXI_RREADY  = axi_rready;

axi_slave #(
    .C_S_DATA_WIDTH(DW),
    .C_S_ADDR_WIDTH(AW),
    .C_S_ID_WIDTH(ID_WD),
    .OPT_LOWPOWER(OPT_LOWPOWER)
    ) u_axi_slave(
    .S_AXI_ACLK(clk),
    .S_AXI_ARSTN(rst_n),
    .S_AXI_AWVALID(M_AXI_AWVALID),
    .S_AXI_AWREADY(M_AXI_AWREADY),
    .S_AXI_AWID(M_AXI_AWID),
    .S_AXI_AWBURST(M_AXI_AWBURST),
    .S_AXI_AWSIZE(M_AXI_AWSIZE),
    .S_AXI_AWLEN(M_AXI_AWLEN),
    .S_AXI_AWADDR(M_AXI_AWADDR),
    .S_AXI_WVALID(M_AXI_WVALID),
    .S_AXI_WREADY(M_AXI_WREADY),
    .S_AXI_WDATA(M_AXI_WDATA),
    .S_AXI_WSTRB(M_AXI_WSTRB),
    .S_AXI_WLAST(M_AXI_WLAST),
    .S_AXI_BVALID(M_AXI_BVALID),
    .S_AXI_BREADY(M_AXI_BREADY),
    .S_AXI_BRESP(M_AXI_BRESP),
    .S_AXI_BID(M_AXI_BID),
    .S_AXI_ARVALID(M_AXI_ARVALID),
    .S_AXI_ARREADY(M_AXI_ARREADY),
    .S_AXI_ARID(M_AXI_ARID),
    .S_AXI_ARADDR(M_AXI_ARADDR),
    .S_AXI_ARLEN(M_AXI_ARLEN),
    .S_AXI_ARBURST(M_AXI_ARBURST),
    .S_AXI_ARSIZE(M_AXI_ARSIZE),
    .S_AXI_RVALID(M_AXI_RVALID),
    .S_AXI_RREADY(M_AXI_RREADY),
    .S_AXI_RDATA(M_AXI_RDATA),
    .S_AXI_RRESP(M_AXI_RRESP),
    .S_AXI_RID(M_AXI_RID),
    .S_AXI_RLAST(M_AXI_RLAST)
    );

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        axi_wlast <= 1'b0;
    end
    else if(M_AXI_AWVALID && M_AXI_AWREADY) begin
        axi_wlast <= M_AXI_AWLEN == 0;
    end
    else if(M_AXI_WVALID && M_AXI_WREADY) begin
        axi_wlast <= cntr == (1 << CNTR_WD) - 2; 
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        cntr     <= 'b0;
        axi_awid <= 'b0;
        axi_arid <= 'b0;
    end
    else begin
        // if(M_AXI_AWVALID && M_AXI_AWREADY) begin
        //     cntr <= cntr + 1'b1;
        // end            选择器比较占面积，最好用组合逻辑代替
        cntr     <= cntr + (M_AXI_WVALID && M_AXI_WREADY); //每一拍发4byte数据，存到4个地址里，WVALID和WREADY握手一次
        axi_awid <= axi_awid + (M_AXI_AWVALID && M_AXI_AWREADY);
        axi_arid <= axi_arid + (M_AXI_ARVALID && M_AXI_ARREADY);
    end
end

localparam RANDOM       = 1'b1;
localparam OPT_LOWPOWER = 1'b0;

generate if(RANDOM) begin
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awvalid <= 1'b0;
            axi_wvalid  <= 1'b0;
            axi_arvalid <= 1'b0;

            axi_bready  <= $random;
            axi_rready  <= $random;
        end
        else begin
            axi_awvalid <= $random;
            axi_wvalid  <= $random;
            axi_arvalid <= $random;

            axi_bready  <= $random;
            axi_rready  <= $random;
        end
    end
end
else begin
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            axi_awvalid <= 1'b0;
            axi_wvalid  <= 1'b0;
            axi_arvalid <= 1'b0;

            axi_bready  <= 1'b1;
            axi_rready  <= 1'b1;
        end
        else begin
            axi_awvalid <= 1'b1;
            axi_wvalid  <= 1'b1;
            axi_arvalid <= 1'b1;

            axi_bready  <= 1'b1;
            axi_rready  <= 1'b1;
        end
    end
end   
endgenerate

endmodule