module tb_axi_master_r #(
    parameter ADDR_WD = 32,
    parameter DATA_WD = 32,
    parameter STRB_WD = ADDR_WD / 8 
) (   
);

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
end

wire                    CMD_VALID;
wire  [1 : 0]           CMD_BURST; //Burst类型选择
wire  [2 : 0]           CMD_SIZE;  //每一拍发多少byte（一般为总线位宽）
wire  [ADDR_WD - 1 : 0] CMD_ADDR;  //起始地址
wire  [ADDR_WD - 1 : 0] CMD_LEN;   //表示要传输的字节数（不是拍数），real_len = r_cmd_len + 1
wire                    CMD_READY;

wire                   M_AXI_ARVALID;
wire [1 : 0]           M_AXI_ARBURST;
wire [2 : 0]           M_AXI_ARSIZE;
wire [ADDR_WD - 1 : 0] M_AXI_ARADDR;
wire [7 : 0]           M_AXI_ARLEN; //考虑INCR和FIXED情况，INCR（0~255） FIXED（0~15） WRAP（ 1 3 7 15）
wire                   M_AXI_ARREADY;

wire                   M_AXI_RVALID;
wire                   M_AXI_RLAST; //读通道的一个burst读取完毕结束信号
wire [DATA_WD - 1 : 0] M_AXI_RDATA; //读通道的返回数据
wire [1 : 0]           M_AXI_RRESP; //反馈信号，00（OK） ； 01（exclusive OK） ；10、11 （error）
wire                   M_AXI_RREADY;

reg                    r_cmd_valid;
// reg  [1 : 0]           r_cmd_burst; //Burst类型选择
// reg  [2 : 0]           r_cmd_size;  //每一拍发多少byte（一般为总线位宽）
reg  [ADDR_WD - 1 : 0] r_cmd_addr;  //起始地址
reg  [ADDR_WD - 1 : 0] r_cmd_len;   //表示要传输的字节数（不是拍数），real_len = r_cmd_len + 1
reg                    r_cmd_rready;
reg                    r_cmd_arready;

assign CMD_VALID     = r_cmd_valid;
assign CMD_BURST     = INCR;
assign CMD_SIZE      = 1 << (DATA_BYTE_WD) - 1;
assign CMD_ADDR      = r_cmd_addr;
assign CMD_LEN       = r_cmd_len;
assign M_AXI_ARREADY = r_cmd_arready;

localparam INCR         = 2'b10;
localparam DEPTH        = 1 << ADDR_WD;
localparam DATA_BYTE_WD = $clog2(DATA_WD / 8);

// reg [7 : 0] mem [DEPTH - 1 : 0];

// initial begin
//     for (integer i=0; i<DEPTH; i=i+1) begin
//         mem[i] = i;
//     end
// end

axi_master_r #(
    .ADDR_WD(ADDR_WD),
    .DATA_WD(DATA_WD),
    .STRB_WD(STRB_WD)
    ) u_axi_master_r(
    .clk(clk),
    .rst_n(rst_n),
    .r_cmd_valid(CMD_VALID),
    .r_cmd_burst(CMD_BURST),
    .r_cmd_size(CMD_SIZE),
    .r_cmd_addr(CMD_ADDR),
    .r_cmd_len(CMD_LEN),
    .r_cmd_ready(CMD_READY),
    .M_AXI_ARVALID(M_AXI_ARVALID),
    .M_AXI_ARBURST(M_AXI_ARBURST),
    .M_AXI_ARSIZE(M_AXI_ARSIZE),
    .M_AXI_ARADDR(M_AXI_ARADDR),
    .M_AXI_ARLEN(M_AXI_ARLEN),
    .M_AXI_ARREADY(M_AXI_ARREADY),
    .M_AXI_RVALID(M_AXI_RVALID),
    .M_AXI_RLAST(M_AXI_RLAST),
    .M_AXI_RDATA(M_AXI_RDATA),
    .M_AXI_RRESP(M_AXI_RRESP),
    .M_AXI_RREADY(M_AXI_RREADY)
    );

localparam RANDOM = 1'b0;

generate
    if(!RANDOM) begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                r_cmd_valid   <= 1'b0;
                r_cmd_arready <= 1'b1;
                r_cmd_addr    <= 'b0;
                r_cmd_len     <= 'b0;
            end
            else begin
                r_cmd_valid   <= 1'b1;
                r_cmd_arready <= 1'b1;
                r_cmd_addr    <= 'b0;
                r_cmd_len     <= 2048 + 255;
            end
        end       
    end
    else begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                r_cmd_valid   <= 1'b0;
                r_cmd_arready <= $random;
                r_cmd_addr    <= 'b0;
                r_cmd_len     <= 'b0;
            end
            else begin
                r_cmd_valid   <= $random;
                r_cmd_arready <= $random;
                r_cmd_addr    <= {$random}%512;
                r_cmd_len     <= {$random}%4096;
            end
        end
    end
endgenerate

endmodule