# AXI_4.0
基于AXI 4.0协议的主从机设计

包含AXI主机的读模块，写模块以及从机的实现

实现了Stream_insert和Stream_remove的处理

支持地址非对齐传输、支持FIXED、INCR和WRAP burst传输、支持outstanding传输、支持跨4K边界的burst传输

完成rtl代码coding，编写Testbench，仿真验证成功
