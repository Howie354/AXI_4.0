##set iverilog_path=C:\iverilog\bin;
##set gtkwave_path=C:\iverilog\gtkwave\bin;
##set path=%iverilog_path%%gtkwave_path%%path%

##在制定源文件的时候可以用通配符*,如本人用的批处理中通常使用这种方式指定RTL文件:set rtl_file="../rtl/*.v"
##set dut_1=../code/axi_lite_master
##set dut_2 = ../code/axi_lite_slave
##set testbentch_module=../code/axi_lite_top_tb
##set dut_3 = ../code/apb2apb_bridge



##set rtl_file = "../code/*.v"
iverilog -o "test.vvp" -c list.txt 

##iverilog -o "test.vvp" %testbentch_module%.v  %dut_1%.v  %dut_2%.v 
vvp -n "test.vvp"

set gtkw_file="test.gtkw"
if exist %gtkw_file% (gtkwave %gtkw_file%) else (gtkwave "test.vcd")
pause

