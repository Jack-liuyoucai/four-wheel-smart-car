@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM 


"E:\anzhuangbao\K60\common\bin\cspybat" "E:\anzhuangbao\K60\arm\bin\armproc.dll" "E:\anzhuangbao\K60\arm\bin\armpemicro.dll"  %1 --plugin "E:\anzhuangbao\K60\arm\bin\armbat.dll" --macro "E:\anzhuangbao\K60\arm\config\debugger\Freescale\Trace_Kxx.dmac" --flash_loader "E:\anzhuangbao\K60\arm\config\flashloader\Freescale\FlashK60Xxxx.board" --backend -B "--endian=little" "--cpu=Cortex-M4" "--fpu=None" "-p" "E:\anzhuangbao\K60\arm\CONFIG\debugger\Freescale\iok60xxxxv2.ddf" "--semihosting" "--device=MK60DN512xxx10" "--pemicro_interface_type=OSJtag" "--pemicro_reset_delay=" "--drv_communication=USB1" 


