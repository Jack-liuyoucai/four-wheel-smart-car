///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.10.1.52143/W32 for ARM    07/Jan/2012  00:00:06 /
// Copyright 1999-2010 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Project_ /
//                    Settings\Startup_Code\start.c                           /
//    Command line =  F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Project_ /
//                    Settings\Startup_Code\start.c -D IAR -D TWR_K60N512     /
//                    -lCN F:\Kinetis\IARK60X256\10_MK60X256_test_pll\bin\Ram /
//                    \List\ -lB F:\Kinetis\IARK60X256\10_MK60X256_test_pll\b /
//                    in\Ram\List\ -o F:\Kinetis\IARK60X256\10_MK60X256_test_ /
//                    pll\bin\Ram\Obj\ --no_cse --no_unroll --no_inline       /
//                    --no_code_motion --no_tbaa --no_clustering              /
//                    --no_scheduling --debug --endian=little                 /
//                    --cpu=Cortex-M4 -e --fpu=None --dlib_config             /
//                    "C:\Program Files\IAR Systems\Embedded Workbench        /
//                    6.0\arm\INC\c\DLib_Config_Normal.h" -I                  /
//                    F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Sources\ /
//                    H\ -I F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\So /
//                    urces\H\Component_H\ -I F:\Kinetis\IARK60X256\10_MK60X2 /
//                    56_test_pll\src\Sources\H\Frame_H\ -Ol --use_c++_inline /
//    List file    =  F:\Kinetis\IARK60X256\10_MK60X256_test_pll\bin\Ram\List /
//                    \start.s                                                /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME start

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION `.data_init`:DATA:REORDER:NOROOT(0)
        SECTION CodeRelocate:DATA:REORDER:NOROOT(0)
        SECTION CodeRelocateRam:DATA:REORDER:NOROOT(0)

        EXTERN __VECTOR_RAM
        EXTERN __VECTOR_TABLE
        EXTERN main
        EXTERN wdog_disable
        EXTERN write_vtor

        PUBLIC common_startup
        PUBLIC start

// F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Project_Settings\Startup_Code\start.c
//    1 /********************************************************
//    2 【平    台】龙丘CORTEX-M4开发板/系统板
//    3 【编    写】龙丘
//    4 【Designed】by Chiu Sir
//    5 【E-mail  】chiusir@yahoo.cn
//    6 【软件版本】V1.0
//    7 【最后更新】2011年12月25日
//    8 【相关信息参考下列地址】
//    9 【网    站】http://www.lqist.cn
//   10 【淘宝店铺】http://shop36265907.taobao.com
//   11 【dev.env.】Code Warrior 10.1
//   12 【Target  】CORTEX-M4
//   13 【Crystal 】50.000Mhz
//   14 【busclock】？MHz
//   15 【pllclock】125MHz
//   16 ********************************************************/ 
//   17 //-------------------------------------------------------------------------*
//   18 // 文件名:start.c                                                          *
//   19 // 说  明: CPU启动后进行系统配置                                           *
//   20 //-------------------------------------------------------------------------*
//   21 
//   22 //头文件
//   23 #include "common.h"
//   24 #include "wdog.h"
//   25 #include "sysinit.h"
//   26 
//   27 #pragma section = ".data"
//   28 #pragma section = ".data_init"
//   29 #pragma section = ".bss"
//   30 #pragma section = "CodeRelocate"
//   31 #pragma section = "CodeRelocateRam" 
//   32 
//   33 //内部函数声明
//   34 //-------------------------------------------------------------------------*
//   35 //函数名: common_startup                                                   *
//   36 //功  能: 复制中断向量表到RAM中                                            * 
//   37 //参  数: 无								   *	
//   38 //说  明: 将ROM中的初始化数据拷贝到RAM中                                   *
//   39 //-------------------------------------------------------------------------*
//   40 void common_startup(void);
//   41 
//   42 //-------------------------------------------------------------------------*
//   43 //函数名: start                                                            *
//   44 //功  能: 系统启动                                                         * 
//   45 //参  数: 无								   *	
//   46 //说  明: 无                                                               *
//   47 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   48 void start(void)
//   49 {
start:
        PUSH     {R7,LR}
//   50     //关闭看门狗
//   51     wdog_disable();		
        BL       wdog_disable
//   52     //复制中断向量表到RAM中
//   53     common_startup();	
        BL       common_startup
//   54     //系统设置,龙丘注销此部分，系统初始化放在MAIN()中
//   55     //sysinit();			
//   56     //进入主函数
//   57     main();				
        BL       main
//   58 }
        POP      {R0,PC}          ;; return
//   59 
//   60 
//   61 //-------------------------------------------------------------------------*
//   62 //函数名: common_startup                                                   *
//   63 //功  能: 复制中断向量表到RAM中                                            * 
//   64 //参  数: 无								   *	
//   65 //说  明: 将ROM中的初始化数据拷贝到RAM中                                   *
//   66 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   67 void common_startup(void)
//   68 {
common_startup:
        PUSH     {R7,LR}
//   69     /* Declare a counter we'll use in all of the copy loops */
//   70     uint32 n;
//   71  
//   72  
//   73     /* Addresses for VECTOR_TABLE and VECTOR_RAM come from the linker file */  
//   74     extern uint32 __VECTOR_TABLE[];
//   75     extern uint32 __VECTOR_RAM[];
//   76 
//   77     /* Copy the vector table to RAM */
//   78     if (__VECTOR_RAM != __VECTOR_TABLE)
        LDR.N    R0,??DataTable0
        LDR.N    R1,??DataTable0_1
        CMP      R0,R1
        BEQ.N    ??common_startup_0
//   79     {
//   80         for (n = 0; n < 0x410; n++)
        MOVS     R0,#+0
        B.N      ??common_startup_1
//   81             __VECTOR_RAM[n] = __VECTOR_TABLE[n];
??common_startup_2:
        LDR.N    R1,??DataTable0
        LDR.N    R2,??DataTable0_1
        LDR      R2,[R2, R0, LSL #+2]
        STR      R2,[R1, R0, LSL #+2]
        ADDS     R0,R0,#+1
??common_startup_1:
        CMP      R0,#+1040
        BCC.N    ??common_startup_2
//   82     }
//   83     /* Point the VTOR to the new copy of the vector table */
//   84     write_vtor((uint32)__VECTOR_RAM);    
??common_startup_0:
        LDR.N    R0,??DataTable0
        BL       write_vtor
//   85     
//   86     /* Get the addresses for the .data section (initialized data section) */
//   87     uint8* data_ram = __section_begin(".data");
        LDR.N    R1,??DataTable0_2
//   88     uint8* data_rom = __section_begin(".data_init");
        LDR.N    R2,??DataTable0_3
//   89     uint8* data_rom_end = __section_end(".data_init");
        LDR.N    R0,??DataTable0_4
//   90     
//   91     /* Copy initialized data from ROM to RAM */
//   92     n = data_rom_end - data_rom;
        SUBS     R0,R0,R2
        B.N      ??common_startup_3
//   93     while (n--)
//   94       *data_ram++ = *data_rom++;
??common_startup_4:
        LDRB     R3,[R2, #+0]
        STRB     R3,[R1, #+0]
        ADDS     R2,R2,#+1
        ADDS     R1,R1,#+1
??common_startup_3:
        MOVS     R3,R0
        SUBS     R0,R3,#+1
        CMP      R3,#+0
        BNE.N    ??common_startup_4
//   95  
//   96  
//   97     /* Get the addresses for the .bss section (zero-initialized data) */
//   98     uint8* bss_start = __section_begin(".bss");
        LDR.N    R1,??DataTable0_5
//   99     uint8* bss_end = __section_end(".bss");
        LDR.N    R0,??DataTable0_6
//  100     
//  101     /* Clear the zero-initialized data section */
//  102     n = bss_end - bss_start;
        SUBS     R0,R0,R1
        B.N      ??common_startup_5
//  103     while(n--)
//  104       *bss_start++ = 0;    
??common_startup_6:
        MOVS     R2,#+0
        STRB     R2,[R1, #+0]
        ADDS     R1,R1,#+1
??common_startup_5:
        MOVS     R2,R0
        SUBS     R0,R2,#+1
        CMP      R2,#+0
        BNE.N    ??common_startup_6
//  105     
//  106     /* Get addresses for any code sections that need to be copied from ROM to RAM.
//  107      * The IAR tools have a predefined keyword that can be used to mark individual
//  108      * functions for execution from RAM. Add "__ramfunc" before the return type in
//  109      * the function prototype for any routines you need to execute from RAM instead 
//  110      * of ROM. ex: __ramfunc void foo(void);
//  111      */
//  112     uint8* code_relocate_ram = __section_begin("CodeRelocateRam");
        LDR.N    R1,??DataTable0_7
//  113     uint8* code_relocate = __section_begin("CodeRelocate");
        LDR.N    R2,??DataTable0_8
//  114     uint8* code_relocate_end = __section_end("CodeRelocate");
        LDR.N    R0,??DataTable0_9
//  115     
//  116     /* Copy functions from ROM to RAM */
//  117     n = code_relocate_end - code_relocate;
        SUBS     R0,R0,R2
        B.N      ??common_startup_7
//  118     while (n--)
//  119       *code_relocate_ram++ = *code_relocate++;
??common_startup_8:
        LDRB     R3,[R2, #+0]
        STRB     R3,[R1, #+0]
        ADDS     R2,R2,#+1
        ADDS     R1,R1,#+1
??common_startup_7:
        MOVS     R3,R0
        SUBS     R0,R3,#+1
        CMP      R3,#+0
        BNE.N    ??common_startup_8
//  120 }
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0:
        DC32     __VECTOR_RAM

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_1:
        DC32     __VECTOR_TABLE

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_2:
        DC32     SFB(`.data`)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_3:
        DC32     SFB(`.data_init`)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_4:
        DC32     SFE(`.data_init`)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_5:
        DC32     SFB(`.bss`)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_6:
        DC32     SFE(`.bss`)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_7:
        DC32     SFB(CodeRelocateRam)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_8:
        DC32     SFB(CodeRelocate)

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_9:
        DC32     SFE(CodeRelocate)

        SECTION `.bss`:DATA:REORDER:NOROOT(0)

        SECTION `.data`:DATA:REORDER:NOROOT(0)

        SECTION `.data_init`:DATA:REORDER:NOROOT(0)

        SECTION CodeRelocate:DATA:REORDER:NOROOT(0)

        SECTION CodeRelocateRam:DATA:REORDER:NOROOT(0)

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)

        END
// 
// 172 bytes in section .text
// 
// 172 bytes of CODE memory
//
//Errors: none
//Warnings: none
