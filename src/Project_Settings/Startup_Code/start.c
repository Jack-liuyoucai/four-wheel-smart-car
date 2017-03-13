/********************************************************
【平    台】龙丘CORTEX-M4开发板/系统板
【编    写】龙丘
【Designed】by Chiu Sir
【E-mail  】chiusir@yahoo.cn
【软件版本】V1.0
【最后更新】2011年12月25日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【dev.env.】Code Warrior 10.1
【Target  】CORTEX-M4
【Crystal 】50.000Mhz
【busclock】？MHz
【pllclock】125MHz
********************************************************/ 
//-------------------------------------------------------------------------*
// 文件名:start.c                                                          *
// 说  明: CPU启动后进行系统配置                                           *
//-------------------------------------------------------------------------*

//头文件
#include "common.h"
#include "wdog.h"
#include "sysinit.h"

#pragma section = ".data"
#pragma section = ".data_init"
#pragma section = ".bss"
#pragma section = "CodeRelocate"
#pragma section = "CodeRelocateRam" 

//内部函数声明
//-------------------------------------------------------------------------*
//函数名: common_startup                                                   *
//功  能: 复制中断向量表到RAM中                                            * 
//参  数: 无								   *	
//说  明: 将ROM中的初始化数据拷贝到RAM中                                   *
//-------------------------------------------------------------------------*
void common_startup(void);

//-------------------------------------------------------------------------*
//函数名: start                                                            *
//功  能: 系统启动                                                         * 
//参  数: 无								   *	
//说  明: 无                                                               *
//-------------------------------------------------------------------------*
void start(void)
{
    //关闭看门狗
    wdog_disable();		
    //复制中断向量表到RAM中
    common_startup();	
    //系统设置,龙丘注销此部分，系统初始化放在MAIN()中
    //sysinit();			
    //进入主函数
    main();				
}


//-------------------------------------------------------------------------*
//函数名: common_startup                                                   *
//功  能: 复制中断向量表到RAM中                                            * 
//参  数: 无								   *	
//说  明: 将ROM中的初始化数据拷贝到RAM中                                   *
//-------------------------------------------------------------------------*
void common_startup(void)
{
    /* Declare a counter we'll use in all of the copy loops */
    uint32 n;
 
 
    /* Addresses for VECTOR_TABLE and VECTOR_RAM come from the linker file */  
    extern uint32 __VECTOR_TABLE[];
    extern uint32 __VECTOR_RAM[];

    /* Copy the vector table to RAM */
    if (__VECTOR_RAM != __VECTOR_TABLE)
    {
        for (n = 0; n < 0x410; n++)
            __VECTOR_RAM[n] = __VECTOR_TABLE[n];
    }
    /* Point the VTOR to the new copy of the vector table */
    write_vtor((uint32)__VECTOR_RAM);    
    
    /* Get the addresses for the .data section (initialized data section) */
    uint8* data_ram = __section_begin(".data");
    uint8* data_rom = __section_begin(".data_init");
    uint8* data_rom_end = __section_end(".data_init");
    
    /* Copy initialized data from ROM to RAM */
    n = data_rom_end - data_rom;
    while (n--)
      *data_ram++ = *data_rom++;
 
 
    /* Get the addresses for the .bss section (zero-initialized data) */
    uint8* bss_start = __section_begin(".bss");
    uint8* bss_end = __section_end(".bss");
    
    /* Clear the zero-initialized data section */
    n = bss_end - bss_start;
    while(n--)
      *bss_start++ = 0;    
    
    /* Get addresses for any code sections that need to be copied from ROM to RAM.
     * The IAR tools have a predefined keyword that can be used to mark individual
     * functions for execution from RAM. Add "__ramfunc" before the return type in
     * the function prototype for any routines you need to execute from RAM instead 
     * of ROM. ex: __ramfunc void foo(void);
     */
    uint8* code_relocate_ram = __section_begin("CodeRelocateRam");
    uint8* code_relocate = __section_begin("CodeRelocate");
    uint8* code_relocate_end = __section_end("CodeRelocate");
    
    /* Copy functions from ROM to RAM */
    n = code_relocate_end - code_relocate;
    while (n--)
      *code_relocate_ram++ = *code_relocate++;
}