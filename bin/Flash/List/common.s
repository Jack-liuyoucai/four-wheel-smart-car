///////////////////////////////////////////////////////////////////////////////
//                                                                            /
//                                                      11/Aug/2012  21:15:56 /
// IAR ANSI C/C++ Compiler V6.30.4.23288/W32 EVALUATION for ARM               /
// Copyright 1999-2011 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  C:\Users\Administrator\Desktop\�ɴ�\src\Sources\C\Frame /
//                    _C\common.c                                             /
//    Command line =  C:\Users\Administrator\Desktop\�ɴ�\src\Sources\C\Frame /
//                    _C\common.c -D IAR -D TWR_K60N512 -lCN                  /
//                    C:\Users\Administrator\Desktop\�ɴ�\bin\Flash\List\     /
//                    -lB C:\Users\Administrator\Desktop\�ɴ�\bin\Flash\List\ /
//                     -o C:\Users\Administrator\Desktop\�ɴ�\bin\Flash\Obj\  /
//                    --no_cse --no_unroll --no_inline --no_code_motion       /
//                    --no_tbaa --no_clustering --no_scheduling --debug       /
//                    --endian=little --cpu=Cortex-M4 -e --fpu=None           /
//                    --dlib_config G:\irm\arm\INC\c\DLib_Config_Normal.h -I  /
//                    C:\Users\Administrator\Desktop\�ɴ�\src\Sources\H\ -I   /
//                    C:\Users\Administrator\Desktop\�ɴ�\src\Sources\H\Compo /
//                    nent_H\ -I C:\Users\Administrator\Desktop\�ɴ�\src\Sour /
//                    ces\H\Frame_H\ -Ol --use_c++_inline                     /
//    List file    =  C:\Users\Administrator\Desktop\�ɴ�\bin\Flash\List\comm /
//                    on.s                                                    /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME common

        #define SHT_PROGBITS 0x1

        PUBLIC disable_irq
        PUBLIC enable_irq
        PUBLIC set_irq_priority
        PUBLIC stop
        PUBLIC wait
        PUBLIC write_vtor

// C:\Users\Administrator\Desktop\�ɴ�\src\Sources\C\Frame_C\common.c
//    1 //-------------------------------------------------------------------------*
//    2 // �ļ���: common.h (ͨ��ͷ�ļ�)                                           *
//    3 // ˵  ��:                                                                 *
//    4 //-------------------------------------------------------------------------*
//    5 
//    6 #include "common.h"
//    7 
//    8 //-------------------------------------------------------------------------*
//    9 //������: stop                                                             *
//   10 //��  ��: ����CPUΪSTOPģʽ                                                * 
//   11 //��  ��: ��								   *	
//   12 //��  ��: ��                                                               *
//   13 //˵  ��: ��                                                               *
//   14 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//   15 void stop (void)
//   16 {
//   17     //��λSLEEPDEEP��ʹ��STOPģʽ
//   18     SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
stop:
        LDR.N    R0,??DataTable4  ;; 0xe000ed10
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.N    R1,??DataTable4  ;; 0xe000ed10
        STR      R0,[R1, #+0]
//   19     //����STOPģʽ
//   20     asm("WFI");
        WFI              
//   21 }
        BX       LR               ;; return
//   22 
//   23 //-------------------------------------------------------------------------*
//   24 //������: wait                                                             *
//   25 //��  ��: ����CPUΪWAITģʽ                                                * 
//   26 //��  ��: ��								   *	
//   27 //��  ��: ��                                                               *
//   28 //˵  ��: ��                                                               *
//   29 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//   30 void wait (void)
//   31 {
//   32     //��SLEEPDEEPλ��ȷ������WAITģʽ
//   33     SCB_SCR &= ~SCB_SCR_SLEEPDEEP_MASK;	
wait:
        LDR.N    R0,??DataTable4  ;; 0xe000ed10
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.N    R1,??DataTable4  ;; 0xe000ed10
        STR      R0,[R1, #+0]
//   34     //����WAITģʽ
//   35     asm("WFI");
        WFI              
//   36 }
        BX       LR               ;; return
//   37 
//   38 //-------------------------------------------------------------------------*
//   39 //������: write_vtor                                                       *
//   40 //��  ��: �����ж�������ƫ�ƼĴ�����ֵ                                     * 
//   41 //��  ��: Ҫ���ĵ�ֵ    						   *	
//   42 //��  ��: ��                                                               *
//   43 //˵  ��: ��                                                               *
//   44 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   45 void write_vtor (int vtor)
//   46 {
//   47     //д��ֵ
//   48     SCB_VTOR = vtor;	
write_vtor:
        LDR.N    R1,??DataTable4_1  ;; 0xe000ed08
        STR      R0,[R1, #+0]
//   49 }
        BX       LR               ;; return
//   50 
//   51 //-------------------------------------------------------------------------*
//   52 //������: enable_irq                                                       *
//   53 //��  ��: ʹ��irq�ж�                                                      * 
//   54 //��  ��: irq:irq��       						   *	
//   55 //��  ��: ��                                                               *
//   56 //˵  ��: irq�Ų����ж�������                                              *
//   57 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   58 void enable_irq (int irq)
//   59 {
//   60     int div;
//   61 
//   62     //ȷ��irq��Ϊ��Ч��irq��
//   63     if (irq > 91)	irq=91;
enable_irq:
        CMP      R0,#+92
        BLT.N    ??enable_irq_0
        MOVS     R0,#+91
//   64     
//   65     //ȷ����Ӧ��NVICISER
//   66     div = irq/32;
??enable_irq_0:
        MOVS     R1,#+32
        SDIV     R1,R0,R1
//   67     
//   68     switch (div)
        CMP      R1,#+0
        BEQ.N    ??enable_irq_1
        CMP      R1,#+2
        BEQ.N    ??enable_irq_2
        BCC.N    ??enable_irq_3
        B.N      ??enable_irq_4
//   69     {
//   70     	case 0x0:
//   71               NVICICPR0 = 1 << (irq%32);
??enable_irq_1:
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R1,R1,R3
        LDR.N    R2,??DataTable4_2  ;; 0xe000e280
        STR      R1,[R2, #+0]
//   72               NVICISER0 = 1 << (irq%32);
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R0,R1,R3
        LDR.N    R1,??DataTable4_3  ;; 0xe000e100
        STR      R0,[R1, #+0]
//   73               break;
        B.N      ??enable_irq_4
//   74     	case 0x1:
//   75               NVICICPR1 = 1 << (irq%32);
??enable_irq_3:
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R1,R1,R3
        LDR.N    R2,??DataTable4_4  ;; 0xe000e284
        STR      R1,[R2, #+0]
//   76               NVICISER1 = 1 << (irq%32);
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R0,R1,R3
        LDR.N    R1,??DataTable4_5  ;; 0xe000e104
        STR      R0,[R1, #+0]
//   77               break;
        B.N      ??enable_irq_4
//   78     	case 0x2:
//   79               NVICICPR2 = 1 << (irq%32);
??enable_irq_2:
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R1,R1,R3
        LDR.N    R2,??DataTable4_6  ;; 0xe000e288
        STR      R1,[R2, #+0]
//   80               NVICISER2 = 1 << (irq%32);
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R0,R1,R3
        LDR.N    R1,??DataTable4_7  ;; 0xe000e108
        STR      R0,[R1, #+0]
//   81               break;
//   82     }              
//   83 }
??enable_irq_4:
        BX       LR               ;; return
//   84 
//   85 //-------------------------------------------------------------------------*
//   86 //������: disable_irq                                                      *
//   87 //��  ��: ��ֹirq�ж�                                                      * 
//   88 //��  ��: irq:irq��       						   *	
//   89 //��  ��: ��                                                               *
//   90 //˵  ��: irq�Ų����ж�������                                              *
//   91 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   92 void disable_irq (int irq)
//   93 {
//   94     int div;
//   95     
//   96     //ȷ��irq��Ϊ��Ч��irq��
//   97     if (irq > 91)	irq=91;
disable_irq:
        CMP      R0,#+92
        BLT.N    ??disable_irq_0
        MOVS     R0,#+91
//   98     
//   99     //ȷ����Ӧ��NVICISER
//  100     div = irq/32;
??disable_irq_0:
        MOVS     R1,#+32
        SDIV     R1,R0,R1
//  101     
//  102     switch (div)
        CMP      R1,#+0
        BEQ.N    ??disable_irq_1
        CMP      R1,#+2
        BEQ.N    ??disable_irq_2
        BCC.N    ??disable_irq_3
        B.N      ??disable_irq_4
//  103     {
//  104     	case 0x0:
//  105                NVICICER0 = 1 << (irq%32);
??disable_irq_1:
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R0,R1,R3
        LDR.N    R1,??DataTable4_8  ;; 0xe000e180
        STR      R0,[R1, #+0]
//  106               break;
        B.N      ??disable_irq_4
//  107     	case 0x1:
//  108               NVICICER1 = 1 << (irq%32);
??disable_irq_3:
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R0,R1,R3
        LDR.N    R1,??DataTable4_9  ;; 0xe000e184
        STR      R0,[R1, #+0]
//  109               break;
        B.N      ??disable_irq_4
//  110     	case 0x2:
//  111               NVICICER2 = 1 << (irq%32);
??disable_irq_2:
        MOVS     R1,#+1
        MOVS     R2,#+32
        SDIV     R3,R0,R2
        MLS      R3,R3,R2,R0
        LSLS     R0,R1,R3
        LDR.N    R1,??DataTable4_10  ;; 0xe000e188
        STR      R0,[R1, #+0]
//  112               break;
//  113     }              
//  114 }
??disable_irq_4:
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4:
        DC32     0xe000ed10

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_1:
        DC32     0xe000ed08

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_2:
        DC32     0xe000e280

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_3:
        DC32     0xe000e100

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_4:
        DC32     0xe000e284

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_5:
        DC32     0xe000e104

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_6:
        DC32     0xe000e288

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_7:
        DC32     0xe000e108

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_8:
        DC32     0xe000e180

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_9:
        DC32     0xe000e184

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4_10:
        DC32     0xe000e188
//  115  
//  116 //-------------------------------------------------------------------------*
//  117 //������: set_irq_priority                                                 *
//  118 //��  ��: ����irq�жϺ����ȼ�                                              * 
//  119 //��  ��: irq:irq��         						   *	
//  120 //        prio:���ȼ�						           *	
//  121 //��  ��: ��                                                               *
//  122 //˵  ��: irq�Ų����ж�������                                              *
//  123 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  124 void set_irq_priority (int irq, int prio)
//  125 {
//  126     uint8 *prio_reg;
//  127 
//  128     //ȷ��irq�ź����ȼ���Ч
//  129     if (irq > 91)	irq=91;
set_irq_priority:
        CMP      R0,#+92
        BLT.N    ??set_irq_priority_0
        MOVS     R0,#+91
//  130     if (prio > 15)	prio=15;
??set_irq_priority_0:
        CMP      R1,#+16
        BLT.N    ??set_irq_priority_1
        MOVS     R1,#+15
//  131 
//  132     //ȷ����Ӧ��NVICISER
//  133     prio_reg = (uint8 *)(((uint32)&NVICIP0) + irq);
??set_irq_priority_1:
        ADD      R0,R0,#-536870912
        ADDS     R0,R0,#+58368
//  134     //�������ȼ�
//  135     *prio_reg = ( (prio&0xF) << (8 - ARM_INTERRUPT_LEVEL_BITS) );             
        LSLS     R1,R1,#+4
        STRB     R1,[R0, #+0]
//  136 }
        BX       LR               ;; return

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  137 
// 
// 342 bytes in section .text
// 
// 342 bytes of CODE memory
//
//Errors: none
//Warnings: none
