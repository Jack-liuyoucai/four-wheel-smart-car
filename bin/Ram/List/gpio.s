///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.10.1.52143/W32 for ARM    06/Jan/2012  23:59:29 /
// Copyright 1999-2010 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Sources\ /
//                    C\Component_C\gpio.c                                    /
//    Command line =  F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Sources\ /
//                    C\Component_C\gpio.c -D IAR -D TWR_K60N512 -lCN         /
//                    F:\Kinetis\IARK60X256\10_MK60X256_test_pll\bin\Ram\List /
//                    \ -lB F:\Kinetis\IARK60X256\10_MK60X256_test_pll\bin\Ra /
//                    m\List\ -o F:\Kinetis\IARK60X256\10_MK60X256_test_pll\b /
//                    in\Ram\Obj\ --no_cse --no_unroll --no_inline            /
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
//                    \gpio.s                                                 /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME gpio

        PUBLIC gpio_ctrl
        PUBLIC gpio_init
        PUBLIC gpio_reverse

// F:\Kinetis\IARK60X256\10_MK60X256_test_pll\src\Sources\C\Component_C\gpio.c
//    1 //-------------------------------------------------------------------------*
//    2 // �ļ���: gpio.c                                                          *
//    3 // ˵  ��: gpio���������ļ�                                                *
//    4 //-------------------------------------------------------------------------*
//    5 
//    6 #include "gpio.h"     //����gpioͷ�ļ�
//    7 
//    8 //-------------------------------------------------------------------------*
//    9 //������: gpio_init                                                        *
//   10 //��  ��: ��ʼ��gpio                                                       * 
//   11 //��  ��: port:�˿���                                                      *
//   12 //        index:ָ���˿�����                                               *
//   13 //        dir:���ŷ���,0=����,1=���                                       * 
//   14 //        data:��ʼ״̬,0=�͵�ƽ,1=�ߵ�ƽ                                  *
//   15 //��  ��: ��                                                               *
//   16 //˵  ��: ��                                                               *
//   17 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   18 void gpio_init (GPIO_MemMapPtr port, int index, int dir,int data)
//   19 {
gpio_init:
        PUSH     {R4-R6}
//   20      PORT_MemMapPtr p;
//   21      switch((uint32)port)
        MOVS     R5,R0
        LDR.N    R6,??DataTable0  ;; 0x400ff000
        SUBS     R5,R5,R6
        BEQ.N    ??gpio_init_0
        SUBS     R5,R5,#+64
        BEQ.N    ??gpio_init_1
        SUBS     R5,R5,#+64
        BEQ.N    ??gpio_init_2
        SUBS     R5,R5,#+64
        BEQ.N    ??gpio_init_3
        SUBS     R5,R5,#+64
        BEQ.N    ??gpio_init_4
        B.N      ??gpio_init_5
//   22      {
//   23      case 0x400FF000u:
//   24          p = PORTA_BASE_PTR;
??gpio_init_0:
        LDR.N    R4,??DataTable0_1  ;; 0x40049000
//   25          break;
        B.N      ??gpio_init_6
//   26      case 0x400FF040u:
//   27          p = PORTB_BASE_PTR;
??gpio_init_1:
        LDR.N    R4,??DataTable0_2  ;; 0x4004a000
//   28          break;
        B.N      ??gpio_init_6
//   29      case 0x400FF080u:
//   30          p = PORTC_BASE_PTR;
??gpio_init_2:
        LDR.N    R4,??DataTable0_3  ;; 0x4004b000
//   31          break;
        B.N      ??gpio_init_6
//   32      case 0x400FF0C0u:
//   33          p = PORTD_BASE_PTR;
??gpio_init_3:
        LDR.N    R4,??DataTable0_4  ;; 0x4004c000
//   34          break;
        B.N      ??gpio_init_6
//   35      case 0x400FF100u:
//   36          p = PORTE_BASE_PTR;
??gpio_init_4:
        LDR.N    R4,??DataTable0_5  ;; 0x4004d000
//   37          break;
        B.N      ??gpio_init_6
//   38      default:
//   39          break;
//   40      }
//   41      PORT_PCR_REG(p,index)=(0|PORT_PCR_MUX(1));
??gpio_init_5:
??gpio_init_6:
        MOV      R5,#+256
        STR      R5,[R4, R1, LSL #+2]
//   42 
//   43      if(dir == 1)//output
        CMP      R2,#+1
        BNE.N    ??gpio_init_7
//   44      {
//   45     	 GPIO_PDDR_REG(port) |= (1<<index);
        LDR      R2,[R0, #+20]
        MOVS     R4,#+1
        LSLS     R4,R4,R1
        ORRS     R2,R4,R2
        STR      R2,[R0, #+20]
//   46     	 if(data == 1)//output
        CMP      R3,#+1
        BNE.N    ??gpio_init_8
//   47 			  GPIO_PDOR_REG(port) |= (1<<index);
        LDR      R2,[R0, #+0]
        MOVS     R3,#+1
        LSLS     R1,R3,R1
        ORRS     R1,R1,R2
        STR      R1,[R0, #+0]
        B.N      ??gpio_init_9
//   48 		 else
//   49 			  GPIO_PDOR_REG(port) &= ~(1<<index);
??gpio_init_8:
        LDR      R2,[R0, #+0]
        MOVS     R3,#+1
        LSLS     R1,R3,R1
        BICS     R1,R2,R1
        STR      R1,[R0, #+0]
        B.N      ??gpio_init_9
//   50      }
//   51          
//   52      else
//   53          GPIO_PDDR_REG(port) &= ~(1<<index);
??gpio_init_7:
        LDR      R2,[R0, #+20]
        MOVS     R3,#+1
        LSLS     R1,R3,R1
        BICS     R1,R2,R1
        STR      R1,[R0, #+20]
//   54 
//   55 }
??gpio_init_9:
        POP      {R4-R6}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0:
        DC32     0x400ff000

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_1:
        DC32     0x40049000

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_2:
        DC32     0x4004a000

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_3:
        DC32     0x4004b000

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_4:
        DC32     0x4004c000

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable0_5:
        DC32     0x4004d000
//   56 
//   57 //-------------------------------------------------------------------------* 
//   58 //������: gpio_ctrl                                                        *
//   59 //��  ��: ��������״̬                                                     *
//   60 //��  ��: port:�˿���                                                      *
//   61 //        index:ָ���˿�����                                               *
//   62 //        data: ״̬,0=�͵�ƽ,1=�ߵ�ƽ                                     *
//   63 //��  ��: ��                                                               *
//   64 //˵  ��: ��                                                               *
//   65 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   66 void gpio_ctrl (GPIO_MemMapPtr port, int index, int data)
//   67 {
//   68     if(data == 1)//output
gpio_ctrl:
        CMP      R2,#+1
        BNE.N    ??gpio_ctrl_0
//   69          GPIO_PDOR_REG(port) |= (1<<index);
        LDR      R2,[R0, #+0]
        MOVS     R3,#+1
        LSLS     R1,R3,R1
        ORRS     R1,R1,R2
        STR      R1,[R0, #+0]
        B.N      ??gpio_ctrl_1
//   70     else
//   71          GPIO_PDOR_REG(port) &= ~(1<<index);
??gpio_ctrl_0:
        LDR      R2,[R0, #+0]
        MOVS     R3,#+1
        LSLS     R1,R3,R1
        BICS     R1,R2,R1
        STR      R1,[R0, #+0]
//   72 }
??gpio_ctrl_1:
        BX       LR               ;; return
//   73 
//   74 //-----------------------------------------------------------------------* 
//   75 //������: gpio_reverse                                                   *
//   76 //��  ��: �ı�����״̬                                                   *
//   77 //��  ��: port:�˿���;                                                   *
//   78 //        index:ָ���˿�����                                             *
//   79 //��  ��: ��                                                             *
//   80 //˵  ��: ��                                                             *
//   81 //-----------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   82 void gpio_reverse (GPIO_MemMapPtr port, int index)
//   83 {
//   84     GPIO_PDOR_REG(port) ^= (1<<index);
gpio_reverse:
        LDR      R2,[R0, #+0]
        MOVS     R3,#+1
        LSLS     R1,R3,R1
        EORS     R1,R1,R2
        STR      R1,[R0, #+0]
//   85 }
        BX       LR               ;; return

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)

        END
// 
// 194 bytes in section .text
// 
// 194 bytes of CODE memory
//
//Errors: none
//Warnings: none
