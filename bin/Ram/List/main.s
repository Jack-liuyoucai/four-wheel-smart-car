///////////////////////////////////////////////////////////////////////////////
//                                                                            /
//                                                      03/May/2013  16:26:13 /
// IAR ANSI C/C++ Compiler V6.30.4.23288/W32 EVALUATION for ARM               /
// Copyright 1999-2011 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  C:\Users\Administrator\Desktop\���12\src\Sources\C\mai /
//                    n.c                                                     /
//    Command line =  C:\Users\Administrator\Desktop\���12\src\Sources\C\mai /
//                    n.c -D IAR -D TWR_K60N512 -lCN                          /
//                    C:\Users\Administrator\Desktop\���12\bin\Ram\List\     /
//                    -lB C:\Users\Administrator\Desktop\���12\bin\Ram\List\ /
//                     -o C:\Users\Administrator\Desktop\���12\bin\Ram\Obj\  /
//                    --no_cse --no_unroll --no_inline --no_code_motion       /
//                    --no_tbaa --no_clustering --no_scheduling --debug       /
//                    --endian=little --cpu=Cortex-M4 -e --fpu=None           /
//                    --dlib_config G:\irm\arm\INC\c\DLib_Config_Normal.h -I  /
//                    C:\Users\Administrator\Desktop\���12\src\Sources\H\    /
//                    -I C:\Users\Administrator\Desktop\���12\src\Sources\H\ /
//                    Component_H\ -I C:\Users\Administrator\Desktop\���12\s /
//                    rc\Sources\H\Frame_H\ -Ol --use_c++_inline              /
//    List file    =  C:\Users\Administrator\Desktop\���12\bin\Ram\List\main /
//                    .s                                                      /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME main

        #define SHT_PROGBITS 0x1

        EXTERN LCD_CLS
        EXTERN LCD_CLS_ROW
        EXTERN LCD_IO_Init
        EXTERN LCD_Init
        EXTERN LCD_P6x8Cha
        EXTERN LCD_P6x8Num
        EXTERN LCD_P6x8Str
        EXTERN __aeabi_d2iz
        EXTERN __aeabi_ddiv
        EXTERN __aeabi_i2f
        EXTERN __aeabi_ui2d
        EXTERN __aeabi_ui2f
        EXTERN enable_irq

        PUBLIC Deal_BlackEdge
        PUBLIC Delay_MS
        PUBLIC EXIT_Init
        PUBLIC ImageReady
        PUBLIC Image_sample_pos
        PUBLIC Initial
        PUBLIC Keyscan
        PUBLIC LCD_change_value
        PUBLIC LPTMR_Init
        PUBLIC LineCount
        PUBLIC OT
        PUBLIC OddEvenStatus
        PUBLIC PORT_Init
        PUBLIC PTC_isr
        PUBLIC SCI0_send_mesage
        PUBLIC Search_BlackEdge
        PUBLIC Search_WhiteBase
        PUBLIC Servor_Control
        PUBLIC UART0_Init
        PUBLIC VideoImage1
        PUBLIC VideoImageLine
        PUBLIC Videoclo_Flag
        PUBLIC add_NUM
        PUBLIC add_page
        PUBLIC angle
        PUBLIC back_weight
        PUBLIC bottom_whitebase
        PUBLIC bow_d
        PUBLIC bow_p
        PUBLIC center_white
        PUBLIC come_bow_d
        PUBLIC come_bow_p
        PUBLIC cross_flag
        PUBLIC current_deal_line
        PUBLIC down_line
        PUBLIC error_server
        PUBLIC error_servor_d
        PUBLIC error_servor_p
        PUBLIC f_abs
        PUBLIC f_abs16
        PUBLIC f_abs32
        PUBLIC find_whitebase_flag
        PUBLIC font_weight
        PUBLIC hw_FTM_init
        PUBLIC key_down
        PUBLIC lcd_line_num
        PUBLIC lcd_page_num
        PUBLIC left_black
        PUBLIC left_lost_end_line
        PUBLIC left_lost_flag
        PUBLIC left_lost_start_line
        PUBLIC left_top_whiteline
        PUBLIC main
        PUBLIC max_speed
        PUBLIC mid_angle
        PUBLIC middle_weight
        PUBLIC min_speed
        PUBLIC pllinit180M
        PUBLIC pre_show
        PUBLIC re_error_server
        PUBLIC re_top_error
        PUBLIC re_top_whiteline
        PUBLIC redraw
        PUBLIC redraw_control
        PUBLIC refer_road_width
        PUBLIC right_black
        PUBLIC right_lost_end_line
        PUBLIC right_lost_flag
        PUBLIC right_lost_start_line
        PUBLIC right_top_whiteline
        PUBLIC scan_boma
        PUBLIC scan_control
        PUBLIC se_scope
        PUBLIC send_mes
        PUBLIC ser_ref_d
        PUBLIC ser_ref_p
        PUBLIC speed
        PUBLIC speed_control
        PUBLIC speed_d
        PUBLIC speed_ed
        PUBLIC speed_ed_p
        PUBLIC speed_error
        PUBLIC speed_except
        PUBLIC speed_feedback
        PUBLIC speed_i
        PUBLIC speed_i_error
        PUBLIC speed_p
        PUBLIC speed_re_error
        PUBLIC speed_set
        PUBLIC stopflag
        PUBLIC straight_d
        PUBLIC straight_p
        PUBLIC sub_NUM
        PUBLIC sub_page
        PUBLIC top_error
        PUBLIC top_white_refer
        PUBLIC top_whiteline
        PUBLIC uart0_isr
        PUBLIC up_line
        PUBLIC white_refer
        PUBLIC whitebase_line_cnt
        PUBLIC whitepoint_end
        PUBLIC whitepoint_start

// C:\Users\Administrator\Desktop\���12\src\Sources\C\main.c
//    1 /*����˵��
//    2 ��������Ϊ ���һ�� 4.29   ��ǿ�Ա����ߵ�ɨ��*   ������С��Һ�����Ͱ������Գɹ�*/
//    3 
//    4 //ʹ������ӵľ����Ч׼ȷ��
//    5 #include "MK60N512VMD100.h " /* include peripheral declarations */
//    6 #include "includes.h"
//    7 
//    8 #include"LCDDriver.h"
//    9 
//   10 #define GPIO_PIN_MASK      0x1Fu    //0x1f=31,����λ��Ϊ0--31��Ч
//   11 #define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //�ѵ�ǰλ��1
//   12 #define BUS_CLOCK  100  //(MHZ)50 82 90 100 105 110 115//�������õ��ں�ʱ�ӵ�������ʱ��100M
//   13 #define BAUD 19200     //������
//   14 #define CORE_CLOCK 180
//   15 
//   16 //--------------------------�ɼ�ͼ�����ر���-------------------------------------//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 bool     OddEvenStatus;		  //��ż��״̬��־
OddEvenStatus:
        DS8 1
//   18 #define  OddStatus  0
//   19 #define  EvenStatus 1
//   20 #define  ODD_EVEN_STATUS  (bool)(0x00000001 & (GPIOC_PDIR >> 1))  //��ż�任��־  ����ptc�˿ڵĵ�1λ���ƶ�����1
//   21 #define VIF_START	0   	 //	��ʼģʽ				 
//   22 #define VIF_WAITSAMPLE	1        //   �ȴ�ģʽ
//   23 #define VIF_SAMPLELINE	2         //   ��ȥ�����е�״̬
//   24 #define VIF Videoclo_Flag         //
//   25 #define PIANYI 172       //ʵ�ʲɼ�����ΪCOLUMN + PIANYI��PIANYIΪÿ��������
//   26 #define VIDEO_START_LINE  24	//ͼ��ɼ���ʼ��

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   27 bool ImageReady;               //ͼ��׼���ñ�־
ImageReady:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   28 uint8 Videoclo_Flag, VideoImageLine;   //�ɼ�״̬��־λ�����ж�ʵ�ʲɼ�����������
Videoclo_Flag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
VideoImageLine:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   29 uint8 LineCount;                       //���жϲɼ�����������
LineCount:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   30 uint8 Image_sample_pos;                //�������������������ʱ����һ��ͼ��ɼ�������������һ��������ͼ��
Image_sample_pos:
        DS8 1
//   31 
//   32 
//   33 //-------------------------����ͼ�����ر���-----------------------------------------//
//   34 #define ROW 65	                 //�ɼ�����
//   35 #define COLUMN	161 		//ÿ�е���
//   36 #define MID  80                 //������ 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   37 uint8 VideoImage1[ROW][COLUMN] =       //ԭʼͼ������[0][0]�����½�
VideoImage1:
        DS8 10468
//   38 {
//   39    0
//   40 };
//   41 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   42 int16 left_black[ROW]=                 //������ߵĲɼ�����
left_black:
        DS8 132
//   43 {
//   44   0
//   45 };

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   46 int16 right_black[ROW]=                //�ұ����ߵĲɼ�����
right_black:
        DS8 132
//   47 {
//   48   0
//   49 };

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   50 int16 center_white[ROW]=              //����������ģ����ߵ�����
center_white:
        DS8 132
//   51 {
//   52   0
//   53 };
//   54 
//   55 //-------------------------------------�����ߺ���----------------------------------//
//   56 #define MIN_WHITEBASE_POINT 20                    //���������׵������Ϊ��׼��Ҫ��
//   57 #define MIN_WHITE_LINE 3                        //��Ϊ���߻�׼�е���С����Ҫ��
//   58 #define WHITE_LOSTPOINT_NUM 3                  //���ߵİ��߿��С�����ֵ���ж�Ϊ�����Ч
//   59 //�⼸λ������ʵ���������д洢�����Ƕ��������б��
//   60 #define LEFT_OUT_LOST 0                       //�����ߵ�����ͼ�����߽���Ϊ��ʧ
//   61 #define RIGHT_OUT_LOST COLUMN-1                     //���������Ķ���   
//   62 #define LOST 1                        // ���ҵ�����Ķ���
//   63 #define UN_LOST  0                 //�����ߵ�����ͼ����ұ߽���Ϊ��ʧ
//   64 
//   65 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   66 uint8 left_lost_start_line=0;
left_lost_start_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   67 uint8 left_lost_end_line=0;
left_lost_end_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   68 bool   left_lost_flag=UN_LOST;
left_lost_flag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   69 uint8 right_lost_start_line=0;
right_lost_start_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   70 uint8 right_lost_end_line=0;
right_lost_end_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   71 bool  right_lost_flag=UN_LOST;
right_lost_flag:
        DS8 1
//   72 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   73 uint8 current_deal_line=0;                //��¼��������׼�е�ʱ��ĵ�ǰ������� 
current_deal_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   74 bool find_whitebase_flag=0;               //�Ƿ��ְ��߻�׼��־
find_whitebase_flag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   75 uint8 whitepoint_start=0;                //�������Ұ׵㿪ʼ��
whitepoint_start:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   76 uint8 whitepoint_end=0;                 //�������Ұ׵������
whitepoint_end:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   77 uint8 whitebase_line_cnt=0;
whitebase_line_cnt:
        DS8 1
//   78 
//   79 #define SEARCH_SCOPE   10               //���������ķ�ΧΪ10

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   80 int se_scope= SEARCH_SCOPE;
se_scope:
        DATA
        DC32 10

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   81 uint16 refer_road_width[ROW] ={118,117,116,177,177,115,114,113,111,110,
refer_road_width:
        DATA
        DC16 118, 117, 116, 177, 177, 115, 114, 113, 111, 110, 108, 108, 107
        DC16 106, 104, 103, 102, 100, 99, 99, 97, 96, 95, 94, 92, 90, 90, 89
        DC16 86, 85, 83, 82, 81, 78, 77, 76, 75, 73, 71, 70, 68, 67, 65, 63, 62
        DC16 60, 59, 57, 57, 54, 52, 52, 50, 47, 46, 43, 43, 40, 37, 37, 35, 34
        DC16 31, 29, 28
        DC8 0, 0
//   82                                108,108,107,106,104,103,102,100,99,99,
//   83                                97,96,95,94,92,90,90,89,86,85,
//   84                                83,82,81,78,77,76,75,73,71,70,
//   85                                68,67,65,63,62,60,59,57,57,54,
//   86                                52,52,50,47,46,43,43,40,37,37,
//   87                                35,34,31,29,28,};

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   88 int OT=60;                                     //�ж�Ϊ�Ҷ�ֵ�������ص���С�Ҷȵ�����ֵ
OT:
        DATA
        DC32 60

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   89 int top_whiteline=ROW-1;                          //ͼ������
top_whiteline:
        DATA
        DC32 64

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   90 int left_top_whiteline=ROW-1;
left_top_whiteline:
        DATA
        DC32 64

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   91 int right_top_whiteline=ROW-1;
right_top_whiteline:
        DATA
        DC32 64

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   92 int bottom_whitebase=0;                       //ͼ��Ļ�׼�� 
bottom_whitebase:
        DS8 4

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   93 int white_refer=0;                            //��׼���ϵ��������е�
white_refer:
        DS8 4

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   94 int top_white_refer=0;                        //ͼ��Ķ��е��������е�
top_white_refer:
        DS8 4
//   95 
//   96 //--------------------------------------�����������ز���-----------------------//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   97 bool cross_flag=0;                            //ʮ�ֵ�·�ı�־
cross_flag:
        DS8 1
//   98 
//   99 //-----���ڹ���ѡ��----//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  100 uint8 send_mes;              //������λ����������������ѡ��ͬ�Ĵ��ڹ���
send_mes:
        DS8 1
//  101 
//  102 //------------------------------------������ƺ����Ĳ���-----------------------------------//

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  103 uint16 speed_feedback=0;               //�������ķ���ֵ                  //
speed_feedback:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  104 int16 speed_re_error=0;
speed_re_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  105 bool stopflag=0;//�ٶȷ���
stopflag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  106 int16 speed_i_error=0;
speed_i_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  107 int16 speed_error=0;
speed_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  108 int16 speed_set=0;
speed_set:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  109 uint16 speed_p=8;
speed_p:
        DATA
        DC16 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  110 uint16 speed_i=0;
speed_i:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  111 uint16 speed_d=0;
speed_d:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  112 uint16 speed_ed_p=0;
speed_ed_p:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  113 int16 speed_ed=0;
speed_ed:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  114 int16 speed=0;
speed:
        DS8 2
//  115 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  116 int16 max_speed=800;
max_speed:
        DATA
        DC16 800

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  117 int16 min_speed=0;
min_speed:
        DS8 2
//  118 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  119 uint16 speed_except=0;
speed_except:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  120 uint8 re_top_whiteline=0;
re_top_whiteline:
        DS8 1
//  121 
//  122 //-----------------------------------������ƺ����ı���---------------------------------//

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  123 uint16 mid_angle=1354;//  �������ҹգ�˵��С�ڰ���ֵ
mid_angle:
        DATA
        DC16 1354

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  124 int16 error_server=0;
error_server:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  125 int16 re_error_server=0;
re_error_server:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  126 int16 re_top_error=0;
re_top_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  127 int16 top_error=0;
top_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  128 int16 angle=0;
angle:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  129 uint8 font_weight=30;
font_weight:
        DATA
        DC8 30

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  130 uint8 middle_weight=35;
middle_weight:
        DATA
        DC8 35

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  131 uint8 back_weight=40;
back_weight:
        DATA
        DC8 40

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  132 uint16 error_servor_p=0;
error_servor_p:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  133 uint16 error_servor_d=0;//except_servor_i=0,
error_servor_d:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  134 uint16 ser_ref_p = 12;
ser_ref_p:
        DATA
        DC16 12

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  135 uint16 ser_ref_d = 0;                
ser_ref_d:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  136 uint16 straight_p = 40;
straight_p:
        DATA
        DC16 40

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  137 uint16 straight_d = 35;
straight_d:
        DATA
        DC16 35

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  138 uint16 come_bow_p = 60;
come_bow_p:
        DATA
        DC16 60

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  139 uint16 come_bow_d = 25;
come_bow_d:
        DATA
        DC16 25

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  140 uint16 bow_p = 70;
bow_p:
        DATA
        DC16 70

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  141 uint16 bow_d = 15;
bow_d:
        DATA
        DC16 15
//  142     //bow �� straightֱ��
//  143 //-------------------------------�����Ķ���-------------------------------------------//
//  144 #define LCD_ROW 7                      //СҺ������ʵ�ʵ�����Ϊ8��

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  145 bool add_page=0;             //��ҳ
add_page:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  146 bool sub_page=0;             //��ҳ
sub_page:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  147 bool up_line=0;              //����   ����
up_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  148 bool down_line=0;            //����   ����
down_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  149 bool add_NUM=0;              //������ֵ  ��
add_NUM:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  150 bool sub_NUM=0;              //������ֵ  �� 
sub_NUM:
        DS8 1
//  151 
//  152 //--------------------------------����Ķ������--------------------------------------//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  153 bool redraw_control=0;         //ˢ���Ŀ���λ
redraw_control:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  154 bool scan_control=0;           //ɨ�貦��Ŀ���λ
scan_control:
        DS8 1
//  155 
//  156 
//  157 //------------------------------------LCD��������-------------------------------------//
//  158 void pre_show(void);        //��һ���Ԥ��ʾ
//  159 void redraw(void);          //ˢƵĻ
//  160 void Keyscan(void);          //ɨ�貦��
//  161 void LCD_change_value(unsigned char page,unsigned char m,int i);//������ֵ
//  162 void Delay_MS(uint32 ms);       //��ʱ����

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  163 uint8 lcd_page_num=0;        //Һ������ҳ��
lcd_page_num:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  164 uint8 lcd_line_num=0;        //Һ����������
lcd_line_num:
        DS8 1
//  165 
//  166 
//  167 //---------------------------�����ʼ��--------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  168 void Initial(void)
//  169 {
Initial:
        PUSH     {R4}
//  170   int16 i,j;
//  171      for(i = 0;i < ROW;i++)
        MOVS     R0,#+0
        B.N      ??Initial_0
//  172        {
//  173          left_black[i] = 0;
??Initial_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable6
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
//  174          right_black[i] = 0;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable6_1
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
//  175          center_white[i] = 0;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable6_2
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
//  176        }
        ADDS     R0,R0,#+1
??Initial_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+65
        BLT.N    ??Initial_1
//  177    //����ͼ��ĵ�28�е�41�д���ͼ�������ƫ�� �����������д������
//  178      for(i = 28;i < 42; i++ )
        MOVS     R0,#+28
        B.N      ??Initial_2
//  179      {
//  180        for(j = 2; j < COLUMN; j++)
//  181        {
//  182          VideoImage1[i][j-2] = VideoImage1[i][j];
??Initial_3:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        MOVS     R2,#+161
        LDR.W    R3,??DataTable6_3
        MLA      R2,R2,R0,R3
        ADDS     R2,R1,R2
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        MOVS     R3,#+161
        LDR.W    R4,??DataTable6_3
        MLA      R3,R3,R0,R4
        LDRB     R3,[R1, R3]
        STRB     R3,[R2, #-2]
//  183        }
        ADDS     R1,R1,#+1
??Initial_4:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+161
        BLT.N    ??Initial_3
        ADDS     R0,R0,#+1
??Initial_2:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+42
        BGE.N    ??Initial_5
        MOVS     R1,#+2
        B.N      ??Initial_4
//  184      }
//  185 }
??Initial_5:
        POP      {R4}
        BX       LR               ;; return
//  186 
//  187 //--------------------�͹��������������ʼ��-----------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  188 void LPTMR_Init()   //PTC5  LPT0_ALT2
//  189 {
//  190    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //����C�˿�ʱ��
LPTMR_Init:
        LDR.W    R0,??DataTable6_4  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.W    R1,??DataTable6_4  ;; 0x40048038
        STR      R0,[R1, #+0]
//  191    PORTC_PCR5 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_5  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_5  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  192    PORTC_PCR5 |= PORT_PCR_MUX(4);  //PTC5����ΪLPTMRģʽ
        LDR.W    R0,??DataTable6_5  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x400
        LDR.W    R1,??DataTable6_5  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  193    PORTC_PCR5 |= PORT_PCR_PE_MASK; //
        LDR.W    R0,??DataTable6_5  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable6_5  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  194    PORTC_PCR5 &= ~PORT_PCR_PS_MASK; //����
        LDR.W    R0,??DataTable6_5  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable6_5  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  195 
//  196    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;  //ʹ��LPTMģ��ʱ��
        LDR.W    R0,??DataTable6_4  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable6_4  ;; 0x40048038
        STR      R0,[R1, #+0]
//  197    LPTMR0_CSR &= ~LPTMR_CSR_TPS_MASK;
        LDR.W    R0,??DataTable6_6  ;; 0x40040000
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x30
        LDR.W    R1,??DataTable6_6  ;; 0x40040000
        STR      R0,[R1, #+0]
//  198    LPTMR0_CSR |= LPTMR_CSR_TPS(2)| LPTMR_CSR_TMS_MASK; //  ALT2  ����ģʽ
        LDR.W    R0,??DataTable6_6  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x22
        LDR.W    R1,??DataTable6_6  ;; 0x40040000
        STR      R0,[R1, #+0]
//  199    LPTMR0_CSR |= LPTMR_CSR_TFC_MASK;  //�����λ 65535
        LDR.W    R0,??DataTable6_6  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable6_6  ;; 0x40040000
        STR      R0,[R1, #+0]
//  200    LPTMR0_CSR &= ~LPTMR_CSR_TPP_MASK;  //�����ؼ���
        LDR.W    R0,??DataTable6_6  ;; 0x40040000
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x8
        LDR.W    R1,??DataTable6_6  ;; 0x40040000
        STR      R0,[R1, #+0]
//  201 
//  202    LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK; //  ���Է�Ƶ���˲�
        LDR.W    R0,??DataTable6_7  ;; 0x40040004
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable6_7  ;; 0x40040004
        STR      R0,[R1, #+0]
//  203    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;  //����LPTģ��
        LDR.W    R0,??DataTable6_6  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable6_6  ;; 0x40040000
        STR      R0,[R1, #+0]
//  204 }
        BX       LR               ;; return
//  205 
//  206 
//  207 //---------------------------���жϲ�׽�˿ڳ�ʼ��-------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  208 void EXIT_Init(void)
//  209 {
//  210     SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  //����C�˿�ʱ��
EXIT_Init:
        LDR.W    R0,??DataTable6_4  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.W    R1,??DataTable6_4  ;; 0x40048038
        STR      R0,[R1, #+0]
//  211     PORTC_PCR3 =PORT_PCR_MUX(1);  //GPIO
        LDR.W    R0,??DataTable6_8  ;; 0x4004b00c
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  212     GPIOC_PDDR &= ~GPIO_PIN(3);   //����
        LDR.W    R0,??DataTable6_9  ;; 0x400ff094
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x8
        LDR.W    R1,??DataTable6_9  ;; 0x400ff094
        STR      R0,[R1, #+0]
//  213     PORTC_PCR3 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; //��������;
        LDR.W    R0,??DataTable6_8  ;; 0x4004b00c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3
        LDR.W    R1,??DataTable6_8  ;; 0x4004b00c
        STR      R0,[R1, #+0]
//  214     PORTC_PCR3 |= PORT_PCR_IRQC(9); //9Ϊ�����ش����ⲿ�ж� 10Ϊ�½��ش�
        LDR.W    R0,??DataTable6_8  ;; 0x4004b00c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x90000
        LDR.W    R1,??DataTable6_8  ;; 0x4004b00c
        STR      R0,[R1, #+0]
//  215 }
        BX       LR               ;; return
//  216 
//  217 
//  218 
//  219 //----------------------------���ڳ�ʼ��-----------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  220 void UART0_Init(void)    //PTB16 RXD    PTB17 TXD
//  221 {
UART0_Init:
        PUSH     {R4}
//  222     uint32 uartclk_khz = CORE_CLOCK*10 * BUS_CLOCK;//ʱ��180MHz    //��ʱ����
        LDR.W    R0,??DataTable6_10  ;; 0x2bf20
//  223     uint32 baud = BAUD;
        MOV      R1,#+19200
//  224     uint16 sbr,brfa;
//  225     
//  226     SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //����B��ʱ��
        LDR.W    R2,??DataTable6_4  ;; 0x40048038
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x400
        LDR.W    R3,??DataTable6_4  ;; 0x40048038
        STR      R2,[R3, #+0]
//  227     PORTB_PCR16|=PORT_PCR_MUX(3);//��PTB16��������Ϊģʽ3����UART0_RX
        LDR.W    R2,??DataTable6_11  ;; 0x4004a040
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.W    R3,??DataTable6_11  ;; 0x4004a040
        STR      R2,[R3, #+0]
//  228     PORTB_PCR17|=PORT_PCR_MUX(3);//��PTB177��������Ϊģʽ3����UART0_TX
        LDR.W    R2,??DataTable6_12  ;; 0x4004a044
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.W    R3,??DataTable6_12  ;; 0x4004a044
        STR      R2,[R3, #+0]
//  229     SIM_SCGC4|=SIM_SCGC4_UART0_MASK;//����UART0ʱ��
        LDR.W    R2,??DataTable6_13  ;; 0x40048034
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x400
        LDR.W    R3,??DataTable6_13  ;; 0x40048034
        STR      R2,[R3, #+0]
//  230     sbr = (uint16)((uartclk_khz*1000)/(baud*16));//���㲢���ò�����
        MOV      R2,#+1000
        MUL      R2,R2,R0
        LSLS     R3,R1,#+4
        UDIV     R2,R2,R3
//  231     
//  232     UART0_BDH = (uint8)((sbr&0x1F00)>>8);//��������19200д����Ӧ�ļĴ���Ȼ�����ʹ�ܣ�ʹ�乤����ǰ���buadֻ��һ�����֣�������ļ����ǽ�19200д������Ĵ�����Ȼ�����ʹ��
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        ASRS     R3,R2,#+8
        ANDS     R3,R3,#0x1F
        LDR.W    R4,??DataTable6_14  ;; 0x4006a000
        STRB     R3,[R4, #+0]
//  233     UART0_BDL=(uint8)(sbr&0x00FF);
        LDR.W    R3,??DataTable6_15  ;; 0x4006a001
        STRB     R2,[R3, #+0]
//  234     brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
        MOV      R3,#+32000
        MULS     R0,R3,R0
        LSLS     R1,R1,#+4
        UDIV     R0,R0,R1
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        SUBS     R0,R0,R2, LSL #+5
//  235     UART0_C4 = (uint8)(brfa & 0x001F);
        ANDS     R0,R0,#0x1F
        LDR.W    R1,??DataTable6_16  ;; 0x4006a00a
        STRB     R0,[R1, #+0]
//  236     UART0_C2 |=(UART_C2_TE_MASK|UART_C2_RE_MASK);
        LDR.W    R0,??DataTable6_17  ;; 0x4006a003
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0xC
        LDR.W    R1,??DataTable6_17  ;; 0x4006a003
        STRB     R0,[R1, #+0]
//  237     UART0_C1 = 0;	
        LDR.W    R0,??DataTable6_18  ;; 0x4006a002
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  238     UART0_C2 |= UART_C2_RIE_MASK;   //��UART0�����ж�
        LDR.W    R0,??DataTable6_17  ;; 0x4006a003
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable6_17  ;; 0x4006a003
        STRB     R0,[R1, #+0]
//  239 }
        POP      {R4}
        BX       LR               ;; return
//  240 
//  241 
//  242 //-------------------------------------ftm��ʼ��-----------------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  243 void hw_FTM_init(void)
//  244 {      	
//  245     SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;//����C�˿�ʱ��
hw_FTM_init:
        LDR.W    R0,??DataTable6_4  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.W    R1,??DataTable6_4  ;; 0x40048038
        STR      R0,[R1, #+0]
//  246     SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //����A�˿�ʱ��
        LDR.W    R0,??DataTable6_4  ;; 0x40048038
        LDR      R0,[R0, #+0]
        MOV      R1,#+512
        ORRS     R0,R1,R0
        LDR.W    R1,??DataTable6_4  ;; 0x40048038
        STR      R0,[R1, #+0]
//  247     
//  248     PORTC_PCR4 &= ~PORT_PCR_MUX_MASK; //����
        LDR.W    R0,??DataTable6_19  ;; 0x4004b010
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_19  ;; 0x4004b010
        STR      R0,[R1, #+0]
//  249     PORTA_PCR12 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_20  ;; 0x40049030
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_20  ;; 0x40049030
        STR      R0,[R1, #+0]
//  250     PORTA_PCR13 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_21  ;; 0x40049034
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_21  ;; 0x40049034
        STR      R0,[R1, #+0]
//  251     PORTA_PCR10 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_22  ;; 0x40049028
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_22  ;; 0x40049028
        STR      R0,[R1, #+0]
//  252     
//  253     PORTC_PCR4 = PORT_PCR_MUX(4); //FTM is alt4 function for this pin
        LDR.W    R0,??DataTable6_19  ;; 0x4004b010
        MOV      R1,#+1024
        STR      R1,[R0, #+0]
//  254     PORTA_PCR10 = PORT_PCR_MUX(3);
        LDR.W    R0,??DataTable6_22  ;; 0x40049028
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  255     PORTA_PCR12 = PORT_PCR_MUX(3);//FTM is alt3 function for this pin 
        LDR.W    R0,??DataTable6_20  ;; 0x40049030
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  256     PORTA_PCR13 = PORT_PCR_MUX(3);
        LDR.W    R0,??DataTable6_21  ;; 0x40049034
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  257   
//  258     SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;     //ʹ��FTM0ʱ��
        LDR.W    R0,??DataTable6_23  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable6_23  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  259     SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;    //����FTM1ģ��ʱ��
        LDR.W    R0,??DataTable6_23  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2000000
        LDR.W    R1,??DataTable6_23  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  260     SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;    //����FTM2ģ��ʱ��
        LDR.W    R0,??DataTable6_24  ;; 0x40048030
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable6_24  ;; 0x40048030
        STR      R0,[R1, #+0]
//  261     
//  262     FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����FTM0_CH3 
        LDR.W    R0,??DataTable6_25  ;; 0x40038024
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_25  ;; 0x40038024
        STR      R0,[R1, #+0]
//  263     FTM1_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����ģʽ CH0
        LDR.W    R0,??DataTable6_26  ;; 0x4003900c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_26  ;; 0x4003900c
        STR      R0,[R1, #+0]
//  264     FTM1_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //FTM1_CH1
        LDR.W    R0,??DataTable6_27  ;; 0x40039014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_27  ;; 0x40039014
        STR      R0,[R1, #+0]
//  265     FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����FTM2_CH0 
        LDR.W    R0,??DataTable6_28  ;; 0x400b800c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_28  ;; 0x400b800c
        STR      R0,[R1, #+0]
//  266     
//  267     FTM0_CNT=0;//���ü�����ֵΪ0
        LDR.W    R0,??DataTable6_29  ;; 0x40038004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  268     FTM1_CNT=0;
        LDR.W    R0,??DataTable6_30  ;; 0x40039004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  269     FTM2_CNT=0;
        LDR.W    R0,??DataTable6_31  ;; 0x400b8004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  270     
//  271     //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 
//  272 
//  273     //���� the pulse width(duty cycle) is determined by (CnV - CNTIN).
//  274     
//  275      FTM1_MOD =1000;              //����PWMƵ��Ϊ10K=100 000 000 /2^2/2500  ���100 000 000 �ǵ��߽��ϵͳƵ��  2500�������õ�FTM1_MOD ֵ
        LDR.W    R0,??DataTable6_32  ;; 0x40039008
        MOV      R1,#+1000
        STR      R1,[R0, #+0]
//  276      FTM0_MOD =3115; //300hz                      //18750  50hz
        LDR.W    R0,??DataTable6_33  ;; 0x40038008
        MOVW     R1,#+3115
        STR      R1,[R0, #+0]
//  277      FTM2_MOD =2; 
        LDR.W    R0,??DataTable6_34  ;; 0x400b8008
        MOVS     R1,#+2
        STR      R1,[R0, #+0]
//  278     
//  279     FTM0_CNTIN=0;//���ó�ʼ������ֵ
        LDR.W    R0,??DataTable6_35  ;; 0x4003804c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  280     FTM1_CNTIN=0;
        LDR.W    R0,??DataTable6_36  ;; 0x4003904c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  281     FTM2_CNTIN=0;
        LDR.W    R0,??DataTable6_37  ;; 0x400b804c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  282       
//  283     FTM0_C3V=mid_angle;//1400;//1490 1.5ms//2ms  1986//2.5ms  2483 //1ms 993// 0.5ms  497  //1614
        LDR.W    R0,??DataTable6_38  ;; 0x40038028
        LDR.W    R1,??DataTable6_39
        LDRH     R1,[R1, #+0]
        STR      R1,[R0, #+0]
//  284 
//  285     FTM1_C0V=0;
        LDR.W    R0,??DataTable6_40  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  286     FTM1_C1V=0;
        LDR.W    R0,??DataTable6_41  ;; 0x40039018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  287     
//  288     FTM2_C0V=1;
        LDR.W    R0,??DataTable6_42  ;; 0x400b8010
        MOVS     R1,#+1
        STR      R1,[R0, #+0]
//  289     
//  290     FTM0_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(6);
        LDR.W    R0,??DataTable6_43  ;; 0x40038000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0xE
        LDR.W    R1,??DataTable6_43  ;; 0x40038000
        STR      R0,[R1, #+0]
//  291     FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(2); //����ʱ�Ӻͷ�Ƶ
        LDR.W    R0,??DataTable6_44  ;; 0x40039000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0xA
        LDR.W    R1,??DataTable6_44  ;; 0x40039000
        STR      R0,[R1, #+0]
//  292     FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(2);
        LDR.W    R0,??DataTable6_45  ;; 0x400b8000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0xA
        LDR.W    R1,??DataTable6_45  ;; 0x400b8000
        STR      R0,[R1, #+0]
//  293 }
        BX       LR               ;; return
//  294 
//  295 //----------------------���໷Ƶ��Ϊ50/15*54=180M���Ժ���-------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  296 void pllinit180M(void)
//  297 {
//  298 	uint32_t temp_reg;
//  299         //ʹ��IO�˿�ʱ��    
//  300     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
//  301                               | SIM_SCGC5_PORTB_MASK
//  302                               | SIM_SCGC5_PORTC_MASK
//  303                               | SIM_SCGC5_PORTD_MASK
//  304                               | SIM_SCGC5_PORTE_MASK );
pllinit180M:
        LDR.W    R0,??DataTable6_4  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3E00
        LDR.W    R1,??DataTable6_4  ;; 0x40048038
        STR      R0,[R1, #+0]
//  305     //���ﴦ��Ĭ�ϵ�FEIģʽ
//  306     //�����ƶ���FBEģʽ
//  307     MCG_C2 = 0;  
        LDR.W    R0,??DataTable6_46  ;; 0x40064001
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  308     //MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
//  309     //��ʼ��������ͷ�����״̬��������GPIO
//  310     SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
        LDR.W    R0,??DataTable6_13  ;; 0x40048034
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x10000000
        LDR.W    R1,??DataTable6_13  ;; 0x40048034
        STR      R0,[R1, #+0]
//  311     LLWU_CS |= LLWU_CS_ACKISO_MASK;
        LDR.W    R0,??DataTable6_47  ;; 0x4007c008
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x80
        LDR.W    R1,??DataTable6_47  ;; 0x4007c008
        STRB     R0,[R1, #+0]
//  312     
//  313     //ѡ���ⲿ���񣬲ο���Ƶ������IREFS�������ⲿ����
//  314     //011 If RANGE = 0, Divide Factor is 8; for all other RANGE values, Divide Factor is 256.
//  315     MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
        LDR.W    R0,??DataTable6_48  ;; 0x40064000
        MOVS     R1,#+152
        STRB     R1,[R0, #+0]
//  316     
//  317     //�ȴ������ȶ�	    
//  318     //while (!(MCG_S & MCG_S_OSCINIT_MASK)){}              //�ȴ����໷��ʼ������
//  319     while (MCG_S & MCG_S_IREFST_MASK){}                  //�ȴ�ʱ���л����ⲿ�ο�ʱ��
??pllinit180M_0:
        LDR.W    R0,??DataTable6_49  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+27
        BMI.N    ??pllinit180M_0
//  320     while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}
??pllinit180M_1:
        LDR.W    R0,??DataTable6_49  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        UBFX     R0,R0,#+2,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+2
        BNE.N    ??pllinit180M_1
//  321     
//  322     //����FBEģʽ,
//  323     //0x18==25��Ƶ=2M,
//  324     //0x08==15��Ƶ=3.333M 
//  325     //0x09==16��Ƶ=3.125M,
//  326     //0x10==17��Ƶ=2.94M 
//  327     //0x11==18��Ƶ=2.7778M 
//  328     //0x12==19��Ƶ=2.63M,
//  329     //0x13==20��Ƶ=2.5M    
//  330     MCG_C5 = MCG_C5_PRDIV(0x0e);                
        LDR.W    R0,??DataTable6_50  ;; 0x40064004
        MOVS     R1,#+14
        STRB     R1,[R0, #+0]
//  331     
//  332     //ȷ��MCG_C6���ڸ�λ״̬����ֹLOLIE��PLL����ʱ�ӿ���������PLL VCO��Ƶ��
//  333     MCG_C6 = 0x0;
        LDR.W    R0,??DataTable6_51  ;; 0x40064005
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  334     
//  335     //����FMC_PFAPR��ǰ��ֵ
//  336     temp_reg = FMC_PFAPR;
        LDR.W    R0,??DataTable6_52  ;; 0x4001f000
        LDR      R0,[R0, #+0]
//  337     
//  338     //ͨ��M&PFD��λM0PFD����ֹԤȡ����
//  339     FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
//  340                      | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
//  341                      | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;    
        LDR.W    R1,??DataTable6_52  ;; 0x4001f000
        LDR      R1,[R1, #+0]
        ORRS     R1,R1,#0xFF0000
        LDR.W    R2,??DataTable6_52  ;; 0x4001f000
        STR      R1,[R2, #+0]
//  342     ///����ϵͳ��Ƶ��
//  343     //MCG=PLL, core = MCG, bus = MCG/3, FlexBus = MCG/3, Flash clock= MCG/8
//  344     SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) 
//  345                  | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);       
        LDR.W    R1,??DataTable6_53  ;; 0x40048044
        LDR.W    R2,??DataTable6_54  ;; 0x2270000
        STR      R2,[R1, #+0]
//  346     
//  347     //���´�FMC_PFAPR��ԭʼֵ
//  348     FMC_PFAPR = temp_reg; 
        LDR.W    R1,??DataTable6_52  ;; 0x4001f000
        STR      R0,[R1, #+0]
//  349     
//  350     //����VCO��Ƶ����ʹ��PLLΪ100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
//  351     MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(30);  //VDIV = 31 (x54)
        LDR.W    R0,??DataTable6_51  ;; 0x40064005
        MOVS     R1,#+94
        STRB     R1,[R0, #+0]
//  352                                                   //VDIV = 26 (x50)
//  353     while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set    
??pllinit180M_2:
        LDR.W    R0,??DataTable6_49  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+26
        BPL.N    ??pllinit180M_2
//  354     while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set    
??pllinit180M_3:
        LDR.W    R0,??DataTable6_49  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+25
        BPL.N    ??pllinit180M_3
//  355     
//  356     //����PBEģʽ    
//  357     //ͨ������CLKSλ������PEEģʽ
//  358     // CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
//  359     MCG_C1 &= ~MCG_C1_CLKS_MASK;
        LDR.W    R0,??DataTable6_48  ;; 0x40064000
        LDRB     R0,[R0, #+0]
        ANDS     R0,R0,#0x3F
        LDR.W    R1,??DataTable6_48  ;; 0x40064000
        STRB     R0,[R1, #+0]
//  360     
//  361     //�ȴ�ʱ��״̬λ����
//  362     while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};
??pllinit180M_4:
        LDR.W    R0,??DataTable6_49  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        UBFX     R0,R0,#+2,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+3
        BNE.N    ??pllinit180M_4
//  363     //SIM_CLKDIV2 |= SIM_CLKDIV2_USBDIV(1);  
//  364     
//  365     //���ø���ʱ��Ϊ�ں�ʱ��
//  366     SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;	
        LDR.W    R0,??DataTable6_55  ;; 0x40048004
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000
        LDR.W    R1,??DataTable6_55  ;; 0x40048004
        STR      R0,[R1, #+0]
//  367     //��PTA6������ʹ��TRACE_CLKOU����
//  368     PORTA_PCR6 = ( PORT_PCR_MUX(0x7));  
        LDR.W    R0,??DataTable6_56  ;; 0x40049018
        MOV      R1,#+1792
        STR      R1,[R0, #+0]
//  369     //ʹ��FlexBusģ��ʱ��
//  370     SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
        LDR.W    R0,??DataTable6_57  ;; 0x40048040
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable6_57  ;; 0x40048040
        STR      R0,[R1, #+0]
//  371     //��PTA6������ʹ��FB_CLKOUT����
//  372     PORTC_PCR3 = ( PORT_PCR_MUX(0x5));
        LDR.N    R0,??DataTable6_8  ;; 0x4004b00c
        MOV      R1,#+1280
        STR      R1,[R0, #+0]
//  373 }
        BX       LR               ;; return
//  374 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  375 int f_abs(int x)
//  376 {
//  377   if(x<0)
f_abs:
        CMP      R0,#+0
        BPL.N    ??f_abs_0
//  378     x=-x;
        RSBS     R0,R0,#+0
        B.N      ??f_abs_1
//  379   else
//  380     x=x;
//  381   return x;
??f_abs_0:
??f_abs_1:
        BX       LR               ;; return
//  382 }
//  383 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  384 int f_abs16(int16 x)
//  385 {
//  386   if(x<0)
f_abs16:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+0
        BPL.N    ??f_abs16_0
//  387     x=-x;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        RSBS     R0,R0,#+0
        B.N      ??f_abs16_1
//  388   else
//  389     x=x;
//  390   return x;
??f_abs16_0:
??f_abs16_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BX       LR               ;; return
//  391 }
//  392 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  393 int f_abs32(int32 x)
//  394 {
//  395   if(x<0)
f_abs32:
        CMP      R0,#+0
        BPL.N    ??f_abs32_0
//  396     x=-x;
        RSBS     R0,R0,#+0
        B.N      ??f_abs32_1
//  397   else
//  398     x=x;
//  399   return x;
??f_abs32_0:
??f_abs32_1:
        BX       LR               ;; return
//  400 }
//  401 
//  402 //-----------------------------------ɨ����߻�׼��---------------------------------//
//  403 /*
//  404 1�����������Ŀ����ͼ������ռ�ı����ϴ󣬹ʿ���Ϊ���е��λ�ã�81������һ�����������У�
//  405 �����ǳ����ܳ���������������Ҫ��������ƫ�������������
//  406 
//  407 ���õ���ȫ�ֱ���������
//  408 bottom_whitebase
//  409 */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  410 void Search_WhiteBase(void)   //��ͼ��ײ��м俪ʼ������ɨ����߻�׼
//  411 {
Search_WhiteBase:
        PUSH     {R4-R6,LR}
//  412   int i = 0,j = 0;//����ʮ��λ���з��ű���   i�����б���  j�����б���
        MOVS     R4,#+0
        MOVS     R5,#+0
//  413   uint16 base_sum = 0;
        MOVS     R6,#+0
//  414   
//  415   bottom_whitebase = 0;//��׼�и���ֵ 
        LDR.W    R0,??DataTable7
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  416   find_whitebase_flag = 0;  //���ֻ�׼�еı�־λ
        LDR.W    R0,??DataTable7_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  417   whitebase_line_cnt = 0;
        LDR.W    R0,??DataTable7_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  418   white_refer = 0;
        LDR.W    R0,??DataTable7_3
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  419 //////////////////////////////�˲���ʼ///////////////////////////////////  
//  420 //���ȶ�����ͼ������˲������õķ�������ֵ�˲�
//  421   for(i = 0;i < ROW;i++)                 //ROWΪ61COLUMNΪ163
        MOVS     R0,#+0
        MOVS     R4,R0
        B.N      ??Search_WhiteBase_0
??Search_WhiteBase_1:
        ADDS     R4,R4,#+1
??Search_WhiteBase_0:
        CMP      R4,#+65
        BGE.N    ??Search_WhiteBase_2
//  422     for(j = 0;j< COLUMN;j++)
        MOVS     R5,#+0
        B.N      ??Search_WhiteBase_3
//  423     {
//  424       if((j == 0||j == COLUMN-1)&&i >= 1 && i <= ROW-2 &&i >=1)
//  425       {
//  426         base_sum = VideoImage1[i-1][j] + VideoImage1[i+1][j];
//  427         base_sum = base_sum>>1;
//  428         if( f_abs16(base_sum-VideoImage1[i][j]) > OT)
//  429            VideoImage1[i][j] = base_sum;
//  430       }
//  431       else if(j >=1 && j <= COLUMN-2)
??Search_WhiteBase_4:
        SUBS     R0,R5,#+1
        CMP      R0,#+159
        BCS.N    ??Search_WhiteBase_5
//  432       {
//  433         base_sum = VideoImage1[i][j-1] + VideoImage1[i][j+1];
        MOVS     R0,#+161
        LDR.N    R1,??DataTable6_3
        MLA      R0,R0,R4,R1
        ADDS     R0,R5,R0
        LDRB     R0,[R0, #-1]
        MOVS     R1,#+161
        LDR.N    R2,??DataTable6_3
        MLA      R1,R1,R4,R2
        ADDS     R1,R5,R1
        LDRB     R1,[R1, #+1]
        UXTAB    R6,R1,R0
//  434         base_sum = base_sum>>1;
        UXTH     R6,R6            ;; ZeroExt  R6,R6,#+16,#+16
        LSRS     R6,R6,#+1
//  435         if( f_abs16( base_sum-VideoImage1[i][j]) > OT)
        MOVS     R0,#+161
        LDR.N    R1,??DataTable6_3
        MLA      R0,R0,R4,R1
        LDRB     R0,[R5, R0]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable7_4
        LDR      R1,[R1, #+0]
        CMP      R1,R0
        BGE.N    ??Search_WhiteBase_5
//  436            VideoImage1[i][j] = base_sum;
        MOVS     R0,#+161
        LDR.N    R1,??DataTable6_3
        MLA      R0,R0,R4,R1
        STRB     R6,[R5, R0]
//  437       }
??Search_WhiteBase_5:
        ADDS     R5,R5,#+1
??Search_WhiteBase_3:
        CMP      R5,#+161
        BGE.N    ??Search_WhiteBase_1
        CMP      R5,#+0
        BEQ.N    ??Search_WhiteBase_6
        CMP      R5,#+160
        BNE.N    ??Search_WhiteBase_4
??Search_WhiteBase_6:
        SUBS     R0,R4,#+1
        CMP      R0,#+63
        BCS.N    ??Search_WhiteBase_4
        CMP      R4,#+1
        BLT.N    ??Search_WhiteBase_4
        MOVS     R0,#+161
        LDR.N    R1,??DataTable6_3
        MLA      R0,R0,R4,R1
        ADDS     R0,R5,R0
        LDRB     R0,[R0, #-161]
        MOVS     R1,#+161
        LDR.N    R2,??DataTable6_3
        MLA      R1,R1,R4,R2
        ADDS     R1,R5,R1
        LDRB     R1,[R1, #+161]
        UXTAB    R6,R1,R0
        UXTH     R6,R6            ;; ZeroExt  R6,R6,#+16,#+16
        LSRS     R6,R6,#+1
        MOVS     R0,#+161
        LDR.N    R1,??DataTable6_3
        MLA      R0,R0,R4,R1
        LDRB     R0,[R5, R0]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable7_4
        LDR      R1,[R1, #+0]
        CMP      R1,R0
        BGE.N    ??Search_WhiteBase_5
        MOVS     R0,#+161
        LDR.N    R1,??DataTable6_3
        MLA      R0,R0,R4,R1
        STRB     R6,[R5, R0]
        B.N      ??Search_WhiteBase_5
//  438     }
//  439 ////////////////////////////�˲�����////////////////////////////////
//  440   
//  441   //////////////////////Ѱ�һ�׼�е����ҵı��ص�///////////////////////
//  442   for(;;) 
//  443   {
//  444       //ÿ�еĴ�������
//  445       whitepoint_start = 0;   
??Search_WhiteBase_2:
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  446       whitepoint_end = 0; 
        LDR.W    R0,??DataTable7_6
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  447       current_deal_line = bottom_whitebase;
        LDR.W    R0,??DataTable7_7
        LDR.W    R1,??DataTable7
        LDR      R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  448      //�������������������
//  449       j=MID;                //MIDΪ81
        MOVS     R5,#+80
        B.N      ??Search_WhiteBase_7
//  450       while( VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] < OT && j >= 2)//һ��Ϊ��ʹ�������ظ��ӵ����ԣ����ø����ж�
//  451       {
//  452         j--;
??Search_WhiteBase_8:
        SUBS     R5,R5,#+1
//  453       }//����������whileѭ������
??Search_WhiteBase_7:
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        MOVS     R1,#+161
        LDR.N    R2,??DataTable6_3
        MLA      R0,R1,R0,R2
        LDRB     R0,[R5, R0]
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+161
        LDR.N    R3,??DataTable6_3
        MLA      R1,R2,R1,R3
        ADDS     R1,R5,R1
        LDRB     R1,[R1, #-2]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable7_4
        LDR      R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_9
        CMP      R5,#+2
        BGE.N    ??Search_WhiteBase_8
//  454       if(j <= 1)
??Search_WhiteBase_9:
        CMP      R5,#+2
        BGE.N    ??Search_WhiteBase_10
//  455       {
//  456         j = LEFT_OUT_LOST;       //��0��ʾ���ǳ���Ķ��� LEFT_OUT_LOST Ϊ0
        MOVS     R5,#+0
//  457       }
//  458       whitepoint_start = j;//���ҵı�����ѹ�ڷֽ����İ�ɫ����  
??Search_WhiteBase_10:
        LDR.W    R0,??DataTable7_5
        STRB     R5,[R0, #+0]
//  459       
//  460       //���ұ���������������
//  461        j=MID;                //MIDΪ81
        MOVS     R5,#+80
        B.N      ??Search_WhiteBase_11
//  462       while( VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+2] < OT && j<=COLUMN-3)//һ��Ϊ��ʹ�������ظ��ӵ����ԣ����ø����ж�
//  463       {
//  464         j++;
??Search_WhiteBase_12:
        ADDS     R5,R5,#+1
//  465       }//����������whileѭ������
??Search_WhiteBase_11:
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        MOVS     R1,#+161
        LDR.N    R2,??DataTable6_3
        MLA      R0,R1,R0,R2
        LDRB     R0,[R5, R0]
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+161
        LDR.N    R3,??DataTable6_3
        MLA      R1,R2,R1,R3
        ADDS     R1,R5,R1
        LDRB     R1,[R1, #+2]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable7_4
        LDR      R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_13
        CMP      R5,#+159
        BLT.N    ??Search_WhiteBase_12
//  466       if(COLUMN-2 <= j)
??Search_WhiteBase_13:
        CMP      R5,#+159
        BLT.N    ??Search_WhiteBase_14
//  467       {
//  468         j = RIGHT_OUT_LOST;       //��0��ʾ���ǳ���Ķ��� RIGHT_OUT_LOST ΪCOLUMN-1
        MOVS     R5,#+160
//  469       }
//  470       whitepoint_end = j;//���ҵı�����ѹ�ڷֽ����İ�ɫ����  
??Search_WhiteBase_14:
        LDR.W    R0,??DataTable7_6
        STRB     R5,[R0, #+0]
//  471       
//  472         center_white[current_deal_line] = (whitepoint_start + whitepoint_end) / 2;  //��¼���ĵ�,����MID˵������ƫ�󣬷�֮��˵������ƫ��
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable7_6
        LDRB     R1,[R1, #+0]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_2
        STRH     R0,[R2, R1, LSL #+1]
//  473         left_black[current_deal_line] = whitepoint_start;   //��¼�����λ�� (��Ϊ0���ܿ���˵����ߺ��߶�ʧ��������ƫ��)
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6
        LDR.W    R2,??DataTable7_5
        LDRB     R2,[R2, #+0]
        STRH     R2,[R1, R0, LSL #+1]
//  474         right_black[current_deal_line] = whitepoint_end;    //��¼�Һ���λ��(��ΪCOLUMN���ܿ���˵���ұߺ��߶�ʧ,������ƫ��)
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_1
        LDR.W    R2,??DataTable7_6
        LDRB     R2,[R2, #+0]
        STRH     R2,[R1, R0, LSL #+1]
//  475       
//  476       if(whitepoint_end - whitepoint_start > MIN_WHITEBASE_POINT && (whitepoint_end-whitepoint_start) < refer_road_width[current_deal_line] + 15)//�������صķ�����ǰ��Ϊȫ�ں�ȫ�׵�������޷������ҵ�������
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable7_5
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        CMP      R0,#+21
        BLT.N    ??Search_WhiteBase_15
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable7_5
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        LDR.W    R2,??DataTable7_8
        LDRH     R1,[R2, R1, LSL #+1]
        ADDS     R1,R1,#+15
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_15
//  477       {//
//  478         whitebase_line_cnt++;
        LDR.W    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable7_2
        STRB     R0,[R1, #+0]
//  479         bottom_whitebase++;
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable7
        STR      R0,[R1, #+0]
//  480         
//  481 
//  482       }
//  483       else
//  484       {
//  485         whitebase_line_cnt = 0;//���︳ֵΪ0,��Ϊ�˱����������Ļ�׼��Ϊ����������
//  486         bottom_whitebase++;
//  487         if(bottom_whitebase >= ROW-1)
//  488         {
//  489           bottom_whitebase = 0;
//  490           break;//������ȫ������������ʱ�򣬽�������һ���Ĳ���
//  491         }
//  492       }
//  493       if (whitebase_line_cnt >= MIN_WHITE_LINE && f_abs(center_white[bottom_whitebase - 1] - center_white[bottom_whitebase - 2]) < 20 
//  494           && f_abs(center_white[bottom_whitebase - 2] - center_white[bottom_whitebase - 3]) < 20)  //�������дﵽ��׼,�����������ɨ��
??Search_WhiteBase_16:
        LDR.W    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+3
        BCC.N    ??Search_WhiteBase_17
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        LDR.N    R1,??DataTable6_2
        ADDS     R0,R1,R0, LSL #+1
        LDRSH    R0,[R0, #-2]
        LDR.W    R1,??DataTable7
        LDR      R1,[R1, #+0]
        LDR.N    R2,??DataTable6_2
        ADDS     R1,R2,R1, LSL #+1
        LDRSH    R1,[R1, #-4]
        SUBS     R0,R0,R1
        BL       f_abs
        CMP      R0,#+20
        BGE.N    ??Search_WhiteBase_17
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        LDR.N    R1,??DataTable6_2
        ADDS     R0,R1,R0, LSL #+1
        LDRSH    R0,[R0, #-4]
        LDR.W    R1,??DataTable7
        LDR      R1,[R1, #+0]
        LDR.N    R2,??DataTable6_2
        ADDS     R1,R2,R1, LSL #+1
        LDRSH    R1,[R1, #-6]
        SUBS     R0,R0,R1
        BL       f_abs
        CMP      R0,#+20
        BGE.N    ??Search_WhiteBase_17
//  495       {
//  496         find_whitebase_flag = 1;//���ֻ�׼��
        LDR.W    R0,??DataTable7_1
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
//  497         bottom_whitebase = bottom_whitebase - MIN_WHITE_LINE;  
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable7
        STR      R0,[R1, #+0]
//  498       }
//  499       
//  500       if(find_whitebase_flag == 1)
??Search_WhiteBase_17:
        LDR.W    R0,??DataTable7_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??Search_WhiteBase_2
//  501       {    
//  502        for(i = bottom_whitebase; i < bottom_whitebase+MIN_WHITE_LINE; i++)
        LDR.W    R0,??DataTable7
        LDR      R4,[R0, #+0]
        B.N      ??Search_WhiteBase_18
??Search_WhiteBase_15:
        LDR.W    R0,??DataTable7_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable7
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        CMP      R0,#+64
        BLT.N    ??Search_WhiteBase_16
        LDR.W    R0,??DataTable7
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??Search_WhiteBase_19
//  503        {
//  504          white_refer += center_white[i];
??Search_WhiteBase_20:
        LDR.W    R0,??DataTable7_3
        LDR      R0,[R0, #+0]
        LDR.N    R1,??DataTable6_2
        LDRSH    R1,[R1, R4, LSL #+1]
        ADDS     R0,R0,R1
        LDR.W    R1,??DataTable7_3
        STR      R0,[R1, #+0]
//  505        }
        ADDS     R4,R4,#+1
??Search_WhiteBase_18:
        LDR.W    R0,??DataTable7
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+3
        CMP      R4,R0
        BLT.N    ??Search_WhiteBase_20
//  506          white_refer /= 3;       //�õ���׼�а������ĵ�white_refer
        LDR.W    R0,??DataTable7_3
        LDR      R0,[R0, #+0]
        MOVS     R1,#+3
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable7_3
        STR      R0,[R1, #+0]
//  507          break;//����������ѭ��
//  508       }
//  509   }//�ڶ���for����
//  510   
//  511 }//��������
??Search_WhiteBase_19:
        POP      {R4-R6,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_1:
        DC32     right_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_2:
        DC32     center_white

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_3:
        DC32     VideoImage1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_4:
        DC32     0x40048038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_5:
        DC32     0x4004b014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_6:
        DC32     0x40040000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_7:
        DC32     0x40040004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_8:
        DC32     0x4004b00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_9:
        DC32     0x400ff094

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_10:
        DC32     0x2bf20

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_11:
        DC32     0x4004a040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_12:
        DC32     0x4004a044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_13:
        DC32     0x40048034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_14:
        DC32     0x4006a000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_15:
        DC32     0x4006a001

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_16:
        DC32     0x4006a00a

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_17:
        DC32     0x4006a003

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_18:
        DC32     0x4006a002

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_19:
        DC32     0x4004b010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_20:
        DC32     0x40049030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_21:
        DC32     0x40049034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_22:
        DC32     0x40049028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_23:
        DC32     0x4004803c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_24:
        DC32     0x40048030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_25:
        DC32     0x40038024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_26:
        DC32     0x4003900c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_27:
        DC32     0x40039014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_28:
        DC32     0x400b800c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_29:
        DC32     0x40038004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_30:
        DC32     0x40039004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_31:
        DC32     0x400b8004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_32:
        DC32     0x40039008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_33:
        DC32     0x40038008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_34:
        DC32     0x400b8008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_35:
        DC32     0x4003804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_36:
        DC32     0x4003904c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_37:
        DC32     0x400b804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_38:
        DC32     0x40038028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_39:
        DC32     mid_angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_40:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_41:
        DC32     0x40039018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_42:
        DC32     0x400b8010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_43:
        DC32     0x40038000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_44:
        DC32     0x40039000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_45:
        DC32     0x400b8000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_46:
        DC32     0x40064001

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_47:
        DC32     0x4007c008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_48:
        DC32     0x40064000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_49:
        DC32     0x40064006

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_50:
        DC32     0x40064004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_51:
        DC32     0x40064005

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_52:
        DC32     0x4001f000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_53:
        DC32     0x40048044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_54:
        DC32     0x2270000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_55:
        DC32     0x40048004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_56:
        DC32     0x40049018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_57:
        DC32     0x40048040
//  512 
//  513 
//  514 //------------------------�ɻ�׼�߶��������ߺ���Ϊ��׼���ҳ�������Ե-----------------------// 
//  515 /*�趨ɨ��ķ�Χʮ����SEARCH_SCOPE 10
//  516 �������Ŀ��С��SEARCH_SCOPE��ʱ���ж�Ϊ�����Ч��
//  517 �������ߺ�ʮ�ֵ�·�Ĵ��������߳��ֶ��㣬�ҳ���ǰһ�������Ĵ��ں�һ�еĿ�Ⱥ�Ԥ��Ŀ�ȵ���Сֵ��
//  518 ����һ�е����ݶ�����Ȼ����û�ж������һ�е��е㣬�ֱ�������ɨ���ҵ���Ӧ�����ӵ�󣬸�����ǰ�����߷�ʽ
//  519 */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  520 void Search_BlackEdge(void)     
//  521 {  
Search_BlackEdge:
        PUSH     {R4-R8,LR}
//  522   int16 i = 0,j = 0,k = 0;
        MOVS     R6,#+0
        MOVS     R7,#+0
        MOVS     R4,#+0
//  523   int n=0;//����������Ͷϵ�������õ� 
        MOVS     R0,#+0
//  524   int amend_offset = 0;
        MOVS     R5,#+0
//  525   int first_amend_offset = 0;
        MOVS     R1,#+0
//  526   int second_amend_offset = 0;
        MOVS     R2,#+0
//  527   left_lost_start_line = 0;
        LDR.W    R3,??DataTable8
        MOVS     R12,#+0
        STRB     R12,[R3, #+0]
//  528   left_lost_end_line= 0 ;
        LDR.W    R3,??DataTable8_1
        MOVS     R12,#+0
        STRB     R12,[R3, #+0]
//  529   left_lost_flag = UN_LOST;         //��ȥ�����ߵı�־
        LDR.W    R3,??DataTable8_2
        MOVS     R12,#+0
        STRB     R12,[R3, #+0]
//  530   right_lost_start_line = 0;
        LDR.W    R3,??DataTable8_3
        MOVS     R12,#+0
        STRB     R12,[R3, #+0]
//  531   right_lost_end_line = 0;
        LDR.W    R3,??DataTable8_4
        MOVS     R12,#+0
        STRB     R12,[R3, #+0]
//  532   right_lost_flag = UN_LOST;
        LDR.W    R3,??DataTable8_5
        MOVS     R12,#+0
        STRB     R12,[R3, #+0]
//  533   top_whiteline = ROW-1 ;
        LDR.W    R3,??DataTable8_6
        MOVS     R12,#+64
        STR      R12,[R3, #+0]
//  534   left_top_whiteline = ROW-1 ;
        LDR.W    R3,??DataTable8_7
        MOVS     R12,#+64
        STR      R12,[R3, #+0]
//  535   right_top_whiteline = ROW-1;
        LDR.W    R3,??DataTable8_8
        MOVS     R12,#+64
        STR      R12,[R3, #+0]
//  536   for(i = bottom_whitebase + MIN_WHITE_LINE;i <= ROW-1;i++) 
        LDR.W    R3,??DataTable7
        LDR      R3,[R3, #+0]
        ADDS     R3,R3,#+3
        MOVS     R6,R3
        B.N      ??Search_BlackEdge_0
//  537   {
//  538     //��ÿһ����Ҫ�ҵ����ҵı߽磬�����Ƕ����������߽��зֿ����١�
//  539     //���ٵķ������ҵ�ǰһ�����ص㣬Ȼ�����ڻ���SEARCH_SCOPE���㣬
//  540     //Ȼ����������2*SEARCH_SCOPE���㣬�����������ˣ�����û��������������Ϊ�Ƕ�ʧ�ˡ�
//  541     //�����㶪����ͬһ�ߣ����G��ǰ������������ͼ���״̬�������ж�Ϊ��s�����
//  542    //���ǁG�߶�����ͼ������ߣ���ʱ�ж�Ϊ���˻�����ʮ�ֵ�·���ú󶪵�������ߵ��е����������ҵ�ͼ��
//  543     //������һ�߶����ˣ�����һ�ߺ�ԭ�ȵĲο���Ȳ���
//  544     
//  545     //ֻ�Ե�����صĶ��������
//  546      //������ߵĸ��� 
//  547    ///////////////�趨������Χ
//  548     if(left_lost_flag == UN_LOST)   //�޶�search_scope(ɨ�跶Χ)�������ֶ����ʱ������ɨ�跶Χ
//  549         se_scope = SEARCH_SCOPE;//��j����ֵ
//  550      else
//  551      {
//  552        se_scope=(uint8)((right_black[i-1] - left_black[i-1]) / 2);
//  553      } 
//  554      
//  555      j=left_black[i-1] + se_scope;// ����ɨ������Ϊleft_black[i-1]+se_scope
//  556      if(j > COLUMN-1)
//  557        j = COLUMN-1;              //��j�����޷�
//  558          /////////////// 
//  559      
//  560      
//  561      left_black[i] = left_black[i-1] + se_scope;//Ԥ���趨left_black[i]��ֵ
//  562      if(left_black[i] > COLUMN-1)    //  ��left_black[i]�����޷�
//  563        left_black[i] = COLUMN-1;
//  564      
//  565      
//  566         while(  j > 0 && j > left_black[i-1] - se_scope )
//  567           { 
//  568             if(VideoImage1[i][j] - VideoImage1[i][j-2] > OT) 
//  569             {
//  570               if(f_abs16(j - left_black[i-1]) < f_abs16(left_black[i] - left_black[i-1]))//�˳�����
//  571                 left_black[i] = j;
//  572             }
//  573             j--;
//  574           }
//  575           if(left_black[i] <= 1)      //��������Χ��û���ҵ�����㣬����Ϊ��ͼ����Ȼ����
//  576           {
//  577             left_black[i] = LEFT_OUT_LOST;   //0
//  578           }
//  579           else if(left_black[i] == left_black[i-1] + se_scope)//left_black[i]��ֵû�б仯��˵��û���ҵ������
//  580           {
//  581             if(i >= 6)     
//  582              {     
//  583                if(left_lost_flag == UN_LOST)//ֻ��¼�ڶϵ�Ŀ�ʼ
//  584                {
//  585                  first_amend_offset = left_black[i-2]+left_black[i-3];
//  586                  second_amend_offset = left_black[i-5]+left_black[i-6];
//  587                  amend_offset = (first_amend_offset-second_amend_offset)/6;  //����ϵ�ǰ�ı仯����
//  588                  left_lost_flag = LOST;                       //���Ϊ�����״̬
//  589                  left_lost_start_line = i-1;                    //��¼�����ǰһ�е���
//  590                }  
//  591                left_black[i] = left_black[i-1] + amend_offset;     //���ڶϵ���е�һ�ε��鹹
//  592 
//  593               //��ʱ��Ҫ����������ж�һ���ǲ��������Ч�еģ���������Ҫ��¼����
//  594               if( left_top_whiteline == ROW - 1)
//  595               {
//  596                n = -1;
//  597                while(VideoImage1[i + n - 1][j+(right_black[i - 1]-left_black[i - 1]) / 2] - VideoImage1[i + n][j + (right_black[i - 1] - left_black[i - 1]) / 2] < OT && n < 5)
//  598                {
//  599                  n++;
//  600                }
//  601               if(n < 5)//��������״̬��ʱ��˵���ҵ��������أ����ж�Ϊ�����Ч��
//  602                  {
//  603                     left_top_whiteline=i;
//  604                  }
//  605                }      
//  606              }
//  607              else
//  608              {
//  609                  left_black[i] = left_black[i-1];
//  610              }
//  611           }
//  612           else         //�ҵ��˵�
//  613           { 
//  614             if(left_lost_flag == LOST && f_abs16(left_black[i] - left_black[i-1]) <= 3 )
//  615             {
//  616               left_lost_end_line = i;//��¼�ϵ�Ľ�������
//  617               left_lost_flag = UN_LOST;    //���״̬Ϊû�ж���
//  618               for(k = left_lost_start_line;k <= left_lost_end_line;k++)
//  619               {
//  620                 left_black[k] = left_black[left_lost_start_line] + (k - left_lost_start_line)*(left_black[left_lost_end_line] - left_black[left_lost_start_line])/(left_lost_end_line - left_lost_start_line);
//  621                                             //���жԶϵ�ĵڶ����鹹
//  622               if(left_black[k] < 0)
//  623                 left_black[k] = LEFT_OUT_LOST;//����ߵĳ���
//  624               else if(left_black[k] > COLUMN-1)
//  625                 left_black[k] = RIGHT_OUT_LOST;//���ұߵĳ���
//  626               }
//  627             }
//  628           }
//  629       //��߸��ٽ���
//  630       
//  631       //�ұ߱����ߵĸ��� 
//  632         if(right_lost_flag == UN_LOST)
//  633           se_scope = SEARCH_SCOPE;//��j����ֵ
//  634        else
//  635          se_scope = (uint8)((right_black[i-1] - left_black[i-1])/2);
//  636        
//  637        j=right_black[i-1] - se_scope;
//  638        if(j < 0)
//  639          j = 0;
//  640        
//  641        right_black[i] = right_black[i-1] - se_scope;
//  642        if(right_black[i] < 0)
//  643          right_black[i] = 0;
//  644        
//  645         while(  j < COLUMN-1 && j < right_black[i-1] + se_scope)
//  646           {
//  647             if( VideoImage1[i][j-2] - VideoImage1[i][j] > OT)
//  648             {
//  649               if(f_abs16(j-right_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
//  650               {
//  651                 right_black[i] = j ;
//  652               }
//  653             }
//  654               j++;
//  655           }
//  656           if(right_black[i] >= COLUMN-2)      //��������Χ��û���ҵ�����㣬����Ϊ��ͼ����Ȼ����
//  657           {
//  658             right_black[i] = RIGHT_OUT_LOST;   //0   ����ȡ���ߵ�ʱ����
//  659           }
//  660           else if(right_black[i] == right_black[i-1] - se_scope)
//  661           {
//  662             if(i >= 6)
//  663             {
//  664               if(right_lost_flag == UN_LOST)
//  665              {
//  666                right_lost_flag = LOST;
//  667                right_lost_start_line = i-1;
//  668                first_amend_offset = right_black[i-2] + right_black[i-3];
//  669                second_amend_offset = right_black[i-5] + right_black[i-6];
//  670                amend_offset = (first_amend_offset - second_amend_offset)/6;
//  671              }
//  672               right_black[i] = right_black[i-1] + amend_offset; 
//  673               
//  674               //��ʱ��Ҫ�ж�һ���ǲ��������Ч�еģ���������Ҫ��¼����
//  675               if( right_top_whiteline == ROW - 1)
//  676               {
//  677                 n = -1;
//  678                while(VideoImage1[i+n-1][j - (right_black[i-1]-left_black[i-1])/2] - VideoImage1[i+n][j - (right_black[i-1]-left_black[i-1])/2] < OT && n < 5)
//  679                {
//  680                  n++;
//  681                }
//  682               if(n < 5)//��������״̬��ʱ��˵���ҵ��������أ����ж�Ϊ�����Ч��
//  683                  {
//  684                     right_top_whiteline=i;
//  685                  }
//  686               }
//  687               
//  688             }
//  689             else
//  690               right_black[i] = right_black[i-1];
//  691            
//  692           }
//  693           else
//  694           {
//  695             if(right_lost_flag == LOST && f_abs16(right_black[i] - right_black[i-1]) <= 3  )
//  696             {
//  697               right_lost_end_line = i;
//  698               right_lost_flag = UN_LOST;
//  699               for(k = right_lost_start_line;k <= right_lost_end_line;k++)
//  700               {
//  701                 right_black[k] = right_black[right_lost_start_line] + (k-right_lost_start_line)*(right_black[right_lost_end_line]-right_black[right_lost_start_line])/(right_lost_end_line-right_lost_start_line);
//  702               
//  703               if(right_black[k] < 0)
//  704                 right_black[k] = LEFT_OUT_LOST;//����ߵĳ���
//  705               else if(right_black[k] > COLUMN-1)
//  706                 right_black[k] = RIGHT_OUT_LOST;//���ұߵĳ���
//  707               }
//  708             }
//  709           } //�ұ߸��ٽ���
//  710         if(right_black[i] < left_black[i])
//  711          left_black[i] = right_black[i];
//  712     //������Ҫ���ߵ�����Ĵ���  
//  713         
//  714   //�����if�����Դ������ʹ���ʮ�ֵ�·�е���� 
//  715         //�����������쳣���Ҷ�δ��������
//  716      if((right_black[i] - left_black[i])-(right_black[i-1] - left_black[i-1])> 3  || (right_black[i]-left_black[i]) > refer_road_width[current_deal_line] + 15)//&& right_black[i] != RIGHT_OUT_LOST && left_black[i] != LEFT_OUT_LOST
//  717     {   
//  718       //������һ�ߵāG�ߵ���� 
//  719       if(( f_abs16(left_black[i] - left_black[i-1]) < f_abs16(right_black[i] - right_black[i-1])))//��ߵ�ͻ��С���ұߵ�  ˵���ұߵĵ㷢����ͻ��
//  720       {
//  721         right_black[i] = left_black[i] + (right_black[i-1] - left_black[i-1]);//+(right_black[i-1]-right_black[i-3])/2
//  722         if(right_black[k] < 0)
//  723           right_black[k] = LEFT_OUT_LOST;//����ߵĳ���
//  724         else if(right_black[k] > COLUMN-1)
//  725           right_black[k] = RIGHT_OUT_LOST;//���ұߵĳ���
//  726       }
//  727       
//  728        if(( f_abs16(left_black[i] - left_black[i-1]) > f_abs16(right_black[i] - right_black[i-1])) )//�ұߵ�ͻ��С����ߵ�  ˵����ߵĵ㷢����ͻ��//���ұߵĵ㲻����ͻ��
//  729       {
//  730           left_black[i] = right_black[i]-(right_black[i-1]-left_black[i-1]);//
//  731           
//  732           if(left_black[k] < 0)
//  733             left_black[k] = LEFT_OUT_LOST;//����ߵĳ���
//  734           else if(left_black[k] > COLUMN-1)
??Search_BlackEdge_1:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+161
        BLT.N    ??Search_BlackEdge_2
//  735             left_black[k] = RIGHT_OUT_LOST;//���ұߵĳ���
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        MOVS     R1,#+160
        STRH     R1,[R0, R4, LSL #+1]
??Search_BlackEdge_2:
        ADDS     R6,R6,#+1
??Search_BlackEdge_0:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+65
        BGE.W    ??Search_BlackEdge_3
        LDR.W    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Search_BlackEdge_4
        LDR.W    R0,??DataTable8_10
        MOVS     R1,#+10
        STR      R1,[R0, #+0]
        B.N      ??Search_BlackEdge_5
??Search_BlackEdge_4:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        SUBS     R0,R0,R1
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable8_10
        STR      R0,[R1, #+0]
??Search_BlackEdge_5:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        LDR.W    R1,??DataTable8_10
        LDR      R1,[R1, #+0]
        ADDS     R7,R1,R0
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,#+161
        BLT.N    ??Search_BlackEdge_6
        MOVS     R7,#+160
??Search_BlackEdge_6:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        LDR.W    R1,??DataTable8_10
        LDR      R1,[R1, #+0]
        ADDS     R0,R1,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        STRH     R0,[R1, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R6, LSL #+1]
        CMP      R0,#+161
        BLT.N    ??Search_BlackEdge_7
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        MOVS     R1,#+160
        STRH     R1,[R0, R6, LSL #+1]
        B.N      ??Search_BlackEdge_7
??Search_BlackEdge_8:
        LDR.W    R0,??DataTable7_4
        LDR      R0,[R0, #+0]
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        MOVS     R1,#+161
        LDR.W    R2,??DataTable7_9
        MLA      R1,R1,R6,R2
        LDRB     R1,[R7, R1]
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        MOVS     R2,#+161
        LDR.W    R3,??DataTable7_9
        MLA      R2,R2,R6,R3
        ADDS     R2,R7,R2
        LDRB     R2,[R2, #-2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_9
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        SUBS     R0,R7,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOV      R8,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R8,R0
        BGE.N    ??Search_BlackEdge_9
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        STRH     R7,[R0, R6, LSL #+1]
??Search_BlackEdge_9:
        SUBS     R7,R7,#+1
??Search_BlackEdge_7:
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,#+1
        BLT.N    ??Search_BlackEdge_10
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-2]
        LDR.W    R1,??DataTable8_10
        LDR      R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R0,R7
        BLT.N    ??Search_BlackEdge_8
??Search_BlackEdge_10:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R6, LSL #+1]
        CMP      R0,#+2
        BGE.N    ??Search_BlackEdge_11
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        MOVS     R1,#+0
        STRH     R1,[R0, R6, LSL #+1]
??Search_BlackEdge_12:
        LDR.W    R0,??DataTable8_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.W    ??Search_BlackEdge_13
        LDR.W    R0,??DataTable8_10
        MOVS     R1,#+10
        STR      R1,[R0, #+0]
        B.N      ??Search_BlackEdge_14
??Search_BlackEdge_11:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        LDR.W    R2,??DataTable8_10
        LDR      R2,[R2, #+0]
        SXTAH    R1,R2,R1
        CMP      R0,R1
        BNE.W    ??Search_BlackEdge_15
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+6
        BLT.W    ??Search_BlackEdge_16
        LDR.W    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Search_BlackEdge_17
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-4]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-6]
        SXTAH    R1,R1,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-10]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_9
        ADDS     R2,R2,R6, LSL #+1
        LDRSH    R2,[R2, #-12]
        SXTAH    R2,R2,R0
        SUBS     R0,R1,R2
        MOVS     R1,#+6
        SDIV     R5,R0,R1
        LDR.W    R0,??DataTable8_2
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        MOVS     R0,R6
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable8
        STRB     R0,[R1, #+0]
??Search_BlackEdge_17:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        ADDS     R0,R5,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        STRH     R0,[R1, R6, LSL #+1]
        LDR.W    R0,??DataTable8_7
        LDR      R0,[R0, #+0]
        CMP      R0,#+64
        BNE.N    ??Search_BlackEdge_12
        MOVS     R0,#-1
        B.N      ??Search_BlackEdge_18
??Search_BlackEdge_19:
        ADDS     R0,R0,#+1
??Search_BlackEdge_18:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_9
        ADDS     R2,R2,R6, LSL #+1
        LDRSH    R2,[R2, #-2]
        SUBS     R1,R1,R2
        MOVS     R2,#+2
        SDIV     R1,R1,R2
        SXTAH    R1,R1,R7
        SXTAH    R2,R0,R6
        MOVS     R3,#+161
        LDR.W    R12,??DataTable7_9
        MLA      R2,R3,R2,R12
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-161]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_11
        ADDS     R2,R2,R6, LSL #+1
        LDRSH    R2,[R2, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R3,??DataTable8_9
        ADDS     R3,R3,R6, LSL #+1
        LDRSH    R3,[R3, #-2]
        SUBS     R2,R2,R3
        MOVS     R3,#+2
        SDIV     R2,R2,R3
        SXTAH    R2,R2,R7
        SXTAH    R3,R0,R6
        MOVS     R12,#+161
        LDR.W    LR,??DataTable7_9
        MLA      R3,R12,R3,LR
        LDRB     R2,[R2, R3]
        SUBS     R1,R1,R2
        LDR.W    R2,??DataTable7_4
        LDR      R2,[R2, #+0]
        CMP      R1,R2
        BGE.N    ??Search_BlackEdge_20
        CMP      R0,#+5
        BLT.N    ??Search_BlackEdge_19
??Search_BlackEdge_20:
        CMP      R0,#+5
        BGE.W    ??Search_BlackEdge_12
        LDR.W    R0,??DataTable8_7
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        STR      R6,[R0, #+0]
        B.N      ??Search_BlackEdge_12
??Search_BlackEdge_16:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        STRH     R0,[R1, R6, LSL #+1]
        B.N      ??Search_BlackEdge_12
??Search_BlackEdge_15:
        LDR.W    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??Search_BlackEdge_12
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+4
        BGE.W    ??Search_BlackEdge_12
        LDR.W    R0,??DataTable8_1
        STRB     R6,[R0, #+0]
        LDR.W    R0,??DataTable8_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable8
        LDRB     R4,[R0, #+0]
        B.N      ??Search_BlackEdge_21
??Search_BlackEdge_22:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+161
        BLT.N    ??Search_BlackEdge_23
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        MOVS     R1,#+160
        STRH     R1,[R0, R4, LSL #+1]
??Search_BlackEdge_23:
        ADDS     R4,R4,#+1
??Search_BlackEdge_21:
        LDR.W    R0,??DataTable8_1
        LDRB     R0,[R0, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BLT.W    ??Search_BlackEdge_12
        LDR.W    R0,??DataTable8
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable8_9
        LDRH     R0,[R1, R0, LSL #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8
        LDRB     R1,[R1, #+0]
        SUBS     R1,R4,R1
        LDR.W    R2,??DataTable8_1
        LDRB     R2,[R2, #+0]
        LDR.W    R3,??DataTable8_9
        LDRSH    R2,[R3, R2, LSL #+1]
        LDR.W    R3,??DataTable8
        LDRB     R3,[R3, #+0]
        LDR.W    R7,??DataTable8_9
        LDRSH    R3,[R7, R3, LSL #+1]
        SUBS     R2,R2,R3
        MULS     R1,R2,R1
        LDR.W    R2,??DataTable8_1
        LDRB     R2,[R2, #+0]
        LDR.W    R3,??DataTable8
        LDRB     R3,[R3, #+0]
        SUBS     R2,R2,R3
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8_9
        STRH     R0,[R1, R4, LSL #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+0
        BPL.N    ??Search_BlackEdge_22
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        MOVS     R1,#+0
        STRH     R1,[R0, R4, LSL #+1]
        B.N      ??Search_BlackEdge_23
??Search_BlackEdge_13:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        SUBS     R0,R0,R1
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable8_10
        STR      R0,[R1, #+0]
??Search_BlackEdge_14:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        LDR.W    R1,??DataTable8_10
        LDR      R1,[R1, #+0]
        SUBS     R7,R0,R1
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,#+0
        BPL.N    ??Search_BlackEdge_24
        MOVS     R7,#+0
??Search_BlackEdge_24:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        LDR.W    R1,??DataTable8_10
        LDR      R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        STRH     R0,[R1, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R6, LSL #+1]
        CMP      R0,#+0
        BPL.N    ??Search_BlackEdge_25
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        MOVS     R1,#+0
        STRH     R1,[R0, R6, LSL #+1]
        B.N      ??Search_BlackEdge_25
??Search_BlackEdge_26:
        LDR.W    R0,??DataTable7_4
        LDR      R0,[R0, #+0]
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        MOVS     R1,#+161
        LDR.W    R2,??DataTable7_9
        MLA      R1,R1,R6,R2
        ADDS     R1,R7,R1
        LDRB     R1,[R1, #-2]
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        MOVS     R2,#+161
        LDR.W    R3,??DataTable7_9
        MLA      R2,R2,R6,R3
        LDRB     R2,[R7, R2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_27
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        SUBS     R0,R7,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOV      R8,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R8,R0
        BGE.N    ??Search_BlackEdge_27
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        STRH     R7,[R0, R6, LSL #+1]
??Search_BlackEdge_27:
        ADDS     R7,R7,#+1
??Search_BlackEdge_25:
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,#+160
        BGE.N    ??Search_BlackEdge_28
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-2]
        LDR.W    R1,??DataTable8_10
        LDR      R1,[R1, #+0]
        SXTAH    R0,R1,R0
        CMP      R7,R0
        BLT.N    ??Search_BlackEdge_26
??Search_BlackEdge_28:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R6, LSL #+1]
        CMP      R0,#+159
        BLT.W    ??Search_BlackEdge_29
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        MOVS     R1,#+160
        STRH     R1,[R0, R6, LSL #+1]
??Search_BlackEdge_30:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        LDRSH    R1,[R1, R6, LSL #+1]
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_31
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        LDRH     R1,[R1, R6, LSL #+1]
        STRH     R1,[R0, R6, LSL #+1]
??Search_BlackEdge_31:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        LDRSH    R1,[R1, R6, LSL #+1]
        SUBS     R0,R0,R1
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        SXTAH    R0,R0,R1
        CMP      R0,#+4
        BGE.N    ??Search_BlackEdge_32
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable7_8
        LDRH     R0,[R1, R0, LSL #+1]
        ADDS     R0,R0,#+15
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        LDRSH    R1,[R1, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_9
        LDRSH    R2,[R2, R6, LSL #+1]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.W    ??Search_BlackEdge_2
??Search_BlackEdge_32:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R7,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R7,R0
        BGE.W    ??Search_BlackEdge_33
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        ADDS     R0,R1,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        STRH     R0,[R1, R6, LSL #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+0
        BPL.W    ??Search_BlackEdge_34
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        MOVS     R1,#+0
        STRH     R1,[R0, R4, LSL #+1]
        B.N      ??Search_BlackEdge_33
??Search_BlackEdge_29:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        LDR.W    R2,??DataTable8_10
        LDR      R2,[R2, #+0]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BNE.W    ??Search_BlackEdge_35
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+6
        BLT.W    ??Search_BlackEdge_36
        LDR.W    R0,??DataTable8_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Search_BlackEdge_37
        LDR.W    R0,??DataTable8_5
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        MOVS     R0,R6
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable8_3
        STRB     R0,[R1, #+0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-4]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-6]
        SXTAH    R1,R1,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRSH    R0,[R0, #-10]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_11
        ADDS     R2,R2,R6, LSL #+1
        LDRSH    R2,[R2, #-12]
        SXTAH    R2,R2,R0
        SUBS     R0,R1,R2
        MOVS     R1,#+6
        SDIV     R5,R0,R1
??Search_BlackEdge_37:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        ADDS     R0,R5,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        STRH     R0,[R1, R6, LSL #+1]
        LDR.W    R0,??DataTable8_8
        LDR      R0,[R0, #+0]
        CMP      R0,#+64
        BNE.W    ??Search_BlackEdge_30
        MOVS     R0,#-1
        B.N      ??Search_BlackEdge_38
??Search_BlackEdge_39:
        ADDS     R0,R0,#+1
??Search_BlackEdge_38:
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRSH    R1,[R1, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_9
        ADDS     R2,R2,R6, LSL #+1
        LDRSH    R2,[R2, #-2]
        SUBS     R1,R1,R2
        MOVS     R2,#+2
        SDIV     R1,R1,R2
        SUBS     R1,R7,R1
        SXTAH    R2,R0,R6
        MOVS     R3,#+161
        LDR.W    R12,??DataTable7_9
        MLA      R2,R3,R2,R12
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-161]
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R2,??DataTable8_11
        ADDS     R2,R2,R6, LSL #+1
        LDRSH    R2,[R2, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R3,??DataTable8_9
        ADDS     R3,R3,R6, LSL #+1
        LDRSH    R3,[R3, #-2]
        SUBS     R2,R2,R3
        MOVS     R3,#+2
        SDIV     R2,R2,R3
        SUBS     R2,R7,R2
        SXTAH    R3,R0,R6
        MOVS     R12,#+161
        LDR.W    LR,??DataTable7_9
        MLA      R3,R12,R3,LR
        LDRB     R2,[R2, R3]
        SUBS     R1,R1,R2
        LDR.N    R2,??DataTable7_4
        LDR      R2,[R2, #+0]
        CMP      R1,R2
        BGE.N    ??Search_BlackEdge_40
        CMP      R0,#+5
        BLT.N    ??Search_BlackEdge_39
??Search_BlackEdge_40:
        CMP      R0,#+5
        BGE.W    ??Search_BlackEdge_30
        LDR.W    R0,??DataTable8_8
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        STR      R6,[R0, #+0]
        B.N      ??Search_BlackEdge_30
??Search_BlackEdge_36:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        ADDS     R0,R0,R6, LSL #+1
        LDRH     R0,[R0, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        STRH     R0,[R1, R6, LSL #+1]
        B.N      ??Search_BlackEdge_30
??Search_BlackEdge_35:
        LDR.W    R0,??DataTable8_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??Search_BlackEdge_30
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+4
        BGE.W    ??Search_BlackEdge_30
        LDR.W    R0,??DataTable8_4
        STRB     R6,[R0, #+0]
        LDR.W    R0,??DataTable8_5
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable8_3
        LDRB     R4,[R0, #+0]
        B.N      ??Search_BlackEdge_41
??Search_BlackEdge_42:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+161
        BLT.N    ??Search_BlackEdge_43
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        MOVS     R1,#+160
        STRH     R1,[R0, R4, LSL #+1]
??Search_BlackEdge_43:
        ADDS     R4,R4,#+1
??Search_BlackEdge_41:
        LDR.W    R0,??DataTable8_4
        LDRB     R0,[R0, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BLT.W    ??Search_BlackEdge_30
        LDR.W    R0,??DataTable8_3
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable8_11
        LDRH     R0,[R1, R0, LSL #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8_3
        LDRB     R1,[R1, #+0]
        SUBS     R1,R4,R1
        LDR.W    R2,??DataTable8_4
        LDRB     R2,[R2, #+0]
        LDR.W    R3,??DataTable8_11
        LDRSH    R2,[R3, R2, LSL #+1]
        LDR.W    R3,??DataTable8_3
        LDRB     R3,[R3, #+0]
        LDR.W    R7,??DataTable8_11
        LDRSH    R3,[R7, R3, LSL #+1]
        SUBS     R2,R2,R3
        MULS     R1,R2,R1
        LDR.W    R2,??DataTable8_4
        LDRB     R2,[R2, #+0]
        LDR.W    R3,??DataTable8_3
        LDRB     R3,[R3, #+0]
        SUBS     R2,R2,R3
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8_11
        STRH     R0,[R1, R4, LSL #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+0
        BPL.N    ??Search_BlackEdge_42
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        MOVS     R1,#+0
        STRH     R1,[R0, R4, LSL #+1]
        B.N      ??Search_BlackEdge_43
??Search_BlackEdge_34:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+161
        BLT.N    ??Search_BlackEdge_33
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_11
        MOVS     R1,#+160
        STRH     R1,[R0, R4, LSL #+1]
??Search_BlackEdge_33:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R7,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R7,R0
        BGE.W    ??Search_BlackEdge_2
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R0,??DataTable8_11
        LDRH     R0,[R0, R6, LSL #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_11
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        ADDS     R1,R1,R6, LSL #+1
        LDRH     R1,[R1, #-2]
        ADDS     R0,R1,R0
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        LDR.W    R1,??DataTable8_9
        STRH     R0,[R1, R6, LSL #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        LDRSH    R0,[R0, R4, LSL #+1]
        CMP      R0,#+0
        BPL.W    ??Search_BlackEdge_1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable8_9
        MOVS     R1,#+0
        STRH     R1,[R0, R4, LSL #+1]
        B.W      ??Search_BlackEdge_2
//  736       }
//  737       //һ�ߵāG������������
//  738     }
//  739     
//  740   }//��ɨ���for����
//  741   
//  742 }
??Search_BlackEdge_3:
        POP      {R4-R8,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_1:
        DC32     find_whitebase_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_2:
        DC32     whitebase_line_cnt

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_3:
        DC32     white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_4:
        DC32     OT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_5:
        DC32     whitepoint_start

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_6:
        DC32     whitepoint_end

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_7:
        DC32     current_deal_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_8:
        DC32     refer_road_width

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_9:
        DC32     VideoImage1
//  743 
//  744 //------------------------ͨ���ҳ����������ԣ��������߽��д�����鹹----------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  745 void Deal_BlackEdge(void)
//  746 {
//  747   int i = 0;
Deal_BlackEdge:
        MOVS     R0,#+0
//  748   //�������Ч�н���ѡ��
//  749   if(left_top_whiteline > right_top_whiteline)
        LDR.W    R1,??DataTable8_8
        LDR      R1,[R1, #+0]
        LDR.W    R2,??DataTable8_7
        LDR      R2,[R2, #+0]
        CMP      R1,R2
        BGE.N    ??Deal_BlackEdge_0
//  750     top_whiteline=left_top_whiteline;
        LDR.W    R0,??DataTable8_6
        LDR.W    R1,??DataTable8_7
        LDR      R1,[R1, #+0]
        STR      R1,[R0, #+0]
        B.N      ??Deal_BlackEdge_1
//  751   else
//  752         top_whiteline=right_top_whiteline;
??Deal_BlackEdge_0:
        LDR.N    R0,??DataTable8_6
        LDR.N    R1,??DataTable8_8
        LDR      R1,[R1, #+0]
        STR      R1,[R0, #+0]
//  753     
//  754     for(i = bottom_whitebase+3;i <= top_whiteline;i++)//��׼�е����б������Ѿ��ҵ���
??Deal_BlackEdge_1:
        LDR.W    R0,??DataTable12
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+3
        B.N      ??Deal_BlackEdge_2
//  755     { 
//  756       if(right_black[i] - left_black[i] >20 )//�ж������Ч��
//  757       {
//  758         if(left_black[i] != LEFT_OUT_LOST && left_black[i] != RIGHT_OUT_LOST 
//  759            && right_black[i] != LEFT_OUT_LOST && right_black[i] != RIGHT_OUT_LOST)//�����߶�û�г����ʱ���������ұ�����������
??Deal_BlackEdge_3:
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_4
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BEQ.N    ??Deal_BlackEdge_4
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_4
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BEQ.N    ??Deal_BlackEdge_4
//  760         {
//  761           center_white[i] = left_black[i] + right_black[i];
        LDR.N    R1,??DataTable8_9
        LDRH     R1,[R1, R0, LSL #+1]
        LDR.N    R2,??DataTable8_11
        LDRH     R2,[R2, R0, LSL #+1]
        ADDS     R1,R2,R1
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  762           center_white[i] = center_white[i] >> 1;
        LDR.W    R1,??DataTable15
        LDRSH    R1,[R1, R0, LSL #+1]
        ASRS     R1,R1,#+1
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  763         }
//  764   
//  765           if(left_black[i] != LEFT_OUT_LOST && left_black[i] != RIGHT_OUT_LOST 
//  766            && (right_black[i] == LEFT_OUT_LOST || right_black[i] == RIGHT_OUT_LOST))//��ֻ��һ�߳����ʱ��   �ұ߳���
??Deal_BlackEdge_4:
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_5
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BEQ.N    ??Deal_BlackEdge_5
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_6
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BNE.N    ??Deal_BlackEdge_5
//  767            {
//  768              
//  769               center_white[i] = left_black[i] + (right_black[i-1] - left_black[i-1] + right_black[i-2] - left_black[i-2])/4;
??Deal_BlackEdge_6:
        LDR.N    R1,??DataTable8_9
        LDRH     R1,[R1, R0, LSL #+1]
        LDR.N    R2,??DataTable8_11
        ADDS     R2,R2,R0, LSL #+1
        LDRSH    R2,[R2, #-2]
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #-2]
        SUBS     R2,R2,R3
        LDR.N    R3,??DataTable8_11
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #-4]
        SXTAH    R2,R2,R3
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #-4]
        SUBS     R2,R2,R3
        MOVS     R3,#+4
        SDIV     R2,R2,R3
        ADDS     R1,R2,R1
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  770               if(center_white[i] > COLUMN-1)
        LDR.W    R1,??DataTable15
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+161
        BLT.N    ??Deal_BlackEdge_5
//  771                 center_white[i] = COLUMN-1;
        LDR.W    R1,??DataTable15
        MOVS     R2,#+160
        STRH     R2,[R1, R0, LSL #+1]
//  772            }
//  773         
//  774          if(right_black[i] != LEFT_OUT_LOST && right_black[i] != RIGHT_OUT_LOST 
//  775            && (left_black[i] == LEFT_OUT_LOST || left_black[i] == RIGHT_OUT_LOST))//��ֻ��һ�߳����ʱ��   ��߳���
??Deal_BlackEdge_5:
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_7
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BEQ.N    ??Deal_BlackEdge_7
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_8
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BNE.N    ??Deal_BlackEdge_7
//  776            {
//  777               center_white[i] = right_black[i] - (right_black[i-1] - left_black[i-1] + right_black[i-2] - left_black[i-2])/4;
??Deal_BlackEdge_8:
        LDR.N    R1,??DataTable8_11
        LDRH     R1,[R1, R0, LSL #+1]
        LDR.N    R2,??DataTable8_11
        ADDS     R2,R2,R0, LSL #+1
        LDRSH    R2,[R2, #-2]
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #-2]
        SUBS     R2,R2,R3
        LDR.N    R3,??DataTable8_11
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #-4]
        SXTAH    R2,R2,R3
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #-4]
        SUBS     R2,R2,R3
        MOVS     R3,#+4
        SDIV     R2,R2,R3
        SUBS     R1,R1,R2
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  778               if(center_white[i] < 0)
        LDR.W    R1,??DataTable15
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BPL.N    ??Deal_BlackEdge_7
//  779                 center_white[i] = 0;
        LDR.W    R1,??DataTable15
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
//  780            }
//  781       }
??Deal_BlackEdge_7:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_2:
        LDR.N    R1,??DataTable8_6
        LDR      R1,[R1, #+0]
        CMP      R1,R0
        BLT.N    ??Deal_BlackEdge_9
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        LDR.N    R2,??DataTable8_9
        LDRSH    R2,[R2, R0, LSL #+1]
        SUBS     R1,R1,R2
        CMP      R1,#+21
        BGE.W    ??Deal_BlackEdge_3
//  782       else//�������������Ч�е�Ҫ��ʱ������forѭ��
//  783       {
//  784         top_whiteline = i-1;
        SUBS     R0,R0,#+1
        LDR.N    R1,??DataTable8_6
        STR      R0,[R1, #+0]
//  785         break;
//  786       }
//  787     }
//  788     if( bottom_whitebase > 0  && bottom_whitebase < ROW)  // ����׼�в��ǵ����е�ʱ�򣬴�ʱ������MIDֵȥ�������ߣ�������bottom_whitebase��ز���
??Deal_BlackEdge_9:
        LDR.W    R0,??DataTable12
        LDR      R0,[R0, #+0]
        SUBS     R0,R0,#+1
        CMP      R0,#+64
        BCS.W    ??Deal_BlackEdge_10
//  789     {
//  790        for(i = bottom_whitebase;i >= 0;i--)
        LDR.W    R0,??DataTable12
        LDR      R0,[R0, #+0]
        B.N      ??Deal_BlackEdge_11
//  791        {
//  792          //����һ�߁G�ߵ����
//  793           if(left_black[i] != LEFT_OUT_LOST && left_black[i] != RIGHT_OUT_LOST 
//  794            && (right_black[i] == LEFT_OUT_LOST || right_black[i] == RIGHT_OUT_LOST))//��ֻ��һ�߳����ʱ��   �ұ߳���
??Deal_BlackEdge_12:
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_13
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BEQ.N    ??Deal_BlackEdge_13
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_14
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BNE.N    ??Deal_BlackEdge_13
//  795            {
//  796              
//  797               center_white[i] = left_black[i] + (right_black[i+1] - left_black[i + 1] + right_black[i + 2] - left_black[i + 2])/4;
??Deal_BlackEdge_14:
        LDR.N    R1,??DataTable8_9
        LDRH     R1,[R1, R0, LSL #+1]
        LDR.N    R2,??DataTable8_11
        ADDS     R2,R2,R0, LSL #+1
        LDRSH    R2,[R2, #+2]
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #+2]
        SUBS     R2,R2,R3
        LDR.N    R3,??DataTable8_11
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #+4]
        SXTAH    R2,R2,R3
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #+4]
        SUBS     R2,R2,R3
        MOVS     R3,#+4
        SDIV     R2,R2,R3
        ADDS     R1,R2,R1
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  798               if(center_white[i] > COLUMN-1)
        LDR.W    R1,??DataTable15
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+161
        BLT.N    ??Deal_BlackEdge_13
//  799                 center_white[i] = COLUMN-1;
        LDR.W    R1,??DataTable15
        MOVS     R2,#+160
        STRH     R2,[R1, R0, LSL #+1]
//  800            }
//  801         
//  802          if(right_black[i] != LEFT_OUT_LOST && right_black[i] != RIGHT_OUT_LOST 
//  803            && (left_black[i] == LEFT_OUT_LOST || left_black[i] == RIGHT_OUT_LOST))//��ֻ��һ�߳����ʱ��   ��߳���
??Deal_BlackEdge_13:
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_15
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BEQ.N    ??Deal_BlackEdge_15
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_16
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BNE.N    ??Deal_BlackEdge_15
//  804            {
//  805               center_white[i] = right_black[i] - (right_black[i + 1] - left_black[i + 1] + right_black[i + 2] - left_black[i + 2])/4;
??Deal_BlackEdge_16:
        LDR.N    R1,??DataTable8_11
        LDRH     R1,[R1, R0, LSL #+1]
        LDR.N    R2,??DataTable8_11
        ADDS     R2,R2,R0, LSL #+1
        LDRSH    R2,[R2, #+2]
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #+2]
        SUBS     R2,R2,R3
        LDR.N    R3,??DataTable8_11
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #+4]
        SXTAH    R2,R2,R3
        LDR.N    R3,??DataTable8_9
        ADDS     R3,R3,R0, LSL #+1
        LDRSH    R3,[R3, #+4]
        SUBS     R2,R2,R3
        MOVS     R3,#+4
        SDIV     R2,R2,R3
        SUBS     R1,R1,R2
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  806               if(center_white[i] < 0)
        LDR.W    R1,??DataTable15
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BPL.N    ??Deal_BlackEdge_15
//  807                 center_white[i] = 0;
        LDR.W    R1,??DataTable15
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
//  808            }
//  809           
//  810           //��������ͬʱ�G�ߵ����
//  811            if((left_black[i]==LEFT_OUT_LOST || left_black[i] == RIGHT_OUT_LOST )
//  812            && (right_black[i] == LEFT_OUT_LOST || right_black[i] == RIGHT_OUT_LOST))//��ֻ��һ�߳����ʱ��   �ұ߳���
??Deal_BlackEdge_15:
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_17
        LDR.N    R1,??DataTable8_9
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BNE.N    ??Deal_BlackEdge_18
??Deal_BlackEdge_17:
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Deal_BlackEdge_19
        LDR.N    R1,??DataTable8_11
        LDRSH    R1,[R1, R0, LSL #+1]
        CMP      R1,#+160
        BNE.N    ??Deal_BlackEdge_18
//  813            {
//  814              
//  815               center_white[i] = center_white[i + 1];
??Deal_BlackEdge_19:
        LDR.W    R1,??DataTable15
        ADDS     R1,R1,R0, LSL #+1
        LDRH     R1,[R1, #+2]
        LDR.W    R2,??DataTable15
        STRH     R1,[R2, R0, LSL #+1]
//  816            }
//  817        }
??Deal_BlackEdge_18:
        SUBS     R0,R0,#+1
??Deal_BlackEdge_11:
        CMP      R0,#+0
        BPL.W    ??Deal_BlackEdge_12
//  818        bottom_whitebase = 0;
        LDR.W    R0,??DataTable12
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  819     }
//  820     
//  821     if(top_whiteline+1 < ROW-1)
??Deal_BlackEdge_10:
        LDR.N    R0,??DataTable8_6
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        CMP      R0,#+64
        BGE.N    ??Deal_BlackEdge_20
//  822     {
//  823       for(i = top_whiteline + 1;i < ROW;i++)
        LDR.N    R0,??DataTable8_6
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        B.N      ??Deal_BlackEdge_21
//  824          center_white[i] =  MID;
??Deal_BlackEdge_22:
        LDR.W    R1,??DataTable15
        MOVS     R2,#+80
        STRH     R2,[R1, R0, LSL #+1]
        ADDS     R0,R0,#+1
??Deal_BlackEdge_21:
        CMP      R0,#+65
        BLT.N    ??Deal_BlackEdge_22
//  825     }
//  826     
//  827 }
??Deal_BlackEdge_20:
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8:
        DC32     left_lost_start_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_1:
        DC32     left_lost_end_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_2:
        DC32     left_lost_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_3:
        DC32     right_lost_start_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_4:
        DC32     right_lost_end_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_5:
        DC32     right_lost_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_6:
        DC32     top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_7:
        DC32     left_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_8:
        DC32     right_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_9:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_10:
        DC32     se_scope

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_11:
        DC32     right_black
//  828 //------------------------���ڷ��ͺ���------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//  829 void SCI0_send_mesage(void)
//  830 {
SCI0_send_mesage:
        PUSH     {R3-R5,LR}
//  831     int i = 0,j = 0;
        MOVS     R4,#+0
        MOVS     R5,#+0
//  832     static bool sci_temp = 0;
//  833     DisableInterrupts;  //����ͼ������ʱ��Ҫ�ر������жϣ���������    
        CPSID i         
//  834     if(send_mes == 1)  //ͼ��
        LDR.W    R0,??DataTable16
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??SCI0_send_mesage_0
//  835     {  
//  836       while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
??SCI0_send_mesage_1:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_1
//  837         UART0_D = OT;//���ڷ�ֵ�Ĳ����ڣ�������ֻ�������д��һ������
        LDR.W    R0,??DataTable15_2  ;; 0x4006a007
        LDR.W    R1,??DataTable14
        LDR      R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  838               while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
??SCI0_send_mesage_2:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_2
//  839         UART0_D = (uint8)ROW;
        LDR.W    R0,??DataTable15_2  ;; 0x4006a007
        MOVS     R1,#+65
        STRB     R1,[R0, #+0]
//  840               while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
??SCI0_send_mesage_3:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_3
//  841         UART0_D = (uint8)COLUMN;
        LDR.W    R0,??DataTable15_2  ;; 0x4006a007
        MOVS     R1,#+161
        STRB     R1,[R0, #+0]
//  842         
//  843         
//  844         //��λ����ʾ�ĵ�һ���������Ͻǣ������ҷ���ʱ���һ����ͷ����Ͻǵĵ�
//  845       for(i =ROW-1;i>=0;i--)
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_4
//  846       {
//  847         for(j=0;j<COLUMN;j++)
//  848         {
//  849           while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
??SCI0_send_mesage_5:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_5
//  850           UART0_D =  VideoImage1[i][j];///�����һ����������
        MOVS     R0,#+161
        LDR.W    R1,??DataTable15_3
        MLA      R0,R0,R4,R1
        LDRB     R0,[R5, R0]
        LDR.W    R1,??DataTable15_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
//  851           Delay_MS(80000);
        LDR.W    R0,??DataTable15_4  ;; 0x13880
        BL       Delay_MS
//  852         }
        ADDS     R5,R5,#+1
??SCI0_send_mesage_6:
        CMP      R5,#+161
        BLT.N    ??SCI0_send_mesage_5
        SUBS     R4,R4,#+1
??SCI0_send_mesage_4:
        CMP      R4,#+0
        BMI.N    ??SCI0_send_mesage_7
        MOVS     R5,#+0
        B.N      ??SCI0_send_mesage_6
//  853       }  
//  854       for (i =ROW-1 ;i >=0 ; i--)
??SCI0_send_mesage_7:
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_8
//  855       {
//  856         while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
??SCI0_send_mesage_9:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_9
//  857            UART0_D = left_black[i];
        LDR.W    R0,??DataTable12_1
        LDRH     R0,[R0, R4, LSL #+1]
        LDR.W    R1,??DataTable15_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
//  858         Delay_MS(80000);
        LDR.W    R0,??DataTable15_4  ;; 0x13880
        BL       Delay_MS
//  859       }
        SUBS     R4,R4,#+1
??SCI0_send_mesage_8:
        CMP      R4,#+0
        BPL.N    ??SCI0_send_mesage_9
//  860       for (i =ROW-1 ;i >=0 ; i--)
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_10
//  861       {
//  862          while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
??SCI0_send_mesage_11:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_11
//  863          UART0_D = right_black[i];
        LDR.W    R0,??DataTable15_5
        LDRH     R0,[R0, R4, LSL #+1]
        LDR.W    R1,??DataTable15_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
//  864          Delay_MS(80000);
        LDR.W    R0,??DataTable15_4  ;; 0x13880
        BL       Delay_MS
//  865       }
        SUBS     R4,R4,#+1
??SCI0_send_mesage_10:
        CMP      R4,#+0
        BPL.N    ??SCI0_send_mesage_11
//  866       for (i =ROW-1 ;i >=0 ; i--)
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_12
//  867       {
//  868         while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
??SCI0_send_mesage_13:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_13
//  869         UART0_D = center_white[i];
        LDR.W    R0,??DataTable15
        LDRH     R0,[R0, R4, LSL #+1]
        LDR.W    R1,??DataTable15_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
//  870         Delay_MS(80000);
        LDR.W    R0,??DataTable15_4  ;; 0x13880
        BL       Delay_MS
//  871       }
        SUBS     R4,R4,#+1
??SCI0_send_mesage_12:
        CMP      R4,#+0
        BPL.N    ??SCI0_send_mesage_13
//  872       send_mes = 0;  //����һ�μ��ɣ�����Ҫ����
        LDR.W    R0,??DataTable16
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??SCI0_send_mesage_14
//  873      }
//  874    // EnableInterrupts;  //���¿��������ж�
//  875  
//  876    else if (send_mes == 3)  //���ڵ���
??SCI0_send_mesage_0:
        LDR.W    R0,??DataTable16
        LDRB     R0,[R0, #+0]
        CMP      R0,#+3
        BNE.N    ??SCI0_send_mesage_15
//  877     {  
//  878         if(!sci_temp)
        LDR.W    R0,??DataTable15_6
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??SCI0_send_mesage_16
//  879         {
//  880        while(!(UART0_S1&UART_S1_TDRE_MASK));
??SCI0_send_mesage_17:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_17
//  881        UART0_D = (uint8)(speed_feedback);
        LDR.W    R0,??DataTable15_2  ;; 0x4006a007
        LDR.W    R1,??DataTable15_7
        LDRH     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  882        sci_temp = !sci_temp;
        LDR.W    R0,??DataTable15_6
        LDRB     R0,[R0, #+0]
        EORS     R0,R0,#0x1
        LDR.W    R1,??DataTable15_6
        STRB     R0,[R1, #+0]
        B.N      ??SCI0_send_mesage_14
//  883         }
//  884         else
//  885         {
//  886          while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
??SCI0_send_mesage_16:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_16
//  887          UART0_D= (uint8)(speed_except);
        LDR.W    R0,??DataTable15_2  ;; 0x4006a007
        LDR.W    R1,??DataTable15_8
        LDRH     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  888           sci_temp = !sci_temp;
        LDR.W    R0,??DataTable15_6
        LDRB     R0,[R0, #+0]
        EORS     R0,R0,#0x1
        LDR.W    R1,??DataTable15_6
        STRB     R0,[R1, #+0]
        B.N      ??SCI0_send_mesage_14
//  889         }
//  890         //       send_mes = 0;����0��Ϊ����������
//  891       }
//  892     else if (send_mes =='p')  //ͣ��
??SCI0_send_mesage_15:
        LDR.W    R0,??DataTable16
        LDRB     R0,[R0, #+0]
        CMP      R0,#+112
        BNE.N    ??SCI0_send_mesage_18
//  893     {
//  894        while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
??SCI0_send_mesage_19:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_19
//  895        stopflag = 1;
        LDR.W    R0,??DataTable15_9
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
//  896        send_mes = 0;
        LDR.W    R0,??DataTable16
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??SCI0_send_mesage_14
//  897     }
//  898     else if (send_mes == 's')  //����
??SCI0_send_mesage_18:
        LDR.W    R0,??DataTable16
        LDRB     R0,[R0, #+0]
        CMP      R0,#+115
        BNE.N    ??SCI0_send_mesage_14
//  899     {
//  900        while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
??SCI0_send_mesage_20:
        LDR.W    R0,??DataTable15_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_20
//  901        stopflag = 0;
        LDR.W    R0,??DataTable15_9
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  902        send_mes = 0;  
        LDR.W    R0,??DataTable16
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  903     }
//  904     EnableInterrupts;
??SCI0_send_mesage_14:
        CPSIE i         
//  905 }
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
??sci_temp:
        DS8 1
//  906 /*//------------------------------------������ƺ����Ĳ���-----------------------------------//
//  907 �����pid�������ڸ�����ʱ�򶼳��Ե�100�������ģ�Ȼ�������ļ�����Ƶ�ֵ��ʱ��
//  908 ��ͳһ�ĳ���100��
//  909  */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  910 void speed_control(void)
//  911 {  
speed_control:
        PUSH     {R4}
//  912   //speed_ed=(int16)(speed_ed_p*(top_whiteline-re_top_whiteline)/100);   
//  913   
//  914  // if(top_whiteline >= 62)
//  915       speed_except=16;
        LDR.W    R0,??DataTable15_8
        MOVS     R1,#+16
        STRH     R1,[R0, #+0]
//  916 //   else 
//  917  //     speed_except=10;
//  918    
//  919     speed_error=speed_except-speed_feedback;
        LDR.W    R0,??DataTable15_8
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable15_7
        LDRSH    R1,[R1, #+0]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
//  920      
//  921      speed_i_error+=speed_error;   
        LDR.W    R0,??DataTable15_10
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable15_10
        STRH     R0,[R1, #+0]
//  922     if(speed_i_error > 500)
        LDR.W    R0,??DataTable15_10
        LDRSH    R0,[R0, #+0]
        MOVW     R1,#+501
        CMP      R0,R1
        BLT.N    ??speed_control_0
//  923       speed_i_error = 500;
        LDR.W    R0,??DataTable15_10
        MOV      R1,#+500
        STRH     R1,[R0, #+0]
//  924     if(speed_i_error < -500)
??speed_control_0:
        LDR.W    R0,??DataTable15_10
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable15_11  ;; 0xfffffe0c
        CMP      R0,R1
        BGE.N    ??speed_control_1
//  925       speed_i_error = -500;
        LDR.W    R0,??DataTable15_10
        LDR.W    R1,??DataTable15_11  ;; 0xfffffe0c
        STRH     R1,[R0, #+0]
//  926     
//  927     speed+=(int16)((speed_p * speed_error + speed_i * speed_i_error + speed_d * (speed_error - speed_re_error) ) / 100 );//- speed_ed
??speed_control_1:
        LDR.W    R0,??DataTable15_12
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable15_13
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable16_1
        LDRSH    R2,[R2, #+0]
        LDR.W    R3,??DataTable15_14
        LDRH     R3,[R3, #+0]
        LDR.W    R4,??DataTable15_10
        LDRSH    R4,[R4, #+0]
        MULS     R3,R4,R3
        MLA      R1,R2,R1,R3
        LDR.W    R2,??DataTable15_15
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable16_1
        LDRSH    R3,[R3, #+0]
        LDR.W    R4,??DataTable16_2
        LDRSH    R4,[R4, #+0]
        SUBS     R3,R3,R4
        MLA      R1,R3,R2,R1
        MOVS     R2,#+100
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable15_12
        STRH     R0,[R1, #+0]
//  928      //speed=1000;
//  929     if(speed > max_speed)
        LDR.W    R0,??DataTable16_3
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable15_12
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??speed_control_2
//  930        speed = max_speed;
        LDR.W    R0,??DataTable15_12
        LDR.W    R1,??DataTable16_3
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  931     if(speed < min_speed)
??speed_control_2:
        LDR.W    R0,??DataTable15_12
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable16_4
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??speed_control_3
//  932        speed = min_speed;
        LDR.W    R0,??DataTable15_12
        LDR.W    R1,??DataTable16_4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  933     
//  934     FTM1_C0V=speed;
??speed_control_3:
        LDR.W    R0,??DataTable16_5  ;; 0x40039010
        LDR.W    R1,??DataTable15_12
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
//  935     
//  936     re_top_whiteline=top_whiteline;
        LDR.W    R0,??DataTable16_6
        LDR.W    R1,??DataTable16_7
        LDR      R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  937     speed_re_error=speed_error;
        LDR.W    R0,??DataTable16_2
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  938 }
        POP      {R4}
        BX       LR               ;; return
//  939 
//  940 
//  941 //-----------------------------------������ƺ����ı���---------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  942 void Servor_Control()
//  943 {
Servor_Control:
        PUSH     {R4-R6}
//  944    int8 i=0;
        MOVS     R3,#+0
//  945    int16 sum_server_a=0,sum_server_b=0,sum_server_c= 0,sum_server=0;
        MOVS     R2,#+0
        MOVS     R0,#+0
        MOVS     R1,#+0
        MOVS     R4,#+0
//  946    if(bottom_whitebase < top_whiteline)
        LDR.N    R5,??DataTable12
        LDR      R5,[R5, #+0]
        LDR.W    R6,??DataTable16_7
        LDR      R6,[R6, #+0]
        CMP      R5,R6
        BGE.W    ??Servor_Control_0
//  947     //��һ������Ŀ��Ʋ����������Ч��
//  948    {
//  949     for(i = bottom_whitebase;i <= ((2*bottom_whitebase+top_whiteline - 2) / 3); i++)
        LDR.N    R3,??DataTable12
        LDR      R3,[R3, #+0]
        B.N      ??Servor_Control_1
//  950     sum_server_a += center_white[i];
??Servor_Control_2:
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R4,??DataTable15
        LDRH     R4,[R4, R3, LSL #+1]
        ADDS     R2,R4,R2
        ADDS     R3,R3,#+1
??Servor_Control_1:
        LDR.N    R4,??DataTable12
        LDR      R4,[R4, #+0]
        LDR.W    R5,??DataTable16_7
        LDR      R5,[R5, #+0]
        ADDS     R4,R5,R4, LSL #+1
        SUBS     R4,R4,#+2
        MOVS     R5,#+3
        SDIV     R4,R4,R5
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R4,R3
        BGE.N    ??Servor_Control_2
//  951     sum_server_a = (int16)(15*sum_server_a/(top_whiteline-bottom_whitebase+1));
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R3,#+15
        MULS     R2,R3,R2
        LDR.W    R3,??DataTable16_7
        LDR      R3,[R3, #+0]
        LDR.N    R4,??DataTable12
        LDR      R4,[R4, #+0]
        SUBS     R3,R3,R4
        ADDS     R3,R3,#+1
        SDIV     R2,R2,R3
//  952     
//  953     for(i = ((2*bottom_whitebase + top_whiteline+1) / 3);i <= (bottom_whitebase + 2*top_whiteline - 1) / 3; i++)
        LDR.N    R3,??DataTable12
        LDR      R3,[R3, #+0]
        LDR.W    R4,??DataTable16_7
        LDR      R4,[R4, #+0]
        ADDS     R3,R4,R3, LSL #+1
        ADDS     R3,R3,#+1
        MOVS     R4,#+3
        SDIV     R3,R3,R4
        B.N      ??Servor_Control_3
//  954      sum_server_b += center_white[i];
??Servor_Control_4:
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R4,??DataTable15
        LDRH     R4,[R4, R3, LSL #+1]
        ADDS     R0,R4,R0
        ADDS     R3,R3,#+1
??Servor_Control_3:
        LDR.N    R4,??DataTable12
        LDR      R4,[R4, #+0]
        LDR.W    R5,??DataTable16_7
        LDR      R5,[R5, #+0]
        ADDS     R4,R4,R5, LSL #+1
        SUBS     R4,R4,#+1
        MOVS     R5,#+3
        SDIV     R4,R4,R5
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R4,R3
        BGE.N    ??Servor_Control_4
//  955      sum_server_b = (int16)(15*sum_server_b/(top_whiteline-bottom_whitebase+1));
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        MOVS     R3,#+15
        MULS     R0,R3,R0
        LDR.W    R3,??DataTable16_7
        LDR      R3,[R3, #+0]
        LDR.N    R4,??DataTable12
        LDR      R4,[R4, #+0]
        SUBS     R3,R3,R4
        ADDS     R3,R3,#+1
        SDIV     R0,R0,R3
//  956      
//  957      for(i = ((bottom_whitebase+2*top_whiteline+2)/3);i<=top_whiteline;i++)
        LDR.N    R3,??DataTable12
        LDR      R3,[R3, #+0]
        LDR.W    R4,??DataTable16_7
        LDR      R4,[R4, #+0]
        ADDS     R3,R3,R4, LSL #+1
        ADDS     R3,R3,#+2
        MOVS     R4,#+3
        SDIV     R3,R3,R4
        B.N      ??Servor_Control_5
//  958      sum_server_c += center_white[i];
??Servor_Control_6:
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R4,??DataTable15
        LDRH     R4,[R4, R3, LSL #+1]
        ADDS     R1,R4,R1
        ADDS     R3,R3,#+1
??Servor_Control_5:
        LDR.W    R4,??DataTable16_7
        LDR      R4,[R4, #+0]
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R4,R3
        BGE.N    ??Servor_Control_6
//  959      sum_server_c = (int16)(15*sum_server_c/(top_whiteline-bottom_whitebase+1));
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        MOVS     R3,#+15
        MULS     R1,R3,R1
        LDR.W    R3,??DataTable16_7
        LDR      R3,[R3, #+0]
        LDR.N    R4,??DataTable12
        LDR      R4,[R4, #+0]
        SUBS     R3,R3,R4
        ADDS     R3,R3,#+1
        SDIV     R1,R1,R3
//  960      
//  961     back_weight = 100-font_weight - middle_weight;
        LDR.W    R3,??DataTable16_8
        LDRB     R3,[R3, #+0]
        RSBS     R3,R3,#+100
        LDR.W    R4,??DataTable16_9
        LDRB     R4,[R4, #+0]
        SUBS     R3,R3,R4
        LDR.W    R4,??DataTable16_10
        STRB     R3,[R4, #+0]
//  962     sum_server = (int16)((font_weight * sum_server_a + middle_weight * sum_server_b + back_weight * sum_server_c) / 100);  //160cm    0.75  0.25
        LDR.W    R3,??DataTable16_8
        LDRB     R3,[R3, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        LDR.W    R4,??DataTable16_9
        LDRB     R4,[R4, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        MUL      R0,R0,R4
        MLA      R0,R2,R3,R0
        LDR.W    R2,??DataTable16_10
        LDRB     R2,[R2, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        MLA      R0,R1,R2,R0
        MOVS     R1,#+100
        SDIV     R4,R0,R1
//  963     error_server = (MID - sum_server / 5);//�õ��ڶ������Ʋ���  �������ƫ��
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+5
        SDIV     R0,R4,R0
        RSBS     R0,R0,#+80
        LDR.W    R1,??DataTable16_11
        STRH     R0,[R1, #+0]
//  964     //////////////////////////
//  965    }
//  966     ////�ڶ���errror
//  967     top_error = 0;
??Servor_Control_0:
        LDR.W    R0,??DataTable16_12
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
//  968     if(top_whiteline > 3)
        LDR.W    R0,??DataTable16_7
        LDR      R0,[R0, #+0]
        CMP      R0,#+4
        BLT.N    ??Servor_Control_7
//  969     {
//  970       for(i = top_whiteline - 3;i < top_whiteline;i++)
        LDR.W    R0,??DataTable16_7
        LDR      R0,[R0, #+0]
        SUBS     R3,R0,#+3
        B.N      ??Servor_Control_8
//  971       {
//  972         top_error += center_white[i];
??Servor_Control_9:
        LDR.W    R0,??DataTable16_12
        LDRH     R0,[R0, #+0]
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R1,??DataTable15
        LDRH     R1,[R1, R3, LSL #+1]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable16_12
        STRH     R0,[R1, #+0]
//  973       }
        ADDS     R3,R3,#+1
??Servor_Control_8:
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R0,??DataTable16_7
        LDR      R0,[R0, #+0]
        CMP      R3,R0
        BLT.N    ??Servor_Control_9
//  974     }
//  975     top_error = white_refer-top_error/3;//�õ����������Ʋ���
??Servor_Control_7:
        LDR.W    R0,??DataTable16_13
        LDR      R0,[R0, #+0]
        LDR.W    R1,??DataTable16_12
        LDRSH    R1,[R1, #+0]
        MOVS     R2,#+3
        SDIV     R1,R1,R2
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable16_12
        STRH     R0,[R1, #+0]
//  976    //////////////////////////
//  977 
//  978    if(top_whiteline >= 62)//ֱ��pd
        LDR.W    R0,??DataTable16_7
        LDR      R0,[R0, #+0]
        CMP      R0,#+62
        BLT.N    ??Servor_Control_10
//  979    {
//  980      error_servor_p = straight_p;
        LDR.W    R0,??DataTable16_14
        LDR.W    R1,??DataTable16_15
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  981      error_servor_d = straight_d;
        LDR.W    R0,??DataTable16_16
        LDR.W    R1,??DataTable16_17
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  982      ser_ref_p = 8;
        LDR.W    R0,??DataTable16_18
        MOVS     R1,#+8
        STRH     R1,[R0, #+0]
        B.N      ??Servor_Control_11
//  983    }
//  984     else if(top_whiteline < 62 && top_whiteline >= 50 )//�������pd
??Servor_Control_10:
        LDR.W    R0,??DataTable16_7
        LDR      R0,[R0, #+0]
        SUBS     R0,R0,#+50
        CMP      R0,#+12
        BCS.N    ??Servor_Control_12
//  985     {
//  986       error_servor_p = come_bow_p;
        LDR.W    R0,??DataTable16_14
        LDR.W    R1,??DataTable16_19
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  987       error_servor_d = come_bow_d;
        LDR.W    R0,??DataTable16_16
        LDR.W    R1,??DataTable16_20
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  988       ser_ref_p = 8;
        LDR.W    R0,??DataTable16_18
        MOVS     R1,#+8
        STRH     R1,[R0, #+0]
        B.N      ??Servor_Control_11
//  989     }
//  990     else   //�����pd
//  991     {
//  992       error_servor_p = bow_p;
??Servor_Control_12:
        LDR.W    R0,??DataTable16_14
        LDR.W    R1,??DataTable16_21
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  993       error_servor_d = bow_d;
        LDR.W    R0,??DataTable16_16
        LDR.W    R1,??DataTable16_22
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  994       ser_ref_p = 14;
        LDR.W    R0,??DataTable16_18
        MOVS     R1,#+14
        STRH     R1,[R0, #+0]
//  995     }
//  996     angle=(uint16)(mid_angle + (error_servor_p * error_server + error_servor_d * (error_server - re_error_server)
//  997                   +ser_ref_p * top_error + ser_ref_d*(top_error - re_top_error)) / 10);
??Servor_Control_11:
        LDR.W    R0,??DataTable16_23
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable16_14
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable16_11
        LDRSH    R2,[R2, #+0]
        LDR.W    R3,??DataTable16_16
        LDRH     R3,[R3, #+0]
        LDR.W    R4,??DataTable16_11
        LDRSH    R4,[R4, #+0]
        LDR.W    R5,??DataTable16_24
        LDRSH    R5,[R5, #+0]
        SUBS     R4,R4,R5
        MULS     R3,R4,R3
        MLA      R1,R2,R1,R3
        LDR.W    R2,??DataTable16_18
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable16_12
        LDRSH    R3,[R3, #+0]
        MLA      R1,R3,R2,R1
        LDR.W    R2,??DataTable16_25
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable16_12
        LDRSH    R3,[R3, #+0]
        LDR.W    R4,??DataTable20
        LDRSH    R4,[R4, #+0]
        SUBS     R3,R3,R4
        MLA      R1,R3,R2,R1
        MOVS     R2,#+10
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable17
        STRH     R0,[R1, #+0]
//  998     
//  999     if(angle > 1708)  
        LDR.W    R0,??DataTable17
        LDRSH    R0,[R0, #+0]
        MOVW     R1,#+1709
        CMP      R0,R1
        BLT.N    ??Servor_Control_13
// 1000       angle = 1708;
        LDR.W    R0,??DataTable17
        MOVW     R1,#+1708
        STRH     R1,[R0, #+0]
// 1001     if(angle<1000)
??Servor_Control_13:
        LDR.W    R0,??DataTable17
        LDRSH    R0,[R0, #+0]
        MOV      R1,#+1000
        CMP      R0,R1
        BGE.N    ??Servor_Control_14
// 1002       angle=1000;
        LDR.W    R0,??DataTable17
        MOV      R1,#+1000
        STRH     R1,[R0, #+0]
// 1003       FTM0_C3V=angle;
??Servor_Control_14:
        LDR.W    R0,??DataTable17_1  ;; 0x40038028
        LDR.W    R1,??DataTable17
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
// 1004 
// 1005    re_error_server=error_server;
        LDR.W    R0,??DataTable16_24
        LDR.W    R1,??DataTable16_11
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1006    top_error=re_top_error;
        LDR.W    R0,??DataTable16_12
        LDR.W    R1,??DataTable20
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1007 }
        POP      {R4-R6}
        BX       LR               ;; return
// 1008 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1009 void scan_boma(void)
// 1010 {
// 1011   uint32 temp1=0;
scan_boma:
        MOVS     R0,#+0
// 1012   uint8 temp=0;
        MOVS     R1,#+0
// 1013   GPIOD_PDOR = 0xffffffff; 
        LDR.W    R2,??DataTable17_2  ;; 0x400ff0c0
        MOVS     R3,#-1
        STR      R3,[R2, #+0]
// 1014   temp1 = GPIOD_PDIR;   //��PTD6~PTD13 
        LDR.W    R2,??DataTable17_3  ;; 0x400ff0d0
        LDR      R2,[R2, #+0]
        MOVS     R0,R2
// 1015   
// 1016   temp = !(uint8)((temp1 >> 13) & 0x00002000);
        MOVS     R2,#+1
        MOVS     R1,R2
// 1017   if(temp == 1)   //��Ӧ���ǲ���8
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+1
        BNE.N    ??scan_boma_0
// 1018      redraw_control=1;
        LDR.W    R0,??DataTable17_4
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??scan_boma_1
// 1019   else
// 1020      redraw_control=0;
??scan_boma_0:
        LDR.W    R0,??DataTable17_4
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1021   
// 1022   temp = !(uint8)((temp1 >> 12) & 0x00001000);
??scan_boma_1:
        MOVS     R1,#+1
// 1023   if(temp == 1)
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+1
        BNE.N    ??scan_boma_2
// 1024     scan_control=1;   //��Ӧ���ǲ���7
        LDR.W    R0,??DataTable20_1
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??scan_boma_3
// 1025   else
// 1026     scan_control=0;
??scan_boma_2:
        LDR.W    R0,??DataTable20_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1027   
// 1028 
// 1029 }
??scan_boma_3:
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12_1:
        DC32     left_black
// 1030 
// 1031 
// 1032 //-----------------------------�ӳ�-------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1033 void Delay_MS(uint32 ms)
// 1034 {
// 1035    while(ms--);
Delay_MS:
??Delay_MS_0:
        MOVS     R1,R0
        SUBS     R0,R1,#+1
        CMP      R1,#+0
        BNE.N    ??Delay_MS_0
// 1036 }
        BX       LR               ;; return
// 1037 
// 1038 
// 1039 /***********************************Ԥ��ʾ**********************************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1040 void pre_show(void)
// 1041 {
pre_show:
        PUSH     {R7,LR}
// 1042   LCD_CLS();
        BL       LCD_CLS
// 1043    switch(lcd_page_num)
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??pre_show_0
        CMP      R0,#+2
        BEQ.W    ??pre_show_1
        BCC.N    ??pre_show_2
        B.N      ??pre_show_3
// 1044    {        
// 1045         case 0:
// 1046             
// 1047             LCD_P6x8Str(0,0,"top_bot:");    //����  ����   //��  ��  Ҫ��ʾ����  
??pre_show_0:
        LDR.W    R2,??DataTable17_6
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 1048                    
// 1049             LCD_P6x8Str(0,1,"whi_top:");    // ����  ����  ���ĵ�
        LDR.W    R2,??DataTable17_7
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 1050                
// 1051             LCD_P6x8Str(0,2,"cc_c:");    //  ���б�� ʮ�ֵ�·  ��s��·��־
        LDR.W    R2,??DataTable17_8
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 1052                    
// 1053             LCD_P6x8Str(0,3,"spe_pi:");    // ����� pi  ����  ��speed_ed ��p
        LDR.W    R2,??DataTable18
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 1054                    
// 1055             LCD_P6x8Str(0,4,"serpd:");    //  �����pid����
        LDR.W    R2,??DataTable18_1
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 1056                    
// 1057              break;
        B.N      ??pre_show_3
// 1058              
// 1059         case 1:
// 1060              LCD_P6x8Cha(0,lcd_line_num,'*');
??pre_show_2:
        MOVS     R2,#+42
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1061              
// 1062              LCD_P6x8Str(10,0,"max_speed:");  
        LDR.W    R2,??DataTable18_3
        MOVS     R1,#+0
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1063              LCD_P6x8Num(80,0,max_speed);    //��һ��,����ٶ�
        LDR.W    R0,??DataTable16_3
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1064         
// 1065              LCD_P6x8Str(10,1,"min_speed:");     //�ڶ��У��м��ٶ�
        LDR.W    R2,??DataTable18_4
        MOVS     R1,#+1
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1066              LCD_P6x8Num(80,1,min_speed);       
        LDR.W    R0,??DataTable16_4
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1067         
// 1068              LCD_P6x8Str(10,2,"speed_set:");      //�����У���С�ٶ�
        LDR.W    R2,??DataTable18_5
        MOVS     R1,#+2
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1069              LCD_P6x8Num(80,2,speed_set);  
        LDR.W    R0,??DataTable18_6
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1070         
// 1071              LCD_P6x8Str(10,3,"speed_p:");     //�����У������p
        LDR.W    R2,??DataTable18_7
        MOVS     R1,#+3
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1072              LCD_P6x8Num(80,3,speed_p);
        LDR.N    R0,??DataTable15_13
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1073              
// 1074              LCD_P6x8Str(10,4,"speed_i:");     //�����У������i
        LDR.W    R2,??DataTable19
        MOVS     R1,#+4
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1075              LCD_P6x8Num(80,4,speed_i); 
        LDR.N    R0,??DataTable15_14
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1076              
// 1077              LCD_P6x8Str(10,5,"spe_ed_p:");     //speed_ed_p
        LDR.W    R2,??DataTable19_1
        MOVS     R1,#+5
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1078              LCD_P6x8Num(80,5,speed_ed_p); 
        LDR.W    R0,??DataTable19_2
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1079           
// 1080              break;   
        B.N      ??pre_show_3
// 1081              
// 1082              
// 1083       case 2:
// 1084              LCD_P6x8Cha(0,lcd_line_num,'*');
??pre_show_1:
        MOVS     R2,#+42
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1085              
// 1086              LCD_P6x8Str(10,0,"straight_p:");    //��һ�У������p
        LDR.W    R2,??DataTable19_3
        MOVS     R1,#+0
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1087              LCD_P6x8Num(80,0,straight_p);
        LDR.W    R0,??DataTable16_15
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1088              
// 1089              LCD_P6x8Str(10,1,"straight_d:");    //�ڶ��� �������i
        LDR.W    R2,??DataTable20_2
        MOVS     R1,#+1
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1090              LCD_P6x8Num(80,1,straight_d);  
        LDR.W    R0,??DataTable16_17
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1091             
// 1092              LCD_P6x8Str(10,2,"come_bow_p:");    //�����У���������p
        LDR.W    R2,??DataTable20_3
        MOVS     R1,#+2
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1093              LCD_P6x8Num(80,2,come_bow_p); 
        LDR.W    R0,??DataTable16_19
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1094 
// 1095              LCD_P6x8Str(10,3,"come_bow_d:");    //�����У������d
        LDR.W    R2,??DataTable20_4
        MOVS     R1,#+3
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1096              LCD_P6x8Num(80,3,come_bow_d); 
        LDR.W    R0,??DataTable16_20
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1097              
// 1098              LCD_P6x8Str(10,4,"bow_p:");    //�����У������d
        LDR.W    R2,??DataTable20_5
        MOVS     R1,#+4
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1099              LCD_P6x8Num(80,4,bow_p); 
        LDR.W    R0,??DataTable16_21
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1100              
// 1101              LCD_P6x8Str(10,5,"bow_d:");    //�����У������d
        LDR.W    R2,??DataTable20_6
        MOVS     R1,#+5
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1102              LCD_P6x8Num(80,5,bow_d); 
        LDR.W    R0,??DataTable16_22
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1103              
// 1104              LCD_P6x8Str(10,6,"fo_wei:");    //  �׶����Ȩ��
        LDR.W    R2,??DataTable20_7
        MOVS     R1,#+6
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1105              LCD_P6x8Num(80,6,font_weight); 
        LDR.W    R0,??DataTable16_8
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+6
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1106              
// 1107              LCD_P6x8Str(10,7,"mi_wei:");    //  �������Ȩ��
        LDR.W    R2,??DataTable20_8
        MOVS     R1,#+7
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1108              LCD_P6x8Num(80,7,middle_weight); 
        LDR.W    R0,??DataTable16_9
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+7
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1109              
// 1110      }
// 1111 
// 1112 }
??pre_show_3:
        POP      {R0,PC}          ;; return
// 1113 
// 1114 /*�����Ķ���
// 1115 redraw_control��������һ�������ϣ�
// 1116 ��ȫ�ֱ�����top_whiteline��bottom_whitebase��white_refer��top_white_refer��
// 1117 max_slope��cross_flag��s_flag��speed_p��speed_i��speed_ed_p��
// 1118 servor_p��speed_i��servor_d��speed��angle��OT
// 1119 
// 1120 С��Һ�����Ĳ�����
// 1121 128*64  д��6*8  ÿ�й�21���ַ�  һ������д8��------
// 1122                                               |
// 1123                                               |
// 1124                                               |
// 1125                                               |
// 1126                                               |
// 1127                                               |
// 1128                                               |
// 1129 ���ڶ�λ��x��y  x��ʾ����������������ÿһ���������㣬������ÿ����Ԫ����
// 1130 ��y������ÿ����Ԫ���Ӽ��㣬�������8
// 1131 
// 1132 */
// 1133 
// 1134 /**************************************ˢ������ʾʱ�����*********************************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1135 void redraw()
// 1136 {
redraw:
        PUSH     {R7,LR}
// 1137 
// 1138   if(lcd_page_num==0&&redraw_control==1)     //��һҳ//redraw_control��Ҫ��һ��������п���
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.W    ??redraw_0
        LDR.W    R0,??DataTable17_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??redraw_0
// 1139      {
// 1140         LCD_CLS_ROW(50,0);  //0��  8��
        MOVS     R1,#+0
        MOVS     R0,#+50
        BL       LCD_CLS_ROW
// 1141         LCD_P6x8Num(50,0,top_whiteline);    //��һ��
        LDR.W    R0,??DataTable16_7
        LDR      R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+50
        BL       LCD_P6x8Num
// 1142         LCD_P6x8Num(70,0,bottom_whitebase); 
        LDR.W    R0,??DataTable20_9
        LDR      R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+70
        BL       LCD_P6x8Num
// 1143    
// 1144         LCD_CLS_ROW(50,1); //�ڶ��У�
        MOVS     R1,#+1
        MOVS     R0,#+50
        BL       LCD_CLS_ROW
// 1145         LCD_P6x8Num(50,1,white_refer);   
        LDR.W    R0,??DataTable16_13
        LDR      R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+50
        BL       LCD_P6x8Num
// 1146         LCD_P6x8Num(70,1,top_white_refer); 
        LDR.W    R0,??DataTable20_10
        LDR      R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+70
        BL       LCD_P6x8Num
// 1147            
// 1148         LCD_CLS_ROW(50,2);//�����У�
        MOVS     R1,#+2
        MOVS     R0,#+50
        BL       LCD_CLS_ROW
// 1149         LCD_P6x8Num(50,2,cross_flag); 
        LDR.W    R0,??DataTable20_11
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+50
        BL       LCD_P6x8Num
// 1150         LCD_P6x8Num(70,2,cross_flag); 
        LDR.W    R0,??DataTable20_11
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+70
        BL       LCD_P6x8Num
// 1151  
// 1152         LCD_CLS_ROW(50,3);    //�����У��Ƕ�ֵ
        MOVS     R1,#+3
        MOVS     R0,#+50
        BL       LCD_CLS_ROW
// 1153         LCD_P6x8Num(50,3,speed_p);
        LDR.N    R0,??DataTable15_13
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+50
        BL       LCD_P6x8Num
// 1154         LCD_P6x8Num(70,3,speed_i); 
        LDR.N    R0,??DataTable15_14
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+70
        BL       LCD_P6x8Num
// 1155         
// 1156          LCD_CLS_ROW(50,4);    //�����У��ٶ��趨ֵ
        MOVS     R1,#+4
        MOVS     R0,#+50
        BL       LCD_CLS_ROW
// 1157         LCD_P6x8Num(50,4,error_servor_p);
        LDR.W    R0,??DataTable16_14
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+50
        BL       LCD_P6x8Num
// 1158         LCD_P6x8Num(70,4,error_servor_d); 
        LDR.W    R0,??DataTable16_16
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+70
        BL       LCD_P6x8Num
// 1159         
// 1160         LCD_CLS_ROW(0,5);    //��6�У�
        MOVS     R1,#+5
        MOVS     R0,#+0
        BL       LCD_CLS_ROW
// 1161         LCD_P6x8Num(0,5,speed); 
        LDR.N    R0,??DataTable15_12
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+0
        BL       LCD_P6x8Num
// 1162         LCD_P6x8Num(50,5,OT); 
        LDR.N    R0,??DataTable14
        LDR      R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+50
        BL       LCD_P6x8Num
// 1163         LCD_P6x8Num(90,5,angle);
        LDR.W    R0,??DataTable17
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+90
        BL       LCD_P6x8Num
// 1164      }  
// 1165 }
??redraw_0:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14:
        DC32     OT
// 1166 
// 1167 /*  GPIOD_PDOR = 0xffffffff; 
// 1168   temp1 = GPIOD_PDIR;   //��PTD6~PTD13   
// 1169 ����Ϊ��������Ҫ����ı���
// 1170 scan_control�������ڹܽ��ϺͲ����ϣ���
// 1171 �����ڰ����ϣ�add_page��sub_page��up_line��down_line��add_NUM��sub_NUM)��
// 1172 page_num��line_num������������������������
// 1173 */
// 1174 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1175 void key_down(void)
// 1176 {
key_down:
        PUSH     {R3-R5,LR}
// 1177   uint8 temp=0;
        MOVS     R4,#+0
// 1178   uint32 temp1=0;
        MOVS     R5,#+0
// 1179   //�˿���c8��ʼ����İ�λ����Ϊ�ߵ�ƽ
// 1180     Delay_MS(200000 * 4);
        LDR.W    R0,??DataTable21  ;; 0xc3500
        BL       Delay_MS
// 1181     GPIOC_PDOR = 0xffffffff; 
        LDR.W    R0,??DataTable21_1  ;; 0x400ff080
        MOVS     R1,#-1
        STR      R1,[R0, #+0]
// 1182     temp1 = GPIOC_PDIR;   //��PTC8~PTC15   
        LDR.W    R0,??DataTable21_2  ;; 0x400ff090
        LDR      R0,[R0, #+0]
        MOVS     R5,R0
// 1183     temp = ~(uint8)((temp1 >> 8) & 0x000000ff);
        LSRS     R0,R5,#+8
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MVNS     R0,R0
        MOVS     R4,R0
// 1184      if(temp==0x01)
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.N    ??key_down_0
// 1185        add_page=1;
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_1
// 1186      else
// 1187        add_page=0;
??key_down_0:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1188      if(temp==0x02)
??key_down_1:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.N    ??key_down_2
// 1189        sub_page=1;
        LDR.W    R0,??DataTable21_4
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_3
// 1190      else
// 1191        sub_page=0;
??key_down_2:
        LDR.W    R0,??DataTable21_4
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1192      if(temp==0x04)
??key_down_3:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+4
        BNE.N    ??key_down_4
// 1193        down_line=1;
        LDR.W    R0,??DataTable21_5
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_5
// 1194      else
// 1195        down_line=0;
??key_down_4:
        LDR.W    R0,??DataTable21_5
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1196      if(temp==0x08)
??key_down_5:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+8
        BNE.N    ??key_down_6
// 1197        up_line=1;
        LDR.W    R0,??DataTable21_6
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_7
// 1198      else
// 1199        up_line=0;
??key_down_6:
        LDR.W    R0,??DataTable21_6
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1200      if(temp==0x10)
??key_down_7:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+16
        BNE.N    ??key_down_8
// 1201        add_NUM=1;
        LDR.W    R0,??DataTable21_7
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_9
// 1202      else
// 1203        add_NUM=0;
??key_down_8:
        LDR.W    R0,??DataTable21_7
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1204      if(temp==0x20)
??key_down_9:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+32
        BNE.N    ??key_down_10
// 1205        sub_NUM=1;
        LDR.W    R0,??DataTable21_8
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_11
// 1206      else
// 1207       sub_NUM=0;
??key_down_10:
        LDR.W    R0,??DataTable21_8
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1208 }
??key_down_11:
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15:
        DC32     center_white

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_1:
        DC32     0x4006a004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_2:
        DC32     0x4006a007

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_3:
        DC32     VideoImage1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_4:
        DC32     0x13880

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_5:
        DC32     right_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_6:
        DC32     ??sci_temp

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_7:
        DC32     speed_feedback

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_8:
        DC32     speed_except

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_9:
        DC32     stopflag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_10:
        DC32     speed_i_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_11:
        DC32     0xfffffe0c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_12:
        DC32     speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_13:
        DC32     speed_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_14:
        DC32     speed_i

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_15:
        DC32     speed_d
// 1209 
// 1210 //---------------------------ȫ����ɨ��-----------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1211 void Keyscan(void)
// 1212 {
Keyscan:
        PUSH     {R7,LR}
// 1213     
// 1214    //scan_control���ò�����п��Ƶģ������Ҫ�����������������������1??????????????????????????????
// 1215     if(scan_control==1)        //ѭ��ɨ��
        LDR.W    R0,??DataTable20_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??Keyscan_0
// 1216     {//��ҳ��
// 1217       key_down();
        BL       key_down
// 1218       
// 1219       if(add_page)  //�����⵽�͵�ƽ��˵����������
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_1
// 1220       {    key_down(); //��ʱȥ����һ��10-20ms
        BL       key_down
// 1221            if(add_page)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_2
        B.N      ??Keyscan_1
// 1222            {   
// 1223               while(add_page)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
// 1224                 key_down();
??Keyscan_3:
        BL       key_down
??Keyscan_2:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_3
// 1225               if(lcd_page_num<2)    //ҳ��żӲ��������ҳ��2�ǿ��ԸĶ���??????????????????????????????????????????????????????????
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BCS.N    ??Keyscan_4
// 1226 	         lcd_page_num++;
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable17_5
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_5
// 1227 	       else
// 1228 	         lcd_page_num=0;
??Keyscan_4:
        LDR.W    R0,??DataTable17_5
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1229                lcd_line_num=0;
??Keyscan_5:
        LDR.W    R0,??DataTable18_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1230               pre_show();//��������ʾ��������ǰд������һ����д��������ĺ����壿����������������������������������
        BL       pre_show
// 1231                
// 1232            }
// 1233       }
// 1234      //��ҳ�� 
// 1235       if(sub_page)  //�����⵽�͵�ƽ��˵����������
??Keyscan_1:
        LDR.W    R0,??DataTable21_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_6
// 1236       {    key_down(); //��ʱȥ����һ��10-20ms
        BL       key_down
// 1237            if(sub_page)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
        LDR.W    R0,??DataTable21_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_7
        B.N      ??Keyscan_6
// 1238            {
// 1239               while(sub_page)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
// 1240                 key_down();
??Keyscan_8:
        BL       key_down
??Keyscan_7:
        LDR.W    R0,??DataTable21_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_8
// 1241               if(lcd_page_num>0)    //ҳ��żӲ��������ҳ��2�ǿ��ԸĶ���??????????????????????????????????????????????????????????
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BCC.N    ??Keyscan_9
// 1242 	         lcd_page_num--;
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable17_5
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_10
// 1243 	       else
// 1244 	         lcd_page_num=2;
??Keyscan_9:
        LDR.W    R0,??DataTable17_5
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
// 1245                lcd_line_num=0;
??Keyscan_10:
        LDR.W    R0,??DataTable18_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1246               pre_show();//��������ʾ��������ǰд������һ����д��������ĺ����壿����������������������������������
        BL       pre_show
// 1247                
// 1248            }
// 1249       }
// 1250       
// 1251       
// 1252      if(lcd_page_num!=0)     //�粻Ϊ��һҳ���������һ��ɨ��  
??Keyscan_6:
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??Keyscan_0
// 1253      { //��ɨ��
// 1254       //����
// 1255        key_down();
        BL       key_down
// 1256       if(up_line)  //�����⵽�͵�ƽ��˵����������
        LDR.W    R0,??DataTable21_6
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_11
// 1257       {
// 1258             key_down(); //��ʱȥ����һ��10-20ms
        BL       key_down
// 1259             if(up_line)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
        LDR.W    R0,??DataTable21_6
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_12
        B.N      ??Keyscan_11
// 1260             {
// 1261                while(up_line)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
// 1262                  key_down();
??Keyscan_13:
        BL       key_down
??Keyscan_12:
        LDR.W    R0,??DataTable21_6
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_13
// 1263                if(lcd_page_num!=0)
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_14
// 1264 	        LCD_P6x8Cha(0,lcd_line_num,' ');
        MOVS     R2,#+32
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1265                
// 1266                
// 1267                 if(lcd_line_num<LCD_ROW)    //����żӲ���??????????????????????????????????????
??Keyscan_14:
        LDR.W    R0,??DataTable18_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+7
        BCS.N    ??Keyscan_15
// 1268 	         lcd_line_num++;
        LDR.W    R0,??DataTable18_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable18_2
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_16
// 1269 		 else
// 1270 		  lcd_line_num=LCD_ROW;
??Keyscan_15:
        LDR.W    R0,??DataTable18_2
        MOVS     R1,#+7
        STRB     R1,[R0, #+0]
// 1271                 if(lcd_page_num!=0)  
??Keyscan_16:
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_11
// 1272               LCD_P6x8Cha(0,lcd_line_num,'*');
        MOVS     R2,#+42
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1273             }
// 1274       }
// 1275       //����
// 1276        if(down_line)  //�����⵽�͵�ƽ��˵����������
??Keyscan_11:
        LDR.W    R0,??DataTable21_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_17
// 1277       {
// 1278             key_down(); //��ʱȥ����һ��10-20ms
        BL       key_down
// 1279             if(down_line)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
        LDR.W    R0,??DataTable21_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_18
        B.N      ??Keyscan_17
// 1280             {
// 1281                while(down_line)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
// 1282                  key_down(); 
??Keyscan_19:
        BL       key_down
??Keyscan_18:
        LDR.W    R0,??DataTable21_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_19
// 1283                 if(lcd_page_num!=0)
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_20
// 1284 	        LCD_P6x8Cha(0,lcd_line_num,' ');
        MOVS     R2,#+32
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1285                 if(lcd_line_num>0)    //����żӲ���??????????????????????????????????????
??Keyscan_20:
        LDR.W    R0,??DataTable18_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BCC.N    ??Keyscan_21
// 1286 	         lcd_line_num--;
        LDR.W    R0,??DataTable18_2
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable18_2
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_22
// 1287 		 else
// 1288 		  lcd_line_num=0;
??Keyscan_21:
        LDR.W    R0,??DataTable18_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1289                   
// 1290               LCD_P6x8Cha(0,lcd_line_num,'*');//????????????????????????//
??Keyscan_22:
        MOVS     R2,#+42
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1291             }
// 1292       }
// 1293       
// 1294        if(add_NUM)  //�����⵽�͵�ƽ��˵����������
??Keyscan_17:
        LDR.W    R0,??DataTable21_7
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_23
// 1295     {
// 1296 	key_down(); //��ʱȥ����һ��10-20ms
        BL       key_down
// 1297      if(add_NUM)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
        LDR.W    R0,??DataTable21_7
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_24
        B.N      ??Keyscan_23
// 1298 	   {
// 1299       while(add_NUM)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
// 1300         key_down();
??Keyscan_25:
        BL       key_down
??Keyscan_24:
        LDR.W    R0,??DataTable21_7
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_25
// 1301         LCD_change_value(lcd_page_num,lcd_line_num,1);
        MOVS     R2,#+1
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        BL       LCD_change_value
// 1302 	   }
// 1303      }
// 1304       
// 1305      
// 1306      if(sub_NUM)  //�����⵽�͵�ƽ��˵����������
??Keyscan_23:
        LDR.W    R0,??DataTable21_8
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_0
// 1307     {
// 1308 	key_down(); //��ʱȥ����һ��10-20ms
        BL       key_down
// 1309      if(sub_NUM)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
        LDR.W    R0,??DataTable21_8
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_26
        B.N      ??Keyscan_0
// 1310 	   {
// 1311         while(sub_NUM)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
// 1312           key_down();
??Keyscan_27:
        BL       key_down
??Keyscan_26:
        LDR.W    R0,??DataTable21_8
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_27
// 1313      LCD_change_value(lcd_page_num,lcd_line_num,-1);
        MOVS     R2,#-1
        LDR.W    R0,??DataTable18_2
        LDRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable17_5
        LDRB     R0,[R0, #+0]
        BL       LCD_change_value
// 1314 	   }
// 1315     }
// 1316     
// 1317     
// 1318      }
// 1319    }
// 1320  }
??Keyscan_0:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16:
        DC32     send_mes

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_1:
        DC32     speed_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_2:
        DC32     speed_re_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_3:
        DC32     max_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_4:
        DC32     min_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_5:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_6:
        DC32     re_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_7:
        DC32     top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_8:
        DC32     font_weight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_9:
        DC32     middle_weight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_10:
        DC32     back_weight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_11:
        DC32     error_server

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_12:
        DC32     top_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_13:
        DC32     white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_14:
        DC32     error_servor_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_15:
        DC32     straight_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_16:
        DC32     error_servor_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_17:
        DC32     straight_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_18:
        DC32     ser_ref_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_19:
        DC32     come_bow_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_20:
        DC32     come_bow_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_21:
        DC32     bow_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_22:
        DC32     bow_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_23:
        DC32     mid_angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_24:
        DC32     re_error_server

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_25:
        DC32     ser_ref_d
// 1321 
// 1322 
// 1323 /*
// 1324 ����Ϊ��������Ҫ�����ȫ�ֱ���
// 1325 max_speed��mid_speed��min_speed��speed_p��speed_i��speed_ed_p��
// 1326 servor_p��servor_i��servor_d��fr_weight��ser_ref_p��ba_weight��
// 1327 */
// 1328 //--------------------------�ϵ�LCD���̵���---------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1329 void LCD_change_value(unsigned char page,unsigned char m,int i)
// 1330 {
LCD_change_value:
        PUSH     {R4-R6,LR}
        MOVS     R4,R0
        MOVS     R5,R1
        MOVS     R6,R2
// 1331      if(page==1)
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.W    ??LCD_change_value_0
// 1332      {
// 1333           switch(m)
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BEQ.N    ??LCD_change_value_1
        CMP      R5,#+2
        BEQ.N    ??LCD_change_value_2
        BCC.N    ??LCD_change_value_3
        CMP      R5,#+4
        BEQ.N    ??LCD_change_value_4
        BCC.N    ??LCD_change_value_5
        CMP      R5,#+5
        BEQ.W    ??LCD_change_value_6
        B.N      ??LCD_change_value_0
// 1334           {
// 1335         case 0: max_speed+=5*i;       
??LCD_change_value_1:
        LDR.W    R0,??DataTable21_9
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#+5
        MUL      R1,R1,R6
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable21_9
        STRH     R0,[R1, #+0]
// 1336                 LCD_P6x8Cha(0,0,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1337                 LCD_CLS_ROW(80,0);
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1338                 LCD_P6x8Num(80,0,max_speed);    //��һ��,����ٶ�
        LDR.W    R0,??DataTable21_9
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1339                 break;
        B.N      ??LCD_change_value_0
// 1340          case 1:min_speed+=i;
??LCD_change_value_3:
        LDR.W    R0,??DataTable21_10
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_10
        STRH     R0,[R1, #+0]
// 1341                 LCD_P6x8Cha(0,1,'*');    
        MOVS     R2,#+42
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1342                 LCD_CLS_ROW(80,1);
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1343                 LCD_P6x8Num(80,1,min_speed);
        LDR.W    R0,??DataTable21_10
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1344                 break;
        B.N      ??LCD_change_value_0
// 1345          case 2:speed_set+=10*i;
??LCD_change_value_2:
        LDR.W    R0,??DataTable18_6
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#+10
        MUL      R1,R1,R6
        ADDS     R0,R1,R0
        LDR.N    R1,??DataTable18_6
        STRH     R0,[R1, #+0]
// 1346                 LCD_P6x8Cha(0,2,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1347                 LCD_CLS_ROW(80,2);
        MOVS     R1,#+2
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1348                 LCD_P6x8Num(80,2,speed_set);
        LDR.N    R0,??DataTable18_6
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1349                 break;
        B.N      ??LCD_change_value_0
// 1350          case 3:speed_p+=10*i;
??LCD_change_value_5:
        LDR.W    R0,??DataTable21_11
        LDRH     R0,[R0, #+0]
        MOVS     R1,#+10
        MUL      R1,R1,R6
        UXTAH    R0,R1,R0
        LDR.W    R1,??DataTable21_11
        STRH     R0,[R1, #+0]
// 1351                 LCD_P6x8Cha(0,3,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1352                 LCD_CLS_ROW(80,3);
        MOVS     R1,#+3
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1353                 LCD_P6x8Num(80,3,speed_p);
        LDR.W    R0,??DataTable21_11
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1354                 break;   
        B.N      ??LCD_change_value_0
// 1355          case 4:speed_i+=i;//���ڸ�����������Ҫ����С����0.9999999��Ӱ��
??LCD_change_value_4:
        LDR.W    R0,??DataTable21_12
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_12
        STRH     R0,[R1, #+0]
// 1356                 LCD_P6x8Cha(0,4,'*');  
        MOVS     R2,#+42
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1357                 LCD_CLS_ROW(80,4);
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1358                 LCD_P6x8Num(80,4,speed_i);
        LDR.W    R0,??DataTable21_12
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1359                 break;   
        B.N      ??LCD_change_value_0
// 1360          case 5:speed_ed_p+=10*i;
??LCD_change_value_6:
        LDR.N    R0,??DataTable19_2
        LDRH     R0,[R0, #+0]
        MOVS     R1,#+10
        MUL      R1,R1,R6
        UXTAH    R0,R1,R0
        LDR.N    R1,??DataTable19_2
        STRH     R0,[R1, #+0]
// 1361                 LCD_P6x8Cha(0,5,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+5
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1362                 LCD_CLS_ROW(80,5);
        MOVS     R1,#+5
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1363                 LCD_P6x8Num(80,5,speed_ed_p);
        LDR.N    R0,??DataTable19_2
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1364                 break;
// 1365           }
// 1366      }
// 1367    if(page==2)
??LCD_change_value_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.W    ??LCD_change_value_7
// 1368    {
// 1369 switch(m)
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BEQ.N    ??LCD_change_value_8
        CMP      R5,#+2
        BEQ.N    ??LCD_change_value_9
        BCC.N    ??LCD_change_value_10
        CMP      R5,#+4
        BEQ.N    ??LCD_change_value_11
        BCC.N    ??LCD_change_value_12
        CMP      R5,#+6
        BEQ.W    ??LCD_change_value_13
        BCC.W    ??LCD_change_value_14
        CMP      R5,#+7
        BEQ.W    ??LCD_change_value_15
        B.N      ??LCD_change_value_7
// 1370         {
// 1371         case 0:straight_p+=i;       
??LCD_change_value_8:
        LDR.W    R0,??DataTable21_13
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_13
        STRH     R0,[R1, #+0]
// 1372                 LCD_P6x8Cha(0,0,'*');  
        MOVS     R2,#+42
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1373                 LCD_CLS_ROW(80,0);
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1374                 LCD_P6x8Num(80,0,straight_p); 
        LDR.W    R0,??DataTable21_13
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1375                 break;
        B.N      ??LCD_change_value_7
// 1376                 
// 1377          case 1:straight_d+=i;
??LCD_change_value_10:
        LDR.W    R0,??DataTable21_14
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_14
        STRH     R0,[R1, #+0]
// 1378                 LCD_P6x8Cha(0,1,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1379                 LCD_CLS_ROW(80,1);
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1380                 LCD_P6x8Num(80,1,straight_d); 
        LDR.W    R0,??DataTable21_14
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1381                 break;
        B.N      ??LCD_change_value_7
// 1382                 
// 1383          case 2:come_bow_p+=i;
??LCD_change_value_9:
        LDR.W    R0,??DataTable21_15
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_15
        STRH     R0,[R1, #+0]
// 1384                 LCD_P6x8Cha(0,2,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1385                 LCD_CLS_ROW(80,2);
        MOVS     R1,#+2
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1386                 LCD_P6x8Num(80,2,come_bow_p); 
        LDR.W    R0,??DataTable21_15
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1387                 break; 
        B.N      ??LCD_change_value_7
// 1388                 
// 1389          case 3:come_bow_d+=i;
??LCD_change_value_12:
        LDR.W    R0,??DataTable21_16
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_16
        STRH     R0,[R1, #+0]
// 1390                 LCD_P6x8Cha(0,3,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1391                 LCD_CLS_ROW(80,3);
        MOVS     R1,#+3
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1392                 LCD_P6x8Num(80,3,come_bow_d); 
        LDR.W    R0,??DataTable21_16
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1393                 break;
        B.N      ??LCD_change_value_7
// 1394                 
// 1395          case 4:bow_p+=i;
??LCD_change_value_11:
        LDR.W    R0,??DataTable21_17
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_17
        STRH     R0,[R1, #+0]
// 1396                 LCD_P6x8Cha(0,4,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1397                 LCD_CLS_ROW(80,4);
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1398                 LCD_P6x8Num(80,4,bow_p); 
        LDR.W    R0,??DataTable21_17
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1399                 break;    
        B.N      ??LCD_change_value_7
// 1400          case 5:bow_d+=i;
??LCD_change_value_14:
        LDR.W    R0,??DataTable21_18
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_18
        STRH     R0,[R1, #+0]
// 1401                 LCD_P6x8Cha(0,5,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+5
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1402                 LCD_CLS_ROW(80,5);
        MOVS     R1,#+5
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1403                 LCD_P6x8Num(80,5,bow_d); 
        LDR.W    R0,??DataTable21_18
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1404                 break;   
        B.N      ??LCD_change_value_7
// 1405                 
// 1406          case 6:font_weight+=i;
??LCD_change_value_13:
        LDR.W    R0,??DataTable21_19
        LDRB     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_19
        STRB     R0,[R1, #+0]
// 1407                 LCD_P6x8Cha(0,6,'*');
        MOVS     R2,#+42
        MOVS     R1,#+6
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1408                 LCD_CLS_ROW(80,6);
        MOVS     R1,#+6
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1409                 LCD_P6x8Num(80,6,font_weight); 
        LDR.W    R0,??DataTable21_19
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+6
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1410                 break;  
        B.N      ??LCD_change_value_7
// 1411          case 7:middle_weight+=i;
??LCD_change_value_15:
        LDR.W    R0,??DataTable21_20
        LDRB     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable21_20
        STRB     R0,[R1, #+0]
// 1412                 LCD_P6x8Cha(0,7,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+7
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1413                 LCD_CLS_ROW(80,7);
        MOVS     R1,#+7
        MOVS     R0,#+80
        BL       LCD_CLS_ROW
// 1414                 LCD_P6x8Num(80,7,middle_weight);
        LDR.W    R0,??DataTable21_20
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+7
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 1415                 break;
// 1416          
// 1417 	      }
// 1418    }
// 1419      
// 1420      
// 1421      
// 1422 }
??LCD_change_value_7:
        POP      {R4-R6,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17:
        DC32     angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_1:
        DC32     0x40038028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_2:
        DC32     0x400ff0c0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_3:
        DC32     0x400ff0d0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_4:
        DC32     redraw_control

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_5:
        DC32     lcd_page_num

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_6:
        DC32     `?<Constant "top_bot:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_7:
        DC32     `?<Constant "whi_top:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_8:
        DC32     `?<Constant "cc_c:">`
// 1423 
// 1424 
// 1425 //-------------------------------------������������˿�---------------------------------------------------//
// 1426 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1427 void PORT_Init(void)
// 1428 {
// 1429     //����PORTA pin14,pin15,pin16,pin17ΪGPIO�� ��Ϊָʾ����
// 1430    // PORTA_PCR14=(0|PORT_PCR_MUX(1));
// 1431   //  PORTA_PCR15=(0|PORT_PCR_MUX(1)); 
// 1432    // PORTA_PCR16=(0|PORT_PCR_MUX(1));
// 1433   //  PORTA_PCR17=(0|PORT_PCR_MUX(1)); 
// 1434     
// 1435     PORTC_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//PTC1��������ΪGPIOģʽ ����
PORT_Init:
        LDR.W    R0,??DataTable21_21  ;; 0x4004b004
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1436     
// 1437     PORTE_PCR0 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E4��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_22  ;; 0x4004d000
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1438     PORTE_PCR1 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E5��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_23  ;; 0x4004d004
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1439     PORTE_PCR2 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E6��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_24  ;; 0x4004d008
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1440     PORTE_PCR3 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E7��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_25  ;; 0x4004d00c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1441     PORTE_PCR4 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E8��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_26  ;; 0x4004d010
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1442     PORTE_PCR5 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E9��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_27  ;; 0x4004d014
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1443     PORTE_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E10��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_28  ;; 0x4004d018
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1444     PORTE_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E11��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_29  ;; 0x4004d01c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1445     
// 1446     
// 1447     //����PORTA pin14,pin15��pin16,pin17
// 1448 	///GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17));
// 1449 	//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17)); //�ȶ�ȡ��Ȼ��������
// 1450         
// 1451         GPIOE_PDDR = 0xffffff00;  //E0~E7����Ϊ����� 
        LDR.W    R0,??DataTable21_30  ;; 0x400ff114
        MVNS     R1,#+255
        STR      R1,[R0, #+0]
// 1452 
// 1453         GPIOC_PDDR = 0xfffffffd;  //PTC1����Ϊ����
        LDR.W    R0,??DataTable21_31  ;; 0x400ff094
        MVNS     R1,#+2
        STR      R1,[R0, #+0]
// 1454         
// 1455      //   GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(15));	//IO������ߵ�ƽ����	     
// 1456    //     GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17));	//IO������ߵ�ƽ����
// 1457         
// 1458         
// 1459          PORTD_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D6��������ΪGPIOģʽ   //���뿪��
        LDR.W    R0,??DataTable21_32  ;; 0x4004c018
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1460          PORTD_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D7��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_33  ;; 0x4004c01c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1461          PORTD_PCR8 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D8��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_34  ;; 0x4004c020
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1462          PORTD_PCR9 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D9��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_35  ;; 0x4004c024
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1463          PORTD_PCR10 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D10��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_36  ;; 0x4004c028
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1464          PORTD_PCR11 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D11��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_37  ;; 0x4004c02c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1465          PORTD_PCR12 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D12��������ΪGPIOģʽ
        LDR.W    R0,??DataTable21_38  ;; 0x4004c030
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1466          PORTD_PCR13 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D13��������ΪGPIOģʽ 
        LDR.W    R0,??DataTable21_39  ;; 0x4004c034
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 1467    
// 1468          GPIOD_PDDR = 0xffffc03f;       
        LDR.W    R0,??DataTable21_40  ;; 0x400ff0d4
        MVNS     R1,#+16320
        STR      R1,[R0, #+0]
// 1469 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18:
        DC32     `?<Constant "spe_pi:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_1:
        DC32     `?<Constant "serpd:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_2:
        DC32     lcd_line_num

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_3:
        DC32     `?<Constant "max_speed:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_4:
        DC32     `?<Constant "min_speed:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_5:
        DC32     `?<Constant "speed_set:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_6:
        DC32     speed_set

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_7:
        DC32     `?<Constant "speed_p:">`
// 1470 
// 1471 //---------------------------------------------------������------------------------------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 1472 void main(void)
// 1473 {
main:
        PUSH     {R7,LR}
// 1474     DisableInterrupts;
        CPSID i         
// 1475     pllinit180M(); 
        BL       pllinit180M
// 1476     
// 1477     LCD_IO_Init();
        BL       LCD_IO_Init
// 1478     LCD_Init(); 
        BL       LCD_Init
// 1479     
// 1480     PORT_Init();              //�˿ڳ�ʼ��
        BL       PORT_Init
// 1481     hw_FTM_init();
        BL       hw_FTM_init
// 1482     UART0_Init();             //���ڳ�ʼ��   
        BL       UART0_Init
// 1483     LPTMR_Init();             //�����������ʼ��
        BL       LPTMR_Init
// 1484     EXIT_Init();
        BL       EXIT_Init
// 1485     Delay_MS(40000000);   //�����ӳ� uint8 ch=3;
        LDR.N    R0,??DataTable21_41  ;; 0x2625a00
        BL       Delay_MS
// 1486     OddEvenStatus = ODD_EVEN_STATUS;
        LDR.N    R0,??DataTable21_2  ;; 0x400ff090
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        ANDS     R0,R0,#0x1
        LDR.N    R1,??DataTable21_42
        STRB     R0,[R1, #+0]
// 1487     VIF = VIF_START;
        LDR.N    R0,??DataTable21_43
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1488  
// 1489     enable_irq(45);           //�򿪴����ж�
        MOVS     R0,#+45
        BL       enable_irq
// 1490     enable_irq(89);           //�����ж�   
        MOVS     R0,#+89
        BL       enable_irq
// 1491     EnableInterrupts;
        CPSIE i         
// 1492      scan_boma();
        BL       scan_boma
// 1493      pre_show();
        BL       pre_show
// 1494 
// 1495     while(1)
// 1496     {      
// 1497        if(ImageReady)                                         //ͼ��׼���ã��پ���
??main_0:
        LDR.N    R0,??DataTable21_44
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_0
// 1498        {                                                                                                                
// 1499           Initial(); 
        BL       Initial
// 1500           Search_WhiteBase();
        BL       Search_WhiteBase
// 1501           Search_BlackEdge();
        BL       Search_BlackEdge
// 1502           Deal_BlackEdge();
        BL       Deal_BlackEdge
// 1503           SCI0_send_mesage(); 
        BL       SCI0_send_mesage
// 1504 
// 1505           scan_boma();
        BL       scan_boma
// 1506           Keyscan();
        BL       Keyscan
// 1507           redraw();//ˢ����ʾ�� 
        BL       redraw
// 1508           Servor_Control();
        BL       Servor_Control
// 1509           while(ImageReady);
??main_1:
        LDR.N    R0,??DataTable21_44
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??main_1
        B.N      ??main_0
// 1510        }
// 1511     }  
// 1512    
// 1513 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19:
        DC32     `?<Constant "speed_i:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_1:
        DC32     `?<Constant "spe_ed_p:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_2:
        DC32     speed_ed_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_3:
        DC32     `?<Constant "straight_p:">`
// 1514   
// 1515 
// 1516 //-----------------------------�жϺ���-----------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 1517 void uart0_isr(void)          //�����ж�
// 1518 {    
// 1519     DisableInterrupts;   // �����ж�Ҳ���ԣ������и��߼��жϴ����ﲻ�Ƽ�
uart0_isr:
        CPSID i         
// 1520       uint8 ch;
// 1521      while(!(UART0_S1&UART_S1_RDRF_MASK));
??uart0_isr_0:
        LDR.N    R0,??DataTable21_45  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+26
        BPL.N    ??uart0_isr_0
// 1522       ch = UART0_D;
        LDR.N    R0,??DataTable21_46  ;; 0x4006a007
        LDRB     R0,[R0, #+0]
// 1523       if(ch == '1')     //���͵���ԭʼͼ��
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+49
        BNE.N    ??uart0_isr_1
// 1524         send_mes = 1; 
        LDR.N    R0,??DataTable21_47
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 1525       else if(ch == '2')  
??uart0_isr_1:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+50
        BNE.N    ??uart0_isr_3
// 1526         send_mes = 2;
        LDR.N    R0,??DataTable21_47
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 1527       else if(ch == '3')  //�ٶ�ͼ��
??uart0_isr_3:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+51
        BNE.N    ??uart0_isr_4
// 1528         send_mes = 3;
        LDR.N    R0,??DataTable21_47
        MOVS     R1,#+3
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 1529       else if (ch >= 64 && ch <= 65)   //�䵵����
??uart0_isr_4:
        SUBS     R1,R0,#+64
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+2
        BCS.N    ??uart0_isr_2
// 1530       {   
// 1531           switch(ch - 64)  //�䵵����
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R0,R0,#+64
        CMP      R0,#+0
        BEQ.N    ??uart0_isr_5
        CMP      R0,#+1
        BEQ.N    ??uart0_isr_6
        B.N      ??uart0_isr_7
// 1532           {
// 1533           case 0:   send_mes='s';break;//stopͣ��
??uart0_isr_5:
        LDR.N    R0,??DataTable21_47
        MOVS     R1,#+115
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 1534           case 1:   send_mes='p';break;
??uart0_isr_6:
        LDR.N    R0,??DataTable21_47
        MOVS     R1,#+112
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 1535           default: speed = 100;
??uart0_isr_7:
        LDR.N    R0,??DataTable21_48
        MOVS     R1,#+100
        STRH     R1,[R0, #+0]
// 1536           }
// 1537       }
// 1538 
// 1539     EnableInterrupts;
??uart0_isr_2:
        CPSIE i         
// 1540 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20:
        DC32     re_top_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_1:
        DC32     scan_control

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_2:
        DC32     `?<Constant "straight_d:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_3:
        DC32     `?<Constant "come_bow_p:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_4:
        DC32     `?<Constant "come_bow_d:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_5:
        DC32     `?<Constant "bow_p:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_6:
        DC32     `?<Constant "bow_d:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_7:
        DC32     `?<Constant "fo_wei:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_8:
        DC32     `?<Constant "mi_wei:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_9:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_10:
        DC32     top_white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_11:
        DC32     cross_flag
// 1541 
// 1542 //------------------------------------ͼ��ɼ��ж�----------------------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 1543 void PTC_isr(void)//���ڳ��ж�20ms�����ж�63us��������ͷ�Ĺ��е�Ƶ�ʣ�����Ҫ�����ȥ��ʱ
// 1544 {
PTC_isr:
        PUSH     {R4,LR}
// 1545     uint16 i;  
// 1546     if (LineCount % 30 == 15)            
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        MOVS     R1,#+30
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        CMP      R0,#+15
        BNE.N    ??PTC_isr_0
// 1547     {
// 1548         speed_feedback = LPTMR0_CNR;                  //����������ֵ
        LDR.N    R0,??DataTable21_50
        LDR.N    R1,??DataTable21_51  ;; 0x4004000c
        LDR      R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1549         if (LineCount == 15)                         //�ڵ�һ�ζ���������ֵʱ������һ�ζ�ֵ�������һ������ʱ�䣬��Ҫ����һ��ϵ��
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        CMP      R0,#+15
        BNE.N    ??PTC_isr_1
// 1550           speed_feedback /= 1.2;
        LDR.N    R0,??DataTable21_50
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2d
        MOVS     R2,#+858993459
        LDR.N    R3,??DataTable21_52  ;; 0x3ff33333
        BL       __aeabi_ddiv
        BL       __aeabi_d2iz
        LDR.N    R1,??DataTable21_50
        STRH     R0,[R1, #+0]
// 1551         LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;
??PTC_isr_1:
        LDR.N    R0,??DataTable21_53  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.N    R1,??DataTable21_53  ;; 0x40040000
        STR      R0,[R1, #+0]
// 1552         LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;                //������������
        LDR.N    R0,??DataTable21_53  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.N    R1,??DataTable21_53  ;; 0x40040000
        STR      R0,[R1, #+0]
// 1553         if(stopflag==0)
        LDR.N    R0,??DataTable21_54
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??PTC_isr_2
// 1554          speed_control();
        BL       speed_control
        B.N      ??PTC_isr_0
// 1555          else 
// 1556            FTM1_C0V=0;     
??PTC_isr_2:
        LDR.N    R0,??DataTable21_55  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 1557     }
// 1558 
// 1559    PORTC_PCR3|=PORT_PCR_ISF_MASK;  //����жϱ�־λ
??PTC_isr_0:
        LDR.N    R0,??DataTable21_56  ;; 0x4004b00c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.N    R1,??DataTable21_56  ;; 0x4004b00c
        STR      R0,[R1, #+0]
// 1560     if (VIF == VIF_START)                              //��ʼ������־
        LDR.N    R0,??DataTable21_43
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??PTC_isr_3
// 1561       {
// 1562         LineCount++;
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable21_49
        STRB     R0,[R1, #+0]
// 1563         if(OddEvenStatus != ODD_EVEN_STATUS)
        LDR.N    R0,??DataTable21_42
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable21_2  ;; 0x400ff090
        LDR      R1,[R1, #+0]
        LSRS     R1,R1,#+1
        ANDS     R1,R1,#0x1
        CMP      R0,R1
        BEQ.N    ??PTC_isr_4
// 1564         {
// 1565           OddEvenStatus = ODD_EVEN_STATUS;	//��ż����־
        LDR.N    R0,??DataTable21_2  ;; 0x400ff090
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        ANDS     R0,R0,#0x1
        LDR.N    R1,??DataTable21_42
        STRB     R0,[R1, #+0]
// 1566           VIF = VIF_WAITSAMPLE;   		//��һ��״̬Ϊ�ȴ�����
        LDR.N    R0,??DataTable21_43
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1567           VideoImageLine = 0;
        LDR.N    R0,??DataTable21_57
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1568           LineCount = 0;
        LDR.N    R0,??DataTable21_49
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1569           ImageReady = 0; 
        LDR.N    R0,??DataTable21_44
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??PTC_isr_4
// 1570         }
// 1571       }
// 1572     else if (VIF == VIF_WAITSAMPLE)                 //�ȴ�����,��ʱ��ȥVIDEO_START_LINE��
??PTC_isr_3:
        LDR.N    R0,??DataTable21_43
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??PTC_isr_5
// 1573       {
// 1574           LineCount++;
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable21_49
        STRB     R0,[R1, #+0]
// 1575           if (LineCount >= VIDEO_START_LINE)
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        CMP      R0,#+24
        BCC.N    ??PTC_isr_4
// 1576           {
// 1577               VIF = VIF_SAMPLELINE;                 //��һ��״̬Ϊ����״̬
        LDR.N    R0,??DataTable21_43
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
        B.N      ??PTC_isr_4
// 1578           }   	
// 1579       }
// 1580     else if (VIF == VIF_SAMPLELINE)              //��ʼ����
??PTC_isr_5:
        LDR.N    R0,??DataTable21_43
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??PTC_isr_4
// 1581       {
// 1582           LineCount++;
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable21_49
        STRB     R0,[R1, #+0]
// 1583           if (LineCount % 3== 0)   //ÿ��һ�в�һ��
        LDR.N    R0,??DataTable21_49
        LDRB     R0,[R0, #+0]
        MOVS     R1,#+3
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        CMP      R0,#+0
        BNE.N    ??PTC_isr_6
// 1584           {
// 1585               for (i = 0; i < COLUMN+PIANYI ; i++)        //ÿ��ɨ��COLUMN+PIANYI����(����PIANYI������Ҫ���޳�������Ϊ����������)
        MOVS     R4,#+0
        B.N      ??PTC_isr_7
// 1586              {
// 1587                   if (i >=PIANYI )
??PTC_isr_8:
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        CMP      R4,#+172
        BCC.N    ??PTC_isr_9
// 1588                    {//�ɼ��ĵ�һ�������������ʵ�������������½ǣ������������д洢�ڵ�һ�е����һ��λ��
// 1589                      VideoImage1[VideoImageLine][COLUMN-1-i+PIANYI] = (uint8)(0x000000ff & GPIOE_PDIR);//���ɼ����ĵ�ֱ�ӷ��뵽VideoImage2[][]����init array�����зŵ�VideoImage1[][]��������
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        RSBS     R0,R4,#+0
        LDR.N    R1,??DataTable21_57
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+161
        LDR.N    R3,??DataTable21_58
        MLA      R1,R2,R1,R3
        ADDS     R0,R0,R1
        LDR.N    R1,??DataTable21_59  ;; 0x400ff110
        LDR      R1,[R1, #+0]
        STRB     R1,[R0, #+332]
// 1590                          Delay_MS(3); 
        MOVS     R0,#+3
        BL       Delay_MS
// 1591                         asm("nop");
        nop              
// 1592                         asm("nop");//�����ʱ
        nop              
// 1593                   }
// 1594               }
??PTC_isr_9:
        ADDS     R4,R4,#+1
??PTC_isr_7:
        MOVW     R0,#+333
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        CMP      R4,R0
        BCC.N    ??PTC_isr_8
// 1595              VideoImageLine++;
        LDR.N    R0,??DataTable21_57
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable21_57
        STRB     R0,[R1, #+0]
// 1596           }
// 1597           if (VideoImageLine == ROW)      //�ɼ����������趨������
??PTC_isr_6:
        LDR.N    R0,??DataTable21_57
        LDRB     R0,[R0, #+0]
        CMP      R0,#+65
        BNE.N    ??PTC_isr_4
// 1598           {
// 1599               ImageReady = 1;           //ͼ��׼����
        LDR.N    R0,??DataTable21_44
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1600               VIF = VIF_START;
        LDR.N    R0,??DataTable21_43
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1601           }
// 1602           
// 1603   }
// 1604 }
??PTC_isr_4:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21:
        DC32     0xc3500

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_1:
        DC32     0x400ff080

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_2:
        DC32     0x400ff090

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_3:
        DC32     add_page

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_4:
        DC32     sub_page

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_5:
        DC32     down_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_6:
        DC32     up_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_7:
        DC32     add_NUM

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_8:
        DC32     sub_NUM

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_9:
        DC32     max_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_10:
        DC32     min_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_11:
        DC32     speed_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_12:
        DC32     speed_i

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_13:
        DC32     straight_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_14:
        DC32     straight_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_15:
        DC32     come_bow_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_16:
        DC32     come_bow_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_17:
        DC32     bow_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_18:
        DC32     bow_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_19:
        DC32     font_weight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_20:
        DC32     middle_weight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_21:
        DC32     0x4004b004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_22:
        DC32     0x4004d000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_23:
        DC32     0x4004d004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_24:
        DC32     0x4004d008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_25:
        DC32     0x4004d00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_26:
        DC32     0x4004d010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_27:
        DC32     0x4004d014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_28:
        DC32     0x4004d018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_29:
        DC32     0x4004d01c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_30:
        DC32     0x400ff114

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_31:
        DC32     0x400ff094

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_32:
        DC32     0x4004c018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_33:
        DC32     0x4004c01c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_34:
        DC32     0x4004c020

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_35:
        DC32     0x4004c024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_36:
        DC32     0x4004c028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_37:
        DC32     0x4004c02c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_38:
        DC32     0x4004c030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_39:
        DC32     0x4004c034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_40:
        DC32     0x400ff0d4

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_41:
        DC32     0x2625a00

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_42:
        DC32     OddEvenStatus

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_43:
        DC32     Videoclo_Flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_44:
        DC32     ImageReady

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_45:
        DC32     0x4006a004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_46:
        DC32     0x4006a007

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_47:
        DC32     send_mes

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_48:
        DC32     speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_49:
        DC32     LineCount

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_50:
        DC32     speed_feedback

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_51:
        DC32     0x4004000c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_52:
        DC32     0x3ff33333

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_53:
        DC32     0x40040000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_54:
        DC32     stopflag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_55:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_56:
        DC32     0x4004b00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_57:
        DC32     VideoImageLine

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_58:
        DC32     VideoImage1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_59:
        DC32     0x400ff110

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "top_bot:">`:
        DATA
        DC8 "top_bot:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "whi_top:">`:
        DATA
        DC8 "whi_top:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "cc_c:">`:
        DATA
        DC8 "cc_c:"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "spe_pi:">`:
        DATA
        DC8 "spe_pi:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "serpd:">`:
        DATA
        DC8 "serpd:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "max_speed:">`:
        DATA
        DC8 "max_speed:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "min_speed:">`:
        DATA
        DC8 "min_speed:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "speed_set:">`:
        DATA
        DC8 "speed_set:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "speed_p:">`:
        DATA
        DC8 "speed_p:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "speed_i:">`:
        DATA
        DC8 "speed_i:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "spe_ed_p:">`:
        DATA
        DC8 "spe_ed_p:"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "straight_p:">`:
        DATA
        DC8 "straight_p:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "straight_d:">`:
        DATA
        DC8 "straight_d:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "come_bow_p:">`:
        DATA
        DC8 "come_bow_p:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "come_bow_d:">`:
        DATA
        DC8 "come_bow_d:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "bow_p:">`:
        DATA
        DC8 "bow_p:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "bow_d:">`:
        DATA
        DC8 "bow_d:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "fo_wei:">`:
        DATA
        DC8 "fo_wei:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "mi_wei:">`:
        DATA
        DC8 "mi_wei:"

        END
// 1605  
// 
// 10 948 bytes in section .bss
//    175 bytes in section .data
//    200 bytes in section .rodata
// 11 194 bytes in section .text
// 
// 11 194 bytes of CODE  memory
//    200 bytes of CONST memory
// 11 123 bytes of DATA  memory
//
//Errors: none
//Warnings: none
