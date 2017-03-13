///////////////////////////////////////////////////////////////////////////////
//                                                                            /
//                                                      16/Mar/2014  15:20:25 /
// IAR ANSI C/C++ Compiler V6.30.4.23288/W32 EVALUATION for ARM               /
// Copyright 1999-2011 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\src\Sources\C /
//                    \main.c                                                 /
//    Command line =  "F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\src\Sources\ /
//                    C\main.c" -D IAR -D TWR_K60N512 -lCN "F:\刘友才         /
//                    (H)\飞思卡尔\程思遥\wust4华南赛\bin\Flash\List\" -lB    /
//                    "F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\bin\Flash\Li /
//                    st\" -o "F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\bin\ /
//                    Flash\Obj\" --no_cse --no_unroll --no_inline            /
//                    --no_code_motion --no_tbaa --no_clustering              /
//                    --no_scheduling --debug --endian=little                 /
//                    --cpu=Cortex-M4 -e --fpu=None --dlib_config             /
//                    E:\anzhuangbao\K60\arm\INC\c\DLib_Config_Normal.h -I    /
//                    "F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\src\Sources\ /
//                    H\" -I "F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\src\S /
//                    ources\H\Component_H\" -I "F:\刘友才                    /
//                    (H)\飞思卡尔\程思遥\wust4华南赛\src\Sources\H\Frame_H\" /
//                     -Ol --use_c++_inline                                   /
//    List file    =  F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\bin\Flash\Lis /
//                    t\main.s                                                /
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
        EXTERN __aeabi_cfcmpeq
        EXTERN __aeabi_cfcmple
        EXTERN __aeabi_cfrcmple
        EXTERN __aeabi_d2f
        EXTERN __aeabi_ddiv
        EXTERN __aeabi_dmul
        EXTERN __aeabi_f2d
        EXTERN __aeabi_fadd
        EXTERN __aeabi_fdiv
        EXTERN __aeabi_fmul
        EXTERN __aeabi_i2d
        EXTERN __aeabi_i2f
        EXTERN __aeabi_ui2f
        EXTERN enable_irq
        EXTERN sqrt

        PUBLIC BASE_OT
        PUBLIC Control
        PUBLIC Deal_BlackEdge
        PUBLIC Delay_MS
        PUBLIC EXIT_Init
        PUBLIC ImageReady
        PUBLIC Initial
        PUBLIC Keyscan
        PUBLIC LCD_change_value
        PUBLIC LPTMR_Init
        PUBLIC LineCount
        PUBLIC MIN_INT
        PUBLIC OT
        PUBLIC OddEvenStatus
        PUBLIC PORT_Init
        PUBLIC PTB_isr
        PUBLIC Row_state
        PUBLIC SCI0_send_mesage
        PUBLIC S_left
        PUBLIC S_right
        PUBLIC S_straight
        PUBLIC Search_BlackEdge
        PUBLIC Search_WhiteBase
        PUBLIC UART0_Init
        PUBLIC VideoImage1
        PUBLIC VideoImage2
        PUBLIC VideoImageLine
        PUBLIC Videoclo_Flag
        PUBLIC WHITE_BLACK_OT
        PUBLIC add_NUM
        PUBLIC angle
        PUBLIC bottom_whitebase
        PUBLIC bow_speed
        PUBLIC car_test_run
        PUBLIC center_average
        PUBLIC center_error_average
        PUBLIC center_linear_average
        PUBLIC center_lost_hang
        PUBLIC center_white
        PUBLIC change_page
        PUBLIC check_start_stop_line
        PUBLIC control_top_whiteline
        PUBLIC current_deal_line
        PUBLIC danger_count
        PUBLIC danger_flag
        PUBLIC dead_stop
        PUBLIC deal_start_line
        PUBLIC direction
        PUBLIC down_line
        PUBLIC error_servo_d
        PUBLIC error_servo_p
        PUBLIC error_servo_ref_d
        PUBLIC error_servo_ref_p
        PUBLIC f_abs16
        PUBLIC f_absf
        PUBLIC find_whitebase_flag
        PUBLIC full_speed_line
        PUBLIC get_line_information
        PUBLIC get_linear_factor
        PUBLIC get_p_errorline
        PUBLIC hang_search_start
        PUBLIC hw_FTM_init
        PUBLIC key_down
        PUBLIC lcd_bow_speed
        PUBLIC lcd_debug
        PUBLIC lcd_error_servo_d
        PUBLIC lcd_error_servo_p
        PUBLIC lcd_line_num
        PUBLIC lcd_page_num
        PUBLIC lcd_ref_d
        PUBLIC lcd_ref_p
        PUBLIC lcd_straight_speed
        PUBLIC lcd_straight_speed_ed
        PUBLIC left_black
        PUBLIC left_top_whiteline
        PUBLIC left_whitebase_searchstart
        PUBLIC linear_factor
        PUBLIC main
        PUBLIC max_speed
        PUBLIC mid_angle
        PUBLIC min_speed
        PUBLIC mySpeedswitch
        PUBLIC p_error
        PUBLIC p_re_error
        PUBLIC pllinit180M
        PUBLIC pre_show
        PUBLIC ramp_delay_time
        PUBLIC ramp_dis_flag
        PUBLIC ramp_dis_time
        PUBLIC ramp_flag
        PUBLIC ramp_speed
        PUBLIC ramp_time
        PUBLIC re_angle
        PUBLIC re_control_top_whiteline
        PUBLIC re_direction
        PUBLIC re_refer_error
        PUBLIC re_top_whiteline
        PUBLIC re_white_refer
        PUBLIC re_whitepoint_end
        PUBLIC re_whitepoint_start
        PUBLIC redraw
        PUBLIC redraw_control
        PUBLIC ref_his_error
        PUBLIC refer_error
        PUBLIC refer_road_width
        PUBLIC right_black
        PUBLIC right_top_whiteline
        PUBLIC right_whitebase_searchstart
        PUBLIC scan_boma
        PUBLIC se_sub_NUM
        PUBLIC send_mes
        PUBLIC speed
        PUBLIC speed_down_cnt
        PUBLIC speed_error
        PUBLIC speed_except
        PUBLIC speed_feedback
        PUBLIC speed_i
        PUBLIC speed_p
        PUBLIC speed_re_error
        PUBLIC speed_select
        PUBLIC start_stop_count
        PUBLIC start_stop_cs
        PUBLIC start_stop_en
        PUBLIC stop_pit_count
        PUBLIC stopflag
        PUBLIC straight_count
        PUBLIC straight_speed
        PUBLIC straight_speed_ed
        PUBLIC sub_NUM
        PUBLIC test_run
        PUBLIC top_error_servo_p
        PUBLIC top_whiteline
        PUBLIC uart0_isr
        PUBLIC up_line
        PUBLIC white_refer
        PUBLIC whitebase_searchstart
        PUBLIC whitepoint_end
        PUBLIC whitepoint_start

// F:\刘友才 (H)\飞思卡尔\程思遥\wust4华南赛\src\Sources\C\main.c
//    1 /*程序说明  */
//    2 //使程序更加的精简高效准确，
//    3 #include "MK60N512VMD100.h " /* include peripheral declarations */
//    4 #include "includes.h"
//    5 #include <math.h>
//    6 #include"LCDDriver.h"
//    7 
//    8 #define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
//    9 #define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1
//   10 #define BUS_CLOCK  100  //(MHZ)50 82 90 100 105 110 115//这里设置的内核时钟等于总线时钟100M
//   11 #define BAUD 19200     //波特率
//   12 #define CORE_CLOCK 180
//   13 
//   14 //--------------------------采集图像的相关变量-------------------------------------//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 bool     OddEvenStatus;		  //奇偶场状态标志
OddEvenStatus:
        DS8 1
//   16 #define  OddStatus  0
//   17 #define  EvenStatus 1
//   18 #define  ODD_EVEN_STATUS  (bool)(0x00000001 & (GPIOB_PDIR  >> 20))  //奇偶变换标志  将第ptc端口的第1位右移动后，置1
//   19 #define VIF_START	0   	 //	开始模式				 
//   20 #define VIF_WAITSAMPLE	1        //   等待模式
//   21 #define VIF_SAMPLELINE	2         //   除去消隐行的状态
//   22 #define VIF Videoclo_Flag         //
//   23 #define PIANYI 150   //实际采集列数为COLUMN + PIANYI，PIANYI为每行消隐点  中心值大于中值，消隐行减少
//   24 #define VIDEO_START_LINE  27	//图像采集起始行

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   25 bool ImageReady;               //图像准备好标志
ImageReady:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   26 uint8 Videoclo_Flag, VideoImageLine;   //采集状态标志位，行中断实际采集行数计数器
Videoclo_Flag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
VideoImageLine:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   27 uint16 LineCount;                       //行中断采集行数计数器   这个数据一定是要定义为uint16  自己以前定义为uint8  伤心痛苦折磨了好几天
LineCount:
        DS8 2
//   28 
//   29 
//   30 //-------------------------处理图像的相关变量-----------------------------------------//
//   31 #define ROW 65	                 //采集行数
//   32 #define COLUMN	159 		//每行点数
//   33 #define MID  79                 //列中心 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   34 uint8 VideoImage1[ROW][COLUMN] =       //原始图像数组[0][0]在左下角
VideoImage1:
        DS8 10336
//   35 {
//   36    0
//   37 };

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   38 uint8 VideoImage2[ROW][COLUMN] =       //原始图像数组[0][0]在左下角
VideoImage2:
        DS8 10336
//   39 {
//   40    0
//   41 };
//   42 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   43 uint8 left_black[ROW]=                 //左边沿线的采集数组
left_black:
        DS8 68
//   44 {
//   45   0
//   46 };

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   47 uint8 right_black[ROW]=                //右边沿线的采集数组
right_black:
        DS8 68
//   48 {
//   49   0
//   50 };

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   51 uint8 center_white[ROW]=              //（虚拟出来的）中线的数组
center_white:
        DS8 68
//   52 {
//   53   0
//   54 };
//   55 
//   56 
//   57 //-------------------------------------搜两边黑线----------------------------------//
//   58 #define MIN_WHITEBASE_POINT 30                    //最少连续白点个数成为基准的要求
//   59 #define WHITE_TOP_WHITELINE_POINT 20                  //两边的黑线的宽度小于这个值，判定为最高有效
//   60 #define CENTER_LOST_POINT 20

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   61 uint8 current_deal_line=0;     //当前处理的行
current_deal_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   62 uint8 deal_start_line = 0;                //这个 值时控制处理的起始行一般定义为基准行 + 4
deal_start_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   63 uint8 hang_search_start = 0;             //定义每次扫描的开始是从哪个点开始的
hang_search_start:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   64 uint8 whitepoint_start=0;                //从左至右白点开始处
whitepoint_start:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   65 uint8 whitepoint_end=0;                 //从左至右白点结束处
whitepoint_end:
        DS8 1

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   66 uint8 whitebase_searchstart = MID;
whitebase_searchstart:
        DATA
        DC8 79

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   67 uint8 left_whitebase_searchstart = 0;
left_whitebase_searchstart:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   68 uint8 right_whitebase_searchstart = 0; 
right_whitebase_searchstart:
        DS8 1

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   69 uint8 re_whitepoint_start = 20;  // 发车的时候车子一定要在赛道的中心左右，否则会出现找不到赛道的危险
re_whitepoint_start:
        DATA
        DC8 20

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   70 uint8 re_whitepoint_end=145;   
re_whitepoint_end:
        DATA
        DC8 145

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   71 uint8 center_lost_hang = 0;
center_lost_hang:
        DS8 1

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   72 uint8 refer_road_width[ROW] ={127,126,125,124,123,122,120,119,118,117,
refer_road_width:
        DATA
        DC8 127, 126, 125, 124, 123, 122, 120, 119, 118, 117, 116, 115, 114
        DC8 113, 112, 110, 108, 106, 104, 112, 100, 98, 97, 95, 93, 92, 90, 89
        DC8 88, 87, 86, 85, 83, 81, 80, 79, 77, 75, 73, 70, 69, 68, 66, 64, 62
        DC8 60, 58, 56, 53, 51, 49, 47, 45, 43, 41, 39, 38, 36, 35, 33, 32, 31
        DC8 30, 28, 26, 0, 0, 0
//   73                               116,115,114,113,112,110,108,106,104,112,
//   74                               100,98,97,95,93,92,90,89,88,87,
//   75                               86,85,83,81,80,79,77,75,73,70,
//   76                               69,68,66,64,62,60,58,56,53,51,
//   77                               49,47,45,43,41,39,38,36,35,33,
//   78                                32,31,30,28,26};//

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   79 uint8 OT=36;                                     //判定为灰度值的跳变沿的最小灰度的跳变值
OT:
        DATA
        DC8 36

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   80 uint8 BASE_OT = 130;
BASE_OT:
        DATA
        DC8 130

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   81 uint8 WHITE_BLACK_OT = 145;           //进行二值化的分界值
WHITE_BLACK_OT:
        DATA
        DC8 145
//   82 #define WHITE 255
//   83 #define BLACK 10

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   84 uint8 top_whiteline = ROW-1;                          //图像的最顶行
top_whiteline:
        DATA
        DC8 64

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   85 uint8 left_top_whiteline = ROW-1;
left_top_whiteline:
        DATA
        DC8 64

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   86 uint8 right_top_whiteline = ROW-1;
right_top_whiteline:
        DATA
        DC8 64
//   87 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   88 uint8 bottom_whitebase = 0;                       //图像的基准行 
bottom_whitebase:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   89 bool find_whitebase_flag = 0;  //基准行的标志位
find_whitebase_flag:
        DS8 1
//   90 

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   91 uint8 re_white_refer = MID;  //这个点作为每场搜索基准行的开始的点  ，最开始的时候定义为 默认为MID
re_white_refer:
        DATA
        DC8 79

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   92 uint8 white_refer = 0;                            //基准行上的赛道的中点
white_refer:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   93 uint8 Row_state[ROW] =
Row_state:
        DS8 68
//   94 {
//   95   0
//   96 };
//   97 //--------------------------------------赛道处理的相关参数-----------------------//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   98 uint8 S_right = 0;//向右拐的计数
S_right:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   99 uint8 S_left =0 ; //向左拐计数
S_left:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  100 uint8 S_straight = 0;
S_straight:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  101 uint8 direction = 0; //4是初始化的值
direction:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  102 uint8 re_direction = 0;//记录上一次的当有的时候，无法判断出赛道的类型的时候，用上一次的状态
re_direction:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  103 uint32 center_average = 0;
center_average:
        DS8 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  104 uint16 center_error_average = 0;  
center_error_average:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  105 uint32 center_linear_average = 0;
center_linear_average:
        DS8 4
//  106 
//  107 #define RAMP_WIDTH  90                  //

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  108 uint16 ramp_delay_time = 25;
ramp_delay_time:
        DATA
        DC16 25

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  109 uint16 ramp_time = 0;                //进入坡道后多长时间重新开启起跑线检测
ramp_time:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  110 uint16 ramp_dis_time = 0;       //防止下坡的误判而延时
ramp_dis_time:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  111 uint16 ramp_speed = 80;                    //坡道减速值70
ramp_speed:
        DATA
        DC16 80

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  112 bool ramp_flag = 0;                          //进入坡道标志,主要用于控制
ramp_flag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  113 bool ramp_dis_flag = 0;                     //主要是防止下坡误判
ramp_dis_flag:
        DS8 1
//  114  
//  115  /*测试的时候，监测函数中局部变量
//  116 float XX_square_sum=0;   //X轴平方和
//  117 float YY_square_sum=0;   //Y轴平方和
//  118 float XYmulti_sum=0;      //XY乘积之和*/

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  119 float linear_factor = 0;
linear_factor:
        DS8 4
//  120 
//  121 //-----串口功能选择----//

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  122 uint8 send_mes=0;              //根据上位机发送来的数据来选择不同的串口功能
send_mes:
        DS8 1
//  123 
//  124 //------------------------------------电机控制函数的参数-----------------------------------//
//  125 #define SPEEDCHOICENUM 6  //定义6当
//  126 typedef struct Tag_SpeedSwitch{
//  127   uint16 Cstraightspeed;
//  128   uint16 Cbowspeed;
//  129   uint16 Cstraightspeed_ed;
//  130 }SpeedSwitch;
//  131 

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//  132 SpeedSwitch mySpeedswitch[SPEEDCHOICENUM]={
mySpeedswitch:
        DATA
        DC16 140, 136, 38, 135, 132, 34, 130, 126, 28, 125, 115, 25, 115, 108
        DC16 15, 110, 90, 10
//  133   {140,136,38},    //
//  134   {135,132,34},
//  135   {130,126,28},
//  136   {125,115,25},
//  137   {115,108,15},
//  138   {110,90,10}  
//  139 };
//  140 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  141 uint16 speed_feedback = 0;               //编码器的返回值                  //
speed_feedback:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  142 int16 speed_re_error = 0;
speed_re_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  143 bool stopflag = 0;//速度反馈
stopflag:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  144 uint16 speed_down_cnt = 0;       //检测到起跑线后延时十场，然后减速
speed_down_cnt:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  145 int16 speed_error = 0;
speed_error:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  146 uint8 speed_p = 80;//44
speed_p:
        DATA
        DC8 80

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  147 uint8 speed_i =95;//65
speed_i:
        DATA
        DC8 95

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  148 int16 speed = 0;
speed:
        DS8 2
//  149 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  150 uint16 lcd_straight_speed = 0;  //方便与lcd调节速度的最大和最小值
lcd_straight_speed:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  151 uint16 lcd_bow_speed = 0;//
lcd_bow_speed:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  152 uint16 lcd_straight_speed_ed = 0;
lcd_straight_speed_ed:
        DS8 2
//  153 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  154 uint16 straight_speed = 115;
straight_speed:
        DATA
        DC16 115

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  155 uint16 bow_speed = 100;
bow_speed:
        DATA
        DC16 100

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  156 uint16 straight_speed_ed = 8;
straight_speed_ed:
        DATA
        DC16 8

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  157 uint16 max_speed=900;
max_speed:
        DATA
        DC16 900

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  158 uint16 min_speed=20;
min_speed:
        DATA
        DC16 20
//  159 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  160 uint16 straight_count = 0;
straight_count:
        DS8 2
//  161 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  162 bool dead_stop = 0; 
dead_stop:
        DS8 1
//  163 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  164 uint16 speed_except=0;
speed_except:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  165 uint8 re_top_whiteline=0;
re_top_whiteline:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  166 uint8 speed_select = 0;//当为1的时候选择低速跑，用于测试跑道和算法 ，通过拨码控制
speed_select:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  167 uint8 full_speed_line = 0;
full_speed_line:
        DS8 1
//  168 //-----------------------------------舵机控制函数的变量---------------------------------//
//  169 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  170 int16 angle=1460;
angle:
        DATA
        DC16 1460

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  171 int16 re_angle= 1460;
re_angle:
        DATA
        DC16 1460

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  172 uint16 mid_angle=1460; //  推着向右拐，说明小于摆正值     当车子跑的稳定于左边时，减去一个值可恢复到中间
mid_angle:
        DATA
        DC16 1460
//  173 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  174 uint16 control_top_whiteline = 0;//re_control_top_whiteline top_error_servo_p
control_top_whiteline:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  175 uint16 re_control_top_whiteline = ROW - 1;
re_control_top_whiteline:
        DATA
        DC16 64

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  176 uint16 danger_count = 0;  //记录危险的点数
danger_count:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  177 bool danger_flag = 1;  //这个的初始值为1.当出了控制最高行了则置为0；
danger_flag:
        DATA
        DC8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  178 int16 p_error=0;
p_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  179 int16 p_re_error = 0;
p_re_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  180 int16 ref_his_error[5] ={0,0,0,0,0};//用这个数组来记录历史的值，然后参与当前的error
ref_his_error:
        DS8 12
//  181 
//  182 //对这两个pd的说明，第一个pd起到的作用是粗调，第二个pd是微调，当发现调节第二个pd,没有很大的改善的时候再且一定去动第一个pd
//  183 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  184 uint16 error_servo_p=0;
error_servo_p:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  185 int16 lcd_error_servo_p = 3; //4  2   4
lcd_error_servo_p:
        DATA
        DC16 3

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  186 uint16 error_servo_d=0;
error_servo_d:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  187 int16 lcd_error_servo_d = 57;  //83  40  65
lcd_error_servo_d:
        DATA
        DC16 57

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  188 uint16 top_error_servo_p = 0;
top_error_servo_p:
        DS8 2
//  189 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  190 uint16 error_servo_ref_p = 0;
error_servo_ref_p:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  191 int16 lcd_ref_p = 6;  //9      7和25使得直线很直   12  6          6
lcd_ref_p:
        DATA
        DC16 6

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  192 uint16 error_servo_ref_d = 0;
error_servo_ref_d:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  193 int16 lcd_ref_d = 40;  // 25                         60   23
lcd_ref_d:
        DATA
        DC16 40
//  194 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  195 uint8 get_p_errorline = 0;
get_p_errorline:
        DS8 1
//  196  

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  197 int16 refer_error =0;
refer_error:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  198 int16 re_refer_error = 0;
re_refer_error:
        DS8 2
//  199 
//  200 //起跑加载检测延时
//  201 /*#define right_tube1   (bool)(GPIOB_PDIR >> 4 & 0x00000001)             //分别读取红外管的状态
//  202 #define right_tube2   (bool)((GPIOB_PDIR >> 5) & 0x00000001)
//  203 #define left_tube1   (bool)((GPIOB_PDIR >> 6) & 0x00000001)
//  204 #define left_tube2   (bool)((GPIOB_PDIR >> 7) & 0x00000001)*/

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  205 uint32 start_stop_count = 0;  //起跑线检测计数
start_stop_count:
        DS8 4

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//  206 uint32 stop_pit_count = 6;
stop_pit_count:
        DATA
        DC32 6

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  207 bool start_stop_en = 0;   //起跑线检测使能
start_stop_en:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  208 bool start_stop_cs =0;   //起跑线检测的片选信号   当为1的时候选中检测起跑线
start_stop_cs:
        DS8 1
//  209 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  210 uint16 car_test_run = 0; 
car_test_run:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  211 bool test_run = 1;  
test_run:
        DATA
        DC8 1
//  212 //-------------------------------按键的定义-------------------------------------------//
//  213 #define LCD_ROW 7                      //小液晶屏的实际的行数为8行

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  214 bool change_page=0;             //翻页
change_page:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  215 bool se_sub_NUM=0;             //翻页
se_sub_NUM:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  216 bool up_line=0;              //换行   向上
up_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  217 bool down_line=0;            //换行   向下
down_line:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  218 bool add_NUM=0;              //更改数值  加
add_NUM:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  219 bool sub_NUM=0;              //更改数值  减 
sub_NUM:
        DS8 1
//  220 
//  221 //--------------------------------拨码的定义参数--------------------------------------//

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  222 bool lcd_debug = 1;            //lcd的调试选择
lcd_debug:
        DATA
        DC8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  223 bool redraw_control=0;         //刷屏的控制位
redraw_control:
        DS8 1
//  224 
//  225 //------------------------------------LCD变量声明-------------------------------------//
//  226 void pre_show(void);        //第一面的预显示
//  227 void redraw(void);          //刷频幕
//  228 void Keyscan(void);          //扫描拨码
//  229 void LCD_change_value(unsigned char page,unsigned char m,int i);//更改数值
//  230 void Delay_MS(uint32 ms);       //延时函数

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  231 uint8 lcd_page_num=1;        //液晶屏的页数
lcd_page_num:
        DATA
        DC8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  232 uint8 lcd_line_num=0;        //液晶屏的行数
lcd_line_num:
        DS8 1
//  233 
//  234 
//  235 //---------------------------数组初始化--------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  236 void Initial(void)
//  237 {
//  238   int16 i;
//  239      for(i = 0;i < ROW;i++)
Initial:
        MOVS     R0,#+0
        B.N      ??Initial_0
//  240        {
//  241          left_black[i] = 0;
??Initial_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable5
        MOVS     R2,#+0
        STRB     R2,[R0, R1]
//  242          right_black[i] = 0;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable5_1
        MOVS     R2,#+0
        STRB     R2,[R0, R1]
//  243          center_white[i] = 0;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable5_2
        MOVS     R2,#+0
        STRB     R2,[R0, R1]
//  244          Row_state[i] = 3; //3代表的是两边都没有出现丢点
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable5_3
        MOVS     R2,#+3
        STRB     R2,[R0, R1]
//  245        }
        ADDS     R0,R0,#+1
??Initial_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+65
        BLT.N    ??Initial_1
//  246        start_stop_count = 0;
        LDR.W    R0,??DataTable5_4
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  247        ramp_dis_flag = 0;
        LDR.W    R0,??DataTable5_5
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  248        ramp_flag = 0;
        LDR.W    R0,??DataTable6
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  249 }
        BX       LR               ;; return
//  250 
//  251 //--------------------低功耗脉冲计数器初始化-----------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  252 void LPTMR_Init()   //PTC5  LPT0_ALT2
//  253 {
//  254    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //开启C端口时钟
LPTMR_Init:
        LDR.W    R0,??DataTable5_6  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.W    R1,??DataTable5_6  ;; 0x40048038
        STR      R0,[R1, #+0]
//  255    PORTC_PCR5 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable5_7  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable5_7  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  256    PORTC_PCR5 |= PORT_PCR_MUX(4);  //PTC5配置为LPTMR模式
        LDR.W    R0,??DataTable5_7  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x400
        LDR.W    R1,??DataTable5_7  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  257    PORTC_PCR5 |= PORT_PCR_PE_MASK; //
        LDR.W    R0,??DataTable5_7  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable5_7  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  258    PORTC_PCR5 &= ~PORT_PCR_PS_MASK; //下拉
        LDR.W    R0,??DataTable5_7  ;; 0x4004b014
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable5_7  ;; 0x4004b014
        STR      R0,[R1, #+0]
//  259 
//  260    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;  //使能LPTM模块时钟
        LDR.W    R0,??DataTable5_6  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable5_6  ;; 0x40048038
        STR      R0,[R1, #+0]
//  261    LPTMR0_CSR &= ~LPTMR_CSR_TPS_MASK;
        LDR.W    R0,??DataTable6_1  ;; 0x40040000
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x30
        LDR.W    R1,??DataTable6_1  ;; 0x40040000
        STR      R0,[R1, #+0]
//  262    LPTMR0_CSR |= LPTMR_CSR_TPS(2)| LPTMR_CSR_TMS_MASK; //  ALT2  计数模式
        LDR.W    R0,??DataTable6_1  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x22
        LDR.W    R1,??DataTable6_1  ;; 0x40040000
        STR      R0,[R1, #+0]
//  263    LPTMR0_CSR |= LPTMR_CSR_TFC_MASK;  //溢出复位 65535
        LDR.W    R0,??DataTable6_1  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable6_1  ;; 0x40040000
        STR      R0,[R1, #+0]
//  264    LPTMR0_CSR &= ~LPTMR_CSR_TPP_MASK;  //上升沿计数
        LDR.W    R0,??DataTable6_1  ;; 0x40040000
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x8
        LDR.W    R1,??DataTable6_1  ;; 0x40040000
        STR      R0,[R1, #+0]
//  265 
//  266    LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK; //  忽略分频和滤波
        LDR.W    R0,??DataTable6_2  ;; 0x40040004
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable6_2  ;; 0x40040004
        STR      R0,[R1, #+0]
//  267    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;  //开启LPT模块
        LDR.W    R0,??DataTable6_1  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable6_1  ;; 0x40040000
        STR      R0,[R1, #+0]
//  268 }
        BX       LR               ;; return
//  269 
//  270 
//  271 //---------------------------行中断捕捉端口初始化-------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  272 void EXIT_Init(void)
//  273 {
//  274     SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  //开启C端口时钟
EXIT_Init:
        LDR.N    R0,??DataTable5_6  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.N    R1,??DataTable5_6  ;; 0x40048038
        STR      R0,[R1, #+0]
//  275     PORTC_PCR3 =PORT_PCR_MUX(1);  //GPIO
        LDR.W    R0,??DataTable6_3  ;; 0x4004b00c
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  276     GPIOC_PDDR &= ~GPIO_PIN(3);   //输入
        LDR.W    R0,??DataTable6_4  ;; 0x400ff094
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x8
        LDR.W    R1,??DataTable6_4  ;; 0x400ff094
        STR      R0,[R1, #+0]
//  277     PORTC_PCR3 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; //上拉电阻;
        LDR.W    R0,??DataTable6_3  ;; 0x4004b00c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3
        LDR.W    R1,??DataTable6_3  ;; 0x4004b00c
        STR      R0,[R1, #+0]
//  278     PORTC_PCR3 |= PORT_PCR_IRQC(9); //9为上升沿触发外部中断 10为下降沿触
        LDR.W    R0,??DataTable6_3  ;; 0x4004b00c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x90000
        LDR.W    R1,??DataTable6_3  ;; 0x4004b00c
        STR      R0,[R1, #+0]
//  279 }
        BX       LR               ;; return
//  280 
//  281 
//  282 
//  283 //----------------------------串口初始化-----------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  284 void UART0_Init(void)    //PTB16 RXD    PTB17 TXD
//  285 {
UART0_Init:
        PUSH     {R4}
//  286     uint32 uartclk_khz = CORE_CLOCK*10 * BUS_CLOCK;//时钟180MHz    //随时更改
        LDR.W    R0,??DataTable6_5  ;; 0x2bf20
//  287     uint32 baud = BAUD;
        MOV      R1,#+19200
//  288     uint16 sbr,brfa;
//  289     
//  290     SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //开启B口时钟
        LDR.N    R2,??DataTable5_6  ;; 0x40048038
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x400
        LDR.N    R3,??DataTable5_6  ;; 0x40048038
        STR      R2,[R3, #+0]
//  291     PORTB_PCR16|=PORT_PCR_MUX(3);//将PTB16引脚设置为模式3，即UART0_RX
        LDR.W    R2,??DataTable6_6  ;; 0x4004a040
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.W    R3,??DataTable6_6  ;; 0x4004a040
        STR      R2,[R3, #+0]
//  292     PORTB_PCR17|=PORT_PCR_MUX(3);//将PTB177引脚设置为模式3，即UART0_TX
        LDR.W    R2,??DataTable6_7  ;; 0x4004a044
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.W    R3,??DataTable6_7  ;; 0x4004a044
        STR      R2,[R3, #+0]
//  293     SIM_SCGC4|=SIM_SCGC4_UART0_MASK;//开启UART0时钟
        LDR.W    R2,??DataTable6_8  ;; 0x40048034
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x400
        LDR.W    R3,??DataTable6_8  ;; 0x40048034
        STR      R2,[R3, #+0]
//  294     sbr = (uint16)((uartclk_khz*1000)/(baud*16));//计算并设置波特率
        MOV      R2,#+1000
        MUL      R2,R2,R0
        LSLS     R3,R1,#+4
        UDIV     R2,R2,R3
//  295     
//  296     UART0_BDH = (uint8)((sbr&0x1F00)>>8);//将波特率19200写入相应的寄存器然后进行使能，使其工作。前面的buad只是一个数字，而后面的计算是将19200写入这个寄存器，然后进行使能
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        ASRS     R3,R2,#+8
        ANDS     R3,R3,#0x1F
        LDR.W    R4,??DataTable6_9  ;; 0x4006a000
        STRB     R3,[R4, #+0]
//  297     UART0_BDL=(uint8)(sbr&0x00FF);
        LDR.W    R3,??DataTable6_10  ;; 0x4006a001
        STRB     R2,[R3, #+0]
//  298     brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
        MOV      R3,#+32000
        MULS     R0,R3,R0
        LSLS     R1,R1,#+4
        UDIV     R0,R0,R1
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        SUBS     R0,R0,R2, LSL #+5
//  299     UART0_C4 = (uint8)(brfa & 0x001F);
        ANDS     R0,R0,#0x1F
        LDR.W    R1,??DataTable6_11  ;; 0x4006a00a
        STRB     R0,[R1, #+0]
//  300     UART0_C2 |=(UART_C2_TE_MASK|UART_C2_RE_MASK);
        LDR.W    R0,??DataTable6_12  ;; 0x4006a003
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0xC
        LDR.W    R1,??DataTable6_12  ;; 0x4006a003
        STRB     R0,[R1, #+0]
//  301     UART0_C1 = 0;	
        LDR.W    R0,??DataTable6_13  ;; 0x4006a002
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  302     UART0_C2 |= UART_C2_RIE_MASK;   //开UART0接收中断
        LDR.W    R0,??DataTable6_12  ;; 0x4006a003
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable6_12  ;; 0x4006a003
        STRB     R0,[R1, #+0]
//  303 }
        POP      {R4}
        BX       LR               ;; return
//  304 
//  305 
//  306 //-------------------------------------ftm初始化-----------------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  307 void hw_FTM_init(void)
//  308 {      	
//  309     SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;//开启C端口时钟
hw_FTM_init:
        LDR.N    R0,??DataTable5_6  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.N    R1,??DataTable5_6  ;; 0x40048038
        STR      R0,[R1, #+0]
//  310     SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //开启A端口时钟
        LDR.N    R0,??DataTable5_6  ;; 0x40048038
        LDR      R0,[R0, #+0]
        MOV      R1,#+512
        ORRS     R0,R1,R0
        LDR.N    R1,??DataTable5_6  ;; 0x40048038
        STR      R0,[R1, #+0]
//  311   
//  312     
//  313     PORTC_PCR4 &= ~PORT_PCR_MUX_MASK; //清零
        LDR.W    R0,??DataTable6_14  ;; 0x4004b010
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_14  ;; 0x4004b010
        STR      R0,[R1, #+0]
//  314     PORTA_PCR12 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_15  ;; 0x40049030
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_15  ;; 0x40049030
        STR      R0,[R1, #+0]
//  315     PORTA_PCR13 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_16  ;; 0x40049034
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_16  ;; 0x40049034
        STR      R0,[R1, #+0]
//  316     PORTA_PCR10 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_17  ;; 0x40049028
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_17  ;; 0x40049028
        STR      R0,[R1, #+0]
//  317     PORTC_PCR3 &= ~PORT_PCR_MUX_MASK;
        LDR.W    R0,??DataTable6_3  ;; 0x4004b00c
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x700
        LDR.W    R1,??DataTable6_3  ;; 0x4004b00c
        STR      R0,[R1, #+0]
//  318     
//  319     PORTC_PCR4 = PORT_PCR_MUX(4); //FTM is alt4 function for this pin
        LDR.W    R0,??DataTable6_14  ;; 0x4004b010
        MOV      R1,#+1024
        STR      R1,[R0, #+0]
//  320     PORTA_PCR10 = PORT_PCR_MUX(3);
        LDR.W    R0,??DataTable6_17  ;; 0x40049028
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  321     PORTA_PCR12 = PORT_PCR_MUX(3);//FTM is alt3 function for this pin 
        LDR.W    R0,??DataTable6_15  ;; 0x40049030
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  322     PORTA_PCR13 = PORT_PCR_MUX(3);
        LDR.W    R0,??DataTable6_16  ;; 0x40049034
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  323       PORTC_PCR3 = PORT_PCR_MUX(3);
        LDR.W    R0,??DataTable6_3  ;; 0x4004b00c
        MOV      R1,#+768
        STR      R1,[R0, #+0]
//  324   
//  325     SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;     //使能FTM0时钟
        LDR.W    R0,??DataTable6_18  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable6_18  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  326     SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;    //开启FTM1模块时钟
        LDR.W    R0,??DataTable6_18  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2000000
        LDR.W    R1,??DataTable6_18  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  327     SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;    //开启FTM2模块时钟
        LDR.W    R0,??DataTable6_19  ;; 0x40048030
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable6_19  ;; 0x40048030
        STR      R0,[R1, #+0]
//  328     
//  329     FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置FTM0_CH3 
        LDR.W    R0,??DataTable6_20  ;; 0x40038024
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_20  ;; 0x40038024
        STR      R0,[R1, #+0]
//  330     FTM1_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置模式 CH0
        LDR.W    R0,??DataTable6_21  ;; 0x4003900c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_21  ;; 0x4003900c
        STR      R0,[R1, #+0]
//  331     FTM1_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //FTM1_CH1
        LDR.W    R0,??DataTable6_22  ;; 0x40039014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_22  ;; 0x40039014
        STR      R0,[R1, #+0]
//  332     FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置FTM2_CH0 
        LDR.W    R0,??DataTable6_23  ;; 0x400b800c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_23  ;; 0x400b800c
        STR      R0,[R1, #+0]
//  333     FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置FTM2_CH0 
        LDR.W    R0,??DataTable6_24  ;; 0x4003801c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x28
        LDR.W    R1,??DataTable6_24  ;; 0x4003801c
        STR      R0,[R1, #+0]
//  334     
//  335     
//  336     FTM0_CNT=0;//设置计数初值为0
        LDR.W    R0,??DataTable6_25  ;; 0x40038004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  337     FTM1_CNT=0;
        LDR.W    R0,??DataTable6_26  ;; 0x40039004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  338     FTM2_CNT=0;
        LDR.W    R0,??DataTable6_27  ;; 0x400b8004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  339     
//  340     //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 
//  341 
//  342     //设置 the pulse width(duty cycle) is determined by (CnV - CNTIN).
//  343     
//  344      FTM1_MOD =1000;              //设置PWM频率为10K=100 000 000 /2^2/2500  这个100 000 000 是第七届的系统频率  2500是其设置的FTM1_MOD 值
        LDR.W    R0,??DataTable6_28  ;; 0x40039008
        MOV      R1,#+1000
        STR      R1,[R0, #+0]
//  345      FTM0_MOD =2; //3145   300hz sd5舵机的频率恰当，会导致舵机很软，这个需要去调整，舵机很软的原因有两个，电压问题，频率问题。。一般在300hz左右  //18750  50hz
        LDR.W    R0,??DataTable6_29  ;; 0x40038008
        MOVS     R1,#+2
        STR      R1,[R0, #+0]
//  346      FTM2_MOD =2; 
        LDR.W    R0,??DataTable6_30  ;; 0x400b8008
        MOVS     R1,#+2
        STR      R1,[R0, #+0]
//  347     
//  348     FTM0_CNTIN=0;//设置初始化计数值
        LDR.W    R0,??DataTable6_31  ;; 0x4003804c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  349     FTM1_CNTIN=0;
        LDR.W    R0,??DataTable6_32  ;; 0x4003904c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  350     FTM2_CNTIN=0;
        LDR.W    R0,??DataTable6_33  ;; 0x400b804c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  351       
//  352     FTM0_C3V=mid_angle;//1400;//1490 1.5ms//2ms  1986//2.5ms  2483 //1ms 993// 0.5ms  497  //1614
        LDR.W    R0,??DataTable6_34  ;; 0x40038028
        LDR.W    R1,??DataTable6_35
        LDRH     R1,[R1, #+0]
        STR      R1,[R0, #+0]
//  353 
//  354     FTM1_C0V=0;
        LDR.W    R0,??DataTable6_36  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  355     FTM1_C1V=0;
        LDR.W    R0,??DataTable6_37  ;; 0x40039018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  356     
//  357     FTM2_C0V=1;
        LDR.W    R0,??DataTable6_38  ;; 0x400b8010
        MOVS     R1,#+1
        STR      R1,[R0, #+0]
//  358     
//  359     FTM0_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);
        LDR.W    R0,??DataTable6_39  ;; 0x40038000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x9
        LDR.W    R1,??DataTable6_39  ;; 0x40038000
        STR      R0,[R1, #+0]
//  360     FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(2); //设置时钟和分频
        LDR.W    R0,??DataTable6_40  ;; 0x40039000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0xA
        LDR.W    R1,??DataTable6_40  ;; 0x40039000
        STR      R0,[R1, #+0]
//  361     FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);
        LDR.W    R0,??DataTable6_41  ;; 0x400b8000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x9
        LDR.W    R1,??DataTable6_41  ;; 0x400b8000
        STR      R0,[R1, #+0]
//  362 }
        BX       LR               ;; return
//  363 
//  364 //----------------------锁相环频率为50/15*54=180M测试函数-------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  365 void pllinit180M(void)
//  366 {
//  367 	uint32_t temp_reg;
//  368         //使能IO端口时钟    
//  369     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
//  370                               | SIM_SCGC5_PORTB_MASK
//  371                               | SIM_SCGC5_PORTC_MASK
//  372                               | SIM_SCGC5_PORTD_MASK
//  373                               | SIM_SCGC5_PORTE_MASK );
pllinit180M:
        LDR.N    R0,??DataTable5_6  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3E00
        LDR.N    R1,??DataTable5_6  ;; 0x40048038
        STR      R0,[R1, #+0]
//  374     //这里处在默认的FEI模式
//  375     //首先移动到FBE模式
//  376     MCG_C2 = 0;  
        LDR.W    R0,??DataTable6_42  ;; 0x40064001
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  377     //MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
//  378     //初始化晶振后释放锁定状态的振荡器和GPIO
//  379     SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
        LDR.W    R0,??DataTable6_8  ;; 0x40048034
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x10000000
        LDR.W    R1,??DataTable6_8  ;; 0x40048034
        STR      R0,[R1, #+0]
//  380     LLWU_CS |= LLWU_CS_ACKISO_MASK;
        LDR.W    R0,??DataTable6_43  ;; 0x4007c008
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x80
        LDR.W    R1,??DataTable6_43  ;; 0x4007c008
        STRB     R0,[R1, #+0]
//  381     
//  382     //选择外部晶振，参考分频器，清IREFS来启动外部晶振
//  383     //011 If RANGE = 0, Divide Factor is 8; for all other RANGE values, Divide Factor is 256.
//  384     MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
        LDR.W    R0,??DataTable6_44  ;; 0x40064000
        MOVS     R1,#+152
        STRB     R1,[R0, #+0]
//  385     
//  386     //等待晶振稳定	    
//  387     //while (!(MCG_S & MCG_S_OSCINIT_MASK)){}              //等待锁相环初始化结束
//  388     while (MCG_S & MCG_S_IREFST_MASK){}                  //等待时钟切换到外部参考时钟
??pllinit180M_0:
        LDR.W    R0,??DataTable6_45  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+27
        BMI.N    ??pllinit180M_0
//  389     while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}
??pllinit180M_1:
        LDR.W    R0,??DataTable6_45  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        UBFX     R0,R0,#+2,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+2
        BNE.N    ??pllinit180M_1
//  390     
//  391     //进入FBE模式,
//  392     //0x18==25分频=2M,
//  393     //0x08==15分频=3.333M 
//  394     //0x09==16分频=3.125M,
//  395     //0x10==17分频=2.94M 
//  396     //0x11==18分频=2.7778M 
//  397     //0x12==19分频=2.63M,
//  398     //0x13==20分频=2.5M    
//  399     MCG_C5 = MCG_C5_PRDIV(0x0e);                
        LDR.W    R0,??DataTable6_46  ;; 0x40064004
        MOVS     R1,#+14
        STRB     R1,[R0, #+0]
//  400     
//  401     //确保MCG_C6处于复位状态，禁止LOLIE、PLL、和时钟控制器，清PLL VCO分频器
//  402     MCG_C6 = 0x0;
        LDR.W    R0,??DataTable6_47  ;; 0x40064005
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  403     
//  404     //保存FMC_PFAPR当前的值
//  405     temp_reg = FMC_PFAPR;
        LDR.W    R0,??DataTable6_48  ;; 0x4001f000
        LDR      R0,[R0, #+0]
//  406     
//  407     //通过M&PFD置位M0PFD来禁止预取功能
//  408     FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
//  409                      | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
//  410                      | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;    
        LDR.W    R1,??DataTable6_48  ;; 0x4001f000
        LDR      R1,[R1, #+0]
        ORRS     R1,R1,#0xFF0000
        LDR.W    R2,??DataTable6_48  ;; 0x4001f000
        STR      R1,[R2, #+0]
//  411     ///设置系统分频器
//  412     //MCG=PLL, core = MCG, bus = MCG/3, FlexBus = MCG/3, Flash clock= MCG/8
//  413     SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) 
//  414                  | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);       
        LDR.W    R1,??DataTable6_49  ;; 0x40048044
        LDR.W    R2,??DataTable6_50  ;; 0x2270000
        STR      R2,[R1, #+0]
//  415     
//  416     //从新存FMC_PFAPR的原始值
//  417     FMC_PFAPR = temp_reg; 
        LDR.W    R1,??DataTable6_48  ;; 0x4001f000
        STR      R0,[R1, #+0]
//  418     
//  419     //设置VCO分频器，使能PLL为100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
//  420     MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(30);  //VDIV = 31 (x54)
        LDR.W    R0,??DataTable6_47  ;; 0x40064005
        MOVS     R1,#+94
        STRB     R1,[R0, #+0]
//  421                                                   //VDIV = 26 (x50)
//  422     while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set    
??pllinit180M_2:
        LDR.W    R0,??DataTable6_45  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+26
        BPL.N    ??pllinit180M_2
//  423     while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set    
??pllinit180M_3:
        LDR.W    R0,??DataTable6_45  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+25
        BPL.N    ??pllinit180M_3
//  424     
//  425     //进入PBE模式    
//  426     //通过清零CLKS位来进入PEE模式
//  427     // CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
//  428     MCG_C1 &= ~MCG_C1_CLKS_MASK;
        LDR.W    R0,??DataTable6_44  ;; 0x40064000
        LDRB     R0,[R0, #+0]
        ANDS     R0,R0,#0x3F
        LDR.W    R1,??DataTable6_44  ;; 0x40064000
        STRB     R0,[R1, #+0]
//  429     
//  430     //等待时钟状态位更新
//  431     while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};
??pllinit180M_4:
        LDR.W    R0,??DataTable6_45  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        UBFX     R0,R0,#+2,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+3
        BNE.N    ??pllinit180M_4
//  432     //SIM_CLKDIV2 |= SIM_CLKDIV2_USBDIV(1);  
//  433     
//  434     //设置跟踪时钟为内核时钟
//  435     SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;	
        LDR.W    R0,??DataTable6_51  ;; 0x40048004
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000
        LDR.W    R1,??DataTable6_51  ;; 0x40048004
        STR      R0,[R1, #+0]
//  436     //在PTA6引脚上使能TRACE_CLKOU功能
//  437     PORTA_PCR6 = ( PORT_PCR_MUX(0x7));  
        LDR.W    R0,??DataTable6_52  ;; 0x40049018
        MOV      R1,#+1792
        STR      R1,[R0, #+0]
//  438     //使能FlexBus模块时钟
//  439     SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
        LDR.W    R0,??DataTable6_53  ;; 0x40048040
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable6_53  ;; 0x40048040
        STR      R0,[R1, #+0]
//  440     //在PTA6引脚上使能FB_CLKOUT功能
//  441     PORTC_PCR3 = ( PORT_PCR_MUX(0x5));
        LDR.W    R0,??DataTable6_3  ;; 0x4004b00c
        MOV      R1,#+1280
        STR      R1,[R0, #+0]
//  442 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_1:
        DC32     right_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_2:
        DC32     center_white

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_3:
        DC32     Row_state

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_4:
        DC32     start_stop_count

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_5:
        DC32     ramp_dis_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_6:
        DC32     0x40048038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable5_7:
        DC32     0x4004b014
//  443 
//  444 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  445 int16 f_abs16(int16 x)
//  446 {
//  447   if(x>0) return x;
f_abs16:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+1
        BLT.N    ??f_abs16_0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        B.N      ??f_abs16_1
//  448   else return -x;
??f_abs16_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        RSBS     R0,R0,#+0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
??f_abs16_1:
        BX       LR               ;; return
//  449 }
//  450 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  451 float f_absf(float x)
//  452 {
f_absf:
        PUSH     {LR}
//  453     if(x>=0.0) return x;
        MOVS     R1,#+0
        BL       __aeabi_cfrcmple
        BLS.N    ??f_absf_0
//  454     else  return -x;
??f_absf_1:
        EORS     R0,R0,#0x80000000
??f_absf_0:
        POP      {PC}             ;; return
//  455 }
//  456 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  457 int MIN_INT( int a, int b)
//  458 {
//  459     if(a>b) return b;
MIN_INT:
        CMP      R1,R0
        BGE.N    ??MIN_INT_0
        MOVS     R0,R1
        B.N      ??MIN_INT_1
//  460     else  return a;
??MIN_INT_0:
??MIN_INT_1:
        BX       LR               ;; return
//  461 }
//  462 
//  463 //-----------------------------------扫描白线基准线---------------------------------//
//  464 /*
//  465 1、由于赛道的宽度在图像中所占的比例较大，故可认为在中点的位置（79处）就一定是在赛道中，
//  466 ，除非车子跑出了赛道，而不需要考虑中心偏离赛道的情况。
//  467 */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  468 void Search_WhiteBase(void)   //从图像底部中间开始向两边扫描白线基准
//  469 { 
Search_WhiteBase:
        PUSH     {R4-R6,LR}
//  470   uint8 i = 0,j = 0 ;//定义十六位的有符号变量   i代表行变量  j代表列变量
        MOVS     R5,#+0
        MOVS     R4,#+0
//  471   uint8 base_sum = 0; 
        MOVS     R6,#+0
//  472   current_deal_line=0;                //记录在搜索基准行的时候的当前处理的行 
        LDR.W    R0,??DataTable6_54
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  473   bottom_whitebase = 0;//基准行赋初值  int
        LDR.W    R0,??DataTable6_55
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  474   find_whitebase_flag = 0;               //是否发现白线基准标志
        LDR.W    R0,??DataTable6_56
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  475 
//  476   //////////////////////////////滤波开始///////////////////////////////////  
//  477 //首先对整幅图像进行滤波，采用的方法是中值滤波
//  478   for(i = 0;i < ROW / 5;i++)                 //只是对图像前几行进行滤波，原因是远处的滤波可能会把跑道的信息滤除掉这里对赛道的前13行滤波
        MOVS     R0,#+0
        MOVS     R5,R0
        B.N      ??Search_WhiteBase_0
//  479     for(j = 1;j< COLUMN-1;j++)
//  480     {
//  481         base_sum = (VideoImage1[i][j-1] + VideoImage1[i][j+1])/ 2;
??Search_WhiteBase_1:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        MOVS     R0,#+159
        LDR.W    R1,??DataTable6_57
        MLA      R0,R0,R5,R1
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        MOVS     R1,#+159
        LDR.W    R2,??DataTable6_57
        MLA      R1,R1,R5,R2
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+1]
        UXTAB    R0,R1,R0
        MOVS     R1,#+2
        SDIV     R6,R0,R1
//  482         if( f_abs16( base_sum - VideoImage1[i][j]) > OT)
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        MOVS     R0,#+159
        LDR.W    R1,??DataTable6_57
        MLA      R0,R0,R5,R1
        LDRB     R0,[R4, R0]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable6_58
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,R0
        BGE.N    ??Search_WhiteBase_2
//  483            VideoImage1[i][j] = base_sum;
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        MOVS     R0,#+159
        LDR.W    R1,??DataTable6_57
        MLA      R0,R0,R5,R1
        STRB     R6,[R4, R0]
//  484     }  //滤波可能带来一个后果，就是可能把远处的边沿线滤除掉，这里只是对近端进行滤波
??Search_WhiteBase_2:
        ADDS     R4,R4,#+1
??Search_WhiteBase_3:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+158
        BCC.N    ??Search_WhiteBase_1
        ADDS     R5,R5,#+1
??Search_WhiteBase_0:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+13
        BCS.N    ??Search_WhiteBase_4
        MOVS     R4,#+1
        B.N      ??Search_WhiteBase_3
//  485   
//  486   
//  487   /*////////////////////////对图像上的噪点进行滤除//////////////   限幅滤波会带来一个后果，导致跳变沿检测出现问题，跳变太小
//  488   for(i = 0;i < ROW ;i++)                 //图像上突然的出现了很多的噪点，这个程序是为了将图像上的这些噪点滤除。经过验证效果很好
//  489     for(j = 0;j< COLUMN;j++)
//  490     {
//  491       if(VideoImage1[i][j] >245 || VideoImage1[i][j]<10)
//  492       {
//  493         if( j>0 && j < COLUMN - 1)
//  494         {
//  495           VideoImage1[i][j] = (VideoImage1[i][j+1] + VideoImage1[i][j-1])/2;
//  496         }
//  497         else
//  498         {
//  499           if( i>0 && i < ROW - 1)
//  500           VideoImage1[i][j] =  (VideoImage1[i+1][j] +  VideoImage1[i-1][j])/2;
//  501         }
//  502       }
//  503     }
//  504 *////////////////////////////滤波结束////////////////////////////////
//  505   
//  506   /*对于搜索基准行最重要的就是解决搜索的开始点的问题，这个点找到了，其他的问题就好解决了*/
//  507   if(VideoImage1[0][re_white_refer] > BASE_OT && VideoImage1[0][re_white_refer-1] >BASE_OT && VideoImage1[0][re_white_refer+1]>BASE_OT)
??Search_WhiteBase_4:
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_60
        LDRB     R1,[R1, #+0]
        LDR.W    R2,??DataTable6_57
        LDRB     R1,[R1, R2]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_5
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_60
        LDRB     R1,[R1, #+0]
        LDR.W    R2,??DataTable6_57
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-1]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_5
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_60
        LDRB     R1,[R1, #+0]
        LDR.W    R2,??DataTable6_57
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #+1]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_5
//  508   {
//  509     whitebase_searchstart = re_white_refer;
        LDR.W    R0,??DataTable6_61
        LDR.W    R1,??DataTable6_60
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_6
//  510   }
//  511   else
//  512   {
//  513     j = MID-1;//从MID开始搜索基准行的开始点
??Search_WhiteBase_5:
        MOVS     R4,#+78
//  514     left_whitebase_searchstart = MID-1;
        LDR.W    R0,??DataTable6_62
        MOVS     R1,#+78
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_7
//  515     while(j > 10)
//  516     {
//  517       if(VideoImage1[0][j] > BASE_OT && VideoImage1[0][j-1] > BASE_OT &&VideoImage1[0][j-2]>BASE_OT
//  518          &&VideoImage1[0][j-3] >BASE_OT &&VideoImage1[0][j-4]>BASE_OT)
//  519       {
//  520         left_whitebase_searchstart = j;
//  521         break;
//  522       }
//  523       j--;
??Search_WhiteBase_8:
        SUBS     R4,R4,#+1
??Search_WhiteBase_7:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+11
        BCC.N    ??Search_WhiteBase_9
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        LDRB     R1,[R4, R1]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_8
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_8
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-2]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_8
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-3]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_8
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-4]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_8
        LDR.W    R0,??DataTable6_62
        STRB     R4,[R0, #+0]
//  524     }
//  525     
//  526     j = MID+1; 
??Search_WhiteBase_9:
        MOVS     R4,#+80
//  527     right_whitebase_searchstart = MID+1;
        LDR.W    R0,??DataTable6_63
        MOVS     R1,#+80
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_10
//  528     while(j < 150)
//  529     {
//  530        if(VideoImage1[0][j] > BASE_OT && VideoImage1[0][j+1] > BASE_OT && VideoImage1[0][j+2]>BASE_OT
//  531          && VideoImage1[0][j+3] > BASE_OT &&VideoImage1[0][j+4] > BASE_OT)
//  532       {
//  533         right_whitebase_searchstart = j;
//  534         break;
//  535       }
//  536       j++;
??Search_WhiteBase_11:
        ADDS     R4,R4,#+1
??Search_WhiteBase_10:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+150
        BCS.N    ??Search_WhiteBase_12
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        LDRB     R1,[R4, R1]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_11
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+1]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_11
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+2]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_11
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+3]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_11
        LDR.W    R0,??DataTable6_59
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_57
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+4]
        CMP      R0,R1
        BCS.N    ??Search_WhiteBase_11
        LDR.W    R0,??DataTable6_63
        STRB     R4,[R0, #+0]
//  537     }
//  538     
//  539     if(right_whitebase_searchstart-MID > MID-left_whitebase_searchstart)
??Search_WhiteBase_12:
        LDR.W    R0,??DataTable6_62
        LDRB     R0,[R0, #+0]
        RSBS     R0,R0,#+79
        LDR.W    R1,??DataTable6_63
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+79
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_13
//  540     {
//  541       whitebase_searchstart  = left_whitebase_searchstart;
        LDR.W    R0,??DataTable6_61
        LDR.W    R1,??DataTable6_62
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_6
//  542     }
//  543       
//  544     else
//  545     {
//  546       whitebase_searchstart = right_whitebase_searchstart;
??Search_WhiteBase_13:
        LDR.W    R0,??DataTable6_61
        LDR.W    R1,??DataTable6_63
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_6
//  547     }
//  548   } 
//  549   
//  550   
//  551   
//  552   while( find_whitebase_flag ==0 && current_deal_line < ROW - 1) //基准行的搜索范围从0到ROW-1 
//  553   {
//  554       //每行的处理清零
//  555       whitepoint_start = whitebase_searchstart;   //uint8 
//  556       //搜索左边的基准边沿/////////////////////////////
//  557        j = whitebase_searchstart;   //有的时候出现前一行的中点在下一行的图像的外面
//  558        //每次从上一场的基准的中点开始向两边搜索基准行
//  559        while(j >= 3  )//一般为了使得跳变沿更加的明显，采用隔点判断
//  560       {
//  561         
//  562           if( whitepoint_start != whitebase_searchstart && f_abs16(j-re_whitepoint_start) > f_abs16(whitepoint_start -re_whitepoint_start) )
//  563           {
//  564             break;
//  565           }
//  566           else if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] > OT 
//  567              && VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-3] > OT
//  568                )
//  569           {//当第一个if不满足的时候说明此时的j到上一场的点的距离一定小于 f_abs16(whitepoint_start -re_whitepoint_start) 
//  570             //所以这里只要遇到了跳变，就给基准行的起始点赋值
//  571                whitepoint_start = j;
//  572           }
//  573           j--;
//  574       }
//  575       
//  576       if( j == 2  && whitepoint_start == whitebase_searchstart)  //到达边界了，但是还没有对whitepoint_start赋值过，说明没有找到基准的开始点
//  577       {
//  578         if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] > OT)
//  579         {
//  580            whitepoint_start = 2; //到达了边界
//  581         }
//  582         else if( VideoImage1[current_deal_line][j-1] - VideoImage1[current_deal_line][j-2] > OT)
//  583         {
//  584            whitepoint_start = 1; //到达了边界
//  585         }
//  586         else
//  587         { 
//  588            whitepoint_start = 0;
//  589         }
//  590       }
//  591       //左边搜索结束///////////////////////
//  592       
//  593       //右边搜索开始/////////////////////
//  594       whitepoint_end = whitebase_searchstart;    //uint8
//  595       j = whitebase_searchstart;   //每次从上一场的white_refer向两边搜索基准行
//  596       while( j <= COLUMN-4 )//一般为了使得跳变沿更加的明显，采用隔点判断
//  597       {
//  598        
//  599           if( whitepoint_end != whitebase_searchstart && f_abs16(j-re_whitepoint_end) > f_abs16(whitepoint_end -re_whitepoint_end) )
//  600           {
//  601              break;
//  602           }
//  603           else if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+2] > OT
//  604              && VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+3] > OT)
//  605           {
//  606              whitepoint_end = j;
//  607           } 
//  608           j++;
//  609       } 
//  610       
//  611       if(j == COLUMN-3  &&   whitepoint_end == whitebase_searchstart)
//  612       {
//  613          if( VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+2] > OT)
//  614          {
//  615            whitepoint_end = COLUMN-3;
//  616          }
//  617          else if( VideoImage1[current_deal_line][j+1] - VideoImage1[current_deal_line][j+2] > OT)
//  618          {
//  619            whitepoint_end = COLUMN-2;
//  620          }
//  621          else
//  622          {
//  623            whitepoint_end = COLUMN-1;
//  624          }
//  625       }
//  626       //右边搜索结束///////////////////////
//  627       //左右边沿线的搜索方法用的是跟踪的搜索方法，目的是只想找到一行可靠的基准行的信息
//  628       
//  629       
//  630       //无论这一行是否符合要求，始终记录赛道信息
//  631 
//  632         left_black[current_deal_line] = whitepoint_start;   //记录左黑线位置 (若为0，很可能说明左边黑线丢失，即车身偏右)
//  633         right_black[current_deal_line] = whitepoint_end;    //记录右黑线位置(若为COLUMN，很可能说明右边黑线丢失,即车身偏左)
//  634         /*对于第一行的状态的判断有两种想法，第一：无论第一行是什么状态始终认为没有丢点 第二：将到达边沿的点视为丢点
//  635         此外这里需要对上面的三个值进行一定的修正，并且在这里将基准行上的状态设定为没有丢点（即使有时候到达了边界）。
//  636         （否则，这里的跟踪搜索的优势就没有了）
//  637         */
//  638         if(left_black[current_deal_line] == 0 && right_black[current_deal_line] < COLUMN - 1 &&  (right_black[current_deal_line] - left_black[current_deal_line]) > 155)
//  639         {  //表示左边到达边界 丢点
//  640           
//  641           left_black[current_deal_line] = re_whitepoint_start;
//  642           center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2; 
//  643         }
//  644         else if(right_black[current_deal_line] == COLUMN - 1 && left_black[current_deal_line] > 0 &&  (right_black[current_deal_line] - left_black[current_deal_line]) > 155)
//  645         { //表示右边到达边界 丢点
//  646            right_black[current_deal_line] = re_whitepoint_end;
//  647          center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2;  //记录中心点,大于MID说明车身偏左，反之，说明车身偏右
//  648         }
//  649         else if(right_black[current_deal_line] == COLUMN - 1 && left_black[current_deal_line] == 0 )
//  650         {//表示两边到达边界 丢点
//  651           left_black[current_deal_line] = re_whitepoint_start;
//  652           right_black[current_deal_line] = re_whitepoint_end;
//  653           center_white[current_deal_line] = re_white_refer;
//  654         }
//  655         //说明由于前三行的信息一般不做处理默认为左右边沿都找到了的点 
//  656         else
//  657         {  //表示两边都没有丢点
//  658           center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2;  //记录中心点,大于MID说明车身偏左，反之，说明车身偏右
//  659         }
//  660         //处理后重新得到图像搜索的开始结束及中心值
//  661       whitepoint_start = left_black[current_deal_line];
//  662       whitepoint_end =  right_black[current_deal_line];
//  663       white_refer = center_white[current_deal_line];
//  664       
//  665         //加上一个赛道的宽度的限制
//  666        if(whitepoint_end - whitepoint_start > MIN_WHITEBASE_POINT ) //这个值设置为20 
//  667         {
//  668           find_whitebase_flag = 1;
??Search_WhiteBase_14:
        LDR.W    R0,??DataTable6_56
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
//  669           re_white_refer = white_refer;  //保存本场图像的信息
        LDR.W    R0,??DataTable6_60
        LDR.W    R1,??DataTable6_64
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  670           re_whitepoint_start = whitepoint_start;
        LDR.W    R0,??DataTable6_65
        LDR.W    R1,??DataTable6_66
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  671           re_whitepoint_end  = whitepoint_end ;
        LDR.W    R0,??DataTable6_67
        LDR.W    R1,??DataTable6_68
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  672           bottom_whitebase = current_deal_line;//记录基准行
        LDR.W    R0,??DataTable6_55
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
//  673           Row_state[bottom_whitebase] = 3; //行的状态标志位
        LDR.W    R0,??DataTable6_55
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_69
        MOVS     R2,#+3
        STRB     R2,[R0, R1]
//  674         }
??Search_WhiteBase_6:
        LDR.W    R0,??DataTable6_56
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.W    ??Search_WhiteBase_15
        LDR.W    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        CMP      R0,#+64
        BCS.W    ??Search_WhiteBase_15
        LDR.W    R0,??DataTable6_66
        LDR.W    R1,??DataTable6_61
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable6_61
        LDRB     R4,[R0, #+0]
        B.N      ??Search_WhiteBase_16
??Search_WhiteBase_17:
        LDR.W    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        LDRB     R1,[R4, R1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.W    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #-2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_18
        LDR.W    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        LDRB     R1,[R4, R1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.W    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #-3]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_18
        LDR.W    R0,??DataTable6_66
        STRB     R4,[R0, #+0]
??Search_WhiteBase_18:
        SUBS     R4,R4,#+1
??Search_WhiteBase_16:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+3
        BCC.N    ??Search_WhiteBase_19
        LDR.W    R0,??DataTable6_66
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_61
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BEQ.N    ??Search_WhiteBase_17
        LDR.W    R0,??DataTable6_66
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_65
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R5,R0
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable6_65
        LDRB     R0,[R0, #+0]
        SUBS     R0,R4,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        CMP      R5,R0
        BGE.N    ??Search_WhiteBase_17
??Search_WhiteBase_19:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.N    ??Search_WhiteBase_20
        LDR.W    R0,??DataTable6_66
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_61
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BNE.N    ??Search_WhiteBase_20
        LDR.W    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        LDRB     R1,[R4, R1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.W    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #-2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_21
        LDR.W    R0,??DataTable6_66
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_20
??Search_WhiteBase_21:
        LDR.W    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.W    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #-2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_22
        LDR.W    R0,??DataTable6_66
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_20
??Search_WhiteBase_22:
        LDR.W    R0,??DataTable6_66
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
??Search_WhiteBase_20:
        LDR.W    R0,??DataTable6_68
        LDR.W    R1,??DataTable6_61
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable6_61
        LDRB     R4,[R0, #+0]
        B.N      ??Search_WhiteBase_23
??Search_WhiteBase_24:
        LDR.W    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        LDRB     R1,[R4, R1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.W    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #+2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_25
        LDR.W    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        LDRB     R1,[R4, R1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.W    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #+3]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_25
        LDR.W    R0,??DataTable6_68
        STRB     R4,[R0, #+0]
??Search_WhiteBase_25:
        ADDS     R4,R4,#+1
??Search_WhiteBase_23:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+156
        BCS.N    ??Search_WhiteBase_26
        LDR.W    R0,??DataTable6_68
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_61
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BEQ.N    ??Search_WhiteBase_24
        LDR.W    R0,??DataTable6_68
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_67
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R5,R0
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable6_67
        LDRB     R0,[R0, #+0]
        SUBS     R0,R4,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        CMP      R5,R0
        BGE.N    ??Search_WhiteBase_24
??Search_WhiteBase_26:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+156
        BNE.N    ??Search_WhiteBase_27
        LDR.W    R0,??DataTable6_68
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable6_61
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BNE.N    ??Search_WhiteBase_27
        LDR.N    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.N    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        LDRB     R1,[R4, R1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.N    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #+2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_28
        LDR.N    R0,??DataTable6_68
        MOVS     R1,#+156
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_27
??Search_WhiteBase_28:
        LDR.N    R0,??DataTable6_58
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        MOVS     R2,#+159
        LDR.N    R3,??DataTable6_57
        MLA      R1,R2,R1,R3
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+1]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R2,??DataTable6_54
        LDRB     R2,[R2, #+0]
        MOVS     R3,#+159
        LDR.N    R5,??DataTable6_57
        MLA      R2,R3,R2,R5
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #+2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_WhiteBase_29
        LDR.N    R0,??DataTable6_68
        MOVS     R1,#+157
        STRB     R1,[R0, #+0]
        B.N      ??Search_WhiteBase_27
??Search_WhiteBase_29:
        LDR.N    R0,??DataTable6_68
        MOVS     R1,#+158
        STRB     R1,[R0, #+0]
??Search_WhiteBase_27:
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDR.N    R2,??DataTable6_66
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDR.N    R2,??DataTable6_68
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        CMP      R0,#+0
        BNE.N    ??Search_WhiteBase_30
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDRB     R0,[R0, R1]
        CMP      R0,#+158
        BCS.N    ??Search_WhiteBase_30
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_70
        LDRB     R1,[R1, R2]
        SUBS     R0,R0,R1
        CMP      R0,#+156
        BLT.N    ??Search_WhiteBase_30
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDR.N    R2,??DataTable6_65
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_71
        LDRB     R1,[R1, R2]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_72
        STRB     R0,[R1, R2]
        B.N      ??Search_WhiteBase_31
??Search_WhiteBase_30:
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDRB     R0,[R0, R1]
        CMP      R0,#+158
        BNE.N    ??Search_WhiteBase_32
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        CMP      R0,#+1
        BCC.N    ??Search_WhiteBase_32
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_70
        LDRB     R1,[R1, R2]
        SUBS     R0,R0,R1
        CMP      R0,#+156
        BLT.N    ??Search_WhiteBase_32
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDR.N    R2,??DataTable6_67
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_71
        LDRB     R1,[R1, R2]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_72
        STRB     R0,[R1, R2]
        B.N      ??Search_WhiteBase_31
??Search_WhiteBase_32:
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDRB     R0,[R0, R1]
        CMP      R0,#+158
        BNE.N    ??Search_WhiteBase_33
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        CMP      R0,#+0
        BNE.N    ??Search_WhiteBase_33
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDR.N    R2,??DataTable6_65
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDR.N    R2,??DataTable6_67
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_72
        LDR.N    R2,??DataTable6_60
        LDRB     R2,[R2, #+0]
        STRB     R2,[R0, R1]
        B.N      ??Search_WhiteBase_31
??Search_WhiteBase_33:
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_71
        LDRB     R1,[R1, R2]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.N    R1,??DataTable6_54
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable6_72
        STRB     R0,[R1, R2]
??Search_WhiteBase_31:
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_70
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_66
        STRB     R0,[R1, #+0]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_71
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_68
        STRB     R0,[R1, #+0]
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_72
        LDRB     R0,[R0, R1]
        LDR.N    R1,??DataTable6_64
        STRB     R0,[R1, #+0]
        LDR.N    R0,??DataTable6_68
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable6_66
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        CMP      R0,#+31
        BGE.W    ??Search_WhiteBase_14
//  675         else
//  676         {
//  677           find_whitebase_flag = 0;
        LDR.N    R0,??DataTable6_56
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  678           current_deal_line++;
        LDR.N    R0,??DataTable6_54
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable6_54
        STRB     R0,[R1, #+0]
        B.N      ??Search_WhiteBase_6
//  679         }
//  680         
//  681   }//while循环的结束
//  682   if(bottom_whitebase > 0)
??Search_WhiteBase_15:
        LDR.N    R0,??DataTable6_55
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BCC.N    ??Search_WhiteBase_34
//  683   {
//  684      for( i = 0 ; i < bottom_whitebase ;i++)
        MOVS     R5,#+0
        B.N      ??Search_WhiteBase_35
//  685      {//对之前的行进行标记
//  686         center_white[i] = MID;  
??Search_WhiteBase_36:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        LDR.N    R0,??DataTable6_72
        MOVS     R1,#+79
        STRB     R1,[R5, R0]
//  687         left_black[i] = MID - 2;   
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        LDR.N    R0,??DataTable6_70
        MOVS     R1,#+77
        STRB     R1,[R5, R0]
//  688         right_black[i] = MID + 2;  
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        LDR.N    R0,??DataTable6_71
        MOVS     R1,#+81
        STRB     R1,[R5, R0]
//  689      }
        ADDS     R5,R5,#+1
??Search_WhiteBase_35:
        LDR.N    R0,??DataTable6_55
        LDRB     R0,[R0, #+0]
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,R0
        BCC.N    ??Search_WhiteBase_36
//  690   }
//  691 }//
??Search_WhiteBase_34:
        POP      {R4-R6,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6:
        DC32     ramp_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_1:
        DC32     0x40040000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_2:
        DC32     0x40040004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_3:
        DC32     0x4004b00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_4:
        DC32     0x400ff094

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_5:
        DC32     0x2bf20

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_6:
        DC32     0x4004a040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_7:
        DC32     0x4004a044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_8:
        DC32     0x40048034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_9:
        DC32     0x4006a000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_10:
        DC32     0x4006a001

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_11:
        DC32     0x4006a00a

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_12:
        DC32     0x4006a003

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_13:
        DC32     0x4006a002

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_14:
        DC32     0x4004b010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_15:
        DC32     0x40049030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_16:
        DC32     0x40049034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_17:
        DC32     0x40049028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_18:
        DC32     0x4004803c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_19:
        DC32     0x40048030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_20:
        DC32     0x40038024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_21:
        DC32     0x4003900c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_22:
        DC32     0x40039014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_23:
        DC32     0x400b800c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_24:
        DC32     0x4003801c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_25:
        DC32     0x40038004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_26:
        DC32     0x40039004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_27:
        DC32     0x400b8004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_28:
        DC32     0x40039008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_29:
        DC32     0x40038008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_30:
        DC32     0x400b8008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_31:
        DC32     0x4003804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_32:
        DC32     0x4003904c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_33:
        DC32     0x400b804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_34:
        DC32     0x40038028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_35:
        DC32     mid_angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_36:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_37:
        DC32     0x40039018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_38:
        DC32     0x400b8010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_39:
        DC32     0x40038000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_40:
        DC32     0x40039000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_41:
        DC32     0x400b8000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_42:
        DC32     0x40064001

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_43:
        DC32     0x4007c008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_44:
        DC32     0x40064000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_45:
        DC32     0x40064006

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_46:
        DC32     0x40064004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_47:
        DC32     0x40064005

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_48:
        DC32     0x4001f000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_49:
        DC32     0x40048044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_50:
        DC32     0x2270000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_51:
        DC32     0x40048004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_52:
        DC32     0x40049018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_53:
        DC32     0x40048040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_54:
        DC32     current_deal_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_55:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_56:
        DC32     find_whitebase_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_57:
        DC32     VideoImage1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_58:
        DC32     OT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_59:
        DC32     BASE_OT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_60:
        DC32     re_white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_61:
        DC32     whitebase_searchstart

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_62:
        DC32     left_whitebase_searchstart

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_63:
        DC32     right_whitebase_searchstart

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_64:
        DC32     white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_65:
        DC32     re_whitepoint_start

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_66:
        DC32     whitepoint_start

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_67:
        DC32     re_whitepoint_end

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_68:
        DC32     whitepoint_end

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_69:
        DC32     Row_state

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_70:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_71:
        DC32     right_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_72:
        DC32     center_white
//  692 
//  693 //------------------------由基准线定出的两边黑线为基准，找出赛道边缘-----------------------// 
//  694 /*本函数的功能定义为找线，为了处理在某些断点的情况能继续在前方找到边沿线，
//  695 只是对于边沿线进行初步的虚构，真正的对赛道两边沿线的处理虚构，主要由下一个函数完成*/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  696 void Search_BlackEdge(void)     
//  697 {   
Search_BlackEdge:
        PUSH     {R3-R7,LR}
//  698   int16 i = 0,j = 0,n = 0, k = 0;
        MOVS     R4,#+0
        MOVS     R6,#+0
        MOVS     R0,#+0
        MOVS     R1,#+0
//  699   int16 un_lost_hang = bottom_whitebase;//这两个变量是用来跟踪记录最近的一行的没有丢点的行，以便于对下一行的状态进行准确的判断.初始值为bottom_whitebase因为第bottom_whitebase行始终判断为没有丢点
        LDR.W    R2,??DataTable7
        LDRB     R5,[R2, #+0]
//  700   deal_start_line = bottom_whitebase + 1;  
        LDR.W    R2,??DataTable7
        LDRB     R2,[R2, #+0]
        ADDS     R2,R2,#+1
        LDR.W    R3,??DataTable7_1
        STRB     R2,[R3, #+0]
//  701   top_whiteline = ROW -1;
        LDR.W    R2,??DataTable7_2
        MOVS     R3,#+64
        STRB     R3,[R2, #+0]
//  702   hang_search_start = white_refer;  //从基准行的中点进行扫描 
        LDR.W    R2,??DataTable7_3
        LDR.W    R3,??DataTable7_4
        LDRB     R3,[R3, #+0]
        STRB     R3,[R2, #+0]
//  703    
//  704   for(i = deal_start_line ; i < ROW ;i++)//对状态标志进行初始化
        LDR.W    R2,??DataTable7_1
        LDRB     R2,[R2, #+0]
        MOVS     R4,R2
        B.N      ??Search_BlackEdge_0
//  705     {
//  706       Row_state[i] = 3;
??Search_BlackEdge_1:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+3
        STRB     R1,[R4, R0]
//  707     }
        ADDS     R4,R4,#+1
??Search_BlackEdge_0:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+65
        BLT.N    ??Search_BlackEdge_1
//  708   
//  709   for(i = deal_start_line ;i < ROW;i++) 
        LDR.W    R0,??DataTable7_1
        LDRB     R4,[R0, #+0]
        B.N      ??Search_BlackEdge_2
??Search_BlackEdge_3:
        ADDS     R4,R4,#+1
??Search_BlackEdge_2:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+65
        BGE.W    ??Search_BlackEdge_4
//  710   {
//  711     //////////////////左右的搜索开始///////////////////////
//  712     //左边搜索
//  713     j = hang_search_start;
        LDR.W    R0,??DataTable7_3
        LDRB     R6,[R0, #+0]
//  714     left_black[i] = hang_search_start;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_5
//  715     while(j >= 2)
//  716     {     
//  717       
//  718       if(VideoImage1[i][j] - VideoImage1[i][j-2] > OT
//  719          && f_abs16(VideoImage1[i][j]-VideoImage1[i][j+1]) < OT && f_abs16(VideoImage1[i][j+1]-VideoImage1[i][j+2]) < OT && VideoImage1[i][j+2]-VideoImage1[i][j+3] < OT)  //滤除边沿噪点
//  720         {
//  721           if(f_abs16(j - left_black[i-1]) < f_abs16(left_black[i] - left_black[i-1]))//滤除干扰
//  722            left_black[i] = j;
//  723         } 
//  724        if(left_black[i] != hang_search_start && (f_abs16(j - left_black[i-1]) > f_abs16(left_black[i] - left_black[i-1])
//  725                                                  || (f_abs16(left_black[i] - left_black[i-1])  < 5 && j<=left_black[i-1] )))
//  726        {
//  727         break;
//  728        }//减少计算量，搜索到最近的一个跳变点，则停止
//  729      
//  730          //当前一个状态是断点的状态时，这个时候当在内部搜索到了跳变沿的时候，则就不进行搜索，若是没有搜到，
//  731           //则再到赛道的两边进行搜索
//  732        if(Row_state[i-1] == 0 || Row_state[i-1] == 2)
//  733           {
//  734             if( j <  left_black[i - 1]  && left_black[i] != hang_search_start) //当前一行为断点状态时，搜索到了点之后，则不允许继续的搜索
//  735              {
//  736               break; 
//  737             }
//  738           }
//  739           j--;
??Search_BlackEdge_6:
        SUBS     R6,R6,#+1
??Search_BlackEdge_5:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+2
        BLT.W    ??Search_BlackEdge_7
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        LDRB     R1,[R6, R1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable7_8
        MLA      R2,R2,R4,R3
        ADDS     R2,R6,R2
        LDRB     R2,[R2, #-2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_8
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable7_8
        MLA      R0,R0,R4,R1
        LDRB     R0,[R6, R0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        ADDS     R1,R6,R1
        LDRB     R1,[R1, #+1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_8
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable7_8
        MLA      R0,R0,R4,R1
        ADDS     R0,R6,R0
        LDRB     R0,[R0, #+1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        ADDS     R1,R6,R1
        LDRB     R1,[R1, #+2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_8
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable7_8
        MLA      R0,R0,R4,R1
        ADDS     R0,R6,R0
        LDRB     R0,[R0, #+2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        ADDS     R1,R6,R1
        LDRB     R1,[R1, #+3]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_8
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R7,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,R0
        BGE.N    ??Search_BlackEdge_8
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        STRB     R6,[R4, R0]
??Search_BlackEdge_8:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BEQ.N    ??Search_BlackEdge_9
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R7,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,R0
        BLT.N    ??Search_BlackEdge_10
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+5
        BGE.N    ??Search_BlackEdge_9
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R0,R6
        BLT.N    ??Search_BlackEdge_9
??Search_BlackEdge_10:
        B.N      ??Search_BlackEdge_7
??Search_BlackEdge_9:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+0
        BEQ.N    ??Search_BlackEdge_11
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+2
        BNE.W    ??Search_BlackEdge_6
??Search_BlackEdge_11:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R6,R0
        BGE.W    ??Search_BlackEdge_6
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BEQ.W    ??Search_BlackEdge_6
//  740     }      //搜索左边沿线的while结束
//  741     //对左边沿线的出界判定
//  742     if(j == 1 && left_black[i] == hang_search_start)      //到达边界了，但是边沿线没有改变时，在搜索范围内没有找到跳变点，则认为是图像依然丢点
??Search_BlackEdge_7:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+1
        BNE.N    ??Search_BlackEdge_12
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BNE.N    ??Search_BlackEdge_12
//  743     {
//  744       if(VideoImage1[i][j] - VideoImage1[i][j-1] > OT)
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        LDRB     R1,[R6, R1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable7_8
        MLA      R2,R2,R4,R3
        ADDS     R2,R6,R2
        LDRB     R2,[R2, #-1]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_13
//  745          left_black[i] = 1;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        MOVS     R1,#+1
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_12
//  746       else
//  747         left_black[i] = 0;
??Search_BlackEdge_13:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        MOVS     R1,#+0
        STRB     R1,[R4, R0]
//  748     }
//  749 
//  750   
//  751     //右边搜索
//  752     j = hang_search_start;
??Search_BlackEdge_12:
        LDR.W    R0,??DataTable7_3
        LDRB     R6,[R0, #+0]
//  753     right_black[i] = hang_search_start;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_14
//  754     
//  755     while( j <=COLUMN-3 )
//  756     { 
//  757       if( VideoImage1[i][j] - VideoImage1[i][j+2] > OT 
//  758          && f_abs16(VideoImage1[i][j]-VideoImage1[i][j-1]) < OT && f_abs16(VideoImage1[i][j-1]-VideoImage1[i][j-2]) < OT && VideoImage1[i][j-2]-VideoImage1[i][j-3] < OT)  //滤除边沿噪点
//  759         {
//  760           if(f_abs16(j-right_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
//  761           {
//  762                 right_black[i] = j ;
//  763           }
//  764         }
//  765       if(right_black[i] != hang_search_start &&( f_abs16(j-right_black[i-1]) > f_abs16(right_black[i] - right_black[i-1])
//  766          ||(f_abs16(right_black[i] - right_black[i-1])<5 && j== right_black[i-1] ) ))//在附近搜索到了点，只要到达了前一行的列位置，则停止
//  767       {
//  768         break;
//  769       }
//  770       if(Row_state[i-1] == 1 || Row_state[i-1] == 2)
//  771           {
//  772             if( j > right_black[i - 1]  && right_black[i] != hang_search_start) //当前一行为断点状态时，搜索到了点之后，则不允许继续的搜索
//  773              {
//  774               break;
//  775             }
//  776             //当搜索到的线大于了
//  777             //if()
//  778           }
//  779         j++;
??Search_BlackEdge_15:
        ADDS     R6,R6,#+1
??Search_BlackEdge_14:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+157
        BGE.W    ??Search_BlackEdge_16
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        LDRB     R1,[R6, R1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable7_8
        MLA      R2,R2,R4,R3
        ADDS     R2,R6,R2
        LDRB     R2,[R2, #+2]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_17
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable7_8
        MLA      R0,R0,R4,R1
        LDRB     R0,[R6, R0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        ADDS     R1,R6,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_17
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable7_8
        MLA      R0,R0,R4,R1
        ADDS     R0,R6,R0
        LDRB     R0,[R0, #-1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        ADDS     R1,R6,R1
        LDRB     R1,[R1, #-2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_17
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable7_8
        MLA      R0,R0,R4,R1
        ADDS     R0,R6,R0
        LDRB     R0,[R0, #-2]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        ADDS     R1,R6,R1
        LDRB     R1,[R1, #-3]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable7_7
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_17
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R7,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,R0
        BGE.N    ??Search_BlackEdge_17
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        STRB     R6,[R4, R0]
??Search_BlackEdge_17:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BEQ.N    ??Search_BlackEdge_18
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R7,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SUBS     R0,R6,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,R0
        BLT.N    ??Search_BlackEdge_19
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+5
        BGE.N    ??Search_BlackEdge_18
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R6,R0
        BNE.N    ??Search_BlackEdge_18
??Search_BlackEdge_19:
        B.N      ??Search_BlackEdge_16
??Search_BlackEdge_18:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+1
        BEQ.N    ??Search_BlackEdge_20
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+2
        BNE.W    ??Search_BlackEdge_15
??Search_BlackEdge_20:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R0,R6
        BGE.W    ??Search_BlackEdge_15
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BEQ.W    ??Search_BlackEdge_15
//  780     }    //右边的while搜索结束
//  781   
//  782     if(j == COLUMN-2 && right_black[i] == hang_search_start)
??Search_BlackEdge_16:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+157
        BNE.N    ??Search_BlackEdge_21
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable7_3
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BNE.N    ??Search_BlackEdge_21
//  783     {
//  784       if( VideoImage1[i][j] - VideoImage1[i][j+1] > OT)
        LDR.W    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable7_8
        MLA      R1,R1,R4,R2
        LDRB     R1,[R6, R1]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable7_8
        MLA      R2,R2,R4,R3
        ADDS     R2,R6,R2
        LDRB     R2,[R2, #+1]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.N    ??Search_BlackEdge_22
//  785          right_black[i] = COLUMN - 2 ;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        MOVS     R1,#+157
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_21
//  786       else
//  787          right_black[i] = COLUMN - 1 ;
??Search_BlackEdge_22:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        MOVS     R1,#+158
        STRB     R1,[R4, R0]
//  788     }
//  789     ///////////////////////左右的搜索结束//////////////////////////
//  790     
//  791     
//  792     //  /////////////赛道的状态标记开始////////////////////////////////
//  793   if(i >= deal_start_line)//只是对处于控制区域的边界进行处理
??Search_BlackEdge_21:
        LDR.W    R0,??DataTable7_1
        LDRB     R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R4,R0
        BLT.W    ??Search_BlackEdge_23
//  794     {    
//  795       //当图像的边沿到达了边界的时候，判定为丢点      ---------------------丢点的第一次判断
//  796       if((left_black[i] <= 1  || left_black[i] >= COLUMN-2 ) && right_black[i] >= 1 && right_black[i] <= COLUMN-2)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+155
        BCC.N    ??Search_BlackEdge_24
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+1
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+157
        BCS.N    ??Search_BlackEdge_24
//  797       {
//  798            Row_state[i] =0;//左边丢点
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+0
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_25
//  799       }
//  800       else if((left_black[i] >= 1 && left_black[i] <= COLUMN-2 ) && (right_black[i] <= 1 || right_black[i] >= COLUMN-2))
??Search_BlackEdge_24:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+1
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+157
        BCS.N    ??Search_BlackEdge_26
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+155
        BCC.N    ??Search_BlackEdge_26
//  801       {
//  802            Row_state[i] = 1;//右边丢点
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+1
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_25
//  803       } 
//  804       else if((left_black[i] <= 1 || left_black[i] >= COLUMN-2 ) && (right_black[i] <= 1 || right_black[i] >= COLUMN-2))
??Search_BlackEdge_26:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+155
        BCC.N    ??Search_BlackEdge_27
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+155
        BCC.N    ??Search_BlackEdge_27
//  805       {
//  806            Row_state[i] = 2;//两边都边丢点
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+2
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_25
//  807       }
//  808       else
//  809       {
//  810            Row_state[i] = 3;//两边都边没有丢点
??Search_BlackEdge_27:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+3
        STRB     R1,[R4, R0]
//  811       }
//  812       
//  813         //对两边沿线的状态进行判断//---------------------------丢点的第二次判断
//  814       //注意这里的判断必须要分两种情况一个是跳变点的状态，其次是前一行的状态（前一行的状态的不同需要作出不同的处理），
//  815       if((right_black[i] - left_black[i])-(right_black[i-1] - left_black[i-1])> 8)//  若是3的话可能出现误判//这里采用绝对值限制是为了防止噪点
??Search_BlackEdge_25:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        LDRB     R1,[R4, R1]
        SUBS     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        UXTAB    R0,R0,R1
        CMP      R0,#+9
        BLT.W    ??Search_BlackEdge_28
//  816       {
//  817          if(( f_abs16(left_black[i] - left_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
//  818             && f_abs16(left_black[i] - left_black[i-1]) <= 4)//左边的突变小于右边的  说明右边的点发生了突变
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R6,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,R0
        BGE.N    ??Search_BlackEdge_29
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+5
        BGE.N    ??Search_BlackEdge_29
//  819          {
//  820            if(Row_state[i] == 0 ||Row_state[i] == 2)//对于第一次进行一个判断
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+0
        BEQ.N    ??Search_BlackEdge_30
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+2
        BNE.N    ??Search_BlackEdge_31
//  821               Row_state[i] = 2;
??Search_BlackEdge_30:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+2
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_32
//  822            else //if(Row_state[i] == 1 ||Row_state[i] == 3)
//  823            {
//  824              Row_state[i] = 1;//1表示的是只有右边丢点
??Search_BlackEdge_31:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+1
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_32
//  825            }
//  826          } 
//  827         else if(( f_abs16(left_black[i] - left_black[i-1]) > f_abs16(right_black[i] - right_black[i-1]))
//  828             && f_abs16(right_black[i] - right_black[i-1]) <= 4)//左边的突变小于右边的  说明右边的点发生了突变
??Search_BlackEdge_29:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R6,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,R0
        BGE.N    ??Search_BlackEdge_33
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+5
        BGE.N    ??Search_BlackEdge_33
//  829          {
//  830            if(Row_state[i] == 1||Row_state[i] == 2)//对于第一次进行一个判断
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+1
        BEQ.N    ??Search_BlackEdge_34
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+2
        BNE.N    ??Search_BlackEdge_35
//  831               Row_state[i] = 2;//0表示的是左边丢点,而右边没有丢点
??Search_BlackEdge_34:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+2
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_32
//  832            else
//  833               Row_state[i] = 0;//0表示的是只有左边丢点
??Search_BlackEdge_35:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+0
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_32
//  834          }
//  835          else 
//  836          {
//  837            Row_state[i] = 2;//2表示的是两边都丢点
??Search_BlackEdge_33:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+2
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_32
//  838          }
//  839       }
//  840       else 
//  841       {
//  842         if(Row_state[i-1] == 0)//左边丢点
??Search_BlackEdge_28:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+0
        BNE.N    ??Search_BlackEdge_36
//  843         {
//  844           if((right_black[i] - left_black[i]) - (right_black[un_lost_hang] - left_black[un_lost_hang]) > 6)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        LDRB     R1,[R4, R1]
        SUBS     R0,R0,R1
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        LDR.W    R1,??DataTable7_9
        LDRB     R1,[R5, R1]
        SUBS     R0,R0,R1
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        LDR.W    R1,??DataTable7_6
        LDRB     R1,[R5, R1]
        ADDS     R0,R0,R1
        CMP      R0,#+7
        BLT.N    ??Search_BlackEdge_32
//  845           {
//  846             if(Row_state[i] == 1 ||Row_state[i] == 2)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+1
        BEQ.N    ??Search_BlackEdge_37
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+2
        BNE.N    ??Search_BlackEdge_38
//  847             {
//  848               Row_state[i] = 2;
??Search_BlackEdge_37:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+2
        STRB     R1,[R4, R0]
//  849             }
//  850             if(Row_state[i] == 3)
??Search_BlackEdge_38:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+3
        BNE.N    ??Search_BlackEdge_32
//  851             {
//  852               Row_state[i] = 0;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+0
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_32
//  853             }
//  854           }
//  855         }
//  856         
//  857        else if(Row_state[i-1] == 1)//左边丢点
??Search_BlackEdge_36:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+1
        BNE.N    ??Search_BlackEdge_32
//  858         {
//  859           if((right_black[i] - left_black[i]) - (right_black[un_lost_hang] - left_black[un_lost_hang]) > 6)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        LDRB     R1,[R4, R1]
        SUBS     R0,R0,R1
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        LDR.W    R1,??DataTable7_9
        LDRB     R1,[R5, R1]
        SUBS     R0,R0,R1
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        LDR.W    R1,??DataTable7_6
        LDRB     R1,[R5, R1]
        ADDS     R0,R0,R1
        CMP      R0,#+7
        BLT.N    ??Search_BlackEdge_32
//  860           {
//  861             if(Row_state[i] == 0 ||Row_state[i] == 2)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+0
        BEQ.N    ??Search_BlackEdge_39
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+2
        BNE.N    ??Search_BlackEdge_40
//  862             {
//  863               Row_state[i] = 2;
??Search_BlackEdge_39:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+2
        STRB     R1,[R4, R0]
//  864             }
//  865            if(Row_state[i] == 3)
??Search_BlackEdge_40:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+3
        BNE.N    ??Search_BlackEdge_32
//  866              Row_state[i] = 1;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        MOVS     R1,#+1
        STRB     R1,[R4, R0]
//  867           }
//  868         }
//  869       }
//  870       
//  871     //记录最近的都没有丢点的行
//  872       if( Row_state[i] == 3)
??Search_BlackEdge_32:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+3
        BNE.N    ??Search_BlackEdge_41
//  873       {
//  874         un_lost_hang = i;
        MOVS     R5,R4
//  875       }
//  876       ////////////左右边沿标记结束/////////////////////////
//  877       
//  878       //前面对赛道进行了状态的判断，这里做出初步的拟合
//  879       if(Row_state[i] == 0)  //左边丢点
??Search_BlackEdge_41:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+0
        BNE.N    ??Search_BlackEdge_42
//  880       {
//  881         if(right_black[i]- (right_black[i-1] - left_black[i-1]) <= 0 )//限幅
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        UXTAB    R0,R0,R1
        CMP      R0,#+1
        BGE.N    ??Search_BlackEdge_43
//  882           left_black[i]=0;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_6
        MOVS     R1,#+0
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_23
//  883         else
//  884         left_black[i] = right_black[i] - (right_black[i-1] - left_black[i-1]);//加上1是由于下向上图像在宽度在减小的原因
??Search_BlackEdge_43:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        STRB     R0,[R4, R1]
        B.N      ??Search_BlackEdge_23
//  885       }      
//  886       else if(Row_state[i] == 1)
??Search_BlackEdge_42:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+1
        BNE.N    ??Search_BlackEdge_44
//  887       {
//  888         if(left_black[i] + (right_black[i-1] - left_black[i-1]) >= COLUMN-1)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        CMP      R0,#+158
        BLT.N    ??Search_BlackEdge_45
//  889           right_black[i] = COLUMN-1;   
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_9
        MOVS     R1,#+158
        STRB     R1,[R4, R0]
        B.N      ??Search_BlackEdge_23
//  890         else
//  891           right_black[i] = left_black[i] + (right_black[i-1] - left_black[i-1]);//
??Search_BlackEdge_45:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_6
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        STRB     R0,[R4, R1]
        B.N      ??Search_BlackEdge_23
//  892       }
//  893       else if(Row_state[i] == 2)
??Search_BlackEdge_44:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_5
        LDRB     R0,[R4, R0]
        CMP      R0,#+2
        BNE.N    ??Search_BlackEdge_23
//  894       {
//  895          left_black[i] = left_black[i-1];
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_6
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        STRB     R0,[R4, R1]
//  896          right_black[i] = right_black[i-1];
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_9
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        STRB     R0,[R4, R1]
//  897        }
//  898     } 
//  899     
//  900     
//  901     hang_search_start = (right_black[i] + left_black[i])/2; 
??Search_BlackEdge_23:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        LDRB     R1,[R4, R1]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.N    R1,??DataTable7_3
        STRB     R0,[R1, #+0]
//  902     //////////////////左右的处理结束///////////////////////////// 
//  903     
//  904     ///对最高有效行的判断/////////////判断一//////////////////
//  905 
//  906 
//  907     if( i> 20 &&i<=top_whiteline && right_black[i] -  left_black[i] < 3*(ROW-i)/5+ WHITE_TOP_WHITELINE_POINT 
//  908        && (right_black[i-1] -  left_black[i-1]) <  3*(ROW-i)/5 + WHITE_TOP_WHITELINE_POINT 
//  909        ) //只判定一次&& top_whiteline >= ROW-1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+21
        BLT.N    ??Search_BlackEdge_46
        LDR.N    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BLT.N    ??Search_BlackEdge_46
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        RSBS     R0,R4,#+65
        MOVS     R1,#+3
        MULS     R0,R1,R0
        MOVS     R1,#+5
        SDIV     R0,R0,R1
        ADDS     R0,R0,#+20
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        LDRB     R1,[R4, R1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R2,??DataTable7_6
        LDRB     R2,[R4, R2]
        SUBS     R1,R1,R2
        CMP      R1,R0
        BGE.N    ??Search_BlackEdge_46
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        RSBS     R0,R4,#+65
        MOVS     R1,#+3
        MULS     R0,R1,R0
        MOVS     R1,#+5
        SDIV     R0,R0,R1
        ADDS     R0,R0,#+20
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_9
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R2,??DataTable7_6
        ADDS     R2,R4,R2
        LDRB     R2,[R2, #-1]
        SUBS     R1,R1,R2
        CMP      R1,R0
        BGE.N    ??Search_BlackEdge_46
//  910     { 
//  911      // if(i<ROW-1) while(1){}
//  912       top_whiteline = i;
        LDR.N    R0,??DataTable7_2
        STRB     R4,[R0, #+0]
//  913     }
//  914     /////////////////////判断二////////////////////////
//  915     center_white[i] = (right_black[i] + left_black[i])/2; 
??Search_BlackEdge_46:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable7_9
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.N    R1,??DataTable7_6
        LDRB     R1,[R4, R1]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8
        STRB     R0,[R4, R1]
//  916     if(i>10 && i < ROW-1 &&top_whiteline >= ROW-1 &&( VideoImage2[i-1][center_white[i]] - VideoImage2[i+1][center_white[i]] > OT - 10) 
//  917        &&  VideoImage2[i-1][center_white[i]-1] - VideoImage2[i+1][center_white[i]-1] > OT - 10
//  918               &&  VideoImage2[i-1][center_white[i]+1] - VideoImage2[i + 1][center_white[i]+1] > OT - 10 )  //最高行的判断用原始图像
        SUBS     R0,R4,#+11
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+53
        BCS.W    ??Search_BlackEdge_3
        LDR.N    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+64
        BCC.W    ??Search_BlackEdge_3
        LDR.N    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8
        LDRB     R1,[R4, R1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable8_1
        MLA      R2,R2,R4,R3
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-159]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R2,??DataTable8
        LDRB     R2,[R4, R2]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R3,#+159
        LDR.W    R6,??DataTable8_1
        MLA      R3,R3,R4,R6
        ADDS     R2,R2,R3
        LDRB     R2,[R2, #+159]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.W    ??Search_BlackEdge_3
        LDR.N    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8
        LDRB     R1,[R4, R1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable8_1
        MLA      R2,R2,R4,R3
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-160]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R2,??DataTable8
        LDRB     R2,[R4, R2]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R3,#+159
        LDR.W    R6,??DataTable8_1
        MLA      R3,R3,R4,R6
        ADDS     R2,R2,R3
        LDRB     R2,[R2, #+158]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.W    ??Search_BlackEdge_3
        LDR.N    R0,??DataTable7_7
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable8
        LDRB     R1,[R4, R1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R2,#+159
        LDR.W    R3,??DataTable8_1
        MLA      R2,R2,R4,R3
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-158]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R2,??DataTable8
        LDRB     R2,[R4, R2]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        MOVS     R3,#+159
        LDR.W    R6,??DataTable8_1
        MLA      R3,R3,R4,R6
        ADDS     R2,R2,R3
        LDRB     R2,[R2, #+160]
        SUBS     R1,R1,R2
        CMP      R0,R1
        BGE.W    ??Search_BlackEdge_3
//  919     {
//  920       //用这种方式有一个弊端就是，图像存在一个突变，无法真实的反应赛道，特别是60度和50度弯道的微小差别，这里对其经行修补
//  921        top_whiteline = i-1;
        MOVS     R0,R4
        SUBS     R0,R0,#+1
        LDR.N    R1,??DataTable7_2
        STRB     R0,[R1, #+0]
//  922       for( n = top_whiteline; n >  top_whiteline - 7;n--)
        LDR.N    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        B.N      ??Search_BlackEdge_47
//  923       {
//  924         if(left_black[n] <= 1)
//  925         {
//  926           for( k = n; k <= top_whiteline ; k++)
//  927           {
//  928              left_black[k] = 0;
//  929              //进行规划后的行的状态需要重新的定义
//  930            if(Row_state[k] == 1 ||Row_state[k] == 2)
//  931             {
//  932               Row_state[k] = 2;
//  933             }
//  934             if(Row_state[k] == 3)
//  935             {
//  936               Row_state[k] = 0;
//  937             }
//  938           }
//  939         //  break;  //这里还不能用break。因为有的时候可能会有一个点跳出来了。如 0 1 0 0 0
//  940         }
//  941         else if(right_black[n] >= COLUMN-2)
//  942         {
//  943           for( k = n; k <= top_whiteline ; k++)
//  944           {
//  945              right_black[k] = COLUMN-1;
??Search_BlackEdge_48:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_9
        MOVS     R3,#+158
        STRB     R3,[R1, R2]
//  946              
//  947            if(Row_state[k] == 0 ||Row_state[k] == 2)
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        LDRB     R2,[R1, R2]
        CMP      R2,#+0
        BEQ.N    ??Search_BlackEdge_49
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        LDRB     R2,[R1, R2]
        CMP      R2,#+2
        BNE.N    ??Search_BlackEdge_50
//  948             {
//  949               Row_state[k] = 2;
??Search_BlackEdge_49:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        MOVS     R3,#+2
        STRB     R3,[R1, R2]
//  950             }
//  951             if(Row_state[k] == 3)
??Search_BlackEdge_50:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        LDRB     R2,[R1, R2]
        CMP      R2,#+3
        BNE.N    ??Search_BlackEdge_51
//  952             {
//  953               Row_state[k] = 1;
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        MOVS     R3,#+1
        STRB     R3,[R1, R2]
//  954             }
//  955           }
??Search_BlackEdge_51:
        ADDS     R1,R1,#+1
??Search_BlackEdge_52:
        LDR.N    R2,??DataTable7_2
        LDRB     R2,[R2, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R2,R1
        BGE.N    ??Search_BlackEdge_48
??Search_BlackEdge_53:
        SUBS     R0,R0,#+1
??Search_BlackEdge_47:
        LDR.N    R1,??DataTable7_2
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+7
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BGE.W    ??Search_BlackEdge_3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable7_6
        LDRB     R1,[R0, R1]
        CMP      R1,#+2
        BCS.N    ??Search_BlackEdge_54
        MOVS     R1,R0
??Search_BlackEdge_55:
        LDR.N    R2,??DataTable7_2
        LDRB     R2,[R2, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R2,R1
        BLT.N    ??Search_BlackEdge_53
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_6
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        LDRB     R2,[R1, R2]
        CMP      R2,#+1
        BEQ.N    ??Search_BlackEdge_56
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        LDRB     R2,[R1, R2]
        CMP      R2,#+2
        BNE.N    ??Search_BlackEdge_57
??Search_BlackEdge_56:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        MOVS     R3,#+2
        STRB     R3,[R1, R2]
??Search_BlackEdge_57:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        LDRB     R2,[R1, R2]
        CMP      R2,#+3
        BNE.N    ??Search_BlackEdge_58
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R2,??DataTable7_5
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
??Search_BlackEdge_58:
        ADDS     R1,R1,#+1
        B.N      ??Search_BlackEdge_55
??Search_BlackEdge_54:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable7_9
        LDRB     R1,[R0, R1]
        CMP      R1,#+157
        BCC.N    ??Search_BlackEdge_53
        MOVS     R1,R0
        B.N      ??Search_BlackEdge_52
//  956           //break;
//  957         }
//  958       }
//  959     }
//  960 
//  961   }//for循环的结束
//  962   
//  963   if(top_whiteline+1 < ROW-1)
??Search_BlackEdge_4:
        LDR.N    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+64
        BCS.N    ??Search_BlackEdge_59
//  964   {
//  965     for(n= top_whiteline+1;n<ROW; n++)
        LDR.N    R0,??DataTable7_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        B.N      ??Search_BlackEdge_60
//  966     {
//  967         center_white[n] = MID;  
??Search_BlackEdge_61:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable8
        MOVS     R2,#+79
        STRB     R2,[R0, R1]
//  968         left_black[n] = MID - 2;   
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable7_6
        MOVS     R2,#+77
        STRB     R2,[R0, R1]
//  969         right_black[n] = MID + 2;  
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable7_9
        MOVS     R2,#+81
        STRB     R2,[R0, R1]
//  970     }
        ADDS     R0,R0,#+1
??Search_BlackEdge_60:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+65
        BLT.N    ??Search_BlackEdge_61
//  971   } 
//  972   
//  973 }
??Search_BlackEdge_59:
        POP      {R0,R4-R7,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_1:
        DC32     deal_start_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_2:
        DC32     top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_3:
        DC32     hang_search_start

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_4:
        DC32     white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_5:
        DC32     Row_state

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_6:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_7:
        DC32     OT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_8:
        DC32     VideoImage1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7_9:
        DC32     right_black
//  974 
//  975 //------------------------通过找出来的赛道对，两边沿线进行处理和虚构并拟合出中心线----------------------//
//  976 /*对赛道进行拉线连接，左右各自连接自己的，然后对于中线，利用赛道的状态标志，再进行一次拟合
//  977  0  表示左边沿线断点
//  978  1  表示右边沿线断点
//  979  2  表示两边都断了 
//  980 到达边界后不能直接的拉线了
//  981 对于虚线和十字道路的处理，只要保证两点就行了，即1、保证能找到在跑道上的点；2、保证对赛道的行状态的记录绝对的正确
//  982 */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  983 void Deal_BlackEdge(void)
//  984 { 
Deal_BlackEdge:
        PUSH     {R4-R7}
//  985   int16 i=0,k=0;
        MOVS     R0,#+0
        MOVS     R1,#+0
//  986   uint8 un_out_hang = bottom_whitebase ;
        LDR.W    R2,??DataTable8_2
        LDRB     R2,[R2, #+0]
//  987   uint8 lost_start_line=0;
        MOVS     R3,#+0
//  988   uint8 lost_end_line=0;
        MOVS     R4,#+0
//  989   left_top_whiteline = top_whiteline;
        LDR.W    R5,??DataTable8_3
        LDR.W    R6,??DataTable8_4
        LDRB     R6,[R6, #+0]
        STRB     R6,[R5, #+0]
//  990   right_top_whiteline = top_whiteline;
        LDR.W    R5,??DataTable8_5
        LDR.W    R6,??DataTable8_4
        LDRB     R6,[R6, #+0]
        STRB     R6,[R5, #+0]
//  991 
//  992   //图像的突变可能是噪点的出现，这里可以试着对Row_state[i]经行一下中值滤波
//  993   //对Row_state[i]滤波
//  994  for( i=bottom_whitebase + 1;i < top_whiteline-2;i++)
        LDR.W    R5,??DataTable8_2
        LDRB     R5,[R5, #+0]
        ADDS     R5,R5,#+1
        MOVS     R0,R5
        B.N      ??Deal_BlackEdge_0
//  995  {
//  996    if(Row_state[i-1] == Row_state[i+1] && Row_state[i-1] != Row_state[i])
??Deal_BlackEdge_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_6
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #+1]
        CMP      R3,R4
        BNE.N    ??Deal_BlackEdge_2
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_6
        LDRB     R4,[R0, R4]
        CMP      R3,R4
        BEQ.N    ??Deal_BlackEdge_2
//  997    {
//  998      Row_state[i] = Row_state[i-1];
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_6
        STRB     R3,[R0, R4]
//  999    }
// 1000  }
??Deal_BlackEdge_2:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_4
        LDRB     R3,[R3, #+0]
        SUBS     R3,R3,#+2
        CMP      R0,R3
        BLT.N    ??Deal_BlackEdge_1
// 1001   
// 1002   
// 1003   
// 1004   for( i=bottom_whitebase+1 ;i < top_whiteline-1;i++)
        LDR.W    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        B.N      ??Deal_BlackEdge_3
??Deal_BlackEdge_4:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_3:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_4
        LDRB     R3,[R3, #+0]
        SUBS     R3,R3,#+1
        CMP      R0,R3
        BGE.W    ??Deal_BlackEdge_5
// 1005   {
// 1006     lost_start_line = 0;
        MOVS     R3,#+0
// 1007     lost_end_line = 0;
        MOVS     R4,#+0
// 1008     
// 1009     if( (Row_state[i-1] == 1 || Row_state[i-1] == 3) && (Row_state[i] == 0 || Row_state[i] == 2))   //判断左边第i 点是否丢点   
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        ADDS     R5,R0,R5
        LDRB     R5,[R5, #-1]
        CMP      R5,#+1
        BEQ.N    ??Deal_BlackEdge_6
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        CMP      R3,#+3
        BNE.N    ??Deal_BlackEdge_4
??Deal_BlackEdge_6:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        LDRB     R3,[R0, R3]
        CMP      R3,#+0
        BEQ.N    ??Deal_BlackEdge_7
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        LDRB     R3,[R0, R3]
        CMP      R3,#+2
        BNE.N    ??Deal_BlackEdge_4
// 1010     {
// 1011        lost_start_line = i - 1;//记录丢点的前一行
??Deal_BlackEdge_7:
        MOVS     R3,R0
        SUBS     R3,R3,#+1
        B.N      ??Deal_BlackEdge_8
// 1012        while(i < top_whiteline-1)
// 1013         {
// 1014           if((Row_state[i] == 1 || Row_state[i] == 3) && (Row_state[i+1] == 1 || Row_state[i+1] == 3))  //
// 1015           {
// 1016             if(lost_end_line >= top_whiteline -2)
// 1017               lost_end_line = i;
// 1018             else
// 1019               lost_end_line = i + 1;
// 1020             
// 1021             break; 
// 1022           }
// 1023            i++;//i++必须要在if的判断之后进行，否则会导致出错
??Deal_BlackEdge_9:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_8:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_4
        LDRB     R5,[R5, #+0]
        SUBS     R5,R5,#+1
        CMP      R0,R5
        BGE.N    ??Deal_BlackEdge_10
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        LDRB     R5,[R0, R5]
        CMP      R5,#+1
        BEQ.N    ??Deal_BlackEdge_11
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        LDRB     R5,[R0, R5]
        CMP      R5,#+3
        BNE.N    ??Deal_BlackEdge_9
??Deal_BlackEdge_11:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        ADDS     R5,R0,R5
        LDRB     R5,[R5, #+1]
        CMP      R5,#+1
        BEQ.N    ??Deal_BlackEdge_12
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        ADDS     R5,R0,R5
        LDRB     R5,[R5, #+1]
        CMP      R5,#+3
        BNE.N    ??Deal_BlackEdge_9
??Deal_BlackEdge_12:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R5,??DataTable8_4
        LDRB     R5,[R5, #+0]
        SUBS     R5,R5,#+2
        CMP      R4,R5
        BLT.N    ??Deal_BlackEdge_13
        MOVS     R4,R0
        B.N      ??Deal_BlackEdge_14
??Deal_BlackEdge_13:
        ADDS     R4,R0,#+1
// 1024         }
// 1025        if(lost_end_line !=0)
??Deal_BlackEdge_14:
??Deal_BlackEdge_10:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BEQ.N    ??Deal_BlackEdge_15
// 1026        {  
// 1027          for(k = lost_start_line+1; k< lost_end_line;k++)
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        ADDS     R1,R3,#+1
??Deal_BlackEdge_16:
        MOVS     R5,R4
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        CMP      R1,R5
        BGE.N    ??Deal_BlackEdge_4
// 1028          {
// 1029           left_black[k] = left_black[lost_start_line] + (k -lost_start_line)*(left_black[lost_end_line]-left_black[lost_start_line])/(lost_end_line - lost_start_line);  
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R5,??DataTable8_7
        LDRB     R5,[R3, R5]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        SUBS     R6,R1,R3
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R7,??DataTable8_7
        LDRB     R7,[R4, R7]
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R12,??DataTable8_7
        LDRB     R12,[R3, R12]
        SUBS     R7,R7,R12
        MULS     R6,R7,R6
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        SUBS     R7,R4,R3
        SDIV     R6,R6,R7
        ADDS     R5,R6,R5
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R6,??DataTable8_7
        STRB     R5,[R1, R6]
// 1030          }
        ADDS     R1,R1,#+1
        B.N      ??Deal_BlackEdge_16
// 1031        }   
// 1032       else if(lost_end_line ==0 && lost_start_line >  top_whiteline/2 && lost_start_line <ROW-1  ) //当最顶行的点到达边沿的时候，不判定
??Deal_BlackEdge_15:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.W    ??Deal_BlackEdge_4
        LDR.W    R4,??DataTable8_4
        LDRB     R4,[R4, #+0]
        MOVS     R5,#+2
        SDIV     R4,R4,R5
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R4,R3
        BGE.W    ??Deal_BlackEdge_4
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R3,#+64
        BCS.W    ??Deal_BlackEdge_4
// 1033        {   
// 1034         if( left_black[top_whiteline] > 1)
        LDR.W    R4,??DataTable8_4
        LDRB     R4,[R4, #+0]
        LDR.W    R5,??DataTable8_7
        LDRB     R4,[R4, R5]
        CMP      R4,#+2
        BCC.W    ??Deal_BlackEdge_4
// 1035          {
// 1036            left_top_whiteline = lost_start_line;
        LDR.W    R0,??DataTable8_3
        STRB     R3,[R0, #+0]
// 1037            break;
// 1038         }
// 1039        }
// 1040          
// 1041     }
// 1042   }
// 1043   
// 1044   //右边
// 1045   for( i=bottom_whitebase ;i < top_whiteline-1;i++)
??Deal_BlackEdge_5:
        LDR.W    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        B.N      ??Deal_BlackEdge_17
??Deal_BlackEdge_18:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_17:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_4
        LDRB     R3,[R3, #+0]
        SUBS     R3,R3,#+1
        CMP      R0,R3
        BGE.W    ??Deal_BlackEdge_19
// 1046   {
// 1047     lost_start_line = 0;
        MOVS     R3,#+0
// 1048     lost_end_line = 0;
        MOVS     R4,#+0
// 1049     if( (Row_state[i-1] == 0 || Row_state[i-1] == 3) && (Row_state[i] == 1 || Row_state[i] == 2))//判断右边第i 点是否丢点
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        ADDS     R5,R0,R5
        LDRB     R5,[R5, #-1]
        CMP      R5,#+0
        BEQ.N    ??Deal_BlackEdge_20
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        CMP      R3,#+3
        BNE.N    ??Deal_BlackEdge_18
??Deal_BlackEdge_20:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        LDRB     R3,[R0, R3]
        CMP      R3,#+1
        BEQ.N    ??Deal_BlackEdge_21
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_6
        LDRB     R3,[R0, R3]
        CMP      R3,#+2
        BNE.N    ??Deal_BlackEdge_18
// 1050     {
// 1051         lost_start_line = i - 1;//记录丢点的前一行
??Deal_BlackEdge_21:
        MOVS     R3,R0
        SUBS     R3,R3,#+1
        B.N      ??Deal_BlackEdge_22
// 1052        while(i< top_whiteline-1)
// 1053         {
// 1054           //连续的两行找到了点则认为找到了连接点
// 1055           if((Row_state[i] == 0 || Row_state[i] == 3) && (Row_state[i+1] == 0 || Row_state[i+1] == 3))
// 1056           {
// 1057             if(lost_end_line >= top_whiteline -2)
// 1058               lost_end_line = i;
// 1059             else
// 1060               lost_end_line = i + 1;
// 1061             break;
// 1062           }  
// 1063           i++;
??Deal_BlackEdge_23:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_22:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_4
        LDRB     R5,[R5, #+0]
        SUBS     R5,R5,#+1
        CMP      R0,R5
        BGE.N    ??Deal_BlackEdge_24
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        LDRB     R5,[R0, R5]
        CMP      R5,#+0
        BEQ.N    ??Deal_BlackEdge_25
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        LDRB     R5,[R0, R5]
        CMP      R5,#+3
        BNE.N    ??Deal_BlackEdge_23
??Deal_BlackEdge_25:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        ADDS     R5,R0,R5
        LDRB     R5,[R5, #+1]
        CMP      R5,#+0
        BEQ.N    ??Deal_BlackEdge_26
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R5,??DataTable8_6
        ADDS     R5,R0,R5
        LDRB     R5,[R5, #+1]
        CMP      R5,#+3
        BNE.N    ??Deal_BlackEdge_23
??Deal_BlackEdge_26:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R5,??DataTable8_4
        LDRB     R5,[R5, #+0]
        SUBS     R5,R5,#+2
        CMP      R4,R5
        BLT.N    ??Deal_BlackEdge_27
        MOVS     R4,R0
        B.N      ??Deal_BlackEdge_28
??Deal_BlackEdge_27:
        ADDS     R4,R0,#+1
// 1064         }
// 1065        if(lost_end_line !=0)
??Deal_BlackEdge_28:
??Deal_BlackEdge_24:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BEQ.N    ??Deal_BlackEdge_29
// 1066        {
// 1067          for(k = lost_start_line+1; k< lost_end_line;k++)
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        ADDS     R1,R3,#+1
??Deal_BlackEdge_30:
        MOVS     R5,R4
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        CMP      R1,R5
        BGE.N    ??Deal_BlackEdge_18
// 1068          {
// 1069           right_black[k] = right_black[lost_start_line] + (k -lost_start_line)*(right_black[lost_end_line]-right_black[lost_start_line])/(lost_end_line - lost_start_line);  
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R5,??DataTable8_8
        LDRB     R5,[R3, R5]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        SUBS     R6,R1,R3
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R7,??DataTable8_8
        LDRB     R7,[R4, R7]
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDR.W    R12,??DataTable8_8
        LDRB     R12,[R3, R12]
        SUBS     R7,R7,R12
        MULS     R6,R7,R6
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        SUBS     R7,R4,R3
        SDIV     R6,R6,R7
        ADDS     R5,R6,R5
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R6,??DataTable8_8
        STRB     R5,[R1, R6]
// 1070          }
        ADDS     R1,R1,#+1
        B.N      ??Deal_BlackEdge_30
// 1071        }
// 1072        else if(lost_end_line ==0 && lost_start_line > top_whiteline/2 && lost_start_line <ROW-1 )
??Deal_BlackEdge_29:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.W    ??Deal_BlackEdge_18
        LDR.W    R4,??DataTable8_4
        LDRB     R4,[R4, #+0]
        MOVS     R5,#+2
        SDIV     R4,R4,R5
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R4,R3
        BGE.W    ??Deal_BlackEdge_18
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R3,#+64
        BCS.W    ??Deal_BlackEdge_18
// 1073        {   
// 1074          if(right_black[top_whiteline] < COLUMN -2)
        LDR.W    R4,??DataTable8_4
        LDRB     R4,[R4, #+0]
        LDR.W    R5,??DataTable8_8
        LDRB     R4,[R4, R5]
        CMP      R4,#+157
        BCS.W    ??Deal_BlackEdge_18
// 1075          {
// 1076            right_top_whiteline  = lost_start_line;
        LDR.W    R0,??DataTable8_5
        STRB     R3,[R0, #+0]
// 1077            break;
// 1078         }
// 1079        }
// 1080        
// 1081     }
// 1082   }
// 1083  
// 1084   if(right_top_whiteline > left_top_whiteline)
??Deal_BlackEdge_19:
        LDR.W    R0,??DataTable8_3
        LDRB     R0,[R0, #+0]
        LDR.W    R3,??DataTable8_5
        LDRB     R3,[R3, #+0]
        CMP      R0,R3
        BCS.N    ??Deal_BlackEdge_31
// 1085   {
// 1086     top_whiteline = right_top_whiteline;//除了定义最高行以外，还要对丢线的那一边做补线处理,这里是右边
        LDR.W    R0,??DataTable8_4
        LDR.W    R3,??DataTable8_5
        LDRB     R3,[R3, #+0]
        STRB     R3,[R0, #+0]
// 1087     for(i = left_top_whiteline;i<=right_top_whiteline;i++)
        LDR.W    R0,??DataTable8_3
        LDRB     R0,[R0, #+0]
        B.N      ??Deal_BlackEdge_32
// 1088     {
// 1089        if(right_black[i]- (right_black[i-1] - left_black[i-1]) <= 0 )//限幅
// 1090           left_black[i]=0;
// 1091         else
// 1092         left_black[i] = right_black[i] - (right_black[i-1] - left_black[i-1]);
??Deal_BlackEdge_33:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_8
        LDRB     R3,[R0, R3]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_8
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        SUBS     R3,R3,R4
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_7
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        ADDS     R3,R4,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_7
        STRB     R3,[R0, R4]
??Deal_BlackEdge_34:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_32:
        LDR.W    R3,??DataTable8_5
        LDRB     R3,[R3, #+0]
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R3,R0
        BLT.N    ??Deal_BlackEdge_35
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_8
        LDRB     R3,[R0, R3]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_8
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        SUBS     R3,R3,R4
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_7
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        UXTAB    R3,R3,R4
        CMP      R3,#+1
        BGE.N    ??Deal_BlackEdge_33
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        MOVS     R4,#+0
        STRB     R4,[R0, R3]
        B.N      ??Deal_BlackEdge_34
// 1093     }
// 1094     
// 1095   }
// 1096  else if(right_top_whiteline < left_top_whiteline)
??Deal_BlackEdge_31:
        LDR.W    R0,??DataTable8_5
        LDRB     R0,[R0, #+0]
        LDR.W    R3,??DataTable8_3
        LDRB     R3,[R3, #+0]
        CMP      R0,R3
        BCS.N    ??Deal_BlackEdge_36
// 1097  {
// 1098    top_whiteline = left_top_whiteline;
        LDR.W    R0,??DataTable8_4
        LDR.W    R3,??DataTable8_3
        LDRB     R3,[R3, #+0]
        STRB     R3,[R0, #+0]
// 1099      for(i = right_top_whiteline;i<=left_top_whiteline;i++)
        LDR.W    R0,??DataTable8_5
        LDRB     R0,[R0, #+0]
        B.N      ??Deal_BlackEdge_37
// 1100   {
// 1101     if(left_black[i] + (right_black[i-1] - left_black[i-1]) >= COLUMN-1)
// 1102       right_black[i] = COLUMN-1;   
// 1103     else
// 1104       right_black[i] = left_black[i] + (right_black[i-1] - left_black[i-1]);//
??Deal_BlackEdge_38:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        LDRB     R3,[R0, R3]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_8
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        ADDS     R3,R4,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_7
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        SUBS     R3,R3,R4
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_8
        STRB     R3,[R0, R4]
??Deal_BlackEdge_39:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_37:
        LDR.W    R3,??DataTable8_3
        LDRB     R3,[R3, #+0]
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R3,R0
        BLT.N    ??Deal_BlackEdge_35
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        LDRB     R3,[R0, R3]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_8
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        ADDS     R3,R4,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R4,??DataTable8_7
        ADDS     R4,R0,R4
        LDRB     R4,[R4, #-1]
        SUBS     R3,R3,R4
        CMP      R3,#+158
        BLT.N    ??Deal_BlackEdge_38
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_8
        MOVS     R4,#+158
        STRB     R4,[R0, R3]
        B.N      ??Deal_BlackEdge_39
// 1105   }
// 1106  }
// 1107   else
// 1108   {
// 1109     top_whiteline = right_top_whiteline;
??Deal_BlackEdge_36:
        LDR.W    R0,??DataTable8_4
        LDR.W    R3,??DataTable8_5
        LDRB     R3,[R3, #+0]
        STRB     R3,[R0, #+0]
// 1110   }
// 1111  if(top_whiteline+1 <= ROW - 1) 
??Deal_BlackEdge_35:
        LDR.W    R0,??DataTable8_4
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+65
        BCS.N    ??Deal_BlackEdge_40
// 1112  {
// 1113    for(k = top_whiteline+1; k < ROW; k++)
        LDR.W    R0,??DataTable8_4
        LDRB     R0,[R0, #+0]
        ADDS     R1,R0,#+1
        B.N      ??Deal_BlackEdge_41
// 1114    {
// 1115       center_white[k] = MID;  
??Deal_BlackEdge_42:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable8
        MOVS     R3,#+79
        STRB     R3,[R1, R0]
// 1116       left_black[k] = MID - 2;   
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable8_7
        MOVS     R3,#+77
        STRB     R3,[R1, R0]
// 1117       right_black[k] = MID + 2;  
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable8_8
        MOVS     R3,#+81
        STRB     R3,[R1, R0]
// 1118    }
        ADDS     R1,R1,#+1
??Deal_BlackEdge_41:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+65
        BLT.N    ??Deal_BlackEdge_42
// 1119  }
// 1120 
// 1121  //滤除左右的变沿线的单个跳变
// 1122   for( i=bottom_whitebase + 1 ;i <= top_whiteline;i++)
??Deal_BlackEdge_40:
        LDR.W    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        B.N      ??Deal_BlackEdge_43
// 1123   {
// 1124     if(left_black[i-1] <= 1 && left_black[i] > 1 && left_black[i+1] <= 1)
??Deal_BlackEdge_44:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        CMP      R3,#+2
        BCS.N    ??Deal_BlackEdge_45
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        LDRB     R3,[R0, R3]
        CMP      R3,#+2
        BCC.N    ??Deal_BlackEdge_45
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #+1]
        CMP      R3,#+2
        BCS.N    ??Deal_BlackEdge_45
// 1125     {
// 1126       left_black[i] = 0;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R3,??DataTable8_7
        MOVS     R4,#+0
        STRB     R4,[R0, R3]
// 1127       
// 1128       if(Row_state[k] == 1 ||Row_state[k] == 2)
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R3,??DataTable8_6
        LDRB     R3,[R1, R3]
        CMP      R3,#+1
        BEQ.N    ??Deal_BlackEdge_46
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R3,??DataTable8_6
        LDRB     R3,[R1, R3]
        CMP      R3,#+2
        BNE.N    ??Deal_BlackEdge_47
// 1129       {
// 1130         Row_state[k] = 2;
??Deal_BlackEdge_46:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R3,??DataTable8_6
        MOVS     R4,#+2
        STRB     R4,[R1, R3]
// 1131       }
// 1132       if(Row_state[k] == 3)
??Deal_BlackEdge_47:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        LDRB     R3,[R1, R3]
        CMP      R3,#+3
        BNE.N    ??Deal_BlackEdge_45
// 1133       {
// 1134         Row_state[k] = 0;
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        MOVS     R4,#+0
        STRB     R4,[R1, R3]
// 1135       }
// 1136     }
// 1137     
// 1138    if(right_black[i-1] >= COLUMN-2 && right_black[i] < COLUMN-2 && right_black[i+1] >= COLUMN-2)
??Deal_BlackEdge_45:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8_8
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-1]
        CMP      R3,#+157
        BCC.N    ??Deal_BlackEdge_48
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8_8
        LDRB     R3,[R0, R3]
        CMP      R3,#+157
        BCS.N    ??Deal_BlackEdge_48
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8_8
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #+1]
        CMP      R3,#+157
        BCC.N    ??Deal_BlackEdge_48
// 1139     {
// 1140       right_black[i] = COLUMN-1;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8_8
        MOVS     R4,#+158
        STRB     R4,[R0, R3]
// 1141       
// 1142       if(Row_state[k] == 0 ||Row_state[k] == 2)
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        LDRB     R3,[R1, R3]
        CMP      R3,#+0
        BEQ.N    ??Deal_BlackEdge_49
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        LDRB     R3,[R1, R3]
        CMP      R3,#+2
        BNE.N    ??Deal_BlackEdge_50
// 1143       {
// 1144         Row_state[k] = 2;
??Deal_BlackEdge_49:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        MOVS     R4,#+2
        STRB     R4,[R1, R3]
// 1145       }
// 1146       if(Row_state[k] == 3)
??Deal_BlackEdge_50:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        LDRB     R3,[R1, R3]
        CMP      R3,#+3
        BNE.N    ??Deal_BlackEdge_48
// 1147       {
// 1148         Row_state[k] = 1;
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.N    R3,??DataTable8_6
        MOVS     R4,#+1
        STRB     R4,[R1, R3]
// 1149       }
// 1150     }
// 1151   }
??Deal_BlackEdge_48:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_43:
        LDR.N    R3,??DataTable8_4
        LDRB     R3,[R3, #+0]
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R3,R0
        BGE.N    ??Deal_BlackEdge_44
// 1152  //边沿线滤波结束
// 1153   
// 1154  //对之前的错误的补线进行重新的虚构
// 1155   //这里的虚构会导致有一种情况发生就是，在弯道的顶行的时候，可能会出现补点，与不补点的区别，这样就导致了舵机的抖动
// 1156   for( i=bottom_whitebase ;i <= top_whiteline;i++)
        LDR.N    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        B.N      ??Deal_BlackEdge_51
// 1157   {
// 1158     if(Row_state[i] != 3)
??Deal_BlackEdge_52:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_6
        LDRB     R1,[R0, R1]
        CMP      R1,#+3
        BEQ.N    ??Deal_BlackEdge_53
// 1159     {
// 1160       center_white[i] = (right_black[i]+left_black[i])/2;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8_7
        LDRB     R3,[R0, R3]
        ADDS     R1,R3,R1
        MOVS     R3,#+2
        SDIV     R1,R1,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8
        STRB     R1,[R0, R3]
// 1161     }
// 1162   }
??Deal_BlackEdge_53:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_51:
        LDR.N    R1,??DataTable8_4
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BGE.N    ??Deal_BlackEdge_52
// 1163   
// 1164   //对于两边出界的点，不利用求平均值的方法去做，而是利用前一行的状态进行补充
// 1165   for(i= bottom_whitebase; i <= top_whiteline; i++)
        LDR.N    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        B.N      ??Deal_BlackEdge_54
// 1166   {
// 1167     if(left_black[i] <= 1 && right_black[i] <= COLUMN-5)  //对于左边出界的点进行补充
// 1168     {
// 1169       if( right_black[i] - (right_black[un_out_hang]-left_black[un_out_hang])/2 < 0)
// 1170         center_white[i] = 0;
// 1171       else
// 1172        center_white[i] = right_black[i] - (right_black[un_out_hang]-left_black[un_out_hang])/2;
// 1173     }
// 1174     else if(right_black[i] >= COLUMN-2 && left_black[i] >= 3)
// 1175     {
// 1176       if( left_black[i] + (right_black[un_out_hang]-left_black[un_out_hang])/2 > COLUMN - 1)
// 1177         center_white[i] = COLUMN - 1 ;
// 1178       else
// 1179        center_white[i] = left_black[i] + (right_black[un_out_hang]-left_black[un_out_hang])/2;
// 1180     }
// 1181     else
// 1182       un_out_hang = i;
??Deal_BlackEdge_55:
        MOVS     R2,R0
??Deal_BlackEdge_56:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_54:
        LDR.N    R1,??DataTable8_4
        LDRB     R1,[R1, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BLT.N    ??Deal_BlackEdge_57
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        LDRB     R1,[R0, R1]
        CMP      R1,#+2
        BCS.N    ??Deal_BlackEdge_58
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        CMP      R1,#+155
        BCS.N    ??Deal_BlackEdge_58
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R3,??DataTable8_8
        LDRB     R3,[R2, R3]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R4,??DataTable8_7
        LDRB     R4,[R2, R4]
        SUBS     R3,R3,R4
        MOVS     R4,#+2
        SDIV     R3,R3,R4
        SUBS     R1,R1,R3
        CMP      R1,#+0
        BPL.N    ??Deal_BlackEdge_59
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        MOVS     R3,#+0
        STRB     R3,[R0, R1]
        B.N      ??Deal_BlackEdge_56
??Deal_BlackEdge_59:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R3,??DataTable8_8
        LDRB     R3,[R2, R3]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R4,??DataTable8_7
        LDRB     R4,[R2, R4]
        SUBS     R3,R3,R4
        MOVS     R4,#+2
        SDIV     R3,R3,R4
        SUBS     R1,R1,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8
        STRB     R1,[R0, R3]
        B.N      ??Deal_BlackEdge_56
??Deal_BlackEdge_58:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        CMP      R1,#+157
        BCC.N    ??Deal_BlackEdge_55
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        LDRB     R1,[R0, R1]
        CMP      R1,#+3
        BCC.N    ??Deal_BlackEdge_55
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        LDRB     R1,[R0, R1]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R3,??DataTable8_8
        LDRB     R3,[R2, R3]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R4,??DataTable8_7
        LDRB     R4,[R2, R4]
        SUBS     R3,R3,R4
        MOVS     R4,#+2
        SDIV     R3,R3,R4
        ADDS     R1,R3,R1
        CMP      R1,#+159
        BLT.N    ??Deal_BlackEdge_60
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        MOVS     R3,#+158
        STRB     R3,[R0, R1]
        B.N      ??Deal_BlackEdge_56
??Deal_BlackEdge_60:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        LDRB     R1,[R0, R1]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R3,??DataTable8_8
        LDRB     R3,[R2, R3]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R4,??DataTable8_7
        LDRB     R4,[R2, R4]
        SUBS     R3,R3,R4
        MOVS     R4,#+2
        SDIV     R3,R3,R4
        ADDS     R1,R3,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R3,??DataTable8
        STRB     R1,[R0, R3]
        B.N      ??Deal_BlackEdge_56
// 1183   }
// 1184  
// 1185  
// 1186  
// 1187  //对中线进行中值滤波
// 1188  //对中线和边沿线进行中值滤波
// 1189  for( i=bottom_whitebase + 1;i < top_whiteline-2;i++)
??Deal_BlackEdge_57:
        LDR.N    R0,??DataTable8_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        B.N      ??Deal_BlackEdge_61
// 1190  {
// 1191    if((center_white[i] > center_white[i-1] && center_white[i] > center_white[i+1]) ||(center_white[i] < center_white[i-1] && center_white[i] < center_white[i+1]))
??Deal_BlackEdge_62:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8
        LDRB     R2,[R0, R2]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_63
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #+1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8
        LDRB     R2,[R0, R2]
        CMP      R1,R2
        BCC.N    ??Deal_BlackEdge_64
??Deal_BlackEdge_63:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #-1]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_65
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+1]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_65
// 1192    {
// 1193      center_white[i] = (center_white[i-1] + center_white[i+1])/2;
??Deal_BlackEdge_64:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+1]
        UXTAB    R1,R2,R1
        MOVS     R2,#+2
        SDIV     R1,R1,R2
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8
        STRB     R1,[R0, R2]
// 1194    }
// 1195       if((left_black[i] > left_black[i-1] && left_black[i] > left_black[i+1]) ||(left_black[i] < left_black[i-1] && left_black[i] < left_black[i+1]))
??Deal_BlackEdge_65:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_7
        LDRB     R2,[R0, R2]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_66
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #+1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_7
        LDRB     R2,[R0, R2]
        CMP      R1,R2
        BCC.N    ??Deal_BlackEdge_67
??Deal_BlackEdge_66:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_7
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #-1]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_68
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_7
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+1]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_68
// 1196    {
// 1197      left_black[i] = (left_black[i-1] + left_black[i+1])/2;
??Deal_BlackEdge_67:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_7
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_7
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+1]
        UXTAB    R1,R2,R1
        MOVS     R2,#+2
        SDIV     R1,R1,R2
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_7
        STRB     R1,[R0, R2]
// 1198    }
// 1199       if((right_black[i] > right_black[i-1] && right_black[i] > right_black[i+1]) ||(right_black[i] < right_black[i-1] && right_black[i] < right_black[i+1]))
??Deal_BlackEdge_68:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_8
        LDRB     R2,[R0, R2]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_69
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #+1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_8
        LDRB     R2,[R0, R2]
        CMP      R1,R2
        BCC.N    ??Deal_BlackEdge_70
??Deal_BlackEdge_69:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_8
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #-1]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_71
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        LDRB     R1,[R0, R1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_8
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+1]
        CMP      R1,R2
        BCS.N    ??Deal_BlackEdge_71
// 1200    {
// 1201      right_black[i] = (right_black[i-1] + right_black[i+1])/2;
??Deal_BlackEdge_70:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_8
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #-1]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_8
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+1]
        UXTAB    R1,R2,R1
        MOVS     R2,#+2
        SDIV     R1,R1,R2
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R2,??DataTable8_8
        STRB     R1,[R0, R2]
// 1202    }
// 1203  } 
??Deal_BlackEdge_71:
        ADDS     R0,R0,#+1
??Deal_BlackEdge_61:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.N    R1,??DataTable8_4
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+2
        CMP      R0,R1
        BLT.W    ??Deal_BlackEdge_62
// 1204  
// 1205 }
        POP      {R4-R7}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8:
        DC32     center_white

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_1:
        DC32     VideoImage2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_2:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_3:
        DC32     left_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_4:
        DC32     top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_5:
        DC32     right_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_6:
        DC32     Row_state

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_7:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_8:
        DC32     right_black
// 1206 
// 1207 /*这个程序包含了两部分，其一是赛道特征的提取；其二是赛道类型的判断
// 1208 */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1209 void get_line_information(void)
// 1210 {
get_line_information:
        PUSH     {R3-R5,LR}
// 1211   int16 i;
// 1212   uint8 ramp_count = 0;  //用于记录宽度超出限制的行的个数
        MOVS     R1,#+0
// 1213   uint16 temp_center_line = 0;
        MOVS     R0,#+0
// 1214   center_lost_hang = 0;
        LDR.W    R2,??DataTable11
        MOVS     R3,#+0
        STRB     R3,[R2, #+0]
// 1215   
// 1216     /*程序的开始首先对中线出现断点的情况进行修补
// 1217   当图像的中线出现了巨大的跳变时，他前面的线全部用左右两边的中值代替
// 1218   */
// 1219   for(i = bottom_whitebase + 10 ; i < top_whiteline-5;i++)  //基准行上的偏差不用处理
        LDR.W    R2,??DataTable11_1
        LDRB     R2,[R2, #+0]
        ADDS     R4,R2,#+10
        B.N      ??get_line_information_0
??get_line_information_1:
        ADDS     R4,R4,#+1
??get_line_information_0:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_2
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+5
        CMP      R4,R0
        BGE.N    ??get_line_information_2
// 1220   {
// 1221     if(f_abs16(center_white[i] - center_white[i+2]) > CENTER_LOST_POINT
// 1222        &&f_abs16(center_white[i] - center_white[i+3]) > CENTER_LOST_POINT)
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+2]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+21
        BLT.N    ??get_line_information_1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+3]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+21
        BLT.N    ??get_line_information_1
// 1223     {
// 1224       center_lost_hang = i;
        LDR.W    R0,??DataTable11
        STRB     R4,[R0, #+0]
// 1225       break;
// 1226     }
// 1227   }
// 1228    if(center_lost_hang >0)
??get_line_information_2:
        LDR.W    R0,??DataTable11
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BCC.N    ??get_line_information_3
// 1229    {
// 1230      for( i = bottom_whitebase ;  i< center_lost_hang + 2;i++)
        LDR.W    R0,??DataTable11_1
        LDRB     R4,[R0, #+0]
        B.N      ??get_line_information_4
// 1231      {
// 1232        center_white[i] = (left_black[i]+right_black[i])/2;  
??get_line_information_5:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_4
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_5
        LDRB     R1,[R4, R1]
        ADDS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        STRB     R0,[R4, R1]
// 1233      }
        ADDS     R4,R4,#+1
??get_line_information_4:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        CMP      R4,R0
        BLT.N    ??get_line_information_5
// 1234    }
// 1235   
// 1236   for( i=top_whiteline;i>20;i--)
??get_line_information_3:
        LDR.W    R0,??DataTable11_2
        LDRB     R4,[R0, #+0]
        B.N      ??get_line_information_6
??get_line_information_7:
        SUBS     R4,R4,#+1
??get_line_information_6:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+21
        BLT.N    ??get_line_information_8
// 1237   {
// 1238     if((f_abs16(center_white[i] - left_black[i])<=5 &&f_abs16(center_white[i-1] - left_black[i-1])<=5)
// 1239        ||(f_abs16(center_white[i] - right_black[i])<=5 &&f_abs16(center_white[i-1] - right_black[i-1])<=5))
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_4
        LDRB     R1,[R4, R1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+6
        BGE.N    ??get_line_information_9
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_4
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+6
        BLT.N    ??get_line_information_10
??get_line_information_9:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_5
        LDRB     R1,[R4, R1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+6
        BGE.N    ??get_line_information_7
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #-1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_5
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+6
        BGE.N    ??get_line_information_7
// 1240     {
// 1241       top_whiteline = i;
??get_line_information_10:
        LDR.W    R0,??DataTable11_2
        STRB     R4,[R0, #+0]
// 1242       break;
// 1243     }
// 1244   }
// 1245   if(top_whiteline < ROW - 2)
??get_line_information_8:
        LDR.W    R0,??DataTable11_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+63
        BCS.N    ??get_line_information_11
// 1246   {
// 1247     for(i = top_whiteline+1;i<ROW;i++)
        LDR.W    R0,??DataTable11_2
        LDRB     R0,[R0, #+0]
        ADDS     R4,R0,#+1
        B.N      ??get_line_information_12
// 1248     {
// 1249       center_white[i] =MID;
??get_line_information_13:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        MOVS     R1,#+79
        STRB     R1,[R4, R0]
// 1250       right_black[i] = MID+2;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_5
        MOVS     R1,#+81
        STRB     R1,[R4, R0]
// 1251       left_black[i] = MID-2;
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_4
        MOVS     R1,#+77
        STRB     R1,[R4, R0]
// 1252     }
        ADDS     R4,R4,#+1
??get_line_information_12:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+65
        BLT.N    ??get_line_information_13
// 1253   }
// 1254   
// 1255   
// 1256   //图像的中心线出来之后，首先要确定能够到那些行，在此之后的行全部用控制的行的
// 1257    /////////////////////求取控制的最高有效行////////////////////////////////
// 1258   danger_count = 0;
??get_line_information_11:
        LDR.W    R0,??DataTable11_6
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1259   danger_flag = 1;//没有判断之前都认为是危险状态
        LDR.W    R0,??DataTable11_7
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1260   control_top_whiteline = top_whiteline;
        LDR.W    R0,??DataTable11_8
        LDR.W    R1,??DataTable11_2
        LDRB     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??get_line_information_14
// 1261   
// 1262   ////////////////////计算控制的最高有效行//////////////////////
// 1263      while(danger_flag ==1)
// 1264      {
// 1265        for(i = control_top_whiteline;i>=deal_start_line ;i--)//表示从上向下遍历
// 1266         {
// 1267           temp_center_line = center_white[control_top_whiteline] + (i - (control_top_whiteline)) *(center_white[deal_start_line] - center_white[control_top_whiteline])/( deal_start_line-(control_top_whiteline) );
// 1268         if( right_black[i] < COLUMN - 2 && left_black[i] > 1&&
// 1269            (right_black[i] - temp_center_line < (refer_road_width[i]/5) || (temp_center_line - left_black[i] < (refer_road_width[i]/5))))
// 1270             danger_count++;  
// 1271         }
// 1272        
// 1273        if(danger_count >0)
// 1274        {
// 1275          control_top_whiteline --;  
??get_line_information_15:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable11_8
        STRH     R0,[R1, #+0]
// 1276          danger_count = 0;
        LDR.W    R0,??DataTable11_6
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1277        }
??get_line_information_14:
        LDR.W    R0,??DataTable11_7
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??get_line_information_16
        LDR.W    R0,??DataTable11_8
        LDRSH    R4,[R0, #+0]
        B.N      ??get_line_information_17
??get_line_information_18:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable11_3
        LDRB     R0,[R0, R1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        SUBS     R1,R4,R1
        LDR.W    R2,??DataTable11_9
        LDRB     R2,[R2, #+0]
        LDR.W    R3,??DataTable11_3
        LDRB     R2,[R2, R3]
        LDR.W    R3,??DataTable11_8
        LDRH     R3,[R3, #+0]
        LDR.W    R5,??DataTable11_3
        LDRB     R3,[R3, R5]
        SUBS     R2,R2,R3
        MULS     R1,R2,R1
        LDR.W    R2,??DataTable11_9
        LDRB     R2,[R2, #+0]
        LDR.W    R3,??DataTable11_8
        LDRH     R3,[R3, #+0]
        SUBS     R2,R2,R3
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_5
        LDRB     R1,[R4, R1]
        CMP      R1,#+157
        BCS.N    ??get_line_information_19
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_4
        LDRB     R1,[R4, R1]
        CMP      R1,#+2
        BCC.N    ??get_line_information_19
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_10
        LDRB     R1,[R4, R1]
        MOVS     R2,#+5
        SDIV     R1,R1,R2
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R2,??DataTable11_5
        LDRB     R2,[R4, R2]
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        SUBS     R2,R2,R0
        CMP      R2,R1
        BLT.N    ??get_line_information_20
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_10
        LDRB     R1,[R4, R1]
        MOVS     R2,#+5
        SDIV     R1,R1,R2
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R2,??DataTable11_4
        LDRB     R2,[R4, R2]
        SUBS     R0,R0,R2
        CMP      R0,R1
        BGE.N    ??get_line_information_19
??get_line_information_20:
        LDR.W    R0,??DataTable11_6
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable11_6
        STRH     R0,[R1, #+0]
??get_line_information_19:
        SUBS     R4,R4,#+1
??get_line_information_17:
        LDR.W    R0,??DataTable11_9
        LDRB     R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R4,R0
        BGE.N    ??get_line_information_18
        LDR.W    R0,??DataTable11_6
        LDRH     R0,[R0, #+0]
        CMP      R0,#+1
        BCS.N    ??get_line_information_15
// 1278        else
// 1279        {
// 1280          danger_flag = 0;//危险消除
        LDR.W    R0,??DataTable11_7
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??get_line_information_14
// 1281        }
// 1282      }
// 1283      
// 1284      
// 1285   /////////////////赛道的类型的判断/////////////////////////
// 1286     S_right = 0;//向右拐的计数
??get_line_information_16:
        LDR.W    R0,??DataTable11_11
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1287     S_left =0 ; //向左拐计数
        LDR.W    R0,??DataTable11_12
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1288     S_straight = 0;
        LDR.W    R0,??DataTable11_13
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1289   for( i=bottom_whitebase ;i < control_top_whiteline;i++)
        LDR.W    R0,??DataTable11_1
        LDRB     R4,[R0, #+0]
        B.N      ??get_line_information_21
// 1290  {
// 1291    if(center_white[i+1]- center_white[i] > 1)
// 1292    {
// 1293      S_right++; //S 弯右加加
// 1294    }
// 1295    else if(center_white[i]- center_white[i+1] > 1)
// 1296    {
// 1297      S_left++; //S 弯左加加
// 1298    }
// 1299    else
// 1300    {
// 1301      S_straight++;
??get_line_information_22:
        LDR.W    R0,??DataTable11_13
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable11_13
        STRB     R0,[R1, #+0]
// 1302    }
??get_line_information_23:
        ADDS     R4,R4,#+1
??get_line_information_21:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        CMP      R4,R0
        BGE.N    ??get_line_information_24
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        ADDS     R0,R4,R0
        LDRB     R0,[R0, #+1]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        LDRB     R1,[R4, R1]
        SUBS     R0,R0,R1
        CMP      R0,#+2
        BLT.N    ??get_line_information_25
        LDR.W    R0,??DataTable11_11
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable11_11
        STRB     R0,[R1, #+0]
        B.N      ??get_line_information_23
??get_line_information_25:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        ADDS     R1,R4,R1
        LDRB     R1,[R1, #+1]
        SUBS     R0,R0,R1
        CMP      R0,#+2
        BLT.N    ??get_line_information_22
        LDR.W    R0,??DataTable11_12
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable11_12
        STRB     R0,[R1, #+0]
        B.N      ??get_line_information_23
// 1303  }
// 1304  
// 1305  /*对赛道的判断
// 1306  对于赛道的判断 只是区分直道(1) 、波浪弯道(2)、 入弯(3)、弯道(4) */
// 1307  
// 1308  
// 1309  if( control_top_whiteline >= 62)
??get_line_information_24:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        CMP      R0,#+62
        BCC.N    ??get_line_information_26
// 1310  {
// 1311    if(S_left<4 && S_right < 4 )
        LDR.W    R0,??DataTable11_12
        LDRB     R0,[R0, #+0]
        CMP      R0,#+4
        BCS.N    ??get_line_information_27
        LDR.W    R0,??DataTable11_11
        LDRB     R0,[R0, #+0]
        CMP      R0,#+4
        BCS.N    ??get_line_information_27
// 1312       direction = 1;  //直道
        LDR.W    R0,??DataTable11_14
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??get_line_information_28
// 1313    else if(f_abs16(S_left-S_right) < 15 )
??get_line_information_27:
        LDR.W    R0,??DataTable11_12
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable11_11
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+15
        BGE.N    ??get_line_information_29
// 1314       direction = 2;  //波浪弯道
        LDR.W    R0,??DataTable11_14
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
        B.N      ??get_line_information_28
// 1315    else
// 1316      direction =3;// re_direction;
??get_line_information_29:
        LDR.W    R0,??DataTable11_14
        MOVS     R1,#+3
        STRB     R1,[R0, #+0]
        B.N      ??get_line_information_28
// 1317  }
// 1318  else if( control_top_whiteline >= 50 && control_top_whiteline < 62)
??get_line_information_26:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+50
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+12
        BCS.N    ??get_line_information_30
// 1319  {
// 1320    direction = 3;  //入弯道
        LDR.W    R0,??DataTable11_14
        MOVS     R1,#+3
        STRB     R1,[R0, #+0]
        B.N      ??get_line_information_28
// 1321  }
// 1322  else
// 1323  {
// 1324    if(ramp_flag == 1)
??get_line_information_30:
        LDR.W    R0,??DataTable11_15
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??get_line_information_31
// 1325       direction = 1;//坡道视为直道
        LDR.W    R0,??DataTable11_14
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??get_line_information_28
// 1326    else
// 1327       direction = 4;  //弯道当中
??get_line_information_31:
        LDR.W    R0,??DataTable11_14
        MOVS     R1,#+4
        STRB     R1,[R0, #+0]
// 1328  }
// 1329  re_direction =direction ;
??get_line_information_28:
        LDR.W    R0,??DataTable11_16
        LDR.W    R1,??DataTable11_14
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
// 1330  //////////////////////赛道的类型的判断结束///////////////////////////////
// 1331  
// 1332 
// 1333      
// 1334   //////////////////////////////对赛道进行优化//////////////////////////////
// 1335  //需要注意的是既然已经将赛道的类型判断出来了，那么就可以按照不同的赛道实现不同的优化特别是针对波浪弯道o
// 1336  if(direction == 2)  //波浪弯道  控制行大于60
        LDR.W    R0,??DataTable11_14
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??get_line_information_32
// 1337  {
// 1338    for( i=bottom_whitebase ;i <= control_top_whiteline;i++)//无论是什么样的赛道，将中线向图像的中心平移
        LDR.W    R0,??DataTable11_1
        LDRB     R4,[R0, #+0]
        B.N      ??get_line_information_33
// 1339      {
// 1340       if(center_white[i] > MID)//归中
// 1341       {
// 1342         if( center_white[i] - (control_top_whiteline - 62 ) >= MID )
// 1343           center_white[i] = center_white[i] - (control_top_whiteline - 62 );
// 1344         else
// 1345           center_white[i] = MID;
// 1346       }
// 1347       else 
// 1348       {
// 1349         if(center_white[i] + (control_top_whiteline - 62 ) <= MID)  //归中防止在波浪弯道的较大的打角
// 1350           center_white[i] = center_white[i] + (control_top_whiteline - 62 );  
// 1351         else
// 1352           center_white[i] = MID; 
??get_line_information_34:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        MOVS     R1,#+79
        STRB     R1,[R4, R0]
??get_line_information_35:
        ADDS     R4,R4,#+1
??get_line_information_33:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BLT.W    ??get_line_information_36
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        CMP      R0,#+80
        BCC.N    ??get_line_information_37
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        SUBS     R0,R0,R1
        ADDS     R0,R0,#+62
        CMP      R0,#+79
        BLT.N    ??get_line_information_38
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        SUBS     R0,R0,R1
        SUBS     R0,R0,#+194
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        STRB     R0,[R4, R1]
        B.N      ??get_line_information_35
??get_line_information_38:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        MOVS     R1,#+79
        STRB     R1,[R4, R0]
        B.N      ??get_line_information_35
??get_line_information_37:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        UXTAH    R0,R0,R1
        SUBS     R0,R0,#+62
        CMP      R0,#+80
        BGE.N    ??get_line_information_34
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        SUBS     R1,R1,#+62
        ADDS     R0,R1,R0
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        STRB     R0,[R4, R1]
        B.N      ??get_line_information_35
// 1353       }
// 1354      }
// 1355  }
// 1356  else if(direction != 1 )   //直线状态不进行归中，防止直道漂浮
??get_line_information_32:
        LDR.W    R0,??DataTable11_14
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??get_line_information_36
// 1357   {
// 1358    for( i=bottom_whitebase ;i <= control_top_whiteline;i++)//无论是什么样的赛道，将中线向图像的中心平移
        LDR.W    R0,??DataTable11_1
        LDRB     R4,[R0, #+0]
        B.N      ??get_line_information_39
// 1359      {
// 1360       if(center_white[i] > MID)//归中
// 1361       {
// 1362         if( center_white[i] - 1 >= MID )
// 1363           center_white[i] = center_white[i] - 1;
// 1364         else
// 1365           center_white[i] = MID;
// 1366       }
// 1367       else 
// 1368       {
// 1369         if(center_white[i] + 1 <= MID)  //归中防止在波浪弯道的较大的打角
// 1370           center_white[i] = center_white[i] + 1;  
// 1371         else
// 1372           center_white[i] = MID; 
??get_line_information_40:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        MOVS     R1,#+79
        STRB     R1,[R4, R0]
??get_line_information_41:
        ADDS     R4,R4,#+1
??get_line_information_39:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BLT.N    ??get_line_information_36
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        CMP      R0,#+80
        BCC.N    ??get_line_information_42
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+1
        CMP      R0,#+79
        BLT.N    ??get_line_information_43
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        SUBS     R0,R0,#+1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        STRB     R0,[R4, R1]
        B.N      ??get_line_information_41
??get_line_information_43:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        MOVS     R1,#+79
        STRB     R1,[R4, R0]
        B.N      ??get_line_information_41
??get_line_information_42:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        ADDS     R0,R0,#+1
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+80
        BCS.N    ??get_line_information_40
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        ADDS     R0,R0,#+1
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        STRB     R0,[R4, R1]
        B.N      ??get_line_information_41
// 1373       }
// 1374      }
// 1375  }
// 1376 //////////////////////////////对赛道的优化结束//////////////////////////////
// 1377   /*
// 1378  对赛道信息的提取，主要包括以下几个量。
// 1379  只是对处于控制行一下的中线求取平均值，
// 1380  */
// 1381 
// 1382  //////////////////////对图像的平均值的提取/////////////////////
// 1383   center_average = 0;//清零
??get_line_information_36:
        LDR.W    R0,??DataTable11_17
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 1384   center_error_average = 0;
        LDR.W    R0,??DataTable11_18
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1385   if(control_top_whiteline > 50)
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        CMP      R0,#+51
        BCC.N    ??get_line_information_44
// 1386   {
// 1387     for(i = bottom_whitebase+1;i<=control_top_whiteline- 10;i++)
        LDR.W    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        ADDS     R4,R0,#+1
        B.N      ??get_line_information_45
// 1388    {
// 1389      center_average +=  center_white[i];
??get_line_information_46:
        LDR.W    R0,??DataTable11_17
        LDR      R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        LDRB     R1,[R4, R1]
        ADDS     R0,R0,R1
        LDR.W    R1,??DataTable11_17
        STR      R0,[R1, #+0]
// 1390      if(i == control_top_whiteline - 10)    //只是对前100cm左右的前瞻进行加权
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        CMP      R4,R0
        BNE.N    ??get_line_information_47
// 1391      {
// 1392        center_average = center_average /(control_top_whiteline - bottom_whitebase -10);
        LDR.W    R0,??DataTable11_17
        LDR      R0,[R0, #+0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable11_1
        LDRB     R2,[R2, #+0]
        SUBS     R1,R1,R2
        SUBS     R1,R1,#+10
        UDIV     R0,R0,R1
        LDR.W    R1,??DataTable11_17
        STR      R0,[R1, #+0]
// 1393        center_linear_average = center_average ;
        LDR.W    R0,??DataTable11_19
        LDR.W    R1,??DataTable11_17
        LDR      R1,[R1, #+0]
        STR      R1,[R0, #+0]
// 1394      }
// 1395    }
??get_line_information_47:
        ADDS     R4,R4,#+1
??get_line_information_45:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BGE.N    ??get_line_information_46
// 1396    
// 1397      //进行偏差的绝对值求和
// 1398     for( i=bottom_whitebase+1 ;i <= control_top_whiteline- 10;i++)
        LDR.W    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        ADDS     R4,R0,#+1
        B.N      ??get_line_information_48
// 1399     {
// 1400       center_error_average += f_abs16( center_white[i]  - center_average);
??get_line_information_49:
        LDR.W    R0,??DataTable11_18
        LDRH     R5,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable11_17
        LDR      R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        ADDS     R0,R0,R5
        LDR.W    R1,??DataTable11_18
        STRH     R0,[R1, #+0]
// 1401     }
        ADDS     R4,R4,#+1
??get_line_information_48:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BGE.N    ??get_line_information_49
// 1402     center_error_average /= (control_top_whiteline - bottom_whitebase - 10) ;    //反应了所有的中线偏离中线中心的绝对的大小。他的大小在一定的程度上反应了，中线的线性度的大小
        LDR.W    R0,??DataTable11_18
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable11_1
        LDRB     R2,[R2, #+0]
        SUBS     R1,R1,R2
        SUBS     R1,R1,#+10
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable11_18
        STRH     R0,[R1, #+0]
        B.N      ??get_line_information_50
// 1403     
// 1404   }
// 1405   else
// 1406   {
// 1407    for(i = bottom_whitebase+1;i<=control_top_whiteline;i++)
??get_line_information_44:
        LDR.W    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        ADDS     R4,R0,#+1
        B.N      ??get_line_information_51
// 1408    {
// 1409      center_average +=  center_white[i];
??get_line_information_52:
        LDR.W    R0,??DataTable11_17
        LDR      R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R1,??DataTable11_3
        LDRB     R1,[R4, R1]
        ADDS     R0,R0,R1
        LDR.W    R1,??DataTable11_17
        STR      R0,[R1, #+0]
// 1410      if(i == control_top_whiteline)    //只是对前100cm左右的前瞻进行加权
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        CMP      R4,R0
        BNE.N    ??get_line_information_53
// 1411      {
// 1412        center_average = center_average /(control_top_whiteline - bottom_whitebase);
        LDR.W    R0,??DataTable11_17
        LDR      R0,[R0, #+0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable11_1
        LDRB     R2,[R2, #+0]
        SUBS     R1,R1,R2
        UDIV     R0,R0,R1
        LDR.W    R1,??DataTable11_17
        STR      R0,[R1, #+0]
// 1413      }
// 1414    }
??get_line_information_53:
        ADDS     R4,R4,#+1
??get_line_information_51:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BGE.N    ??get_line_information_52
// 1415         //进行偏差的绝对值求和
// 1416     for( i=bottom_whitebase+1 ;i <= control_top_whiteline;i++)
        LDR.W    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        ADDS     R4,R0,#+1
        B.N      ??get_line_information_54
// 1417     {
// 1418       center_error_average += f_abs16( center_white[i]  - center_average);
??get_line_information_55:
        LDR.W    R0,??DataTable11_18
        LDRH     R5,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable11_17
        LDR      R1,[R1, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        ADDS     R0,R0,R5
        LDR.W    R1,??DataTable11_18
        STRH     R0,[R1, #+0]
// 1419     }
        ADDS     R4,R4,#+1
??get_line_information_54:
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R0,R4
        BGE.N    ??get_line_information_55
// 1420     center_error_average /= (control_top_whiteline - bottom_whitebase) ;    //反应了所有的中线偏离中线中心的绝对的大小。他的大小在一定的程度上反应了，中线的线性度的大小
        LDR.W    R0,??DataTable11_18
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable11_1
        LDRB     R2,[R2, #+0]
        SUBS     R1,R1,R2
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable11_18
        STRH     R0,[R1, #+0]
// 1421     
// 1422   }
// 1423    
// 1424     //为了准确的判断出赛道的变化趋势，十分有必要的是对这个数的历史进行存储。这里存储7个历史值，然后进行模糊判断。
// 1425     //通过测试发现 这个值的变化趋势在0 到 25之间变化  当在直道的时候，值在0 - -8之间在弯道中的时候，是在16--25之间变化
// 1426     //当数值达到19后，则认为已经到达弯道中间或是在出弯道
// 1427   ///////////////均值提取结束//////////////////////////////
// 1428   
// 1429   /*/////对坡道的判断，当坡道判断出来之后，用标志位标志，且其只是作用在电机的给定控制上
// 1430   对于坡道的检测只需要将上坡道检测出来（最高行接近顶行，且其宽度达到了一定的范围）下坡检测比较的困难，
// 1431   所以这里不检测下坡，一般只是做一下坡道状态延时就行了（延时时间为1s--2s之间），
// 1432   且检测出来之后，只需要降速，对于舵机可以不用去管。图像已经做得可以了。
// 1433   对坡道的检测不能只是用宽度去判定，这样容易和弯道出现误检。所以还要加上对端点的限制，将其限制在某一个范围之内,
// 1434   这就要求进入弯道之前车子是摆正的。
// 1435   #define RAMP_WIDTH  45                  //图像10~20行的宽度范围超过该范围确定为坡道
// 1436   #define RAMP_TIME   60
// 1437   uint8 ramp_time = 50;                     //进入坡道后多长时间重新开启起跑线检测
// 1438   int8 ramp_speed = 0;                    //坡道减速值
// 1439   bool ramp_flag;                          //进入坡道标志,主要用于控制
// 1440   bool ramp_dis_flag;                     //主要是防止下坡误判
// 1441   
// 1442   对于坡道的检测不能用最高的几行，因为这样容易和十字道路误检
// 1443  */
// 1444   
// 1445   ramp_count = 0;//3,65,4,25
??get_line_information_50:
        MOVS     R1,#+0
// 1446   if( ramp_dis_flag ==0 && direction == 1 && control_top_whiteline >= ROW - 2 ) //直线状态检测  //加入这个ramp_dis_flag标志，是为了让车子在检测数跑道后的这段时间里，不对坡道进行检测
        LDR.W    R0,??DataTable12
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??get_line_information_56
        LDR.W    R0,??DataTable11_14
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??get_line_information_56
        LDR.W    R0,??DataTable11_8
        LDRH     R0,[R0, #+0]
        CMP      R0,#+63
        BCC.N    ??get_line_information_56
// 1447   {
// 1448     for(i = 35; i<60; i++)
        MOVS     R4,#+35
        B.N      ??get_line_information_57
// 1449       {
// 1450         if(right_black[i]-left_black[i] > refer_road_width[i] + 8)
??get_line_information_58:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable11_10
        LDRB     R0,[R4, R0]
        ADDS     R0,R0,#+8
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R2,??DataTable11_5
        LDRB     R2,[R4, R2]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R3,??DataTable11_4
        LDRB     R3,[R4, R3]
        SUBS     R2,R2,R3
        CMP      R0,R2
        BGE.N    ??get_line_information_59
// 1451           ramp_count ++;
        ADDS     R1,R1,#+1
// 1452       }        
??get_line_information_59:
        ADDS     R4,R4,#+1
??get_line_information_57:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+60
        BLT.N    ??get_line_information_58
// 1453     if(ramp_count >= 20)
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+20
        BCC.N    ??get_line_information_56
// 1454     { 
// 1455       if(left_black[ 40] - left_black[35] > 0 && left_black[ 40] - right_black[35] < 5
// 1456          && right_black[35] -right_black[40] > 0 && right_black[35] -right_black[40] < 5)
        LDR.W    R0,??DataTable11_4
        LDRB     R0,[R0, #+40]
        LDR.W    R1,??DataTable11_4
        LDRB     R1,[R1, #+35]
        SUBS     R0,R0,R1
        CMP      R0,#+1
        BLT.N    ??get_line_information_56
        LDR.W    R0,??DataTable11_4
        LDRB     R0,[R0, #+40]
        LDR.W    R1,??DataTable11_5
        LDRB     R1,[R1, #+35]
        SUBS     R0,R0,R1
        CMP      R0,#+5
        BGE.N    ??get_line_information_56
        LDR.W    R0,??DataTable11_5
        LDRB     R0,[R0, #+35]
        LDR.W    R1,??DataTable11_5
        LDRB     R1,[R1, #+40]
        SUBS     R0,R0,R1
        SUBS     R0,R0,#+1
        CMP      R0,#+4
        BCS.N    ??get_line_information_56
// 1457       {   
// 1458       ramp_flag = 1;
        LDR.W    R0,??DataTable11_15
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1459       ramp_dis_flag = 1;
        LDR.W    R0,??DataTable12
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1460       }
// 1461     } 
// 1462   }
// 1463   if(ramp_flag == 1)
??get_line_information_56:
        LDR.W    R0,??DataTable11_15
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??get_line_information_60
// 1464   {
// 1465     ramp_time++;
        LDR.W    R0,??DataTable11_20
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable11_20
        STRH     R0,[R1, #+0]
// 1466     if(ramp_time >= ramp_delay_time)
        LDR.W    R0,??DataTable11_20
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable11_21
        LDRH     R1,[R1, #+0]
        CMP      R0,R1
        BCC.N    ??get_line_information_60
// 1467     {
// 1468       ramp_time = 0;
        LDR.W    R0,??DataTable11_20
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1469       ramp_flag = 0;
        LDR.W    R0,??DataTable11_15
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1470     }
// 1471   }
// 1472   if(ramp_dis_flag == 1)
??get_line_information_60:
        LDR.W    R0,??DataTable12
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??get_line_information_61
// 1473   {
// 1474     ramp_dis_time++;
        LDR.W    R0,??DataTable11_22
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable11_22
        STRH     R0,[R1, #+0]
// 1475     if(ramp_dis_time >= 4 * ramp_delay_time)
        LDR.W    R0,??DataTable11_22
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable11_21
        LDRH     R1,[R1, #+0]
        CMP      R0,R1, LSL #+2
        BLT.N    ??get_line_information_61
// 1476     {
// 1477       ramp_dis_time = 0;
        LDR.W    R0,??DataTable11_22
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1478       ramp_dis_flag = 0;
        LDR.W    R0,??DataTable12
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1479     }
// 1480   }
// 1481 }
??get_line_information_61:
        POP      {R0,R4,R5,PC}    ;; return
// 1482 
// 1483 
// 1484 /*-------------------------------获取中线的线性相关系数-----------------------------*/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1485 float get_linear_factor(uint8 bottom,uint8 top,uint8 average)            //传递三个参数基准行，顶行，所有行与MID的差值
// 1486 {
get_linear_factor:
        PUSH     {R0-R2,R4-R11,LR}
        MOVS     R1,R0
// 1487     uint8 i;
// 1488     uint8 Y_aver=0;
        MOVS     R4,#+0
// 1489     float X_square_sum=0;   //X轴平方和
        MOVS     R5,#+0
// 1490     float Y_square_sum=0;   //Y轴平方和
        MOVS     R7,#+0
// 1491     float multi_sum=0;      //XY乘积之和
        MOVS     R6,#+0
// 1492     int temp=0,temp1=0,temp2=0; //减轻浮点运算的寄存器
        MOVS     R8,#+0
        MOVS     R9,#+0
        MOVS     R10,#+0
// 1493     float factor=0;
        MOVS     R0,#+0
// 1494     
// 1495      Y_aver=(uint8)((bottom+top)/2);  //Y坐标的范围
        LDRB     R2,[SP, #+4]
        UXTAB    R2,R2,R1
        MOVS     R3,#+2
        SDIV     R2,R2,R3
        MOVS     R4,R2
// 1496     for(i=bottom;i<=top;i++)
        MOV      R11,R1
        B.N      ??get_linear_factor_0
// 1497     {
// 1498         temp=temp+(center_white[i]-average)*(center_white[i]-average);
??get_linear_factor_1:
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        LDR.W    R0,??DataTable11_3
        LDRB     R0,[R11, R0]
        LDRB     R1,[SP, #+8]
        SUBS     R0,R0,R1
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        LDR.W    R1,??DataTable11_3
        LDRB     R1,[R11, R1]
        LDRB     R2,[SP, #+8]
        SUBS     R1,R1,R2
        MLA      R8,R1,R0,R8
// 1499         if(temp>30000)
        MOVW     R0,#+30001
        CMP      R8,R0
        BLT.N    ??get_linear_factor_2
// 1500         {
// 1501             X_square_sum=X_square_sum+temp;   //X平方和
        MOV      R0,R8
        BL       __aeabi_i2f
        MOVS     R1,R5
        BL       __aeabi_fadd
        MOVS     R5,R0
// 1502             temp=0;
        MOVS     R8,#+0
// 1503         }
// 1504 
// 1505         temp1=temp1+(i-Y_aver)*(i-Y_aver);
??get_linear_factor_2:
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R0,R11,R4
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R1,R11,R4
        MLA      R9,R1,R0,R9
// 1506         if(temp1>30000)
        MOVW     R0,#+30001
        CMP      R9,R0
        BLT.N    ??get_linear_factor_3
// 1507         {
// 1508             Y_square_sum=Y_square_sum+temp1;   //Y平方和
        MOV      R0,R9
        BL       __aeabi_i2f
        MOVS     R1,R7
        BL       __aeabi_fadd
        MOVS     R7,R0
// 1509             temp1=0;
        MOVS     R9,#+0
// 1510         }
// 1511 
// 1512         temp2=temp2+(center_white[i]-average)*(i-Y_aver);
??get_linear_factor_3:
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        LDR.N    R0,??DataTable11_3
        LDRB     R0,[R11, R0]
        LDRB     R1,[SP, #+8]
        SUBS     R0,R0,R1
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R1,R11,R4
        MLA      R10,R1,R0,R10
// 1513         if(f_abs16(temp2)>30000)
        MOV      R0,R10
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVW     R1,#+30001
        CMP      R0,R1
        BLT.N    ??get_linear_factor_4
// 1514         {
// 1515             multi_sum=multi_sum+temp2;    //X、Y的积
        MOV      R0,R10
        BL       __aeabi_i2f
        MOVS     R1,R6
        BL       __aeabi_fadd
        MOVS     R6,R0
// 1516             temp2=0;
        MOVS     R10,#+0
// 1517         }
// 1518     }
??get_linear_factor_4:
        ADDS     R11,R11,#+1
??get_linear_factor_0:
        LDRB     R0,[SP, #+4]
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        CMP      R0,R11
        BCS.N    ??get_linear_factor_1
// 1519      
// 1520         X_square_sum=X_square_sum+temp;   //得出x的平方和
        MOV      R0,R8
        BL       __aeabi_i2f
        MOVS     R1,R5
        BL       __aeabi_fadd
        MOVS     R5,R0
// 1521         Y_square_sum=Y_square_sum+temp1;  //计算出y的平方和
        MOV      R0,R9
        BL       __aeabi_i2f
        MOVS     R1,R7
        BL       __aeabi_fadd
        MOVS     R7,R0
// 1522         multi_sum=multi_sum+temp2;        //计算出xy的乘积
        MOV      R0,R10
        BL       __aeabi_i2f
        MOVS     R1,R6
        BL       __aeabi_fadd
        MOVS     R6,R0
// 1523     
// 1524        /* XX_square_sum =X_square_sum;      //用于检测
// 1525         YY_square_sum =Y_square_sum;
// 1526         XYmulti_sum = multi_sum;*/
// 1527         
// 1528         if(X_square_sum<0.1)   //防止除数为0
        MOVS     R0,R5
        LDR.W    R1,??DataTable12_1  ;; 0x3dcccccd
        BL       __aeabi_cfcmple
        BCS.N    ??get_linear_factor_5
// 1529             X_square_sum=0.1;
        LDR.W    R5,??DataTable12_1  ;; 0x3dcccccd
// 1530         if(Y_square_sum<0.1)
??get_linear_factor_5:
        MOVS     R0,R7
        LDR.W    R1,??DataTable12_1  ;; 0x3dcccccd
        BL       __aeabi_cfcmple
        BCS.N    ??get_linear_factor_6
// 1531             Y_square_sum=0.1;
        LDR.W    R7,??DataTable12_1  ;; 0x3dcccccd
// 1532         
// 1533         if(X_square_sum<300)  //小于300出现在直道
??get_linear_factor_6:
        MOVS     R0,R5
        LDR.W    R1,??DataTable12_2  ;; 0x43960000
        BL       __aeabi_cfcmple
        BCS.N    ??get_linear_factor_7
// 1534             factor=multi_sum/f_absf(multi_sum); // =1 or =-1 //完全是直线
        MOVS     R0,R6
        BL       f_absf
        MOVS     R1,R0
        MOVS     R0,R6
        BL       __aeabi_fdiv
        B.N      ??get_linear_factor_8
// 1535         else  //否则用公式计算 //注意处理速度，小于200时处理时间少很多
// 1536             factor=multi_sum/sqrt(X_square_sum*Y_square_sum)*(bottom_whitebase+control_top_whiteline-20)/(65-20);
??get_linear_factor_7:
        MOVS     R0,R5
        MOVS     R1,R7
        BL       __aeabi_fmul
        BL       __aeabi_f2d
        BL       sqrt
        MOVS     R4,R0
        MOVS     R5,R1
        MOVS     R0,R6
        BL       __aeabi_f2d
        MOVS     R2,R4
        MOVS     R3,R5
        BL       __aeabi_ddiv
        MOVS     R4,R0
        MOVS     R5,R1
        LDR.N    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable11_8
        LDRH     R1,[R1, #+0]
        UXTAH    R0,R0,R1
        SUBS     R0,R0,#+20
        BL       __aeabi_i2d
        MOVS     R2,R4
        MOVS     R3,R5
        BL       __aeabi_dmul
        MOVS     R2,#+0
        LDR.W    R3,??DataTable12_3  ;; 0x40468000
        BL       __aeabi_ddiv
        BL       __aeabi_d2f
// 1537         
// 1538         if(factor>0.95)
??get_linear_factor_8:
        LDR.W    R1,??DataTable12_4  ;; 0x3f733334
        BL       __aeabi_cfrcmple
        BHI.N    ??get_linear_factor_9
// 1539           factor=1;
        MOVS     R0,#+1065353216
// 1540         if(factor<-0.95)
??get_linear_factor_9:
        LDR.W    R1,??DataTable12_5  ;; 0xbf733333
        BL       __aeabi_cfcmple
        BCS.N    ??get_linear_factor_10
// 1541           factor=-1;
        LDR.W    R0,??DataTable14  ;; 0xbf800000
// 1542     
// 1543         return factor;
??get_linear_factor_10:
        POP      {R1-R11,PC}      ;; return
// 1544 }
// 1545 
// 1546 
// 1547 /*加入起跑线检测的停车程序，利用摄像头进行检测
// 1548  这里的主要目的是给stop_flag置位,为了能够检测起跑线，必须保证的是检测的距离为起跑线前的20厘米以上。
// 1549 这里的图像而言的话就是图像的30行，
// 1550 以五米的速度来看的话，也就是必须保证两场图像至少有一场检测到，40ms*5m = 20厘米
// 1551 注意这里的检测是检测黑到白的跳变。这个比检测白到黑的跳变更加的准确
// 1552 */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1553 void check_start_stop_line()
// 1554 { 
check_start_stop_line:
        PUSH     {R4-R8,LR}
// 1555   int i,j;
// 1556   uint8 left_start_stop_hang = 0;
        MOVS     R4,#+0
// 1557   uint8 left_start_stop_flag = 0;
        MOVS     R5,#+0
// 1558   uint8 right_start_stop_hang = 0;
        MOVS     R6,#+0
// 1559   uint8 right_start_stop_flag = 0;
        MOVS     R7,#+0
// 1560   //弯道不检测起跑线
// 1561   if(top_whiteline - bottom_whitebase > 50)
        LDR.N    R0,??DataTable11_2
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable11_1
        LDRB     R1,[R1, #+0]
        SUBS     R0,R0,R1
        CMP      R0,#+51
        BLT.W    ??check_start_stop_line_0
// 1562   {
// 1563     for(i=bottom_whitebase+3;i< bottom_whitebase + 40;i++) //只是检测前二十五行，大于车身的前20几个厘米  
        LDR.N    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        ADDS     R8,R0,#+3
        B.N      ??check_start_stop_line_1
??check_start_stop_line_2:
        ADDS     R8,R8,#+1
??check_start_stop_line_1:
        LDR.N    R0,??DataTable11_1
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+40
        CMP      R8,R0
        BGE.W    ??check_start_stop_line_0
// 1564     {
// 1565         //至少保证中线的左右的三个点是白色的 并且要保证这个时候的行的状态为3
// 1566       if( top_whiteline >= 50
// 1567          && (VideoImage2[i - 1][ center_white[i]] -  VideoImage2[i + 1][ center_white[i]])< OT
// 1568           && (VideoImage2[i - 1][ center_white[i] - 1] -  VideoImage2[i + 1][ center_white[i] - 1]) < OT
// 1569            && (VideoImage2[i - 1][ center_white[i] + 1] -  VideoImage2[i + 1][ center_white[i] + 1]) < OT)
        LDR.N    R0,??DataTable11_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+50
        BCC.W    ??check_start_stop_line_3
        LDR.N    R0,??DataTable11_3
        LDRB     R0,[R8, R0]
        MOVS     R1,#+159
        LDR.W    R2,??DataTable13
        MLA      R1,R1,R8,R2
        ADDS     R0,R0,R1
        LDRB     R0,[R0, #-159]
        LDR.N    R1,??DataTable11_3
        LDRB     R1,[R8, R1]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #+159]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BGE.W    ??check_start_stop_line_3
        LDR.N    R0,??DataTable11_3
        LDRB     R0,[R8, R0]
        MOVS     R1,#+159
        LDR.W    R2,??DataTable13
        MLA      R1,R1,R8,R2
        ADDS     R0,R0,R1
        LDRB     R0,[R0, #-160]
        LDR.N    R1,??DataTable11_3
        LDRB     R1,[R8, R1]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #+158]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BGE.W    ??check_start_stop_line_3
        LDR.N    R0,??DataTable11_3
        LDRB     R0,[R8, R0]
        MOVS     R1,#+159
        LDR.W    R2,??DataTable13
        MLA      R1,R1,R8,R2
        ADDS     R0,R0,R1
        LDRB     R0,[R0, #-158]
        LDR.N    R1,??DataTable11_3
        LDRB     R1,[R8, R1]
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #+160]
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        CMP      R0,R1
        BGE.W    ??check_start_stop_line_3
// 1570       {//左右两边能够有两段是黑色的
// 1571         //这两段白色的检测用的是左右边沿各自向内缩减5格子
// 1572         for(j = left_black[i] + 5 ; j <   center_white[i] - 2;j++)
        LDR.N    R0,??DataTable11_4
        LDRB     R0,[R8, R0]
        ADDS     R0,R0,#+5
        B.N      ??check_start_stop_line_4
// 1573         {
// 1574           if((VideoImage2[i + 1][ j ] -  VideoImage2[i - 1][ j])> OT-10
// 1575              &&(VideoImage2[i + 1][ j + 1] -  VideoImage2[i - 1][ j+1])> OT-10
// 1576                &&(VideoImage2[i + 1][ j + 2] -  VideoImage2[i - 1][ j + 2])> OT-10)
??check_start_stop_line_5:
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+10
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+159]
        MOVS     R3,#+159
        LDR.W    R12,??DataTable13
        MLA      R3,R3,R8,R12
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-159]
        SUBS     R2,R2,R3
        CMP      R1,R2
        BGE.N    ??check_start_stop_line_6
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+10
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+160]
        MOVS     R3,#+159
        LDR.W    R12,??DataTable13
        MLA      R3,R3,R8,R12
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-158]
        SUBS     R2,R2,R3
        CMP      R1,R2
        BGE.N    ??check_start_stop_line_6
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+10
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+161]
        MOVS     R3,#+159
        LDR.W    R12,??DataTable13
        MLA      R3,R3,R8,R12
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-157]
        SUBS     R2,R2,R3
        CMP      R1,R2
        BGE.N    ??check_start_stop_line_6
// 1577           {
// 1578              left_start_stop_hang = i;
        MOV      R4,R8
// 1579              left_start_stop_flag = 1;
        MOVS     R5,#+1
// 1580           }
// 1581         }
??check_start_stop_line_6:
        ADDS     R0,R0,#+1
??check_start_stop_line_4:
        LDR.N    R1,??DataTable11_3
        LDRB     R1,[R8, R1]
        SUBS     R1,R1,#+2
        CMP      R0,R1
        BLT.N    ??check_start_stop_line_5
// 1582         
// 1583         for(j = right_black[i] - 5 ; j > center_white[i] + 2;j--)
        LDR.N    R0,??DataTable11_5
        LDRB     R0,[R8, R0]
        SUBS     R0,R0,#+5
        B.N      ??check_start_stop_line_7
// 1584         {
// 1585           if((VideoImage2[i + 1][ j] -  VideoImage2[i - 1][ j]) > OT-10
// 1586              &&(VideoImage2[i + 1][ j - 1] -  VideoImage2[i - 1][ j - 1])> OT-10
// 1587                &&(VideoImage2[i + 1][ j - 2] -  VideoImage2[i - 1][ j - 2])> OT-10 )
??check_start_stop_line_8:
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+10
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+159]
        MOVS     R3,#+159
        LDR.W    R12,??DataTable13
        MLA      R3,R3,R8,R12
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-159]
        SUBS     R2,R2,R3
        CMP      R1,R2
        BGE.N    ??check_start_stop_line_9
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+10
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+158]
        MOVS     R3,#+159
        LDR.W    R12,??DataTable13
        MLA      R3,R3,R8,R12
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-160]
        SUBS     R2,R2,R3
        CMP      R1,R2
        BGE.N    ??check_start_stop_line_9
        LDR.W    R1,??DataTable13_1
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+10
        MOVS     R2,#+159
        LDR.W    R3,??DataTable13
        MLA      R2,R2,R8,R3
        ADDS     R2,R0,R2
        LDRB     R2,[R2, #+157]
        MOVS     R3,#+159
        LDR.W    R12,??DataTable13
        MLA      R3,R3,R8,R12
        ADDS     R3,R0,R3
        LDRB     R3,[R3, #-161]
        SUBS     R2,R2,R3
        CMP      R1,R2
        BGE.N    ??check_start_stop_line_9
// 1588           {
// 1589              right_start_stop_hang = i;
        MOV      R6,R8
// 1590              right_start_stop_flag = 1;
        MOVS     R7,#+1
// 1591           }
// 1592         }//右边搜索 
??check_start_stop_line_9:
        SUBS     R0,R0,#+1
??check_start_stop_line_7:
        LDR.N    R1,??DataTable11_3
        LDRB     R1,[R8, R1]
        ADDS     R1,R1,#+2
        CMP      R1,R0
        BLT.N    ??check_start_stop_line_8
// 1593       } //中线符合标准
// 1594         //当找到了符合的起跑线的时候，跳出循环//判断是还要防止图像的错位
// 1595         if(  left_start_stop_flag == 1 && right_start_stop_flag == 1 && f_abs16(right_start_stop_hang - left_start_stop_hang) < 3)
??check_start_stop_line_3:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+1
        BNE.W    ??check_start_stop_line_2
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        CMP      R7,#+1
        BNE.W    ??check_start_stop_line_2
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R0,R6,R4
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        CMP      R0,#+3
        BGE.W    ??check_start_stop_line_2
// 1596         {
// 1597           stopflag = 1;
        LDR.W    R0,??DataTable15
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1598           break;//
// 1599         }
// 1600     }//for循环
// 1601   }
// 1602 }
??check_start_stop_line_0:
        POP      {R4-R8,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11:
        DC32     center_lost_hang

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_1:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_2:
        DC32     top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_3:
        DC32     center_white

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_4:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_5:
        DC32     right_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_6:
        DC32     danger_count

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_7:
        DC32     danger_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_8:
        DC32     control_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_9:
        DC32     deal_start_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_10:
        DC32     refer_road_width

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_11:
        DC32     S_right

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_12:
        DC32     S_left

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_13:
        DC32     S_straight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_14:
        DC32     direction

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_15:
        DC32     ramp_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_16:
        DC32     re_direction

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_17:
        DC32     center_average

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_18:
        DC32     center_error_average

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_19:
        DC32     center_linear_average

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_20:
        DC32     ramp_time

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_21:
        DC32     ramp_delay_time

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11_22:
        DC32     ramp_dis_time
// 1603 
// 1604 
// 1605 /*-----------------------------------舵机和电机的控制函数的变量---------------------------------
// 1606 这个函数用于进行赛道的优化和控制，其中控制包括电机和舵机，这两部分。对其进行统一的控制
// 1607 */
// 1608 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1609 void Control()
// 1610 {
Control:
        PUSH     {R3-R5,LR}
// 1611  int16 i=0,j=0; //
        MOVS     R1,#+0
        MOVS     R0,#+0
// 1612   p_error = 0;
        LDR.W    R2,??DataTable14_1
        MOVS     R3,#+0
        STRH     R3,[R2, #+0]
// 1613   refer_error = 0;
        LDR.W    R2,??DataTable14_2
        MOVS     R3,#+0
        STRH     R3,[R2, #+0]
// 1614   get_p_errorline = 55;
        LDR.W    R2,??DataTable14_3
        MOVS     R3,#+55
        STRB     R3,[R2, #+0]
// 1615   
// 1616   //求解偏差和舵机的控制p、d参数
// 1617   if(control_top_whiteline -deal_start_line >10)
        LDR.W    R2,??DataTable14_4
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable14_5
        LDRB     R3,[R3, #+0]
        SUBS     R2,R2,R3
        CMP      R2,#+11
        BLT.W    ??Control_0
// 1618   {
// 1619     if(control_top_whiteline < get_p_errorline )
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable14_3
        LDRB     R1,[R1, #+0]
        UXTH     R1,R1            ;; ZeroExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BCS.N    ??Control_1
// 1620       get_p_errorline = control_top_whiteline ;  //限制舵机的控制行舵机的控制行不能太长，太长容易在弯道入直道出现震荡
        LDR.W    R0,??DataTable14_3
        LDR.W    R1,??DataTable14_4
        LDRH     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
// 1621     
// 1622     for(j= get_p_errorline; j>= get_p_errorline -4; j--)
??Control_1:
        LDR.W    R0,??DataTable14_3
        LDRB     R0,[R0, #+0]
        B.N      ??Control_2
// 1623     {
// 1624       p_error += center_white[j];
??Control_3:
        LDR.W    R1,??DataTable14_1
        LDRH     R1,[R1, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R2,??DataTable15_1
        LDRB     R2,[R0, R2]
        ADDS     R1,R1,R2
        LDR.W    R2,??DataTable14_1
        STRH     R1,[R2, #+0]
// 1625     }
        SUBS     R0,R0,#+1
??Control_2:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable14_3
        LDRB     R1,[R1, #+0]
        SUBS     R1,R1,#+4
        CMP      R0,R1
        BGE.N    ??Control_3
// 1626     //得出本场的error  
// 1627     p_error  = (MID -  p_error/ 5);
        LDR.W    R0,??DataTable14_1
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#+5
        SDIV     R0,R0,R1
        RSBS     R0,R0,#+79
        LDR.W    R1,??DataTable14_1
        STRH     R0,[R1, #+0]
// 1628     
// 1629     //获取全场的一个偏差
// 1630     refer_error = MID - center_average;       //在车子出入直角弯道和一般的弯道进入直道的时候，容易出现震荡，这个和以前直接用加权平均的算法很相似，所以这里将其变的连续写
        LDR.W    R0,??DataTable14_6
        LDR      R0,[R0, #+0]
        RSBS     R0,R0,#+79
        LDR.W    R1,??DataTable14_2
        STRH     R0,[R1, #+0]
// 1631     
// 1632     ref_his_error[0] = ref_his_error[1] ;
        LDR.W    R0,??DataTable14_7
        LDRH     R0,[R0, #+2]
        LDR.W    R1,??DataTable14_7
        STRH     R0,[R1, #+0]
// 1633     ref_his_error[1] = ref_his_error[2] ;
        LDR.W    R0,??DataTable14_7
        LDR.W    R1,??DataTable14_7
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+2]
// 1634     ref_his_error[2] = ref_his_error[3] ;
        LDR.W    R0,??DataTable14_7
        LDR.W    R1,??DataTable14_7
        LDRH     R1,[R1, #+6]
        STRH     R1,[R0, #+4]
// 1635     ref_his_error[3] = ref_his_error[4] ;
        LDR.W    R0,??DataTable14_7
        LDR.W    R1,??DataTable14_7
        LDRH     R1,[R1, #+8]
        STRH     R1,[R0, #+6]
// 1636     ref_his_error[4] = refer_error ;
        LDR.W    R0,??DataTable14_7
        LDR.W    R1,??DataTable14_2
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+8]
// 1637     
// 1638     //得到加权之后的引导的偏差
// 1639    refer_error  =((ref_his_error[0] + ref_his_error[1] + ref_his_error[2]+ 2*ref_his_error[3])+95*ref_his_error[4])/100;  //取出顶端的10行，有利于对波浪弯道的控制
        LDR.W    R0,??DataTable14_7
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable14_7
        LDRSH    R1,[R1, #+2]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable14_7
        LDRSH    R1,[R1, #+4]
        ADDS     R0,R0,R1
        LDR.W    R1,??DataTable14_7
        LDRSH    R1,[R1, #+6]
        ADDS     R0,R0,R1, LSL #+1
        LDR.W    R1,??DataTable14_7
        LDRSH    R1,[R1, #+8]
        MOVS     R2,#+95
        MLA      R0,R2,R1,R0
        MOVS     R1,#+100
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable14_2
        STRH     R0,[R1, #+0]
// 1640     
// 1641     //引导的error
// 1642    /*if((re_control_top_whiteline < control_top_whiteline) && (direction == 3 || direction== 4))  //re_control_top_whiteline top_error_servo_p
// 1643      top_error_servo_p = 1*(control_top_whiteline - re_control_top_whiteline)/6;
// 1644    else if((re_control_top_whiteline > control_top_whiteline) &&  direction== 4)        //入弯道加大打角
// 1645      top_error_servo_p = 1*(re_control_top_whiteline - control_top_whiteline)/6;
// 1646    else
// 1647    
// 1648    uint16 side_count_p = 0;
// 1649    side_count_p = 0;
// 1650    for(i=deal_start_line;i<=control_top_whiteline;i++)
// 1651    {
// 1652      if(center_white[i]>MID)
// 1653      {
// 1654         side_count_p ++;
// 1655      }
// 1656      else if(center_white[i]<MID)
// 1657      {
// 1658        side_count_p--;
// 1659      }
// 1660    }
// 1661    */
// 1662    if(direction == 1 && control_top_whiteline >= ROW - 2)  //当时直线状态的时候，顶部与底部有较大的偏差的时候，这个时候仍然需要一个p的作用
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??Control_4
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        CMP      R0,#+63
        BCC.N    ??Control_4
// 1663    {
// 1664      top_error_servo_p = 3*f_abs16(center_white[deal_start_line] - center_white[control_top_whiteline-1])/4;       //顶部与底部的偏差最大能达到70左右
        LDR.W    R0,??DataTable14_5
        LDRB     R0,[R0, #+0]
        LDR.W    R1,??DataTable15_1
        LDRB     R0,[R0, R1]
        LDR.W    R1,??DataTable14_4
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable15_1
        ADDS     R1,R1,R2
        LDRB     R1,[R1, #-1]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       f_abs16
        MOVS     R1,#+3
        MULS     R0,R1,R0
        MOVS     R1,#+4
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable15_3
        STRH     R0,[R1, #+0]
        B.N      ??Control_5
// 1665    }
// 1666    else
// 1667      top_error_servo_p = 0;
??Control_4:
        LDR.W    R0,??DataTable15_3
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1668      
// 1669     error_servo_p = 5*(ROW - control_top_whiteline )/13  +  1*f_abs16(p_error)/8 + lcd_error_servo_p;  //增加这个p有利于进弯道切
??Control_5:
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        RSBS     R0,R0,#+65
        MOVS     R1,#+5
        MULS     R0,R1,R0
        MOVS     R1,#+13
        SDIV     R4,R0,R1
        LDR.W    R0,??DataTable14_1
        LDRSH    R0,[R0, #+0]
        BL       f_abs16
        MOVS     R1,#+8
        SDIV     R0,R0,R1
        ADDS     R0,R0,R4
        LDR.W    R1,??DataTable15_4
        LDRSH    R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable15_5
        STRH     R0,[R1, #+0]
// 1670     error_servo_d =lcd_error_servo_d;//control_top_whiteline ;  //当在比较直的线路上要求d稍微大一点 波浪弯道，过不了一直是d的问题 ，太大了
        LDR.W    R0,??DataTable15_6
        LDR.W    R1,??DataTable15_7
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1671     
// 1672     //归中的error
// 1673     error_servo_ref_p = 1 * f_abs16(refer_error)/7 + lcd_ref_p + top_error_servo_p;  //增加这个p有利于出弯道内切     1/7
        LDR.W    R0,??DataTable14_2
        LDRSH    R0,[R0, #+0]
        BL       f_abs16
        MOVS     R1,#+7
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable15_8
        LDRSH    R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable15_3
        LDRH     R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable15_9
        STRH     R0,[R1, #+0]
// 1674     error_servo_ref_d = lcd_ref_d;//2*center_white_average;
        LDR.W    R0,??DataTable15_10
        LDR.W    R1,??DataTable15_11
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1675     
// 1676     angle=(uint16)(mid_angle + (error_servo_p * p_error + error_servo_d * (p_error - p_re_error) + 
// 1677                                 error_servo_ref_p * refer_error + error_servo_ref_d * (refer_error - re_refer_error)) / 10);//
        LDR.W    R0,??DataTable15_12
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable15_5
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable14_1
        LDRSH    R2,[R2, #+0]
        LDR.W    R3,??DataTable15_6
        LDRH     R3,[R3, #+0]
        LDR.W    R4,??DataTable14_1
        LDRSH    R4,[R4, #+0]
        LDR.W    R5,??DataTable15_13
        LDRSH    R5,[R5, #+0]
        SUBS     R4,R4,R5
        MULS     R3,R4,R3
        MLA      R1,R2,R1,R3
        LDR.W    R2,??DataTable15_9
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable14_2
        LDRSH    R3,[R3, #+0]
        MLA      R1,R3,R2,R1
        LDR.W    R2,??DataTable15_10
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable14_2
        LDRSH    R3,[R3, #+0]
        LDR.W    R4,??DataTable15_14
        LDRSH    R4,[R4, #+0]
        SUBS     R3,R3,R4
        MLA      R1,R3,R2,R1
        MOVS     R2,#+10
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable15_15
        STRH     R0,[R1, #+0]
        B.N      ??Control_6
// 1678   } 
// 1679  else
// 1680    angle = re_angle;
??Control_0:
        LDR.W    R0,??DataTable15_15
        LDR.W    R1,??DataTable15_16
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1681 
// 1682     if(angle > mid_angle +240)   //1570 235
??Control_6:
        LDR.W    R0,??DataTable15_12
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+240
        LDR.W    R1,??DataTable15_15
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??Control_7
// 1683       angle = mid_angle +240;
        LDR.W    R0,??DataTable15_12
        LDRSH    R0,[R0, #+0]
        ADDS     R0,R0,#+240
        LDR.W    R1,??DataTable15_15
        STRH     R0,[R1, #+0]
// 1684     if(angle<mid_angle -240)
??Control_7:
        LDR.W    R0,??DataTable15_15
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable15_12
        LDRH     R1,[R1, #+0]
        SUBS     R1,R1,#+240
        CMP      R0,R1
        BGE.N    ??Control_8
// 1685       angle=mid_angle -240;
        LDR.W    R0,??DataTable15_12
        LDRSH    R0,[R0, #+0]
        SUBS     R0,R0,#+240
        LDR.W    R1,??DataTable15_15
        STRH     R0,[R1, #+0]
// 1686       FTM0_C3V=angle;
??Control_8:
        LDR.W    R0,??DataTable15_17  ;; 0x40038028
        LDR.W    R1,??DataTable15_15
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
// 1687       //这个时候可以调节舵机的中心值来使得车子跑正
// 1688       //历史值的保留
// 1689       re_angle= angle;
        LDR.W    R0,??DataTable15_16
        LDR.W    R1,??DataTable15_15
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1690   p_re_error = p_error;
        LDR.W    R0,??DataTable15_13
        LDR.W    R1,??DataTable14_1
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1691   re_refer_error =refer_error;
        LDR.W    R0,??DataTable15_14
        LDR.W    R1,??DataTable14_2
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1692   re_control_top_whiteline = control_top_whiteline;
        LDR.W    R0,??DataTable15_18
        LDR.W    R1,??DataTable14_4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1693   
// 1694   
// 1695   //下面是电机的控制程序 速度的控制融入了偏差和处理的有效行
// 1696  //对于这两点的应用，根据最高有效行动态的改变最高速和最低速
// 1697   //然后通过偏差来计算准确的速度
// 1698   
// 1699   //每次对速度需要重新的设定
// 1700 
// 1701     if(speed_select == 0)
        LDR.W    R0,??DataTable15_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Control_9
// 1702   {
// 1703        straight_speed = mySpeedswitch[0].Cstraightspeed;
        LDR.W    R0,??DataTable16
        LDR.W    R1,??DataTable15_20
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 1704        bow_speed = mySpeedswitch[0].Cbowspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+2]
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1705        straight_speed_ed = mySpeedswitch[0].Cstraightspeed_ed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+4]
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_10
// 1706    }
// 1707    else if(speed_select == 1)
??Control_9:
        LDR.W    R0,??DataTable15_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??Control_11
// 1708    {
// 1709        straight_speed = mySpeedswitch[1].Cstraightspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+6]
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1710        bow_speed = mySpeedswitch[1].Cbowspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+8]
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1711        straight_speed_ed = mySpeedswitch[1].Cstraightspeed_ed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+10]
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_10
// 1712    }
// 1713       else if(speed_select == 2)
??Control_11:
        LDR.W    R0,??DataTable15_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??Control_12
// 1714    {
// 1715        straight_speed = mySpeedswitch[2].Cstraightspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+12]
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1716        bow_speed = mySpeedswitch[2].Cbowspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+14]
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1717        straight_speed_ed = mySpeedswitch[2].Cstraightspeed_ed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+16]
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_10
// 1718    }
// 1719       else if(speed_select == 3)
??Control_12:
        LDR.W    R0,??DataTable15_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+3
        BNE.N    ??Control_13
// 1720    {
// 1721        straight_speed = mySpeedswitch[3].Cstraightspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+18]
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1722        bow_speed = mySpeedswitch[3].Cbowspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+20]
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1723        straight_speed_ed = mySpeedswitch[3].Cstraightspeed_ed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+22]
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_10
// 1724    }
// 1725       else if(speed_select == 4)
??Control_13:
        LDR.W    R0,??DataTable15_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+4
        BNE.N    ??Control_14
// 1726    {
// 1727        straight_speed = mySpeedswitch[4].Cstraightspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+24]
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1728        bow_speed = mySpeedswitch[4].Cbowspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+26]
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1729        straight_speed_ed = mySpeedswitch[4].Cstraightspeed_ed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+28]
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_10
// 1730    }
// 1731    else 
// 1732    {
// 1733        straight_speed = mySpeedswitch[5].Cstraightspeed;
??Control_14:
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+30]
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1734        bow_speed = mySpeedswitch[5].Cbowspeed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+32]
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1735        straight_speed_ed = mySpeedswitch[5].Cstraightspeed_ed;
        LDR.W    R0,??DataTable15_20
        LDRH     R0,[R0, #+34]
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
// 1736    }
// 1737    
// 1738    straight_speed += lcd_straight_speed;
??Control_10:
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable16_3
        LDRH     R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1739    bow_speed += lcd_bow_speed;
        LDR.W    R0,??DataTable16_1
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable16_4
        LDRH     R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1740    straight_speed_ed += lcd_straight_speed_ed;
        LDR.W    R0,??DataTable16_2
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable17
        LDRH     R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable16_2
        STRH     R0,[R1, #+0]
// 1741    
// 1742       if( direction == 1 )  //对于速度的控制实现不同的赛道用不同的控速
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??Control_15
// 1743        {
// 1744          straight_speed = (control_top_whiteline+ROW)*straight_speed/ROW/2;  //限定了最高速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+65
        LDR.W    R1,??DataTable16
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1745          bow_speed = (control_top_whiteline+ROW)*(bow_speed)/2/ROW;         //只是限定了最低速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+65
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
        B.N      ??Control_16
// 1746        }    
// 1747       else if( direction == 2 )  //波浪弯
??Control_15:
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??Control_17
// 1748        {
// 1749          straight_speed = (control_top_whiteline-2)*straight_speed/ROW;  //   限定了最高速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable16
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1750          bow_speed =  (control_top_whiteline-2)*(bow_speed)/ROW;         //只是限定了最低速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
        B.N      ??Control_16
// 1751        }
// 1752         else if( direction == 3)  //入弯
??Control_17:
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+3
        BNE.N    ??Control_18
// 1753        {
// 1754          straight_speed = (control_top_whiteline-4)*straight_speed/ROW;  //   限定了最高速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable16
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1755          bow_speed =  (control_top_whiteline-4)*(bow_speed)/ROW;         //只是限定了最低速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
        B.N      ??Control_16
// 1756        }
// 1757     
// 1758        else   //弯道
// 1759        { 
// 1760         straight_speed = (control_top_whiteline-4)*straight_speed/ROW;  //   限定了最高速度
??Control_18:
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable16
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16
        STRH     R0,[R1, #+0]
// 1761          bow_speed =   (control_top_whiteline-4)*(bow_speed)/ROW;         //只是限定了最低速度
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        MULS     R0,R1,R0
        MOVS     R1,#+65
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable16_1
        STRH     R0,[R1, #+0]
// 1762        }
// 1763       
// 1764         
// 1765         //center_error_average的变化范围不超过30  
// 1766       if(control_top_whiteline ==ROW - 1&&(direction==1||direction==2) )
??Control_16:
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        CMP      R0,#+64
        BNE.N    ??Control_19
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Control_20
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??Control_19
// 1767           {
// 1768             if (direction == 1 )
??Control_20:
        LDR.W    R0,??DataTable15_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??Control_21
// 1769             {
// 1770               straight_count++;
        LDR.W    R0,??DataTable17_1
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable17_1
        STRH     R0,[R1, #+0]
// 1771               if(straight_count > 2)
        LDR.W    R0,??DataTable17_1
        LDRH     R0,[R0, #+0]
        CMP      R0,#+3
        BCC.N    ??Control_22
// 1772                speed_except = straight_speed + straight_speed_ed;  //直线和波浪弯道给全速行驶
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable16_2
        LDRH     R1,[R1, #+0]
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_23
// 1773               else
// 1774                speed_except = straight_speed; 
??Control_22:
        LDR.W    R0,??DataTable17_2
        LDR.W    R1,??DataTable16
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??Control_23
// 1775             }
// 1776             else
// 1777             {
// 1778               speed_except = straight_speed + 20;
??Control_21:
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+20
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_23
// 1779             }
// 1780           }
// 1781       /*处理长直道入弯道，首先是利用线性相关系数，计算出固定前瞻的线性相关性，（比如有1.9的前瞻，只是计算出前1.2米的相关系数），
// 1782       然后利用剩下前瞻，进行对是否入弯道进行判断，判断的方法是判断顶端行的变化趋势。若是同时弯向一边，则说明是进入了弯道。利用
// 1783       这点信息进行直道入弯道的判断，具体的减速是将速度减到弯道的速度，而不是将速度减小到比弯道还低。否则容易引起车子在弯道的不稳定。
// 1784       */
// 1785  
// 1786       else if(f_absf(linear_factor) == 1)   //由于只是在进弯道的时候进行判断，所以这里就是入弯状态
??Control_19:
        LDR.W    R0,??DataTable17_3
        LDR      R0,[R0, #+0]
        BL       f_absf
        MOVS     R1,#+1065353216
        BL       __aeabi_cfcmpeq
        BNE.W    ??Control_24
// 1787       {
// 1788         straight_count = 0;
        LDR.W    R0,??DataTable17_1
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1789         for(i = 50;i<control_top_whiteline;i++)
        MOVS     R1,#+50
        B.N      ??Control_25
??Control_26:
        ADDS     R1,R1,#+1
??Control_25:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        CMP      R1,R0
        BGE.N    ??Control_27
// 1790         {
// 1791         if((center_white[i-1]<center_white[i-2] && center_white[i]<center_white[i-1])
// 1792              || (center_white[i-1] > center_white[i-2] && center_white[i]>center_white[i-1]))
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable15_1
        ADDS     R0,R1,R0
        LDRB     R0,[R0, #-1]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R2,??DataTable15_1
        ADDS     R2,R1,R2
        LDRB     R2,[R2, #-2]
        CMP      R0,R2
        BCS.N    ??Control_28
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable15_1
        LDRB     R0,[R1, R0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R2,??DataTable15_1
        ADDS     R2,R1,R2
        LDRB     R2,[R2, #-1]
        CMP      R0,R2
        BCC.N    ??Control_29
??Control_28:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable15_1
        ADDS     R0,R1,R0
        LDRB     R0,[R0, #-2]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R2,??DataTable15_1
        ADDS     R2,R1,R2
        LDRB     R2,[R2, #-1]
        CMP      R0,R2
        BCS.N    ??Control_26
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable15_1
        ADDS     R0,R1,R0
        LDRB     R0,[R0, #-1]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R2,??DataTable15_1
        LDRB     R2,[R1, R2]
        CMP      R0,R2
        BCS.N    ??Control_26
// 1793                break;
// 1794         }
// 1795          full_speed_line = i;
??Control_29:
??Control_27:
        LDR.W    R0,??DataTable17_4
        STRB     R1,[R0, #+0]
// 1796          
// 1797          if(full_speed_line > 62)
        LDR.W    R0,??DataTable17_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+63
        BCC.N    ??Control_30
// 1798          {
// 1799            speed_except = MIN_INT((3*straight_speed+1*bow_speed)/4,120);
        MOVS     R1,#+120
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        MOVS     R2,#+3
        MULS     R0,R2,R0
        LDR.W    R2,??DataTable16_1
        LDRH     R2,[R2, #+0]
        UXTAH    R0,R0,R2
        MOVS     R2,#+4
        SDIV     R0,R0,R2
        BL       MIN_INT
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_23
// 1800          }
// 1801          else if(full_speed_line > 58)
??Control_30:
        LDR.W    R0,??DataTable17_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+59
        BCC.N    ??Control_31
// 1802          {
// 1803            speed_except = MIN_INT((3*straight_speed+bow_speed)/4,110);
        MOVS     R1,#+110
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        MOVS     R2,#+3
        MULS     R0,R2,R0
        LDR.W    R2,??DataTable16_1
        LDRH     R2,[R2, #+0]
        UXTAH    R0,R0,R2
        MOVS     R2,#+4
        SDIV     R0,R0,R2
        BL       MIN_INT
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_23
// 1804          }
// 1805          else if(full_speed_line > 54)
??Control_31:
        LDR.W    R0,??DataTable17_4
        LDRB     R0,[R0, #+0]
        CMP      R0,#+55
        BCC.N    ??Control_32
// 1806          {
// 1807            speed_except = MIN_INT((2*straight_speed+bow_speed)/3,100);
        MOVS     R1,#+100
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        LSLS     R0,R0,#+1
        LDR.W    R2,??DataTable16_1
        LDRH     R2,[R2, #+0]
        UXTAH    R0,R0,R2
        MOVS     R2,#+3
        SDIV     R0,R0,R2
        BL       MIN_INT
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_23
// 1808          }
// 1809          else
// 1810          {
// 1811            speed_except = MIN_INT((straight_speed+2*bow_speed)/3,94);
??Control_32:
        MOVS     R1,#+94
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        LDR.W    R2,??DataTable16_1
        LDRH     R2,[R2, #+0]
        LSLS     R2,R2,#+1
        UXTAH    R0,R2,R0
        MOVS     R2,#+3
        SDIV     R0,R0,R2
        BL       MIN_INT
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
        B.N      ??Control_23
// 1812          }
// 1813          
// 1814       }
// 1815       else
// 1816       {
// 1817         straight_count = 0;
??Control_24:
        LDR.W    R0,??DataTable17_1
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1818         if(straight_speed - center_error_average*center_error_average*(straight_speed - bow_speed)/900 < bow_speed )
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable18
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable18
        LDRH     R2,[R2, #+0]
        MULS     R1,R2,R1
        LDR.W    R2,??DataTable16
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable16_1
        LDRH     R3,[R3, #+0]
        SUBS     R2,R2,R3
        MULS     R1,R2,R1
        MOV      R2,#+900
        SDIV     R1,R1,R2
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??Control_33
// 1819           speed_except =  bow_speed; 
        LDR.W    R0,??DataTable17_2
        LDR.W    R1,??DataTable16_1
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??Control_23
// 1820         else 
// 1821           speed_except = straight_speed - center_error_average*center_error_average*(straight_speed - bow_speed)/900 ;
??Control_33:
        LDR.W    R0,??DataTable16
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable18
        LDRH     R1,[R1, #+0]
        LDR.W    R2,??DataTable18
        LDRH     R2,[R2, #+0]
        MULS     R1,R2,R1
        LDR.W    R2,??DataTable16
        LDRH     R2,[R2, #+0]
        LDR.W    R3,??DataTable16_1
        LDRH     R3,[R3, #+0]
        SUBS     R2,R2,R3
        MULS     R1,R2,R1
        MOV      R2,#+900
        SDIV     R1,R1,R2
        SUBS     R0,R0,R1
        LDR.W    R1,??DataTable17_2
        STRH     R0,[R1, #+0]
// 1822       }
// 1823       
// 1824       if(stopflag == 1 && speed_down_cnt <= 10)
??Control_23:
        LDR.W    R0,??DataTable15
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Control_34
        LDR.W    R0,??DataTable19
        LDRH     R0,[R0, #+0]
        CMP      R0,#+11
        BCS.N    ??Control_34
// 1825       {
// 1826         speed_down_cnt ++;
        LDR.W    R0,??DataTable19
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable19
        STRH     R0,[R1, #+0]
// 1827       }
// 1828 
// 1829 }
??Control_34:
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12:
        DC32     ramp_dis_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12_1:
        DC32     0x3dcccccd

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12_2:
        DC32     0x43960000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12_3:
        DC32     0x40468000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12_4:
        DC32     0x3f733334

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable12_5:
        DC32     0xbf733333
// 1830 
// 1831 //------------------------串口发送函数------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 1832 void SCI0_send_mesage(void)
// 1833 {
SCI0_send_mesage:
        PUSH     {R3-R5,LR}
// 1834     int i = 0,j = 0;
        MOVS     R4,#+0
        MOVS     R5,#+0
// 1835     static bool sci_temp = 0;
// 1836     DisableInterrupts;  //发送图像数据时，要关闭所有中断，否则会出错    
        CPSID i         
// 1837     if(send_mes == 1)  //图像
        LDR.W    R0,??DataTable19_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??SCI0_send_mesage_0
// 1838     {  
// 1839       while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
??SCI0_send_mesage_1:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_1
// 1840         UART0_D = WHITE_BLACK_OT;//由于阀值的不存在，故这里只是随便填写的一个数字
        LDR.W    R0,??DataTable18_2  ;; 0x4006a007
        LDR.W    R1,??DataTable18_3
        LDRB     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
// 1841               while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
??SCI0_send_mesage_2:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_2
// 1842         UART0_D = (uint8)ROW;
        LDR.W    R0,??DataTable18_2  ;; 0x4006a007
        MOVS     R1,#+65
        STRB     R1,[R0, #+0]
// 1843               while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
??SCI0_send_mesage_3:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_3
// 1844         UART0_D = (uint8)COLUMN;
        LDR.W    R0,??DataTable18_2  ;; 0x4006a007
        MOVS     R1,#+159
        STRB     R1,[R0, #+0]
// 1845         
// 1846         
// 1847         //上位机显示的第一个点是左上角，所以我发的时候第一个点就发左上角的点
// 1848       for(i =ROW-1;i>=0;i--)
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_4
// 1849       {
// 1850         for(j=0;j<COLUMN;j++)
// 1851         {
// 1852           while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
??SCI0_send_mesage_5:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_5
// 1853           UART0_D =  VideoImage2[i][j];///见最后一个函数讲解
        MOVS     R0,#+159
        LDR.N    R1,??DataTable13
        MLA      R0,R0,R4,R1
        LDRB     R0,[R5, R0]
        LDR.W    R1,??DataTable18_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
// 1854           Delay_MS(80000);
        LDR.W    R0,??DataTable18_4  ;; 0x13880
        BL       Delay_MS
// 1855         }
        ADDS     R5,R5,#+1
??SCI0_send_mesage_6:
        CMP      R5,#+159
        BLT.N    ??SCI0_send_mesage_5
        SUBS     R4,R4,#+1
??SCI0_send_mesage_4:
        CMP      R4,#+0
        BMI.N    ??SCI0_send_mesage_7
        MOVS     R5,#+0
        B.N      ??SCI0_send_mesage_6
// 1856       }  
// 1857       for (i =ROW-1 ;i >=0 ; i--)
??SCI0_send_mesage_7:
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_8
// 1858       {
// 1859         while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
??SCI0_send_mesage_9:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_9
// 1860            UART0_D = left_black[i];
        LDR.W    R0,??DataTable18_5
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable18_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
// 1861         Delay_MS(80000);
        LDR.W    R0,??DataTable18_4  ;; 0x13880
        BL       Delay_MS
// 1862       }
        SUBS     R4,R4,#+1
??SCI0_send_mesage_8:
        CMP      R4,#+0
        BPL.N    ??SCI0_send_mesage_9
// 1863       for (i =ROW-1 ;i >=0 ; i--)
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_10
// 1864       {
// 1865          while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
??SCI0_send_mesage_11:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_11
// 1866          UART0_D = right_black[i];
        LDR.W    R0,??DataTable18_6
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable18_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
// 1867          Delay_MS(80000);
        LDR.W    R0,??DataTable18_4  ;; 0x13880
        BL       Delay_MS
// 1868       }
        SUBS     R4,R4,#+1
??SCI0_send_mesage_10:
        CMP      R4,#+0
        BPL.N    ??SCI0_send_mesage_11
// 1869       for (i =ROW-1 ;i >=0 ; i--)
        MOVS     R4,#+64
        B.N      ??SCI0_send_mesage_12
// 1870       {
// 1871         while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
??SCI0_send_mesage_13:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_13
// 1872         UART0_D = center_white[i];
        LDR.W    R0,??DataTable15_1
        LDRB     R0,[R4, R0]
        LDR.W    R1,??DataTable18_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
// 1873         Delay_MS(80000);
        LDR.W    R0,??DataTable18_4  ;; 0x13880
        BL       Delay_MS
// 1874       }
        SUBS     R4,R4,#+1
??SCI0_send_mesage_12:
        CMP      R4,#+0
        BPL.N    ??SCI0_send_mesage_13
// 1875       send_mes = 0;  //发送一次即可，所以要清零
        LDR.W    R0,??DataTable19_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??SCI0_send_mesage_14
// 1876      }
// 1877    // EnableInterrupts;  //重新开启所有中断
// 1878     
// 1879     else if(send_mes == 2)
??SCI0_send_mesage_0:
        LDR.W    R0,??DataTable19_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??SCI0_send_mesage_15
// 1880     {
// 1881       for(i =0;i<=ROW - 1;i++)
        MOVS     R4,#+0
??SCI0_send_mesage_16:
        CMP      R4,#+65
        BGE.N    ??SCI0_send_mesage_14
// 1882       {
// 1883           while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
??SCI0_send_mesage_17:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_17
// 1884           UART0_D = center_white[control_top_whiteline];
        LDR.N    R0,??DataTable14_4
        LDRH     R0,[R0, #+0]
        LDR.N    R1,??DataTable15_1
        LDRB     R0,[R0, R1]
        LDR.W    R1,??DataTable18_2  ;; 0x4006a007
        STRB     R0,[R1, #+0]
// 1885           Delay_MS(80000);
        LDR.W    R0,??DataTable18_4  ;; 0x13880
        BL       Delay_MS
// 1886       }
        ADDS     R4,R4,#+1
        B.N      ??SCI0_send_mesage_16
// 1887        
// 1888     }
// 1889  
// 1890    else if (send_mes == 3)  //便于调试
??SCI0_send_mesage_15:
        LDR.W    R0,??DataTable19_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+3
        BNE.N    ??SCI0_send_mesage_18
// 1891     {  
// 1892         if(!sci_temp)
        LDR.W    R0,??DataTable18_7
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??SCI0_send_mesage_19
// 1893         {
// 1894        while(!(UART0_S1&UART_S1_TDRE_MASK));
??SCI0_send_mesage_20:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_20
// 1895        UART0_D = (uint8)(speed_feedback);//speed_feedback
        LDR.W    R0,??DataTable18_2  ;; 0x4006a007
        LDR.W    R1,??DataTable18_8
        LDRH     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
// 1896        sci_temp = !sci_temp;
        LDR.W    R0,??DataTable18_7
        LDRB     R0,[R0, #+0]
        EORS     R0,R0,#0x1
        LDR.W    R1,??DataTable18_7
        STRB     R0,[R1, #+0]
        B.N      ??SCI0_send_mesage_14
// 1897         }
// 1898         else
// 1899         {
// 1900          while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
??SCI0_send_mesage_19:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_19
// 1901          UART0_D= (uint8)(speed_except);
        LDR.W    R0,??DataTable18_2  ;; 0x4006a007
        LDR.W    R1,??DataTable17_2
        LDRH     R1,[R1, #+0]
        STRB     R1,[R0, #+0]
// 1902           sci_temp = !sci_temp;
        LDR.W    R0,??DataTable18_7
        LDRB     R0,[R0, #+0]
        EORS     R0,R0,#0x1
        LDR.W    R1,??DataTable18_7
        STRB     R0,[R1, #+0]
        B.N      ??SCI0_send_mesage_14
// 1903         }
// 1904         //       send_mes = 0;不清0是为了连续发送
// 1905       }
// 1906     else if (send_mes =='p')  //停车
??SCI0_send_mesage_18:
        LDR.W    R0,??DataTable19_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+112
        BNE.N    ??SCI0_send_mesage_21
// 1907     {
// 1908        while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
??SCI0_send_mesage_22:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_22
// 1909        stopflag = 1;
        LDR.N    R0,??DataTable15
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 1910        send_mes = 0;
        LDR.W    R0,??DataTable19_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??SCI0_send_mesage_14
// 1911     }
// 1912     else if (send_mes == 's')  //启动
??SCI0_send_mesage_21:
        LDR.W    R0,??DataTable19_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+115
        BNE.N    ??SCI0_send_mesage_14
// 1913     {
// 1914        while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
??SCI0_send_mesage_23:
        LDR.W    R0,??DataTable18_1  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??SCI0_send_mesage_23
// 1915        stopflag = 0;
        LDR.N    R0,??DataTable15
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1916        send_mes = 0;  
        LDR.W    R0,??DataTable19_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1917     }
// 1918     EnableInterrupts;
??SCI0_send_mesage_14:
        CPSIE i         
// 1919 }
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13:
        DC32     VideoImage2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_1:
        DC32     OT

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
??sci_temp:
        DS8 1
// 1920 
// 1921 //拨码用的八位对应到k60上是 PTD6---- PTD13
// 1922 // 0000 0000 0000 0000 0000 0000 0000 0000 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1923 void scan_boma(void)
// 1924 {
// 1925   uint32 temp1=0;
scan_boma:
        MOVS     R0,#+0
// 1926   uint8 temp=0;
        MOVS     R1,#+0
// 1927   GPIOD_PDOR = 0xffffffff; 
        LDR.W    R2,??DataTable19_2  ;; 0x400ff0c0
        MOVS     R3,#-1
        STR      R3,[R2, #+0]
// 1928   temp1 = GPIOD_PDIR;   //读PTD6~PTD13 
        LDR.W    R2,??DataTable19_3  ;; 0x400ff0d0
        LDR      R2,[R2, #+0]
        MOVS     R0,R2
// 1929   
// 1930   temp = !(uint8)((temp1 >> 13) & 0x00000001);
        LSRS     R2,R0,#+13
        ANDS     R2,R2,#0x1
        EORS     R2,R2,#0x1
        MOVS     R1,R2
// 1931   if(lcd_debug == 1)
        LDR.W    R2,??DataTable19_4
        LDRB     R2,[R2, #+0]
        CMP      R2,#+0
        BEQ.N    ??scan_boma_0
// 1932   {
// 1933     if(temp == 1)   //对应的是拨码8
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+1
        BNE.N    ??scan_boma_1
// 1934      lcd_debug = 1;  //点亮的状态为按键调节时间，若调节完毕，则拨码检测结束，程序向下运行
        LDR.W    R1,??DataTable19_4
        MOVS     R2,#+1
        STRB     R2,[R1, #+0]
        B.N      ??scan_boma_0
// 1935   else
// 1936      lcd_debug = 0;
??scan_boma_1:
        LDR.W    R1,??DataTable19_4
        MOVS     R2,#+0
        STRB     R2,[R1, #+0]
// 1937   }
// 1938   temp = !(uint8)((temp1 >> 12) & 0x00000001);//对应的是拨码7
??scan_boma_0:
        LSRS     R1,R0,#+12
        ANDS     R1,R1,#0x1
        EORS     R1,R1,#0x1
// 1939   if(temp == 1)
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+1
        BNE.N    ??scan_boma_2
// 1940        redraw_control=1;  //拨亮则刷屏
        LDR.W    R1,??DataTable19_5
        MOVS     R2,#+1
        STRB     R2,[R1, #+0]
        B.N      ??scan_boma_3
// 1941   else
// 1942        redraw_control=0; 
??scan_boma_2:
        LDR.W    R1,??DataTable19_5
        MOVS     R2,#+0
        STRB     R2,[R1, #+0]
// 1943   
// 1944    //  start_stop_cs 起跑线检测的片选信号
// 1945     temp = !(uint8)((temp1 >> 11) & 0x00000001);//0x00000800
??scan_boma_3:
        LSRS     R0,R0,#+11
        ANDS     R0,R0,#0x1
        EORS     R1,R0,#0x1
// 1946    if(temp == 1)
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+1
        BNE.N    ??scan_boma_4
// 1947      start_stop_cs = 1;
        LDR.W    R0,??DataTable21
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??scan_boma_5
// 1948    else
// 1949      start_stop_cs = 0;
??scan_boma_4:
        LDR.W    R0,??DataTable21
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 1950    
// 1951   //  速度选择信号
// 1952    /*temp = !(uint8)((temp1 >> 10) & 0x00000001);//0x00000800
// 1953       if(temp == 1)
// 1954      stop_delay_check = 1;  //匀速，用于测试
// 1955       else
// 1956      stop_delay_check = 0;*/
// 1957    
// 1958 }
??scan_boma_5:
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14:
        DC32     0xbf800000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_1:
        DC32     p_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_2:
        DC32     refer_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_3:
        DC32     get_p_errorline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_4:
        DC32     control_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_5:
        DC32     deal_start_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_6:
        DC32     center_average

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable14_7:
        DC32     ref_his_error
// 1959 
// 1960 
// 1961 //-----------------------------延迟-------------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1962 void Delay_MS(uint32 ms)
// 1963 {
// 1964    while(ms--);
Delay_MS:
??Delay_MS_0:
        MOVS     R1,R0
        SUBS     R0,R1,#+1
        CMP      R1,#+0
        BNE.N    ??Delay_MS_0
// 1965 }
        BX       LR               ;; return
// 1966 
// 1967 
// 1968 /***********************************预显示**********************************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1969 void pre_show(void)
// 1970 {
pre_show:
        PUSH     {R7,LR}
// 1971   LCD_CLS();
        BL       LCD_CLS
// 1972    switch(lcd_page_num)
        LDR.W    R0,??DataTable19_6
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??pre_show_0
        CMP      R0,#+2
        BEQ.N    ??pre_show_1
        BCC.N    ??pre_show_2
        B.N      ??pre_show_3
// 1973    {        
// 1974       case 0:     //第一页只是用来显示参数      
// 1975              break;        
??pre_show_0:
        B.N      ??pre_show_3
// 1976              
// 1977       case 1:
// 1978              LCD_P6x8Cha(0,lcd_line_num,'*');
??pre_show_2:
        MOVS     R2,#+42
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 1979              
// 1980              LCD_P6x8Str(10,0,"lcd_debug:");      //调试选择
        LDR.W    R2,??DataTable19_7
        MOVS     R1,#+0
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1981              LCD_P6x8Num(100,0,lcd_debug);  
        LDR.W    R0,??DataTable19_4
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 1982              
// 1983              LCD_P6x8Str(10,1,"speed_select:");      //速度档位选择
        LDR.W    R2,??DataTable20_1
        MOVS     R1,#+1
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1984              LCD_P6x8Num(100,1,speed_select);
        LDR.N    R0,??DataTable15_19
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 1985              
// 1986              LCD_P6x8Str(10,2,"s_pit_count:");      //起跑线的延时检测计数
        LDR.W    R2,??DataTable20_2
        MOVS     R1,#+2
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1987              LCD_P6x8Num(100,2,stop_pit_count);
        LDR.W    R0,??DataTable20_3
        LDR      R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 1988              
// 1989              LCD_P6x8Str(10,3,"rampdetime:");      //坡道延时时间
        LDR.W    R2,??DataTable20_4
        MOVS     R1,#+3
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1990              LCD_P6x8Num(100,3,ramp_delay_time);
        LDR.W    R0,??DataTable20_5
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 1991              
// 1992              LCD_P6x8Str(10,4,"mid_angle:");      //第五行，舵机中值
        LDR.W    R2,??DataTable20_6
        MOVS     R1,#+4
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1993              LCD_P6x8Num(100,4,mid_angle);  
        LDR.N    R0,??DataTable15_12
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 1994                      
// 1995              LCD_P6x8Str(10,5,"WHITE_BLACK_OT:");      //第六行，二值化的阈值
        LDR.W    R2,??DataTable20_7
        MOVS     R1,#+5
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1996              LCD_P6x8Num(100,5,WHITE_BLACK_OT);  
        LDR.W    R0,??DataTable18_3
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 1997              
// 1998              LCD_P6x8Str(10,6,"test_run:");      //第六行，二值化的阈值
        LDR.W    R2,??DataTable20_8
        MOVS     R1,#+6
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 1999              LCD_P6x8Num(100,6,test_run); 
        LDR.W    R0,??DataTable20_9
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+6
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2000              
// 2001             break;
        B.N      ??pre_show_3
// 2002             
// 2003       case 2:
// 2004              LCD_P6x8Cha(0,lcd_line_num,'*');
??pre_show_1:
        MOVS     R2,#+42
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2005         
// 2006              LCD_P6x8Str(10,0,"l_er_ser_p:");    //第一行：舵机的p
        LDR.W    R2,??DataTable20_10
        MOVS     R1,#+0
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2007              LCD_P6x8Num(100,0,lcd_error_servo_p);
        LDR.N    R0,??DataTable15_4
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2008              
// 2009              LCD_P6x8Str(10,1,"l_er_ser_d:");    //第二行 ：舵机的d
        LDR.W    R2,??DataTable20_11
        MOVS     R1,#+1
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2010              LCD_P6x8Num(100,1,lcd_error_servo_d); 
        LDR.N    R0,??DataTable15_7
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2011              
// 2012              LCD_P6x8Str(10,2,"lcd_ref_p:");    //第三行 ：舵机的p
        LDR.W    R2,??DataTable20_12
        MOVS     R1,#+2
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2013              LCD_P6x8Num(100,2,lcd_ref_p);
        LDR.N    R0,??DataTable15_8
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2014              
// 2015              LCD_P6x8Str(10,3,"lcd_ref_d:");    //第四行 ：舵机的d
        LDR.W    R2,??DataTable20_13
        MOVS     R1,#+3
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2016              LCD_P6x8Num(100,3,lcd_ref_d);         
        LDR.N    R0,??DataTable15_11
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2017              
// 2018              LCD_P6x8Str(10,4,"l_str_speed:");     //直线速度
        LDR.W    R2,??DataTable20_14
        MOVS     R1,#+4
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2019              LCD_P6x8Num(100,4,lcd_straight_speed); 
        LDR.N    R0,??DataTable16_3
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2020              
// 2021              LCD_P6x8Str(10,5,"l_bow_speed:");     //弯道速度
        LDR.W    R2,??DataTable20_15
        MOVS     R1,#+5
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2022              LCD_P6x8Num(100,5,lcd_bow_speed); 
        LDR.N    R0,??DataTable16_4
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2023              
// 2024              
// 2025              LCD_P6x8Str(10,6,"lcd_strspe_ed:");     //长直道的附加速度
        LDR.W    R2,??DataTable20_16
        MOVS     R1,#+6
        MOVS     R0,#+10
        BL       LCD_P6x8Str
// 2026              LCD_P6x8Num(100,6,lcd_straight_speed_ed);
        LDR.N    R0,??DataTable17
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+6
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2027              
// 2028 
// 2029              break;
// 2030      }
// 2031 
// 2032 }
??pre_show_3:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15:
        DC32     stopflag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_1:
        DC32     center_white

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_2:
        DC32     direction

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_3:
        DC32     top_error_servo_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_4:
        DC32     lcd_error_servo_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_5:
        DC32     error_servo_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_6:
        DC32     error_servo_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_7:
        DC32     lcd_error_servo_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_8:
        DC32     lcd_ref_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_9:
        DC32     error_servo_ref_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_10:
        DC32     error_servo_ref_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_11:
        DC32     lcd_ref_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_12:
        DC32     mid_angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_13:
        DC32     p_re_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_14:
        DC32     re_refer_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_15:
        DC32     angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_16:
        DC32     re_angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_17:
        DC32     0x40038028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_18:
        DC32     re_control_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_19:
        DC32     speed_select

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable15_20:
        DC32     mySpeedswitch
// 2033 
// 2034 
// 2035 /*
// 2036 小的液晶屏的参数：
// 2037 128*64  写成6*8  每行共21个字符  一共可以写8列------
// 2038                                               |
// 2039                                               |
// 2040                                               |
// 2041                                               |
// 2042                                               |
// 2043                                               |
// 2044                                               |
// 2045 对于定位的x和y  x表示的是列数。这里以每一个点来计算，而不是每个单元格子
// 2046 而y则是以每个单元格子计算，代表的是8
// 2047 
// 2048 */
// 2049 /**************************************刷屏，显示时变变量*********************************/
// 2050 /*刷屏的时候特别是要注意时间的问题，可能刷屏的时间过长会导致控制的不及时，可能会出错
// 2051 这里将采用一场只刷屏一次的策略*/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2052 void redraw()
// 2053 {
redraw:
        PUSH     {R4,LR}
// 2054   byte lcd_hang = 1 ; 
        MOVS     R4,#+1
// 2055   if(lcd_page_num==0&&redraw_control==1)     //第一页//redraw_control需要另一个拨码进行控制
        LDR.W    R0,??DataTable19_6
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.W    ??redraw_0
        LDR.W    R0,??DataTable19_5
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??redraw_0
// 2056      {
// 2057          if(lcd_hang == 1)
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.N    ??redraw_1
// 2058          {
// 2059            LCD_CLS_ROW(0,0);      //0行  8列
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_CLS_ROW
// 2060            LCD_P6x8Num(0,0,top_whiteline);    //第一行  图像相关
        LDR.W    R0,??DataTable21_1
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Num
// 2061            LCD_P6x8Num(40,0,control_top_whiteline);  
        LDR.W    R0,??DataTable21_2
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+40
        BL       LCD_P6x8Num
// 2062            LCD_P6x8Num(80,0,deal_start_line); 
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 2063            lcd_hang ++;
        ADDS     R4,R4,#+1
// 2064          }
// 2065          if(lcd_hang == 2)
??redraw_1:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.N    ??redraw_2
// 2066          {
// 2067           LCD_CLS_ROW(0,1);       //第二行，舵机偏差相关
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_CLS_ROW
// 2068           LCD_P6x8Num(0,1,white_refer);
        LDR.W    R0,??DataTable21_4
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Num
// 2069           LCD_P6x8Num(40,1,center_average);
        LDR.W    R0,??DataTable21_5
        LDR      R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+40
        BL       LCD_P6x8Num
// 2070           LCD_P6x8Num(80,1,p_error);
        LDR.W    R0,??DataTable22
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 2071           LCD_P6x8Num(100,1, center_error_average);
        LDR.W    R0,??DataTable18
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2072           lcd_hang ++;
        ADDS     R4,R4,#+1
// 2073          }
// 2074         if(lcd_hang == 3)
??redraw_2:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+3
        BNE.N    ??redraw_3
// 2075          {
// 2076           LCD_CLS_ROW(0,2);       //第三行， 电机相关
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_CLS_ROW
// 2077           LCD_P6x8Num(0,2,speed); 
        LDR.W    R0,??DataTable22_1
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Num
// 2078           LCD_P6x8Num(60,2,speed_except); 
        LDR.N    R0,??DataTable17_2
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+60
        BL       LCD_P6x8Num
// 2079           LCD_P6x8Num(100,2, speed_feedback);
        LDR.W    R0,??DataTable18_8
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2080           lcd_hang ++;
        ADDS     R4,R4,#+1
// 2081          }
// 2082          if(lcd_hang == 4)
??redraw_3:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+4
        BNE.N    ??redraw_4
// 2083          {
// 2084           LCD_CLS_ROW(0,3);       //第四行，   //赛道的类型标志
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_CLS_ROW
// 2085           LCD_P6x8Num(0,3,S_straight);
        LDR.W    R0,??DataTable22_2
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Num
// 2086           LCD_P6x8Num(40,3,S_left); 
        LDR.W    R0,??DataTable22_3
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+40
        BL       LCD_P6x8Num
// 2087           LCD_P6x8Num(70,3,S_right);
        LDR.W    R0,??DataTable22_4
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+70
        BL       LCD_P6x8Num
// 2088           LCD_P6x8Num(110,3,direction);
        LDR.W    R0,??DataTable22_5
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+110
        BL       LCD_P6x8Num
// 2089           lcd_hang ++;
        ADDS     R4,R4,#+1
// 2090          }
// 2091            if(lcd_hang == 5)
??redraw_4:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+5
        BNE.N    ??redraw_0
// 2092          {
// 2093           LCD_CLS_ROW(0,4);       //第五行，   //标志位
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_CLS_ROW
// 2094           LCD_P6x8Num(0,4, stopflag); 
        LDR.N    R0,??DataTable18_9
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_P6x8Num
// 2095           LCD_P6x8Num(40,4, ramp_flag);
        LDR.W    R0,??DataTable22_6
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+40
        BL       LCD_P6x8Num
// 2096           LCD_P6x8Num(80,4, linear_factor);
        LDR.N    R0,??DataTable17_3
        LDR      R2,[R0, #+0]
        MOVS     R1,#+4
        MOVS     R0,#+80
        BL       LCD_P6x8Num
// 2097           lcd_hang =1;
        MOVS     R4,#+1
// 2098          }
// 2099        /*    if(lcd_hang == 6)
// 2100          {
// 2101           LCD_CLS_ROW(0,5);//第三行，   //线性相关
// 2102            LCD_P6x8Num(0,5, XX_square_sum); 
// 2103            LCD_P6x8Num(60,5, YY_square_sum);
// 2104            LCD_P6x8Num(110,5, XYmulti_sum);
// 2105            lcd_hang = 1;   //恢复置1
// 2106          }
// 2107                 */    
// 2108      }  
// 2109 }
??redraw_0:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16:
        DC32     straight_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_1:
        DC32     bow_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_2:
        DC32     straight_speed_ed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_3:
        DC32     lcd_straight_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable16_4:
        DC32     lcd_bow_speed
// 2110 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2111 void key_down(void)
// 2112 {
key_down:
        PUSH     {R3-R5,LR}
// 2113   uint8 temp=0;
        MOVS     R4,#+0
// 2114   uint32 temp1=0;
        MOVS     R5,#+0
// 2115   //端口由c8开始往后的八位设置为高点平
// 2116     Delay_MS(200000 * 4);
        LDR.W    R0,??DataTable22_7  ;; 0xc3500
        BL       Delay_MS
// 2117     GPIOC_PDOR = 0xffffffff; 
        LDR.W    R0,??DataTable22_8  ;; 0x400ff080
        MOVS     R1,#-1
        STR      R1,[R0, #+0]
// 2118     temp1 = GPIOC_PDIR;   //读PTC8~PTC15   
        LDR.W    R0,??DataTable22_9  ;; 0x400ff090
        LDR      R0,[R0, #+0]
        MOVS     R5,R0
// 2119     temp = ~(uint8)((temp1 >> 8) & 0x000000ff);
        LSRS     R0,R5,#+8
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MVNS     R0,R0
        MOVS     R4,R0
// 2120     
// 2121      if(temp==0x01)
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.N    ??key_down_0
// 2122         sub_NUM=1;//   
        LDR.W    R0,??DataTable22_10
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_1
// 2123      else
// 2124         sub_NUM=0;//  
??key_down_0:
        LDR.W    R0,??DataTable22_10
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2125      if(temp==0x02)
??key_down_1:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.N    ??key_down_2
// 2126        se_sub_NUM=1;//  
        LDR.W    R0,??DataTable22_11
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_3
// 2127      else
// 2128        se_sub_NUM=0;  //  
??key_down_2:
        LDR.W    R0,??DataTable22_11
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2129      if(temp==0x04)
??key_down_3:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+4
        BNE.N    ??key_down_4
// 2130         up_line=1;//   
        LDR.W    R0,??DataTable22_12
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_5
// 2131      else
// 2132         up_line=0; //   
??key_down_4:
        LDR.W    R0,??DataTable22_12
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2133      if(temp==0x08)
??key_down_5:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+8
        BNE.N    ??key_down_6
// 2134        change_page=1 ; // 
        LDR.W    R0,??DataTable22_13
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_7
// 2135      else
// 2136         change_page=0;// 
??key_down_6:
        LDR.W    R0,??DataTable22_13
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2137      if(temp==0x10)
??key_down_7:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+16
        BNE.N    ??key_down_8
// 2138         add_NUM=1 ;  //
        LDR.W    R0,??DataTable22_14
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_9
// 2139      else
// 2140         add_NUM=0    ; //
??key_down_8:
        LDR.W    R0,??DataTable22_14
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2141      if(temp==0x20)
??key_down_9:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+32
        BNE.N    ??key_down_10
// 2142          down_line=1;    
        LDR.W    R0,??DataTable23
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??key_down_11
// 2143      else
// 2144         down_line=0; 
??key_down_10:
        LDR.W    R0,??DataTable23
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2145     
// 2146    
// 2147 }
??key_down_11:
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17:
        DC32     lcd_straight_speed_ed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_1:
        DC32     straight_count

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_2:
        DC32     speed_except

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_3:
        DC32     linear_factor

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable17_4:
        DC32     full_speed_line
// 2148 
// 2149 //---------------------------全键盘扫描-----------------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2150 void Keyscan(void)
// 2151 {
Keyscan:
        PUSH     {R7,LR}
// 2152       key_down();
        BL       key_down
// 2153       
// 2154       if(change_page)  //如果检测到低电平，说明按键按下
        LDR.W    R0,??DataTable22_13
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_0
// 2155       {    key_down(); //延时去抖，一般10-20ms
        BL       key_down
// 2156            if(change_page)     //再次确认按键是否按下，没有按下则退出
        LDR.W    R0,??DataTable22_13
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_1
        B.N      ??Keyscan_0
// 2157            {   
// 2158               while(change_page)//如果确认按下按键等待按键释放，没有释放则一直等待
// 2159                 key_down();
??Keyscan_2:
        BL       key_down
??Keyscan_1:
        LDR.W    R0,??DataTable22_13
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_2
// 2160               if(lcd_page_num<2)    //页序号加操作这里的页数2是可以改动的
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BCS.N    ??Keyscan_3
// 2161 	         lcd_page_num++;
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable23_1
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_4
// 2162 	       else
// 2163 	         lcd_page_num=1;
??Keyscan_3:
        LDR.W    R0,??DataTable23_1
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2164                lcd_line_num=0;
??Keyscan_4:
        LDR.W    R0,??DataTable20
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2165               pre_show();//这里是显示函数，提前写在这里一会再写这个函数的函数体
        BL       pre_show
// 2166                
// 2167            }
// 2168       }
// 2169      //减页数 
// 2170       if(se_sub_NUM)  //如果检测到低电平，说明按键按下
??Keyscan_0:
        LDR.W    R0,??DataTable22_11
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_5
// 2171       {    key_down(); //延时去抖，一般10-20ms
        BL       key_down
// 2172            if(se_sub_NUM)     //再次确认按键是否按下，没有按下则退出
        LDR.W    R0,??DataTable22_11
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_6
        B.N      ??Keyscan_5
// 2173            {
// 2174               while(se_sub_NUM)//如果确认按下按键等待按键释放，没有释放则一直等待
// 2175               
// 2176                key_down();
??Keyscan_7:
        BL       key_down
??Keyscan_6:
        LDR.W    R0,??DataTable22_11
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_7
// 2177                LCD_change_value(lcd_page_num,lcd_line_num,-1);
        MOVS     R2,#-1
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        BL       LCD_change_value
// 2178                
// 2179            }
// 2180       }
// 2181       
// 2182       
// 2183      if(lcd_page_num!=0)     //如不为第一页，则进行下一步扫描  
??Keyscan_5:
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.W    ??Keyscan_8
// 2184      { //行扫描
// 2185       //向上
// 2186        key_down();
        BL       key_down
// 2187       if(up_line)  //如果检测到低电平，说明按键按下
        LDR.W    R0,??DataTable22_12
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_9
// 2188       {
// 2189             key_down(); //延时去抖，一般10-20ms
        BL       key_down
// 2190             if(up_line)     //再次确认按键是否按下，没有按下则退出
        LDR.W    R0,??DataTable22_12
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_10
        B.N      ??Keyscan_9
// 2191             {
// 2192                while(up_line)//如果确认按下按键等待按键释放，没有释放则一直等待
// 2193                  key_down();
??Keyscan_11:
        BL       key_down
??Keyscan_10:
        LDR.W    R0,??DataTable22_12
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_11
// 2194                if(lcd_page_num!=0)
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_12
// 2195 	        LCD_P6x8Cha(0,lcd_line_num,' ');
        MOVS     R2,#+32
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2196                
// 2197                
// 2198                 if(lcd_line_num<LCD_ROW)    //行序号加操作
??Keyscan_12:
        LDR.W    R0,??DataTable20
        LDRB     R0,[R0, #+0]
        CMP      R0,#+7
        BCS.N    ??Keyscan_13
// 2199 	         lcd_line_num++;
        LDR.W    R0,??DataTable20
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable20
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_14
// 2200 		 else
// 2201 		  lcd_line_num=0;
??Keyscan_13:
        LDR.W    R0,??DataTable20
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2202                 if(lcd_page_num!=0)  
??Keyscan_14:
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_9
// 2203               LCD_P6x8Cha(0,lcd_line_num,'*');
        MOVS     R2,#+42
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2204             }
// 2205       }
// 2206       //向下
// 2207        if(down_line)  //如果检测到低电平，说明按键按下
??Keyscan_9:
        LDR.W    R0,??DataTable23
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_15
// 2208       {
// 2209             key_down(); //延时去抖，一般10-20ms
        BL       key_down
// 2210             if(down_line)     //再次确认按键是否按下，没有按下则退出
        LDR.W    R0,??DataTable23
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_16
        B.N      ??Keyscan_15
// 2211             {
// 2212                while(down_line)//如果确认按下按键等待按键释放，没有释放则一直等待
// 2213                  key_down(); 
??Keyscan_17:
        BL       key_down
??Keyscan_16:
        LDR.W    R0,??DataTable23
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_17
// 2214                 if(lcd_page_num!=0)
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_18
// 2215 	        LCD_P6x8Cha(0,lcd_line_num,' ');
        MOVS     R2,#+32
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2216                 if(lcd_line_num>0)    //行序号加操作
??Keyscan_18:
        LDR.W    R0,??DataTable20
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BCC.N    ??Keyscan_19
// 2217 	         lcd_line_num--;
        LDR.W    R0,??DataTable20
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable20
        STRB     R0,[R1, #+0]
        B.N      ??Keyscan_20
// 2218 		 else
// 2219 		  lcd_line_num=LCD_ROW;
??Keyscan_19:
        LDR.W    R0,??DataTable20
        MOVS     R1,#+7
        STRB     R1,[R0, #+0]
// 2220                   
// 2221               LCD_P6x8Cha(0,lcd_line_num,'*');//
??Keyscan_20:
        MOVS     R2,#+42
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2222             }
// 2223       }
// 2224       
// 2225        if(add_NUM)  //如果检测到低电平，说明按键按下
??Keyscan_15:
        LDR.W    R0,??DataTable22_14
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_21
// 2226     {
// 2227 	key_down(); //延时去抖，一般10-20ms
        BL       key_down
// 2228      if(add_NUM)     //再次确认按键是否按下，没有按下则退出
        LDR.W    R0,??DataTable22_14
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_22
        B.N      ??Keyscan_21
// 2229 	   {
// 2230       while(add_NUM)//如果确认按下按键等待按键释放，没有释放则一直等待
// 2231         key_down();
??Keyscan_23:
        BL       key_down
??Keyscan_22:
        LDR.W    R0,??DataTable22_14
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_23
// 2232         LCD_change_value(lcd_page_num,lcd_line_num,1);
        MOVS     R2,#+1
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        BL       LCD_change_value
// 2233 	   }
// 2234      }
// 2235       
// 2236      
// 2237      if(sub_NUM)  //如果检测到低电平，说明按键按下
??Keyscan_21:
        LDR.W    R0,??DataTable22_10
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??Keyscan_8
// 2238     {
// 2239 	key_down(); //延时去抖，一般10-20ms
        BL       key_down
// 2240      if(sub_NUM)     //再次确认按键是否按下，没有按下则退出
        LDR.W    R0,??DataTable22_10
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_24
        B.N      ??Keyscan_8
// 2241 	   {
// 2242         while(sub_NUM)//如果确认按下按键等待按键释放，没有释放则一直等待
// 2243           key_down();
??Keyscan_25:
        BL       key_down
??Keyscan_24:
        LDR.W    R0,??DataTable22_10
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??Keyscan_25
// 2244      LCD_change_value(lcd_page_num,lcd_line_num,-1);
        MOVS     R2,#-1
        LDR.W    R0,??DataTable20
        LDRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable23_1
        LDRB     R0,[R0, #+0]
        BL       LCD_change_value
// 2245 	   }
// 2246     }
// 2247     
// 2248     
// 2249      }
// 2250  }
??Keyscan_8:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18:
        DC32     center_error_average

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_1:
        DC32     0x4006a004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_2:
        DC32     0x4006a007

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_3:
        DC32     WHITE_BLACK_OT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_4:
        DC32     0x13880

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_5:
        DC32     left_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_6:
        DC32     right_black

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_7:
        DC32     ??sci_temp

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_8:
        DC32     speed_feedback

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_9:
        DC32     stopflag
// 2251 //-------------------------上-电LCD键盘调试---------------------//

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2252 void LCD_change_value(unsigned char page,unsigned char m,int i)
// 2253 {
LCD_change_value:
        PUSH     {R4-R6,LR}
        MOVS     R4,R0
        MOVS     R5,R1
        MOVS     R6,R2
// 2254     
// 2255    if(page==1)
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.W    ??LCD_change_value_0
// 2256    {
// 2257     switch(m)
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BEQ.N    ??LCD_change_value_1
        CMP      R5,#+2
        BEQ.N    ??LCD_change_value_2
        BCC.N    ??LCD_change_value_3
        CMP      R5,#+4
        BEQ.N    ??LCD_change_value_4
        BCC.N    ??LCD_change_value_5
        CMP      R5,#+6
        BEQ.W    ??LCD_change_value_6
        BCC.W    ??LCD_change_value_7
        B.N      ??LCD_change_value_0
// 2258        {
// 2259         case 0:lcd_debug+=i;
??LCD_change_value_1:
        LDR.N    R0,??DataTable19_4
        LDRB     R0,[R0, #+0]
        ADDS     R0,R6,R0
        CMP      R0,#+0
        BEQ.N    ??LCD_change_value_8
        MOVS     R0,#+1
        B.N      ??LCD_change_value_9
??LCD_change_value_8:
        MOVS     R0,#+0
??LCD_change_value_9:
        LDR.N    R1,??DataTable19_4
        STRB     R0,[R1, #+0]
// 2260                 LCD_P6x8Cha(0,0,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2261                 LCD_CLS_ROW(100,0);
        MOVS     R1,#+0
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2262                 LCD_P6x8Num(100,0,lcd_debug);
        LDR.N    R0,??DataTable19_4
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2263                 break; 
        B.N      ??LCD_change_value_0
// 2264         case 1:speed_select+=i;
??LCD_change_value_3:
        LDR.W    R0,??DataTable23_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_2
        STRB     R0,[R1, #+0]
// 2265                 LCD_P6x8Cha(0,1,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2266                 LCD_CLS_ROW(100,1);
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2267                 LCD_P6x8Num(100,1,speed_select);
        LDR.W    R0,??DataTable23_2
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2268                 break;
        B.N      ??LCD_change_value_0
// 2269          case 2:stop_pit_count+=i;
??LCD_change_value_2:
        LDR.N    R0,??DataTable20_3
        LDR      R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.N    R1,??DataTable20_3
        STR      R0,[R1, #+0]
// 2270                 LCD_P6x8Cha(0,2,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2271                 LCD_CLS_ROW(100,2);
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2272                 LCD_P6x8Num(100,2,stop_pit_count);//lcd_ref_p
        LDR.N    R0,??DataTable20_3
        LDR      R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2273                 break;     
        B.N      ??LCD_change_value_0
// 2274          case 3:ramp_delay_time+=i;
??LCD_change_value_5:
        LDR.N    R0,??DataTable20_5
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.N    R1,??DataTable20_5
        STRH     R0,[R1, #+0]
// 2275                 LCD_P6x8Cha(0,3,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2276                 LCD_CLS_ROW(100,3);
        MOVS     R1,#+3
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2277                 LCD_P6x8Num(100,3,ramp_delay_time);//lcd_ref_p
        LDR.N    R0,??DataTable20_5
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2278                 break; 
        B.N      ??LCD_change_value_0
// 2279          case 4:mid_angle+=i;
??LCD_change_value_4:
        LDR.W    R0,??DataTable23_3
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_3
        STRH     R0,[R1, #+0]
// 2280                 LCD_P6x8Cha(0,4,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2281                 LCD_CLS_ROW(100,4);
        MOVS     R1,#+4
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2282                 LCD_P6x8Num(100,4,mid_angle);//lcd_ref_p
        LDR.W    R0,??DataTable23_3
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2283                 break;        
        B.N      ??LCD_change_value_0
// 2284          case 5:WHITE_BLACK_OT +=i;
??LCD_change_value_7:
        LDR.W    R0,??DataTable23_4
        LDRB     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_4
        STRB     R0,[R1, #+0]
// 2285                 LCD_P6x8Cha(0,5,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+5
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2286                 LCD_CLS_ROW(100,5);
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2287                 LCD_P6x8Num(100,5,WHITE_BLACK_OT);
        LDR.W    R0,??DataTable23_4
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2288                 break;  
        B.N      ??LCD_change_value_0
// 2289          case 6:test_run +=i;
??LCD_change_value_6:
        LDR.N    R0,??DataTable20_9
        LDRB     R0,[R0, #+0]
        ADDS     R0,R6,R0
        CMP      R0,#+0
        BEQ.N    ??LCD_change_value_10
        MOVS     R0,#+1
        B.N      ??LCD_change_value_11
??LCD_change_value_10:
        MOVS     R0,#+0
??LCD_change_value_11:
        LDR.N    R1,??DataTable20_9
        STRB     R0,[R1, #+0]
// 2290                 LCD_P6x8Cha(0,6,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+6
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2291                 LCD_CLS_ROW(100,6);
        MOVS     R1,#+6
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2292                 LCD_P6x8Num(100,6,test_run);
        LDR.N    R0,??DataTable20_9
        LDRB     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+6
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2293                 break;  
// 2294        }
// 2295    }
// 2296 
// 2297  if(page==2)
??LCD_change_value_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.W    ??LCD_change_value_12
// 2298      {
// 2299           switch(m)
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BEQ.N    ??LCD_change_value_13
        CMP      R5,#+2
        BEQ.N    ??LCD_change_value_14
        BCC.N    ??LCD_change_value_15
        CMP      R5,#+4
        BEQ.N    ??LCD_change_value_16
        BCC.N    ??LCD_change_value_17
        CMP      R5,#+6
        BEQ.W    ??LCD_change_value_18
        BCC.W    ??LCD_change_value_19
        B.N      ??LCD_change_value_12
// 2300           {
// 2301          case 0:lcd_error_servo_p+=i;
??LCD_change_value_13:
        LDR.W    R0,??DataTable23_5
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_5
        STRH     R0,[R1, #+0]
// 2302                 LCD_P6x8Cha(0,0,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2303                 LCD_CLS_ROW(100,0);
        MOVS     R1,#+0
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2304                 LCD_P6x8Num(100,0,lcd_error_servo_p);
        LDR.W    R0,??DataTable23_5
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+0
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2305                 break;
        B.N      ??LCD_change_value_12
// 2306          case 1:lcd_error_servo_d+=i;
??LCD_change_value_15:
        LDR.W    R0,??DataTable23_6
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_6
        STRH     R0,[R1, #+0]
// 2307                 LCD_P6x8Cha(0,1,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2308                 LCD_CLS_ROW(100,1);
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2309                 LCD_P6x8Num(100,1,lcd_error_servo_d);
        LDR.W    R0,??DataTable23_6
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+1
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2310                 break;       
        B.N      ??LCD_change_value_12
// 2311          case 2:lcd_ref_p+=i;
??LCD_change_value_14:
        LDR.W    R0,??DataTable23_7
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_7
        STRH     R0,[R1, #+0]
// 2312                 LCD_P6x8Cha(0,2,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2313                 LCD_CLS_ROW(100,2);
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2314                 LCD_P6x8Num(100,2,lcd_ref_p);
        LDR.W    R0,??DataTable23_7
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+2
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2315                 break;        
        B.N      ??LCD_change_value_12
// 2316          case 3:lcd_ref_d += i;
??LCD_change_value_17:
        LDR.W    R0,??DataTable23_8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_8
        STRH     R0,[R1, #+0]
// 2317                 LCD_P6x8Cha(0,3,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2318                 LCD_CLS_ROW(100,3);
        MOVS     R1,#+3
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2319                 LCD_P6x8Num(100,3,lcd_ref_d);
        LDR.W    R0,??DataTable23_8
        LDRSH    R0,[R0, #+0]
        BL       __aeabi_i2f
        MOVS     R2,R0
        MOVS     R1,#+3
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2320                 break;
        B.N      ??LCD_change_value_12
// 2321         case 4:lcd_straight_speed += i;
??LCD_change_value_16:
        LDR.W    R0,??DataTable23_9
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_9
        STRH     R0,[R1, #+0]
// 2322                 LCD_P6x8Cha(0,4,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+4
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2323                 LCD_CLS_ROW(100,4);
        MOVS     R1,#+4
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2324                 LCD_P6x8Num(100,4,lcd_straight_speed);
        LDR.W    R0,??DataTable23_9
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+4
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2325                 break;
        B.N      ??LCD_change_value_12
// 2326         case 5:lcd_bow_speed += i;
??LCD_change_value_19:
        LDR.W    R0,??DataTable23_10
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_10
        STRH     R0,[R1, #+0]
// 2327                 LCD_P6x8Cha(0,5,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+5
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2328                 LCD_CLS_ROW(100,5);
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2329                 LCD_P6x8Num(100,5,lcd_bow_speed);
        LDR.W    R0,??DataTable23_10
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2330                 break;
        B.N      ??LCD_change_value_12
// 2331        case 6:lcd_straight_speed_ed += i;
??LCD_change_value_18:
        LDR.W    R0,??DataTable23_11
        LDRH     R0,[R0, #+0]
        ADDS     R0,R6,R0
        LDR.W    R1,??DataTable23_11
        STRH     R0,[R1, #+0]
// 2332                 LCD_P6x8Cha(0,6,'*'); 
        MOVS     R2,#+42
        MOVS     R1,#+6
        MOVS     R0,#+0
        BL       LCD_P6x8Cha
// 2333                 LCD_CLS_ROW(100,6);
        MOVS     R1,#+6
        MOVS     R0,#+100
        BL       LCD_CLS_ROW
// 2334                 LCD_P6x8Num(100,6,lcd_straight_speed_ed);
        LDR.W    R0,??DataTable23_11
        LDRH     R0,[R0, #+0]
        BL       __aeabi_ui2f
        MOVS     R2,R0
        MOVS     R1,#+6
        MOVS     R0,#+100
        BL       LCD_P6x8Num
// 2335                 break;
// 2336 
// 2337           }
// 2338      }   
// 2339 }
??LCD_change_value_12:
        POP      {R4-R6,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19:
        DC32     speed_down_cnt

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_1:
        DC32     send_mes

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_2:
        DC32     0x400ff0c0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_3:
        DC32     0x400ff0d0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_4:
        DC32     lcd_debug

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_5:
        DC32     redraw_control

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_6:
        DC32     lcd_page_num

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_7:
        DC32     `?<Constant "lcd_debug:">`
// 2340 
// 2341 
// 2342 //-------------------------------------定义输入输出端口---------------------------------------------------//
// 2343 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2344 void PORT_Init(void)
// 2345 { 
// 2346     
// 2347     PORTB_PCR20 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//PTC1引脚设置为GPIO模式 上拉
PORT_Init:
        LDR.W    R0,??DataTable23_12  ;; 0x4004a050
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2348     
// 2349     PORTE_PCR0 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E4引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_13  ;; 0x4004d000
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2350     PORTE_PCR1 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E5引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_14  ;; 0x4004d004
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2351     PORTE_PCR2 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E6引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_15  ;; 0x4004d008
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2352     PORTE_PCR3 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E7引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_16  ;; 0x4004d00c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2353     PORTE_PCR4 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E8引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_17  ;; 0x4004d010
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2354     PORTE_PCR5 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E9引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_18  ;; 0x4004d014
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2355     PORTE_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E10引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_19  ;; 0x4004d018
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2356     PORTE_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E11引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_20  ;; 0x4004d01c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2357     
// 2358         
// 2359         GPIOE_PDDR = 0xffffff00;  //E0~E7设置为输入口 
        LDR.W    R0,??DataTable23_21  ;; 0x400ff114
        MVNS     R1,#+255
        STR      R1,[R0, #+0]
// 2360 
// 2361         GPIOB_PDDR = 0xffefffff;  //PTC1设置为输入
        LDR.W    R0,??DataTable23_22  ;; 0x400ff054
        MVNS     R1,#+1048576
        STR      R1,[R0, #+0]
// 2362         
// 2363         
// 2364          PORTD_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D6引脚设置为GPIO模式   //拨码开关
        LDR.W    R0,??DataTable23_23  ;; 0x4004c018
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2365          PORTD_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D7引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_24  ;; 0x4004c01c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2366          PORTD_PCR8 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D8引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_25  ;; 0x4004c020
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2367          PORTD_PCR9 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D9引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_26  ;; 0x4004c024
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2368          PORTD_PCR10 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D10引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_27  ;; 0x4004c028
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2369          PORTD_PCR11 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D11引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_28  ;; 0x4004c02c
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2370          PORTD_PCR12 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D12引脚设置为GPIO模式
        LDR.W    R0,??DataTable23_29  ;; 0x4004c030
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2371          PORTD_PCR13 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D13引脚设置为GPIO模式 
        LDR.W    R0,??DataTable23_30  ;; 0x4004c034
        MOVW     R1,#+259
        STR      R1,[R0, #+0]
// 2372    
// 2373          GPIOD_PDDR = 0xffffc03f;       
        LDR.W    R0,??DataTable23_31  ;; 0x400ff0d4
        MVNS     R1,#+16320
        STR      R1,[R0, #+0]
// 2374 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20:
        DC32     lcd_line_num

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_1:
        DC32     `?<Constant "speed_select:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_2:
        DC32     `?<Constant "s_pit_count:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_3:
        DC32     stop_pit_count

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_4:
        DC32     `?<Constant "rampdetime:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_5:
        DC32     ramp_delay_time

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_6:
        DC32     `?<Constant "mid_angle:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_7:
        DC32     `?<Constant "WHITE_BLACK_OT:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_8:
        DC32     `?<Constant "test_run:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_9:
        DC32     test_run

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_10:
        DC32     `?<Constant "l_er_ser_p:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_11:
        DC32     `?<Constant "l_er_ser_d:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_12:
        DC32     `?<Constant "lcd_ref_p:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_13:
        DC32     `?<Constant "lcd_ref_d:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_14:
        DC32     `?<Constant "l_str_speed:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_15:
        DC32     `?<Constant "l_bow_speed:">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20_16:
        DC32     `?<Constant "lcd_strspe_ed:">`
// 2375 
// 2376 //---------------------------------------------------主函数------------------------------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 2377 void main(void)
// 2378 {
main:
        PUSH     {R3-R5,LR}
// 2379    uint16 i=0,j=0;
        MOVS     R4,#+0
        MOVS     R5,#+0
// 2380    lcd_debug = 1;
        LDR.W    R0,??DataTable23_32
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2381     DisableInterrupts;
        CPSID i         
// 2382     pllinit180M(); 
        BL       pllinit180M
// 2383     
// 2384     LCD_IO_Init();
        BL       LCD_IO_Init
// 2385     LCD_Init(); 
        BL       LCD_Init
// 2386     //单片机上电后，检测拨码和按键，检测完毕设置后相应的参数后，退出
// 2387 
// 2388     PORT_Init();              //端口初始化
        BL       PORT_Init
// 2389     hw_FTM_init();
        BL       hw_FTM_init
// 2390     UART0_Init();             //串口初始化   
        BL       UART0_Init
// 2391     LPTMR_Init();             //脉冲计数器初始化
        BL       LPTMR_Init
// 2392     EXIT_Init();   
        BL       EXIT_Init
        B.N      ??main_0
// 2393     while(lcd_debug)
// 2394     { 
// 2395       pre_show();
??main_1:
        BL       pre_show
// 2396       scan_boma();
        BL       scan_boma
// 2397       Keyscan();  //检测拨码的时间过长，所以检测拨码放在while之前，而且检测完后，将其关闭，让车子跑动
        BL       Keyscan
// 2398     } 
??main_0:
        LDR.W    R0,??DataTable23_32
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??main_1
// 2399     lcd_page_num = 0;
        LDR.W    R0,??DataTable23_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2400     Delay_MS(40000000);   //起跑延迟 uint8 ch=3;
        LDR.W    R0,??DataTable23_33  ;; 0x2625a00
        BL       Delay_MS
// 2401     Delay_MS(40000000);  
        LDR.W    R0,??DataTable23_33  ;; 0x2625a00
        BL       Delay_MS
// 2402     OddEvenStatus = ODD_EVEN_STATUS;
        LDR.W    R0,??DataTable23_34  ;; 0x400ff050
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+20
        ANDS     R0,R0,#0x1
        LDR.W    R1,??DataTable23_35
        STRB     R0,[R1, #+0]
// 2403     VIF = VIF_START;
        LDR.W    R0,??DataTable23_36
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2404     Initial();
        BL       Initial
// 2405     
// 2406     enable_irq(45);           //打开串口中断
        MOVS     R0,#+45
        BL       enable_irq
// 2407     enable_irq(88);           //打开行中断 
        MOVS     R0,#+88
        BL       enable_irq
// 2408     EnableInterrupts;
        CPSIE i         
// 2409 
// 2410     
// 2411     while(1)
// 2412     {      
// 2413        if(ImageReady)                                         //图像准备好，再决策
??main_2:
        LDR.W    R0,??DataTable23_37
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_2
// 2414        {   
// 2415          for(i = 0;i< ROW ; i++)
        MOVS     R4,#+0
        B.N      ??main_3
??main_4:
        ADDS     R4,R4,#+1
??main_3:
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        CMP      R4,#+65
        BCS.N    ??main_5
// 2416            for(j= 0;j<COLUMN;j++)
        MOVS     R5,#+0
        B.N      ??main_6
// 2417            {
// 2418              if(VideoImage2[i][j] > WHITE_BLACK_OT)
// 2419              {
// 2420                  VideoImage1[i][j] = WHITE;
// 2421              }
// 2422              else
// 2423              {
// 2424                 VideoImage1[i][j] =BLACK;
??main_7:
        UXTH     R5,R5            ;; ZeroExt  R5,R5,#+16,#+16
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable23_38
        MLA      R0,R0,R4,R1
        MOVS     R1,#+10
        STRB     R1,[R5, R0]
// 2425              }
??main_8:
        ADDS     R5,R5,#+1
??main_6:
        UXTH     R5,R5            ;; ZeroExt  R5,R5,#+16,#+16
        CMP      R5,#+159
        BCS.N    ??main_4
        LDR.W    R0,??DataTable23_4
        LDRB     R0,[R0, #+0]
        UXTH     R5,R5            ;; ZeroExt  R5,R5,#+16,#+16
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        MOVS     R1,#+159
        LDR.W    R2,??DataTable23_39
        MLA      R1,R1,R4,R2
        LDRB     R1,[R5, R1]
        CMP      R0,R1
        BCS.N    ??main_7
        UXTH     R5,R5            ;; ZeroExt  R5,R5,#+16,#+16
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        MOVS     R0,#+159
        LDR.W    R1,??DataTable23_38
        MLA      R0,R0,R4,R1
        MOVS     R1,#+255
        STRB     R1,[R5, R0]
        B.N      ??main_8
// 2426            }
// 2427          
// 2428          if(start_stop_en == 0 && start_stop_cs ==1)
??main_5:
        LDR.W    R0,??DataTable23_40
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable21
        LDRB     R1,[R1, #+0]
        EORS     R1,R1,#0x1
        ORRS     R0,R1,R0
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BNE.N    ??main_9
// 2429          {
// 2430            start_stop_count++;
        LDR.W    R0,??DataTable23_41
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable23_41
        STR      R0,[R1, #+0]
// 2431            if(start_stop_count > 50*stop_pit_count)  //延时6s检测起跑线300
        LDR.W    R0,??DataTable23_42
        LDR      R0,[R0, #+0]
        MOVS     R1,#+50
        MULS     R0,R1,R0
        LDR.W    R1,??DataTable23_41
        LDR      R1,[R1, #+0]
        CMP      R0,R1
        BCS.N    ??main_9
// 2432            {
// 2433              start_stop_en = 1;
        LDR.W    R0,??DataTable23_40
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2434              start_stop_count = stop_pit_count;//300
        LDR.W    R0,??DataTable23_41
        LDR.W    R1,??DataTable23_42
        LDR      R1,[R1, #+0]
        STR      R1,[R0, #+0]
// 2435            }
// 2436          }
// 2437          //用于测试赛道。
// 2438          if(test_run == 0)
??main_9:
        LDR.W    R0,??DataTable23_43
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??main_10
// 2439          {
// 2440            car_test_run++;
        LDR.W    R0,??DataTable23_44
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable23_44
        STRH     R0,[R1, #+0]
// 2441            if(car_test_run >= 200)
        LDR.W    R0,??DataTable23_44
        LDRH     R0,[R0, #+0]
        CMP      R0,#+200
        BCC.N    ??main_10
// 2442            {
// 2443              car_test_run = 200;
        LDR.W    R0,??DataTable23_44
        MOVS     R1,#+200
        STRH     R1,[R0, #+0]
// 2444              stopflag=1;
        LDR.W    R0,??DataTable23_45
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2445            }
// 2446          }
// 2447           Search_WhiteBase();
??main_10:
        BL       Search_WhiteBase
// 2448         if(find_whitebase_flag == 1)  //这个条件的加入，有两点好处。1 防止没有找到基准行的时候溢出， 2、当没有找到基准行的时候保持上一场的数据
        LDR.W    R0,??DataTable23_46
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_11
// 2449         {
// 2450           Search_BlackEdge();
        BL       Search_BlackEdge
// 2451           Deal_BlackEdge();
        BL       Deal_BlackEdge
// 2452           get_line_information();
        BL       get_line_information
// 2453           if(control_top_whiteline >=50 && control_top_whiteline <ROW-1)  //当车子处于弯道当中的时候，不进行线性相关的计算
        LDR.N    R0,??DataTable21_2
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+50
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+14
        BCS.N    ??main_12
// 2454           {
// 2455             linear_factor=get_linear_factor(bottom_whitebase,control_top_whiteline - 10,center_linear_average);
        LDR.W    R0,??DataTable23_47
        LDR      R2,[R0, #+0]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.N    R0,??DataTable21_2
        LDRH     R0,[R0, #+0]
        SUBS     R1,R0,#+10
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        LDR.W    R0,??DataTable23_48
        LDRB     R0,[R0, #+0]
        BL       get_linear_factor
        LDR.W    R1,??DataTable23_49
        STR      R0,[R1, #+0]
        B.N      ??main_11
// 2456           }
// 2457           else
// 2458           {
// 2459             linear_factor = 0.5;
??main_12:
        LDR.W    R0,??DataTable23_49
        MOVS     R1,#+1056964608
        STR      R1,[R0, #+0]
// 2460           }
// 2461         }
// 2462         if(start_stop_en == 1 )  //start_stop_en
??main_11:
        LDR.W    R0,??DataTable23_40
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_13
// 2463         {
// 2464           check_start_stop_line();
        BL       check_start_stop_line
// 2465         }
// 2466           Control();
??main_13:
        BL       Control
// 2467           redraw();//刷新显示屏   
        BL       redraw
// 2468           SCI0_send_mesage(); 
        BL       SCI0_send_mesage
// 2469           while(ImageReady);
??main_14:
        LDR.W    R0,??DataTable23_37
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??main_14
        B.N      ??main_2
// 2470        }
// 2471     }  
// 2472    
// 2473 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21:
        DC32     start_stop_cs

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_1:
        DC32     top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_2:
        DC32     control_top_whiteline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_3:
        DC32     deal_start_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_4:
        DC32     white_refer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_5:
        DC32     center_average
// 2474   
// 2475 
// 2476 //-----------------------------中断函数-----------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 2477 void uart0_isr(void)          //串口中断
// 2478 {    
// 2479     DisableInterrupts;   // 关总中断也可以，但在有更高级中断存在里不推荐
uart0_isr:
        CPSID i         
// 2480       uint8 ch;
// 2481      while(!(UART0_S1&UART_S1_RDRF_MASK));
??uart0_isr_0:
        LDR.W    R0,??DataTable23_50  ;; 0x4006a004
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+26
        BPL.N    ??uart0_isr_0
// 2482       ch = UART0_D;
        LDR.W    R0,??DataTable23_51  ;; 0x4006a007
        LDRB     R0,[R0, #+0]
// 2483       if(ch == '1')     //发送的是原始图像
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+49
        BNE.N    ??uart0_isr_1
// 2484         send_mes = 1; 
        LDR.W    R0,??DataTable23_52
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 2485       else if(ch == '2')  //发送图像的变化趋势
??uart0_isr_1:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+50
        BNE.N    ??uart0_isr_3
// 2486         send_mes = 2;
        LDR.W    R0,??DataTable23_52
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 2487       else if(ch == '3')  //速度图像
??uart0_isr_3:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+51
        BNE.N    ??uart0_isr_4
// 2488         send_mes = 3;
        LDR.W    R0,??DataTable23_52
        MOVS     R1,#+3
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 2489       else if (ch >= 64 && ch <= 65)   //变档调速
??uart0_isr_4:
        SUBS     R1,R0,#+64
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+2
        BCS.N    ??uart0_isr_2
// 2490       {   
// 2491           switch(ch - 64)  //变档调速
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R0,R0,#+64
        CMP      R0,#+0
        BEQ.N    ??uart0_isr_5
        CMP      R0,#+1
        BEQ.N    ??uart0_isr_6
        B.N      ??uart0_isr_7
// 2492           {
// 2493           case 0:   send_mes='s';break;//stop停车
??uart0_isr_5:
        LDR.W    R0,??DataTable23_52
        MOVS     R1,#+115
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 2494           case 1:   send_mes='p';break;
??uart0_isr_6:
        LDR.W    R0,??DataTable23_52
        MOVS     R1,#+112
        STRB     R1,[R0, #+0]
        B.N      ??uart0_isr_2
// 2495           default: speed = 100;
??uart0_isr_7:
        LDR.W    R0,??DataTable23_53
        MOVS     R1,#+100
        STRH     R1,[R0, #+0]
// 2496           }
// 2497       }
// 2498 
// 2499     EnableInterrupts;
??uart0_isr_2:
        CPSIE i         
// 2500 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22:
        DC32     p_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_1:
        DC32     speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_2:
        DC32     S_straight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_3:
        DC32     S_left

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_4:
        DC32     S_right

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_5:
        DC32     direction

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_6:
        DC32     ramp_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_7:
        DC32     0xc3500

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_8:
        DC32     0x400ff080

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_9:
        DC32     0x400ff090

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_10:
        DC32     sub_NUM

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_11:
        DC32     se_sub_NUM

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_12:
        DC32     up_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_13:
        DC32     change_page

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable22_14:
        DC32     add_NUM
// 2501 
// 2502 //------------------------------------图像采集中断----------------------------------------//

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 2503 void PTB_isr(void)//对于场中断20ms和行中断63us都是摄像头的固有的频率，不需要用软件去定时
// 2504 {
PTB_isr:
        PUSH     {R4,LR}
// 2505   /*
// 2506         图像采集的行数为 27 30 33 36 39 42 45 48 51 54 57 60 63 
// 2507                          66 69 72 75 78 81 84 87 90 93 96 99 102 
// 2508                          105 108 111 114 117 120 123 126 129 132 135 138 141
// 2509                          144 147 150 153 156 159 162 165 168 171 174 177 180 
// 2510                          183 186 189 192 195 198 201 204 207 210 213 216 219 
// 2511                  
// 2512         */ 
// 2513     uint16 i;  
// 2514     
// 2515   
// 2516    PORTB_PCR22|=PORT_PCR_ISF_MASK;  //清除中断标志位
        LDR.W    R0,??DataTable23_54  ;; 0x4004a058
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable23_54  ;; 0x4004a058
        STR      R0,[R1, #+0]
// 2517     if (VIF == VIF_START)                              //开始采样标志
        LDR.N    R0,??DataTable23_36
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??PTB_isr_0
// 2518       {
// 2519         LineCount++;
        LDR.W    R0,??DataTable23_55
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable23_55
        STRH     R0,[R1, #+0]
// 2520         if(OddEvenStatus != ODD_EVEN_STATUS)
        LDR.N    R0,??DataTable23_35
        LDRB     R0,[R0, #+0]
        LDR.N    R1,??DataTable23_34  ;; 0x400ff050
        LDR      R1,[R1, #+0]
        LSRS     R1,R1,#+20
        ANDS     R1,R1,#0x1
        CMP      R0,R1
        BEQ.N    ??PTB_isr_1
// 2521         {
// 2522           OddEvenStatus = ODD_EVEN_STATUS;	//奇偶场标志
        LDR.N    R0,??DataTable23_34  ;; 0x400ff050
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+20
        ANDS     R0,R0,#0x1
        LDR.N    R1,??DataTable23_35
        STRB     R0,[R1, #+0]
// 2523           VIF = VIF_WAITSAMPLE;   		//下一个状态为等待采样
        LDR.N    R0,??DataTable23_36
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2524           VideoImageLine = 0;
        LDR.N    R0,??DataTable23_56
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2525           LineCount = 0;
        LDR.N    R0,??DataTable23_55
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 2526           ImageReady = 0; 
        LDR.N    R0,??DataTable23_37
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        B.N      ??PTB_isr_1
// 2527         }
// 2528       }
// 2529     else if (VIF == VIF_WAITSAMPLE)                 //等待采样,此时略去VIDEO_START_LINE行
??PTB_isr_0:
        LDR.N    R0,??DataTable23_36
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??PTB_isr_2
// 2530       {
// 2531           LineCount++;
        LDR.N    R0,??DataTable23_55
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable23_55
        STRH     R0,[R1, #+0]
// 2532           if (LineCount >= VIDEO_START_LINE)
        LDR.N    R0,??DataTable23_55
        LDRH     R0,[R0, #+0]
        CMP      R0,#+27
        BCC.N    ??PTB_isr_1
// 2533           {
// 2534               VIF = VIF_SAMPLELINE;                 //下一个状态为采样状态
        LDR.N    R0,??DataTable23_36
        MOVS     R1,#+2
        STRB     R1,[R0, #+0]
        B.N      ??PTB_isr_1
// 2535           }   	
// 2536       }
// 2537     else if (VIF == VIF_SAMPLELINE)              //开始采样
??PTB_isr_2:
        LDR.N    R0,??DataTable23_36
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BNE.N    ??PTB_isr_1
// 2538       {
// 2539           LineCount++;
        LDR.N    R0,??DataTable23_55
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable23_55
        STRH     R0,[R1, #+0]
// 2540           if (LineCount % 3== 0)                //每隔一行采一行
        LDR.N    R0,??DataTable23_55
        LDRH     R0,[R0, #+0]
        MOVS     R1,#+3
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        CMP      R0,#+0
        BNE.N    ??PTB_isr_3
// 2541           {
// 2542               for (i = 0; i < COLUMN+PIANYI ; i++)        //每行扫描COLUMN+PIANYI个点(其中PIANYI个点需要被剔除掉，因为是行消隐点)
        MOVS     R4,#+0
        B.N      ??PTB_isr_4
// 2543              {
// 2544                   if (i >=PIANYI )
??PTB_isr_5:
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        CMP      R4,#+150
        BCC.N    ??PTB_isr_6
// 2545                    {//采集的第一个点的坐标在真实的世界里是右下角，所以在数组中存储在第一行的最后一个位置
// 2546                      VideoImage2[VideoImageLine][i-PIANYI] = (uint8)(0x000000ff & GPIOE_PDIR);//将采集到的点直接放入到VideoImage2[][]中在init array（）中放到VideoImage1[][]中做处理
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        LDR.N    R0,??DataTable23_56
        LDRB     R0,[R0, #+0]
        MOVS     R1,#+159
        LDR.N    R2,??DataTable23_39
        MLA      R0,R1,R0,R2
        ADDS     R0,R4,R0
        LDR.N    R1,??DataTable23_57  ;; 0x400ff110
        LDR      R1,[R1, #+0]
        STRB     R1,[R0, #-150]
// 2547                          Delay_MS(3); 
        MOVS     R0,#+3
        BL       Delay_MS
// 2548                         asm("nop");
        nop              
// 2549                         asm("nop");//汇编延时
        nop              
// 2550                   }
// 2551               }
??PTB_isr_6:
        ADDS     R4,R4,#+1
??PTB_isr_4:
        MOVW     R0,#+309
        UXTH     R4,R4            ;; ZeroExt  R4,R4,#+16,#+16
        CMP      R4,R0
        BCC.N    ??PTB_isr_5
// 2552              VideoImageLine++;
        LDR.N    R0,??DataTable23_56
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable23_56
        STRB     R0,[R1, #+0]
// 2553           }
// 2554           if (VideoImageLine == ROW)      //采集行数大于设定的行数
??PTB_isr_3:
        LDR.N    R0,??DataTable23_56
        LDRB     R0,[R0, #+0]
        CMP      R0,#+65
        BNE.N    ??PTB_isr_1
// 2555           {
// 2556               ImageReady = 1;           //图像准备好
        LDR.N    R0,??DataTable23_37
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2557               VIF = VIF_START;
        LDR.N    R0,??DataTable23_36
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 2558           }
// 2559           
// 2560      }
// 2561     
// 2562   /* if (start_stop_en = 1 && start_stop_cs ==1 && (delay_detective && ((left_tube1 || left_tube2) && (right_tube1 || right_tube2))))        //每行中断检测一次起跑线
// 2563     {
// 2564       stopflag = 1;
// 2565     }
// 2566     //加入光电管子对起跑线的检测。
// 2567     */
// 2568     
// 2569     if (LineCount % 45 == 42)         //   7次  每隔45行控制一次  第一次控制位 43 88 133 178 223 268 313    
??PTB_isr_1:
        LDR.N    R0,??DataTable23_55
        LDRH     R0,[R0, #+0]
        MOVS     R1,#+45
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        CMP      R0,#+42
        BNE.W    ??PTB_isr_7
// 2570     {
// 2571         speed_feedback = LPTMR0_CNR;                  //读编码器的值
        LDR.N    R0,??DataTable23_58
        LDR.N    R1,??DataTable23_59  ;; 0x4004000c
        LDR      R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 2572         LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;
        LDR.N    R0,??DataTable23_60  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.N    R1,??DataTable23_60  ;; 0x40040000
        STR      R0,[R1, #+0]
// 2573         LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;                //溢出后继续计数
        LDR.N    R0,??DataTable23_60  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.N    R1,??DataTable23_60  ;; 0x40040000
        STR      R0,[R1, #+0]
// 2574         
// 2575        if(stopflag == 1 && speed_down_cnt>10) 
        LDR.N    R0,??DataTable23_45
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??PTB_isr_8
        LDR.N    R0,??DataTable23_61
        LDRH     R0,[R0, #+0]
        CMP      R0,#+11
        BCC.N    ??PTB_isr_8
// 2576         {
// 2577             if( speed_feedback >= 18 )
        LDR.N    R0,??DataTable23_58
        LDRH     R0,[R0, #+0]
        CMP      R0,#+18
        BCC.N    ??PTB_isr_9
// 2578               {
// 2579                  FTM1_C0V=600;  //正转
        LDR.N    R0,??DataTable23_62  ;; 0x40039010
        MOV      R1,#+600
        STR      R1,[R0, #+0]
// 2580                 FTM1_C1V=0;   //反转
        LDR.N    R0,??DataTable23_63  ;; 0x40039018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??PTB_isr_10
// 2581               }
// 2582             else if(speed_feedback <18)
??PTB_isr_9:
        LDR.N    R0,??DataTable23_58
        LDRH     R0,[R0, #+0]
        CMP      R0,#+18
        BCS.N    ??PTB_isr_10
// 2583               {
// 2584                 FTM1_C0V=0;  //正转
        LDR.N    R0,??DataTable23_62  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 2585                 FTM1_C1V=0;   //反转
        LDR.N    R0,??DataTable23_63  ;; 0x40039018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 2586                 dead_stop = 1;
        LDR.N    R0,??DataTable23_64
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
// 2587               }
// 2588             
// 2589              if(dead_stop == 1)
??PTB_isr_10:
        LDR.N    R0,??DataTable23_64
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??PTB_isr_7
// 2590              {
// 2591                 FTM1_C0V=0;  //正转
        LDR.N    R0,??DataTable23_62  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 2592                 FTM1_C1V=0;   //反转
        LDR.N    R0,??DataTable23_63  ;; 0x40039018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??PTB_isr_7
// 2593              }
// 2594         }
// 2595         else
// 2596         {
// 2597          if(ramp_flag == 1)
??PTB_isr_8:
        LDR.N    R0,??DataTable23_65
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??PTB_isr_11
// 2598            speed_except = ramp_speed;//26
        LDR.N    R0,??DataTable23_66
        LDR.N    R1,??DataTable23_67
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 2599          
// 2600           speed_error = speed_except - speed_feedback;
??PTB_isr_11:
        LDR.N    R0,??DataTable23_66
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable23_58
        LDRSH    R1,[R1, #+0]
        SUBS     R0,R0,R1
        LDR.N    R1,??DataTable23_68
        STRH     R0,[R1, #+0]
// 2601           speed +=(int16)((speed_p * (speed_error - speed_re_error) + speed_i * speed_error)/10 );//- speed_ed
        LDR.N    R0,??DataTable23_53
        LDRH     R0,[R0, #+0]
        LDR.N    R1,??DataTable23_69
        LDRB     R1,[R1, #+0]
        LDR.N    R2,??DataTable23_68
        LDRSH    R2,[R2, #+0]
        LDR.N    R3,??DataTable23_70
        LDRSH    R3,[R3, #+0]
        SUBS     R2,R2,R3
        LDR.N    R3,??DataTable23_71
        LDRB     R3,[R3, #+0]
        LDR.N    R4,??DataTable23_68
        LDRSH    R4,[R4, #+0]
        MULS     R3,R4,R3
        MLA      R1,R2,R1,R3
        MOVS     R2,#+10
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        LDR.N    R1,??DataTable23_53
        STRH     R0,[R1, #+0]
// 2602          //speed=1000;
// 2603           if(speed > max_speed)
        LDR.N    R0,??DataTable23_72
        LDRH     R0,[R0, #+0]
        LDR.N    R1,??DataTable23_53
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??PTB_isr_12
// 2604              speed = max_speed;
        LDR.N    R0,??DataTable23_53
        LDR.N    R1,??DataTable23_72
        LDRSH    R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 2605           if(speed < min_speed)
??PTB_isr_12:
        LDR.N    R0,??DataTable23_53
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable23_73
        LDRH     R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??PTB_isr_13
// 2606              speed = min_speed;
        LDR.N    R0,??DataTable23_53
        LDR.N    R1,??DataTable23_73
        LDRSH    R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 2607           
// 2608           if( speed_error > 15)  //测试发现当反转较大，如700时，在减速的时候摄像头会出现拉黑的危险
??PTB_isr_13:
        LDR.N    R0,??DataTable23_68
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+16
        BLT.N    ??PTB_isr_14
// 2609           {
// 2610           //  FTM1_C0V = 800;
// 2611            // FTM1_C1V = 0;
// 2612           FTM1_C0V = 0;
        LDR.N    R0,??DataTable23_62  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 2613           FTM1_C1V = 780;
        LDR.N    R0,??DataTable23_63  ;; 0x40039018
        MOV      R1,#+780
        STR      R1,[R0, #+0]
        B.N      ??PTB_isr_15
// 2614           }
// 2615          else if(speed_error < -13)  //用于反转控制
??PTB_isr_14:
        LDR.N    R0,??DataTable23_68
        LDRSH    R0,[R0, #+0]
        MVNS     R1,#+12
        CMP      R0,R1
        BGE.N    ??PTB_isr_16
// 2616           {
// 2617             //FTM1_C0V=0;  //正转
// 2618            // FTM1_C1V=650;   //反转750
// 2619             FTM1_C0V=950;  //正转
        LDR.N    R0,??DataTable23_62  ;; 0x40039010
        MOVW     R1,#+950
        STR      R1,[R0, #+0]
// 2620            FTM1_C1V=0;   //反转750
        LDR.N    R0,??DataTable23_63  ;; 0x40039018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??PTB_isr_15
// 2621           }
// 2622           else
// 2623           {
// 2624           //  FTM1_C0V=speed;
// 2625            // FTM1_C1V = 0;
// 2626             FTM1_C0V=0;
??PTB_isr_16:
        LDR.N    R0,??DataTable23_62  ;; 0x40039010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
// 2627             FTM1_C1V = speed;
        LDR.N    R0,??DataTable23_63  ;; 0x40039018
        LDR.N    R1,??DataTable23_53
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
// 2628             
// 2629           }
// 2630           speed_re_error=speed_error;
??PTB_isr_15:
        LDR.N    R0,??DataTable23_70
        LDR.N    R1,??DataTable23_68
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 2631         }
// 2632 
// 2633     }
// 2634 
// 2635 
// 2636 }
??PTB_isr_7:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23:
        DC32     down_line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_1:
        DC32     lcd_page_num

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_2:
        DC32     speed_select

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_3:
        DC32     mid_angle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_4:
        DC32     WHITE_BLACK_OT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_5:
        DC32     lcd_error_servo_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_6:
        DC32     lcd_error_servo_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_7:
        DC32     lcd_ref_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_8:
        DC32     lcd_ref_d

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_9:
        DC32     lcd_straight_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_10:
        DC32     lcd_bow_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_11:
        DC32     lcd_straight_speed_ed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_12:
        DC32     0x4004a050

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_13:
        DC32     0x4004d000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_14:
        DC32     0x4004d004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_15:
        DC32     0x4004d008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_16:
        DC32     0x4004d00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_17:
        DC32     0x4004d010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_18:
        DC32     0x4004d014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_19:
        DC32     0x4004d018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_20:
        DC32     0x4004d01c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_21:
        DC32     0x400ff114

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_22:
        DC32     0x400ff054

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_23:
        DC32     0x4004c018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_24:
        DC32     0x4004c01c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_25:
        DC32     0x4004c020

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_26:
        DC32     0x4004c024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_27:
        DC32     0x4004c028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_28:
        DC32     0x4004c02c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_29:
        DC32     0x4004c030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_30:
        DC32     0x4004c034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_31:
        DC32     0x400ff0d4

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_32:
        DC32     lcd_debug

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_33:
        DC32     0x2625a00

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_34:
        DC32     0x400ff050

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_35:
        DC32     OddEvenStatus

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_36:
        DC32     Videoclo_Flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_37:
        DC32     ImageReady

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_38:
        DC32     VideoImage1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_39:
        DC32     VideoImage2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_40:
        DC32     start_stop_en

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_41:
        DC32     start_stop_count

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_42:
        DC32     stop_pit_count

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_43:
        DC32     test_run

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_44:
        DC32     car_test_run

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_45:
        DC32     stopflag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_46:
        DC32     find_whitebase_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_47:
        DC32     center_linear_average

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_48:
        DC32     bottom_whitebase

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_49:
        DC32     linear_factor

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_50:
        DC32     0x4006a004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_51:
        DC32     0x4006a007

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_52:
        DC32     send_mes

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_53:
        DC32     speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_54:
        DC32     0x4004a058

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_55:
        DC32     LineCount

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_56:
        DC32     VideoImageLine

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_57:
        DC32     0x400ff110

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_58:
        DC32     speed_feedback

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_59:
        DC32     0x4004000c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_60:
        DC32     0x40040000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_61:
        DC32     speed_down_cnt

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_62:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_63:
        DC32     0x40039018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_64:
        DC32     dead_stop

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_65:
        DC32     ramp_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_66:
        DC32     speed_except

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_67:
        DC32     ramp_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_68:
        DC32     speed_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_69:
        DC32     speed_p

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_70:
        DC32     speed_re_error

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_71:
        DC32     speed_i

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_72:
        DC32     max_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_73:
        DC32     min_speed

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "lcd_debug:">`:
        DATA
        DC8 "lcd_debug:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "speed_select:">`:
        DATA
        DC8 "speed_select:"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "s_pit_count:">`:
        DATA
        DC8 "s_pit_count:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "rampdetime:">`:
        DATA
        DC8 "rampdetime:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "mid_angle:">`:
        DATA
        DC8 "mid_angle:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "WHITE_BLACK_OT:">`:
        DATA
        DC8 "WHITE_BLACK_OT:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "test_run:">`:
        DATA
        DC8 "test_run:"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "l_er_ser_p:">`:
        DATA
        DC8 "l_er_ser_p:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "l_er_ser_d:">`:
        DATA
        DC8 "l_er_ser_d:"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "lcd_ref_p:">`:
        DATA
        DC8 "lcd_ref_p:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "lcd_ref_d:">`:
        DATA
        DC8 "lcd_ref_d:"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "l_str_speed:">`:
        DATA
        DC8 "l_str_speed:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "l_bow_speed:">`:
        DATA
        DC8 "l_bow_speed:"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "lcd_strspe_ed:">`:
        DATA
        DC8 "lcd_strspe_ed:"
        DC8 0

        END
// 2637  
// 
// 21 064 bytes in section .bss
//    154 bytes in section .data
//    192 bytes in section .rodata
// 18 712 bytes in section .text
// 
// 18 712 bytes of CODE  memory
//    192 bytes of CONST memory
// 21 218 bytes of DATA  memory
//
//Errors: none
//Warnings: none
