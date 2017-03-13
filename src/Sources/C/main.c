/*程序说明  */
//使程序更加的精简高效准确，
#include "MK60N512VMD100.h " /* include peripheral declarations */
#include "includes.h"
#include <math.h>
#include"LCDDriver.h"

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1
#define BUS_CLOCK  100  //(MHZ)50 82 90 100 105 110 115//这里设置的内核时钟等于总线时钟100M
#define BAUD 19200     //波特率
#define CORE_CLOCK 180

//--------------------------采集图像的相关变量-------------------------------------//
bool     OddEvenStatus;		  //奇偶场状态标志
#define  OddStatus  0
#define  EvenStatus 1
#define  ODD_EVEN_STATUS  (bool)(0x00000001 & (GPIOB_PDIR  >> 20))  //奇偶变换标志  将第ptc端口的第1位右移动后，置1
#define VIF_START	0   	 //	开始模式				 
#define VIF_WAITSAMPLE	1        //   等待模式
#define VIF_SAMPLELINE	2         //   除去消隐行的状态
#define VIF Videoclo_Flag         //
#define PIANYI 150   //实际采集列数为COLUMN + PIANYI，PIANYI为每行消隐点  中心值大于中值，消隐行减少
#define VIDEO_START_LINE  27	//图像采集起始行
bool ImageReady;               //图像准备好标志
uint8 Videoclo_Flag, VideoImageLine;   //采集状态标志位，行中断实际采集行数计数器
uint16 LineCount;                       //行中断采集行数计数器   这个数据一定是要定义为uint16  自己以前定义为uint8  伤心痛苦折磨了好几天


//-------------------------处理图像的相关变量-----------------------------------------//
#define ROW 65	                 //采集行数
#define COLUMN	159 		//每行点数
#define MID  79                 //列中心 
uint8 VideoImage1[ROW][COLUMN] =       //原始图像数组[0][0]在左下角
{
   0
};
uint8 VideoImage2[ROW][COLUMN] =       //原始图像数组[0][0]在左下角
{
   0
};

uint8 left_black[ROW]=                 //左边沿线的采集数组
{
  0
};
uint8 right_black[ROW]=                //右边沿线的采集数组
{
  0
};
uint8 center_white[ROW]=              //（虚拟出来的）中线的数组
{
  0
};


//-------------------------------------搜两边黑线----------------------------------//
#define MIN_WHITEBASE_POINT 30                    //最少连续白点个数成为基准的要求
#define WHITE_TOP_WHITELINE_POINT 20                  //两边的黑线的宽度小于这个值，判定为最高有效
#define CENTER_LOST_POINT 20
uint8 current_deal_line=0;     //当前处理的行
uint8 deal_start_line = 0;                //这个 值时控制处理的起始行一般定义为基准行 + 4
uint8 hang_search_start = 0;             //定义每次扫描的开始是从哪个点开始的
uint8 whitepoint_start=0;                //从左至右白点开始处
uint8 whitepoint_end=0;                 //从左至右白点结束处
uint8 whitebase_searchstart = MID;
uint8 left_whitebase_searchstart = 0;
uint8 right_whitebase_searchstart = 0; 
uint8 re_whitepoint_start = 20;  // 发车的时候车子一定要在赛道的中心左右，否则会出现找不到赛道的危险
uint8 re_whitepoint_end=145;   
uint8 center_lost_hang = 0;
uint8 refer_road_width[ROW] ={127,126,125,124,123,122,120,119,118,117,
                              116,115,114,113,112,110,108,106,104,112,
                              100,98,97,95,93,92,90,89,88,87,
                              86,85,83,81,80,79,77,75,73,70,
                              69,68,66,64,62,60,58,56,53,51,
                              49,47,45,43,41,39,38,36,35,33,
                               32,31,30,28,26};//
uint8 OT=36;                                     //判定为灰度值的跳变沿的最小灰度的跳变值
uint8 BASE_OT = 130;
uint8 WHITE_BLACK_OT = 145;           //进行二值化的分界值
#define WHITE 255
#define BLACK 10
uint8 top_whiteline = ROW-1;                          //图像的最顶行
uint8 left_top_whiteline = ROW-1;
uint8 right_top_whiteline = ROW-1;

uint8 bottom_whitebase = 0;                       //图像的基准行 
bool find_whitebase_flag = 0;  //基准行的标志位

uint8 re_white_refer = MID;  //这个点作为每场搜索基准行的开始的点  ，最开始的时候定义为 默认为MID
uint8 white_refer = 0;                            //基准行上的赛道的中点
uint8 Row_state[ROW] =
{
  0
};
//--------------------------------------赛道处理的相关参数-----------------------//
uint8 S_right = 0;//向右拐的计数
uint8 S_left =0 ; //向左拐计数
uint8 S_straight = 0;
uint8 direction = 0; //4是初始化的值
uint8 re_direction = 0;//记录上一次的当有的时候，无法判断出赛道的类型的时候，用上一次的状态
uint32 center_average = 0;
uint16 center_error_average = 0;  
uint32 center_linear_average = 0;

#define RAMP_WIDTH  90                  //
uint16 ramp_delay_time = 25;
uint16 ramp_time = 0;                //进入坡道后多长时间重新开启起跑线检测
uint16 ramp_dis_time = 0;       //防止下坡的误判而延时
uint16 ramp_speed = 80;                    //坡道减速值70
bool ramp_flag = 0;                          //进入坡道标志,主要用于控制
bool ramp_dis_flag = 0;                     //主要是防止下坡误判
 
 /*测试的时候，监测函数中局部变量
float XX_square_sum=0;   //X轴平方和
float YY_square_sum=0;   //Y轴平方和
float XYmulti_sum=0;      //XY乘积之和*/
float linear_factor = 0;

//-----串口功能选择----//
uint8 send_mes=0;              //根据上位机发送来的数据来选择不同的串口功能

//------------------------------------电机控制函数的参数-----------------------------------//
#define SPEEDCHOICENUM 6  //定义6当
typedef struct Tag_SpeedSwitch{
  uint16 Cstraightspeed;
  uint16 Cbowspeed;
  uint16 Cstraightspeed_ed;
}SpeedSwitch;

SpeedSwitch mySpeedswitch[SPEEDCHOICENUM]={
  {140,136,38},    //
  {135,132,34},
  {130,126,28},
  {125,115,25},
  {115,108,15},
  {110,90,10}  
};

uint16 speed_feedback = 0;               //编码器的返回值                  //
int16 speed_re_error = 0;
bool stopflag = 0;//速度反馈
uint16 speed_down_cnt = 0;       //检测到起跑线后延时十场，然后减速
int16 speed_error = 0;
uint8 speed_p = 80;//44
uint8 speed_i =95;//65
int16 speed = 0;

uint16 lcd_straight_speed = 0;  //方便与lcd调节速度的最大和最小值
uint16 lcd_bow_speed = 0;//
uint16 lcd_straight_speed_ed = 0;

uint16 straight_speed = 115;
uint16 bow_speed = 100;
uint16 straight_speed_ed = 8;
uint16 max_speed=900;
uint16 min_speed=20;

uint16 straight_count = 0;

bool dead_stop = 0; 

uint16 speed_except=0;
uint8 re_top_whiteline=0;
uint8 speed_select = 0;//当为1的时候选择低速跑，用于测试跑道和算法 ，通过拨码控制
uint8 full_speed_line = 0;
//-----------------------------------舵机控制函数的变量---------------------------------//

int16 angle=1460;
int16 re_angle= 1460;
uint16 mid_angle=1460; //  推着向右拐，说明小于摆正值     当车子跑的稳定于左边时，减去一个值可恢复到中间

uint16 control_top_whiteline = 0;//re_control_top_whiteline top_error_servo_p
uint16 re_control_top_whiteline = ROW - 1;
uint16 danger_count = 0;  //记录危险的点数
bool danger_flag = 1;  //这个的初始值为1.当出了控制最高行了则置为0；
int16 p_error=0;
int16 p_re_error = 0;
int16 ref_his_error[5] ={0,0,0,0,0};//用这个数组来记录历史的值，然后参与当前的error

//对这两个pd的说明，第一个pd起到的作用是粗调，第二个pd是微调，当发现调节第二个pd,没有很大的改善的时候再且一定去动第一个pd

uint16 error_servo_p=0;
int16 lcd_error_servo_p = 3; //4  2   4
uint16 error_servo_d=0;
int16 lcd_error_servo_d = 57;  //83  40  65
uint16 top_error_servo_p = 0;

uint16 error_servo_ref_p = 0;
int16 lcd_ref_p = 6;  //9      7和25使得直线很直   12  6          6
uint16 error_servo_ref_d = 0;
int16 lcd_ref_d = 40;  // 25                         60   23

uint8 get_p_errorline = 0;
 
int16 refer_error =0;
int16 re_refer_error = 0;

//起跑加载检测延时
/*#define right_tube1   (bool)(GPIOB_PDIR >> 4 & 0x00000001)             //分别读取红外管的状态
#define right_tube2   (bool)((GPIOB_PDIR >> 5) & 0x00000001)
#define left_tube1   (bool)((GPIOB_PDIR >> 6) & 0x00000001)
#define left_tube2   (bool)((GPIOB_PDIR >> 7) & 0x00000001)*/
uint32 start_stop_count = 0;  //起跑线检测计数
uint32 stop_pit_count = 6;
bool start_stop_en = 0;   //起跑线检测使能
bool start_stop_cs =0;   //起跑线检测的片选信号   当为1的时候选中检测起跑线

uint16 car_test_run = 0; 
bool test_run = 1;  
//-------------------------------按键的定义-------------------------------------------//
#define LCD_ROW 7                      //小液晶屏的实际的行数为8行
bool change_page=0;             //翻页
bool se_sub_NUM=0;             //翻页
bool up_line=0;              //换行   向上
bool down_line=0;            //换行   向下
bool add_NUM=0;              //更改数值  加
bool sub_NUM=0;              //更改数值  减 

//--------------------------------拨码的定义参数--------------------------------------//
bool lcd_debug = 1;            //lcd的调试选择
bool redraw_control=0;         //刷屏的控制位

//------------------------------------LCD变量声明-------------------------------------//
void pre_show(void);        //第一面的预显示
void redraw(void);          //刷频幕
void Keyscan(void);          //扫描拨码
void LCD_change_value(unsigned char page,unsigned char m,int i);//更改数值
void Delay_MS(uint32 ms);       //延时函数
uint8 lcd_page_num=1;        //液晶屏的页数
uint8 lcd_line_num=0;        //液晶屏的行数


//---------------------------数组初始化--------------------------//
void Initial(void)
{
  int16 i;
     for(i = 0;i < ROW;i++)
       {
         left_black[i] = 0;
         right_black[i] = 0;
         center_white[i] = 0;
         Row_state[i] = 3; //3代表的是两边都没有出现丢点
       }
       start_stop_count = 0;
       ramp_dis_flag = 0;
       ramp_flag = 0;
}

//--------------------低功耗脉冲计数器初始化-----------------------//
void LPTMR_Init()   //PTC5  LPT0_ALT2
{
   SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //开启C端口时钟
   PORTC_PCR5 &= ~PORT_PCR_MUX_MASK;
   PORTC_PCR5 |= PORT_PCR_MUX(4);  //PTC5配置为LPTMR模式
   PORTC_PCR5 |= PORT_PCR_PE_MASK; //
   PORTC_PCR5 &= ~PORT_PCR_PS_MASK; //下拉

   SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;  //使能LPTM模块时钟
   LPTMR0_CSR &= ~LPTMR_CSR_TPS_MASK;
   LPTMR0_CSR |= LPTMR_CSR_TPS(2)| LPTMR_CSR_TMS_MASK; //  ALT2  计数模式
   LPTMR0_CSR |= LPTMR_CSR_TFC_MASK;  //溢出复位 65535
   LPTMR0_CSR &= ~LPTMR_CSR_TPP_MASK;  //上升沿计数

   LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK; //  忽略分频和滤波
   LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;  //开启LPT模块
}


//---------------------------行中断捕捉端口初始化-------------------//
void EXIT_Init(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  //开启C端口时钟
    PORTC_PCR3 =PORT_PCR_MUX(1);  //GPIO
    GPIOC_PDDR &= ~GPIO_PIN(3);   //输入
    PORTC_PCR3 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; //上拉电阻;
    PORTC_PCR3 |= PORT_PCR_IRQC(9); //9为上升沿触发外部中断 10为下降沿触
}



//----------------------------串口初始化-----------------------------//
void UART0_Init(void)    //PTB16 RXD    PTB17 TXD
{
    uint32 uartclk_khz = CORE_CLOCK*10 * BUS_CLOCK;//时钟180MHz    //随时更改
    uint32 baud = BAUD;
    uint16 sbr,brfa;
    
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //开启B口时钟
    PORTB_PCR16|=PORT_PCR_MUX(3);//将PTB16引脚设置为模式3，即UART0_RX
    PORTB_PCR17|=PORT_PCR_MUX(3);//将PTB177引脚设置为模式3，即UART0_TX
    SIM_SCGC4|=SIM_SCGC4_UART0_MASK;//开启UART0时钟
    sbr = (uint16)((uartclk_khz*1000)/(baud*16));//计算并设置波特率
    
    UART0_BDH = (uint8)((sbr&0x1F00)>>8);//将波特率19200写入相应的寄存器然后进行使能，使其工作。前面的buad只是一个数字，而后面的计算是将19200写入这个寄存器，然后进行使能
    UART0_BDL=(uint8)(sbr&0x00FF);
    brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
    UART0_C4 = (uint8)(brfa & 0x001F);
    UART0_C2 |=(UART_C2_TE_MASK|UART_C2_RE_MASK);
    UART0_C1 = 0;	
    UART0_C2 |= UART_C2_RIE_MASK;   //开UART0接收中断
}


//-------------------------------------ftm初始化-----------------------------------------//
void hw_FTM_init(void)
{      	
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;//开启C端口时钟
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //开启A端口时钟
  
    
    PORTC_PCR4 &= ~PORT_PCR_MUX_MASK; //清零
    PORTA_PCR12 &= ~PORT_PCR_MUX_MASK;
    PORTA_PCR13 &= ~PORT_PCR_MUX_MASK;
    PORTA_PCR10 &= ~PORT_PCR_MUX_MASK;
    PORTC_PCR3 &= ~PORT_PCR_MUX_MASK;
    
    PORTC_PCR4 = PORT_PCR_MUX(4); //FTM is alt4 function for this pin
    PORTA_PCR10 = PORT_PCR_MUX(3);
    PORTA_PCR12 = PORT_PCR_MUX(3);//FTM is alt3 function for this pin 
    PORTA_PCR13 = PORT_PCR_MUX(3);
      PORTC_PCR3 = PORT_PCR_MUX(3);
  
    SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;     //使能FTM0时钟
    SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;    //开启FTM1模块时钟
    SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;    //开启FTM2模块时钟
    
    FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置FTM0_CH3 
    FTM1_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置模式 CH0
    FTM1_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //FTM1_CH1
    FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置FTM2_CH0 
    FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //配置FTM2_CH0 
    
    
    FTM0_CNT=0;//设置计数初值为0
    FTM1_CNT=0;
    FTM2_CNT=0;
    
    //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 

    //设置 the pulse width(duty cycle) is determined by (CnV - CNTIN).
    
     FTM1_MOD =1000;              //设置PWM频率为10K=100 000 000 /2^2/2500  这个100 000 000 是第七届的系统频率  2500是其设置的FTM1_MOD 值
     FTM0_MOD =2; //3145   300hz sd5舵机的频率恰当，会导致舵机很软，这个需要去调整，舵机很软的原因有两个，电压问题，频率问题。。一般在300hz左右  //18750  50hz
     FTM2_MOD =2; 
    
    FTM0_CNTIN=0;//设置初始化计数值
    FTM1_CNTIN=0;
    FTM2_CNTIN=0;
      
    FTM0_C3V=mid_angle;//1400;//1490 1.5ms//2ms  1986//2.5ms  2483 //1ms 993// 0.5ms  497  //1614

    FTM1_C0V=0;
    FTM1_C1V=0;
    
    FTM2_C0V=1;
    
    FTM0_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);
    FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(2); //设置时钟和分频
    FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);
}

//----------------------锁相环频率为50/15*54=180M测试函数-------------------------------//
void pllinit180M(void)
{
	uint32_t temp_reg;
        //使能IO端口时钟    
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                              | SIM_SCGC5_PORTB_MASK
                              | SIM_SCGC5_PORTC_MASK
                              | SIM_SCGC5_PORTD_MASK
                              | SIM_SCGC5_PORTE_MASK );
    //这里处在默认的FEI模式
    //首先移动到FBE模式
    MCG_C2 = 0;  
    //MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
    //初始化晶振后释放锁定状态的振荡器和GPIO
    SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
    LLWU_CS |= LLWU_CS_ACKISO_MASK;
    
    //选择外部晶振，参考分频器，清IREFS来启动外部晶振
    //011 If RANGE = 0, Divide Factor is 8; for all other RANGE values, Divide Factor is 256.
    MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
    
    //等待晶振稳定	    
    //while (!(MCG_S & MCG_S_OSCINIT_MASK)){}              //等待锁相环初始化结束
    while (MCG_S & MCG_S_IREFST_MASK){}                  //等待时钟切换到外部参考时钟
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}
    
    //进入FBE模式,
    //0x18==25分频=2M,
    //0x08==15分频=3.333M 
    //0x09==16分频=3.125M,
    //0x10==17分频=2.94M 
    //0x11==18分频=2.7778M 
    //0x12==19分频=2.63M,
    //0x13==20分频=2.5M    
    MCG_C5 = MCG_C5_PRDIV(0x0e);                
    
    //确保MCG_C6处于复位状态，禁止LOLIE、PLL、和时钟控制器，清PLL VCO分频器
    MCG_C6 = 0x0;
    
    //保存FMC_PFAPR当前的值
    temp_reg = FMC_PFAPR;
    
    //通过M&PFD置位M0PFD来禁止预取功能
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;    
    ///设置系统分频器
    //MCG=PLL, core = MCG, bus = MCG/3, FlexBus = MCG/3, Flash clock= MCG/8
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) 
                 | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);       
    
    //从新存FMC_PFAPR的原始值
    FMC_PFAPR = temp_reg; 
    
    //设置VCO分频器，使能PLL为100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(30);  //VDIV = 31 (x54)
                                                  //VDIV = 26 (x50)
    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set    
    while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set    
    
    //进入PBE模式    
    //通过清零CLKS位来进入PEE模式
    // CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
    MCG_C1 &= ~MCG_C1_CLKS_MASK;
    
    //等待时钟状态位更新
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};
    //SIM_CLKDIV2 |= SIM_CLKDIV2_USBDIV(1);  
    
    //设置跟踪时钟为内核时钟
    SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;	
    //在PTA6引脚上使能TRACE_CLKOU功能
    PORTA_PCR6 = ( PORT_PCR_MUX(0x7));  
    //使能FlexBus模块时钟
    SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
    //在PTA6引脚上使能FB_CLKOUT功能
    PORTC_PCR3 = ( PORT_PCR_MUX(0x5));
}


int16 f_abs16(int16 x)
{
  if(x>0) return x;
  else return -x;
}

float f_absf(float x)
{
    if(x>=0.0) return x;
    else  return -x;
}

int MIN_INT( int a, int b)
{
    if(a>b) return b;
    else  return a;
}

//-----------------------------------扫描白线基准线---------------------------------//
/*
1、由于赛道的宽度在图像中所占的比例较大，故可认为在中点的位置（79处）就一定是在赛道中，
，除非车子跑出了赛道，而不需要考虑中心偏离赛道的情况。
*/
void Search_WhiteBase(void)   //从图像底部中间开始向两边扫描白线基准
{ 
  uint8 i = 0,j = 0 ;//定义十六位的有符号变量   i代表行变量  j代表列变量
  uint8 base_sum = 0; 
  current_deal_line=0;                //记录在搜索基准行的时候的当前处理的行 
  bottom_whitebase = 0;//基准行赋初值  int
  find_whitebase_flag = 0;               //是否发现白线基准标志

  //////////////////////////////滤波开始///////////////////////////////////  
//首先对整幅图像进行滤波，采用的方法是中值滤波
  for(i = 0;i < ROW / 5;i++)                 //只是对图像前几行进行滤波，原因是远处的滤波可能会把跑道的信息滤除掉这里对赛道的前13行滤波
    for(j = 1;j< COLUMN-1;j++)
    {
        base_sum = (VideoImage1[i][j-1] + VideoImage1[i][j+1])/ 2;
        if( f_abs16( base_sum - VideoImage1[i][j]) > OT)
           VideoImage1[i][j] = base_sum;
    }  //滤波可能带来一个后果，就是可能把远处的边沿线滤除掉，这里只是对近端进行滤波
  
  
  /*////////////////////////对图像上的噪点进行滤除//////////////   限幅滤波会带来一个后果，导致跳变沿检测出现问题，跳变太小
  for(i = 0;i < ROW ;i++)                 //图像上突然的出现了很多的噪点，这个程序是为了将图像上的这些噪点滤除。经过验证效果很好
    for(j = 0;j< COLUMN;j++)
    {
      if(VideoImage1[i][j] >245 || VideoImage1[i][j]<10)
      {
        if( j>0 && j < COLUMN - 1)
        {
          VideoImage1[i][j] = (VideoImage1[i][j+1] + VideoImage1[i][j-1])/2;
        }
        else
        {
          if( i>0 && i < ROW - 1)
          VideoImage1[i][j] =  (VideoImage1[i+1][j] +  VideoImage1[i-1][j])/2;
        }
      }
    }
*////////////////////////////滤波结束////////////////////////////////
  
  /*对于搜索基准行最重要的就是解决搜索的开始点的问题，这个点找到了，其他的问题就好解决了*/
  if(VideoImage1[0][re_white_refer] > BASE_OT && VideoImage1[0][re_white_refer-1] >BASE_OT && VideoImage1[0][re_white_refer+1]>BASE_OT)
  {
    whitebase_searchstart = re_white_refer;
  }
  else
  {
    j = MID-1;//从MID开始搜索基准行的开始点
    left_whitebase_searchstart = MID-1;
    while(j > 10)
    {
      if(VideoImage1[0][j] > BASE_OT && VideoImage1[0][j-1] > BASE_OT &&VideoImage1[0][j-2]>BASE_OT
         &&VideoImage1[0][j-3] >BASE_OT &&VideoImage1[0][j-4]>BASE_OT)
      {
        left_whitebase_searchstart = j;
        break;
      }
      j--;
    }
    
    j = MID+1; 
    right_whitebase_searchstart = MID+1;
    while(j < 150)
    {
       if(VideoImage1[0][j] > BASE_OT && VideoImage1[0][j+1] > BASE_OT && VideoImage1[0][j+2]>BASE_OT
         && VideoImage1[0][j+3] > BASE_OT &&VideoImage1[0][j+4] > BASE_OT)
      {
        right_whitebase_searchstart = j;
        break;
      }
      j++;
    }
    
    if(right_whitebase_searchstart-MID > MID-left_whitebase_searchstart)
    {
      whitebase_searchstart  = left_whitebase_searchstart;
    }
      
    else
    {
      whitebase_searchstart = right_whitebase_searchstart;
    }
  } 
  
  
  
  while( find_whitebase_flag ==0 && current_deal_line < ROW - 1) //基准行的搜索范围从0到ROW-1 
  {
      //每行的处理清零
      whitepoint_start = whitebase_searchstart;   //uint8 
      //搜索左边的基准边沿/////////////////////////////
       j = whitebase_searchstart;   //有的时候出现前一行的中点在下一行的图像的外面
       //每次从上一场的基准的中点开始向两边搜索基准行
       while(j >= 3  )//一般为了使得跳变沿更加的明显，采用隔点判断
      {
        
          if( whitepoint_start != whitebase_searchstart && f_abs16(j-re_whitepoint_start) > f_abs16(whitepoint_start -re_whitepoint_start) )
          {
            break;
          }
          else if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] > OT 
             && VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-3] > OT
               )
          {//当第一个if不满足的时候说明此时的j到上一场的点的距离一定小于 f_abs16(whitepoint_start -re_whitepoint_start) 
            //所以这里只要遇到了跳变，就给基准行的起始点赋值
               whitepoint_start = j;
          }
          j--;
      }
      
      if( j == 2  && whitepoint_start == whitebase_searchstart)  //到达边界了，但是还没有对whitepoint_start赋值过，说明没有找到基准的开始点
      {
        if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] > OT)
        {
           whitepoint_start = 2; //到达了边界
        }
        else if( VideoImage1[current_deal_line][j-1] - VideoImage1[current_deal_line][j-2] > OT)
        {
           whitepoint_start = 1; //到达了边界
        }
        else
        { 
           whitepoint_start = 0;
        }
      }
      //左边搜索结束///////////////////////
      
      //右边搜索开始/////////////////////
      whitepoint_end = whitebase_searchstart;    //uint8
      j = whitebase_searchstart;   //每次从上一场的white_refer向两边搜索基准行
      while( j <= COLUMN-4 )//一般为了使得跳变沿更加的明显，采用隔点判断
      {
       
          if( whitepoint_end != whitebase_searchstart && f_abs16(j-re_whitepoint_end) > f_abs16(whitepoint_end -re_whitepoint_end) )
          {
             break;
          }
          else if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+2] > OT
             && VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+3] > OT)
          {
             whitepoint_end = j;
          } 
          j++;
      } 
      
      if(j == COLUMN-3  &&   whitepoint_end == whitebase_searchstart)
      {
         if( VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j+2] > OT)
         {
           whitepoint_end = COLUMN-3;
         }
         else if( VideoImage1[current_deal_line][j+1] - VideoImage1[current_deal_line][j+2] > OT)
         {
           whitepoint_end = COLUMN-2;
         }
         else
         {
           whitepoint_end = COLUMN-1;
         }
      }
      //右边搜索结束///////////////////////
      //左右边沿线的搜索方法用的是跟踪的搜索方法，目的是只想找到一行可靠的基准行的信息
      
      
      //无论这一行是否符合要求，始终记录赛道信息

        left_black[current_deal_line] = whitepoint_start;   //记录左黑线位置 (若为0，很可能说明左边黑线丢失，即车身偏右)
        right_black[current_deal_line] = whitepoint_end;    //记录右黑线位置(若为COLUMN，很可能说明右边黑线丢失,即车身偏左)
        /*对于第一行的状态的判断有两种想法，第一：无论第一行是什么状态始终认为没有丢点 第二：将到达边沿的点视为丢点
        此外这里需要对上面的三个值进行一定的修正，并且在这里将基准行上的状态设定为没有丢点（即使有时候到达了边界）。
        （否则，这里的跟踪搜索的优势就没有了）
        */
        if(left_black[current_deal_line] == 0 && right_black[current_deal_line] < COLUMN - 1 &&  (right_black[current_deal_line] - left_black[current_deal_line]) > 155)
        {  //表示左边到达边界 丢点
          
          left_black[current_deal_line] = re_whitepoint_start;
          center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2; 
        }
        else if(right_black[current_deal_line] == COLUMN - 1 && left_black[current_deal_line] > 0 &&  (right_black[current_deal_line] - left_black[current_deal_line]) > 155)
        { //表示右边到达边界 丢点
           right_black[current_deal_line] = re_whitepoint_end;
         center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2;  //记录中心点,大于MID说明车身偏左，反之，说明车身偏右
        }
        else if(right_black[current_deal_line] == COLUMN - 1 && left_black[current_deal_line] == 0 )
        {//表示两边到达边界 丢点
          left_black[current_deal_line] = re_whitepoint_start;
          right_black[current_deal_line] = re_whitepoint_end;
          center_white[current_deal_line] = re_white_refer;
        }
        //说明由于前三行的信息一般不做处理默认为左右边沿都找到了的点 
        else
        {  //表示两边都没有丢点
          center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2;  //记录中心点,大于MID说明车身偏左，反之，说明车身偏右
        }
        //处理后重新得到图像搜索的开始结束及中心值
      whitepoint_start = left_black[current_deal_line];
      whitepoint_end =  right_black[current_deal_line];
      white_refer = center_white[current_deal_line];
      
        //加上一个赛道的宽度的限制
       if(whitepoint_end - whitepoint_start > MIN_WHITEBASE_POINT ) //这个值设置为20 
        {
          find_whitebase_flag = 1;
          re_white_refer = white_refer;  //保存本场图像的信息
          re_whitepoint_start = whitepoint_start;
          re_whitepoint_end  = whitepoint_end ;
          bottom_whitebase = current_deal_line;//记录基准行
          Row_state[bottom_whitebase] = 3; //行的状态标志位
        }
        else
        {
          find_whitebase_flag = 0;
          current_deal_line++;
        }
        
  }//while循环的结束
  if(bottom_whitebase > 0)
  {
     for( i = 0 ; i < bottom_whitebase ;i++)
     {//对之前的行进行标记
        center_white[i] = MID;  
        left_black[i] = MID - 2;   
        right_black[i] = MID + 2;  
     }
  }
}//

//------------------------由基准线定出的两边黑线为基准，找出赛道边缘-----------------------// 
/*本函数的功能定义为找线，为了处理在某些断点的情况能继续在前方找到边沿线，
只是对于边沿线进行初步的虚构，真正的对赛道两边沿线的处理虚构，主要由下一个函数完成*/
void Search_BlackEdge(void)     
{   
  int16 i = 0,j = 0,n = 0, k = 0;
  int16 un_lost_hang = bottom_whitebase;//这两个变量是用来跟踪记录最近的一行的没有丢点的行，以便于对下一行的状态进行准确的判断.初始值为bottom_whitebase因为第bottom_whitebase行始终判断为没有丢点
  deal_start_line = bottom_whitebase + 1;  
  top_whiteline = ROW -1;
  hang_search_start = white_refer;  //从基准行的中点进行扫描 
   
  for(i = deal_start_line ; i < ROW ;i++)//对状态标志进行初始化
    {
      Row_state[i] = 3;
    }
  
  for(i = deal_start_line ;i < ROW;i++) 
  {
    //////////////////左右的搜索开始///////////////////////
    //左边搜索
    j = hang_search_start;
    left_black[i] = hang_search_start;
    while(j >= 2)
    {     
      
      if(VideoImage1[i][j] - VideoImage1[i][j-2] > OT
         && f_abs16(VideoImage1[i][j]-VideoImage1[i][j+1]) < OT && f_abs16(VideoImage1[i][j+1]-VideoImage1[i][j+2]) < OT && VideoImage1[i][j+2]-VideoImage1[i][j+3] < OT)  //滤除边沿噪点
        {
          if(f_abs16(j - left_black[i-1]) < f_abs16(left_black[i] - left_black[i-1]))//滤除干扰
           left_black[i] = j;
        } 
       if(left_black[i] != hang_search_start && (f_abs16(j - left_black[i-1]) > f_abs16(left_black[i] - left_black[i-1])
                                                 || (f_abs16(left_black[i] - left_black[i-1])  < 5 && j<=left_black[i-1] )))
       {
        break;
       }//减少计算量，搜索到最近的一个跳变点，则停止
     
         //当前一个状态是断点的状态时，这个时候当在内部搜索到了跳变沿的时候，则就不进行搜索，若是没有搜到，
          //则再到赛道的两边进行搜索
       if(Row_state[i-1] == 0 || Row_state[i-1] == 2)
          {
            if( j <  left_black[i - 1]  && left_black[i] != hang_search_start) //当前一行为断点状态时，搜索到了点之后，则不允许继续的搜索
             {
              break; 
            }
          }
          j--;
    }      //搜索左边沿线的while结束
    //对左边沿线的出界判定
    if(j == 1 && left_black[i] == hang_search_start)      //到达边界了，但是边沿线没有改变时，在搜索范围内没有找到跳变点，则认为是图像依然丢点
    {
      if(VideoImage1[i][j] - VideoImage1[i][j-1] > OT)
         left_black[i] = 1;
      else
        left_black[i] = 0;
    }

  
    //右边搜索
    j = hang_search_start;
    right_black[i] = hang_search_start;
    
    while( j <=COLUMN-3 )
    { 
      if( VideoImage1[i][j] - VideoImage1[i][j+2] > OT 
         && f_abs16(VideoImage1[i][j]-VideoImage1[i][j-1]) < OT && f_abs16(VideoImage1[i][j-1]-VideoImage1[i][j-2]) < OT && VideoImage1[i][j-2]-VideoImage1[i][j-3] < OT)  //滤除边沿噪点
        {
          if(f_abs16(j-right_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
          {
                right_black[i] = j ;
          }
        }
      if(right_black[i] != hang_search_start &&( f_abs16(j-right_black[i-1]) > f_abs16(right_black[i] - right_black[i-1])
         ||(f_abs16(right_black[i] - right_black[i-1])<5 && j== right_black[i-1] ) ))//在附近搜索到了点，只要到达了前一行的列位置，则停止
      {
        break;
      }
      if(Row_state[i-1] == 1 || Row_state[i-1] == 2)
          {
            if( j > right_black[i - 1]  && right_black[i] != hang_search_start) //当前一行为断点状态时，搜索到了点之后，则不允许继续的搜索
             {
              break;
            }
            //当搜索到的线大于了
            //if()
          }
        j++;
    }    //右边的while搜索结束
  
    if(j == COLUMN-2 && right_black[i] == hang_search_start)
    {
      if( VideoImage1[i][j] - VideoImage1[i][j+1] > OT)
         right_black[i] = COLUMN - 2 ;
      else
         right_black[i] = COLUMN - 1 ;
    }
    ///////////////////////左右的搜索结束//////////////////////////
    
    
    //  /////////////赛道的状态标记开始////////////////////////////////
  if(i >= deal_start_line)//只是对处于控制区域的边界进行处理
    {    
      //当图像的边沿到达了边界的时候，判定为丢点      ---------------------丢点的第一次判断
      if((left_black[i] <= 1  || left_black[i] >= COLUMN-2 ) && right_black[i] >= 1 && right_black[i] <= COLUMN-2)
      {
           Row_state[i] =0;//左边丢点
      }
      else if((left_black[i] >= 1 && left_black[i] <= COLUMN-2 ) && (right_black[i] <= 1 || right_black[i] >= COLUMN-2))
      {
           Row_state[i] = 1;//右边丢点
      } 
      else if((left_black[i] <= 1 || left_black[i] >= COLUMN-2 ) && (right_black[i] <= 1 || right_black[i] >= COLUMN-2))
      {
           Row_state[i] = 2;//两边都边丢点
      }
      else
      {
           Row_state[i] = 3;//两边都边没有丢点
      }
      
        //对两边沿线的状态进行判断//---------------------------丢点的第二次判断
      //注意这里的判断必须要分两种情况一个是跳变点的状态，其次是前一行的状态（前一行的状态的不同需要作出不同的处理），
      if((right_black[i] - left_black[i])-(right_black[i-1] - left_black[i-1])> 8)//  若是3的话可能出现误判//这里采用绝对值限制是为了防止噪点
      {
         if(( f_abs16(left_black[i] - left_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
            && f_abs16(left_black[i] - left_black[i-1]) <= 4)//左边的突变小于右边的  说明右边的点发生了突变
         {
           if(Row_state[i] == 0 ||Row_state[i] == 2)//对于第一次进行一个判断
              Row_state[i] = 2;
           else //if(Row_state[i] == 1 ||Row_state[i] == 3)
           {
             Row_state[i] = 1;//1表示的是只有右边丢点
           }
         } 
        else if(( f_abs16(left_black[i] - left_black[i-1]) > f_abs16(right_black[i] - right_black[i-1]))
            && f_abs16(right_black[i] - right_black[i-1]) <= 4)//左边的突变小于右边的  说明右边的点发生了突变
         {
           if(Row_state[i] == 1||Row_state[i] == 2)//对于第一次进行一个判断
              Row_state[i] = 2;//0表示的是左边丢点,而右边没有丢点
           else
              Row_state[i] = 0;//0表示的是只有左边丢点
         }
         else 
         {
           Row_state[i] = 2;//2表示的是两边都丢点
         }
      }
      else 
      {
        if(Row_state[i-1] == 0)//左边丢点
        {
          if((right_black[i] - left_black[i]) - (right_black[un_lost_hang] - left_black[un_lost_hang]) > 6)
          {
            if(Row_state[i] == 1 ||Row_state[i] == 2)
            {
              Row_state[i] = 2;
            }
            if(Row_state[i] == 3)
            {
              Row_state[i] = 0;
            }
          }
        }
        
       else if(Row_state[i-1] == 1)//左边丢点
        {
          if((right_black[i] - left_black[i]) - (right_black[un_lost_hang] - left_black[un_lost_hang]) > 6)
          {
            if(Row_state[i] == 0 ||Row_state[i] == 2)
            {
              Row_state[i] = 2;
            }
           if(Row_state[i] == 3)
             Row_state[i] = 1;
          }
        }
      }
      
    //记录最近的都没有丢点的行
      if( Row_state[i] == 3)
      {
        un_lost_hang = i;
      }
      ////////////左右边沿标记结束/////////////////////////
      
      //前面对赛道进行了状态的判断，这里做出初步的拟合
      if(Row_state[i] == 0)  //左边丢点
      {
        if(right_black[i]- (right_black[i-1] - left_black[i-1]) <= 0 )//限幅
          left_black[i]=0;
        else
        left_black[i] = right_black[i] - (right_black[i-1] - left_black[i-1]);//加上1是由于下向上图像在宽度在减小的原因
      }      
      else if(Row_state[i] == 1)
      {
        if(left_black[i] + (right_black[i-1] - left_black[i-1]) >= COLUMN-1)
          right_black[i] = COLUMN-1;   
        else
          right_black[i] = left_black[i] + (right_black[i-1] - left_black[i-1]);//
      }
      else if(Row_state[i] == 2)
      {
         left_black[i] = left_black[i-1];
         right_black[i] = right_black[i-1];
       }
    } 
    
    
    hang_search_start = (right_black[i] + left_black[i])/2; 
    //////////////////左右的处理结束///////////////////////////// 
    
    ///对最高有效行的判断/////////////判断一//////////////////


    if( i> 20 &&i<=top_whiteline && right_black[i] -  left_black[i] < 3*(ROW-i)/5+ WHITE_TOP_WHITELINE_POINT 
       && (right_black[i-1] -  left_black[i-1]) <  3*(ROW-i)/5 + WHITE_TOP_WHITELINE_POINT 
       ) //只判定一次&& top_whiteline >= ROW-1
    { 
     // if(i<ROW-1) while(1){}
      top_whiteline = i;
    }
    /////////////////////判断二////////////////////////
    center_white[i] = (right_black[i] + left_black[i])/2; 
    if(i>10 && i < ROW-1 &&top_whiteline >= ROW-1 &&( VideoImage2[i-1][center_white[i]] - VideoImage2[i+1][center_white[i]] > OT - 10) 
       &&  VideoImage2[i-1][center_white[i]-1] - VideoImage2[i+1][center_white[i]-1] > OT - 10
              &&  VideoImage2[i-1][center_white[i]+1] - VideoImage2[i + 1][center_white[i]+1] > OT - 10 )  //最高行的判断用原始图像
    {
      //用这种方式有一个弊端就是，图像存在一个突变，无法真实的反应赛道，特别是60度和50度弯道的微小差别，这里对其经行修补
       top_whiteline = i-1;
      for( n = top_whiteline; n >  top_whiteline - 7;n--)
      {
        if(left_black[n] <= 1)
        {
          for( k = n; k <= top_whiteline ; k++)
          {
             left_black[k] = 0;
             //进行规划后的行的状态需要重新的定义
           if(Row_state[k] == 1 ||Row_state[k] == 2)
            {
              Row_state[k] = 2;
            }
            if(Row_state[k] == 3)
            {
              Row_state[k] = 0;
            }
          }
        //  break;  //这里还不能用break。因为有的时候可能会有一个点跳出来了。如 0 1 0 0 0
        }
        else if(right_black[n] >= COLUMN-2)
        {
          for( k = n; k <= top_whiteline ; k++)
          {
             right_black[k] = COLUMN-1;
             
           if(Row_state[k] == 0 ||Row_state[k] == 2)
            {
              Row_state[k] = 2;
            }
            if(Row_state[k] == 3)
            {
              Row_state[k] = 1;
            }
          }
          //break;
        }
      }
    }

  }//for循环的结束
  
  if(top_whiteline+1 < ROW-1)
  {
    for(n= top_whiteline+1;n<ROW; n++)
    {
        center_white[n] = MID;  
        left_black[n] = MID - 2;   
        right_black[n] = MID + 2;  
    }
  } 
  
}

//------------------------通过找出来的赛道对，两边沿线进行处理和虚构并拟合出中心线----------------------//
/*对赛道进行拉线连接，左右各自连接自己的，然后对于中线，利用赛道的状态标志，再进行一次拟合
 0  表示左边沿线断点
 1  表示右边沿线断点
 2  表示两边都断了 
到达边界后不能直接的拉线了
对于虚线和十字道路的处理，只要保证两点就行了，即1、保证能找到在跑道上的点；2、保证对赛道的行状态的记录绝对的正确
*/
void Deal_BlackEdge(void)
{ 
  int16 i=0,k=0;
  uint8 un_out_hang = bottom_whitebase ;
  uint8 lost_start_line=0;
  uint8 lost_end_line=0;
  left_top_whiteline = top_whiteline;
  right_top_whiteline = top_whiteline;

  //图像的突变可能是噪点的出现，这里可以试着对Row_state[i]经行一下中值滤波
  //对Row_state[i]滤波
 for( i=bottom_whitebase + 1;i < top_whiteline-2;i++)
 {
   if(Row_state[i-1] == Row_state[i+1] && Row_state[i-1] != Row_state[i])
   {
     Row_state[i] = Row_state[i-1];
   }
 }
  
  
  
  for( i=bottom_whitebase+1 ;i < top_whiteline-1;i++)
  {
    lost_start_line = 0;
    lost_end_line = 0;
    
    if( (Row_state[i-1] == 1 || Row_state[i-1] == 3) && (Row_state[i] == 0 || Row_state[i] == 2))   //判断左边第i 点是否丢点   
    {
       lost_start_line = i - 1;//记录丢点的前一行
       while(i < top_whiteline-1)
        {
          if((Row_state[i] == 1 || Row_state[i] == 3) && (Row_state[i+1] == 1 || Row_state[i+1] == 3))  //
          {
            if(lost_end_line >= top_whiteline -2)
              lost_end_line = i;
            else
              lost_end_line = i + 1;
            
            break; 
          }
           i++;//i++必须要在if的判断之后进行，否则会导致出错
        }
       if(lost_end_line !=0)
       {  
         for(k = lost_start_line+1; k< lost_end_line;k++)
         {
          left_black[k] = left_black[lost_start_line] + (k -lost_start_line)*(left_black[lost_end_line]-left_black[lost_start_line])/(lost_end_line - lost_start_line);  
         }
       }   
      else if(lost_end_line ==0 && lost_start_line >  top_whiteline/2 && lost_start_line <ROW-1  ) //当最顶行的点到达边沿的时候，不判定
       {   
        if( left_black[top_whiteline] > 1)
         {
           left_top_whiteline = lost_start_line;
           break;
        }
       }
         
    }
  }
  
  //右边
  for( i=bottom_whitebase ;i < top_whiteline-1;i++)
  {
    lost_start_line = 0;
    lost_end_line = 0;
    if( (Row_state[i-1] == 0 || Row_state[i-1] == 3) && (Row_state[i] == 1 || Row_state[i] == 2))//判断右边第i 点是否丢点
    {
        lost_start_line = i - 1;//记录丢点的前一行
       while(i< top_whiteline-1)
        {
          //连续的两行找到了点则认为找到了连接点
          if((Row_state[i] == 0 || Row_state[i] == 3) && (Row_state[i+1] == 0 || Row_state[i+1] == 3))
          {
            if(lost_end_line >= top_whiteline -2)
              lost_end_line = i;
            else
              lost_end_line = i + 1;
            break;
          }  
          i++;
        }
       if(lost_end_line !=0)
       {
         for(k = lost_start_line+1; k< lost_end_line;k++)
         {
          right_black[k] = right_black[lost_start_line] + (k -lost_start_line)*(right_black[lost_end_line]-right_black[lost_start_line])/(lost_end_line - lost_start_line);  
         }
       }
       else if(lost_end_line ==0 && lost_start_line > top_whiteline/2 && lost_start_line <ROW-1 )
       {   
         if(right_black[top_whiteline] < COLUMN -2)
         {
           right_top_whiteline  = lost_start_line;
           break;
        }
       }
       
    }
  }
 
  if(right_top_whiteline > left_top_whiteline)
  {
    top_whiteline = right_top_whiteline;//除了定义最高行以外，还要对丢线的那一边做补线处理,这里是右边
    for(i = left_top_whiteline;i<=right_top_whiteline;i++)
    {
       if(right_black[i]- (right_black[i-1] - left_black[i-1]) <= 0 )//限幅
          left_black[i]=0;
        else
        left_black[i] = right_black[i] - (right_black[i-1] - left_black[i-1]);
    }
    
  }
 else if(right_top_whiteline < left_top_whiteline)
 {
   top_whiteline = left_top_whiteline;
     for(i = right_top_whiteline;i<=left_top_whiteline;i++)
  {
    if(left_black[i] + (right_black[i-1] - left_black[i-1]) >= COLUMN-1)
      right_black[i] = COLUMN-1;   
    else
      right_black[i] = left_black[i] + (right_black[i-1] - left_black[i-1]);//
  }
 }
  else
  {
    top_whiteline = right_top_whiteline;
  }
 if(top_whiteline+1 <= ROW - 1) 
 {
   for(k = top_whiteline+1; k < ROW; k++)
   {
      center_white[k] = MID;  
      left_black[k] = MID - 2;   
      right_black[k] = MID + 2;  
   }
 }

 //滤除左右的变沿线的单个跳变
  for( i=bottom_whitebase + 1 ;i <= top_whiteline;i++)
  {
    if(left_black[i-1] <= 1 && left_black[i] > 1 && left_black[i+1] <= 1)
    {
      left_black[i] = 0;
      
      if(Row_state[k] == 1 ||Row_state[k] == 2)
      {
        Row_state[k] = 2;
      }
      if(Row_state[k] == 3)
      {
        Row_state[k] = 0;
      }
    }
    
   if(right_black[i-1] >= COLUMN-2 && right_black[i] < COLUMN-2 && right_black[i+1] >= COLUMN-2)
    {
      right_black[i] = COLUMN-1;
      
      if(Row_state[k] == 0 ||Row_state[k] == 2)
      {
        Row_state[k] = 2;
      }
      if(Row_state[k] == 3)
      {
        Row_state[k] = 1;
      }
    }
  }
 //边沿线滤波结束
  
 //对之前的错误的补线进行重新的虚构
  //这里的虚构会导致有一种情况发生就是，在弯道的顶行的时候，可能会出现补点，与不补点的区别，这样就导致了舵机的抖动
  for( i=bottom_whitebase ;i <= top_whiteline;i++)
  {
    if(Row_state[i] != 3)
    {
      center_white[i] = (right_black[i]+left_black[i])/2;
    }
  }
  
  //对于两边出界的点，不利用求平均值的方法去做，而是利用前一行的状态进行补充
  for(i= bottom_whitebase; i <= top_whiteline; i++)
  {
    if(left_black[i] <= 1 && right_black[i] <= COLUMN-5)  //对于左边出界的点进行补充
    {
      if( right_black[i] - (right_black[un_out_hang]-left_black[un_out_hang])/2 < 0)
        center_white[i] = 0;
      else
       center_white[i] = right_black[i] - (right_black[un_out_hang]-left_black[un_out_hang])/2;
    }
    else if(right_black[i] >= COLUMN-2 && left_black[i] >= 3)
    {
      if( left_black[i] + (right_black[un_out_hang]-left_black[un_out_hang])/2 > COLUMN - 1)
        center_white[i] = COLUMN - 1 ;
      else
       center_white[i] = left_black[i] + (right_black[un_out_hang]-left_black[un_out_hang])/2;
    }
    else
      un_out_hang = i;
  }
 
 
 
 //对中线进行中值滤波
 //对中线和边沿线进行中值滤波
 for( i=bottom_whitebase + 1;i < top_whiteline-2;i++)
 {
   if((center_white[i] > center_white[i-1] && center_white[i] > center_white[i+1]) ||(center_white[i] < center_white[i-1] && center_white[i] < center_white[i+1]))
   {
     center_white[i] = (center_white[i-1] + center_white[i+1])/2;
   }
      if((left_black[i] > left_black[i-1] && left_black[i] > left_black[i+1]) ||(left_black[i] < left_black[i-1] && left_black[i] < left_black[i+1]))
   {
     left_black[i] = (left_black[i-1] + left_black[i+1])/2;
   }
      if((right_black[i] > right_black[i-1] && right_black[i] > right_black[i+1]) ||(right_black[i] < right_black[i-1] && right_black[i] < right_black[i+1]))
   {
     right_black[i] = (right_black[i-1] + right_black[i+1])/2;
   }
 } 
 
}

/*这个程序包含了两部分，其一是赛道特征的提取；其二是赛道类型的判断
*/
void get_line_information(void)
{
  int16 i;
  uint8 ramp_count = 0;  //用于记录宽度超出限制的行的个数
  uint16 temp_center_line = 0;
  center_lost_hang = 0;
  
    /*程序的开始首先对中线出现断点的情况进行修补
  当图像的中线出现了巨大的跳变时，他前面的线全部用左右两边的中值代替
  */
  for(i = bottom_whitebase + 10 ; i < top_whiteline-5;i++)  //基准行上的偏差不用处理
  {
    if(f_abs16(center_white[i] - center_white[i+2]) > CENTER_LOST_POINT
       &&f_abs16(center_white[i] - center_white[i+3]) > CENTER_LOST_POINT)
    {
      center_lost_hang = i;
      break;
    }
  }
   if(center_lost_hang >0)
   {
     for( i = bottom_whitebase ;  i< center_lost_hang + 2;i++)
     {
       center_white[i] = (left_black[i]+right_black[i])/2;  
     }
   }
  
  for( i=top_whiteline;i>20;i--)
  {
    if((f_abs16(center_white[i] - left_black[i])<=5 &&f_abs16(center_white[i-1] - left_black[i-1])<=5)
       ||(f_abs16(center_white[i] - right_black[i])<=5 &&f_abs16(center_white[i-1] - right_black[i-1])<=5))
    {
      top_whiteline = i;
      break;
    }
  }
  if(top_whiteline < ROW - 2)
  {
    for(i = top_whiteline+1;i<ROW;i++)
    {
      center_white[i] =MID;
      right_black[i] = MID+2;
      left_black[i] = MID-2;
    }
  }
  
  
  //图像的中心线出来之后，首先要确定能够到那些行，在此之后的行全部用控制的行的
   /////////////////////求取控制的最高有效行////////////////////////////////
  danger_count = 0;
  danger_flag = 1;//没有判断之前都认为是危险状态
  control_top_whiteline = top_whiteline;
  
  ////////////////////计算控制的最高有效行//////////////////////
     while(danger_flag ==1)
     {
       for(i = control_top_whiteline;i>=deal_start_line ;i--)//表示从上向下遍历
        {
          temp_center_line = center_white[control_top_whiteline] + (i - (control_top_whiteline)) *(center_white[deal_start_line] - center_white[control_top_whiteline])/( deal_start_line-(control_top_whiteline) );
        if( right_black[i] < COLUMN - 2 && left_black[i] > 1&&
           (right_black[i] - temp_center_line < (refer_road_width[i]/5) || (temp_center_line - left_black[i] < (refer_road_width[i]/5))))
            danger_count++;  
        }
       
       if(danger_count >0)
       {
         control_top_whiteline --;  
         danger_count = 0;
       }
       else
       {
         danger_flag = 0;//危险消除
       }
     }
     
     
  /////////////////赛道的类型的判断/////////////////////////
    S_right = 0;//向右拐的计数
    S_left =0 ; //向左拐计数
    S_straight = 0;
  for( i=bottom_whitebase ;i < control_top_whiteline;i++)
 {
   if(center_white[i+1]- center_white[i] > 1)
   {
     S_right++; //S 弯右加加
   }
   else if(center_white[i]- center_white[i+1] > 1)
   {
     S_left++; //S 弯左加加
   }
   else
   {
     S_straight++;
   }
 }
 
 /*对赛道的判断
 对于赛道的判断 只是区分直道(1) 、波浪弯道(2)、 入弯(3)、弯道(4) */
 
 
 if( control_top_whiteline >= 62)
 {
   if(S_left<4 && S_right < 4 )
      direction = 1;  //直道
   else if(f_abs16(S_left-S_right) < 15 )
      direction = 2;  //波浪弯道
   else
     direction =3;// re_direction;
 }
 else if( control_top_whiteline >= 50 && control_top_whiteline < 62)
 {
   direction = 3;  //入弯道
 }
 else
 {
   if(ramp_flag == 1)
      direction = 1;//坡道视为直道
   else
      direction = 4;  //弯道当中
 }
 re_direction =direction ;
 //////////////////////赛道的类型的判断结束///////////////////////////////
 

     
  //////////////////////////////对赛道进行优化//////////////////////////////
 //需要注意的是既然已经将赛道的类型判断出来了，那么就可以按照不同的赛道实现不同的优化特别是针对波浪弯道o
 if(direction == 2)  //波浪弯道  控制行大于60
 {
   for( i=bottom_whitebase ;i <= control_top_whiteline;i++)//无论是什么样的赛道，将中线向图像的中心平移
     {
      if(center_white[i] > MID)//归中
      {
        if( center_white[i] - (control_top_whiteline - 62 ) >= MID )
          center_white[i] = center_white[i] - (control_top_whiteline - 62 );
        else
          center_white[i] = MID;
      }
      else 
      {
        if(center_white[i] + (control_top_whiteline - 62 ) <= MID)  //归中防止在波浪弯道的较大的打角
          center_white[i] = center_white[i] + (control_top_whiteline - 62 );  
        else
          center_white[i] = MID; 
      }
     }
 }
 else if(direction != 1 )   //直线状态不进行归中，防止直道漂浮
  {
   for( i=bottom_whitebase ;i <= control_top_whiteline;i++)//无论是什么样的赛道，将中线向图像的中心平移
     {
      if(center_white[i] > MID)//归中
      {
        if( center_white[i] - 1 >= MID )
          center_white[i] = center_white[i] - 1;
        else
          center_white[i] = MID;
      }
      else 
      {
        if(center_white[i] + 1 <= MID)  //归中防止在波浪弯道的较大的打角
          center_white[i] = center_white[i] + 1;  
        else
          center_white[i] = MID; 
      }
     }
 }
//////////////////////////////对赛道的优化结束//////////////////////////////
  /*
 对赛道信息的提取，主要包括以下几个量。
 只是对处于控制行一下的中线求取平均值，
 */

 //////////////////////对图像的平均值的提取/////////////////////
  center_average = 0;//清零
  center_error_average = 0;
  if(control_top_whiteline > 50)
  {
    for(i = bottom_whitebase+1;i<=control_top_whiteline- 10;i++)
   {
     center_average +=  center_white[i];
     if(i == control_top_whiteline - 10)    //只是对前100cm左右的前瞻进行加权
     {
       center_average = center_average /(control_top_whiteline - bottom_whitebase -10);
       center_linear_average = center_average ;
     }
   }
   
     //进行偏差的绝对值求和
    for( i=bottom_whitebase+1 ;i <= control_top_whiteline- 10;i++)
    {
      center_error_average += f_abs16( center_white[i]  - center_average);
    }
    center_error_average /= (control_top_whiteline - bottom_whitebase - 10) ;    //反应了所有的中线偏离中线中心的绝对的大小。他的大小在一定的程度上反应了，中线的线性度的大小
    
  }
  else
  {
   for(i = bottom_whitebase+1;i<=control_top_whiteline;i++)
   {
     center_average +=  center_white[i];
     if(i == control_top_whiteline)    //只是对前100cm左右的前瞻进行加权
     {
       center_average = center_average /(control_top_whiteline - bottom_whitebase);
     }
   }
        //进行偏差的绝对值求和
    for( i=bottom_whitebase+1 ;i <= control_top_whiteline;i++)
    {
      center_error_average += f_abs16( center_white[i]  - center_average);
    }
    center_error_average /= (control_top_whiteline - bottom_whitebase) ;    //反应了所有的中线偏离中线中心的绝对的大小。他的大小在一定的程度上反应了，中线的线性度的大小
    
  }
   
    //为了准确的判断出赛道的变化趋势，十分有必要的是对这个数的历史进行存储。这里存储7个历史值，然后进行模糊判断。
    //通过测试发现 这个值的变化趋势在0 到 25之间变化  当在直道的时候，值在0 - -8之间在弯道中的时候，是在16--25之间变化
    //当数值达到19后，则认为已经到达弯道中间或是在出弯道
  ///////////////均值提取结束//////////////////////////////
  
  /*/////对坡道的判断，当坡道判断出来之后，用标志位标志，且其只是作用在电机的给定控制上
  对于坡道的检测只需要将上坡道检测出来（最高行接近顶行，且其宽度达到了一定的范围）下坡检测比较的困难，
  所以这里不检测下坡，一般只是做一下坡道状态延时就行了（延时时间为1s--2s之间），
  且检测出来之后，只需要降速，对于舵机可以不用去管。图像已经做得可以了。
  对坡道的检测不能只是用宽度去判定，这样容易和弯道出现误检。所以还要加上对端点的限制，将其限制在某一个范围之内,
  这就要求进入弯道之前车子是摆正的。
  #define RAMP_WIDTH  45                  //图像10~20行的宽度范围超过该范围确定为坡道
  #define RAMP_TIME   60
  uint8 ramp_time = 50;                     //进入坡道后多长时间重新开启起跑线检测
  int8 ramp_speed = 0;                    //坡道减速值
  bool ramp_flag;                          //进入坡道标志,主要用于控制
  bool ramp_dis_flag;                     //主要是防止下坡误判
  
  对于坡道的检测不能用最高的几行，因为这样容易和十字道路误检
 */
  
  ramp_count = 0;//3,65,4,25
  if( ramp_dis_flag ==0 && direction == 1 && control_top_whiteline >= ROW - 2 ) //直线状态检测  //加入这个ramp_dis_flag标志，是为了让车子在检测数跑道后的这段时间里，不对坡道进行检测
  {
    for(i = 35; i<60; i++)
      {
        if(right_black[i]-left_black[i] > refer_road_width[i] + 8)
          ramp_count ++;
      }        
    if(ramp_count >= 20)
    { 
      if(left_black[ 40] - left_black[35] > 0 && left_black[ 40] - right_black[35] < 5
         && right_black[35] -right_black[40] > 0 && right_black[35] -right_black[40] < 5)
      {   
      ramp_flag = 1;
      ramp_dis_flag = 1;
      }
    } 
  }
  if(ramp_flag == 1)
  {
    ramp_time++;
    if(ramp_time >= ramp_delay_time)
    {
      ramp_time = 0;
      ramp_flag = 0;
    }
  }
  if(ramp_dis_flag == 1)
  {
    ramp_dis_time++;
    if(ramp_dis_time >= 4 * ramp_delay_time)
    {
      ramp_dis_time = 0;
      ramp_dis_flag = 0;
    }
  }
}


/*-------------------------------获取中线的线性相关系数-----------------------------*/
float get_linear_factor(uint8 bottom,uint8 top,uint8 average)            //传递三个参数基准行，顶行，所有行与MID的差值
{
    uint8 i;
    uint8 Y_aver=0;
    float X_square_sum=0;   //X轴平方和
    float Y_square_sum=0;   //Y轴平方和
    float multi_sum=0;      //XY乘积之和
    int temp=0,temp1=0,temp2=0; //减轻浮点运算的寄存器
    float factor=0;
    
     Y_aver=(uint8)((bottom+top)/2);  //Y坐标的范围
    for(i=bottom;i<=top;i++)
    {
        temp=temp+(center_white[i]-average)*(center_white[i]-average);
        if(temp>30000)
        {
            X_square_sum=X_square_sum+temp;   //X平方和
            temp=0;
        }

        temp1=temp1+(i-Y_aver)*(i-Y_aver);
        if(temp1>30000)
        {
            Y_square_sum=Y_square_sum+temp1;   //Y平方和
            temp1=0;
        }

        temp2=temp2+(center_white[i]-average)*(i-Y_aver);
        if(f_abs16(temp2)>30000)
        {
            multi_sum=multi_sum+temp2;    //X、Y的积
            temp2=0;
        }
    }
     
        X_square_sum=X_square_sum+temp;   //得出x的平方和
        Y_square_sum=Y_square_sum+temp1;  //计算出y的平方和
        multi_sum=multi_sum+temp2;        //计算出xy的乘积
    
       /* XX_square_sum =X_square_sum;      //用于检测
        YY_square_sum =Y_square_sum;
        XYmulti_sum = multi_sum;*/
        
        if(X_square_sum<0.1)   //防止除数为0
            X_square_sum=0.1;
        if(Y_square_sum<0.1)
            Y_square_sum=0.1;
        
        if(X_square_sum<300)  //小于300出现在直道
            factor=multi_sum/f_absf(multi_sum); // =1 or =-1 //完全是直线
        else  //否则用公式计算 //注意处理速度，小于200时处理时间少很多
            factor=multi_sum/sqrt(X_square_sum*Y_square_sum)*(bottom_whitebase+control_top_whiteline-20)/(65-20);
        
        if(factor>0.95)
          factor=1;
        if(factor<-0.95)
          factor=-1;
    
        return factor;
}


/*加入起跑线检测的停车程序，利用摄像头进行检测
 这里的主要目的是给stop_flag置位,为了能够检测起跑线，必须保证的是检测的距离为起跑线前的20厘米以上。
这里的图像而言的话就是图像的30行，
以五米的速度来看的话，也就是必须保证两场图像至少有一场检测到，40ms*5m = 20厘米
注意这里的检测是检测黑到白的跳变。这个比检测白到黑的跳变更加的准确
*/
void check_start_stop_line()
{ 
  int i,j;
  uint8 left_start_stop_hang = 0;
  uint8 left_start_stop_flag = 0;
  uint8 right_start_stop_hang = 0;
  uint8 right_start_stop_flag = 0;
  //弯道不检测起跑线
  if(top_whiteline - bottom_whitebase > 50)
  {
    for(i=bottom_whitebase+3;i< bottom_whitebase + 40;i++) //只是检测前二十五行，大于车身的前20几个厘米  
    {
        //至少保证中线的左右的三个点是白色的 并且要保证这个时候的行的状态为3
      if( top_whiteline >= 50
         && (VideoImage2[i - 1][ center_white[i]] -  VideoImage2[i + 1][ center_white[i]])< OT
          && (VideoImage2[i - 1][ center_white[i] - 1] -  VideoImage2[i + 1][ center_white[i] - 1]) < OT
           && (VideoImage2[i - 1][ center_white[i] + 1] -  VideoImage2[i + 1][ center_white[i] + 1]) < OT)
      {//左右两边能够有两段是黑色的
        //这两段白色的检测用的是左右边沿各自向内缩减5格子
        for(j = left_black[i] + 5 ; j <   center_white[i] - 2;j++)
        {
          if((VideoImage2[i + 1][ j ] -  VideoImage2[i - 1][ j])> OT-10
             &&(VideoImage2[i + 1][ j + 1] -  VideoImage2[i - 1][ j+1])> OT-10
               &&(VideoImage2[i + 1][ j + 2] -  VideoImage2[i - 1][ j + 2])> OT-10)
          {
             left_start_stop_hang = i;
             left_start_stop_flag = 1;
          }
        }
        
        for(j = right_black[i] - 5 ; j > center_white[i] + 2;j--)
        {
          if((VideoImage2[i + 1][ j] -  VideoImage2[i - 1][ j]) > OT-10
             &&(VideoImage2[i + 1][ j - 1] -  VideoImage2[i - 1][ j - 1])> OT-10
               &&(VideoImage2[i + 1][ j - 2] -  VideoImage2[i - 1][ j - 2])> OT-10 )
          {
             right_start_stop_hang = i;
             right_start_stop_flag = 1;
          }
        }//右边搜索 
      } //中线符合标准
        //当找到了符合的起跑线的时候，跳出循环//判断是还要防止图像的错位
        if(  left_start_stop_flag == 1 && right_start_stop_flag == 1 && f_abs16(right_start_stop_hang - left_start_stop_hang) < 3)
        {
          stopflag = 1;
          break;//
        }
    }//for循环
  }
}


/*-----------------------------------舵机和电机的控制函数的变量---------------------------------
这个函数用于进行赛道的优化和控制，其中控制包括电机和舵机，这两部分。对其进行统一的控制
*/

void Control()
{
 int16 i=0,j=0; //
  p_error = 0;
  refer_error = 0;
  get_p_errorline = 55;
  
  //求解偏差和舵机的控制p、d参数
  if(control_top_whiteline -deal_start_line >10)
  {
    if(control_top_whiteline < get_p_errorline )
      get_p_errorline = control_top_whiteline ;  //限制舵机的控制行舵机的控制行不能太长，太长容易在弯道入直道出现震荡
    
    for(j= get_p_errorline; j>= get_p_errorline -4; j--)
    {
      p_error += center_white[j];
    }
    //得出本场的error  
    p_error  = (MID -  p_error/ 5);
    
    //获取全场的一个偏差
    refer_error = MID - center_average;       //在车子出入直角弯道和一般的弯道进入直道的时候，容易出现震荡，这个和以前直接用加权平均的算法很相似，所以这里将其变的连续写
    
    ref_his_error[0] = ref_his_error[1] ;
    ref_his_error[1] = ref_his_error[2] ;
    ref_his_error[2] = ref_his_error[3] ;
    ref_his_error[3] = ref_his_error[4] ;
    ref_his_error[4] = refer_error ;
    
    //得到加权之后的引导的偏差
   refer_error  =((ref_his_error[0] + ref_his_error[1] + ref_his_error[2]+ 2*ref_his_error[3])+95*ref_his_error[4])/100;  //取出顶端的10行，有利于对波浪弯道的控制
    
    //引导的error
   /*if((re_control_top_whiteline < control_top_whiteline) && (direction == 3 || direction== 4))  //re_control_top_whiteline top_error_servo_p
     top_error_servo_p = 1*(control_top_whiteline - re_control_top_whiteline)/6;
   else if((re_control_top_whiteline > control_top_whiteline) &&  direction== 4)        //入弯道加大打角
     top_error_servo_p = 1*(re_control_top_whiteline - control_top_whiteline)/6;
   else
   
   uint16 side_count_p = 0;
   side_count_p = 0;
   for(i=deal_start_line;i<=control_top_whiteline;i++)
   {
     if(center_white[i]>MID)
     {
        side_count_p ++;
     }
     else if(center_white[i]<MID)
     {
       side_count_p--;
     }
   }
   */
   if(direction == 1 && control_top_whiteline >= ROW - 2)  //当时直线状态的时候，顶部与底部有较大的偏差的时候，这个时候仍然需要一个p的作用
   {
     top_error_servo_p = 3*f_abs16(center_white[deal_start_line] - center_white[control_top_whiteline-1])/4;       //顶部与底部的偏差最大能达到70左右
   }
   else
     top_error_servo_p = 0;
     
    error_servo_p = 5*(ROW - control_top_whiteline )/13  +  1*f_abs16(p_error)/8 + lcd_error_servo_p;  //增加这个p有利于进弯道切
    error_servo_d =lcd_error_servo_d;//control_top_whiteline ;  //当在比较直的线路上要求d稍微大一点 波浪弯道，过不了一直是d的问题 ，太大了
    
    //归中的error
    error_servo_ref_p = 1 * f_abs16(refer_error)/7 + lcd_ref_p + top_error_servo_p;  //增加这个p有利于出弯道内切     1/7
    error_servo_ref_d = lcd_ref_d;//2*center_white_average;
    
    angle=(uint16)(mid_angle + (error_servo_p * p_error + error_servo_d * (p_error - p_re_error) + 
                                error_servo_ref_p * refer_error + error_servo_ref_d * (refer_error - re_refer_error)) / 10);//
  } 
 else
   angle = re_angle;

    if(angle > mid_angle +240)   //1570 235
      angle = mid_angle +240;
    if(angle<mid_angle -240)
      angle=mid_angle -240;
      FTM0_C3V=angle;
      //这个时候可以调节舵机的中心值来使得车子跑正
      //历史值的保留
      re_angle= angle;
  p_re_error = p_error;
  re_refer_error =refer_error;
  re_control_top_whiteline = control_top_whiteline;
  
  
  //下面是电机的控制程序 速度的控制融入了偏差和处理的有效行
 //对于这两点的应用，根据最高有效行动态的改变最高速和最低速
  //然后通过偏差来计算准确的速度
  
  //每次对速度需要重新的设定

    if(speed_select == 0)
  {
       straight_speed = mySpeedswitch[0].Cstraightspeed;
       bow_speed = mySpeedswitch[0].Cbowspeed;
       straight_speed_ed = mySpeedswitch[0].Cstraightspeed_ed;
   }
   else if(speed_select == 1)
   {
       straight_speed = mySpeedswitch[1].Cstraightspeed;
       bow_speed = mySpeedswitch[1].Cbowspeed;
       straight_speed_ed = mySpeedswitch[1].Cstraightspeed_ed;
   }
      else if(speed_select == 2)
   {
       straight_speed = mySpeedswitch[2].Cstraightspeed;
       bow_speed = mySpeedswitch[2].Cbowspeed;
       straight_speed_ed = mySpeedswitch[2].Cstraightspeed_ed;
   }
      else if(speed_select == 3)
   {
       straight_speed = mySpeedswitch[3].Cstraightspeed;
       bow_speed = mySpeedswitch[3].Cbowspeed;
       straight_speed_ed = mySpeedswitch[3].Cstraightspeed_ed;
   }
      else if(speed_select == 4)
   {
       straight_speed = mySpeedswitch[4].Cstraightspeed;
       bow_speed = mySpeedswitch[4].Cbowspeed;
       straight_speed_ed = mySpeedswitch[4].Cstraightspeed_ed;
   }
   else 
   {
       straight_speed = mySpeedswitch[5].Cstraightspeed;
       bow_speed = mySpeedswitch[5].Cbowspeed;
       straight_speed_ed = mySpeedswitch[5].Cstraightspeed_ed;
   }
   
   straight_speed += lcd_straight_speed;
   bow_speed += lcd_bow_speed;
   straight_speed_ed += lcd_straight_speed_ed;
   
      if( direction == 1 )  //对于速度的控制实现不同的赛道用不同的控速
       {
         straight_speed = (control_top_whiteline+ROW)*straight_speed/ROW/2;  //限定了最高速度
         bow_speed = (control_top_whiteline+ROW)*(bow_speed)/2/ROW;         //只是限定了最低速度
       }    
      else if( direction == 2 )  //波浪弯
       {
         straight_speed = (control_top_whiteline-2)*straight_speed/ROW;  //   限定了最高速度
         bow_speed =  (control_top_whiteline-2)*(bow_speed)/ROW;         //只是限定了最低速度
       }
        else if( direction == 3)  //入弯
       {
         straight_speed = (control_top_whiteline-4)*straight_speed/ROW;  //   限定了最高速度
         bow_speed =  (control_top_whiteline-4)*(bow_speed)/ROW;         //只是限定了最低速度
       }
    
       else   //弯道
       { 
        straight_speed = (control_top_whiteline-4)*straight_speed/ROW;  //   限定了最高速度
         bow_speed =   (control_top_whiteline-4)*(bow_speed)/ROW;         //只是限定了最低速度
       }
      
        
        //center_error_average的变化范围不超过30  
      if(control_top_whiteline ==ROW - 1&&(direction==1||direction==2) )
          {
            if (direction == 1 )
            {
              straight_count++;
              if(straight_count > 2)
               speed_except = straight_speed + straight_speed_ed;  //直线和波浪弯道给全速行驶
              else
               speed_except = straight_speed; 
            }
            else
            {
              speed_except = straight_speed + 20;
            }
          }
      /*处理长直道入弯道，首先是利用线性相关系数，计算出固定前瞻的线性相关性，（比如有1.9的前瞻，只是计算出前1.2米的相关系数），
      然后利用剩下前瞻，进行对是否入弯道进行判断，判断的方法是判断顶端行的变化趋势。若是同时弯向一边，则说明是进入了弯道。利用
      这点信息进行直道入弯道的判断，具体的减速是将速度减到弯道的速度，而不是将速度减小到比弯道还低。否则容易引起车子在弯道的不稳定。
      */
 
      else if(f_absf(linear_factor) == 1)   //由于只是在进弯道的时候进行判断，所以这里就是入弯状态
      {
        straight_count = 0;
        for(i = 50;i<control_top_whiteline;i++)
        {
        if((center_white[i-1]<center_white[i-2] && center_white[i]<center_white[i-1])
             || (center_white[i-1] > center_white[i-2] && center_white[i]>center_white[i-1]))
               break;
        }
         full_speed_line = i;
         
         if(full_speed_line > 62)
         {
           speed_except = MIN_INT((3*straight_speed+1*bow_speed)/4,120);
         }
         else if(full_speed_line > 58)
         {
           speed_except = MIN_INT((3*straight_speed+bow_speed)/4,110);
         }
         else if(full_speed_line > 54)
         {
           speed_except = MIN_INT((2*straight_speed+bow_speed)/3,100);
         }
         else
         {
           speed_except = MIN_INT((straight_speed+2*bow_speed)/3,94);
         }
         
      }
      else
      {
        straight_count = 0;
        if(straight_speed - center_error_average*center_error_average*(straight_speed - bow_speed)/900 < bow_speed )
          speed_except =  bow_speed; 
        else 
          speed_except = straight_speed - center_error_average*center_error_average*(straight_speed - bow_speed)/900 ;
      }
      
      if(stopflag == 1 && speed_down_cnt <= 10)
      {
        speed_down_cnt ++;
      }

}

//------------------------串口发送函数------------------------//
void SCI0_send_mesage(void)
{
    int i = 0,j = 0;
    static bool sci_temp = 0;
    DisableInterrupts;  //发送图像数据时，要关闭所有中断，否则会出错    
    if(send_mes == 1)  //图像
    {  
      while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
        UART0_D = WHITE_BLACK_OT;//由于阀值的不存在，故这里只是随便填写的一个数字
              while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
        UART0_D = (uint8)ROW;
              while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
        UART0_D = (uint8)COLUMN;
        
        
        //上位机显示的第一个点是左上角，所以我发的时候第一个点就发左上角的点
      for(i =ROW-1;i>=0;i--)
      {
        for(j=0;j<COLUMN;j++)
        {
          while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
          UART0_D =  VideoImage2[i][j];///见最后一个函数讲解
          Delay_MS(80000);
        }
      }  
      for (i =ROW-1 ;i >=0 ; i--)
      {
        while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
           UART0_D = left_black[i];
        Delay_MS(80000);
      }
      for (i =ROW-1 ;i >=0 ; i--)
      {
         while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
         UART0_D = right_black[i];
         Delay_MS(80000);
      }
      for (i =ROW-1 ;i >=0 ; i--)
      {
        while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
        UART0_D = center_white[i];
        Delay_MS(80000);
      }
      send_mes = 0;  //发送一次即可，所以要清零
     }
   // EnableInterrupts;  //重新开启所有中断
    
    else if(send_mes == 2)
    {
      for(i =0;i<=ROW - 1;i++)
      {
          while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
          UART0_D = center_white[control_top_whiteline];
          Delay_MS(80000);
      }
       
    }
 
   else if (send_mes == 3)  //便于调试
    {  
        if(!sci_temp)
        {
       while(!(UART0_S1&UART_S1_TDRE_MASK));
       UART0_D = (uint8)(speed_feedback);//speed_feedback
       sci_temp = !sci_temp;
        }
        else
        {
         while(!(UART0_S1&UART_S1_TDRE_MASK));//等待数据到达
         UART0_D= (uint8)(speed_except);
          sci_temp = !sci_temp;
        }
        //       send_mes = 0;不清0是为了连续发送
      }
    else if (send_mes =='p')  //停车
    {
       while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
       stopflag = 1;
       send_mes = 0;
    }
    else if (send_mes == 's')  //启动
    {
       while(!(UART0_S1&UART_S1_TDRE_MASK));   //等待数据到达
       stopflag = 0;
       send_mes = 0;  
    }
    EnableInterrupts;
}

//拨码用的八位对应到k60上是 PTD6---- PTD13
// 0000 0000 0000 0000 0000 0000 0000 0000 
void scan_boma(void)
{
  uint32 temp1=0;
  uint8 temp=0;
  GPIOD_PDOR = 0xffffffff; 
  temp1 = GPIOD_PDIR;   //读PTD6~PTD13 
  
  temp = !(uint8)((temp1 >> 13) & 0x00000001);
  if(lcd_debug == 1)
  {
    if(temp == 1)   //对应的是拨码8
     lcd_debug = 1;  //点亮的状态为按键调节时间，若调节完毕，则拨码检测结束，程序向下运行
  else
     lcd_debug = 0;
  }
  temp = !(uint8)((temp1 >> 12) & 0x00000001);//对应的是拨码7
  if(temp == 1)
       redraw_control=1;  //拨亮则刷屏
  else
       redraw_control=0; 
  
   //  start_stop_cs 起跑线检测的片选信号
    temp = !(uint8)((temp1 >> 11) & 0x00000001);//0x00000800
   if(temp == 1)
     start_stop_cs = 1;
   else
     start_stop_cs = 0;
   
  //  速度选择信号
   /*temp = !(uint8)((temp1 >> 10) & 0x00000001);//0x00000800
      if(temp == 1)
     stop_delay_check = 1;  //匀速，用于测试
      else
     stop_delay_check = 0;*/
   
}


//-----------------------------延迟-------------------------------//
void Delay_MS(uint32 ms)
{
   while(ms--);
}


/***********************************预显示**********************************/
void pre_show(void)
{
  LCD_CLS();
   switch(lcd_page_num)
   {        
      case 0:     //第一页只是用来显示参数      
             break;        
             
      case 1:
             LCD_P6x8Cha(0,lcd_line_num,'*');
             
             LCD_P6x8Str(10,0,"lcd_debug:");      //调试选择
             LCD_P6x8Num(100,0,lcd_debug);  
             
             LCD_P6x8Str(10,1,"speed_select:");      //速度档位选择
             LCD_P6x8Num(100,1,speed_select);
             
             LCD_P6x8Str(10,2,"s_pit_count:");      //起跑线的延时检测计数
             LCD_P6x8Num(100,2,stop_pit_count);
             
             LCD_P6x8Str(10,3,"rampdetime:");      //坡道延时时间
             LCD_P6x8Num(100,3,ramp_delay_time);
             
             LCD_P6x8Str(10,4,"mid_angle:");      //第五行，舵机中值
             LCD_P6x8Num(100,4,mid_angle);  
                     
             LCD_P6x8Str(10,5,"WHITE_BLACK_OT:");      //第六行，二值化的阈值
             LCD_P6x8Num(100,5,WHITE_BLACK_OT);  
             
             LCD_P6x8Str(10,6,"test_run:");      //第六行，二值化的阈值
             LCD_P6x8Num(100,6,test_run); 
             
            break;
            
      case 2:
             LCD_P6x8Cha(0,lcd_line_num,'*');
        
             LCD_P6x8Str(10,0,"l_er_ser_p:");    //第一行：舵机的p
             LCD_P6x8Num(100,0,lcd_error_servo_p);
             
             LCD_P6x8Str(10,1,"l_er_ser_d:");    //第二行 ：舵机的d
             LCD_P6x8Num(100,1,lcd_error_servo_d); 
             
             LCD_P6x8Str(10,2,"lcd_ref_p:");    //第三行 ：舵机的p
             LCD_P6x8Num(100,2,lcd_ref_p);
             
             LCD_P6x8Str(10,3,"lcd_ref_d:");    //第四行 ：舵机的d
             LCD_P6x8Num(100,3,lcd_ref_d);         
             
             LCD_P6x8Str(10,4,"l_str_speed:");     //直线速度
             LCD_P6x8Num(100,4,lcd_straight_speed); 
             
             LCD_P6x8Str(10,5,"l_bow_speed:");     //弯道速度
             LCD_P6x8Num(100,5,lcd_bow_speed); 
             
             
             LCD_P6x8Str(10,6,"lcd_strspe_ed:");     //长直道的附加速度
             LCD_P6x8Num(100,6,lcd_straight_speed_ed);
             

             break;
     }

}


/*
小的液晶屏的参数：
128*64  写成6*8  每行共21个字符  一共可以写8列------
                                              |
                                              |
                                              |
                                              |
                                              |
                                              |
                                              |
对于定位的x和y  x表示的是列数。这里以每一个点来计算，而不是每个单元格子
而y则是以每个单元格子计算，代表的是8

*/
/**************************************刷屏，显示时变变量*********************************/
/*刷屏的时候特别是要注意时间的问题，可能刷屏的时间过长会导致控制的不及时，可能会出错
这里将采用一场只刷屏一次的策略*/
void redraw()
{
  byte lcd_hang = 1 ; 
  if(lcd_page_num==0&&redraw_control==1)     //第一页//redraw_control需要另一个拨码进行控制
     {
         if(lcd_hang == 1)
         {
           LCD_CLS_ROW(0,0);      //0行  8列
           LCD_P6x8Num(0,0,top_whiteline);    //第一行  图像相关
           LCD_P6x8Num(40,0,control_top_whiteline);  
           LCD_P6x8Num(80,0,deal_start_line); 
           lcd_hang ++;
         }
         if(lcd_hang == 2)
         {
          LCD_CLS_ROW(0,1);       //第二行，舵机偏差相关
          LCD_P6x8Num(0,1,white_refer);
          LCD_P6x8Num(40,1,center_average);
          LCD_P6x8Num(80,1,p_error);
          LCD_P6x8Num(100,1, center_error_average);
          lcd_hang ++;
         }
        if(lcd_hang == 3)
         {
          LCD_CLS_ROW(0,2);       //第三行， 电机相关
          LCD_P6x8Num(0,2,speed); 
          LCD_P6x8Num(60,2,speed_except); 
          LCD_P6x8Num(100,2, speed_feedback);
          lcd_hang ++;
         }
         if(lcd_hang == 4)
         {
          LCD_CLS_ROW(0,3);       //第四行，   //赛道的类型标志
          LCD_P6x8Num(0,3,S_straight);
          LCD_P6x8Num(40,3,S_left); 
          LCD_P6x8Num(70,3,S_right);
          LCD_P6x8Num(110,3,direction);
          lcd_hang ++;
         }
           if(lcd_hang == 5)
         {
          LCD_CLS_ROW(0,4);       //第五行，   //标志位
          LCD_P6x8Num(0,4, stopflag); 
          LCD_P6x8Num(40,4, ramp_flag);
          LCD_P6x8Num(80,4, linear_factor);
          lcd_hang =1;
         }
       /*    if(lcd_hang == 6)
         {
          LCD_CLS_ROW(0,5);//第三行，   //线性相关
           LCD_P6x8Num(0,5, XX_square_sum); 
           LCD_P6x8Num(60,5, YY_square_sum);
           LCD_P6x8Num(110,5, XYmulti_sum);
           lcd_hang = 1;   //恢复置1
         }
                */    
     }  
}

void key_down(void)
{
  uint8 temp=0;
  uint32 temp1=0;
  //端口由c8开始往后的八位设置为高点平
    Delay_MS(200000 * 4);
    GPIOC_PDOR = 0xffffffff; 
    temp1 = GPIOC_PDIR;   //读PTC8~PTC15   
    temp = ~(uint8)((temp1 >> 8) & 0x000000ff);
    
     if(temp==0x01)
        sub_NUM=1;//   
     else
        sub_NUM=0;//  
     if(temp==0x02)
       se_sub_NUM=1;//  
     else
       se_sub_NUM=0;  //  
     if(temp==0x04)
        up_line=1;//   
     else
        up_line=0; //   
     if(temp==0x08)
       change_page=1 ; // 
     else
        change_page=0;// 
     if(temp==0x10)
        add_NUM=1 ;  //
     else
        add_NUM=0    ; //
     if(temp==0x20)
         down_line=1;    
     else
        down_line=0; 
    
   
}

//---------------------------全键盘扫描-----------------------------//
void Keyscan(void)
{
      key_down();
      
      if(change_page)  //如果检测到低电平，说明按键按下
      {    key_down(); //延时去抖，一般10-20ms
           if(change_page)     //再次确认按键是否按下，没有按下则退出
           {   
              while(change_page)//如果确认按下按键等待按键释放，没有释放则一直等待
                key_down();
              if(lcd_page_num<2)    //页序号加操作这里的页数2是可以改动的
	         lcd_page_num++;
	       else
	         lcd_page_num=1;
               lcd_line_num=0;
              pre_show();//这里是显示函数，提前写在这里一会再写这个函数的函数体
               
           }
      }
     //减页数 
      if(se_sub_NUM)  //如果检测到低电平，说明按键按下
      {    key_down(); //延时去抖，一般10-20ms
           if(se_sub_NUM)     //再次确认按键是否按下，没有按下则退出
           {
              while(se_sub_NUM)//如果确认按下按键等待按键释放，没有释放则一直等待
              
               key_down();
               LCD_change_value(lcd_page_num,lcd_line_num,-1);
               
           }
      }
      
      
     if(lcd_page_num!=0)     //如不为第一页，则进行下一步扫描  
     { //行扫描
      //向上
       key_down();
      if(up_line)  //如果检测到低电平，说明按键按下
      {
            key_down(); //延时去抖，一般10-20ms
            if(up_line)     //再次确认按键是否按下，没有按下则退出
            {
               while(up_line)//如果确认按下按键等待按键释放，没有释放则一直等待
                 key_down();
               if(lcd_page_num!=0)
	        LCD_P6x8Cha(0,lcd_line_num,' ');
               
               
                if(lcd_line_num<LCD_ROW)    //行序号加操作
	         lcd_line_num++;
		 else
		  lcd_line_num=0;
                if(lcd_page_num!=0)  
              LCD_P6x8Cha(0,lcd_line_num,'*');
            }
      }
      //向下
       if(down_line)  //如果检测到低电平，说明按键按下
      {
            key_down(); //延时去抖，一般10-20ms
            if(down_line)     //再次确认按键是否按下，没有按下则退出
            {
               while(down_line)//如果确认按下按键等待按键释放，没有释放则一直等待
                 key_down(); 
                if(lcd_page_num!=0)
	        LCD_P6x8Cha(0,lcd_line_num,' ');
                if(lcd_line_num>0)    //行序号加操作
	         lcd_line_num--;
		 else
		  lcd_line_num=LCD_ROW;
                  
              LCD_P6x8Cha(0,lcd_line_num,'*');//
            }
      }
      
       if(add_NUM)  //如果检测到低电平，说明按键按下
    {
	key_down(); //延时去抖，一般10-20ms
     if(add_NUM)     //再次确认按键是否按下，没有按下则退出
	   {
      while(add_NUM)//如果确认按下按键等待按键释放，没有释放则一直等待
        key_down();
        LCD_change_value(lcd_page_num,lcd_line_num,1);
	   }
     }
      
     
     if(sub_NUM)  //如果检测到低电平，说明按键按下
    {
	key_down(); //延时去抖，一般10-20ms
     if(sub_NUM)     //再次确认按键是否按下，没有按下则退出
	   {
        while(sub_NUM)//如果确认按下按键等待按键释放，没有释放则一直等待
          key_down();
     LCD_change_value(lcd_page_num,lcd_line_num,-1);
	   }
    }
    
    
     }
 }
//-------------------------上-电LCD键盘调试---------------------//
void LCD_change_value(unsigned char page,unsigned char m,int i)
{
    
   if(page==1)
   {
    switch(m)
       {
        case 0:lcd_debug+=i;
                LCD_P6x8Cha(0,0,'*'); 
                LCD_CLS_ROW(100,0);
                LCD_P6x8Num(100,0,lcd_debug);
                break; 
        case 1:speed_select+=i;
                LCD_P6x8Cha(0,1,'*'); 
                LCD_CLS_ROW(100,1);
                LCD_P6x8Num(100,1,speed_select);
                break;
         case 2:stop_pit_count+=i;
                LCD_P6x8Cha(0,2,'*'); 
                LCD_CLS_ROW(100,2);
                LCD_P6x8Num(100,2,stop_pit_count);//lcd_ref_p
                break;     
         case 3:ramp_delay_time+=i;
                LCD_P6x8Cha(0,3,'*'); 
                LCD_CLS_ROW(100,3);
                LCD_P6x8Num(100,3,ramp_delay_time);//lcd_ref_p
                break; 
         case 4:mid_angle+=i;
                LCD_P6x8Cha(0,4,'*'); 
                LCD_CLS_ROW(100,4);
                LCD_P6x8Num(100,4,mid_angle);//lcd_ref_p
                break;        
         case 5:WHITE_BLACK_OT +=i;
                LCD_P6x8Cha(0,5,'*'); 
                LCD_CLS_ROW(100,5);
                LCD_P6x8Num(100,5,WHITE_BLACK_OT);
                break;  
         case 6:test_run +=i;
                LCD_P6x8Cha(0,6,'*'); 
                LCD_CLS_ROW(100,6);
                LCD_P6x8Num(100,6,test_run);
                break;  
       }
   }

 if(page==2)
     {
          switch(m)
          {
         case 0:lcd_error_servo_p+=i;
                LCD_P6x8Cha(0,0,'*'); 
                LCD_CLS_ROW(100,0);
                LCD_P6x8Num(100,0,lcd_error_servo_p);
                break;
         case 1:lcd_error_servo_d+=i;
                LCD_P6x8Cha(0,1,'*'); 
                LCD_CLS_ROW(100,1);
                LCD_P6x8Num(100,1,lcd_error_servo_d);
                break;       
         case 2:lcd_ref_p+=i;
                LCD_P6x8Cha(0,2,'*'); 
                LCD_CLS_ROW(100,2);
                LCD_P6x8Num(100,2,lcd_ref_p);
                break;        
         case 3:lcd_ref_d += i;
                LCD_P6x8Cha(0,3,'*'); 
                LCD_CLS_ROW(100,3);
                LCD_P6x8Num(100,3,lcd_ref_d);
                break;
        case 4:lcd_straight_speed += i;
                LCD_P6x8Cha(0,4,'*'); 
                LCD_CLS_ROW(100,4);
                LCD_P6x8Num(100,4,lcd_straight_speed);
                break;
        case 5:lcd_bow_speed += i;
                LCD_P6x8Cha(0,5,'*'); 
                LCD_CLS_ROW(100,5);
                LCD_P6x8Num(100,5,lcd_bow_speed);
                break;
       case 6:lcd_straight_speed_ed += i;
                LCD_P6x8Cha(0,6,'*'); 
                LCD_CLS_ROW(100,6);
                LCD_P6x8Num(100,6,lcd_straight_speed_ed);
                break;

          }
     }   
}


//-------------------------------------定义输入输出端口---------------------------------------------------//

void PORT_Init(void)
{ 
    
    PORTB_PCR20 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//PTC1引脚设置为GPIO模式 上拉
    
    PORTE_PCR0 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E4引脚设置为GPIO模式
    PORTE_PCR1 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E5引脚设置为GPIO模式
    PORTE_PCR2 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E6引脚设置为GPIO模式
    PORTE_PCR3 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E7引脚设置为GPIO模式
    PORTE_PCR4 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E8引脚设置为GPIO模式
    PORTE_PCR5 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E9引脚设置为GPIO模式
    PORTE_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E10引脚设置为GPIO模式
    PORTE_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E11引脚设置为GPIO模式
    
        
        GPIOE_PDDR = 0xffffff00;  //E0~E7设置为输入口 

        GPIOB_PDDR = 0xffefffff;  //PTC1设置为输入
        
        
         PORTD_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D6引脚设置为GPIO模式   //拨码开关
         PORTD_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D7引脚设置为GPIO模式
         PORTD_PCR8 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D8引脚设置为GPIO模式
         PORTD_PCR9 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D9引脚设置为GPIO模式
         PORTD_PCR10 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D10引脚设置为GPIO模式
         PORTD_PCR11 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D11引脚设置为GPIO模式
         PORTD_PCR12 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D12引脚设置为GPIO模式
         PORTD_PCR13 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D13引脚设置为GPIO模式 
   
         GPIOD_PDDR = 0xffffc03f;       
}

//---------------------------------------------------主函数------------------------------------------------//
void main(void)
{
   uint16 i=0,j=0;
   lcd_debug = 1;
    DisableInterrupts;
    pllinit180M(); 
    
    LCD_IO_Init();
    LCD_Init(); 
    //单片机上电后，检测拨码和按键，检测完毕设置后相应的参数后，退出

    PORT_Init();              //端口初始化
    hw_FTM_init();
    UART0_Init();             //串口初始化   
    LPTMR_Init();             //脉冲计数器初始化
    EXIT_Init();   
    while(lcd_debug)
    { 
      pre_show();
      scan_boma();
      Keyscan();  //检测拨码的时间过长，所以检测拨码放在while之前，而且检测完后，将其关闭，让车子跑动
    } 
    lcd_page_num = 0;
    Delay_MS(40000000);   //起跑延迟 uint8 ch=3;
    Delay_MS(40000000);  
    OddEvenStatus = ODD_EVEN_STATUS;
    VIF = VIF_START;
    Initial();
    
    enable_irq(45);           //打开串口中断
    enable_irq(88);           //打开行中断 
    EnableInterrupts;

    
    while(1)
    {      
       if(ImageReady)                                         //图像准备好，再决策
       {   
         for(i = 0;i< ROW ; i++)
           for(j= 0;j<COLUMN;j++)
           {
             if(VideoImage2[i][j] > WHITE_BLACK_OT)
             {
                 VideoImage1[i][j] = WHITE;
             }
             else
             {
                VideoImage1[i][j] =BLACK;
             }
           }
         
         if(start_stop_en == 0 && start_stop_cs ==1)
         {
           start_stop_count++;
           if(start_stop_count > 50*stop_pit_count)  //延时6s检测起跑线300
           {
             start_stop_en = 1;
             start_stop_count = stop_pit_count;//300
           }
         }
         //用于测试赛道。
         if(test_run == 0)
         {
           car_test_run++;
           if(car_test_run >= 200)
           {
             car_test_run = 200;
             stopflag=1;
           }
         }
          Search_WhiteBase();
        if(find_whitebase_flag == 1)  //这个条件的加入，有两点好处。1 防止没有找到基准行的时候溢出， 2、当没有找到基准行的时候保持上一场的数据
        {
          Search_BlackEdge();
          Deal_BlackEdge();
          get_line_information();
          if(control_top_whiteline >=50 && control_top_whiteline <ROW-1)  //当车子处于弯道当中的时候，不进行线性相关的计算
          {
            linear_factor=get_linear_factor(bottom_whitebase,control_top_whiteline - 10,center_linear_average);
          }
          else
          {
            linear_factor = 0.5;
          }
        }
        if(start_stop_en == 1 )  //start_stop_en
        {
          check_start_stop_line();
        }
          Control();
          redraw();//刷新显示屏   
          SCI0_send_mesage(); 
          while(ImageReady);
       }
    }  
   
}
  

//-----------------------------中断函数-----------------------------//
void uart0_isr(void)          //串口中断
{    
    DisableInterrupts;   // 关总中断也可以，但在有更高级中断存在里不推荐
      uint8 ch;
     while(!(UART0_S1&UART_S1_RDRF_MASK));
      ch = UART0_D;
      if(ch == '1')     //发送的是原始图像
        send_mes = 1; 
      else if(ch == '2')  //发送图像的变化趋势
        send_mes = 2;
      else if(ch == '3')  //速度图像
        send_mes = 3;
      else if (ch >= 64 && ch <= 65)   //变档调速
      {   
          switch(ch - 64)  //变档调速
          {
          case 0:   send_mes='s';break;//stop停车
          case 1:   send_mes='p';break;
          default: speed = 100;
          }
      }

    EnableInterrupts;
}

//------------------------------------图像采集中断----------------------------------------//
void PTB_isr(void)//对于场中断20ms和行中断63us都是摄像头的固有的频率，不需要用软件去定时
{
  /*
        图像采集的行数为 27 30 33 36 39 42 45 48 51 54 57 60 63 
                         66 69 72 75 78 81 84 87 90 93 96 99 102 
                         105 108 111 114 117 120 123 126 129 132 135 138 141
                         144 147 150 153 156 159 162 165 168 171 174 177 180 
                         183 186 189 192 195 198 201 204 207 210 213 216 219 
                 
        */ 
    uint16 i;  
    
  
   PORTB_PCR22|=PORT_PCR_ISF_MASK;  //清除中断标志位
    if (VIF == VIF_START)                              //开始采样标志
      {
        LineCount++;
        if(OddEvenStatus != ODD_EVEN_STATUS)
        {
          OddEvenStatus = ODD_EVEN_STATUS;	//奇偶场标志
          VIF = VIF_WAITSAMPLE;   		//下一个状态为等待采样
          VideoImageLine = 0;
          LineCount = 0;
          ImageReady = 0; 
        }
      }
    else if (VIF == VIF_WAITSAMPLE)                 //等待采样,此时略去VIDEO_START_LINE行
      {
          LineCount++;
          if (LineCount >= VIDEO_START_LINE)
          {
              VIF = VIF_SAMPLELINE;                 //下一个状态为采样状态
          }   	
      }
    else if (VIF == VIF_SAMPLELINE)              //开始采样
      {
          LineCount++;
          if (LineCount % 3== 0)                //每隔一行采一行
          {
              for (i = 0; i < COLUMN+PIANYI ; i++)        //每行扫描COLUMN+PIANYI个点(其中PIANYI个点需要被剔除掉，因为是行消隐点)
             {
                  if (i >=PIANYI )
                   {//采集的第一个点的坐标在真实的世界里是右下角，所以在数组中存储在第一行的最后一个位置
                     VideoImage2[VideoImageLine][i-PIANYI] = (uint8)(0x000000ff & GPIOE_PDIR);//将采集到的点直接放入到VideoImage2[][]中在init array（）中放到VideoImage1[][]中做处理
                         Delay_MS(3); 
                        asm("nop");
                        asm("nop");//汇编延时
                  }
              }
             VideoImageLine++;
          }
          if (VideoImageLine == ROW)      //采集行数大于设定的行数
          {
              ImageReady = 1;           //图像准备好
              VIF = VIF_START;
          }
          
     }
    
  /* if (start_stop_en = 1 && start_stop_cs ==1 && (delay_detective && ((left_tube1 || left_tube2) && (right_tube1 || right_tube2))))        //每行中断检测一次起跑线
    {
      stopflag = 1;
    }
    //加入光电管子对起跑线的检测。
    */
    
    if (LineCount % 45 == 42)         //   7次  每隔45行控制一次  第一次控制位 43 88 133 178 223 268 313    
    {
        speed_feedback = LPTMR0_CNR;                  //读编码器的值
        LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;
        LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;                //溢出后继续计数
        
       if(stopflag == 1 && speed_down_cnt>10) 
        {
            if( speed_feedback >= 18 )
              {
                 FTM1_C0V=600;  //正转
                FTM1_C1V=0;   //反转
              }
            else if(speed_feedback <18)
              {
                FTM1_C0V=0;  //正转
                FTM1_C1V=0;   //反转
                dead_stop = 1;
              }
            
             if(dead_stop == 1)
             {
                FTM1_C0V=0;  //正转
                FTM1_C1V=0;   //反转
             }
        }
        else
        {
         if(ramp_flag == 1)
           speed_except = ramp_speed;//26
         
          speed_error = speed_except - speed_feedback;
          speed +=(int16)((speed_p * (speed_error - speed_re_error) + speed_i * speed_error)/10 );//- speed_ed
         //speed=1000;
          if(speed > max_speed)
             speed = max_speed;
          if(speed < min_speed)
             speed = min_speed;
          
          if( speed_error > 15)  //测试发现当反转较大，如700时，在减速的时候摄像头会出现拉黑的危险
          {
          //  FTM1_C0V = 800;
           // FTM1_C1V = 0;
          FTM1_C0V = 0;
          FTM1_C1V = 780;
          }
         else if(speed_error < -13)  //用于反转控制
          {
            //FTM1_C0V=0;  //正转
           // FTM1_C1V=650;   //反转750
            FTM1_C0V=950;  //正转
           FTM1_C1V=0;   //反转750
          }
          else
          {
          //  FTM1_C0V=speed;
           // FTM1_C1V = 0;
            FTM1_C0V=0;
            FTM1_C1V = speed;
            
          }
          speed_re_error=speed_error;
        }

    }


}
 
