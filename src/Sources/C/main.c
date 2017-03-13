/*����˵��  */
//ʹ������ӵľ����Ч׼ȷ��
#include "MK60N512VMD100.h " /* include peripheral declarations */
#include "includes.h"
#include <math.h>
#include"LCDDriver.h"

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,����λ��Ϊ0--31��Ч
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //�ѵ�ǰλ��1
#define BUS_CLOCK  100  //(MHZ)50 82 90 100 105 110 115//�������õ��ں�ʱ�ӵ�������ʱ��100M
#define BAUD 19200     //������
#define CORE_CLOCK 180

//--------------------------�ɼ�ͼ�����ر���-------------------------------------//
bool     OddEvenStatus;		  //��ż��״̬��־
#define  OddStatus  0
#define  EvenStatus 1
#define  ODD_EVEN_STATUS  (bool)(0x00000001 & (GPIOB_PDIR  >> 20))  //��ż�任��־  ����ptc�˿ڵĵ�1λ���ƶ�����1
#define VIF_START	0   	 //	��ʼģʽ				 
#define VIF_WAITSAMPLE	1        //   �ȴ�ģʽ
#define VIF_SAMPLELINE	2         //   ��ȥ�����е�״̬
#define VIF Videoclo_Flag         //
#define PIANYI 150   //ʵ�ʲɼ�����ΪCOLUMN + PIANYI��PIANYIΪÿ��������  ����ֵ������ֵ�������м���
#define VIDEO_START_LINE  27	//ͼ��ɼ���ʼ��
bool ImageReady;               //ͼ��׼���ñ�־
uint8 Videoclo_Flag, VideoImageLine;   //�ɼ�״̬��־λ�����ж�ʵ�ʲɼ�����������
uint16 LineCount;                       //���жϲɼ�����������   �������һ����Ҫ����Ϊuint16  �Լ���ǰ����Ϊuint8  ����ʹ����ĥ�˺ü���


//-------------------------����ͼ�����ر���-----------------------------------------//
#define ROW 65	                 //�ɼ�����
#define COLUMN	159 		//ÿ�е���
#define MID  79                 //������ 
uint8 VideoImage1[ROW][COLUMN] =       //ԭʼͼ������[0][0]�����½�
{
   0
};
uint8 VideoImage2[ROW][COLUMN] =       //ԭʼͼ������[0][0]�����½�
{
   0
};

uint8 left_black[ROW]=                 //������ߵĲɼ�����
{
  0
};
uint8 right_black[ROW]=                //�ұ����ߵĲɼ�����
{
  0
};
uint8 center_white[ROW]=              //����������ģ����ߵ�����
{
  0
};


//-------------------------------------�����ߺ���----------------------------------//
#define MIN_WHITEBASE_POINT 30                    //���������׵������Ϊ��׼��Ҫ��
#define WHITE_TOP_WHITELINE_POINT 20                  //���ߵĺ��ߵĿ��С�����ֵ���ж�Ϊ�����Ч
#define CENTER_LOST_POINT 20
uint8 current_deal_line=0;     //��ǰ�������
uint8 deal_start_line = 0;                //��� ֵʱ���ƴ������ʼ��һ�㶨��Ϊ��׼�� + 4
uint8 hang_search_start = 0;             //����ÿ��ɨ��Ŀ�ʼ�Ǵ��ĸ��㿪ʼ��
uint8 whitepoint_start=0;                //�������Ұ׵㿪ʼ��
uint8 whitepoint_end=0;                 //�������Ұ׵������
uint8 whitebase_searchstart = MID;
uint8 left_whitebase_searchstart = 0;
uint8 right_whitebase_searchstart = 0; 
uint8 re_whitepoint_start = 20;  // ������ʱ����һ��Ҫ���������������ң����������Ҳ���������Σ��
uint8 re_whitepoint_end=145;   
uint8 center_lost_hang = 0;
uint8 refer_road_width[ROW] ={127,126,125,124,123,122,120,119,118,117,
                              116,115,114,113,112,110,108,106,104,112,
                              100,98,97,95,93,92,90,89,88,87,
                              86,85,83,81,80,79,77,75,73,70,
                              69,68,66,64,62,60,58,56,53,51,
                              49,47,45,43,41,39,38,36,35,33,
                               32,31,30,28,26};//
uint8 OT=36;                                     //�ж�Ϊ�Ҷ�ֵ�������ص���С�Ҷȵ�����ֵ
uint8 BASE_OT = 130;
uint8 WHITE_BLACK_OT = 145;           //���ж�ֵ���ķֽ�ֵ
#define WHITE 255
#define BLACK 10
uint8 top_whiteline = ROW-1;                          //ͼ������
uint8 left_top_whiteline = ROW-1;
uint8 right_top_whiteline = ROW-1;

uint8 bottom_whitebase = 0;                       //ͼ��Ļ�׼�� 
bool find_whitebase_flag = 0;  //��׼�еı�־λ

uint8 re_white_refer = MID;  //�������Ϊÿ��������׼�еĿ�ʼ�ĵ�  ���ʼ��ʱ����Ϊ Ĭ��ΪMID
uint8 white_refer = 0;                            //��׼���ϵ��������е�
uint8 Row_state[ROW] =
{
  0
};
//--------------------------------------�����������ز���-----------------------//
uint8 S_right = 0;//���ҹյļ���
uint8 S_left =0 ; //����ռ���
uint8 S_straight = 0;
uint8 direction = 0; //4�ǳ�ʼ����ֵ
uint8 re_direction = 0;//��¼��һ�εĵ��е�ʱ���޷��жϳ����������͵�ʱ������һ�ε�״̬
uint32 center_average = 0;
uint16 center_error_average = 0;  
uint32 center_linear_average = 0;

#define RAMP_WIDTH  90                  //
uint16 ramp_delay_time = 25;
uint16 ramp_time = 0;                //�����µ���೤ʱ�����¿��������߼��
uint16 ramp_dis_time = 0;       //��ֹ���µ����ж���ʱ
uint16 ramp_speed = 80;                    //�µ�����ֵ70
bool ramp_flag = 0;                          //�����µ���־,��Ҫ���ڿ���
bool ramp_dis_flag = 0;                     //��Ҫ�Ƿ�ֹ��������
 
 /*���Ե�ʱ�򣬼�⺯���оֲ�����
float XX_square_sum=0;   //X��ƽ����
float YY_square_sum=0;   //Y��ƽ����
float XYmulti_sum=0;      //XY�˻�֮��*/
float linear_factor = 0;

//-----���ڹ���ѡ��----//
uint8 send_mes=0;              //������λ����������������ѡ��ͬ�Ĵ��ڹ���

//------------------------------------������ƺ����Ĳ���-----------------------------------//
#define SPEEDCHOICENUM 6  //����6��
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

uint16 speed_feedback = 0;               //�������ķ���ֵ                  //
int16 speed_re_error = 0;
bool stopflag = 0;//�ٶȷ���
uint16 speed_down_cnt = 0;       //��⵽�����ߺ���ʱʮ����Ȼ�����
int16 speed_error = 0;
uint8 speed_p = 80;//44
uint8 speed_i =95;//65
int16 speed = 0;

uint16 lcd_straight_speed = 0;  //������lcd�����ٶȵ�������Сֵ
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
uint8 speed_select = 0;//��Ϊ1��ʱ��ѡ������ܣ����ڲ����ܵ����㷨 ��ͨ���������
uint8 full_speed_line = 0;
//-----------------------------------������ƺ����ı���---------------------------------//

int16 angle=1460;
int16 re_angle= 1460;
uint16 mid_angle=1460; //  �������ҹգ�˵��С�ڰ���ֵ     �������ܵ��ȶ������ʱ����ȥһ��ֵ�ɻָ����м�

uint16 control_top_whiteline = 0;//re_control_top_whiteline top_error_servo_p
uint16 re_control_top_whiteline = ROW - 1;
uint16 danger_count = 0;  //��¼Σ�յĵ���
bool danger_flag = 1;  //����ĳ�ʼֵΪ1.�����˿��������������Ϊ0��
int16 p_error=0;
int16 p_re_error = 0;
int16 ref_his_error[5] ={0,0,0,0,0};//�������������¼��ʷ��ֵ��Ȼ����뵱ǰ��error

//��������pd��˵������һ��pd�𵽵������Ǵֵ����ڶ���pd��΢���������ֵ��ڵڶ���pd,û�кܴ�ĸ��Ƶ�ʱ������һ��ȥ����һ��pd

uint16 error_servo_p=0;
int16 lcd_error_servo_p = 3; //4  2   4
uint16 error_servo_d=0;
int16 lcd_error_servo_d = 57;  //83  40  65
uint16 top_error_servo_p = 0;

uint16 error_servo_ref_p = 0;
int16 lcd_ref_p = 6;  //9      7��25ʹ��ֱ�ߺ�ֱ   12  6          6
uint16 error_servo_ref_d = 0;
int16 lcd_ref_d = 40;  // 25                         60   23

uint8 get_p_errorline = 0;
 
int16 refer_error =0;
int16 re_refer_error = 0;

//���ܼ��ؼ����ʱ
/*#define right_tube1   (bool)(GPIOB_PDIR >> 4 & 0x00000001)             //�ֱ��ȡ����ܵ�״̬
#define right_tube2   (bool)((GPIOB_PDIR >> 5) & 0x00000001)
#define left_tube1   (bool)((GPIOB_PDIR >> 6) & 0x00000001)
#define left_tube2   (bool)((GPIOB_PDIR >> 7) & 0x00000001)*/
uint32 start_stop_count = 0;  //�����߼�����
uint32 stop_pit_count = 6;
bool start_stop_en = 0;   //�����߼��ʹ��
bool start_stop_cs =0;   //�����߼���Ƭѡ�ź�   ��Ϊ1��ʱ��ѡ�м��������

uint16 car_test_run = 0; 
bool test_run = 1;  
//-------------------------------�����Ķ���-------------------------------------------//
#define LCD_ROW 7                      //СҺ������ʵ�ʵ�����Ϊ8��
bool change_page=0;             //��ҳ
bool se_sub_NUM=0;             //��ҳ
bool up_line=0;              //����   ����
bool down_line=0;            //����   ����
bool add_NUM=0;              //������ֵ  ��
bool sub_NUM=0;              //������ֵ  �� 

//--------------------------------����Ķ������--------------------------------------//
bool lcd_debug = 1;            //lcd�ĵ���ѡ��
bool redraw_control=0;         //ˢ���Ŀ���λ

//------------------------------------LCD��������-------------------------------------//
void pre_show(void);        //��һ���Ԥ��ʾ
void redraw(void);          //ˢƵĻ
void Keyscan(void);          //ɨ�貦��
void LCD_change_value(unsigned char page,unsigned char m,int i);//������ֵ
void Delay_MS(uint32 ms);       //��ʱ����
uint8 lcd_page_num=1;        //Һ������ҳ��
uint8 lcd_line_num=0;        //Һ����������


//---------------------------�����ʼ��--------------------------//
void Initial(void)
{
  int16 i;
     for(i = 0;i < ROW;i++)
       {
         left_black[i] = 0;
         right_black[i] = 0;
         center_white[i] = 0;
         Row_state[i] = 3; //3����������߶�û�г��ֶ���
       }
       start_stop_count = 0;
       ramp_dis_flag = 0;
       ramp_flag = 0;
}

//--------------------�͹��������������ʼ��-----------------------//
void LPTMR_Init()   //PTC5  LPT0_ALT2
{
   SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //����C�˿�ʱ��
   PORTC_PCR5 &= ~PORT_PCR_MUX_MASK;
   PORTC_PCR5 |= PORT_PCR_MUX(4);  //PTC5����ΪLPTMRģʽ
   PORTC_PCR5 |= PORT_PCR_PE_MASK; //
   PORTC_PCR5 &= ~PORT_PCR_PS_MASK; //����

   SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;  //ʹ��LPTMģ��ʱ��
   LPTMR0_CSR &= ~LPTMR_CSR_TPS_MASK;
   LPTMR0_CSR |= LPTMR_CSR_TPS(2)| LPTMR_CSR_TMS_MASK; //  ALT2  ����ģʽ
   LPTMR0_CSR |= LPTMR_CSR_TFC_MASK;  //�����λ 65535
   LPTMR0_CSR &= ~LPTMR_CSR_TPP_MASK;  //�����ؼ���

   LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK; //  ���Է�Ƶ���˲�
   LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;  //����LPTģ��
}


//---------------------------���жϲ�׽�˿ڳ�ʼ��-------------------//
void EXIT_Init(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  //����C�˿�ʱ��
    PORTC_PCR3 =PORT_PCR_MUX(1);  //GPIO
    GPIOC_PDDR &= ~GPIO_PIN(3);   //����
    PORTC_PCR3 |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; //��������;
    PORTC_PCR3 |= PORT_PCR_IRQC(9); //9Ϊ�����ش����ⲿ�ж� 10Ϊ�½��ش�
}



//----------------------------���ڳ�ʼ��-----------------------------//
void UART0_Init(void)    //PTB16 RXD    PTB17 TXD
{
    uint32 uartclk_khz = CORE_CLOCK*10 * BUS_CLOCK;//ʱ��180MHz    //��ʱ����
    uint32 baud = BAUD;
    uint16 sbr,brfa;
    
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //����B��ʱ��
    PORTB_PCR16|=PORT_PCR_MUX(3);//��PTB16��������Ϊģʽ3����UART0_RX
    PORTB_PCR17|=PORT_PCR_MUX(3);//��PTB177��������Ϊģʽ3����UART0_TX
    SIM_SCGC4|=SIM_SCGC4_UART0_MASK;//����UART0ʱ��
    sbr = (uint16)((uartclk_khz*1000)/(baud*16));//���㲢���ò�����
    
    UART0_BDH = (uint8)((sbr&0x1F00)>>8);//��������19200д����Ӧ�ļĴ���Ȼ�����ʹ�ܣ�ʹ�乤����ǰ���buadֻ��һ�����֣�������ļ����ǽ�19200д������Ĵ�����Ȼ�����ʹ��
    UART0_BDL=(uint8)(sbr&0x00FF);
    brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
    UART0_C4 = (uint8)(brfa & 0x001F);
    UART0_C2 |=(UART_C2_TE_MASK|UART_C2_RE_MASK);
    UART0_C1 = 0;	
    UART0_C2 |= UART_C2_RIE_MASK;   //��UART0�����ж�
}


//-------------------------------------ftm��ʼ��-----------------------------------------//
void hw_FTM_init(void)
{      	
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;//����C�˿�ʱ��
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //����A�˿�ʱ��
  
    
    PORTC_PCR4 &= ~PORT_PCR_MUX_MASK; //����
    PORTA_PCR12 &= ~PORT_PCR_MUX_MASK;
    PORTA_PCR13 &= ~PORT_PCR_MUX_MASK;
    PORTA_PCR10 &= ~PORT_PCR_MUX_MASK;
    PORTC_PCR3 &= ~PORT_PCR_MUX_MASK;
    
    PORTC_PCR4 = PORT_PCR_MUX(4); //FTM is alt4 function for this pin
    PORTA_PCR10 = PORT_PCR_MUX(3);
    PORTA_PCR12 = PORT_PCR_MUX(3);//FTM is alt3 function for this pin 
    PORTA_PCR13 = PORT_PCR_MUX(3);
      PORTC_PCR3 = PORT_PCR_MUX(3);
  
    SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;     //ʹ��FTM0ʱ��
    SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;    //����FTM1ģ��ʱ��
    SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;    //����FTM2ģ��ʱ��
    
    FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����FTM0_CH3 
    FTM1_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����ģʽ CH0
    FTM1_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //FTM1_CH1
    FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����FTM2_CH0 
    FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //����FTM2_CH0 
    
    
    FTM0_CNT=0;//���ü�����ֵΪ0
    FTM1_CNT=0;
    FTM2_CNT=0;
    
    //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 

    //���� the pulse width(duty cycle) is determined by (CnV - CNTIN).
    
     FTM1_MOD =1000;              //����PWMƵ��Ϊ10K=100 000 000 /2^2/2500  ���100 000 000 �ǵ��߽��ϵͳƵ��  2500�������õ�FTM1_MOD ֵ
     FTM0_MOD =2; //3145   300hz sd5�����Ƶ��ǡ�����ᵼ�¶�����������Ҫȥ��������������ԭ������������ѹ���⣬Ƶ�����⡣��һ����300hz����  //18750  50hz
     FTM2_MOD =2; 
    
    FTM0_CNTIN=0;//���ó�ʼ������ֵ
    FTM1_CNTIN=0;
    FTM2_CNTIN=0;
      
    FTM0_C3V=mid_angle;//1400;//1490 1.5ms//2ms  1986//2.5ms  2483 //1ms 993// 0.5ms  497  //1614

    FTM1_C0V=0;
    FTM1_C1V=0;
    
    FTM2_C0V=1;
    
    FTM0_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);
    FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(2); //����ʱ�Ӻͷ�Ƶ
    FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(1);
}

//----------------------���໷Ƶ��Ϊ50/15*54=180M���Ժ���-------------------------------//
void pllinit180M(void)
{
	uint32_t temp_reg;
        //ʹ��IO�˿�ʱ��    
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                              | SIM_SCGC5_PORTB_MASK
                              | SIM_SCGC5_PORTC_MASK
                              | SIM_SCGC5_PORTD_MASK
                              | SIM_SCGC5_PORTE_MASK );
    //���ﴦ��Ĭ�ϵ�FEIģʽ
    //�����ƶ���FBEģʽ
    MCG_C2 = 0;  
    //MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
    //��ʼ��������ͷ�����״̬��������GPIO
    SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
    LLWU_CS |= LLWU_CS_ACKISO_MASK;
    
    //ѡ���ⲿ���񣬲ο���Ƶ������IREFS�������ⲿ����
    //011 If RANGE = 0, Divide Factor is 8; for all other RANGE values, Divide Factor is 256.
    MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
    
    //�ȴ������ȶ�	    
    //while (!(MCG_S & MCG_S_OSCINIT_MASK)){}              //�ȴ����໷��ʼ������
    while (MCG_S & MCG_S_IREFST_MASK){}                  //�ȴ�ʱ���л����ⲿ�ο�ʱ��
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}
    
    //����FBEģʽ,
    //0x18==25��Ƶ=2M,
    //0x08==15��Ƶ=3.333M 
    //0x09==16��Ƶ=3.125M,
    //0x10==17��Ƶ=2.94M 
    //0x11==18��Ƶ=2.7778M 
    //0x12==19��Ƶ=2.63M,
    //0x13==20��Ƶ=2.5M    
    MCG_C5 = MCG_C5_PRDIV(0x0e);                
    
    //ȷ��MCG_C6���ڸ�λ״̬����ֹLOLIE��PLL����ʱ�ӿ���������PLL VCO��Ƶ��
    MCG_C6 = 0x0;
    
    //����FMC_PFAPR��ǰ��ֵ
    temp_reg = FMC_PFAPR;
    
    //ͨ��M&PFD��λM0PFD����ֹԤȡ����
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;    
    ///����ϵͳ��Ƶ��
    //MCG=PLL, core = MCG, bus = MCG/3, FlexBus = MCG/3, Flash clock= MCG/8
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) 
                 | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);       
    
    //���´�FMC_PFAPR��ԭʼֵ
    FMC_PFAPR = temp_reg; 
    
    //����VCO��Ƶ����ʹ��PLLΪ100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(30);  //VDIV = 31 (x54)
                                                  //VDIV = 26 (x50)
    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set    
    while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set    
    
    //����PBEģʽ    
    //ͨ������CLKSλ������PEEģʽ
    // CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
    MCG_C1 &= ~MCG_C1_CLKS_MASK;
    
    //�ȴ�ʱ��״̬λ����
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};
    //SIM_CLKDIV2 |= SIM_CLKDIV2_USBDIV(1);  
    
    //���ø���ʱ��Ϊ�ں�ʱ��
    SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;	
    //��PTA6������ʹ��TRACE_CLKOU����
    PORTA_PCR6 = ( PORT_PCR_MUX(0x7));  
    //ʹ��FlexBusģ��ʱ��
    SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
    //��PTA6������ʹ��FB_CLKOUT����
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

//-----------------------------------ɨ����߻�׼��---------------------------------//
/*
1�����������Ŀ����ͼ������ռ�ı����ϴ󣬹ʿ���Ϊ���е��λ�ã�79������һ�����������У�
�����ǳ����ܳ���������������Ҫ��������ƫ�������������
*/
void Search_WhiteBase(void)   //��ͼ��ײ��м俪ʼ������ɨ����߻�׼
{ 
  uint8 i = 0,j = 0 ;//����ʮ��λ���з��ű���   i�����б���  j�����б���
  uint8 base_sum = 0; 
  current_deal_line=0;                //��¼��������׼�е�ʱ��ĵ�ǰ������� 
  bottom_whitebase = 0;//��׼�и���ֵ  int
  find_whitebase_flag = 0;               //�Ƿ��ְ��߻�׼��־

  //////////////////////////////�˲���ʼ///////////////////////////////////  
//���ȶ�����ͼ������˲������õķ�������ֵ�˲�
  for(i = 0;i < ROW / 5;i++)                 //ֻ�Ƕ�ͼ��ǰ���н����˲���ԭ����Զ�����˲����ܻ���ܵ�����Ϣ�˳��������������ǰ13���˲�
    for(j = 1;j< COLUMN-1;j++)
    {
        base_sum = (VideoImage1[i][j-1] + VideoImage1[i][j+1])/ 2;
        if( f_abs16( base_sum - VideoImage1[i][j]) > OT)
           VideoImage1[i][j] = base_sum;
    }  //�˲����ܴ���һ����������ǿ��ܰ�Զ���ı������˳���������ֻ�ǶԽ��˽����˲�
  
  
  /*////////////////////////��ͼ���ϵ��������˳�//////////////   �޷��˲������һ����������������ؼ��������⣬����̫С
  for(i = 0;i < ROW ;i++)                 //ͼ����ͻȻ�ĳ����˺ܶ����㣬���������Ϊ�˽�ͼ���ϵ���Щ����˳���������֤Ч���ܺ�
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
*////////////////////////////�˲�����////////////////////////////////
  
  /*����������׼������Ҫ�ľ��ǽ�������Ŀ�ʼ������⣬������ҵ��ˣ�����������ͺý����*/
  if(VideoImage1[0][re_white_refer] > BASE_OT && VideoImage1[0][re_white_refer-1] >BASE_OT && VideoImage1[0][re_white_refer+1]>BASE_OT)
  {
    whitebase_searchstart = re_white_refer;
  }
  else
  {
    j = MID-1;//��MID��ʼ������׼�еĿ�ʼ��
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
  
  
  
  while( find_whitebase_flag ==0 && current_deal_line < ROW - 1) //��׼�е�������Χ��0��ROW-1 
  {
      //ÿ�еĴ�������
      whitepoint_start = whitebase_searchstart;   //uint8 
      //������ߵĻ�׼����/////////////////////////////
       j = whitebase_searchstart;   //�е�ʱ�����ǰһ�е��е�����һ�е�ͼ�������
       //ÿ�δ���һ���Ļ�׼���е㿪ʼ������������׼��
       while(j >= 3  )//һ��Ϊ��ʹ�������ظ��ӵ����ԣ����ø����ж�
      {
        
          if( whitepoint_start != whitebase_searchstart && f_abs16(j-re_whitepoint_start) > f_abs16(whitepoint_start -re_whitepoint_start) )
          {
            break;
          }
          else if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] > OT 
             && VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-3] > OT
               )
          {//����һ��if�������ʱ��˵����ʱ��j����һ���ĵ�ľ���һ��С�� f_abs16(whitepoint_start -re_whitepoint_start) 
            //��������ֻҪ���������䣬�͸���׼�е���ʼ�㸳ֵ
               whitepoint_start = j;
          }
          j--;
      }
      
      if( j == 2  && whitepoint_start == whitebase_searchstart)  //����߽��ˣ����ǻ�û�ж�whitepoint_start��ֵ����˵��û���ҵ���׼�Ŀ�ʼ��
      {
        if(VideoImage1[current_deal_line][j] - VideoImage1[current_deal_line][j-2] > OT)
        {
           whitepoint_start = 2; //�����˱߽�
        }
        else if( VideoImage1[current_deal_line][j-1] - VideoImage1[current_deal_line][j-2] > OT)
        {
           whitepoint_start = 1; //�����˱߽�
        }
        else
        { 
           whitepoint_start = 0;
        }
      }
      //�����������///////////////////////
      
      //�ұ�������ʼ/////////////////////
      whitepoint_end = whitebase_searchstart;    //uint8
      j = whitebase_searchstart;   //ÿ�δ���һ����white_refer������������׼��
      while( j <= COLUMN-4 )//һ��Ϊ��ʹ�������ظ��ӵ����ԣ����ø����ж�
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
      //�ұ���������///////////////////////
      //���ұ����ߵ����������õ��Ǹ��ٵ�����������Ŀ����ֻ���ҵ�һ�пɿ��Ļ�׼�е���Ϣ
      
      
      //������һ���Ƿ����Ҫ��ʼ�ռ�¼������Ϣ

        left_black[current_deal_line] = whitepoint_start;   //��¼�����λ�� (��Ϊ0���ܿ���˵����ߺ��߶�ʧ��������ƫ��)
        right_black[current_deal_line] = whitepoint_end;    //��¼�Һ���λ��(��ΪCOLUMN���ܿ���˵���ұߺ��߶�ʧ,������ƫ��)
        /*���ڵ�һ�е�״̬���ж��������뷨����һ�����۵�һ����ʲô״̬ʼ����Ϊû�ж��� �ڶ�����������صĵ���Ϊ����
        ����������Ҫ�����������ֵ����һ�������������������ｫ��׼���ϵ�״̬�趨Ϊû�ж��㣨��ʹ��ʱ�򵽴��˱߽磩��
        ����������ĸ������������ƾ�û���ˣ�
        */
        if(left_black[current_deal_line] == 0 && right_black[current_deal_line] < COLUMN - 1 &&  (right_black[current_deal_line] - left_black[current_deal_line]) > 155)
        {  //��ʾ��ߵ���߽� ����
          
          left_black[current_deal_line] = re_whitepoint_start;
          center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2; 
        }
        else if(right_black[current_deal_line] == COLUMN - 1 && left_black[current_deal_line] > 0 &&  (right_black[current_deal_line] - left_black[current_deal_line]) > 155)
        { //��ʾ�ұߵ���߽� ����
           right_black[current_deal_line] = re_whitepoint_end;
         center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2;  //��¼���ĵ�,����MID˵������ƫ�󣬷�֮��˵������ƫ��
        }
        else if(right_black[current_deal_line] == COLUMN - 1 && left_black[current_deal_line] == 0 )
        {//��ʾ���ߵ���߽� ����
          left_black[current_deal_line] = re_whitepoint_start;
          right_black[current_deal_line] = re_whitepoint_end;
          center_white[current_deal_line] = re_white_refer;
        }
        //˵������ǰ���е���Ϣһ�㲻������Ĭ��Ϊ���ұ��ض��ҵ��˵ĵ� 
        else
        {  //��ʾ���߶�û�ж���
          center_white[current_deal_line] = (left_black[current_deal_line] +  right_black[current_deal_line]) / 2;  //��¼���ĵ�,����MID˵������ƫ�󣬷�֮��˵������ƫ��
        }
        //��������µõ�ͼ�������Ŀ�ʼ����������ֵ
      whitepoint_start = left_black[current_deal_line];
      whitepoint_end =  right_black[current_deal_line];
      white_refer = center_white[current_deal_line];
      
        //����һ�������Ŀ�ȵ�����
       if(whitepoint_end - whitepoint_start > MIN_WHITEBASE_POINT ) //���ֵ����Ϊ20 
        {
          find_whitebase_flag = 1;
          re_white_refer = white_refer;  //���汾��ͼ�����Ϣ
          re_whitepoint_start = whitepoint_start;
          re_whitepoint_end  = whitepoint_end ;
          bottom_whitebase = current_deal_line;//��¼��׼��
          Row_state[bottom_whitebase] = 3; //�е�״̬��־λ
        }
        else
        {
          find_whitebase_flag = 0;
          current_deal_line++;
        }
        
  }//whileѭ���Ľ���
  if(bottom_whitebase > 0)
  {
     for( i = 0 ; i < bottom_whitebase ;i++)
     {//��֮ǰ���н��б��
        center_white[i] = MID;  
        left_black[i] = MID - 2;   
        right_black[i] = MID + 2;  
     }
  }
}//

//------------------------�ɻ�׼�߶��������ߺ���Ϊ��׼���ҳ�������Ե-----------------------// 
/*�������Ĺ��ܶ���Ϊ���ߣ�Ϊ�˴�����ĳЩ�ϵ������ܼ�����ǰ���ҵ������ߣ�
ֻ�Ƕ��ڱ����߽��г������鹹�������Ķ������������ߵĴ����鹹����Ҫ����һ���������*/
void Search_BlackEdge(void)     
{   
  int16 i = 0,j = 0,n = 0, k = 0;
  int16 un_lost_hang = bottom_whitebase;//�������������������ټ�¼�����һ�е�û�ж�����У��Ա��ڶ���һ�е�״̬����׼ȷ���ж�.��ʼֵΪbottom_whitebase��Ϊ��bottom_whitebase��ʼ���ж�Ϊû�ж���
  deal_start_line = bottom_whitebase + 1;  
  top_whiteline = ROW -1;
  hang_search_start = white_refer;  //�ӻ�׼�е��е����ɨ�� 
   
  for(i = deal_start_line ; i < ROW ;i++)//��״̬��־���г�ʼ��
    {
      Row_state[i] = 3;
    }
  
  for(i = deal_start_line ;i < ROW;i++) 
  {
    //////////////////���ҵ�������ʼ///////////////////////
    //�������
    j = hang_search_start;
    left_black[i] = hang_search_start;
    while(j >= 2)
    {     
      
      if(VideoImage1[i][j] - VideoImage1[i][j-2] > OT
         && f_abs16(VideoImage1[i][j]-VideoImage1[i][j+1]) < OT && f_abs16(VideoImage1[i][j+1]-VideoImage1[i][j+2]) < OT && VideoImage1[i][j+2]-VideoImage1[i][j+3] < OT)  //�˳��������
        {
          if(f_abs16(j - left_black[i-1]) < f_abs16(left_black[i] - left_black[i-1]))//�˳�����
           left_black[i] = j;
        } 
       if(left_black[i] != hang_search_start && (f_abs16(j - left_black[i-1]) > f_abs16(left_black[i] - left_black[i-1])
                                                 || (f_abs16(left_black[i] - left_black[i-1])  < 5 && j<=left_black[i-1] )))
       {
        break;
       }//���ټ������������������һ������㣬��ֹͣ
     
         //��ǰһ��״̬�Ƕϵ��״̬ʱ�����ʱ�����ڲ��������������ص�ʱ����Ͳ���������������û���ѵ���
          //���ٵ����������߽�������
       if(Row_state[i-1] == 0 || Row_state[i-1] == 2)
          {
            if( j <  left_black[i - 1]  && left_black[i] != hang_search_start) //��ǰһ��Ϊ�ϵ�״̬ʱ���������˵�֮�����������������
             {
              break; 
            }
          }
          j--;
    }      //����������ߵ�while����
    //��������ߵĳ����ж�
    if(j == 1 && left_black[i] == hang_search_start)      //����߽��ˣ����Ǳ�����û�иı�ʱ����������Χ��û���ҵ�����㣬����Ϊ��ͼ����Ȼ����
    {
      if(VideoImage1[i][j] - VideoImage1[i][j-1] > OT)
         left_black[i] = 1;
      else
        left_black[i] = 0;
    }

  
    //�ұ�����
    j = hang_search_start;
    right_black[i] = hang_search_start;
    
    while( j <=COLUMN-3 )
    { 
      if( VideoImage1[i][j] - VideoImage1[i][j+2] > OT 
         && f_abs16(VideoImage1[i][j]-VideoImage1[i][j-1]) < OT && f_abs16(VideoImage1[i][j-1]-VideoImage1[i][j-2]) < OT && VideoImage1[i][j-2]-VideoImage1[i][j-3] < OT)  //�˳��������
        {
          if(f_abs16(j-right_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
          {
                right_black[i] = j ;
          }
        }
      if(right_black[i] != hang_search_start &&( f_abs16(j-right_black[i-1]) > f_abs16(right_black[i] - right_black[i-1])
         ||(f_abs16(right_black[i] - right_black[i-1])<5 && j== right_black[i-1] ) ))//�ڸ����������˵㣬ֻҪ������ǰһ�е���λ�ã���ֹͣ
      {
        break;
      }
      if(Row_state[i-1] == 1 || Row_state[i-1] == 2)
          {
            if( j > right_black[i - 1]  && right_black[i] != hang_search_start) //��ǰһ��Ϊ�ϵ�״̬ʱ���������˵�֮�����������������
             {
              break;
            }
            //�����������ߴ�����
            //if()
          }
        j++;
    }    //�ұߵ�while��������
  
    if(j == COLUMN-2 && right_black[i] == hang_search_start)
    {
      if( VideoImage1[i][j] - VideoImage1[i][j+1] > OT)
         right_black[i] = COLUMN - 2 ;
      else
         right_black[i] = COLUMN - 1 ;
    }
    ///////////////////////���ҵ���������//////////////////////////
    
    
    //  /////////////������״̬��ǿ�ʼ////////////////////////////////
  if(i >= deal_start_line)//ֻ�ǶԴ��ڿ�������ı߽���д���
    {    
      //��ͼ��ı��ص����˱߽��ʱ���ж�Ϊ����      ---------------------����ĵ�һ���ж�
      if((left_black[i] <= 1  || left_black[i] >= COLUMN-2 ) && right_black[i] >= 1 && right_black[i] <= COLUMN-2)
      {
           Row_state[i] =0;//��߶���
      }
      else if((left_black[i] >= 1 && left_black[i] <= COLUMN-2 ) && (right_black[i] <= 1 || right_black[i] >= COLUMN-2))
      {
           Row_state[i] = 1;//�ұ߶���
      } 
      else if((left_black[i] <= 1 || left_black[i] >= COLUMN-2 ) && (right_black[i] <= 1 || right_black[i] >= COLUMN-2))
      {
           Row_state[i] = 2;//���߶��߶���
      }
      else
      {
           Row_state[i] = 3;//���߶���û�ж���
      }
      
        //���������ߵ�״̬�����ж�//---------------------------����ĵڶ����ж�
      //ע��������жϱ���Ҫ���������һ����������״̬�������ǰһ�е�״̬��ǰһ�е�״̬�Ĳ�ͬ��Ҫ������ͬ�Ĵ�����
      if((right_black[i] - left_black[i])-(right_black[i-1] - left_black[i-1])> 8)//  ����3�Ļ����ܳ�������//������þ���ֵ������Ϊ�˷�ֹ���
      {
         if(( f_abs16(left_black[i] - left_black[i-1]) < f_abs16(right_black[i] - right_black[i-1]))
            && f_abs16(left_black[i] - left_black[i-1]) <= 4)//��ߵ�ͻ��С���ұߵ�  ˵���ұߵĵ㷢����ͻ��
         {
           if(Row_state[i] == 0 ||Row_state[i] == 2)//���ڵ�һ�ν���һ���ж�
              Row_state[i] = 2;
           else //if(Row_state[i] == 1 ||Row_state[i] == 3)
           {
             Row_state[i] = 1;//1��ʾ����ֻ���ұ߶���
           }
         } 
        else if(( f_abs16(left_black[i] - left_black[i-1]) > f_abs16(right_black[i] - right_black[i-1]))
            && f_abs16(right_black[i] - right_black[i-1]) <= 4)//��ߵ�ͻ��С���ұߵ�  ˵���ұߵĵ㷢����ͻ��
         {
           if(Row_state[i] == 1||Row_state[i] == 2)//���ڵ�һ�ν���һ���ж�
              Row_state[i] = 2;//0��ʾ������߶���,���ұ�û�ж���
           else
              Row_state[i] = 0;//0��ʾ����ֻ����߶���
         }
         else 
         {
           Row_state[i] = 2;//2��ʾ�������߶�����
         }
      }
      else 
      {
        if(Row_state[i-1] == 0)//��߶���
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
        
       else if(Row_state[i-1] == 1)//��߶���
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
      
    //��¼����Ķ�û�ж������
      if( Row_state[i] == 3)
      {
        un_lost_hang = i;
      }
      ////////////���ұ��ر�ǽ���/////////////////////////
      
      //ǰ�������������״̬���жϣ������������������
      if(Row_state[i] == 0)  //��߶���
      {
        if(right_black[i]- (right_black[i-1] - left_black[i-1]) <= 0 )//�޷�
          left_black[i]=0;
        else
        left_black[i] = right_black[i] - (right_black[i-1] - left_black[i-1]);//����1������������ͼ���ڿ���ڼ�С��ԭ��
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
    //////////////////���ҵĴ������///////////////////////////// 
    
    ///�������Ч�е��ж�/////////////�ж�һ//////////////////


    if( i> 20 &&i<=top_whiteline && right_black[i] -  left_black[i] < 3*(ROW-i)/5+ WHITE_TOP_WHITELINE_POINT 
       && (right_black[i-1] -  left_black[i-1]) <  3*(ROW-i)/5 + WHITE_TOP_WHITELINE_POINT 
       ) //ֻ�ж�һ��&& top_whiteline >= ROW-1
    { 
     // if(i<ROW-1) while(1){}
      top_whiteline = i;
    }
    /////////////////////�ж϶�////////////////////////
    center_white[i] = (right_black[i] + left_black[i])/2; 
    if(i>10 && i < ROW-1 &&top_whiteline >= ROW-1 &&( VideoImage2[i-1][center_white[i]] - VideoImage2[i+1][center_white[i]] > OT - 10) 
       &&  VideoImage2[i-1][center_white[i]-1] - VideoImage2[i+1][center_white[i]-1] > OT - 10
              &&  VideoImage2[i-1][center_white[i]+1] - VideoImage2[i + 1][center_white[i]+1] > OT - 10 )  //����е��ж���ԭʼͼ��
    {
      //�����ַ�ʽ��һ���׶˾��ǣ�ͼ�����һ��ͻ�䣬�޷���ʵ�ķ�Ӧ�������ر���60�Ⱥ�50�������΢С���������侭���޲�
       top_whiteline = i-1;
      for( n = top_whiteline; n >  top_whiteline - 7;n--)
      {
        if(left_black[n] <= 1)
        {
          for( k = n; k <= top_whiteline ; k++)
          {
             left_black[k] = 0;
             //���й滮����е�״̬��Ҫ���µĶ���
           if(Row_state[k] == 1 ||Row_state[k] == 2)
            {
              Row_state[k] = 2;
            }
            if(Row_state[k] == 3)
            {
              Row_state[k] = 0;
            }
          }
        //  break;  //���ﻹ������break����Ϊ�е�ʱ����ܻ���һ�����������ˡ��� 0 1 0 0 0
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

  }//forѭ���Ľ���
  
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

//------------------------ͨ���ҳ����������ԣ��������߽��д�����鹹����ϳ�������----------------------//
/*�����������������ӣ����Ҹ��������Լ��ģ�Ȼ��������ߣ�����������״̬��־���ٽ���һ�����
 0  ��ʾ������߶ϵ�
 1  ��ʾ�ұ����߶ϵ�
 2  ��ʾ���߶����� 
����߽����ֱ�ӵ�������
�������ߺ�ʮ�ֵ�·�Ĵ���ֻҪ��֤��������ˣ���1����֤���ҵ����ܵ��ϵĵ㣻2����֤����������״̬�ļ�¼���Ե���ȷ
*/
void Deal_BlackEdge(void)
{ 
  int16 i=0,k=0;
  uint8 un_out_hang = bottom_whitebase ;
  uint8 lost_start_line=0;
  uint8 lost_end_line=0;
  left_top_whiteline = top_whiteline;
  right_top_whiteline = top_whiteline;

  //ͼ���ͻ����������ĳ��֣�����������Ŷ�Row_state[i]����һ����ֵ�˲�
  //��Row_state[i]�˲�
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
    
    if( (Row_state[i-1] == 1 || Row_state[i-1] == 3) && (Row_state[i] == 0 || Row_state[i] == 2))   //�ж���ߵ�i ���Ƿ񶪵�   
    {
       lost_start_line = i - 1;//��¼�����ǰһ��
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
           i++;//i++����Ҫ��if���ж�֮����У�����ᵼ�³���
        }
       if(lost_end_line !=0)
       {  
         for(k = lost_start_line+1; k< lost_end_line;k++)
         {
          left_black[k] = left_black[lost_start_line] + (k -lost_start_line)*(left_black[lost_end_line]-left_black[lost_start_line])/(lost_end_line - lost_start_line);  
         }
       }   
      else if(lost_end_line ==0 && lost_start_line >  top_whiteline/2 && lost_start_line <ROW-1  ) //����еĵ㵽����ص�ʱ�򣬲��ж�
       {   
        if( left_black[top_whiteline] > 1)
         {
           left_top_whiteline = lost_start_line;
           break;
        }
       }
         
    }
  }
  
  //�ұ�
  for( i=bottom_whitebase ;i < top_whiteline-1;i++)
  {
    lost_start_line = 0;
    lost_end_line = 0;
    if( (Row_state[i-1] == 0 || Row_state[i-1] == 3) && (Row_state[i] == 1 || Row_state[i] == 2))//�ж��ұߵ�i ���Ƿ񶪵�
    {
        lost_start_line = i - 1;//��¼�����ǰһ��
       while(i< top_whiteline-1)
        {
          //�����������ҵ��˵�����Ϊ�ҵ������ӵ�
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
    top_whiteline = right_top_whiteline;//���˶�����������⣬��Ҫ�Զ��ߵ���һ�������ߴ���,�������ұ�
    for(i = left_top_whiteline;i<=right_top_whiteline;i++)
    {
       if(right_black[i]- (right_black[i-1] - left_black[i-1]) <= 0 )//�޷�
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

 //�˳����ҵı����ߵĵ�������
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
 //�������˲�����
  
 //��֮ǰ�Ĵ���Ĳ��߽������µ��鹹
  //������鹹�ᵼ����һ������������ǣ�������Ķ��е�ʱ�򣬿��ܻ���ֲ��㣬�벻��������������͵����˶���Ķ���
  for( i=bottom_whitebase ;i <= top_whiteline;i++)
  {
    if(Row_state[i] != 3)
    {
      center_white[i] = (right_black[i]+left_black[i])/2;
    }
  }
  
  //�������߳���ĵ㣬��������ƽ��ֵ�ķ���ȥ������������ǰһ�е�״̬���в���
  for(i= bottom_whitebase; i <= top_whiteline; i++)
  {
    if(left_black[i] <= 1 && right_black[i] <= COLUMN-5)  //������߳���ĵ���в���
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
 
 
 
 //�����߽�����ֵ�˲�
 //�����ߺͱ����߽�����ֵ�˲�
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

/*�����������������֣���һ��������������ȡ��������������͵��ж�
*/
void get_line_information(void)
{
  int16 i;
  uint8 ramp_count = 0;  //���ڼ�¼��ȳ������Ƶ��еĸ���
  uint16 temp_center_line = 0;
  center_lost_hang = 0;
  
    /*����Ŀ�ʼ���ȶ����߳��ֶϵ����������޲�
  ��ͼ������߳����˾޴������ʱ����ǰ�����ȫ�����������ߵ���ֵ����
  */
  for(i = bottom_whitebase + 10 ; i < top_whiteline-5;i++)  //��׼���ϵ�ƫ��ô���
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
  
  
  //ͼ��������߳���֮������Ҫȷ���ܹ�����Щ�У��ڴ�֮�����ȫ���ÿ��Ƶ��е�
   /////////////////////��ȡ���Ƶ������Ч��////////////////////////////////
  danger_count = 0;
  danger_flag = 1;//û���ж�֮ǰ����Ϊ��Σ��״̬
  control_top_whiteline = top_whiteline;
  
  ////////////////////������Ƶ������Ч��//////////////////////
     while(danger_flag ==1)
     {
       for(i = control_top_whiteline;i>=deal_start_line ;i--)//��ʾ�������±���
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
         danger_flag = 0;//Σ������
       }
     }
     
     
  /////////////////���������͵��ж�/////////////////////////
    S_right = 0;//���ҹյļ���
    S_left =0 ; //����ռ���
    S_straight = 0;
  for( i=bottom_whitebase ;i < control_top_whiteline;i++)
 {
   if(center_white[i+1]- center_white[i] > 1)
   {
     S_right++; //S ���ҼӼ�
   }
   else if(center_white[i]- center_white[i+1] > 1)
   {
     S_left++; //S ����Ӽ�
   }
   else
   {
     S_straight++;
   }
 }
 
 /*���������ж�
 �����������ж� ֻ������ֱ��(1) ���������(2)�� ����(3)�����(4) */
 
 
 if( control_top_whiteline >= 62)
 {
   if(S_left<4 && S_right < 4 )
      direction = 1;  //ֱ��
   else if(f_abs16(S_left-S_right) < 15 )
      direction = 2;  //�������
   else
     direction =3;// re_direction;
 }
 else if( control_top_whiteline >= 50 && control_top_whiteline < 62)
 {
   direction = 3;  //�����
 }
 else
 {
   if(ramp_flag == 1)
      direction = 1;//�µ���Ϊֱ��
   else
      direction = 4;  //�������
 }
 re_direction =direction ;
 //////////////////////���������͵��жϽ���///////////////////////////////
 

     
  //////////////////////////////�����������Ż�//////////////////////////////
 //��Ҫע����Ǽ�Ȼ�Ѿ��������������жϳ����ˣ���ô�Ϳ��԰��ղ�ͬ������ʵ�ֲ�ͬ���Ż��ر�����Բ������o
 if(direction == 2)  //�������  �����д���60
 {
   for( i=bottom_whitebase ;i <= control_top_whiteline;i++)//������ʲô������������������ͼ�������ƽ��
     {
      if(center_white[i] > MID)//����
      {
        if( center_white[i] - (control_top_whiteline - 62 ) >= MID )
          center_white[i] = center_white[i] - (control_top_whiteline - 62 );
        else
          center_white[i] = MID;
      }
      else 
      {
        if(center_white[i] + (control_top_whiteline - 62 ) <= MID)  //���з�ֹ�ڲ�������Ľϴ�Ĵ��
          center_white[i] = center_white[i] + (control_top_whiteline - 62 );  
        else
          center_white[i] = MID; 
      }
     }
 }
 else if(direction != 1 )   //ֱ��״̬�����й��У���ֱֹ��Ư��
  {
   for( i=bottom_whitebase ;i <= control_top_whiteline;i++)//������ʲô������������������ͼ�������ƽ��
     {
      if(center_white[i] > MID)//����
      {
        if( center_white[i] - 1 >= MID )
          center_white[i] = center_white[i] - 1;
        else
          center_white[i] = MID;
      }
      else 
      {
        if(center_white[i] + 1 <= MID)  //���з�ֹ�ڲ�������Ľϴ�Ĵ��
          center_white[i] = center_white[i] + 1;  
        else
          center_white[i] = MID; 
      }
     }
 }
//////////////////////////////���������Ż�����//////////////////////////////
  /*
 ��������Ϣ����ȡ����Ҫ�������¼�������
 ֻ�ǶԴ��ڿ�����һ�µ�������ȡƽ��ֵ��
 */

 //////////////////////��ͼ���ƽ��ֵ����ȡ/////////////////////
  center_average = 0;//����
  center_error_average = 0;
  if(control_top_whiteline > 50)
  {
    for(i = bottom_whitebase+1;i<=control_top_whiteline- 10;i++)
   {
     center_average +=  center_white[i];
     if(i == control_top_whiteline - 10)    //ֻ�Ƕ�ǰ100cm���ҵ�ǰհ���м�Ȩ
     {
       center_average = center_average /(control_top_whiteline - bottom_whitebase -10);
       center_linear_average = center_average ;
     }
   }
   
     //����ƫ��ľ���ֵ���
    for( i=bottom_whitebase+1 ;i <= control_top_whiteline- 10;i++)
    {
      center_error_average += f_abs16( center_white[i]  - center_average);
    }
    center_error_average /= (control_top_whiteline - bottom_whitebase - 10) ;    //��Ӧ�����е�����ƫ���������ĵľ��ԵĴ�С�����Ĵ�С��һ���ĳ̶��Ϸ�Ӧ�ˣ����ߵ����ԶȵĴ�С
    
  }
  else
  {
   for(i = bottom_whitebase+1;i<=control_top_whiteline;i++)
   {
     center_average +=  center_white[i];
     if(i == control_top_whiteline)    //ֻ�Ƕ�ǰ100cm���ҵ�ǰհ���м�Ȩ
     {
       center_average = center_average /(control_top_whiteline - bottom_whitebase);
     }
   }
        //����ƫ��ľ���ֵ���
    for( i=bottom_whitebase+1 ;i <= control_top_whiteline;i++)
    {
      center_error_average += f_abs16( center_white[i]  - center_average);
    }
    center_error_average /= (control_top_whiteline - bottom_whitebase) ;    //��Ӧ�����е�����ƫ���������ĵľ��ԵĴ�С�����Ĵ�С��һ���ĳ̶��Ϸ�Ӧ�ˣ����ߵ����ԶȵĴ�С
    
  }
   
    //Ϊ��׼ȷ���жϳ������ı仯���ƣ�ʮ���б�Ҫ���Ƕ����������ʷ���д洢������洢7����ʷֵ��Ȼ�����ģ���жϡ�
    //ͨ�����Է��� ���ֵ�ı仯������0 �� 25֮��仯  ����ֱ����ʱ��ֵ��0 - -8֮��������е�ʱ������16--25֮��仯
    //����ֵ�ﵽ19������Ϊ�Ѿ���������м�����ڳ����
  ///////////////��ֵ��ȡ����//////////////////////////////
  
  /*/////���µ����жϣ����µ��жϳ���֮���ñ�־λ��־������ֻ�������ڵ���ĸ���������
  �����µ��ļ��ֻ��Ҫ�����µ�������������нӽ����У������ȴﵽ��һ���ķ�Χ�����¼��Ƚϵ����ѣ�
  �������ﲻ������£�һ��ֻ����һ���µ�״̬��ʱ�����ˣ���ʱʱ��Ϊ1s--2s֮�䣩��
  �Ҽ�����֮��ֻ��Ҫ���٣����ڶ�����Բ���ȥ�ܡ�ͼ���Ѿ����ÿ����ˡ�
  ���µ��ļ�ⲻ��ֻ���ÿ��ȥ�ж����������׺����������졣���Ի�Ҫ���϶Զ˵�����ƣ�����������ĳһ����Χ֮��,
  ���Ҫ��������֮ǰ�����ǰ����ġ�
  #define RAMP_WIDTH  45                  //ͼ��10~20�еĿ�ȷ�Χ�����÷�Χȷ��Ϊ�µ�
  #define RAMP_TIME   60
  uint8 ramp_time = 50;                     //�����µ���೤ʱ�����¿��������߼��
  int8 ramp_speed = 0;                    //�µ�����ֵ
  bool ramp_flag;                          //�����µ���־,��Ҫ���ڿ���
  bool ramp_dis_flag;                     //��Ҫ�Ƿ�ֹ��������
  
  �����µ��ļ�ⲻ������ߵļ��У���Ϊ�������׺�ʮ�ֵ�·���
 */
  
  ramp_count = 0;//3,65,4,25
  if( ramp_dis_flag ==0 && direction == 1 && control_top_whiteline >= ROW - 2 ) //ֱ��״̬���  //�������ramp_dis_flag��־����Ϊ���ó����ڼ�����ܵ�������ʱ��������µ����м��
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


/*-------------------------------��ȡ���ߵ��������ϵ��-----------------------------*/
float get_linear_factor(uint8 bottom,uint8 top,uint8 average)            //��������������׼�У����У���������MID�Ĳ�ֵ
{
    uint8 i;
    uint8 Y_aver=0;
    float X_square_sum=0;   //X��ƽ����
    float Y_square_sum=0;   //Y��ƽ����
    float multi_sum=0;      //XY�˻�֮��
    int temp=0,temp1=0,temp2=0; //���ḡ������ļĴ���
    float factor=0;
    
     Y_aver=(uint8)((bottom+top)/2);  //Y����ķ�Χ
    for(i=bottom;i<=top;i++)
    {
        temp=temp+(center_white[i]-average)*(center_white[i]-average);
        if(temp>30000)
        {
            X_square_sum=X_square_sum+temp;   //Xƽ����
            temp=0;
        }

        temp1=temp1+(i-Y_aver)*(i-Y_aver);
        if(temp1>30000)
        {
            Y_square_sum=Y_square_sum+temp1;   //Yƽ����
            temp1=0;
        }

        temp2=temp2+(center_white[i]-average)*(i-Y_aver);
        if(f_abs16(temp2)>30000)
        {
            multi_sum=multi_sum+temp2;    //X��Y�Ļ�
            temp2=0;
        }
    }
     
        X_square_sum=X_square_sum+temp;   //�ó�x��ƽ����
        Y_square_sum=Y_square_sum+temp1;  //�����y��ƽ����
        multi_sum=multi_sum+temp2;        //�����xy�ĳ˻�
    
       /* XX_square_sum =X_square_sum;      //���ڼ��
        YY_square_sum =Y_square_sum;
        XYmulti_sum = multi_sum;*/
        
        if(X_square_sum<0.1)   //��ֹ����Ϊ0
            X_square_sum=0.1;
        if(Y_square_sum<0.1)
            Y_square_sum=0.1;
        
        if(X_square_sum<300)  //С��300������ֱ��
            factor=multi_sum/f_absf(multi_sum); // =1 or =-1 //��ȫ��ֱ��
        else  //�����ù�ʽ���� //ע�⴦���ٶȣ�С��200ʱ����ʱ���ٺܶ�
            factor=multi_sum/sqrt(X_square_sum*Y_square_sum)*(bottom_whitebase+control_top_whiteline-20)/(65-20);
        
        if(factor>0.95)
          factor=1;
        if(factor<-0.95)
          factor=-1;
    
        return factor;
}


/*���������߼���ͣ��������������ͷ���м��
 �������ҪĿ���Ǹ�stop_flag��λ,Ϊ���ܹ���������ߣ����뱣֤���Ǽ��ľ���Ϊ������ǰ��20�������ϡ�
�����ͼ����ԵĻ�����ͼ���30�У�
�����׵��ٶ������Ļ���Ҳ���Ǳ��뱣֤����ͼ��������һ����⵽��40ms*5m = 20����
ע������ļ���Ǽ��ڵ��׵����䡣����ȼ��׵��ڵ�������ӵ�׼ȷ
*/
void check_start_stop_line()
{ 
  int i,j;
  uint8 left_start_stop_hang = 0;
  uint8 left_start_stop_flag = 0;
  uint8 right_start_stop_hang = 0;
  uint8 right_start_stop_flag = 0;
  //��������������
  if(top_whiteline - bottom_whitebase > 50)
  {
    for(i=bottom_whitebase+3;i< bottom_whitebase + 40;i++) //ֻ�Ǽ��ǰ��ʮ���У����ڳ����ǰ20��������  
    {
        //���ٱ�֤���ߵ����ҵ��������ǰ�ɫ�� ����Ҫ��֤���ʱ����е�״̬Ϊ3
      if( top_whiteline >= 50
         && (VideoImage2[i - 1][ center_white[i]] -  VideoImage2[i + 1][ center_white[i]])< OT
          && (VideoImage2[i - 1][ center_white[i] - 1] -  VideoImage2[i + 1][ center_white[i] - 1]) < OT
           && (VideoImage2[i - 1][ center_white[i] + 1] -  VideoImage2[i + 1][ center_white[i] + 1]) < OT)
      {//���������ܹ��������Ǻ�ɫ��
        //�����ΰ�ɫ�ļ���õ������ұ��ظ�����������5����
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
        }//�ұ����� 
      } //���߷��ϱ�׼
        //���ҵ��˷��ϵ������ߵ�ʱ������ѭ��//�ж��ǻ�Ҫ��ֹͼ��Ĵ�λ
        if(  left_start_stop_flag == 1 && right_start_stop_flag == 1 && f_abs16(right_start_stop_hang - left_start_stop_hang) < 3)
        {
          stopflag = 1;
          break;//
        }
    }//forѭ��
  }
}


/*-----------------------------------����͵���Ŀ��ƺ����ı���---------------------------------
����������ڽ����������Ż��Ϳ��ƣ����п��ư�������Ͷ�����������֡��������ͳһ�Ŀ���
*/

void Control()
{
 int16 i=0,j=0; //
  p_error = 0;
  refer_error = 0;
  get_p_errorline = 55;
  
  //���ƫ��Ͷ���Ŀ���p��d����
  if(control_top_whiteline -deal_start_line >10)
  {
    if(control_top_whiteline < get_p_errorline )
      get_p_errorline = control_top_whiteline ;  //���ƶ���Ŀ����ж���Ŀ����в���̫����̫�������������ֱ��������
    
    for(j= get_p_errorline; j>= get_p_errorline -4; j--)
    {
      p_error += center_white[j];
    }
    //�ó�������error  
    p_error  = (MID -  p_error/ 5);
    
    //��ȡȫ����һ��ƫ��
    refer_error = MID - center_average;       //�ڳ��ӳ���ֱ�������һ����������ֱ����ʱ�����׳����𵴣��������ǰֱ���ü�Ȩƽ�����㷨�����ƣ��������ｫ��������д
    
    ref_his_error[0] = ref_his_error[1] ;
    ref_his_error[1] = ref_his_error[2] ;
    ref_his_error[2] = ref_his_error[3] ;
    ref_his_error[3] = ref_his_error[4] ;
    ref_his_error[4] = refer_error ;
    
    //�õ���Ȩ֮���������ƫ��
   refer_error  =((ref_his_error[0] + ref_his_error[1] + ref_his_error[2]+ 2*ref_his_error[3])+95*ref_his_error[4])/100;  //ȡ�����˵�10�У������ڶԲ�������Ŀ���
    
    //������error
   /*if((re_control_top_whiteline < control_top_whiteline) && (direction == 3 || direction== 4))  //re_control_top_whiteline top_error_servo_p
     top_error_servo_p = 1*(control_top_whiteline - re_control_top_whiteline)/6;
   else if((re_control_top_whiteline > control_top_whiteline) &&  direction== 4)        //������Ӵ���
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
   if(direction == 1 && control_top_whiteline >= ROW - 2)  //��ʱֱ��״̬��ʱ�򣬶�����ײ��нϴ��ƫ���ʱ�����ʱ����Ȼ��Ҫһ��p������
   {
     top_error_servo_p = 3*f_abs16(center_white[deal_start_line] - center_white[control_top_whiteline-1])/4;       //������ײ���ƫ������ܴﵽ70����
   }
   else
     top_error_servo_p = 0;
     
    error_servo_p = 5*(ROW - control_top_whiteline )/13  +  1*f_abs16(p_error)/8 + lcd_error_servo_p;  //�������p�����ڽ������
    error_servo_d =lcd_error_servo_d;//control_top_whiteline ;  //���ڱȽ�ֱ����·��Ҫ��d��΢��һ�� ���������������һֱ��d������ ��̫����
    
    //���е�error
    error_servo_ref_p = 1 * f_abs16(refer_error)/7 + lcd_ref_p + top_error_servo_p;  //�������p�����ڳ��������     1/7
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
      //���ʱ����Ե��ڶ��������ֵ��ʹ�ó�������
      //��ʷֵ�ı���
      re_angle= angle;
  p_re_error = p_error;
  re_refer_error =refer_error;
  re_control_top_whiteline = control_top_whiteline;
  
  
  //�����ǵ���Ŀ��Ƴ��� �ٶȵĿ���������ƫ��ʹ������Ч��
 //�����������Ӧ�ã����������Ч�ж�̬�ĸı�����ٺ������
  //Ȼ��ͨ��ƫ��������׼ȷ���ٶ�
  
  //ÿ�ζ��ٶ���Ҫ���µ��趨

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
   
      if( direction == 1 )  //�����ٶȵĿ���ʵ�ֲ�ͬ�������ò�ͬ�Ŀ���
       {
         straight_speed = (control_top_whiteline+ROW)*straight_speed/ROW/2;  //�޶�������ٶ�
         bow_speed = (control_top_whiteline+ROW)*(bow_speed)/2/ROW;         //ֻ���޶�������ٶ�
       }    
      else if( direction == 2 )  //������
       {
         straight_speed = (control_top_whiteline-2)*straight_speed/ROW;  //   �޶�������ٶ�
         bow_speed =  (control_top_whiteline-2)*(bow_speed)/ROW;         //ֻ���޶�������ٶ�
       }
        else if( direction == 3)  //����
       {
         straight_speed = (control_top_whiteline-4)*straight_speed/ROW;  //   �޶�������ٶ�
         bow_speed =  (control_top_whiteline-4)*(bow_speed)/ROW;         //ֻ���޶�������ٶ�
       }
    
       else   //���
       { 
        straight_speed = (control_top_whiteline-4)*straight_speed/ROW;  //   �޶�������ٶ�
         bow_speed =   (control_top_whiteline-4)*(bow_speed)/ROW;         //ֻ���޶�������ٶ�
       }
      
        
        //center_error_average�ı仯��Χ������30  
      if(control_top_whiteline ==ROW - 1&&(direction==1||direction==2) )
          {
            if (direction == 1 )
            {
              straight_count++;
              if(straight_count > 2)
               speed_except = straight_speed + straight_speed_ed;  //ֱ�ߺͲ��������ȫ����ʻ
              else
               speed_except = straight_speed; 
            }
            else
            {
              speed_except = straight_speed + 20;
            }
          }
      /*����ֱ��������������������������ϵ����������̶�ǰհ����������ԣ���������1.9��ǰհ��ֻ�Ǽ����ǰ1.2�׵����ϵ������
      Ȼ������ʣ��ǰհ�����ж��Ƿ�����������жϣ��жϵķ������ж϶����еı仯���ơ�����ͬʱ����һ�ߣ���˵���ǽ��������������
      �����Ϣ����ֱ����������жϣ�����ļ����ǽ��ٶȼ���������ٶȣ������ǽ��ٶȼ�С����������͡���������������������Ĳ��ȶ���
      */
 
      else if(f_absf(linear_factor) == 1)   //����ֻ���ڽ������ʱ������жϣ����������������״̬
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

//------------------------���ڷ��ͺ���------------------------//
void SCI0_send_mesage(void)
{
    int i = 0,j = 0;
    static bool sci_temp = 0;
    DisableInterrupts;  //����ͼ������ʱ��Ҫ�ر������жϣ���������    
    if(send_mes == 1)  //ͼ��
    {  
      while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART0_D = WHITE_BLACK_OT;//���ڷ�ֵ�Ĳ����ڣ�������ֻ�������д��һ������
              while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART0_D = (uint8)ROW;
              while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
        UART0_D = (uint8)COLUMN;
        
        
        //��λ����ʾ�ĵ�һ���������Ͻǣ������ҷ���ʱ���һ����ͷ����Ͻǵĵ�
      for(i =ROW-1;i>=0;i--)
      {
        for(j=0;j<COLUMN;j++)
        {
          while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
          UART0_D =  VideoImage2[i][j];///�����һ����������
          Delay_MS(80000);
        }
      }  
      for (i =ROW-1 ;i >=0 ; i--)
      {
        while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
           UART0_D = left_black[i];
        Delay_MS(80000);
      }
      for (i =ROW-1 ;i >=0 ; i--)
      {
         while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
         UART0_D = right_black[i];
         Delay_MS(80000);
      }
      for (i =ROW-1 ;i >=0 ; i--)
      {
        while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
        UART0_D = center_white[i];
        Delay_MS(80000);
      }
      send_mes = 0;  //����һ�μ��ɣ�����Ҫ����
     }
   // EnableInterrupts;  //���¿��������ж�
    
    else if(send_mes == 2)
    {
      for(i =0;i<=ROW - 1;i++)
      {
          while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
          UART0_D = center_white[control_top_whiteline];
          Delay_MS(80000);
      }
       
    }
 
   else if (send_mes == 3)  //���ڵ���
    {  
        if(!sci_temp)
        {
       while(!(UART0_S1&UART_S1_TDRE_MASK));
       UART0_D = (uint8)(speed_feedback);//speed_feedback
       sci_temp = !sci_temp;
        }
        else
        {
         while(!(UART0_S1&UART_S1_TDRE_MASK));//�ȴ����ݵ���
         UART0_D= (uint8)(speed_except);
          sci_temp = !sci_temp;
        }
        //       send_mes = 0;����0��Ϊ����������
      }
    else if (send_mes =='p')  //ͣ��
    {
       while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
       stopflag = 1;
       send_mes = 0;
    }
    else if (send_mes == 's')  //����
    {
       while(!(UART0_S1&UART_S1_TDRE_MASK));   //�ȴ����ݵ���
       stopflag = 0;
       send_mes = 0;  
    }
    EnableInterrupts;
}

//�����õİ�λ��Ӧ��k60���� PTD6---- PTD13
// 0000 0000 0000 0000 0000 0000 0000 0000 
void scan_boma(void)
{
  uint32 temp1=0;
  uint8 temp=0;
  GPIOD_PDOR = 0xffffffff; 
  temp1 = GPIOD_PDIR;   //��PTD6~PTD13 
  
  temp = !(uint8)((temp1 >> 13) & 0x00000001);
  if(lcd_debug == 1)
  {
    if(temp == 1)   //��Ӧ���ǲ���8
     lcd_debug = 1;  //������״̬Ϊ��������ʱ�䣬��������ϣ������������������������
  else
     lcd_debug = 0;
  }
  temp = !(uint8)((temp1 >> 12) & 0x00000001);//��Ӧ���ǲ���7
  if(temp == 1)
       redraw_control=1;  //������ˢ��
  else
       redraw_control=0; 
  
   //  start_stop_cs �����߼���Ƭѡ�ź�
    temp = !(uint8)((temp1 >> 11) & 0x00000001);//0x00000800
   if(temp == 1)
     start_stop_cs = 1;
   else
     start_stop_cs = 0;
   
  //  �ٶ�ѡ���ź�
   /*temp = !(uint8)((temp1 >> 10) & 0x00000001);//0x00000800
      if(temp == 1)
     stop_delay_check = 1;  //���٣����ڲ���
      else
     stop_delay_check = 0;*/
   
}


//-----------------------------�ӳ�-------------------------------//
void Delay_MS(uint32 ms)
{
   while(ms--);
}


/***********************************Ԥ��ʾ**********************************/
void pre_show(void)
{
  LCD_CLS();
   switch(lcd_page_num)
   {        
      case 0:     //��һҳֻ��������ʾ����      
             break;        
             
      case 1:
             LCD_P6x8Cha(0,lcd_line_num,'*');
             
             LCD_P6x8Str(10,0,"lcd_debug:");      //����ѡ��
             LCD_P6x8Num(100,0,lcd_debug);  
             
             LCD_P6x8Str(10,1,"speed_select:");      //�ٶȵ�λѡ��
             LCD_P6x8Num(100,1,speed_select);
             
             LCD_P6x8Str(10,2,"s_pit_count:");      //�����ߵ���ʱ������
             LCD_P6x8Num(100,2,stop_pit_count);
             
             LCD_P6x8Str(10,3,"rampdetime:");      //�µ���ʱʱ��
             LCD_P6x8Num(100,3,ramp_delay_time);
             
             LCD_P6x8Str(10,4,"mid_angle:");      //�����У������ֵ
             LCD_P6x8Num(100,4,mid_angle);  
                     
             LCD_P6x8Str(10,5,"WHITE_BLACK_OT:");      //�����У���ֵ������ֵ
             LCD_P6x8Num(100,5,WHITE_BLACK_OT);  
             
             LCD_P6x8Str(10,6,"test_run:");      //�����У���ֵ������ֵ
             LCD_P6x8Num(100,6,test_run); 
             
            break;
            
      case 2:
             LCD_P6x8Cha(0,lcd_line_num,'*');
        
             LCD_P6x8Str(10,0,"l_er_ser_p:");    //��һ�У������p
             LCD_P6x8Num(100,0,lcd_error_servo_p);
             
             LCD_P6x8Str(10,1,"l_er_ser_d:");    //�ڶ��� �������d
             LCD_P6x8Num(100,1,lcd_error_servo_d); 
             
             LCD_P6x8Str(10,2,"lcd_ref_p:");    //������ �������p
             LCD_P6x8Num(100,2,lcd_ref_p);
             
             LCD_P6x8Str(10,3,"lcd_ref_d:");    //������ �������d
             LCD_P6x8Num(100,3,lcd_ref_d);         
             
             LCD_P6x8Str(10,4,"l_str_speed:");     //ֱ���ٶ�
             LCD_P6x8Num(100,4,lcd_straight_speed); 
             
             LCD_P6x8Str(10,5,"l_bow_speed:");     //����ٶ�
             LCD_P6x8Num(100,5,lcd_bow_speed); 
             
             
             LCD_P6x8Str(10,6,"lcd_strspe_ed:");     //��ֱ���ĸ����ٶ�
             LCD_P6x8Num(100,6,lcd_straight_speed_ed);
             

             break;
     }

}


/*
С��Һ�����Ĳ�����
128*64  д��6*8  ÿ�й�21���ַ�  һ������д8��------
                                              |
                                              |
                                              |
                                              |
                                              |
                                              |
                                              |
���ڶ�λ��x��y  x��ʾ����������������ÿһ���������㣬������ÿ����Ԫ����
��y������ÿ����Ԫ���Ӽ��㣬�������8

*/
/**************************************ˢ������ʾʱ�����*********************************/
/*ˢ����ʱ���ر���Ҫע��ʱ������⣬����ˢ����ʱ������ᵼ�¿��ƵĲ���ʱ�����ܻ����
���ｫ����һ��ֻˢ��һ�εĲ���*/
void redraw()
{
  byte lcd_hang = 1 ; 
  if(lcd_page_num==0&&redraw_control==1)     //��һҳ//redraw_control��Ҫ��һ��������п���
     {
         if(lcd_hang == 1)
         {
           LCD_CLS_ROW(0,0);      //0��  8��
           LCD_P6x8Num(0,0,top_whiteline);    //��һ��  ͼ�����
           LCD_P6x8Num(40,0,control_top_whiteline);  
           LCD_P6x8Num(80,0,deal_start_line); 
           lcd_hang ++;
         }
         if(lcd_hang == 2)
         {
          LCD_CLS_ROW(0,1);       //�ڶ��У����ƫ�����
          LCD_P6x8Num(0,1,white_refer);
          LCD_P6x8Num(40,1,center_average);
          LCD_P6x8Num(80,1,p_error);
          LCD_P6x8Num(100,1, center_error_average);
          lcd_hang ++;
         }
        if(lcd_hang == 3)
         {
          LCD_CLS_ROW(0,2);       //�����У� ������
          LCD_P6x8Num(0,2,speed); 
          LCD_P6x8Num(60,2,speed_except); 
          LCD_P6x8Num(100,2, speed_feedback);
          lcd_hang ++;
         }
         if(lcd_hang == 4)
         {
          LCD_CLS_ROW(0,3);       //�����У�   //���������ͱ�־
          LCD_P6x8Num(0,3,S_straight);
          LCD_P6x8Num(40,3,S_left); 
          LCD_P6x8Num(70,3,S_right);
          LCD_P6x8Num(110,3,direction);
          lcd_hang ++;
         }
           if(lcd_hang == 5)
         {
          LCD_CLS_ROW(0,4);       //�����У�   //��־λ
          LCD_P6x8Num(0,4, stopflag); 
          LCD_P6x8Num(40,4, ramp_flag);
          LCD_P6x8Num(80,4, linear_factor);
          lcd_hang =1;
         }
       /*    if(lcd_hang == 6)
         {
          LCD_CLS_ROW(0,5);//�����У�   //�������
           LCD_P6x8Num(0,5, XX_square_sum); 
           LCD_P6x8Num(60,5, YY_square_sum);
           LCD_P6x8Num(110,5, XYmulti_sum);
           lcd_hang = 1;   //�ָ���1
         }
                */    
     }  
}

void key_down(void)
{
  uint8 temp=0;
  uint32 temp1=0;
  //�˿���c8��ʼ����İ�λ����Ϊ�ߵ�ƽ
    Delay_MS(200000 * 4);
    GPIOC_PDOR = 0xffffffff; 
    temp1 = GPIOC_PDIR;   //��PTC8~PTC15   
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

//---------------------------ȫ����ɨ��-----------------------------//
void Keyscan(void)
{
      key_down();
      
      if(change_page)  //�����⵽�͵�ƽ��˵����������
      {    key_down(); //��ʱȥ����һ��10-20ms
           if(change_page)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
           {   
              while(change_page)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
                key_down();
              if(lcd_page_num<2)    //ҳ��żӲ��������ҳ��2�ǿ��ԸĶ���
	         lcd_page_num++;
	       else
	         lcd_page_num=1;
               lcd_line_num=0;
              pre_show();//��������ʾ��������ǰд������һ����д��������ĺ�����
               
           }
      }
     //��ҳ�� 
      if(se_sub_NUM)  //�����⵽�͵�ƽ��˵����������
      {    key_down(); //��ʱȥ����һ��10-20ms
           if(se_sub_NUM)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
           {
              while(se_sub_NUM)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
              
               key_down();
               LCD_change_value(lcd_page_num,lcd_line_num,-1);
               
           }
      }
      
      
     if(lcd_page_num!=0)     //�粻Ϊ��һҳ���������һ��ɨ��  
     { //��ɨ��
      //����
       key_down();
      if(up_line)  //�����⵽�͵�ƽ��˵����������
      {
            key_down(); //��ʱȥ����һ��10-20ms
            if(up_line)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
            {
               while(up_line)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
                 key_down();
               if(lcd_page_num!=0)
	        LCD_P6x8Cha(0,lcd_line_num,' ');
               
               
                if(lcd_line_num<LCD_ROW)    //����żӲ���
	         lcd_line_num++;
		 else
		  lcd_line_num=0;
                if(lcd_page_num!=0)  
              LCD_P6x8Cha(0,lcd_line_num,'*');
            }
      }
      //����
       if(down_line)  //�����⵽�͵�ƽ��˵����������
      {
            key_down(); //��ʱȥ����һ��10-20ms
            if(down_line)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
            {
               while(down_line)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
                 key_down(); 
                if(lcd_page_num!=0)
	        LCD_P6x8Cha(0,lcd_line_num,' ');
                if(lcd_line_num>0)    //����żӲ���
	         lcd_line_num--;
		 else
		  lcd_line_num=LCD_ROW;
                  
              LCD_P6x8Cha(0,lcd_line_num,'*');//
            }
      }
      
       if(add_NUM)  //�����⵽�͵�ƽ��˵����������
    {
	key_down(); //��ʱȥ����һ��10-20ms
     if(add_NUM)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
	   {
      while(add_NUM)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
        key_down();
        LCD_change_value(lcd_page_num,lcd_line_num,1);
	   }
     }
      
     
     if(sub_NUM)  //�����⵽�͵�ƽ��˵����������
    {
	key_down(); //��ʱȥ����һ��10-20ms
     if(sub_NUM)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
	   {
        while(sub_NUM)//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
          key_down();
     LCD_change_value(lcd_page_num,lcd_line_num,-1);
	   }
    }
    
    
     }
 }
//-------------------------��-��LCD���̵���---------------------//
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


//-------------------------------------������������˿�---------------------------------------------------//

void PORT_Init(void)
{ 
    
    PORTB_PCR20 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//PTC1��������ΪGPIOģʽ ����
    
    PORTE_PCR0 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E4��������ΪGPIOģʽ
    PORTE_PCR1 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E5��������ΪGPIOģʽ
    PORTE_PCR2 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E6��������ΪGPIOģʽ
    PORTE_PCR3 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E7��������ΪGPIOģʽ
    PORTE_PCR4 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E8��������ΪGPIOģʽ
    PORTE_PCR5 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E9��������ΪGPIOģʽ
    PORTE_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E10��������ΪGPIOģʽ
    PORTE_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//E11��������ΪGPIOģʽ
    
        
        GPIOE_PDDR = 0xffffff00;  //E0~E7����Ϊ����� 

        GPIOB_PDDR = 0xffefffff;  //PTC1����Ϊ����
        
        
         PORTD_PCR6 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D6��������ΪGPIOģʽ   //���뿪��
         PORTD_PCR7 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D7��������ΪGPIOģʽ
         PORTD_PCR8 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D8��������ΪGPIOģʽ
         PORTD_PCR9 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D9��������ΪGPIOģʽ
         PORTD_PCR10 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D10��������ΪGPIOģʽ
         PORTD_PCR11 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D11��������ΪGPIOģʽ
         PORTD_PCR12 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D12��������ΪGPIOģʽ
         PORTD_PCR13 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK |  PORT_PCR_PS_MASK;//D13��������ΪGPIOģʽ 
   
         GPIOD_PDDR = 0xffffc03f;       
}

//---------------------------------------------------������------------------------------------------------//
void main(void)
{
   uint16 i=0,j=0;
   lcd_debug = 1;
    DisableInterrupts;
    pllinit180M(); 
    
    LCD_IO_Init();
    LCD_Init(); 
    //��Ƭ���ϵ�󣬼�Ⲧ��Ͱ��������������ú���Ӧ�Ĳ������˳�

    PORT_Init();              //�˿ڳ�ʼ��
    hw_FTM_init();
    UART0_Init();             //���ڳ�ʼ��   
    LPTMR_Init();             //�����������ʼ��
    EXIT_Init();   
    while(lcd_debug)
    { 
      pre_show();
      scan_boma();
      Keyscan();  //��Ⲧ���ʱ����������Լ�Ⲧ�����while֮ǰ�����Ҽ����󣬽���رգ��ó����ܶ�
    } 
    lcd_page_num = 0;
    Delay_MS(40000000);   //�����ӳ� uint8 ch=3;
    Delay_MS(40000000);  
    OddEvenStatus = ODD_EVEN_STATUS;
    VIF = VIF_START;
    Initial();
    
    enable_irq(45);           //�򿪴����ж�
    enable_irq(88);           //�����ж� 
    EnableInterrupts;

    
    while(1)
    {      
       if(ImageReady)                                         //ͼ��׼���ã��پ���
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
           if(start_stop_count > 50*stop_pit_count)  //��ʱ6s���������300
           {
             start_stop_en = 1;
             start_stop_count = stop_pit_count;//300
           }
         }
         //���ڲ���������
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
        if(find_whitebase_flag == 1)  //��������ļ��룬������ô���1 ��ֹû���ҵ���׼�е�ʱ������� 2����û���ҵ���׼�е�ʱ�򱣳���һ��������
        {
          Search_BlackEdge();
          Deal_BlackEdge();
          get_line_information();
          if(control_top_whiteline >=50 && control_top_whiteline <ROW-1)  //�����Ӵ���������е�ʱ�򣬲�����������صļ���
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
          redraw();//ˢ����ʾ��   
          SCI0_send_mesage(); 
          while(ImageReady);
       }
    }  
   
}
  

//-----------------------------�жϺ���-----------------------------//
void uart0_isr(void)          //�����ж�
{    
    DisableInterrupts;   // �����ж�Ҳ���ԣ������и��߼��жϴ����ﲻ�Ƽ�
      uint8 ch;
     while(!(UART0_S1&UART_S1_RDRF_MASK));
      ch = UART0_D;
      if(ch == '1')     //���͵���ԭʼͼ��
        send_mes = 1; 
      else if(ch == '2')  //����ͼ��ı仯����
        send_mes = 2;
      else if(ch == '3')  //�ٶ�ͼ��
        send_mes = 3;
      else if (ch >= 64 && ch <= 65)   //�䵵����
      {   
          switch(ch - 64)  //�䵵����
          {
          case 0:   send_mes='s';break;//stopͣ��
          case 1:   send_mes='p';break;
          default: speed = 100;
          }
      }

    EnableInterrupts;
}

//------------------------------------ͼ��ɼ��ж�----------------------------------------//
void PTB_isr(void)//���ڳ��ж�20ms�����ж�63us��������ͷ�Ĺ��е�Ƶ�ʣ�����Ҫ�����ȥ��ʱ
{
  /*
        ͼ��ɼ�������Ϊ 27 30 33 36 39 42 45 48 51 54 57 60 63 
                         66 69 72 75 78 81 84 87 90 93 96 99 102 
                         105 108 111 114 117 120 123 126 129 132 135 138 141
                         144 147 150 153 156 159 162 165 168 171 174 177 180 
                         183 186 189 192 195 198 201 204 207 210 213 216 219 
                 
        */ 
    uint16 i;  
    
  
   PORTB_PCR22|=PORT_PCR_ISF_MASK;  //����жϱ�־λ
    if (VIF == VIF_START)                              //��ʼ������־
      {
        LineCount++;
        if(OddEvenStatus != ODD_EVEN_STATUS)
        {
          OddEvenStatus = ODD_EVEN_STATUS;	//��ż����־
          VIF = VIF_WAITSAMPLE;   		//��һ��״̬Ϊ�ȴ�����
          VideoImageLine = 0;
          LineCount = 0;
          ImageReady = 0; 
        }
      }
    else if (VIF == VIF_WAITSAMPLE)                 //�ȴ�����,��ʱ��ȥVIDEO_START_LINE��
      {
          LineCount++;
          if (LineCount >= VIDEO_START_LINE)
          {
              VIF = VIF_SAMPLELINE;                 //��һ��״̬Ϊ����״̬
          }   	
      }
    else if (VIF == VIF_SAMPLELINE)              //��ʼ����
      {
          LineCount++;
          if (LineCount % 3== 0)                //ÿ��һ�в�һ��
          {
              for (i = 0; i < COLUMN+PIANYI ; i++)        //ÿ��ɨ��COLUMN+PIANYI����(����PIANYI������Ҫ���޳�������Ϊ����������)
             {
                  if (i >=PIANYI )
                   {//�ɼ��ĵ�һ�������������ʵ�������������½ǣ������������д洢�ڵ�һ�е����һ��λ��
                     VideoImage2[VideoImageLine][i-PIANYI] = (uint8)(0x000000ff & GPIOE_PDIR);//���ɼ����ĵ�ֱ�ӷ��뵽VideoImage2[][]����init array�����зŵ�VideoImage1[][]��������
                         Delay_MS(3); 
                        asm("nop");
                        asm("nop");//�����ʱ
                  }
              }
             VideoImageLine++;
          }
          if (VideoImageLine == ROW)      //�ɼ����������趨������
          {
              ImageReady = 1;           //ͼ��׼����
              VIF = VIF_START;
          }
          
     }
    
  /* if (start_stop_en = 1 && start_stop_cs ==1 && (delay_detective && ((left_tube1 || left_tube2) && (right_tube1 || right_tube2))))        //ÿ���жϼ��һ��������
    {
      stopflag = 1;
    }
    //��������Ӷ������ߵļ�⡣
    */
    
    if (LineCount % 45 == 42)         //   7��  ÿ��45�п���һ��  ��һ�ο���λ 43 88 133 178 223 268 313    
    {
        speed_feedback = LPTMR0_CNR;                  //����������ֵ
        LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;
        LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;                //������������
        
       if(stopflag == 1 && speed_down_cnt>10) 
        {
            if( speed_feedback >= 18 )
              {
                 FTM1_C0V=600;  //��ת
                FTM1_C1V=0;   //��ת
              }
            else if(speed_feedback <18)
              {
                FTM1_C0V=0;  //��ת
                FTM1_C1V=0;   //��ת
                dead_stop = 1;
              }
            
             if(dead_stop == 1)
             {
                FTM1_C0V=0;  //��ת
                FTM1_C1V=0;   //��ת
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
          
          if( speed_error > 15)  //���Է��ֵ���ת�ϴ���700ʱ���ڼ��ٵ�ʱ������ͷ��������ڵ�Σ��
          {
          //  FTM1_C0V = 800;
           // FTM1_C1V = 0;
          FTM1_C0V = 0;
          FTM1_C1V = 780;
          }
         else if(speed_error < -13)  //���ڷ�ת����
          {
            //FTM1_C0V=0;  //��ת
           // FTM1_C1V=650;   //��ת750
            FTM1_C0V=950;  //��ת
           FTM1_C1V=0;   //��ת750
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
 
