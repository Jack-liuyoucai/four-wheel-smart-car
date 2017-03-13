//-------------------------------------------------------------------------*
//文件名: includes.h                                                       *
//说  明: 总头文件,本文件包含:                                             *
//        主函数(main)文件中用到的头文件、外部函数声明及有关常量命名       *
//-------------------------------------------------------------------------*
#ifndef INCLUDE_H_
#define INCLUDE_H_

    //1 头文件
    //1.1通用头文件
    #include "common.h"            //通用函数头文件
            
    //1.2包含面向硬件对象头文件(即构件模块)       
    #include  "light.h"                  //小灯构件头文件
    
    //2 宏定义

#endif
//-------------------------------------------------------------------------
// 文件名称：hw_adc.h                                                          
// 功能概要：adc构件头文件
// 版权所有: 苏州大学飞思卡尔嵌入式中心(sumcu.suda.edu.cn)
// 版本更新:    时间                         版本                     作者                          修改
//           2011-11-17     V1.0        stone    编写了K60的ADC驱动
//-------------------------------------------------------------------------

#ifndef __ADC_H__
#define __ADC_H__

#define ADC0_irq_no 57
#define ADC1_irq_no 58

#include "common.h"

  


#define A                 0x0
#define B                 0x1


#define COCO_COMPLETE     ADC_SC1_COCO_MASK
#define COCO_NOT          0x00

// ADC 中断: 使能或者禁止
#define AIEN_ON           ADC_SC1_AIEN_MASK
#define AIEN_OFF          0x00

//查分或者单端ADC输入
#define DIFF_SINGLE       0x00
#define DIFF_DIFFERENTIAL ADC_SC1_DIFF_MASK

// ADCCFG1

//ADC电源设置 
#define ADLPC_LOW         ADC_CFG1_ADLPC_MASK
#define ADLPC_NORMAL      0x00

//时钟分频
#define ADIV_1            0x00
#define ADIV_2            0x01
#define ADIV_4            0x02
#define ADIV_8            0x03

//长采样时间或者短采样时间
#define ADLSMP_LONG       ADC_CFG1_ADLSMP_MASK
#define ADLSMP_SHORT      0x00

//转换精度 8, 12, 10, 或者 16 (单端).
#define MODE_8            0x00
#define MODE_12           0x01
#define MODE_10           0x02
#define MODE_16           0x03


//ADC输入时钟源选择 总线，总线/2，”altclk“或者ADC自身异步时钟以减少噪声
#define ADICLK_BUS        0x00
#define ADICLK_BUS_2      0x01
#define ADICLK_ALTCLK     0x02
#define ADICLK_ADACK      0x03

// ADCCFG2
//选择通道A，通道B
#define MUXSEL_ADCB       ADC_CFG2_MUXSEL_MASK
#define MUXSEL_ADCA       0x00

//异步时钟输出使能：使能，或者禁止输出
#define ADACKEN_ENABLED   ADC_CFG2_ADACKEN_MASK
#define ADACKEN_DISABLED  0x00

//高速低速转换时间
#define ADHSC_HISPEED     ADC_CFG2_ADHSC_MASK
#define ADHSC_NORMAL      0x00

//长采样时间选择：20,12,6或者2个额外的时钟对于长采样时间
#define ADLSTS_20          0x00
#define ADLSTS_12          0x01
#define ADLSTS_6           0x02
#define ADLSTS_2           0x03

//ADCSC2
//只读状态位只是转换状态
#define ADACT_ACTIVE       ADC_SC2_ADACT_MASK
#define ADACT_INACTIVE     0x00


//触发开始转换:硬件触发，软件触发
#define ADTRG_HW           ADC_SC2_ADTRG_MASK
#define ADTRG_SW           0x00

// ADC Compare Function Enable: Disabled, or Enabled.
//ADC比较功能使能：禁止或者使能
#define ACFE_DISABLED      0x00
#define ACFE_ENABLED       ADC_SC2_ACFE_MASK

// Compare Function Greater Than Enable: Greater, or Less.
//比较功能大于比较使能：大于或者小于
#define ACFGT_GREATER      ADC_SC2_ACFGT_MASK
#define ACFGT_LESS         0x00

// Compare Function Range Enable: Enabled or Disabled.
//比较功能范围使能：使能或者禁止
#define ACREN_ENABLED      ADC_SC2_ACREN_MASK
#define ACREN_DISABLED     0x00

// DMA enable: enabled or disabled.
//DMA使能：使能或者禁止
#define DMAEN_ENABLED      ADC_SC2_DMAEN_MASK
#define DMAEN_DISABLED     0x00

//ADC转换的电压参考选择
#define REFSEL_EXT         0x00
#define REFSEL_ALT         0x01
#define REFSEL_RES         0x02     //预留
#define REFSEL_RES_EXT     0x03     //预留默认指向Vref

//ADCSC3

//校准开始或者关闭
#define CAL_BEGIN          ADC_SC3_CAL_MASK
#define CAL_OFF            0x00


//指示校准失败成功的状态
#define CALF_FAIL          ADC_SC3_CALF_MASK
#define CALF_NORMAL        0x00


//ADC连续转换或者一次转换
#define ADCO_CONTINUOUS    ADC_SC3_ADCO_MASK
#define ADCO_SINGLE        0x00

//平均采样使能或者禁止
#define AVGE_ENABLED       ADC_SC3_AVGE_MASK
#define AVGE_DISABLED      0x00


//MCU产生中断前的平均次数4,8,16，或者32
#define AVGS_4             0x00
#define AVGS_8             0x01
#define AVGS_16            0x02
#define AVGS_32            0x03




typedef struct adc_cfg {
  uint8_t  CONFIG1; 
  uint8_t  CONFIG2; 
  uint16_t COMPARE1; 
  uint16_t COMPARE2; 
  uint8_t  STATUS2;
  uint8_t  STATUS3; 
  uint8_t  STATUS1A; 
  uint8_t  STATUS1B;
  uint32_t PGA;
  } *tADC_ConfigPtr, tADC_Config ;  
  

#define CAL_BLK_NUMREC 18 

typedef struct adc_cal {
 
uint16_t  OFS;
uint16_t  PG;
uint16_t  MG;
uint8_t   CLPD;
uint8_t   CLPS;
uint16_t  CLP4;
uint16_t  CLP3;
uint8_t   CLP2;
uint8_t   CLP1;
uint8_t   CLP0;
uint8_t   dummy;
uint8_t   CLMD;
uint8_t   CLMS;
uint16_t  CLM4;
uint16_t  CLM3;
uint8_t   CLM2;
uint8_t   CLM1;
uint8_t   CLM0;
} tADC_Cal_Blk ;  

   

#define ADC1_CHANA    26      


#define ADC0_DLYA     0x2000                                // ADC0 触发A延迟 
#define ADC0_DLYB     0x4000                                // ADC0 触发B延迟
#define ADC1_DLYA     0x6000                                // ADC1 触发A延迟 
#define ADC1_DLYB     0x7fff                                // ADC1 触发B延迟


#define ADC0A_DONE   0x01       
#define ADC0B_DONE   0x02       
#define ADC1A_DONE   0x04       
#define ADC1B_DONE   0x08     

//函数接口声明

//============================================================================
//函数名称：hw_adc_init
//函数返回：0 成功 ，1 失败
//参数说明：MoudelNumber：模块号
//功能概要：AD初始化
//============================================================================
uint8 hw_adc_init(int MoudelNumber);




//============================================================================
//函数名称：hw_adc_convertstop
//函数返回：0 成功 ，1 失败
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//功能概要：停止ADC转换  
//============================================================================
uint8 hw_adc_convertstop(int MoudelNumbe,int Channel);



//============================================================================
//函数名称：hw_adc_convertstart
//函数返回：0 成功 ，1 失败
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//功能概要：开始Adc转换
//============================================================================
uint8 hw_adc_convertstart(int MoudelNumber,int Channel,uint8 accuracy);



//============================================================================
//函数名称：hw_ad_once
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//功能概要：采集一次一路模拟量的AD值    
//============================================================================
uint16 hw_ad_once(int MoudelNumber,int Channel,uint8 accuracy); //采集某路模拟量的AD值

//============================================================================
//函数名称：hw_ad_mid
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//功能概要：中值滤波后的结果(范围:0-4095) 
//============================================================================
uint16 hw_ad_mid(int MoudelNumber,int Channel,uint8 accuracy); //中值滤波




//============================================================================
//函数名称：hw_ad_ave
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//                     N:均值滤波次数(范围:0~255)
//功能概要：均值滤波后的结果(范围:0-4095) 
//============================================================================
uint16 hw_ad_ave(int MoudelNumber,int Channel,uint8 accuracy,uint8 N); 





//============================================================================
//函数名称：hw_adc_config_alt
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：adcmap：adc基址寄存器地址
//          ADC_CfgPtr: 存放 寄存器值的结构体    
//功能概要：将adc寄存器结构体配置进adc寄存器 
//============================================================================
void hw_adc_config_alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr);
#endif /* __ADC_H__ */
