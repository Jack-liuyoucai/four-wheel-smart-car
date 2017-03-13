//-------------------------------------------------------------------------*
//�ļ���: includes.h                                                       *
//˵  ��: ��ͷ�ļ�,���ļ�����:                                             *
//        ������(main)�ļ����õ���ͷ�ļ����ⲿ�����������йس�������       *
//-------------------------------------------------------------------------*
#ifndef INCLUDE_H_
#define INCLUDE_H_

    //1 ͷ�ļ�
    //1.1ͨ��ͷ�ļ�
    #include "common.h"            //ͨ�ú���ͷ�ļ�
            
    //1.2��������Ӳ������ͷ�ļ�(������ģ��)       
    #include  "light.h"                  //С�ƹ���ͷ�ļ�
    
    //2 �궨��

#endif
//-------------------------------------------------------------------------
// �ļ����ƣ�hw_adc.h                                                          
// ���ܸ�Ҫ��adc����ͷ�ļ�
// ��Ȩ����: ���ݴ�ѧ��˼����Ƕ��ʽ����(sumcu.suda.edu.cn)
// �汾����:    ʱ��                         �汾                     ����                          �޸�
//           2011-11-17     V1.0        stone    ��д��K60��ADC����
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

// ADC �ж�: ʹ�ܻ��߽�ֹ
#define AIEN_ON           ADC_SC1_AIEN_MASK
#define AIEN_OFF          0x00

//��ֻ��ߵ���ADC����
#define DIFF_SINGLE       0x00
#define DIFF_DIFFERENTIAL ADC_SC1_DIFF_MASK

// ADCCFG1

//ADC��Դ���� 
#define ADLPC_LOW         ADC_CFG1_ADLPC_MASK
#define ADLPC_NORMAL      0x00

//ʱ�ӷ�Ƶ
#define ADIV_1            0x00
#define ADIV_2            0x01
#define ADIV_4            0x02
#define ADIV_8            0x03

//������ʱ����߶̲���ʱ��
#define ADLSMP_LONG       ADC_CFG1_ADLSMP_MASK
#define ADLSMP_SHORT      0x00

//ת������ 8, 12, 10, ���� 16 (����).
#define MODE_8            0x00
#define MODE_12           0x01
#define MODE_10           0x02
#define MODE_16           0x03


//ADC����ʱ��Դѡ�� ���ߣ�����/2����altclk������ADC�����첽ʱ���Լ�������
#define ADICLK_BUS        0x00
#define ADICLK_BUS_2      0x01
#define ADICLK_ALTCLK     0x02
#define ADICLK_ADACK      0x03

// ADCCFG2
//ѡ��ͨ��A��ͨ��B
#define MUXSEL_ADCB       ADC_CFG2_MUXSEL_MASK
#define MUXSEL_ADCA       0x00

//�첽ʱ�����ʹ�ܣ�ʹ�ܣ����߽�ֹ���
#define ADACKEN_ENABLED   ADC_CFG2_ADACKEN_MASK
#define ADACKEN_DISABLED  0x00

//���ٵ���ת��ʱ��
#define ADHSC_HISPEED     ADC_CFG2_ADHSC_MASK
#define ADHSC_NORMAL      0x00

//������ʱ��ѡ��20,12,6����2�������ʱ�Ӷ��ڳ�����ʱ��
#define ADLSTS_20          0x00
#define ADLSTS_12          0x01
#define ADLSTS_6           0x02
#define ADLSTS_2           0x03

//ADCSC2
//ֻ��״̬λֻ��ת��״̬
#define ADACT_ACTIVE       ADC_SC2_ADACT_MASK
#define ADACT_INACTIVE     0x00


//������ʼת��:Ӳ���������������
#define ADTRG_HW           ADC_SC2_ADTRG_MASK
#define ADTRG_SW           0x00

// ADC Compare Function Enable: Disabled, or Enabled.
//ADC�ȽϹ���ʹ�ܣ���ֹ����ʹ��
#define ACFE_DISABLED      0x00
#define ACFE_ENABLED       ADC_SC2_ACFE_MASK

// Compare Function Greater Than Enable: Greater, or Less.
//�ȽϹ��ܴ��ڱȽ�ʹ�ܣ����ڻ���С��
#define ACFGT_GREATER      ADC_SC2_ACFGT_MASK
#define ACFGT_LESS         0x00

// Compare Function Range Enable: Enabled or Disabled.
//�ȽϹ��ܷ�Χʹ�ܣ�ʹ�ܻ��߽�ֹ
#define ACREN_ENABLED      ADC_SC2_ACREN_MASK
#define ACREN_DISABLED     0x00

// DMA enable: enabled or disabled.
//DMAʹ�ܣ�ʹ�ܻ��߽�ֹ
#define DMAEN_ENABLED      ADC_SC2_DMAEN_MASK
#define DMAEN_DISABLED     0x00

//ADCת���ĵ�ѹ�ο�ѡ��
#define REFSEL_EXT         0x00
#define REFSEL_ALT         0x01
#define REFSEL_RES         0x02     //Ԥ��
#define REFSEL_RES_EXT     0x03     //Ԥ��Ĭ��ָ��Vref

//ADCSC3

//У׼��ʼ���߹ر�
#define CAL_BEGIN          ADC_SC3_CAL_MASK
#define CAL_OFF            0x00


//ָʾУ׼ʧ�ܳɹ���״̬
#define CALF_FAIL          ADC_SC3_CALF_MASK
#define CALF_NORMAL        0x00


//ADC����ת������һ��ת��
#define ADCO_CONTINUOUS    ADC_SC3_ADCO_MASK
#define ADCO_SINGLE        0x00

//ƽ������ʹ�ܻ��߽�ֹ
#define AVGE_ENABLED       ADC_SC3_AVGE_MASK
#define AVGE_DISABLED      0x00


//MCU�����ж�ǰ��ƽ������4,8,16������32
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


#define ADC0_DLYA     0x2000                                // ADC0 ����A�ӳ� 
#define ADC0_DLYB     0x4000                                // ADC0 ����B�ӳ�
#define ADC1_DLYA     0x6000                                // ADC1 ����A�ӳ� 
#define ADC1_DLYB     0x7fff                                // ADC1 ����B�ӳ�


#define ADC0A_DONE   0x01       
#define ADC0B_DONE   0x02       
#define ADC1A_DONE   0x04       
#define ADC1B_DONE   0x08     

//�����ӿ�����

//============================================================================
//�������ƣ�hw_adc_init
//�������أ�0 �ɹ� ��1 ʧ��
//����˵����MoudelNumber��ģ���
//���ܸ�Ҫ��AD��ʼ��
//============================================================================
uint8 hw_adc_init(int MoudelNumber);




//============================================================================
//�������ƣ�hw_adc_convertstop
//�������أ�0 �ɹ� ��1 ʧ��
//����˵����MoudelNumber��ģ���
//               Channel��ͨ����
//���ܸ�Ҫ��ֹͣADCת��  
//============================================================================
uint8 hw_adc_convertstop(int MoudelNumbe,int Channel);



//============================================================================
//�������ƣ�hw_adc_convertstart
//�������أ�0 �ɹ� ��1 ʧ��
//����˵����MoudelNumber��ģ���
//               Channel��ͨ����
//              accuracy������
//���ܸ�Ҫ����ʼAdcת��
//============================================================================
uint8 hw_adc_convertstart(int MoudelNumber,int Channel,uint8 accuracy);



//============================================================================
//�������ƣ�hw_ad_once
//�������أ��޷��Ž��ֵ(��Χ:0-4095) 
//����˵����MoudelNumber��ģ���
//               Channel��ͨ����
//              accuracy������
//���ܸ�Ҫ���ɼ�һ��һ·ģ������ADֵ    
//============================================================================
uint16 hw_ad_once(int MoudelNumber,int Channel,uint8 accuracy); //�ɼ�ĳ·ģ������ADֵ

//============================================================================
//�������ƣ�hw_ad_mid
//�������أ��޷��Ž��ֵ(��Χ:0-4095) 
//����˵����MoudelNumber��ģ���
//               Channel��ͨ����
//              accuracy������
//���ܸ�Ҫ����ֵ�˲���Ľ��(��Χ:0-4095) 
//============================================================================
uint16 hw_ad_mid(int MoudelNumber,int Channel,uint8 accuracy); //��ֵ�˲�




//============================================================================
//�������ƣ�hw_ad_ave
//�������أ��޷��Ž��ֵ(��Χ:0-4095) 
//����˵����MoudelNumber��ģ���
//               Channel��ͨ����
//              accuracy������
//                     N:��ֵ�˲�����(��Χ:0~255)
//���ܸ�Ҫ����ֵ�˲���Ľ��(��Χ:0-4095) 
//============================================================================
uint16 hw_ad_ave(int MoudelNumber,int Channel,uint8 accuracy,uint8 N); 





//============================================================================
//�������ƣ�hw_adc_config_alt
//�������أ��޷��Ž��ֵ(��Χ:0-4095) 
//����˵����adcmap��adc��ַ�Ĵ�����ַ
//          ADC_CfgPtr: ��� �Ĵ���ֵ�Ľṹ��    
//���ܸ�Ҫ����adc�Ĵ����ṹ�����ý�adc�Ĵ��� 
//============================================================================
void hw_adc_config_alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr);
#endif /* __ADC_H__ */
