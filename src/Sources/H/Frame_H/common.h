//-------------------------------------------------------------------------*
// �ļ���: common.h (ͨ��ͷ�ļ�)                                           *
// ˵  ��: �����˼Ĵ���ӳ��ͷ�ļ������ͺ궨���                            *
//-------------------------------------------------------------------------*

#ifndef _COMMON_H_
#define _COMMON_H_

    //1 ͷ�ļ�
    #include "MK60N512VMD100.h"   //�Ĵ���ӳ��ͷ�ļ�
    
    //2 �궨��
    //2.1 �����жϵĺ궨��    
    #define ARM_INTERRUPT_LEVEL_BITS     4//�ж����ȼ��궨��
    #define EnableInterrupts asm(" CPSIE i");//�����ж�
    #define DisableInterrupts asm(" CPSID i");//�����ж�
    
    
    //2.2 ���ͱ����궨��
    typedef unsigned char	uint8;  /*  8 bits */
    typedef unsigned short int	uint16; /* 16 bits */
    typedef unsigned long int	uint32; /* 32 bits */
    
    typedef char		int8;   /*  8 bits */
    typedef short int	       	int16;  /* 16 bits */
    typedef int		       	int32;  /* 32 bits */
    
    typedef volatile int8	vint8;  /*  8 bits */
    typedef volatile int16	vint16; /* 16 bits */
    typedef volatile int32	vint32; /* 32 bits */
    
    typedef volatile uint8	vuint8;  /*  8 bits */
    typedef volatile uint16	vuint16; /* 16 bits */
    typedef volatile uint32	vuint32; /* 32 bits */

    //3 ��������
    //-------------------------------------------------------------------------*
    //������: stop                                                             *
    //��  ��: ����CPUΪSTOPģʽ                                                * 
    //��  ��: ��						 	       *	
    //��  ��: ��                                                               *
    //˵  ��: ��                                                               *
    //-------------------------------------------------------------------------*
    void stop (void);
    
    //-------------------------------------------------------------------------*
    //������: wait                                                             *
    //��  ��: ����CPUΪWAITģʽ                                                * 
    //��  ��: ��					     		       *	
    //��  ��: ��                                                               *
    //˵  ��: ��                                                               *
    //-------------------------------------------------------------------------*
    void wait (void);
    
    //-------------------------------------------------------------------------*
    //������: write_vtor                                                       *
    //��  ��: �����ж�������ƫ�ƼĴ�����ֵ                                     * 
    //��  ��: Ҫ���ĵ�ֵ    						       *	
    //��  ��: ��                                                               *
    //˵  ��: ��                                                               *
    //-------------------------------------------------------------------------*
    void write_vtor (int);
    
    //-------------------------------------------------------------------------*
    //������: enable_irq                                                       *
    //��  ��: ʹ��irq�ж�                                                      * 
    //��  ��: irq:irq��       						       *	
    //��  ��: ��                                                               *
    //˵  ��: irq�Ų����ж�������                                              *
    //-------------------------------------------------------------------------*
    void enable_irq (int);
    
    //-------------------------------------------------------------------------*
    //������: disable_irq                                                      *
    //��  ��: ��ֹirq�ж�                                                      * 
    //��  ��: irq:irq��       						       *	
    //��  ��: ��                                                               *
    //˵  ��: irq�Ų����ж�������                                              *
    //-------------------------------------------------------------------------*
    void disable_irq (int);
    
    //-------------------------------------------------------------------------*
    //������: set_irq_priority                                                 *
    //��  ��: ����irq�жϺ����ȼ�                                              * 
    //��  ��: irq:irq��         					       *	
    //        prio:���ȼ�						       *	
    //��  ��: ��                                                               *
    //˵  ��: irq�Ų����ж�������                                              *
    //-------------------------------------------------------------------------*
    void set_irq_priority (int, int);
    
    //-------------------------------------------------------------------------*
    //������: main                                                             *
    //��  ��: ����������                                                       * 
    //��  ��: ��         						       *	
    //��  ��: ��                                                               *
    //˵  ��: ��                                                               *
    //-------------------------------------------------------------------------*
    void main(void);
#endif 

/* stdbool.h header */
/* Copyright 2003-2010 IAR Systems AB.  */

/* NOTE: IAR Extensions must be enabled in order to use the bool type! */

#ifndef _STDBOOL
#define _STDBOOL

#ifndef _SYSTEM_BUILD
  #pragma system_include
#endif


#ifndef __C99_BOOL__
  #error "<stdbool.h>  compiled with wrong (version of IAR) compiler"
#endif

#ifndef __cplusplus

#define bool _Bool
#define true 1
#define false 0

#endif /* !__cplusplus */

#define __bool_true_false_are_defined 1

#endif /* !_STDBOOL */

/*
 * Copyright (c) 1992-2009 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V5.04:0576 */
