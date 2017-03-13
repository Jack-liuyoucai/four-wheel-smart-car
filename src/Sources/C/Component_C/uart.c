//-------------------------------------------------------------------------*
// �ļ���: uart.c                                                          *
// ˵  ��: uart����Դ�ļ�                                                  *
//-------------------------------------------------------------------------*

#include "uart.h"

//-------------------------------------------------------------------------*
//������: uart_init                                                        *
//��  ��: ��ʼ��uartxģ�顣                                                *
//��  ��: uartch:���ں�                                                    *
//        sysclk:ϵͳ����ʱ�ӣ���MHzΪ��λ                                 *
//        baud:�����ʣ���9600��38400�ȣ�һ����˵���ٶ�Խ����ͨ��Խ��       *
//��  ��: ��                                                               *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
void uart_init (UART_MemMapPtr uartch, uint32 sysclk, uint32 baud)
{
	register uint16 sbr, brfa;
	uint8 temp;

	//ʹ������
	if (uartch == UART0_BASE_PTR)
	{
		//��PTD6��ʹ��UART0_TXD����
		PORTD_PCR6 = PORT_PCR_MUX(0x3);
		//��PTD7��ʹ��UART0_RXD
		PORTD_PCR7 = PORT_PCR_MUX(0x3); 
	}else if (uartch == UART1_BASE_PTR)
	{
		//��PTC4��ʹ��UART1_TXD����
		PORTC_PCR4 = PORT_PCR_MUX(0x3); 
		
		//��PTC3��ʹ��UART1_RXD
		PORTC_PCR3 = PORT_PCR_MUX(0x3); 
	}else if (uartch == UART2_BASE_PTR)
	{
		//��PTD3��ʹ��UART2_TXD����
		PORTD_PCR3 = PORT_PCR_MUX(0x3); 
		//��PTD2��ʹ��UART2_RXD
		PORTD_PCR2 = PORT_PCR_MUX(0x3); 
	}else if (uartch == UART3_BASE_PTR)
	{
		//��PTC17��ʹ��UART3_TXD����
		PORTC_PCR17 = PORT_PCR_MUX(0x3); 
		//��PTC16��ʹ��UART3_RXD
		PORTC_PCR16 = PORT_PCR_MUX(0x3); 
	}else if (uartch == UART4_BASE_PTR)
	{
		//��PTE24��ʹ��UART4_TXD����
		PORTE_PCR24 = PORT_PCR_MUX(0x3); 
		//��PTE25��ʹ��UART4_RXD
		PORTE_PCR25 = PORT_PCR_MUX(0x3); 
	}else if (uartch == UART5_BASE_PTR)
	{
		//��PTE8��ʹ��UART5_TXD����
		PORTE_PCR8 = PORT_PCR_MUX(0x3); 
		//��PTE9��ʹ��UART5_RXD
		PORTE_PCR9 = PORT_PCR_MUX(0x3); 
	}
	 
	//ʹ�ܴ���ʱ��    
	if(uartch == UART0_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	else
		if (uartch == UART1_BASE_PTR)
			SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
		else
			if (uartch == UART2_BASE_PTR)
				SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
			else
				if(uartch == UART3_BASE_PTR)
					SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
				else
					if(uartch == UART4_BASE_PTR)
						SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
					else
						SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
								
	//��ֹ���ͽ���
	UART_C2_REG(uartch) &= ~(UART_C2_TE_MASK
				| UART_C2_RE_MASK );
	
	//���ó�8λ��У��ģʽ
	UART_C1_REG(uartch) = 0;
	
	//���㲨���ʣ�����0��1ʹ���ں�ʱ�ӣ���������ʹ������ʱ�ӣ�ϵͳʱ��Ϊ
	//����ʱ�ӵ�2��
	if ((uartch == UART0_BASE_PTR) | (uartch == UART1_BASE_PTR))//
		sysclk+=sysclk;
	
	sbr = (uint16)((sysclk*1000)/(baud * 16));
	temp = UART_BDH_REG(uartch) & ~(UART_BDH_SBR(0x1F));
	UART_BDH_REG(uartch) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
	UART_BDL_REG(uartch) = (uint8)(sbr & UART_BDL_SBR_MASK);
	brfa = (((sysclk*32000)/(baud * 16)) - (sbr * 32));
	temp = UART_C4_REG(uartch) & ~(UART_C4_BRFA(0x1F));
	UART_C4_REG(uartch) = temp |  UART_C4_BRFA(brfa);    
	
	//ʹ�ܷ��ͽ���
	UART_C2_REG(uartch) |= (UART_C2_TE_MASK
				| UART_C2_RE_MASK );
}

//-------------------------------------------------------------------------*
//������: uart_re1                                                         *
//��  ��: ���н���1���ֽ�                                                  *
//��  ��: uartch: ���ں�                                                   *
//         ch:    ���յ����ֽ�                                             *
//��  ��: �ɹ�:1;ʧ��:0                                                    *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
uint8 uart_re1 (UART_MemMapPtr uartch,uint8 *ch)
{
    uint32 k;
    
    for (k = 0; k < 0xfbbb; k++)//��ʱ������
		if((UART_S1_REG(uartch) & UART_S1_RDRF_MASK)!= 0)//�жϽ��ջ������Ƿ���
		{
			*ch = UART_D_REG(uartch);
			return 1; 			//���ܳɹ�
		} 
	if(k>=0xfbbb) 
	{
		return 0;			//����ʧ��
	} 
    return 0;
}

//-------------------------------------------------------------------------*
//������: uart_send1                                                       *
//��  ��: ���з���1���ֽ�                                                  *
//��  ��: uartch: ���ں�                                                   *
//         ch:    Ҫ���͵��ֽ�                                             *
//��  ��: ��                                                               *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
void uart_send1 (UART_MemMapPtr uartch, uint8 ch)
{
    //�ȴ����ͻ�������
    while(!(UART_S1_REG(uartch) & UART_S1_TDRE_MASK));
    //��������
    UART_D_REG(uartch) = (uint8)ch;
 }

//-------------------------------------------------------------------------*
//������: uart_reN                                                         *
//��  ��: ���� ����n���ֽ�                                                 *
//��  ��: uartch: ���ں�                                                   *
//        buff: ���ջ�����                                                 *
//		  len:���ճ���                                             *
//��  ��: 1:�ɹ�;0:ʧ��                                                    *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
uint8 uart_reN (UART_MemMapPtr uartch ,uint8* buff,uint16 len)
{
    uint16 m=0; 
    while (m < len)
    { 	          
  	    if(0==uart_re1(uartch,&buff[m]))
  	    	return 0;  //����ʧ��
  	    else m++;
    } 
    
    return 1;          //���ճɹ�
    
}

//-------------------------------------------------------------------------*
//������: uart_sendN                                                       *
//��  ��: ���� ����n���ֽ�                                                 *
//��  ��: uartch: ���ں�                                                   *
//        buff: ���ͻ�����                                                 *
//		  len:���ͳ���                                             *
//��  ��: ��                                                               *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
void uart_sendN (UART_MemMapPtr uartch ,uint8* buff,uint16 len)
{
    int i;
	for(i=0;i<len;i++)
    {
		uart_send1(uartch,buff[i]);
    }
}

//-------------------------------------------------------------------------*
//������: enableuartreint                                                  *
//��  ��: �����ڽ����ж�                                                   *
//��  ��: uartch: ���ں�                                                   *
//        irqno: ��Ӧirq��                                                 *
//��  ��: ��                                                               *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
void enableuartreint(UART_MemMapPtr uartch,uint8 irqno)
{
	UART_C2_REG(uartch)|=UART_C2_RIE_MASK;   //����UART�����ж�
	enable_irq(irqno);			 //���������ŵ�IRQ�ж�
}

//-------------------------------------------------------------------------*
//������: disableuartreint                                                 *
//��  ��: �ش��ڽ����ж�                                                   *
//��  ��: uartch: ���ں�                                                   *
//        irqno: ��Ӧirq��                                                 *
//��  ��: ��                                                               *
//˵  ��:                                                                  *
//-------------------------------------------------------------------------*
void disableuartreint(UART_MemMapPtr uartch,uint8 irqno)
{
	UART_C2_REG(uartch)&=~UART_C2_RIE_MASK;   //��ֹUART�����ж�
	disable_irq(irqno);			  //�ؽ������ŵ�IRQ�ж�
}















