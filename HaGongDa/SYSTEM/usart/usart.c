#include "usart.h"
#include "dma.h"
#include "led.h"
#include "adc.h"
#include "can.h"
//////////////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
_sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART2->SR & 0X40) == 0); //ѭ������,ֱ���������
    USART2->DR = (u8) ch;
    return ch;
}
#endif
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA = 0;     //����״̬���

void uart1_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
    //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART ��ʼ������

    USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������

    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); //�������ڿ����ж�

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //����DMA��ʽ����
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); //����DMA��ʽ����
    USART_Cmd(USART1, ENABLE);
}
void uart2_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART2
    //USART1_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART ��ʼ������

    USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART2, &USART_InitStructure); //��ʼ������

    USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //�������ڿ����ж�

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); //����DMA��ʽ����
//	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);//����DMA��ʽ����
    USART_Cmd(USART2, ENABLE);
}
//Uart1_Send[0]	����
//Uart1_Send[1]	�ٶ�
//Uart1_Send[2]	����/����
//Uart1_Send[3]	ֱ��/б��
//Uart1_Send[4]	�½�
//Uart1_Send[5]	��ͣ
//Uart1_Send[6]	����/����
//Uart1_Send[7]	����
float adc_x, adc_y, adc_z;
float flagX, flagY, flagZ;
u16 anjian_temp,DianGang_temp;
u8 jici_flag=0;
int cansendLast[4]={0,0,0,1};
int yaokong_fenxi()
{
	float max = 140.0, min = 110.0;
	float center = 150.0,pianzhi = 10.0;
	adc_z = After_filter[0] * 3.3 / 4096 * 100; //Z
	adc_y = After_filter[1] * 3.3 / 4096 * 100; //Y
	adc_x = After_filter[2] * 3.3 / 4096 * 100; //X
	cansend[3] = YaoGan_Key;
	
	//------����---------------------------
	//X��
	if(adc_x>(center+pianzhi))
		cansend[0] = (adc_x-center-pianzhi)/(292.0-center-pianzhi)*100;
	else
		if(adc_x<(center-pianzhi))
			cansend[0] = -(adc_x-center+pianzhi)/(0.3-center+pianzhi)*100;
		else
			cansend[0] = 0;
	//Y��
	if(adc_y>(center+pianzhi))
		cansend[1] = (adc_y-center-pianzhi)/(292.0-center-pianzhi)*100;
	else
		if(adc_y<(center-pianzhi))
			cansend[1] = -(adc_y-center+pianzhi)/(0.48-center+pianzhi)*100;
		else
			cansend[1] = 0;
	//Z��
	if(adc_z>(center+pianzhi))
		cansend[2] = (adc_z-center-pianzhi)/(288.0-center-pianzhi)*100;
	else
		if(adc_z<(center-pianzhi))
			cansend[2] = -(adc_z-center+pianzhi)/(0.56-center+pianzhi)*100;
		else
			cansend[2] = 0;
		
		cansend[0]=cansend[0]>100?100:cansend[0];
		cansend[1]=cansend[1]>100?100:cansend[1];
		cansend[2]=cansend[2]>100?100:cansend[2];
	

	//�б仯
	if((cansend[0]!=cansendLast[0])||(cansend[1]!=cansendLast[1])||(cansend[2]!=cansendLast[2])||(cansend[3] != YaoGan_Key))
	{
		//���·���
		cansendLast[0] = cansend[0];
		cansendLast[1] = cansend[1];
		cansendLast[2] = cansend[2];
		//��������
		can_send();
		return 1;
	}
	else
		return 0;
}

void usart_send(void)
{
    if(flag.change_flag == 1)
        Uart1_Start_DMA_Tx(3);
}
u16 USART1_Length = 0, USART2_Length = 0;
void USART1_IRQHandler(void)                	//����1�жϷ������
{
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
    OSIntEnter();
#endif
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd(DMA1_Channel5, DISABLE);
        USART1_Length = USART1->SR;
        USART1_Length = USART1->DR;
        USART1_Length = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
        DMA1_Channel5->CNDTR = UART_RX_LEN;

        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
    OSIntExit();
#endif
}
void USART2_IRQHandler(void)                	//����2�жϷ������
{
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
    OSIntEnter();
#endif
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd(DMA1_Channel6, DISABLE);
        USART2_Length = USART2->SR;
        USART2_Length = USART2->DR;
        USART2_Length = UART_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel6);
        DMA1_Channel6->CNDTR = UART_RX_LEN;

        DMA_Cmd(DMA1_Channel6, ENABLE);
    }
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
    OSIntExit();
#endif
}
#endif

