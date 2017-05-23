#include "Mouse_Drive_Init.h"
/*********************************************************************************************************
  ����ȫ�ֱ���
*********************************************************************************************************/
#define SYS_CLK   72000000
#define ADC1_DR_Address   ((u32)0x40012400+0x4c)
__IO uint16_t ADC_ConvertedValue[5];
/*********************************************************************************************************
** Function name:       SysTick_Configuration
** Descriptions:        ϵͳ���Ķ�ʱ����ʼ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void SysTick_Configuration(void)
{ 
    if (SysTick_Config(SYS_CLK/1000))     //1ms
    {        
        while (1);
    }
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        �� KEY��START�����г�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __keyInit (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    //������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //KEY
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //����ģʽ����������
    GPIO_Init(GPIOC, &GPIO_InitStructure);  //��ʼ��PC.5
}

void IR_GPIO_Config(void)//GPIO�ڳ�ʼ��  ���⴫���������
{
	GPIO_InitTypeDef GPIO_InitStruct;//������
        
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5; //IR1
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//����
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//f=50MHz
	GPIO_Init(GPIOA,&GPIO_InitStruct);	//A.5
        
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;//IR2
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3; //IR3
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);	
	
        GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;//IR4
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);	
        
        // led����
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12; //IR1
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//����
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//f=50MHz
	GPIO_Init(GPIOB,&GPIO_InitStruct);	//A.5

	GPIO_ResetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_13);
        GPIO_ResetBits(GPIOA,GPIO_Pin_3|GPIO_Pin_5);

        // led����
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void AD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;//ģ������
	GPIO_Init(GPIOC,&GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4|GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
        GPIO_Init(GPIOA,&GPIO_InitStruct);  
}

void AD_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStruct;//������
        
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//APB2����ʱ��ʹ��ADC1
	
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;//ADC1��ADC2��������ģʽ
	ADC_InitStruct.ADC_ScanConvMode=ENABLE; //ENABLE;��ͨ��ģʽ��DISABLE;��ͨ��ģʽ
	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;  //ENABLE �������� DISABLE;��ͨ�β���
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//�������������
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;//�����Ҷ���
	ADC_InitStruct.ADC_NbrOfChannel=5;  //���й���ת����ADC��Ŀ
	
	ADC_Init(ADC1,&ADC_InitStruct);
        
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//adcʱ��Ƶ��=PLCK2/8=9M
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_71Cycles5);//9M/239.5=37.58k 26.61us����һ��ת��
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,2,ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,3,ADC_SampleTime_71Cycles5);   //����ת������3��ͨ��15     
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,4,ADC_SampleTime_71Cycles5);
        ADC_RegularChannelConfig(ADC1,ADC_Channel_4,5,ADC_SampleTime_71Cycles5);
	ADC_DMACmd(ADC1,ENABLE);//ADC1 DMAͨ��ʹ��
	ADC_Cmd(ADC1,ENABLE);   //ADC1ʹ��
	ADC_ResetCalibration(ADC1);//ʹ��ǰ����У׼
	while(ADC_GetResetCalibrationStatus(ADC1));//���У׼�Ǽǽ���ADC1��λ
	ADC_StartCalibration(ADC1);//��ʼADCУ׼
	while(ADC_GetCalibrationStatus(ADC1));//������־λ
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//У׼���� ��ʼADC
}

void AD_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct; //����һ��DMA�ṹ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//����DMAʱ��
	
	DMA_DeInit(DMA1_Channel1); //��DMA1ͨ��
	DMA_InitStruct.DMA_PeripheralBaseAddr=ADC1_DR_Address;//����DMA���������ַ
	DMA_InitStruct.DMA_MemoryBaseAddr=(u32)(&ADC_ConvertedValue); //�ڴ��ַ��Ҫ����ı� ����ָ�룩
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC; //���򣺴��ڴ浽����
	DMA_InitStruct.DMA_BufferSize=5; //�����С
	
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable; //�����ַ����
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;  //�ڴ��ַ����
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;//�������ݵ�λ ���� 16λ
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;//�ڴ����ݵ�λ ���� 16λ
	
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//DMAģʽ��ѭ������
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;//���ȼ�����
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;//��ֹ�ڴ浽�ڴ洫��
	
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);//����DMA1��һͨ��
	
	DMA_Cmd(DMA1_Channel1,ENABLE);//ʹ��DMA
}

void AD_Init(void)  //���⴫�������ն�
{
	AD_GPIO_Config();   //ADC��GPIO�ڳ�ʼ��
	AD_DMA_Config();    //DMAͨ����ʼ��
	AD_Mode_Config();   //ADCģʽ��ʼ��
}

void Motor_Mode_Config(void)   //�����ʼ��
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; //����ṹ�������
	TIM_OCInitTypeDef TIM_OCInitStruct; //ͬ��
	GPIO_InitTypeDef GPIO_InitStruct;  //ͬ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //����ʱ��ʹ�� GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//��ʱ��ʱ��ʹ��
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //����
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA,&GPIO_InitStruct);//GPIO��ʼ��

	TIM_DeInit(TIM1);  
	
	TIM_TimeBaseInitStruct.TIM_Period = 2880-1;  //�趨�������Զ���װֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0; //Ԥ��Ƶϵ��
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0; //ʱ������TDTS = Tck_tim
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ��� 
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); //������ʼ��ʱ������趨	
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;   //TIM�����ȵ���ģʽ2
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;//�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable; //ʹ��
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High ;//����Ƚϼ���Ϊ�ߵ�ƽ
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High ; //���û����������
	TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Set;//ѡ�����״̬�µķǹ���״̬
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//ѡ�񻥲�״̬�µķǹ���״̬
	
	TIM_OCInitStruct.TIM_Pulse = 0;//���ô�װ�벶��Ƚ���������ֵ  �Ƚ�ֵ
		
	TIM_OC1Init(TIM1, &TIM_OCInitStruct); //������ʼ������TIM1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); //ʹ��TIM��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC2Init(TIM1, &TIM_OCInitStruct); //������ʼ������TIM1
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIM��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC3Init(TIM1, &TIM_OCInitStruct); //������ʼ������TIM1
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIM��CCR3�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC4Init(TIM1, &TIM_OCInitStruct); //������ʼ������TIM1
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//ʹ��TIM��CCR4�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //ʹ��TIM��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
        TIM_SetCompare1(TIM1,0);//ͨ��1�趨�Ƚ�ֵΪ0                                          
        TIM_SetCompare2(TIM1,0);//ͨ��2�趨�Ƚ�ֵΪ0
	TIM_SetCompare4(TIM1,0);//ͨ��3�趨�Ƚ�ֵΪ0                                          
        TIM_SetCompare3(TIM1,0);//ͨ��4�趨�Ƚ�ֵΪ0
	TIM_CtrlPWMOutputs(TIM1, ENABLE); //ʹ��PWM
}


void ENCL_Init(void)//���������ʼ��
{
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //������
  TIM_ICInitTypeDef TIM_ICInitStructure;  //ͬ��
  GPIO_InitTypeDef GPIO_InitStructure;    //ͬ��
  NVIC_InitTypeDef NVIC_InitStructure;    //ͬ��
   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ͬ��
	
  GPIO_StructInit(&GPIO_InitStructure); //GPIOʹ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//���� 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//�������� 
  GPIO_Init(GPIOA, &GPIO_InitStructure); //GPIOʹ��
    
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn; //TM2�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�Ϊ1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //�ж�ʹ��
  NVIC_Init(&NVIC_InitStructure); //������ʼ���ж�ʹ��

  TIM_DeInit(TIM2);  //ʱ��Ԥ����
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); //�ض�������ʼ��
   
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; //  Ԥ��Ƶϵ��          
  TIM_TimeBaseStructure.TIM_Period = 65535;    //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //���ϼ���
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //������ʼ����ʱ��ʹ��
  
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //���ö�ʱ���ı������ӿ�
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6;//�����˲���

  TIM_ICInit(TIM2, &TIM_ICInitStructure); //������ʼ��
   
  TIM_ClearFlag(TIM2,TIM_IT_Update);//���´����ж�
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //�ж�ʹ��

   
  TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
  TIM_SetCounter(TIM2, 0);  //��������ֵ
}

void ENCR_Init(void) //�ұ�������ʼ��  ͬ��
{ 
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct; 
   TIM_ICInitTypeDef TIM_ICInitStruct;  
   GPIO_InitTypeDef GPIO_InitStruct; 
   NVIC_InitTypeDef NVIC_InitStruct; 
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
  GPIO_StructInit(&GPIO_InitStruct); 
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStruct); 
    
  NVIC_InitStruct.NVIC_IRQChannel =TIM3_IRQn; 
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStruct); 

  TIM_DeInit(TIM3);  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct); 
   
  TIM_TimeBaseStruct.TIM_Prescaler =0;          
  TIM_TimeBaseStruct.TIM_Period = 65535-1;   
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct); 
  
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ICStructInit(&TIM_ICInitStruct);        
  TIM_ICInitStruct.TIM_ICFilter = 5; 
  TIM_ICInit(TIM3, &TIM_ICInitStruct); 
   
  TIM_ClearFlag(TIM3,TIM_IT_Update);
  
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 

  //TIM_SetCounter(TIM3, 32768);
  TIM_Cmd(TIM3, ENABLE);  
  TIM_SetCounter(TIM3, 0);
} 

void MouseInit(void)
{
	SysTick_Configuration();//�δ�ʱ��
        IR_GPIO_Config();//���⴫���������
	Motor_Mode_Config();//���
	ENCL_Init();//�������
	ENCR_Init();//�ұ�����
        __keyInit ();//����
        AD_Init();//���⴫�������ն�
	//PIDInit();
}
