#include "Mouse_Drive_Init.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
#define SYS_CLK   72000000
#define ADC1_DR_Address   ((u32)0x40012400+0x4c)
__IO uint16_t ADC_ConvertedValue[5];
/*********************************************************************************************************
** Function name:       SysTick_Configuration
** Descriptions:        系统节拍定时器初始化。
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        对 KEY和START键进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __keyInit (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    //变量名
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //KEY
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //工作模式：上拉输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化PC.5
}

void IR_GPIO_Config(void)//GPIO口初始化  红外传感器发射端
{
	GPIO_InitTypeDef GPIO_InitStruct;//变量名
        
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5; //IR1
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽
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
        
        // led设置
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12; //IR1
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//f=50MHz
	GPIO_Init(GPIOB,&GPIO_InitStruct);	//A.5

	GPIO_ResetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_13);
        GPIO_ResetBits(GPIOA,GPIO_Pin_3|GPIO_Pin_5);

        // led设置
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void AD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;//模拟输入
	GPIO_Init(GPIOC,&GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4|GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
        GPIO_Init(GPIOA,&GPIO_InitStruct);  
}

void AD_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStruct;//变量名
        
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//APB2总线时钟使能ADC1
	
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;//ADC1和ADC2独立工作模式
	ADC_InitStruct.ADC_ScanConvMode=ENABLE; //ENABLE;多通道模式，DISABLE;单通道模式
	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;  //ENABLE 连续采样 DISABLE;单通次采样
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//采样由软件驱动
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;//数据右对齐
	ADC_InitStruct.ADC_NbrOfChannel=5;  //进行规则转换的ADC数目
	
	ADC_Init(ADC1,&ADC_InitStruct);
        
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//adc时钟频率=PLCK2/8=9M
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_71Cycles5);//9M/239.5=37.58k 26.61us进行一次转换
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,2,ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,3,ADC_SampleTime_71Cycles5);   //常规转换序列3，通道15     
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,4,ADC_SampleTime_71Cycles5);
        ADC_RegularChannelConfig(ADC1,ADC_Channel_4,5,ADC_SampleTime_71Cycles5);
	ADC_DMACmd(ADC1,ENABLE);//ADC1 DMA通道使能
	ADC_Cmd(ADC1,ENABLE);   //ADC1使能
	ADC_ResetCalibration(ADC1);//使用前进行校准
	while(ADC_GetResetCalibrationStatus(ADC1));//检查校准登记结束ADC1复位
	ADC_StartCalibration(ADC1);//开始ADC校准
	while(ADC_GetCalibrationStatus(ADC1));//结束标志位
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//校准结束 开始ADC
}

void AD_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct; //定义一个DMA结构体
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//开启DMA时钟
	
	DMA_DeInit(DMA1_Channel1); //打开DMA1通道
	DMA_InitStruct.DMA_PeripheralBaseAddr=ADC1_DR_Address;//设置DMA传输外设地址
	DMA_InitStruct.DMA_MemoryBaseAddr=(u32)(&ADC_ConvertedValue); //内存地址（要传输的变 量的指针）
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC; //方向：从内存到外设
	DMA_InitStruct.DMA_BufferSize=5; //传输大小
	
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable; //外设地址不增
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;  //内存地址递增
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;//外设数据单位 半字 16位
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;//内存数据单位 半字 16位
	
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//DMA模式：循环发送
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;//优先级：高
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;//禁止内存到内存传输
	
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);//配置DMA1的一通道
	
	DMA_Cmd(DMA1_Channel1,ENABLE);//使能DMA
}

void AD_Init(void)  //红外传感器接收端
{
	AD_GPIO_Config();   //ADC中GPIO口初始化
	AD_DMA_Config();    //DMA通道初始化
	AD_Mode_Config();   //ADC模式初始化
}

void Motor_Mode_Config(void)   //电机初始化
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; //定义结构体变量名
	TIM_OCInitTypeDef TIM_OCInitStruct; //同上
	GPIO_InitTypeDef GPIO_InitStruct;  //同上
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //总线时钟使能 GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//定时器时钟使能
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; //引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA,&GPIO_InitStruct);//GPIO初始化

	TIM_DeInit(TIM1);  
	
	TIM_TimeBaseInitStruct.TIM_Period = 2880-1;  //设定计数器自动重装值
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0; //预分频系数
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0; //时钟设置TDTS = Tck_tim
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//向上计数 
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); //参数初始化时间基数设定	
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;   //TIM脉冲宽度调制模式2
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable; //使能
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High ;//输出比较极性为高电平
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High ; //设置互补输出极性
	TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Set;//选择空闲状态下的非工作状态
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//选择互补状态下的非工作状态
	
	TIM_OCInitStruct.TIM_Pulse = 0;//设置待装入捕获比较器的脉冲值  比较值
		
	TIM_OC1Init(TIM1, &TIM_OCInitStruct); //参数初始化外设TIM1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); //使能TIM在CCR1上的预装载寄存器
	
	TIM_OC2Init(TIM1, &TIM_OCInitStruct); //参数初始化外设TIM1
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM在CCR2上的预装载寄存器
	
	TIM_OC3Init(TIM1, &TIM_OCInitStruct); //参数初始化外设TIM1
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM在CCR3上的预装载寄存器
	
	TIM_OC4Init(TIM1, &TIM_OCInitStruct); //参数初始化外设TIM1
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM在CCR4上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIM在ARR上的预装载寄存器

	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
        TIM_SetCompare1(TIM1,0);//通道1设定比较值为0                                          
        TIM_SetCompare2(TIM1,0);//通道2设定比较值为0
	TIM_SetCompare4(TIM1,0);//通道3设定比较值为0                                          
        TIM_SetCompare3(TIM1,0);//通道4设定比较值为0
	TIM_CtrlPWMOutputs(TIM1, ENABLE); //使能PWM
}


void ENCL_Init(void)//左编码器初始化
{
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //变量名
  TIM_ICInitTypeDef TIM_ICInitStructure;  //同上
  GPIO_InitTypeDef GPIO_InitStructure;    //同上
  NVIC_InitTypeDef NVIC_InitStructure;    //同上
   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //同上
	
  GPIO_StructInit(&GPIO_InitStructure); //GPIO使能
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//引脚 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入 
  GPIO_Init(GPIOA, &GPIO_InitStructure); //GPIO使能
    
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn; //TM2中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //中断使能
  NVIC_Init(&NVIC_InitStructure); //参数初始化中断使能

  TIM_DeInit(TIM2);  //时钟预是能
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); //特定参数初始化
   
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; //  预分频系数          
  TIM_TimeBaseStructure.TIM_Period = 65535;    //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //参数初始化定时器使能
  
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //配置定时器的编码器接口
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6;//输入滤波器

  TIM_ICInit(TIM2, &TIM_ICInitStructure); //参数初始化
   
  TIM_ClearFlag(TIM2,TIM_IT_Update);//更新触发中断
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //中断使能

   
  TIM_Cmd(TIM2, ENABLE); //使能定时器2
  TIM_SetCounter(TIM2, 0);  //计数器初值
}

void ENCR_Init(void) //右编码器初始化  同左
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
	SysTick_Configuration();//滴答定时器
        IR_GPIO_Config();//红外传感器发射端
	Motor_Mode_Config();//电机
	ENCL_Init();//左编码器
	ENCR_Init();//右编码器
        __keyInit ();//按键
        AD_Init();//红外传感器接收端
	//PIDInit();
}
