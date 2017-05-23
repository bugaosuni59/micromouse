#include "Mouse_Drive.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
uint8             GucMouseDir                     = UP;                 /*  保存电脑鼠当前方向          */
uint8             GmcMouse;
static uint32     GW;                                                       /*小车转动角度*/
static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0, 0};    /*  定义并初始化左电机状态      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0, 0};    /*  定义并初始化右电机状态      */
static __PID    __GmLPID;                                                 /*  定义左电机PID      */
static __PID    __GmRPID;                                                 /*  定义右电机PID     */
static __PID    __GmSPID;                                                 /*  直线PID     */
static __PID    __GmWPID;                                                 /*  旋转PID     */
static uint8    __GucMouseState                   = __STOP;             /*  保存电脑鼠当前运行状态      */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  保存允许运行的最大速度      */
static uint8    __GucDistance[5]                  = {0};                /*  记录传感器状态              */
uint16   GusFreq_F                         = 36200;   //33.8,33,327        /*  前方红外频率              */
uint16   GusFreq_FJ                        = 19200;   //26.3,266,275              /*  前方近距红外频率              */
uint16   GusFreq_X                         = 30000;   //35,33.8          /*  斜45度红外频率              */
uint16   GusFreq_LF                        = 31700;   //34000           /*  左右红外远距频率              */
uint16   GusFreq_L                         = 18300;              /*  左右红外近距频率              */
static  int16   GsTpusle_T                       = 0;                  /*  左轮校正减少的速度值              */
static uint8    GuiSpeedCtr                       = 0;//加减速标志位
static uint16   GuiTpusle_LR                      = 0;
static uint16   GuiTpusle_S                       = 0;
static uint8    GucFrontNear                      = 0;
 uint8    GucGoHead                     = 0;
extern u16 voltageDetectRef;//陀螺仪反馈电压
extern __IO uint16_t ADC_ConvertedValue[5];//ADC采样值
uint8 backflag;
float W;//旋转角度控制量
uint16 ucIRCheck[4];//红外接收端电压值
//红外的设定值，最后改成数组形式
uint16 GusDistance_L_Near=764; //左红外近
uint16 GusDistance_L_Mid=437;  //左红外中
uint16 GusDistance_L_Far=192;  //左红外远
uint16 GusDistance_R_Near=628; //右红外近
uint16 GusDistance_R_Mid=410;  //右红外中
uint16 GusDistance_R_Far=240;  //右红外近
uint16 GusDistance_FL_Near =490;  //近距，用来判断停止    原始修改参数500 680    180 192   
uint16 GusDistance_FR_Near =440;
uint16 GusDistance_FL_Far = 245;   //远距用来判断墙壁    测得参数    200    229
uint16 GusDistance_FR_Far = 220;
/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Delay_us(u32 Nus)   
{    
	 SysTick->LOAD=Nus*9;          		        
	 SysTick->CTRL|=0x01;                  
	 while(!(SysTick->CTRL&(1<<16)));   
	 SysTick->CTRL=0X00000000;         
	 SysTick->VAL=0X00000000;              
}
/*********************************************************************************************************
** Function name:       PIDInit
** Descriptions:        PID初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIDInit(void) 
{  
    __GmLPID.usEncoder_new = 32768;//左编码器计数值
    __GmLPID.usFeedBack = 0 ;  //速度反馈值
    __GmLPID.sFeedBack = 0 ;
    
    __GmRPID.usEncoder_new = 32768;
    __GmRPID.usFeedBack = 0 ;  //速度反馈值
    __GmRPID.sFeedBack = 0 ;
    
    __GmSPID.sRef = 0 ;        //速度设定值 
    __GmSPID.sFeedBack = 0 ;        
    __GmSPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmSPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
        
    __GmSPID.fKp = __KP; 
    __GmSPID.fKi = __KI;
    __GmSPID.fKd = __KD; 
       
    __GmSPID.iPreU = 0 ;      //电机控制输出值 
    
    __GmWPID.sRef = 0 ;        //速度设定值 
    __GmWPID.sFeedBack = 0 ;       
    __GmWPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmWPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
    
    __GmWPID.fKp = __KP;  
    __GmWPID.fKi = __KI;  
    __GmWPID.fKd = __KD; 
       
    __GmWPID.iPreU = 0 ;      //电机控制输出值 
    
}
/*********************************************************************************************************
** Function name:       __SPIDContr
** Descriptions:        直线PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SPIDContr(void) 
{ 
    float  error,d_error,dd_error;
    static uint8   K_I=1;
    error = __GmSPID.sRef - __GmSPID.sFeedBack; // 偏差计算:设定目标速度减去当前速度作为error  PID：error perror and  pperror
    d_error = error - __GmSPID.sPreError; 
    dd_error = d_error - __GmSPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmSPID.sPreError = error; //存储当前偏差 
    __GmSPID.sPreDerror = d_error;
    
    __GmSPID.iPreU += (int16)(  __GmSPID.fKp * d_error + K_I*__GmSPID.fKi * error  + __GmSPID.fKd*dd_error); 
}
/*********************************************************************************************************
** Function name:       __WPIDContr
** Descriptions:        旋转方向PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __WPIDContr(void) 
{ 
    float  error,d_error,dd_error; 
    static uint8   K_I=1;
    error = __GmWPID.sRef + GsTpusle_T- __GmWPID.sFeedBack; // 偏差计算 
    d_error = error - __GmWPID.sPreError; 
    dd_error = d_error - __GmWPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmWPID.sPreError = error; //存储当前偏差 
    __GmWPID.sPreDerror = d_error;
    __GmWPID.iPreU += (int16)(  __GmWPID.fKp * d_error + K_I*__GmWPID.fKi * error  + __GmWPID.fKd*dd_error);
        
}
/*********************************************************************************************************
** Function name:       voltageDetect
** Descriptions:        旋转角度控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void voltageDetect(void)
{
  u16 w;
  if(ADC_ConvertedValue[4]>=voltageDetectRef)
    
  {
     w= ADC_ConvertedValue[4] - voltageDetectRef;
  }
  else
  {
     w= voltageDetectRef - ADC_ConvertedValue[4];
  }
  GW=GW+w;
}
/*********************************************************************************************************
** Function name:      __PIDContr
** Descriptions:        PID控制，通过脉冲数控制电机
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __PIDContr(void)
{
    __SPIDContr();//直行PID控制函数
    __WPIDContr();//旋转PPID控制函数
    __GmLeft.sSpeed = __GmSPID.iPreU - __GmWPID.iPreU ;//左电机占空比
    if(__GmLeft.sSpeed>=0){
     __GmLeft.cDir=__MOTORGOAHEAD; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
    else{
      __GmLeft.cDir=__MOTORGOBACK;
      __GmLeft.sSpeed *=-1; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
      
    __GmRight.sSpeed = __GmSPID.iPreU + __GmWPID.iPreU ;//右电机占空比
    if(__GmRight.sSpeed>=0){
     __GmRight.cDir=__MOTORGOAHEAD; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    else{
      __GmRight.cDir=__MOTORGOBACK;
      __GmRight.sSpeed *=-1; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    __rightMotorContr();
    __leftMotorContr();
    
}
/*********************************************************************************************************
** Function name:       __Encoder
** Descriptions:        采集编码器输出的脉冲
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __Encoder(void)
{
    static u16 Dir_L;
    static u16 Dir_R;
	
   
    __GmLPID.usEncoder_new = TIM_GetCounter(TIM2);//左编码器计数值
    __GmRPID.usEncoder_new = TIM_GetCounter(TIM3);//右编码器计数值
	
    Dir_R=TIM3->CR1;//TIM3_CR1寄存器的值  16位  判断电机正反转
    Dir_R=(Dir_R&0x0010)>>4;//为了判断TIM3_CR1寄存器DIR位的值  1：向下计数  0：向上计数	
	
     Dir_L=TIM2->CR1;//TIM2
    Dir_L=(Dir_L&0x0010)>>4;//同上	
		
	if(Dir_L==1)//向下计数  后退
	{
             __GmLPID.usFeedBack = 32768 - __GmLPID.usEncoder_new;//一定时间内得到左定时器脉冲数
             TIM_SetCounter(TIM2, 32768);//设定计数值为32768
             __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;//左行程总脉冲数	
	     __GmLeft.cRealDir = __MOTORGOBACK;//左电机运行方向
             __GmLPID.sFeedBack= -1*__GmLPID.usFeedBack;//__GmLPID.usFeedBack 的互补值	
	}
	else//向上计数  前进
	{   
            __GmLPID.usFeedBack = __GmLPID.usEncoder_new - 32768;//一定时间内得到左定时器脉冲数
            TIM_SetCounter(TIM2, 32768);//设定计数值为32768
            __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;//左行程总脉冲数
	    __GmLeft.cRealDir = __MOTORGOAHEAD;
            __GmLPID.sFeedBack= __GmLPID.usFeedBack; //__GmLPID.usFeedBack 的相反数
	}
	
	if(Dir_R==1)//向下计数  前进
	{
            __GmRPID.usFeedBack = 32768 - __GmRPID.usEncoder_new;//一定时间内得到左定时器脉冲数
            TIM_SetCounter(TIM3, 32768);//设定计数值为32768
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;//右行程总脉冲数
	    __GmRight.cRealDir = __MOTORGOAHEAD;
	    __GmRPID.sFeedBack = __GmRPID.usFeedBack;//__GmRPID.usFeedBack 的相反数
        }  
	else//向上计数 后退
	{ 
            __GmRPID.usFeedBack = __GmRPID.usEncoder_new - 32768;//一定时间内得到左定时器脉冲数
            TIM_SetCounter(TIM3, 32768);//设定计数值为32768
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;  //右行程总脉冲数 
            __GmRight.cRealDir = __MOTORGOBACK;
            __GmRPID.sFeedBack = -1*__GmRPID.usFeedBack;//__GmRPID.usFeedBack 的相反数
	}
		
		
        __GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;      //编码器采集数值，计算旋转速度与直行速度
        __GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;    
}
/*********************************************************************************************************
** Function name:       __irSendFreq  __irCheck
** Descriptions:        红外传感器
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irSendFreq (int8  __cNumber)
{
    switch (__cNumber) 
    {
      case 1: /*前方红外*/                                                            
          GPIO_SetBits(GPIOA,GPIO_Pin_5);  //前方左 开
           break;
  
      case 2:  /*左右红外*/                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_13); //前方右 开
          break;  
      case 3:                                                         
          GPIO_SetBits(GPIOA,GPIO_Pin_3); //左 开
          break; 
      case 4:                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_2); //右 开
          break;           
      default:
          break;
    }
}
void __irCheck (void)
{      
    static uint8 ucState = 0;//只定义一次 有记忆
    switch(ucState)
    {
        case 0:
          ucIRCheck[3] = ADC_ConvertedValue[3];    //获取红外数据
          GPIO_ResetBits(GPIOC,GPIO_Pin_2); 
          if(ucIRCheck[3]>GusDistance_R_Far)    //（有红外近）右上红外大于阈值，则右侧有墙，bit0置为1.
          {
              __GucDistance[__RIGHT]  |= 0x01;//右侧有墙但比较远
          }         
          else
          {
              __GucDistance[__RIGHT]  &= 0xfe;            
          }
          
          if(ucIRCheck[3]>GusDistance_R_Mid)//右侧有墙 更近了
          {
              __GucDistance[__RIGHT]  |= 0x02;  // __GucDistance[__RIGHT] =0x03 
          }
          
          else
          {
              __GucDistance[__RIGHT]  &= 0xfd;
          }  
          
           if(ucIRCheck[3]>GusDistance_R_Near)//右侧有墙 很近了
          {
              __GucDistance[__RIGHT]  |= 0x04;// __GucDistance[__RIGHT] =0x07
          }
          
          else
          {
              __GucDistance[__RIGHT]  &= 0xfb;//右侧无墙__GucDistance[__RIGHT] =0x03
          }           
          __irSendFreq (1);
          break;
          
        case 1:
          ucIRCheck[0] = ADC_ConvertedValue[0];
          GPIO_ResetBits(GPIOA,GPIO_Pin_5);
          
          if(ucIRCheck[0]>GusDistance_FL_Far)
          {
              __GucDistance[__FRONTL]  |= 0x01;
          }
          else
          {
               __GucDistance[__FRONTL] &= 0xfe;
          }  
          if(ucIRCheck[0]>GusDistance_FL_Near)
          {
              __GucDistance[__FRONTL]  |= 0x02; //__GucDistance[__FRONTL]=0x03
          }
          else
          {
              __GucDistance[__FRONTL]  &= 0xfd;
          }
          __irSendFreq (2);            
          break;
          
        case 2:          
          ucIRCheck[1] = ADC_ConvertedValue[1];
          GPIO_ResetBits(GPIOC,GPIO_Pin_13);
          if(ucIRCheck[1]>GusDistance_FR_Far)
          {
              __GucDistance[__FRONTR]  |= 0x01;
          }
          else
          {
               __GucDistance[__FRONTR] &= 0xfe;
          } 
          
          if(ucIRCheck[1]>GusDistance_FL_Near)
          {
              __GucDistance[__FRONTR]  |= 0x02;//__GucDistance[__FRONTR]=0x03
          }
          else
          {
               __GucDistance[__FRONTR]  &= 0xfd;
          }          
          if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))//前方有墙
            GucGoHead =1;//该标准位用于控制电脑鼠启动
          else
            GucGoHead =0;  
          __irSendFreq (3);  
          break;
          
        case 3:
          ucIRCheck[2] = ADC_ConvertedValue[2];
          GPIO_ResetBits(GPIOA,GPIO_Pin_3);
          if(ucIRCheck[2]>GusDistance_L_Far)
          {
              __GucDistance[__LEFT]   |= 0x01;
          }
          else
          {
              __GucDistance[__LEFT]   &= 0xfe;
          }
          
          if(ucIRCheck[2]>GusDistance_L_Mid)
          {
              __GucDistance[__LEFT]   |= 0x02;
          }
          else
          {
              __GucDistance[__LEFT]   &= 0xfd;
          }
          
          if(ucIRCheck[2]>GusDistance_L_Near)
          {
              __GucDistance[__LEFT]   |= 0x04;//同上
          }
          else
          {
              __GucDistance[__LEFT]   &= 0xfb;
          }          
          __irSendFreq (4);  
          break;
          default: 
            break;
    }  
    ucState = (ucState + 1) % 4;      //为了保证中断内不断扫描检测
}
/*********************************************************************************************************
** Function name:       sensorDebug 
** Descriptions:        红外传感器调试
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void sensorDebug (void)
{
    zlg7289Download(2, 0, 0, __GucDistance[__LEFT  ]);//传感器左
    zlg7289Download(2, 1, 0, __GucDistance[__FRONTL]);//传感器前左
    zlg7289Download(2, 2, 0, __GucDistance[__FRONTR]);//传感器前右    
    zlg7289Download(2, 3, 0, __GucDistance[__RIGHT ]);//传感器右
}
/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        右直流电机驱动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __rightMotorContr(void)           //H桥控制  
{
    switch (__GmRight.cDir) 
    {
    case __MOTORGOAHEAD:                                                
      TIM_SetCompare1(TIM1,__GmRight.sSpeed);//PWM波输出占空比，设定比较值
      TIM_SetCompare2(TIM1,0);
        break;

    case __MOTORGOBACK:                                                 
      TIM_SetCompare1(TIM1,0);
      TIM_SetCompare2(TIM1,__GmRight.sSpeed);
        break;
		
    case __MOTORGOSTOP:                                                   
      TIM_SetCompare1(TIM1,0);		
      TIM_SetCompare2(TIM1,0);
        break;

    default:
        break;
    }
}
/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        左直流电机驱动
** input parameters:    __GmLeft.cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __leftMotorContr(void)//H桥控制
{
    switch (__GmLeft.cDir) 
    {
    case __MOTORGOAHEAD:                                            
      TIM_SetCompare4(TIM1,0);                                          
      TIM_SetCompare3(TIM1,__GmLeft.sSpeed);//设定比较值
        break;

    case __MOTORGOBACK:                                                 
      TIM_SetCompare4(TIM1,__GmLeft.sSpeed);                                          
      TIM_SetCompare3(TIM1,0);
        break;
		
    case __MOTORGOSTOP:                                                 
      TIM_SetCompare3(TIM1,0);                                          
      TIM_SetCompare4(TIM1,0);
        break;
		
    default:
        break;
    }
}
/*********************************************************************************************************
** Function name:       __SpeedUp
** Descriptions:        电脑鼠加速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SpeedUp (void)
{
    uint16 Speed;              
    Speed=__GmSPID.sFeedBack;          //编码器速度采集
    if(__GmSPID.sRef<__GiMaxSpeed){       //如果当前速度小于设定直行速度，加速到，，，，加速到MAX
      if(Speed >=__GmSPID.sRef)        //速度大于设定值
      {
        __GmSPID.sRef=__GmSPID.sRef+8;          //进行加速
      }
    }   
}
/*********************************************************************************************************
** Function name:       __SpeedDown
** Descriptions:        电脑鼠减速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SpeedDown (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;         //加减速设定值与实际速度偏差进行自动速度调节
    if(__GmSPID.sRef>=MINSPEED)       //当前速度大于设定最小速度
    {
      if(Speed <=__GmSPID.sRef+3)
      {
       __GmSPID.sRef=__GmSPID.sRef-3;//进行减速
      }
    }
}
/*********************************************************************************************************
** Function name:       TIM2_IROHandler  TIM3_IRQHandler
** Descriptions:        中断函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void TIM2_IROHandler(void)                     //中断服务
{
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除中断标志位
}
void TIM3_IRQHandler(void)//中断服务
{
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//清除中断标志位
}
/*********************************************************************************************************
** Function name:       SysTick_Handler
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SysTick_Handler(void)                                   //其中不做修正，实地调试红外数据
{  
    static int8 n = 0,m = 0,k=0,l=0,a=0,b=0,c=0,t=0,w=0;
    uint16 Sp;
    __Encoder();                           //定时采集编码器数据以及红外数据
    __irCheck ();                          //红外发射端检测
    Sp=__GmSPID.sFeedBack;                 //Sp代表直行速度
      switch (__GmRight.cState) {         //右电机状态选择
          
        
      
        case __MOTORSTOP:                                                   /*  停止，同时清零速度和脉冲值  */
              __GmRight.uiPulse    = 0;
              __GmRight.uiPulseCtr = 0;    //清零右脉冲数
              __GmLeft.uiPulse    = 0;
              __GmLeft.uiPulseCtr = 0;     //清零左脉冲数
              break;
      
        case __WAITONESTEP:                                                       //暂停一步 进行左右微调
              __GmRight.cState = __MOTORRUN;
              if((((ucIRCheck[2]>GusDistance_L_Near)&&(ucIRCheck[3]<GusDistance_R_Near))&&(ucIRCheck[3]>GusDistance_R_Mid)))          //偏左
              {
                GsTpusle_T = -10;
              }
              else if((((ucIRCheck[2]<GusDistance_L_Near)&&(ucIRCheck[3]>GusDistance_R_Near))&&(ucIRCheck[2]>GusDistance_L_Mid)))     //偏右
              {
                GsTpusle_T = 3;
              }
              if((ucIRCheck[2]>GusDistance_L_Near)&&(ucIRCheck[3]<GusDistance_R_Mid))          //偏左
              {
                GsTpusle_T = -12;
              }
              else if((ucIRCheck[3]>GusDistance_R_Near)&&(ucIRCheck[2]<GusDistance_L_Mid))     //偏右             
              {
                GsTpusle_T = 5;
              }          
              else if((ucIRCheck[2]<GusDistance_L_Far)&&((ucIRCheck[3]>GusDistance_R_Far)&&(ucIRCheck[3]<GusDistance_R_Mid)))//偏左
              {
                  GsTpusle_T = -11;
              }
         
              else if((ucIRCheck[3]<GusDistance_R_Far)&&((ucIRCheck[2]>GusDistance_L_Far)&&(ucIRCheck[2]<GusDistance_L_Mid)))//偏左
              {
                   GsTpusle_T = 4;
              }
                
              __PIDContr();
              break;
      
          case __MOTORRUN:                                                    /*  电机运行                    */
            if (__GucMouseState == __GOAHEAD)                                 /*  根据传感器状态微调电机位置  */
            {                              
                  if ((ucIRCheck[2]>GusDistance_L_Near)&&(ucIRCheck[3]<GusDistance_R_Near))//以下为检测一次WAITONESTEP调整后是否达到目标
                  {
                    if (n == 1)
                    {
                          __GmRight.cState = __WAITONESTEP;
                    }               
                    n++;//给出case_WAITONESTEP的调整时间
                    n %= 2;//若1个周期内未调整成功 则重新再调整
                  }
                  else if((ucIRCheck[2]<GusDistance_L_Near)&&(ucIRCheck[3]>GusDistance_R_Near))
                  {
                    if (a == 1)
                    {
                          __GmRight.cState = __WAITONESTEP;
                    }               
                    a++;
                    a %= 2;
                  }
                  
                  else if((ucIRCheck[2]<GusDistance_L_Mid)&&(ucIRCheck[2]>GusDistance_L_Far))
                  {
                    if (b == 1)
                    {
                          __GmRight.cState = __WAITONESTEP;
                    }               
                    b++;
                    b %= 2;
                  }
                  else if((ucIRCheck[3]<GusDistance_R_Mid)&&(ucIRCheck[3]>GusDistance_R_Far))
                  {
                   if (c == 1)
                    {
                          __GmRight.cState = __WAITONESTEP;
                    }               
                    c++;
                    c %= 2;
                  }                  
                 else 
                  {
                      m  = 0;
                      n = 0;
                      GsTpusle_T = 0;//清零
                  }
              
                  if(GuiSpeedCtr==__SPEEDUP)//OBJECTGOTO
                  { 
                    k=(k+1)%5;//20
                    if(k==4)
                    __SpeedUp();
                  }
                  else if(GuiSpeedCtr==__SPEEDDOWN)         //重点
                  {
                      k=(k+1)%10;   //加速度
                      if(k==9)
                      __SpeedDown(); 

                  }
                  else;
              }
            else{
              GsTpusle_T = 0;
             voltageDetect();
            }     
              __PIDContr();
              break;
          
          default:
              break;
    }
      
}
/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mazeSearch(void)                  //搜索连续转弯
{
    int8 cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState)
    {
        cCoor = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))//转弯后进行部分调整
    {
        __GmLeft.uiPulseCtr =30000;   
        __GmRight.uiPulseCtr =30000;
        cL = 1;
        cR = 1;
        if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))||((__GucDistance[ __LEFT] & 0x01) == 0)||((__GucDistance[__RIGHT] & 0x01) == 0)){
          if((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))
            GuiTpusle_LR = 15000;
          else
            GuiTpusle_LR =15000;
        }
        else{
          GuiTpusle_LR =0; 
        }
    }
    else{
      GuiTpusle_LR =0;
    }
    __GucMouseState   = __GOAHEAD;//SysTick_Handler中调用负责控制
    __GiMaxSpeed      =   SEARCHSPEED;    //给定最大速度 反应在PID上   
    __GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
    __GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;//设定前进上限 若16格都无墙 会停
    __GmRight.cState  = __MOTORRUN;//状态位
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
    while (__GmLeft.cState != __MOTORSTOP) 
    {
    
        if (__GmLeft.uiPulseCtr >= ONEBLOCK)
        {                          /*  判断是否走完一格*/
            __GmLeft.uiPulse    -= ONEBLOCK;//左电机数据更新
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))&&(ucIRCheck[2]>GusDistance_L_Far)&&(ucIRCheck[3]>GusDistance_R_Far))//0x01
              {          
               GucFrontNear=1;//前方有墙  左右都有墙
               
                goto End;
              }
             
            } 
            else 
            {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;//右电机数据更新
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        
        if (cL) {                                                       /*  是否允许检测左边            */
            if  ((__GucDistance[__LEFT]  & 0x01)==0)
            {                 /*  左边有支路，跳出程序        */
            
                __GmRight.uiPulse =  __GmRight.uiPulseCtr + 20000 - GuiTpusle_LR;      //左转弯时机调节20500
                __GmLeft.uiPulse  =  __GmLeft.uiPulseCtr  + 20000 - GuiTpusle_LR;
                //持续检查 防止误判
                while ((__GucDistance[__LEFT]  & 0x01)==0)
                {
                 
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {

                        goto End;
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;//若误判 进行数值回复
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {                                                        /*  左边有墙时开始允许检测左边  */
            if (ucIRCheck[2]>GusDistance_L_Far) {
                cL = 1; 
               
            }
        }
        if (cR) {                                                       /*  是否允许检测右边            */
            if ((__GucDistance[__RIGHT]  & 0x01)==0){               /*  右边有支路，跳出程序        */
            //继续前进一段 是转弯时机位于单元中心
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 20500 - GuiTpusle_LR; 
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 20500 - GuiTpusle_LR;//转弯时机延迟
                while ((__GucDistance[__RIGHT]  & 0x01)==0) {
                 //防止误判
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {                         
                        goto End;
                        
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {
            if (ucIRCheck[3]>GusDistance_R_Far)
            {                       
                cR = 1;
             
            }
        }
    }
End:   
}
/*********************************************************************************************************
** Function name:       mouseStop
** Descriptions:        数据清零
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseStop(void)
{   
  __GmSPID.sRef=0;
  __GmWPID.sRef=0;                   //速度值置为0；
  GuiSpeedCtr=5;                    //stop设置该位标志位
}
/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        根据前方近距，旋转180度
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnback(void)
{ 
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  
  if(GucFrontNear)
  {
      __GmSPID.sRef=110;                              //速度设定值
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));   //有墙后不停，红外小于停止判据，不停，直到大于跳出
     __GmRight.uiPulse =10000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 10000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
       GucFrontNear=0;             //转弯后置为1
       backflag=1;
       __GmSPID.sRef=0; 
       mouseStop();
       GW=0;
       __GucMouseState   = __TURNBACK;
       __GmLeft.cState   = __MOTORRUN;
       __GmRight.cState  = __MOTORRUN;
       GucMouseDir = (GucMouseDir + 2) % 4;    //方向更新
       __GmWPID.sRef=90;//转弯速度设定值
   while(1)
   {
       if(GW>280000)        //300000  280000//旋转角度控制
       {
         break;
       }                        
   }
     __GmWPID.sRef=0;
     __GucMouseState   = __GOBACK ;
     __GmSPID.sRef=-50;//后退
     __GmRight.uiPulse =5000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 5000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);   //转弯后后置一段位移  
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);     //用于校准身姿
        mouseStop();
        __GucMouseState   = __GOAHEAD ;
        GuiSpeedCtr=__SPEEDUP;
        __GmSPID.sRef=80;             
        __GmRight.uiPulse =5000;             
        __GmRight.uiPulseCtr=0;
        __GmLeft.uiPulse = 5000;
        __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);   
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 30000;
    __GmLeft.uiPulseCtr = 30000;
    
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
/*********************************************************************************************************
** Function name:       onestep
** Descriptions:        步长测试
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void onestep(void)
{   
 
   __GmRight.cState = __MOTORRUN;        //电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
    __GmRight.sSpeed = 500;              //控制占空比
    __rightMotorContr();                 //右电机控制函数
    __GmLeft.sSpeed = 500;
    __leftMotorContr();                  //左电机控制函数
   
   __GucMouseState   = __GOAHEAD;        //运行中姿态矫正标志位
  
   __GmLeft.uiPulse =28000;              //步长对应的脉冲数
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =28000;         
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);   //循环
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
  
    __GmRight.cState = __MOTORSTOP;       //电脑鼠状态
    __GmLeft.cState  = __MOTORSTOP;      
    __GmRight.sSpeed = 0;                 //控制占空比
    __rightMotorContr();                  //右电机控制函数
    __GmLeft.sSpeed = 0;
    __leftMotorContr();                   //左电机控制函数
    
    __GmRight.uiPulseCtr = 0;             //清零脉冲数
    __GmLeft.uiPulseCtr = 0;
}
