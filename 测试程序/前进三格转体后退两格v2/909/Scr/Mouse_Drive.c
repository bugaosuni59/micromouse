#include "Mouse_Drive.h"
/*********************************************************************************************************
  ����ȫ�ֱ���
*********************************************************************************************************/
uint8             GucMouseDir                     = UP;                 /*  ���������ǰ����          */
uint8             GmcMouse;
static uint32     GW;                                                       /*С��ת���Ƕ�*/
static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0, 0};    /*  ���岢��ʼ������״̬      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0, 0};    /*  ���岢��ʼ���ҵ��״̬      */
static __PID    __GmLPID;                                                 /*  ��������PID      */
static __PID    __GmRPID;                                                 /*  �����ҵ��PID     */
static __PID    __GmSPID;                                                 /*  ֱ��PID     */
static __PID    __GmWPID;                                                 /*  ��תPID     */
static uint8    __GucMouseState                   = __STOP;             /*  ���������ǰ����״̬      */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  �����������е�����ٶ�      */
static uint8    __GucDistance[5]                  = {0};                /*  ��¼������״̬              */
uint16   GusFreq_F                         = 36200;   //33.8,33,327        /*  ǰ������Ƶ��              */
uint16   GusFreq_FJ                        = 19200;   //26.3,266,275              /*  ǰ���������Ƶ��              */
uint16   GusFreq_X                         = 30000;   //35,33.8          /*  б45�Ⱥ���Ƶ��              */
uint16   GusFreq_LF                        = 31700;   //34000           /*  ���Һ���Զ��Ƶ��              */
uint16   GusFreq_L                         = 18300;              /*  ���Һ������Ƶ��              */
static  int16   GsTpusle_T                       = 0;                  /*  ����У�����ٵ��ٶ�ֵ              */
static uint8    GuiSpeedCtr                       = 0;//�Ӽ��ٱ�־λ
static uint16   GuiTpusle_LR                      = 0;
static uint16   GuiTpusle_S                       = 0;
static uint8    GucFrontNear                      = 0;
 uint8    GucGoHead                     = 0;
extern u16 voltageDetectRef;//�����Ƿ�����ѹ
extern __IO uint16_t ADC_ConvertedValue[5];//ADC����ֵ
uint8 backflag;
float W;//��ת�Ƕȿ�����
uint16 ucIRCheck[4];//������ն˵�ѹֵ
//������趨ֵ�����ĳ�������ʽ
uint16 GusDistance_L_Near=764; //������
uint16 GusDistance_L_Mid=437;  //�������
uint16 GusDistance_L_Far=192;  //�����Զ
uint16 GusDistance_R_Near=628; //�Һ����
uint16 GusDistance_R_Mid=410;  //�Һ�����
uint16 GusDistance_R_Far=240;  //�Һ����
uint16 GusDistance_FL_Near =490;  //���࣬�����ж�ֹͣ    ԭʼ�޸Ĳ���500 680    180 192   
uint16 GusDistance_FR_Near =440;
uint16 GusDistance_FL_Far = 245;   //Զ�������ж�ǽ��    ��ò���    200    229
uint16 GusDistance_FR_Far = 220;
/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        PID��ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void PIDInit(void) 
{  
    __GmLPID.usEncoder_new = 32768;//�����������ֵ
    __GmLPID.usFeedBack = 0 ;  //�ٶȷ���ֵ
    __GmLPID.sFeedBack = 0 ;
    
    __GmRPID.usEncoder_new = 32768;
    __GmRPID.usFeedBack = 0 ;  //�ٶȷ���ֵ
    __GmRPID.sFeedBack = 0 ;
    
    __GmSPID.sRef = 0 ;        //�ٶ��趨ֵ 
    __GmSPID.sFeedBack = 0 ;        
    __GmSPID.sPreError = 0 ;   //ǰһ�Σ��ٶ����,,vi_Ref - vi_FeedBack 
    __GmSPID.sPreDerror = 0 ;   //ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
        
    __GmSPID.fKp = __KP; 
    __GmSPID.fKi = __KI;
    __GmSPID.fKd = __KD; 
       
    __GmSPID.iPreU = 0 ;      //����������ֵ 
    
    __GmWPID.sRef = 0 ;        //�ٶ��趨ֵ 
    __GmWPID.sFeedBack = 0 ;       
    __GmWPID.sPreError = 0 ;   //ǰһ�Σ��ٶ����,,vi_Ref - vi_FeedBack 
    __GmWPID.sPreDerror = 0 ;   //ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
    
    __GmWPID.fKp = __KP;  
    __GmWPID.fKi = __KI;  
    __GmWPID.fKd = __KD; 
       
    __GmWPID.iPreU = 0 ;      //����������ֵ 
    
}
/*********************************************************************************************************
** Function name:       __SPIDContr
** Descriptions:        ֱ��PID����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __SPIDContr(void) 
{ 
    float  error,d_error,dd_error;
    static uint8   K_I=1;
    error = __GmSPID.sRef - __GmSPID.sFeedBack; // ƫ�����:�趨Ŀ���ٶȼ�ȥ��ǰ�ٶ���Ϊerror  PID��error perror and  pperror
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
    
    __GmSPID.sPreError = error; //�洢��ǰƫ�� 
    __GmSPID.sPreDerror = d_error;
    
    __GmSPID.iPreU += (int16)(  __GmSPID.fKp * d_error + K_I*__GmSPID.fKi * error  + __GmSPID.fKd*dd_error); 
}
/*********************************************************************************************************
** Function name:       __WPIDContr
** Descriptions:        ��ת����PID����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __WPIDContr(void) 
{ 
    float  error,d_error,dd_error; 
    static uint8   K_I=1;
    error = __GmWPID.sRef + GsTpusle_T- __GmWPID.sFeedBack; // ƫ����� 
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
    
    __GmWPID.sPreError = error; //�洢��ǰƫ�� 
    __GmWPID.sPreDerror = d_error;
    __GmWPID.iPreU += (int16)(  __GmWPID.fKp * d_error + K_I*__GmWPID.fKi * error  + __GmWPID.fKd*dd_error);
        
}
/*********************************************************************************************************
** Function name:       voltageDetect
** Descriptions:        ��ת�Ƕȿ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        PID���ƣ�ͨ�����������Ƶ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __PIDContr(void)
{
    __SPIDContr();//ֱ��PID���ƺ���
    __WPIDContr();//��תPPID���ƺ���
    __GmLeft.sSpeed = __GmSPID.iPreU - __GmWPID.iPreU ;//����ռ�ձ�
    if(__GmLeft.sSpeed>=0){
     __GmLeft.cDir=__MOTORGOAHEAD; 
    if( __GmLeft.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmLeft.sSpeed = U_MIN;
    }
    else{
      __GmLeft.cDir=__MOTORGOBACK;
      __GmLeft.sSpeed *=-1; 
    if( __GmLeft.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmLeft.sSpeed = U_MIN;
    }
      
    __GmRight.sSpeed = __GmSPID.iPreU + __GmWPID.iPreU ;//�ҵ��ռ�ձ�
    if(__GmRight.sSpeed>=0){
     __GmRight.cDir=__MOTORGOAHEAD; 
    if( __GmRight.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmRight.sSpeed = U_MIN;
    }
    else{
      __GmRight.cDir=__MOTORGOBACK;
      __GmRight.sSpeed *=-1; 
    if( __GmRight.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmRight.sSpeed = U_MIN;
    }
    __rightMotorContr();
    __leftMotorContr();
    
}
/*********************************************************************************************************
** Function name:       __Encoder
** Descriptions:        �ɼ����������������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __Encoder(void)
{
    static u16 Dir_L;
    static u16 Dir_R;
	
   
    __GmLPID.usEncoder_new = TIM_GetCounter(TIM2);//�����������ֵ
    __GmRPID.usEncoder_new = TIM_GetCounter(TIM3);//�ұ���������ֵ
	
    Dir_R=TIM3->CR1;//TIM3_CR1�Ĵ�����ֵ  16λ  �жϵ������ת
    Dir_R=(Dir_R&0x0010)>>4;//Ϊ���ж�TIM3_CR1�Ĵ���DIRλ��ֵ  1�����¼���  0�����ϼ���	
	
     Dir_L=TIM2->CR1;//TIM2
    Dir_L=(Dir_L&0x0010)>>4;//ͬ��	
		
	if(Dir_L==1)//���¼���  ����
	{
             __GmLPID.usFeedBack = 32768 - __GmLPID.usEncoder_new;//һ��ʱ���ڵõ���ʱ��������
             TIM_SetCounter(TIM2, 32768);//�趨����ֵΪ32768
             __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;//���г���������	
	     __GmLeft.cRealDir = __MOTORGOBACK;//�������з���
             __GmLPID.sFeedBack= -1*__GmLPID.usFeedBack;//__GmLPID.usFeedBack �Ļ���ֵ	
	}
	else//���ϼ���  ǰ��
	{   
            __GmLPID.usFeedBack = __GmLPID.usEncoder_new - 32768;//һ��ʱ���ڵõ���ʱ��������
            TIM_SetCounter(TIM2, 32768);//�趨����ֵΪ32768
            __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;//���г���������
	    __GmLeft.cRealDir = __MOTORGOAHEAD;
            __GmLPID.sFeedBack= __GmLPID.usFeedBack; //__GmLPID.usFeedBack ���෴��
	}
	
	if(Dir_R==1)//���¼���  ǰ��
	{
            __GmRPID.usFeedBack = 32768 - __GmRPID.usEncoder_new;//һ��ʱ���ڵõ���ʱ��������
            TIM_SetCounter(TIM3, 32768);//�趨����ֵΪ32768
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;//���г���������
	    __GmRight.cRealDir = __MOTORGOAHEAD;
	    __GmRPID.sFeedBack = __GmRPID.usFeedBack;//__GmRPID.usFeedBack ���෴��
        }  
	else//���ϼ��� ����
	{ 
            __GmRPID.usFeedBack = __GmRPID.usEncoder_new - 32768;//һ��ʱ���ڵõ���ʱ��������
            TIM_SetCounter(TIM3, 32768);//�趨����ֵΪ32768
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;  //���г��������� 
            __GmRight.cRealDir = __MOTORGOBACK;
            __GmRPID.sFeedBack = -1*__GmRPID.usFeedBack;//__GmRPID.usFeedBack ���෴��
	}
		
		
        __GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;      //�������ɼ���ֵ��������ת�ٶ���ֱ���ٶ�
        __GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;    
}
/*********************************************************************************************************
** Function name:       __irSendFreq  __irCheck
** Descriptions:        ���⴫����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __irSendFreq (int8  __cNumber)
{
    switch (__cNumber) 
    {
      case 1: /*ǰ������*/                                                            
          GPIO_SetBits(GPIOA,GPIO_Pin_5);  //ǰ���� ��
           break;
  
      case 2:  /*���Һ���*/                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_13); //ǰ���� ��
          break;  
      case 3:                                                         
          GPIO_SetBits(GPIOA,GPIO_Pin_3); //�� ��
          break; 
      case 4:                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_2); //�� ��
          break;           
      default:
          break;
    }
}
void __irCheck (void)
{      
    static uint8 ucState = 0;//ֻ����һ�� �м���
    switch(ucState)
    {
        case 0:
          ucIRCheck[3] = ADC_ConvertedValue[3];    //��ȡ��������
          GPIO_ResetBits(GPIOC,GPIO_Pin_2); 
          if(ucIRCheck[3]>GusDistance_R_Far)    //���к���������Ϻ��������ֵ�����Ҳ���ǽ��bit0��Ϊ1.
          {
              __GucDistance[__RIGHT]  |= 0x01;//�Ҳ���ǽ���Ƚ�Զ
          }         
          else
          {
              __GucDistance[__RIGHT]  &= 0xfe;            
          }
          
          if(ucIRCheck[3]>GusDistance_R_Mid)//�Ҳ���ǽ ������
          {
              __GucDistance[__RIGHT]  |= 0x02;  // __GucDistance[__RIGHT] =0x03 
          }
          
          else
          {
              __GucDistance[__RIGHT]  &= 0xfd;
          }  
          
           if(ucIRCheck[3]>GusDistance_R_Near)//�Ҳ���ǽ �ܽ���
          {
              __GucDistance[__RIGHT]  |= 0x04;// __GucDistance[__RIGHT] =0x07
          }
          
          else
          {
              __GucDistance[__RIGHT]  &= 0xfb;//�Ҳ���ǽ__GucDistance[__RIGHT] =0x03
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
          if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))//ǰ����ǽ
            GucGoHead =1;//�ñ�׼λ���ڿ��Ƶ���������
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
              __GucDistance[__LEFT]   |= 0x04;//ͬ��
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
    ucState = (ucState + 1) % 4;      //Ϊ�˱�֤�ж��ڲ���ɨ����
}
/*********************************************************************************************************
** Function name:       sensorDebug 
** Descriptions:        ���⴫��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void sensorDebug (void)
{
    zlg7289Download(2, 0, 0, __GucDistance[__LEFT  ]);//��������
    zlg7289Download(2, 1, 0, __GucDistance[__FRONTL]);//������ǰ��
    zlg7289Download(2, 2, 0, __GucDistance[__FRONTR]);//������ǰ��    
    zlg7289Download(2, 3, 0, __GucDistance[__RIGHT ]);//��������
}
/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        ��ֱ���������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __rightMotorContr(void)           //H�ſ���  
{
    switch (__GmRight.cDir) 
    {
    case __MOTORGOAHEAD:                                                
      TIM_SetCompare1(TIM1,__GmRight.sSpeed);//PWM�����ռ�ձȣ��趨�Ƚ�ֵ
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
** Descriptions:        ��ֱ���������
** input parameters:    __GmLeft.cDir :������з���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __leftMotorContr(void)//H�ſ���
{
    switch (__GmLeft.cDir) 
    {
    case __MOTORGOAHEAD:                                            
      TIM_SetCompare4(TIM1,0);                                          
      TIM_SetCompare3(TIM1,__GmLeft.sSpeed);//�趨�Ƚ�ֵ
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
** Descriptions:        ��������ٳ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __SpeedUp (void)
{
    uint16 Speed;              
    Speed=__GmSPID.sFeedBack;          //�������ٶȲɼ�
    if(__GmSPID.sRef<__GiMaxSpeed){       //�����ǰ�ٶ�С���趨ֱ���ٶȣ����ٵ������������ٵ�MAX
      if(Speed >=__GmSPID.sRef)        //�ٶȴ����趨ֵ
      {
        __GmSPID.sRef=__GmSPID.sRef+8;          //���м���
      }
    }   
}
/*********************************************************************************************************
** Function name:       __SpeedDown
** Descriptions:        ��������ٳ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __SpeedDown (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;         //�Ӽ����趨ֵ��ʵ���ٶ�ƫ������Զ��ٶȵ���
    if(__GmSPID.sRef>=MINSPEED)       //��ǰ�ٶȴ����趨��С�ٶ�
    {
      if(Speed <=__GmSPID.sRef+3)
      {
       __GmSPID.sRef=__GmSPID.sRef-3;//���м���
      }
    }
}
/*********************************************************************************************************
** Function name:       TIM2_IROHandler  TIM3_IRQHandler
** Descriptions:        �жϺ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void TIM2_IROHandler(void)                     //�жϷ���
{
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//����жϱ�־λ
}
void TIM3_IRQHandler(void)//�жϷ���
{
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//����жϱ�־λ
}
/*********************************************************************************************************
** Function name:       SysTick_Handler
** Descriptions:        ��ʱ�ж�ɨ�衣
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void SysTick_Handler(void)                                   //���в���������ʵ�ص��Ժ�������
{  
    static int8 n = 0,m = 0,k=0,l=0,a=0,b=0,c=0,t=0,w=0;
    uint16 Sp;
    __Encoder();                           //��ʱ�ɼ������������Լ���������
    __irCheck ();                          //���ⷢ��˼��
    Sp=__GmSPID.sFeedBack;                 //Sp����ֱ���ٶ�
      switch (__GmRight.cState) {         //�ҵ��״̬ѡ��
          
        
      
        case __MOTORSTOP:                                                   /*  ֹͣ��ͬʱ�����ٶȺ�����ֵ  */
              __GmRight.uiPulse    = 0;
              __GmRight.uiPulseCtr = 0;    //������������
              __GmLeft.uiPulse    = 0;
              __GmLeft.uiPulseCtr = 0;     //������������
              break;
      
        case __WAITONESTEP:                                                       //��ͣһ�� ��������΢��
              __GmRight.cState = __MOTORRUN;
              if((((ucIRCheck[2]>GusDistance_L_Near)&&(ucIRCheck[3]<GusDistance_R_Near))&&(ucIRCheck[3]>GusDistance_R_Mid)))          //ƫ��
              {
                GsTpusle_T = -10;
              }
              else if((((ucIRCheck[2]<GusDistance_L_Near)&&(ucIRCheck[3]>GusDistance_R_Near))&&(ucIRCheck[2]>GusDistance_L_Mid)))     //ƫ��
              {
                GsTpusle_T = 3;
              }
              if((ucIRCheck[2]>GusDistance_L_Near)&&(ucIRCheck[3]<GusDistance_R_Mid))          //ƫ��
              {
                GsTpusle_T = -12;
              }
              else if((ucIRCheck[3]>GusDistance_R_Near)&&(ucIRCheck[2]<GusDistance_L_Mid))     //ƫ��             
              {
                GsTpusle_T = 5;
              }          
              else if((ucIRCheck[2]<GusDistance_L_Far)&&((ucIRCheck[3]>GusDistance_R_Far)&&(ucIRCheck[3]<GusDistance_R_Mid)))//ƫ��
              {
                  GsTpusle_T = -11;
              }
         
              else if((ucIRCheck[3]<GusDistance_R_Far)&&((ucIRCheck[2]>GusDistance_L_Far)&&(ucIRCheck[2]<GusDistance_L_Mid)))//ƫ��
              {
                   GsTpusle_T = 4;
              }
                
              __PIDContr();
              break;
      
          case __MOTORRUN:                                                    /*  �������                    */
            if (__GucMouseState == __GOAHEAD)                                 /*  ���ݴ�����״̬΢�����λ��  */
            {                              
                  if ((ucIRCheck[2]>GusDistance_L_Near)&&(ucIRCheck[3]<GusDistance_R_Near))//����Ϊ���һ��WAITONESTEP�������Ƿ�ﵽĿ��
                  {
                    if (n == 1)
                    {
                          __GmRight.cState = __WAITONESTEP;
                    }               
                    n++;//����case_WAITONESTEP�ĵ���ʱ��
                    n %= 2;//��1��������δ�����ɹ� �������ٵ���
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
                      GsTpusle_T = 0;//����
                  }
              
                  if(GuiSpeedCtr==__SPEEDUP)//OBJECTGOTO
                  { 
                    k=(k+1)%5;//20
                    if(k==4)
                    __SpeedUp();
                  }
                  else if(GuiSpeedCtr==__SPEEDDOWN)         //�ص�
                  {
                      k=(k+1)%10;   //���ٶ�
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
** Descriptions:        ǰ��N��
** input parameters:    iNblock: ǰ���ĸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mazeSearch(void)                  //��������ת��
{
    int8 cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState)
    {
        cCoor = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))//ת�����в��ֵ���
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
    __GucMouseState   = __GOAHEAD;//SysTick_Handler�е��ø������
    __GiMaxSpeed      =   SEARCHSPEED;    //��������ٶ� ��Ӧ��PID��   
    __GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
    __GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;//�趨ǰ������ ��16����ǽ ��ͣ
    __GmRight.cState  = __MOTORRUN;//״̬λ
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
    while (__GmLeft.cState != __MOTORSTOP) 
    {
    
        if (__GmLeft.uiPulseCtr >= ONEBLOCK)
        {                          /*  �ж��Ƿ�����һ��*/
            __GmLeft.uiPulse    -= ONEBLOCK;//�������ݸ���
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))&&(ucIRCheck[2]>GusDistance_L_Far)&&(ucIRCheck[3]>GusDistance_R_Far))//0x01
              {          
               GucFrontNear=1;//ǰ����ǽ  ���Ҷ���ǽ
               
                goto End;
              }
             
            } 
            else 
            {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            __GmRight.uiPulse    -= ONEBLOCK;//�ҵ�����ݸ���
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        
        if (cL) {                                                       /*  �Ƿ����������            */
            if  ((__GucDistance[__LEFT]  & 0x01)==0)
            {                 /*  �����֧·����������        */
            
                __GmRight.uiPulse =  __GmRight.uiPulseCtr + 20000 - GuiTpusle_LR;      //��ת��ʱ������20500
                __GmLeft.uiPulse  =  __GmLeft.uiPulseCtr  + 20000 - GuiTpusle_LR;
                //������� ��ֹ����
                while ((__GucDistance[__LEFT]  & 0x01)==0)
                {
                 
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {

                        goto End;
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;//������ ������ֵ�ظ�
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {                                                        /*  �����ǽʱ��ʼ���������  */
            if (ucIRCheck[2]>GusDistance_L_Far) {
                cL = 1; 
               
            }
        }
        if (cR) {                                                       /*  �Ƿ��������ұ�            */
            if ((__GucDistance[__RIGHT]  & 0x01)==0){               /*  �ұ���֧·����������        */
            //����ǰ��һ�� ��ת��ʱ��λ�ڵ�Ԫ����
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 20500 - GuiTpusle_LR; 
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 20500 - GuiTpusle_LR;//ת��ʱ���ӳ�
                while ((__GucDistance[__RIGHT]  & 0x01)==0) {
                 //��ֹ����
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
** Descriptions:        ��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseStop(void)
{   
  __GmSPID.sRef=0;
  __GmWPID.sRef=0;                   //�ٶ�ֵ��Ϊ0��
  GuiSpeedCtr=5;                    //stop���ø�λ��־λ
}
/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        ����ǰ�����࣬��ת180��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnback(void)
{ 
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  
  if(GucFrontNear)
  {
      __GmSPID.sRef=110;                              //�ٶ��趨ֵ
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));   //��ǽ��ͣ������С��ֹͣ�оݣ���ͣ��ֱ����������
     __GmRight.uiPulse =10000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 10000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
       GucFrontNear=0;             //ת�����Ϊ1
       backflag=1;
       __GmSPID.sRef=0; 
       mouseStop();
       GW=0;
       __GucMouseState   = __TURNBACK;
       __GmLeft.cState   = __MOTORRUN;
       __GmRight.cState  = __MOTORRUN;
       GucMouseDir = (GucMouseDir + 2) % 4;    //�������
       __GmWPID.sRef=90;//ת���ٶ��趨ֵ
   while(1)
   {
       if(GW>280000)        //300000  280000//��ת�Ƕȿ���
       {
         break;
       }                        
   }
     __GmWPID.sRef=0;
     __GucMouseState   = __GOBACK ;
     __GmSPID.sRef=-50;//����
     __GmRight.uiPulse =5000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 5000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);   //ת������һ��λ��  
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);     //����У׼����
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
** Descriptions:        ��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void onestep(void)
{   
 
   __GmRight.cState = __MOTORRUN;        //������״̬
   __GmLeft.cState  = __MOTORRUN;
    __GmRight.sSpeed = 500;              //����ռ�ձ�
    __rightMotorContr();                 //�ҵ�����ƺ���
    __GmLeft.sSpeed = 500;
    __leftMotorContr();                  //�������ƺ���
   
   __GucMouseState   = __GOAHEAD;        //��������̬������־λ
  
   __GmLeft.uiPulse =28000;              //������Ӧ��������
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =28000;         
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);   //ѭ��
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
  
    __GmRight.cState = __MOTORSTOP;       //������״̬
    __GmLeft.cState  = __MOTORSTOP;      
    __GmRight.sSpeed = 0;                 //����ռ�ձ�
    __rightMotorContr();                  //�ҵ�����ƺ���
    __GmLeft.sSpeed = 0;
    __leftMotorContr();                   //�������ƺ���
    
    __GmRight.uiPulseCtr = 0;             //����������
    __GmLeft.uiPulseCtr = 0;
}
