/**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:V1.5
** Description:2017-3-7�� ���� ��ת������д
*********************************************************************************************************/


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Maze.h"
#include "stm32f10x.h"
#include "zlg7289.h"
#include "BitBand.h"
#include "Mouse_Drive.h"

/*********************************************************************************************************
  ȫ�ֱ�������
*********************************************************************************************************/
extern uint8    GucGoHead;
static uint8    GucMouseTask                        = WAIT;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */
extern __IO uint16_t ADC_ConvertedValue[5];
float w=0;
u16 voltageDetectRef;
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}
/*********************************************************************************************************
** Function name:       RCC_Init
** Descriptions:        ʱ�ӳ�ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void RCC_Init(void)
{    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
}
/*********************************************************************************************************
** Function name:       JTAG_Set
** Descriptions:        ��������ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void JTAG_Set(u8 mode)
{
    u32 temp;
    temp<<=25;
    RCC->APB2ENR|=1<<0;
    AFIO->MAPR&=0XF8FFFFFF;
    AFIO->MAPR|=temp;
}
/*********************************************************************************************************
** Function name:       VoltageDetectRef
** Descriptions:        �����Ǻ����ɼ� �����˾�ֵ�˲�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
u16 VoltageDetectRef(void)//��⵱ǰ�����Ǿ�ֹ��ѹ ����ֵ����ʵ�ʵ�ѹ
{
    u8 i;
    u32 Sum=0;
    for(i=0;i<50;i++)
    {
        Sum+=ADC_ConvertedValue[4];
        delay(1000);
    }
    return (Sum/50); //��ֵ�˲�
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
main (void)
{
    uint8 n          = 0;                                               /*  GmcCrossway[]�±�           */
    uint8 ucRoadStat = 0;                                               /*  ͳ��ĳһ�����ǰ����֧·��  */
    uint8 ucTemp     = 0;                                               /*  ����START״̬������ת�� */
    uint8 i,j          = 0;                                               //���ڳ�ʼ���Թ���ͼ����
    uint8 start=0;
    SystemInit();      //ϵͳ��ʼ��//
    RCC_Init();        //ʱ�ӳ�ʼ��
    JTAG_Set(1);       //���س�ʼ������
    MouseInit();       //������ ������ ��� ���� ��ʱ�� ��ʼ��
    PIDInit();         //PID��ʼ��
    ZLG7289Init();     //7289��ʼ��
    delay(100000);     //��ʱ
    voltageDetectRef=VoltageDetectRef();
    while (1)    //��ѭ��
    {
        switch (GucMouseTask) //״̬�� ��ʼΪWAIT
        {                                         
            case WAIT:
               sensorDebug();//����������
               delay(10000);

               if(GucGoHead)//GucGoHead��ʼֵΪ0 �������� ����startͬʱ��ס2�����⴫����
              
               {
                
                  zlg7289Reset(); 
                  GucMouseTask = START;
                 while(GucGoHead);//������ס2�����⴫����ͬʱ��ʱ��ִ��START
                  delay(1000000);
               }
                 break;
               

        case START:
//          mazeSearch(); //��������
          onestep();
          onestep();
          onestep();
          mouseTurnback();//��ת��
          onestep();
          onestep();
           GucMouseTask = STOP;
          
        case STOP:
              
        }
    }
}
          
          
          
          