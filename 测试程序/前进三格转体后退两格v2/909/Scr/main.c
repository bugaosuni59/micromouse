/**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:V1.5
** Description:2017-3-7日 例程 回转函数编写
*********************************************************************************************************/


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Maze.h"
#include "stm32f10x.h"
#include "zlg7289.h"
#include "BitBand.h"
#include "Mouse_Drive.h"

/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
extern uint8    GucGoHead;
static uint8    GucMouseTask                        = WAIT;             /*  状态机，初始状态为等待      */
extern __IO uint16_t ADC_ConvertedValue[5];
float w=0;
u16 voltageDetectRef;
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}
/*********************************************************************************************************
** Function name:       RCC_Init
** Descriptions:        时钟初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void RCC_Init(void)
{    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
}
/*********************************************************************************************************
** Function name:       JTAG_Set
** Descriptions:        下载器初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        陀螺仪函数采集 采用了均值滤波
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
u16 VoltageDetectRef(void)//检测当前陀螺仪静止电压 返回值不给实际电压
{
    u8 i;
    u32 Sum=0;
    for(i=0;i<50;i++)
    {
        Sum+=ADC_ConvertedValue[4];
        delay(1000);
    }
    return (Sum/50); //均值滤波
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        主函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
main (void)
{
    uint8 n          = 0;                                               /*  GmcCrossway[]下标           */
    uint8 ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
    uint8 ucTemp     = 0;                                               /*  用于START状态中坐标转换 */
    uint8 i,j          = 0;                                               //用于初始化迷宫地图数据
    uint8 start=0;
    SystemInit();      //系统初始化//
    RCC_Init();        //时钟初始化
    JTAG_Set(1);       //下载初始化设置
    MouseInit();       //传感器 编码器 电机 按键 定时器 初始化
    PIDInit();         //PID初始化
    ZLG7289Init();     //7289初始化
    delay(100000);     //延时
    voltageDetectRef=VoltageDetectRef();
    while (1)    //死循环
    {
        switch (GucMouseTask) //状态机 初始为WAIT
        {                                         
            case WAIT:
               sensorDebug();//传感器调试
               delay(10000);

               if(GucGoHead)//GucGoHead初始值为0 启动命令 按下start同时遮住2个红外传感器
              
               {
                
                  zlg7289Reset(); 
                  GucMouseTask = START;
                 while(GucGoHead);//当不遮住2个红外传感器同时延时后执行START
                  delay(1000000);
               }
                 break;
               

        case START:
//          mazeSearch(); //搜索函数
          onestep();
          onestep();
          onestep();
          mouseTurnback();//右转弯
          onestep();
          onestep();
           GucMouseTask = STOP;
          
        case STOP:
              
        }
    }
}
          
          
          
          