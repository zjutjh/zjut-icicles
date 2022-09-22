#include "timer.h"
#include "led.h"
#include "includes.h"
#include "adc.h"
#include "graphyt.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//定时器中断驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3中断   
}


//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    __HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
    __HAL_RCC_TIM4_CLK_ENABLE();            //使能TIM4时钟
    HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断  
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();    
#endif	
    
    HAL_TIM_IRQHandler(&TIM3_Handler);
    
#if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//退出中断
#endif
}

//定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int count=0;
    if(htim==(&TIM3_Handler))
    {
        if((oscilldev.buffer!=NULL)&&(oscilldev.adflag==0))	//buffer为空，并且允许进行AD采集
		{
			oscilldev.buffer[count]=(int)((float)Get_Adc(ADC_CHANNEL_5)*oscilldev.resolut);//电压值,因为单位mv
			count++;
			if(count>oscilldev.dataxsize)//一次数据采集完成
			{
				count=0;
				oscilldev.adflag=1;//标记AD采集完成
			}
		}
    }
}



/***************************************************************************
****************************************************************************
  下面是TIM7相关源码实验相关函数源码， 串口3 接收数据会用到
****************************************************************************
****************************************************************************/

TIM_HandleTypeDef TIM7_Handler;         //定时器7句柄

//通用定时器7中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器7!(定时器7挂在APB1上，时钟为HCLK/2)
void TIM7_Init(u16 arr,u16 psc)
{  
    __HAL_RCC_TIM7_CLK_ENABLE();                        //使能TIM7时钟
    
    TIM7_Handler.Instance=TIM7;                          //通用定时器7
    TIM7_Handler.Init.Prescaler=psc;                     //分频系数
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM7_Handler.Init.Period=arr;                        //自动装载值
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM7_Handler);
    HAL_TIM_Base_Start_IT(&TIM7_Handler);   //使能定时器7和定时器7更新中断：TIM_IT_UPDATE   
    
    HAL_NVIC_SetPriority(TIM7_IRQn,0,1);    //设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM7_IRQn);          //开启ITM7中断  
}

extern vu16 USART3_RX_STA;
//定时器7中断服务函数
void TIM7_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&TIM7_Handler,TIM_FLAG_UPDATE)!=RESET)    //更新中断
    {
        __HAL_TIM_CLEAR_IT(&TIM7_Handler,TIM_IT_UPDATE);            //清除中断
        USART3_RX_STA|=1<<15;	                                    //标记接收完成
        __HAL_TIM_DISABLE(&TIM7_Handler);                           //关闭定时器7 
    }
}


























