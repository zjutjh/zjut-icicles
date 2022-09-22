#ifndef __DS18B20_H
#define __DS18B20_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//DHT11驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//IO方向设置
#define DHT11_IO_IN()  {GPIOA->MODER&=~(3<<(6*2));GPIOA->MODER|=0<<(6*2);}	//PB12输入模式
#define DHT11_IO_OUT() {GPIOA->MODER&=~(3<<(6*2));GPIOA->MODER|=1<<(6*2);} 	//PB12输出模式
 
////IO操作函数											   
#define	DHT11_DQ_OUT    PAout(6)//数据端口	PB12
#define	DHT11_DQ_IN     PAin(6) //数据端口	PB12 

typedef struct
{
	u8 temp;    //温度整数部分
	u8 humi;     //湿度整数部分
	u8 temp1;    //温度小数部分
	u8 humi1;    //湿度小数部分
	u8 isUpdate;	
}dht22_dev;
   	
u8 DHT11_Init(void);//初始化DHT11
u8 DHT11_Read_Data(u8 *temp,u8 *humi);//读取温湿度
u8 DHT11_Read_Byte(void);//读出一个字节
u8 DHT11_Read_Bit(void);//读出一个位
u8 DHT11_Check(void);//检测是否存在DHT11
void DHT11_Rst(void);//复位DHT11  
u8 DHT11_Read(dht22_dev *dht22);
#endif
