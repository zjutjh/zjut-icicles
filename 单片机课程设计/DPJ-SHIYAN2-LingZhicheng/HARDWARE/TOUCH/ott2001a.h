#ifndef __OTT2001A_H
#define __OTT2001A_H	
#include "sys.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//4.3寸电容触摸屏-OTT2001A 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/28
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 


//IO操作函数	
#define OTT_RST PIout(8) //OTT2001A复位引脚
#define OTT_INT PHin(7)  //OTT2001A断引脚		
  
//通过OTT_SET_REG指令,可以查询到这个信息 
//注意,这里的X,Y和屏幕的坐标系刚好是反的.
#define OTT_MAX_X 		2700	 	//TP X方向的最大值(竖方向)
#define OTT_MAX_Y 		1500    	//TP Y方向的最大值(横方向)

//缩放因子
#define OTT_SCAL_X		0.2963		//屏幕的 纵坐标/OTT_MAX_X		
#define OTT_SCAL_Y		0.32		//屏幕的 横坐标/OTT_MAX_Y		
 
//I2C读写命令	
#define OTT_CMD_WR 		0XB2     	//写命令
#define OTT_CMD_RD 		0XB3		//读命令
 
//寄存器地址
#define OTT_GSTID_REG 	0X0000   	//OTT2001A当前检测到的触摸情况
#define OTT_TP1_REG 	0X0100  	//第一个触摸点数据地址
#define OTT_TP2_REG 	0X0500		//第二个触摸点数据地址
#define OTT_TP3_REG 	0X1000		//第三个触摸点数据地址
#define OTT_TP4_REG 	0X1400		//第四个触摸点数据地址
#define OTT_TP5_REG 	0X1800		//第五个触摸点数据地址  
#define OTT_SET_REG   	0X0900   	//分辨率设置寄存器地址
#define OTT_CTRL_REG  	0X0D00   	//传感器控制(开/关)  

 	

u8 OTT2001A_WR_Reg(u16 reg,u8 *buf,u8 len);		//写寄存器(实际无用)
void OTT2001A_RD_Reg(u16 reg,u8 *buf,u8 len);	//读寄存器
void OTT2001A_SensorControl(u8 cmd);//传感器打开/关闭操作
u8 OTT2001A_Init(void); 			//4.3电容触摸屏始化函数
u8 OTT2001A_Scan(u8 mode);			//电容触摸屏扫描函数

#endif













