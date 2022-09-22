#ifndef _GRAHPYT_H
#define _GRAHPYT_H
#include "sys.h"
#include "GRAPH.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//STemwin GRAPH控件使用   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/3/19
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
typedef struct 
{
	int *buffer;				        //AD数据存放区
	u8 	adflag;					        //AD采集是否完成标志
	int dataxsize,dataysize;	        //示波器数据区X,Y大小
	u8 	linecolorindex,backcolorindex;	//线条颜色索引值和背景颜色索引值
	GUI_COLOR linecolor,backcolor;		//线条颜色和背景颜色
	GRAPH_DATA_Handle  graphdata; 	    //GRAHP_DATA的数据句柄
	int div;					        //一格代表几秒
    float resolut;                      //每个AD原始值代表多大的电压(单位为mv)
}_oscill_dev;

extern _oscill_dev oscilldev;

void Graphyt_demo(void); 
#endif
