#include "GUI.h"
#include "sdram.h"
#include "malloc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//STemWin内存分配代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/7/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define USE_EXRAM  1	//使用外部RAM
//设置EMWIN内存大小
#define GUI_NUMBYTES  (8*1024*1024)
#define GUI_BLOCKSIZE 0X80  //块大小

//GUI_X_Config
//初始化的时候调用,用来设置emwin所使用的内存
void GUI_X_Config(void) {
	if(USE_EXRAM) //使用外部RAM
	{	
		U32 *aMemory = mymalloc(SRAMEX,GUI_NUMBYTES); //从外部SRAM中分配GUI_NUMBYTES字节的内存
		GUI_ALLOC_AssignMemory((void*)aMemory, GUI_NUMBYTES); //为存储管理系统分配一个存储块
		GUI_ALLOC_SetAvBlockSize(GUI_BLOCKSIZE); //设置存储快的平均尺寸,该区越大,可用的存储快数量越少
		GUI_SetDefaultFont(GUI_FONT_6X8); //设置默认字体
	}else  //使用内部RAM
	{
		U32 *aMemory = mymalloc(SRAMIN,GUI_NUMBYTES); //从内部RAM中分配GUI_NUMBYTES字节的内存
		GUI_ALLOC_AssignMemory((U32 *)aMemory, GUI_NUMBYTES); //为存储管理系统分配一个存储块
		GUI_ALLOC_SetAvBlockSize(GUI_BLOCKSIZE); //设置存储快的平均尺寸,该区越大,可用的存储快数量越少
		GUI_SetDefaultFont(GUI_FONT_6X8); //设置默认字体
	}
}

