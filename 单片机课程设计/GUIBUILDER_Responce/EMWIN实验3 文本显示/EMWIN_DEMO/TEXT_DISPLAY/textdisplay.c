#include "textdisplay.h"
#include "GUI.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//STemWin 文本显示 
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/3/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//WEWIN文本显示例程
void emwin_texttest(void)
{
	int i;
	char acText[]	= "This example demostrates text wrapping";
	GUI_RECT Rect	={200,240,259,299};	//定义矩形显示区域
	GUI_WRAPMODE aWm[] = {GUI_WRAPMODE_NONE,
						  GUI_WRAPMODE_CHAR,
						  GUI_WRAPMODE_WORD};
				  
	GUI_SetBkColor(GUI_BLUE);		//设置背景颜色
	GUI_Clear();					//清屏
	GUI_SetFont(&GUI_Font24_ASCII); //设置字体
	GUI_SetColor(GUI_YELLOW);       //设置前景色(如文本，画线等颜色)
	GUI_DispString("HELLO WORD!");
	
	GUI_SetFont(&GUI_Font8x16);		//设置字体
	GUI_SetPenSize(10);				//设置笔大小
	GUI_SetColor(GUI_RED);			//红色字体
	GUI_DrawLine(300,50,500,130);	//绘线
	GUI_DrawLine(300,130,500,50);	//绘线
	GUI_SetBkColor(GUI_BLACK);		//设置黑色背景
	GUI_SetColor(GUI_WHITE);		//设置字体颜色为白色
	GUI_SetTextMode(GUI_TM_NORMAL);	//正常模式
	GUI_DispStringHCenterAt("GUI_TM_NORMAL",400,50);
	GUI_SetTextMode(GUI_TM_REV);	//反转文本
	GUI_DispStringHCenterAt("GUI_TM_REV"   ,400,66);
	GUI_SetTextMode(GUI_TM_TRANS);	//透明文本
	GUI_DispStringHCenterAt("GUI_TM_TRANS" ,400,82);
	GUI_SetTextMode(GUI_TM_XOR);   	//异或文本
	GUI_DispStringHCenterAt("GUI_TM_XOR"   ,400,98);
	GUI_SetTextMode(GUI_TM_TRANS|GUI_TM_REV);//透明反转文本
	GUI_DispStringHCenterAt("GUI_EM_TRANS|GUI_TM_REV",400,114);
	
	GUI_SetTextMode(GUI_TM_TRANS);  //透明文本
	for(i=0;i<3;i++)
	{
		GUI_SetColor(GUI_WHITE);
		GUI_FillRectEx(&Rect);
		GUI_SetColor(GUI_BLACK);
		GUI_DispStringInRectWrap(acText,&Rect,GUI_TA_LEFT,aWm[i]);//在当前窗口指定的矩形区域内显示字符串(并可自动换行)
		Rect.x0 += 70;
		Rect.x1 += 70;
	}	
}



