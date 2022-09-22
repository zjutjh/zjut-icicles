/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.16 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to  ARM LIMITED whose registered office
is situated at  110 Fulbourn Road,  Cambridge CB1 9NJ,  England solely
for  the  purposes  of  creating  libraries  for  ARM7, ARM9, Cortex-M
series,  and   Cortex-R4   processor-based  devices,  sublicensed  and
distributed as part of the  MDK-ARM  Professional  under the terms and
conditions  of  the   End  User  License  supplied  with  the  MDK-ARM
Professional. 
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : GUI_TOUCH_X.C
Purpose     : Config / System dependent externals for GUI
---------------------------END-OF-HEADER------------------------------
*/
#include "GUI.h"
#include "touch.h"
#include "tftlcd.h"
#include "usart.h"


void GUI_TOUCH_X_ActivateX(void) 
{
 // XPT2046_WriteCMD(0x90);
}


void GUI_TOUCH_X_ActivateY(void)
{
  //XPT2046_WriteCMD(0xd0);
}

int  GUI_TOUCH_X_MeasureX(void) 
{
	int32_t xvalue;
	if((lcddev.id == 0X5510)||(lcddev.id == 0X1963)||(lcddev.id==0X7084)||(lcddev.id==0X4342)||(lcddev.id==0X7016)) //电容屏的触摸值获取
	{
		tp_dev.scan(0);
		xvalue=tp_dev.x[0];
		return xvalue;
	}else				//电阻屏
	{
		return TP_Read_XOY(0XD0);  //CMD_RDX=0XD0
	}
}

int  GUI_TOUCH_X_MeasureY(void) 
{	
	int32_t yvalue;
    
	if((lcddev.id == 0X5510)||(lcddev.id == 0X1963)||(lcddev.id==0X7084)||(lcddev.id==0X4342)||(lcddev.id==0X7016))//电容屏的触摸值获取
	{
		tp_dev.scan(0);
		yvalue = tp_dev.y[0];
		return yvalue;
	}else				//电阻屏
	{
		return TP_Read_XOY(0X90);  //CMD_RDX=0XD0
	}
}

