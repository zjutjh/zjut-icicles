#ifndef GUICONF_H
#define GUICONF_H
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//STemWin功能模块管理	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/7/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define GUI_NUM_LAYERS            1     //最大支持1层


#define GUI_OS                    (1)   // 使用操作系统
#define GUI_MAXTASK               (5)   // 最大可调用EMWIN的任务数
#define GUI_SUPPORT_TOUCH         (1)   // 支持触摸

#define GUI_DEFAULT_FONT          &GUI_Font6x8	//默认字体
#define GUI_SUPPORT_MOUSE         (1)    	//支持鼠标
#define GUI_WINSUPPORT            (1)    	//窗口管理
#define GUI_SUPPORT_MEMDEV        (1)    	//存储设备
#define GUI_SUPPORT_DEVICES       (1)    	//使用设备指针
#endif  

