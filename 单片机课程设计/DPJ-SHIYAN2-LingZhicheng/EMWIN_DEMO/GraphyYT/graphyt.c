#include "graphyt.h"
#include "DIALOG.h"
#include "stdlib.h"
#include "math.h"
#include "GUI.h"
#include "usart.h"
#include "adc.h"
#include "malloc.h"
#include "includes.h"
#include "timer.h"
#include "led.h"
#include "value.h"
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
const u8*  mmenu_tbl[9]=//系统一级目录的个数
{
	// 温度、湿度、MQ2(甲烷)、pm2.5、pm1.0、PM10、MQ7(co)、co2、TVOC
	"Temp",
	"Humi",
	"MQ-2",
	"PM2.5",
	"PM1.0",
	"PM10",
	"O3",
	"Co2",
	"TVOC", 
	 
	 
};

int				GClose=0;
//控件ID
#define ID_FRAMEWIN_0 	(GUI_ID_USER + 0x00)
#define ID_GRAPH_0 		(GUI_ID_USER + 0x01)
#define ID_BUTTON_0 	(GUI_ID_USER + 0x02)
#define ID_BUTTON_1 	(GUI_ID_USER + 0x03)
#define ID_BUTTON_2 	(GUI_ID_USER + 0x04)
#define ID_BUTTON_3 	(GUI_ID_USER + 0x05)
#define ID_BUTTON_4 	(GUI_ID_USER + 0x06)
#define ID_BUTTON_5 	(GUI_ID_USER + 0x07)

#define BORDER_TOP        	10
#define BORDER_BOTTOM     	15
#define BORDER_LEFT       	40
#define BORDER_RIGHT      	10

#define VERMULTIPLE         (10)        //垂直刻度缩放因子

static GRAPH_SCALE_Handle hScaleV;   	//垂直刻度句柄 
static GRAPH_SCALE_Handle hScaleH;   	//水平刻度句柄


_oscill_dev oscilldev;

GUI_COLOR LineColor[]={GUI_RED,GUI_BLACK,GUI_WHITE,GUI_YELLOW,GUI_BLUE,GUI_DARKMAGENTA};
GUI_COLOR BackColor[]={GUI_BLACK,GUI_LIGHTRED,GUI_DARKMAGENTA,GUI_WHITE,GUI_GREEN};
//对话框资源表
 
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = 
{
	{ FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0, 480, 272, 0, 0x64, 0},
	{ GRAPH_CreateIndirect, "Graph", ID_GRAPH_0, 8, 8, 454, 230, 0, 0x0, 0},
 
};
//FrameWin控件回调函数
static void _cbFrame_exe(WM_MESSAGE * pMsg) 
{ 
  switch (pMsg->MsgId) 
 { 
  case WM_NOTIFY_PARENT: 
    if (pMsg->Data.v == WM_NOTIFICATION_RELEASED)
	  { 
      int Id = WM_GetId(pMsg->hWinSrc);      // Id of widget 
      if (Id == GUI_ID_CLOSE) 
		  { 
          GClose=1;  //我需要在关闭对话框前保存数据 
      } 
    } 
    FRAMEWIN_Callback(pMsg); 
    break; 
  default: 
    FRAMEWIN_Callback(pMsg); 
    break;   
 }  
}
 
//对话框回调函数
static void _cbDialog(WM_MESSAGE * pMsg) 
{
	WM_HWIN hItem;
	//int NCode;
//	int Id;
	
	switch (pMsg->MsgId) 
	{
		case WM_INIT_DIALOG:
			//初始化FRAMEWIN
			hItem = pMsg->hWin;
			FRAMEWIN_SetText(hItem, (void*)mmenu_tbl[DataInfo.DtypeSel]);//标题名称
			FRAMEWIN_SetFont(hItem, GUI_FONT_COMIC24B_ASCII);//GUI_FONT_24B_ASCII
			FRAMEWIN_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
	 FRAMEWIN_AddCloseButton(hItem,FRAMEWIN_BUTTON_RIGHT,0);
			//初始化GRAPH
			hItem=WM_GetDialogItem(pMsg->hWin, ID_GRAPH_0);
			oscilldev.linecolorindex=0;	        //线条默认红色
			oscilldev.backcolorindex=2;	        //背景默认黑色
			oscilldev.linecolor=LineColor[oscilldev.linecolorindex];
			oscilldev.backcolor=BackColor[oscilldev.backcolorindex];
			oscilldev.graphdata=GRAPH_DATA_YT_Create(oscilldev.linecolor,401,0,0);  //创建YT数据对象
			GRAPH_AttachData(hItem,oscilldev.graphdata);		                    //将数据对象添加到GRAPH控件中
			GRAPH_SetBorder(hItem,BORDER_LEFT,BORDER_TOP,BORDER_RIGHT,BORDER_BOTTOM);//设置边界
            GRAPH_SetGridDistX(hItem,50);		//设置水平网格间距
			GRAPH_SetGridDistY(hItem,20);		//设置垂直网格间距
			GRAPH_SetGridVis(hItem,1);			//设置网格可见
		
			hScaleV=GRAPH_SCALE_Create(35,GUI_TA_RIGHT,GRAPH_SCALE_CF_VERTICAL,20); //绘制垂直刻度
			GRAPH_SCALE_SetTextColor(hScaleV,GUI_YELLOW);
			GRAPH_AttachScale(hItem,hScaleV);   //将刻度对象附加到图形小工具
            GRAPH_SCALE_SetFactor(hScaleV,DataInfo.Yxs);  //设置刻度系数
            GRAPH_SCALE_SetOff(hScaleV,000);    //0  设置 0 不需要负值 垂直刻度向上移动200个像素点，这样垂直刻度会有负值
			
			hScaleH=GRAPH_SCALE_Create(218,GUI_TA_HCENTER,GRAPH_SCALE_CF_HORIZONTAL,50);//绘制水平刻度
			GRAPH_SCALE_SetFactor(hScaleH,0.1);  //设置刻度系数
			GRAPH_SCALE_SetTextColor(hScaleH,GUI_DARKGREEN);
			GRAPH_AttachScale(hItem,hScaleH);   //将刻度对象附加到图形小工具
 
			break;
		 case WM_NOTIFY_PARENT:  
			 
		 
		default:
			WM_DefaultProc(pMsg);
			break;
  }
}

//背景窗口WM_HBKWIN回调函数
static void _cbBkWindow(WM_MESSAGE* pMsg)
{
	switch(pMsg->MsgId)
	{
		case WM_PAINT:
			GUI_SetBkColor(GUI_BLACK);
			GUI_Clear(); 
			break;
		default:
			WM_DefaultProc(pMsg);
	}
}

//图形显示demo
void Graphyt_demo(void) 
{
	WM_HWIN hWin,ClientWin;
	//int Graph_xSize,Graph_ySize;                
	int i;
   
	WM_SetCallback(WM_HBKWIN,_cbBkWindow);    //设置背景窗口回调函数
	hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN,0,0);
	
	//为框架窗口设置回调函数
  WM_SetCallback(hWin,_cbFrame_exe);
	ClientWin = WM_GetDialogItem(hWin, ID_GRAPH_0);
	WM_GetWindowSizeX(ClientWin);//Graph_xSize=
	WM_GetWindowSizeY(ClientWin);//Graph_ySize=
	 
	 
	GUI_Exec();
	GClose=0;
    	for(i=0;i<DataInfo.Revnum[DataInfo.DtypeSel];i++) //向GRAPH图形小工具添加数据,注意要缩20倍，因为垂直坐标扩大(缩放)DataInfo.Yxs倍， 
			{  
			 	GRAPH_DATA_YT_AddValue(oscilldev.graphdata,DataInfo.DataBuff[DataInfo.DtypeSel][i]/DataInfo.Yxs+ 0);
			}
			
	while(1)
	{  LED1 = !LED1;
		if(GClose==1)
    { 
		  WM_DeleteWindow(hWin);//删除窗体 
	    return;
	  }
		if(DataInfo.qxdate==1)//来一个数据更新一个数据
		{	
			GRAPH_DATA_YT_AddValue(oscilldev.graphdata,DataInfo.DataBuff[DataInfo.DtypeSel][DataInfo.Revnum[DataInfo.DtypeSel]-1]/DataInfo.Yxs+ 0);
			 
 			DataInfo.qxdate=0;//标志清零	
		}
		GUI_Delay(100);
	}
	 
}

