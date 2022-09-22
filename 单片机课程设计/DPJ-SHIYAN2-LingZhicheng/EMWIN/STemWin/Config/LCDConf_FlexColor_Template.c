#include "GUI.h"
#include "tftlcd.h"
#include "touch.h"
#include "GUIDRV_Template.h"
#include "GUIDRV_FlexColor.h"

//与触摸屏有关定义，根据实际情况填写
#define TOUCH_AD_TOP		160  	//按下触摸屏的顶部，写下 Y 轴模拟输入值。
#define TOUCH_AD_BOTTOM		3990 	//按下触摸屏的底部，写下 Y 轴模拟输入值。
#define TOUCH_AD_LEFT 		160		//按下触摸屏的左侧，写下 X 轴模拟输入值。
#define TOUCH_AD_RIGHT		3990	//按下触摸屏的右侧，写下 X 轴模拟输入值。


//屏幕大小
#define XSIZE_PHYS  320 //X轴
#define YSIZE_PHYS  240 //Y轴
#define VXSIZE_PHYS	320 
#define VYSIZE_PHYS 240


//配置检查
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   GUICC_565
  #error Color conversion not defined!
#endif
#ifndef   GUIDRV_FLEXCOLOR
  #error No display driver defined!
#endif

  

//配置程序,用于创建显示驱动器件,设置颜色转换程序和显示尺寸
void LCD_X_Config(void) {
	GUI_DEVICE_CreateAndLink(&GUIDRV_Template_API,GUICC_M565,0,0); //创建显示驱动器件
	LCD_SetSizeEx(0,lcddev.width,lcddev.height);
	LCD_SetVSizeEx(0,lcddev.width,lcddev.height);
	if((lcddev.id == 0X5510)||(lcddev.id == 0X1963)||(lcddev.id==0X7084)||(lcddev.id==0X4342)||(lcddev.id==0X7016)) //电容屏
	{
        GUI_TOUCH_Calibrate(GUI_COORD_X,0,lcddev.width,0,lcddev.width-1);   
        GUI_TOUCH_Calibrate(GUI_COORD_Y,0,lcddev.height,0,lcddev.height-1);
	}else if(lcddev.id == 0X5310||lcddev.id == 0X6804) //0X5510 0X6804为3.5寸 320x480
	{
		if(lcddev.dir == 0) //竖屏 	
		{							
			GUI_TOUCH_Calibrate(GUI_COORD_X,0,320,3931,226);
			GUI_TOUCH_Calibrate(GUI_COORD_Y,0,480,3812,196);
		}else //横屏
		{
			GUI_TOUCH_SetOrientation(GUI_SWAP_XY|GUI_MIRROR_Y); 
			GUI_TOUCH_Calibrate(GUI_COORD_X,0,320,3931,226);
			GUI_TOUCH_Calibrate(GUI_COORD_Y,0,480,3812,196); 	
		}
	}
	else             //其他屏幕全部默认为2.8寸 320X240
	{
		if(lcddev.dir == 0) //竖屏
		{					
			GUI_TOUCH_Calibrate(GUI_COORD_X,0,lcddev.width,155,3903);
			GUI_TOUCH_Calibrate(GUI_COORD_Y,0,lcddev.height,188,3935);
		}else //横屏
		{
			GUI_TOUCH_SetOrientation(GUI_SWAP_XY|GUI_MIRROR_Y); 
			GUI_TOUCH_Calibrate(GUI_COORD_X,0,240,155,3903); 	
			GUI_TOUCH_Calibrate(GUI_COORD_Y,0,320,188,3935);
		}
	}
}

//显示器驱动的回调函数
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  (void) LayerIndex;
  (void) pData;
  
  switch (Cmd) {
  case LCD_X_INITCONTROLLER: {
	//当初始化的时候被调用,主要是设置显示控制器,如果显示控制器在外部初始化则需要用户初始化
		
	//	TFTLCD_Init(); //初始化LCD 已经在开始初始化了,所以此处不需要初始化。
    return 0;
  }
		default:
    r = -1;
	}
  return r;
}
