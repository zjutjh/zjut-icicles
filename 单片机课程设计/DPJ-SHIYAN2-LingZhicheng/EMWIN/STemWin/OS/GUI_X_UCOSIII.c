#include "GUI.h"
#include "includes.h" //因为使用UCOS所以此处应该包含UCOS的代码


/*
*********************************************************************************************************
*                                         GLOBAL VARIABLES
*********************************************************************************************************
*/

static OS_SEM 	*DispSem;  	//显示的信号量
static OS_SEM 	*EventSem;  

static OS_SEM	*KeySem;  	//按键信号量
static int		KeyPressed;
static char		KeyIsInited;

/*
*********************************************************************************************************
*                                        TIMING FUNCTIONS
*
* Notes: Some timing dependent routines of uC/GUI require a GetTime and delay funtion. 
*        Default time unit (tick), normally is 1 ms.
*********************************************************************************************************
*/
int GUI_X_GetTime(void) { 
	OS_ERR err;
	return ((int)OSTimeGet(&err));   //获取系统时间,此处时间单元为1ms
}

//GUI延时函数
void GUI_X_Delay(int period) { 
	
	OS_ERR err;
	CPU_INT32U ticks;
	ticks = (period * 1000) / OSCfg_TickRate_Hz;
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);//UCOSIII延时采用周期模式
}

/*
*********************************************************************************************************
*                                          GUI_X_ExecIdle()
*********************************************************************************************************
*/
void GUI_X_ExecIdle(void)
{
	
	GUI_X_Delay(1);
	
}

/*
*********************************************************************************************************
*                                    MULTITASKING INTERFACE FUNCTIONS
*
* Note(1): 1) The following routines are required only if uC/GUI is used in a true multi task environment, 
*             which means you have more than one thread using the uC/GUI API.  In this case the #define 
*             GUI_OS 1   needs to be in GUIConf.h
*********************************************************************************************************
*/
void GUI_X_InitOS(void)
{
	OS_ERR err;
	//创建初始值为1的信号量，用于共享资源
	OSSemCreate ((OS_SEM*	)DispSem,	
                 (CPU_CHAR*	)"Disp_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
	//创建初始值为1的信号量，用于事件触发
	OSSemCreate ((OS_SEM*	)EventSem,	
                 (CPU_CHAR*	)"Event_SEM",
                 (OS_SEM_CTR)0,		
                 (OS_ERR*	)&err);
}

//等待信号量
void GUI_X_Lock(void)
{
	OS_ERR err;
	OSSemPend(DispSem,0,OS_OPT_PEND_BLOCKING,0,&err); 	//请求信号量
}
//发送信号量
void GUI_X_Unlock(void)
{
	OS_ERR err;
	OSSemPost(DispSem,OS_OPT_POST_1,&err);				//发送信号量
}
//放回任务ID，此处返回的是任务优先级，由于UCOSIII支持时间片
//轮转调度，因此如果使用了时间片轮转调度功能的话有可能会出错！
U32 GUI_X_GetTaskId(void)
{
	return ((U32)(OSTCBCurPtr->Prio)); //获取任务优先级,也就是任务ID
}

/*
*********************************************************************************************************
*                                        GUI_X_WaitEvent()
*                                        GUI_X_SignalEvent()
*********************************************************************************************************
*/

void GUI_X_WaitEvent(void)
{
	OS_ERR err;
	OSSemPend(EventSem,0,OS_OPT_PEND_BLOCKING,0,&err); 	//请求信号量
}

void GUI_X_SignalEvent(void)
{
	OS_ERR err;
	OSSemPost(EventSem,OS_OPT_POST_1,&err);				//发送信号量
}
/*
*********************************************************************************************************
*                                      KEYBOARD INTERFACE FUNCTIONS
*
* Purpose: The keyboard routines are required only by some widgets.
*          If widgets are not used, they may be eliminated.
*
* Note(s): If uC/OS-II is used, characters typed into the log window will be placed	in the keyboard buffer. 
*          This is a neat feature which allows you to operate your target system without having to use or 
*          even to have a keyboard connected to it. (useful for demos !)
*********************************************************************************************************
*/

static void CheckInit(void)
{
	if(KeyIsInited == DEF_FALSE){
		KeyIsInited = DEF_TRUE;
		GUI_X_Init();
	}
}

void GUI_X_Init(void)
{
	OS_ERR err;
	//创建初始值为1的信号量
	OSSemCreate ((OS_SEM*	)KeySem,	
                 (CPU_CHAR*	)"Key_SEM",
                 (OS_SEM_CTR)0,		
                 (OS_ERR*	)&err);	
}

int GUI_X_GetKey(void) 
{
	int r;
	
	r = KeyPressed;
	CheckInit();
	KeyPressed = 0;
	return (r);
}

int GUI_X_WaitKey(void)
{
	int r;
	OS_ERR err;
	
	CheckInit();
	if(KeyPressed == 0)
	{
		OSSemPend(KeySem,0,OS_OPT_PEND_BLOCKING,0,&err); 	//请求信号量
	}
	r = KeyPressed;
	KeyPressed = 0;
	return (r);
}

void GUI_X_StoreKey(int k)
{
	OS_ERR err;
	KeyPressed = k;
	OSSemPost(KeySem,OS_OPT_POST_1,&err);				//发送信号量
}

/*********************************************************************
*
*      Logging: OS dependent

Note:
  Logging is used in higher debug levels only. The typical target
  build does not use logging and does therefor not require any of
  the logging routines below. For a release build without logging
  the routines below may be eliminated to save some space.
  (If the linker is not function aware and eliminates unreferenced
  functions automatically)

*/

void GUI_X_Log     (const char *s) { GUI_USE_PARA(s); }
void GUI_X_Warn    (const char *s) { GUI_USE_PARA(s); }
void GUI_X_ErrorOut(const char *s) { GUI_USE_PARA(s); }

