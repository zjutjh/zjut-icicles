#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "tftlcd.h"
#include "adc.h"
#include "pcf8574.h"
#include "timer.h"
#include "sdram.h"
#include "malloc.h"
#include "touch.h"
#include "GUI.h"
#include "WM.h"
#include "graphyt.h"
#include "includes.h"
#include "DIALOG.h"
#include "Display.h"
#include "value.h"
 

//任务优先级
#define START_TASK_PRIO				3
//任务堆栈大小	
#define START_STK_SIZE 				128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//TOUCH任务
//设置任务优先级
#define TOUCH_TASK_PRIO				4
//任务堆栈大小
#define TOUCH_STK_SIZE				128
//任务控制块
OS_TCB TouchTaskTCB;
//任务堆栈
CPU_STK TOUCH_TASK_STK[TOUCH_STK_SIZE];
//touch任务
void touch_task(void *p_arg);

//LED0任务
//设置任务优先级
#define LED0_TASK_PRIO 				5
//任务堆栈大小
#define LED0_STK_SIZE				628
//任务控制块
OS_TCB Led0TaskTCB;
//任务堆栈
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
//led0任务
void led0_task(void *p_arg);

//EMWINDEMO任务
//设置任务优先级
#define EMWINDEMO_TASK_PRIO			6
//任务堆栈大小
#define EMWINDEMO_STK_SIZE			712
//任务控制块
OS_TCB EmwindemoTaskTCB;
//任务堆栈
CPU_STK EMWINDEMO_TASK_STK[EMWINDEMO_STK_SIZE];
//emwindemo_task任务
void emwindemo_task(void *p_arg);
 //PORT初始化
void PORT_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOF_CLK_ENABLE();   //使能GPIOF时钟
    
	  //PF7用于传感器板和开发板共地
    //PF7初始化设置
    GPIO_Initure.Pin=GPIO_PIN_7;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //快速
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
	
	
	  __HAL_RCC_GPIOH_CLK_ENABLE();   //使能GPIOH时钟
    
	  //PH2,PH3控制MQ7加热电平
    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3 ;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //高速
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
		
		   
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);   //PF7初始化置为0
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);   //PH2初始化置为0,控制Q1
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET);   //PH3初始化置为1，控制Q0
	
}

int main(void)
{
		u8 i=0;
		OS_ERR err;
		CPU_SR_ALLOC();
    
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz  
	
    HAL_Init();                     //初始化HAL库
    delay_init(180);                //初始化延时函数
	
    uart_init(115200);              //初始化USART

	  usart3_init(115200);
	
	
    LED_Init();                     //初始化LED 
    KEY_Init();                     //初始化按键
    PCF8574_Init();                 //初始化PCF8574
    MY_ADC_Init();                  //初始化ADC
    SDRAM_Init();                   //SDRAM初始化
    TFTLCD_Init();  		        		//LCD初始化
    TP_Init();				        			//触摸屏初始化
	
    my_mem_init(SRAMIN);    		    //初始化内部内存池
		my_mem_init(SRAMEX);    		    //初始化外部内存池
		my_mem_init(SRAMCCM);    		    //初始化CCM内存池
		
	
	  SGP30_Init(); 
    SGP30_Start_Measure();
		DHT11_Init(); 
		
		PORT_Init();
		
		for(i=0;i<Dtype;i++)
		  AT24CXX_Read(adrr+i*2,(u8*)&DataInfo.Avalue[i],2) ;//读取报警值参数
	
//		while(1) 
//		{
//			DHT11_Read(data.dht22);
//		
//		}
	 
    
    
    OSInit(&err);		            //初始化UCOSIII
	OS_CRITICAL_ENTER();            //进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	            //退出临界区	 
	OSStart(&err);                  //开启UCOSIII
	while(1);
}

//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	//使能时间片轮转调度功能,设置默认的时间片长度
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	__HAL_RCC_CRC_CLK_ENABLE();		//使能CRC时钟
    WM_SetCreateFlags(WM_CF_MEMDEV); //启动所有窗口的存储设备
	GUI_Init();  			//STemWin初始化
	WM_MULTIBUF_Enable(1);  //开启STemWin多缓冲,RGB屏可能会用到
	OS_CRITICAL_ENTER();	//进入临界区
	//STemWin Demo任务	
	OSTaskCreate((OS_TCB*     )&EmwindemoTaskTCB,		
				 (CPU_CHAR*   )"Emwindemo task", 		
                 (OS_TASK_PTR )emwindemo_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )EMWINDEMO_TASK_PRIO,     
                 (CPU_STK*    )&EMWINDEMO_TASK_STK[0],	
                 (CPU_STK_SIZE)EMWINDEMO_STK_SIZE/10,	
                 (CPU_STK_SIZE)EMWINDEMO_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);
	//触摸屏任务
	OSTaskCreate((OS_TCB*     )&TouchTaskTCB,		
				 (CPU_CHAR*   )"Touch task", 		
                 (OS_TASK_PTR )touch_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )TOUCH_TASK_PRIO,     
                 (CPU_STK*    )&TOUCH_TASK_STK[0],	
                 (CPU_STK_SIZE)TOUCH_STK_SIZE/10,	
                 (CPU_STK_SIZE)TOUCH_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);			 
	//LED0任务
	OSTaskCreate((OS_TCB*     )&Led0TaskTCB,		
				 (CPU_CHAR*   )"Led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK*    )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);	                 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//退出临界区
}
extern WM_HWIN CreateFramewin(void);
extern WM_HWIN CreateIamgeFramewin(void);
extern WM_HWIN CreateInputPassword(void);
//EMWINDEMO任务
u8 mf=1; 
void emwindemo_task(void *p_arg)
{
	GUI_CURSOR_Show();
	//更换皮肤
	BUTTON_SetDefaultSkin(BUTTON_SKIN_FLEX); 
	CHECKBOX_SetDefaultSkin(CHECKBOX_SKIN_FLEX);
	DROPDOWN_SetDefaultSkin(DROPDOWN_SKIN_FLEX);
	FRAMEWIN_SetDefaultSkin(FRAMEWIN_SKIN_FLEX);
	HEADER_SetDefaultSkin(HEADER_SKIN_FLEX);
	MENU_SetDefaultSkin(MENU_SKIN_FLEX);
	MULTIPAGE_SetDefaultSkin(MULTIPAGE_SKIN_FLEX);
	PROGBAR_SetDefaultSkin(PROGBAR_SKIN_FLEX);
	RADIO_SetDefaultSkin(RADIO_SKIN_FLEX);
	SCROLLBAR_SetDefaultSkin(SCROLLBAR_SKIN_FLEX);
	SLIDER_SetDefaultSkin(SLIDER_SKIN_FLEX);
	SPINBOX_SetDefaultSkin(SPINBOX_SKIN_FLEX);
	 CreateIamgeFramewin();//显示图片
	 CreateInputPassword();//密码验证
	 mf=0;//密码输入正确
		while(1)
		{
			CreateFramewin(); 
		}
}

//TOUCH任务
void touch_task(void *p_arg)
{
	OS_ERR err;
	
	while(1)
	{
		GUI_TOUCH_Exec();	
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_PERIODIC,&err);//延时5ms
	}
}

//LED0任务
void led0_task(void *p_arg)
{	CPU_SR_ALLOC();
	OS_ERR err;
	u8 cnt=0,modeT=0xff,key,i=0,res=0;
	 u16 co2 = 0,tt=1000;
	 u16 tvoc = 0;
	 u16 mq2,mq7;
//	u8 constate=0;	//连接状态
	char p[100]; 
  data.adc = mymalloc(SRAMEX, sizeof(adc_dev));
	data.dht22 = mymalloc(SRAMEX, sizeof(dht22_dev));
	data.pms = mymalloc(SRAMEX, sizeof(_pms_dev));
	data.sgp = mymalloc(SRAMEX, sizeof(sgp30_dev));
	
	//DataInfo.mode=1;
  DataInfo.mode=0;
	
	printf("pre-11111111111111111111111111DataInfo.mode=%d\r\n",DataInfo.mode);
	
	DataInfo.wifiF=0;
	DataInfo.RevAdrr=0;//默认接收全部采集终端 数据
	
	//防止第一次采集数据错误 先采集一次
	
	if (USART_RX_STA & 0x8000)//获取PMS5003 数据 PM1.0 2.5 10
	{		 
			Proces_Data(USART_RX_BUF, data.pms) ; 
			USART_RX_STA = 0;  
	}
	
	OS_CRITICAL_ENTER();	//进入临界区
	DHT11_Read(data.dht22);//获取温湿度数据 
	Read_Air_Quality(&co2, &tvoc);
	OS_CRITICAL_EXIT();	//退出临界区
	
	while(mf)
		delay_ms(10);//密码输入正确后在进入采集模式
	
	
	while(1)
	{  
		if(cnt++>50)
		{
			LED0 = !LED0;
			cnt=0;
		}
		
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_PERIODIC,&err);//延时10ms
	  key=KEY_Scan(0);
		
	  if(key==WKUP_PRES)
			//DataInfo.mode=!DataInfo.mode;			//切换工作模式
		DataInfo.mode=0;
		
		else if(key==KEY2_PRES)
		{
			if(DataInfo.mode==0)//wifi模式
		  { 
				DataInfo.RevAdrr++;
				
				if(DataInfo.RevAdrr>9)
					DataInfo.RevAdrr=0;
			}
			
			DataInfo.RevAdrr=0;//使用WIFI0接收、‘
			
			
		}
		
		if(modeT!=DataInfo.mode)//模式切换
		{
		  modeT=DataInfo.mode;//保存，防止每次都进入
			
			if(DataInfo.mode==0)//WiFi模式
			{  
				if(DataInfo.wifiF==0)//未连接 初始化连接
				{
					atk_8266_send_cmd("AT","OK",50);
					
				  atk_8266_send_cmd("AT+CWMODE=1","OK",50);		//设置WIFI STA模式
					atk_8266_send_cmd("AT+RST","OK",20);		//重启模块 
					
					delay_ms(4000);         //延时2S等待重启成功
					
					PCF8574_WriteBit(BEEP_IO,1);
					
				  printf("pre-222222222222222222222\r\n");
					
			  	sprintf((char*)p,"AT+CWJAP=\"%s\",\"%s\"",wifista_ssid,wifista_password);//设置无线参数:ssid,密码
	        while(atk_8266_send_cmd(p,"WIFI GOT IP",300));					//连接目标路由器,并且获得IP
					
					PCF8574_WriteBit(BEEP_IO,1);
					printf("pre-33333333333333333333333\r\n");
					
					delay_ms(1000);
				
					sprintf((char*)p,"AT+CIPSTART=\"UDP\",\"255.255.255.255\",8080,8086");    //配置目标UDP服务器 8080 远程端口 8086 本地端口   ipbuf,,(u8*)portnum
					delay_ms(200);
					atk_8266_send_cmd("AT+CIPMUX=0","OK",20);  //单链接模式
					delay_ms(200); 
					
					while(atk_8266_send_cmd(p,"OK",500)); 
					
					printf("pre-44444444444444444444444444444\r\n");
					
					memset(USART3_RX_BUF,0,sizeof(USART3_RX_BUF));//将缓存区清0
					
				  USART3_RX_STA=0;		   	//启动下一次接收
					
				}
				
			  DataInfo.wifiF=0;// 
			}
			else//采集模式
			{
				SGP30_Init(); //重新初始化
				SGP30_Start_Measure();
				DHT11_Init(); 
		    uart_init(9600);              //初始化USART
			}
		}
//		
		if(DataInfo.mode==1)//采集模式
		{ 
			if(++tt>100)
			{tt=0;
				if (USART_RX_STA & 0x8000)//获取PMS5003 数据 PM1.0 2.5 10
				{		 
						Proces_Data(USART_RX_BUF, data.pms) ; 
						USART_RX_STA = 0;  
				}
			  OS_CRITICAL_ENTER();	//进入临界区
		    DHT11_Read(data.dht22);//获取温湿度数据
			  OS_CRITICAL_EXIT();	//退出临界区
				mq2 = Get_Adc_Average(ADC_CHANNEL_7, 2)*1.22;
			  data.adc->mq2 =mq2;// MQ2_get(mq2); 
			//mq7=	(300- Get_Adc_Average(ADC_CHANNEL_6, 5))*1.5;//O3
		   
				 mq7 = Get_Adc_Average(ADC_CHANNEL_6, 2)*1.22;
			  data.adc->mq7 = MQ7_get(mq7);//mq7;//
				
				Read_Air_Quality(&co2, &tvoc);
					
			  data.sgp->co2 = co2;
			  data.sgp->tvoc = tvoc;
				//处理的数据类型 温度、湿度、MQ2(甲烷)、pm2.5、pm1.0、PM10、MQ7(co)、co2、TVOC
				sprintf(p,"data,%d,%d,%d,%d,%d,%d,%d,%d,%d,ok",//temp,humi,Get_Adc_Average(5,5),data.data5,data.data4,data.data6,Get_Adc_Average(4,5),CO2,TVOC);
								      	data.dht22->temp, 
												data.dht22->humi, 
												data.adc->mq2,
												data.pms->data5,
												data.pms->data4, 
												data.pms->data6, 
												data.adc->mq7,
												data.sgp->co2,
												data.sgp->tvoc);	
			 data_Analysis( (u8*)p);//分析字符串
       DataInfo.update=1;//刷新标志置1
		   DataInfo.qxdate=1;//刷新标志置1
			 BjzBj();	// 报警值比较函数
		 } 
		}   
    else //WiFi 接收模式
		{
		//	printf("pre-5555555555555555555555555\r\n");   
			if(USART3_RX_STA&0X8000)		//接收到一次数据了
			{ 
				printf("pre-6666666666666666666666666666666\r\n");
				
				res=data_Analysis( (u8*)USART3_RX_BUF);//分析字符串
				
				if(res)//解析成功 且是需要接收的设备ID
				{
					DataInfo.Wificnt++;  
					for(i=0;i<90;i++)
						DataInfo.Wifidat[i]=USART3_RX_BUF[i];
					
					DataInfo.Wifidat[i]=0; 
				}
				
				memset(USART3_RX_BUF,0,sizeof(USART3_RX_BUF));//将缓存区清0
				USART3_RX_STA=0;		   	//启动下一次接收
				DataInfo.update=1;//刷新标志置1
				DataInfo.qxdate=1;//刷新标志置1
			
				BjzBj();	// 报警值比较函数
				DataInfo.wifiF=1;//能接受数据标志已连接
			} 
			
			
			
		}
		
		
		
		
		
		
	}
}



