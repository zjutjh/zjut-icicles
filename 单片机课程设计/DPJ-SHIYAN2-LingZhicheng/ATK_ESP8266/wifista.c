#include "common.h"
#include "stdlib.h"
#include "value.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//ATK-ESP8266 WIFI模块 WIFI STA驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2015/4/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

//ATK-ESP8266 WIFI STA测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 netpro=0;	//网络模式
u8 atk_8266_wifista_test(void)
{
	//u8 netpro=0;	//网络模式
	u8 key;
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u16 t=999;		//加速第一次获取链接状态
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//连接状态
	p=mymalloc(SRAMIN,32);							//申请32字节内存
	atk_8266_send_cmd("AT+CWMODE=1","OK",50);		//设置WIFI STA模式
	atk_8266_send_cmd("AT+RST","OK",20);		//DHCP服务器关闭(仅AP模式有效) 
	delay_ms(1000);         //延时3S等待重启成功
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	//设置连接到的WIFI网络名称/加密方式/密码,这几个参数需要根据您自己的路由器设置进行修改!! 
	sprintf((char*)p,"AT+CWJAP=\"%s\",\"%s\"",wifista_ssid,wifista_password);//设置无线参数:ssid,密码
	while(atk_8266_send_cmd(p,"WIFI GOT IP",300));					//连接目标路由器,并且获得IP
PRESTA:
	    //UDP
	 
//				LCD_Clear(WHITE);
//				POINT_COLOR=RED;
				//Show_Str_Mid(0,30,"ATK-ESP WIFI-STA 测试",16,240); 
				//Show_Str(30,50,200,16,"正在配置ATK-ESP模块,请稍等...",12,0);
				//if(atk_8266_ip_set("WIFI-STA 远端UDP IP设置",(u8*)ATK_ESP8266_WORKMODE_TBL[netpro],(u8*)portnum,ipbuf))goto PRESTA;	//IP输入
				sprintf((char*)p,"AT+CIPSTART=\"UDP\",\"255.255.255.255\",8080,8086");    //配置目标UDP服务器 8080 远程端口 8086 本地端口   ipbuf,,(u8*)portnum
				delay_ms(200);
				atk_8266_send_cmd("AT+CIPMUX=0","OK",20);  //单链接模式
				delay_ms(200);
//				LCD_Clear(WHITE);
				while(atk_8266_send_cmd(p,"OK",500));
	 
	 
//			LCD_Clear(WHITE);
//			POINT_COLOR=RED;
			//Show_Str_Mid(0,30,"ATK-ESP WIFI-STA 测试",16,240);
			//Show_Str(30,50,200,16,"正在配置ATK-ESP模块,请稍等...",12,0);			
			//LCD_Fill(30,50,239,50+12,WHITE);			//清除之前的显示
			//Show_Str(30,50,200,16,"WK_UP:退出测试  KEY0:发送数据",12,0);
			//LCD_Fill(30,80,239,80+12,WHITE);
			atk_8266_get_wanip(ipbuf);//服务器模式,获取WAN IP
			//sprintf((char*)p,"IP地址:%s 端口:%s",ipbuf,(u8*)portnum);
			//Show_Str(30,65,200,12,p,12,0);				//显示IP地址和端口	
			//Show_Str(30,80,200,12,"状态:",12,0); 		//连接状态
			//Show_Str(120,80,200,12,"模式:",12,0); 		//连接状态
			//Show_Str(30,100,200,12,"发送数据:",12,0); 	//发送数据
			//Show_Str(30,115,200,12,"接收数据:",12,0);	//接收数据
			//atk_8266_wificonf_show(30,180,"请设置路由器无线参数为:",(u8*)wifista_ssid,(u8*)wifista_encryption,(u8*)wifista_password);
			//POINT_COLOR=BLUE;
			//Show_Str(120+30,80,200,12,(u8*)ATK_ESP8266_WORKMODE_TBL[netpro],12,0); 		//连接状态
			USART3_RX_STA=0;
			while(1)
			{
				key=KEY_Scan(0);
				if(key==WKUP_PRES)			//WK_UP 退出测试		 
				{ 
					res=0;					
					atk_8266_quit_trans();	//退出透传
					atk_8266_send_cmd("AT+CIPMODE=0","OK",20);   //关闭透传模式
					break;												 
				}
				else if(key==KEY0_PRES)	//KEY0 发送数据 
				{
				
					if((netpro==3)||(netpro==2))   //UDP
					{
						sprintf((char*)p,"ATK-8266%s测试%02d\r\n",ATK_ESP8266_WORKMODE_TBL[netpro],t/10);//测试数据
						//Show_Str(30+54,100,200,12,p,12,0);
						atk_8266_send_cmd("AT+CIPSEND=25","OK",200);  //发送指定长度的数据
						delay_ms(200);
						atk_8266_send_data(p,"OK",100);  //发送指定长度的数据
						timex=100;
					}
					else if((netpro==1))   //TCP Client
					{
						atk_8266_quit_trans();
						atk_8266_send_cmd("AT+CIPSEND","OK",20);         //开始透传           
						sprintf((char*)p,"ATK-8266%s测试%d\r\n",ATK_ESP8266_WORKMODE_TBL[netpro],t/10);//测试数据
						//Show_Str(30+54,100,200,12,p,12,0);
						u3_printf("%s",p);
						timex=100;
					}
					else    //TCP Server
					{
						sprintf((char*)p,"ATK-8266%s测试%02d\r\n",ATK_ESP8266_WORKMODE_TBL[netpro],t/10);//测试数据
						//Show_Str(30+54,100,200,12,p,12,0);
						atk_8266_send_cmd("AT+CIPSEND=0,25","OK",200);  //发送指定长度的数据
						delay_ms(200);
						atk_8266_send_data(p,"OK",100);  //发送指定长度的数据
						timex=100;
					}
				}else;
			
				if(timex)timex--;
				if(timex==1)LCD_Fill(30+54,100,239,112,WHITE);
				t++;
				delay_ms(10);
				if(USART3_RX_STA&0X8000)		//接收到一次数据了
				{ 
					rlen=USART3_RX_STA&0X7FFF;	//得到本次接收到的数据长度
					USART3_RX_BUF[rlen]=0;		//添加结束符 
					printf("%s",USART3_RX_BUF);	//发送到串口   
					sprintf((char*)p,"收到%d字节,内容如下",rlen);//接收到的字节数 
					LCD_Fill(30+54,115,239,130,WHITE);
					POINT_COLOR=BRED;
					//Show_Str(30+54,115,156,12,p,12,0); 			//显示接收到的数据长度
					POINT_COLOR=BLUE;
					LCD_Fill(30,130,239,319,WHITE);
					//Show_Str(30,130,180,190,USART3_RX_BUF,12,0);//显示接收到的数据  
					USART3_RX_STA=0;
					if(constate!='+')t=1000;		//状态为还未连接,立即更新连接状态
					else t=0;                   //状态为已经连接了,10秒后再检查
				}  
				if(t==1000)//连续10秒钟没有收到任何数据,检查连接是不是还存在.
				{
//			//		LCD_Fill(30+54,125,239,130,WHITE);
//					LCD_Fill(60,80,120,92,WHITE);
					constate=atk_8266_consta_check();//得到连接状态
//					if(constate=='+')//Show_Str(30+30,80,200,12,"连接成功",12,0);  //连接状态
//					else //Show_Str(30+30,80,200,12,"连接失败",12,0); 	 
					t=0;
				}
				if((t%20)==0)LED0=!LED0;
				atk_8266_at_response(1);
			}
	myfree(SRAMIN,p);		//释放内存 
	return res;		
} 




























