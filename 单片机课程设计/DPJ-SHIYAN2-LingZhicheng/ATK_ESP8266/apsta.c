#include "common.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//ATK-ESP8266 AP+STA模式驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/4/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 


//ATK-ESP8266 AP+STA模式测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 atk_8266_apsta_test(void)
{
	u8 netpro;
	u8 key=0;
	u8 timex=0; 
	u8 ipbuf[16]; 	//IP缓存
	u8 *p;
	u16 t=999;		//加速第一次获取链接状态
	u8 res=0;
	u16 rlen=0;
	u8 constate=0;	//连接状态
	p=mymalloc(SRAMIN,100);							//申请32字节内存
	atk_8266_send_cmd("AT+CWMODE=3","OK",50);		//设置WIFI AP+STA模式
//	atk_8266_send_cmd("AT+RST","OK",20);		//重启模块 
//	delay_ms(1000);         //延时2S等待重启成功
//	delay_ms(1000);
//	delay_ms(1000);
	//设置模块AP模式的WIFI网络名称/加密方式/密码，这几个参数看自己喜好设置
	sprintf((char*)p,"AT+CWSAP=\"%s\",\"%s\",1,4",wifiap_ssid,wifiap_password);//设置无线参数:ssid,密码
	atk_8266_send_cmd(p,"OK",1000);					//设置AP模式参数
	//设置连接到的WIFI网络名称/加密方式/密码,这几个参数需要根据您自己的路由器设置进行修改!! 
	sprintf((char*)p,"AT+CWJAP=\"%s\",\"%s\"",wifista_ssid,wifista_password);//设置无线参数:ssid,密码
	atk_8266_send_cmd(p,"WIFI GOT IP",1000);						//连接目标路由器，并获得IP
	while(atk_8266_send_cmd("AT+CIFSR","STAIP",20));   //检测是否获得STA IP
	while(atk_8266_send_cmd("AT+CIFSR","APIP",20));   //检测是否获得AP IP
//	while((atk_8266_consta_check()-'2'));   //获得IP
	LCD_Clear(WHITE);
	POINT_COLOR=RED;
	Show_Str(30,30,200,16,"ATK-ESP AP+STA模式测试",16,0); 
	atk_8266_send_cmd("AT+CIPMUX=1","OK",50);   //0：单连接，1：多连接
	delay_ms(500);
	sprintf((char*)p,"AT+CIPSERVER=1,%s",(u8*)portnum);
	atk_8266_send_cmd(p,"OK",50);     //开启Server模式，端口号为8086
	delay_ms(500);
	atk_8266_send_cmd("AT+CIPSTO=1200","OK",50);     //设置服务器超时时间
	
PRESTA:
	netpro=atk_8266_netpro_sel(50,30,(u8*)ATK_ESP8266_CWMODE_TBL[0]);	//AP+STA模式网络模式选择
	if(netpro&0X02)   //STA UDP
	{
				LCD_Clear(WHITE);
				POINT_COLOR=RED;
				Show_Str_Mid(0,30,"ATK-ESP WIFI-STA测试",16,240); 
				Show_Str(30,50,200,16,"正在配置ATK-ESP模块,请稍等...",12,0);
				if(atk_8266_ip_set("WIFI-STA远端UDP IP设置","UDP模式",(u8*)portnum,ipbuf))goto PRESTA;	//IP输入
			//	sprintf((char*)p,"AT+CIPSTART=0,\"UDP\",\"%s\",%s",ipbuf,(u8*)portnum);    //配置目标UDP服务器,及ID号，STA模式下为0
				sprintf((char*)p,"AT+CIPSTART=\"UDP\",\"%s\",%s,8086",ipbuf,(u8*)portnum);    //配置目标UDP服务器,及ID号，STA模式下为0
			
		LCD_Clear(WHITE);
				Show_Str_Mid(0,30,"ATK-ESP WIFI-STA测试",16,240); 
				Show_Str(30,50,200,16,"正在配置ATK-ESP模块,请稍等...",12,0);
				atk_8266_send_cmd(p,"OK",200);
				netpro=atk_8266_mode_cofig(netpro);     //AP模式网络模式配置			
	}
	else     //TCP
	{
		if(netpro&0X01)     //STA TCP Client  
		{
			LCD_Clear(WHITE);
			POINT_COLOR=RED;
			Show_Str_Mid(0,30,"ATK-ESP WIFI-STA 测试",16,240); 
			Show_Str(30,50,200,16,"正在配置ATK-ESP模块,请稍等...",12,0);
			if(atk_8266_ip_set("WIFI-STA 远端IP设置",(u8*)ATK_ESP8266_WORKMODE_TBL[netpro],(u8*)portnum,ipbuf))goto PRESTA;	//IP输入
			sprintf((char*)p,"AT+CIPSTART=0,\"TCP\",\"%s\",%s",ipbuf,(u8*)portnum);    //配置目标TCP服务器,及ID号，STA模式下为0
			while(atk_8266_send_cmd(p,"OK",200))
			{
				LCD_Clear(WHITE);
				POINT_COLOR=RED;
				Show_Str_Mid(0,40,"WK_UP:返回重选",16,240);
				Show_Str(30,80,200,12,"ATK-ESP 连接UDP失败",12,0); //连接失败	 
				key=KEY_Scan(0);
				if(key==WKUP_PRES)goto PRESTA;
			}	
			netpro=atk_8266_mode_cofig(netpro);     //AP模式网络模式配置	
		}
		else netpro=atk_8266_mode_cofig(netpro);     //TCP SERVER不用配置
	}
	
	
	
	LCD_Clear(WHITE);
	POINT_COLOR=RED;
	Show_Str_Mid(0,30,"ATK-ESP WIFI-STA+AP 测试",16,240);
	Show_Str(15,50,200,16,"正在配置ATK-ESP模块,请稍等...",12,0);			
	LCD_Fill(15,50,239,50+12,WHITE);			//清除之前的显示
	Show_Str_Mid(0,50,"WK_UP:退出 KEY0:ID0发送 KEY1:ID1发送",12,240);
	LCD_Fill(15,80,239,80+12,WHITE);
	atk_8266_get_ip(15,65);//STA+AP模式,获取 IP,并显示	
	
	
	Show_Str(15,95,200,12,"连接状态:",12,0); 		//连接状态
	Show_Str(15,110,200,12,"STA模式:",12,0); 		//STA连接状态
	Show_Str(120+15,110,200,12,"AP模式:",12,0); 		//AP连接状态
	Show_Str(15,125,200,12,"发送数据:",12,0); 	//发送数据
	Show_Str(15,140,200,12,"接收数据:",12,0);	//接收数据
	atk_8266_wificonf_show(15,195,"请设置路由器无线参数为:",(u8*)wifista_ssid,(u8*)wifista_encryption,(u8*)wifista_password);
	POINT_COLOR=BLUE;		
	Show_Str(48+15,110,200,12,(u8*)ATK_ESP8266_WORKMODE_TBL[netpro&0X03],12,0); 		//STA连接状态
	Show_Str(162+15,110,200,12,(u8*)ATK_ESP8266_WORKMODE_TBL[netpro>>4],12,0); 		//AP连接状态
	USART3_RX_STA=0;
	while(1)
	{
		key=KEY_Scan(0);
		if(key==WKUP_PRES)			//WK_UP 退出测试		 
		{ 
			res=0;	
			break;												 
		}
		else if(key==KEY0_PRES)	//KEY0 向ID0发送数据 
		{
			  sprintf((char*)p,"ATK-8266模块ID0发数据%02d\r\n",t/10);//测试数据
				Show_Str(15+54,125,200,12,p,12,0);
				atk_8266_send_cmd("AT+CIPSEND=0,25","OK",200);  //发送指定长度的数据
				delay_ms(200);
				atk_8266_send_data(p,"OK",100);  //发送指定长度的数据
				timex=100;
		}
		else if(key==KEY1_PRES)	//KEY1 向ID1发送数据 
		{
			  sprintf((char*)p,"ATK-8266模块ID1发数据%02d\r\n",t/10);//测试数据
				Show_Str(15+54,125,200,12,p,12,0);
				atk_8266_send_cmd("AT+CIPSEND=1,25","OK",200);  //发送指定长度的数据
				delay_ms(200);
				atk_8266_send_data(p,"OK",100);  //发送指定长度的数据
				timex=100;
		}
			
	 if(timex)timex--;
 	 if(timex==1)LCD_Fill(30+54,125,239,122,WHITE);
	 t++;
	 delay_ms(10);
	 if(USART3_RX_STA&0X8000)		//接收到一次数据了
	{ 
		rlen=USART3_RX_STA&0X7FFF;	//得到本次接收到的数据长度
		USART3_RX_BUF[rlen]=0;		//添加结束符 
		printf("%s",USART3_RX_BUF);	//发送到串口   
		sprintf((char*)p,"收到%d字节,内容如下",rlen);//接收到的字节数 
		LCD_Fill(15+54,140,239,130,WHITE);
		POINT_COLOR=BRED;
		Show_Str(15+54,140,156,12,p,12,0); 			//显示接收到的数据长度
		POINT_COLOR=BLUE;
		LCD_Fill(15,155,239,319,WHITE);
		Show_Str(15,155,180,190,USART3_RX_BUF,12,0);//显示接收到的数据  
		USART3_RX_STA=0;
		if(constate!='+')t=1000;		//状态为还未连接,立即更新连接状态
		else t=0;                   //状态为已经连接了,10秒后再检查
	}  
	if(t==1000)//连续10秒钟没有收到任何数据,检查连接是不是还存在.
	{
		LCD_Fill(15+54,125,239,138,WHITE);
//		LCD_Fill(15+54,95,120,110,WHITE);
		constate=atk_8266_consta_check();//得到连接状态
		if(constate=='+')Show_Str(15+54,95,200,12,"连接成功",12,0);  //连接状态
		else Show_Str(15+54,95,200,12,"连接失败",12,0); 	 
		t=0;
	}
	if((t%20)==0)LED0=!LED0;
	atk_8266_at_response(1);
}
	myfree(SRAMIN,p);		//释放内存 
	return res;		
} 


























