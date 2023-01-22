/************************************************************************************************************************************
 硬件接口：---K1----------P2^0---
           ---K2----------P2^1--- 
           ---K3----------P2^2---
           ---K4----------P2^3---
           ---K5----------P2^4---
           ---BEEP--------P2^5---
					 
硬件环境：单片机：STC90C516RD+
          晶振  ：11.0592M
					波特率：19200
					
****************************************************************************************************************************************

 功能按键：K1 添加指纹  ；K2 删除单个指纹  ；K3 搜索指纹  ：K4 读指纹用户总数   k5 清空指纹库
 
 注释；1、每个操作成功都是亮4个灯加蜂鸣器滴1声，操作失败时8个灯全亮，蜂鸣器滴3声。
       2、搜索指纹成功后，匹配到的用户ID号通过8个LED灯以二进制的形式显示。
       3、读取到的指纹用户总数也是通过8个LED灯以二进制的形式显示。
       4、模块上电后进行添加指纹时， 指纹膜板存放地址从0开始依次增加（已经录入的指纹掉电后也能保存）			 
************************************************************************************************************************************/
#include <reg52.h>
#include "config.h"

	extern unsigned char l_ucFPID; //用户编号

void BBP(unsigned char times);

void main()
{
	unsigned char MODE, P2_BUS, User_ID;
	UartInit();
	while (1)
	{
		P2_BUS = P2 & 0x1F;
		if (P2_BUS != 0x1f)
		{
			Delay_ms(10);
			P2_BUS = P2 & 0x1F;
			if (P2_BUS != 0x1f)
			{
				if (~P2_BUS == 0xe1)
					MODE = 1;
				if (~P2_BUS == 0xe2)
					MODE = 2;
				if (~P2_BUS == 0xe4)
					MODE = 3;
				if (~P2_BUS == 0xe8)
					MODE = 4;
				if (~P2_BUS == 0xf0)
					MODE = 5;
				while (P2_BUS != 0x1f)
					P2_BUS = P2 & 0x1F;
			}
		}
		switch (MODE)
		{
		case 0:
			LED8 = 0xFF; //无操作
			break;

		case 1:
			MODE = 0; // 增加指纹用户
			if (Finger_Enroll(++User_ID))
			{
				LED8 = ~0x0f;
				BBP(1);
			}
			else
			{
				LED8 = ~0xff;
				BBP(3);
			}
			Delay_ms(2000);
			break;

		case 2:
			MODE = 0; // 删除单个指纹用户
			if (Finger_Search())
			{
				LED8 = ~l_ucFPID;
			}
			else
			{
				LED8 = ~0xff;
				BBP(3);
				break;
			}
			if (Finger_Delete(l_ucFPID))
			{
				LED8 = ~0x0f;
				BBP(1);
			}
			else
			{
				LED8 = ~0xff;
				BBP(3);
			}
			Delay_ms(1000);
			break;

		case 3:
			MODE = 0; //1：N搜索指纹库
			if (Finger_Search())
			{
				LED8 = ~l_ucFPID;
				BBP(1);
			}
			else
			{
				LED8 = ~0xff;
				BBP(3);
			}
			Delay_ms(1000);
			break;
		case 4:
			MODE = 0; // 读指纹用户总数
			if (Finger_Read())
			{
				LED8 = ~l_ucFPID;
				BBP(1);
			}
			else
			{
				LED8 = ~0xff;
				BBP(3);
			}
			Delay_ms(1000);
			break;
		case 5:
			MODE = 0; // 清除指纹库
			if (Finger_Clear())
			{
				LED8 = ~0x0f;
				BBP(1);
			}
			else
			{
				LED8 = ~0xff;
				BBP(3);
			}
			Delay_ms(1000);
			break;
		default:
			break;
		}
	}
}
void BBP(unsigned char times)
{
	uchar i;
	switch (times)
	{
	case 1:
		for (i = 0; i < 100; i++)
		{
			BEEP = ~BEEP;
			Delay_ms(1);
		}
		break;
	case 2:
		for (i = 0; i < 100; i++)
		{
			BEEP = ~BEEP;
			Delay_ms(1);
		}
		Delay_ms(200);
		for (i = 0; i < 100; i++)
		{
			BEEP = ~BEEP;
			Delay_ms(1);
		}
		break;
	case 3:
		for (i = 0; i < 100; i++)
		{
			BEEP = ~BEEP;
			Delay_ms(1);
		}
		Delay_ms(200);
		for (i = 0; i < 100; i++)
		{
			BEEP = ~BEEP;
			Delay_ms(1);
		}
		Delay_ms(200);
		for (i = 0; i < 100; i++)
		{
			BEEP = ~BEEP;
			Delay_ms(1);
		}
		break;
	default:
		break;
	}
}