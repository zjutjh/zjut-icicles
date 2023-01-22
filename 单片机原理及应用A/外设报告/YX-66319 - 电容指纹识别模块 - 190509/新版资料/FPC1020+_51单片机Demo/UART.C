#include "reg52.H"
#include "UART.H"
extern unsigned char rBuf[8];          //接收返回信息
extern unsigned char g_ucUartRxEnd ;              //接收返回信息结束标志 
unsigned char p; 

void UartInit(void)		//19200bps@11.0592MHz
{
	PCON |= 0x80;		//使能波特率倍速位SMOD
	SCON = 0x50;		//8位数据,可变波特率
	TMOD &= 0x0F;		//清除定时器1模式位
	TMOD |= 0x20;		//设定定时器1为8位自动重装方式
	TL1 = 0xFD;		//设定定时初值
	TH1 = 0xFD;		//设定定时器重装值
	ET1 = 0;		//禁止定时器1中断
	TR1 = 1;		//启动定时器1
	ES = 1 ;
	EA = 1 ;
}

/*----------------------------*/
void SendData( unsigned char dat)
{
    SBUF=dat;
    while(!TI);
    TI=0;
}

/****************************************************************/
void UartSend(uchar *Datar,uchar cLength)
{
	 do
     {
         SendData(*(Datar++));
     }   while (--cLength != 0);
}
/****************************************************************/

void Uart_Isr() interrupt 4 using 1
{    
    if (RI)
    {
        RI = 0;             //Clear receive interrupt flag
			  rBuf[p++] = SBUF;	
			  if(p==8) g_ucUartRxEnd = 0xff;
    }

}