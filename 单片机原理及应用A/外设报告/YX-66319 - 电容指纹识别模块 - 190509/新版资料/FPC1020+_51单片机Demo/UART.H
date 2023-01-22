#ifndef __UART_H__
#define __UART_H__

typedef unsigned char uchar;
typedef unsigned int uint;

void SendData( unsigned char dat);
void UartInit(void)	;
void UartSend(uchar *Datar,uchar cLength);
uchar ReadPacket();
void delay(uint ms);
#endif

