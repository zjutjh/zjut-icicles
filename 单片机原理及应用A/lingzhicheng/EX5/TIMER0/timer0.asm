//P2.0接LED模块J12的A	
	ORG		0000H
RESET:
	LJMP	MAIN
	ORG		000BH;定时器中断0入口
	LJMP	T0_INT
	ORG		0100H
	
MAIN:
	MOV		SP,#60H
	MOV		TMOD,#01H;计时工作方式1
	MOV		30H,#00H;定时器超时一百次，达到64h即一秒
	;12MHz，机器周期1μs，需要10000个计数，初值=65536-10000=55536=D8F0H
	MOV		TH0,#0D8H
	MOV		TL0,#0F0H
	SETB	P2.0
	SETB	ET0;开放T0中断和总中断
	SETB	EA
	SETB	TR0;启动T0
	JNB		TF0,$;wait for TF0

T0_INT:
	PUSH	PSW;保护状态字寄存器
	PUSH	ACC
	CLR		EA;关闭总中断
	MOV		TH0,#0D8H
	MOV		TL0,#0F0H
	INC		30H
	MOV		A,30H
	CJNE	A,#100,CONTINUE_T0
	MOV		30H,#00H;定时器超时一百次，达到64h即一秒
	CPL		P2.0

CONTINUE_T0:
	SETB	EA;开总中断
	POP		ACC
	POP		PSW;保护状态字寄存器
	RETI
	
	END