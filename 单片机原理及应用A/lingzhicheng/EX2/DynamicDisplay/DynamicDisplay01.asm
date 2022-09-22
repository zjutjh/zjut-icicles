		ORG 	0000H
		AJMP	START
		ORG		0100H
START:	
		MOV		SP,#60H
		
MAIN:	MOV   	R7,#08H; 显示八位
		//MOV		R6,#80H;1000‘0000B，第一次左移后从最低位开始显示
		MOV		R6,#0
		MOV		R0,#30H;显示缓存区30H
LOOP:
		//MOV 	A,R6;读取显示位
		//RL		A;显示位左移
		//MOV		R6,A;放回R6暂存显示位
		//MOV		P2,A;向P2输出显示位
		MOV		P2,R6
		INC		R6
		MOV		A,@R0;读取显示值
		MOV		P0,A;输出送入P0口
		LCALL	DELAY;调用延时子程序
		INC		R0;显示缓存区+1
		DJNZ	R7,LOOP;判断是否显示完八位，未完则继续loop
		AJMP	MAIN
		
DELAY:	MOV		R2,#10;延时子程序10*1*20*2=0.4ms
D2:		MOV		R3,#1
D1:		MOV		R4,#20
D3:		DJNZ	R4,D3
		DJNZ	R3,D1
		DJNZ	R2,D2
		RET

		END
		