//交通灯模块、动态数码管模块，利用定时器、十进制数拆分
//用 8 根的排线连接单片机 P0 的 JP10 到动态数码管的 J12，用 8 根的排线连接单片机 P1 的 JP8 到 交通灯模块的 JP1
//单片机的 P22、P23、P24 分别接到 74HC138 模块的A、B、C。JP165 跳线帽一定要拔掉，插在其中一根针上，以免丢掉。
//南北绿灯30s，红灯30s；黄灯均为5s；东西绿灯25s，红灯35s，
//人行道不显示倒计时，车道显示倒计时
	ORG 		0000H
RESET:
	AJMP		MAIN
	ORG 		000BH			//定时器中断0入口
	AJMP 		INTI0
	ORG 		0100H
		
MAIN:
	MOV			SP,#60H			//计时工作方式1
	MOV		 	TMOD,#01H
	MOV			DPTR,#TABLE
//12MHz，机器周期1μs，需要10000个计数，初值=65536-10000=55536=D8F0H
	MOV			TH0,#0D8H
	MOV			TL0,#0F0H
	SETB		ET0				//开放T0中断和总中断
	SETB		EA
	SETB 		TR0 			//启动T0
	MOV 		R0,#01			//计数器初值为01
	MOV			30H,#00H		//定时器超时一百次，达到64h即一秒
	MOV			31H,#30			//南北方向红黄绿时间，30，5，30	
	MOV			32H,#35			//东西方向红黄绿时间，35，5，25
	MOV 		P1,#11001101B	//110东西红'011南北绿'01人行南北绿
	CLR 		P3.0	  		//10人行东西红
	SETB 		P3.1
	LCALL		CALCU
	LJMP		DISPLAY

CALCU:
/***********南北方向倒计时计算********************/
 	MOV 		R1,#10			//除数为10
	MOV 		A,31H			//倒计时数字放到A
	MOV 		B,R1			//除数10放到B
	DIV 		AB				//A为商，B为余数
	MOV 		R4,A			//R4存放商
	MOV 		R5,B			//R5存放余数
/***********东西方向倒计时计算********************/
	MOV 		R1,#10			//除数为10
	MOV 		A,32H			//倒计时数字放到A
	MOV 		B,R1			//除数10放到B
	DIV 		AB				//A为商，B为余数
	MOV 		R6,A			//R6存放商
	MOV 		R7,B			//R7存放余数
	RET
	
DISPLAY:
/***********南北方向倒计时显示********************/
	CLR			P2.2
	CLR			P2.3
	CLR			P2.4
	MOV 		A,R4			//LED0输出商
	MOVC 		A,@A+DPTR
	MOV 		P0,A
	LCALL 		D04MS
	
	SETB		P2.2
	MOV 		A,R5			//LED1输出余数
	MOVC 		A,@A+DPTR
	MOV 		P0,A
/***********东西方向倒计时显示********************/
	CLR			P2.2
	CLR			P2.3
	SETB		P2.4
	LCALL 		D04MS 
	MOV 		A,R6			//LED4输出商
	MOVC 		A,@A+DPTR
	MOV 		P0,A
	LCALL 		D04MS
	
	SETB		P2.2
	MOV 		A,R7			//LED5输出余数
	MOVC 		A,@A+DPTR
	MOV 		P0,A
/*********************************************/
	JNB			TF0,DISPLAY		//wait for TF0

INTI0:
	PUSH		PSW				//保护状态字寄存器
	PUSH		ACC
	CLR			EA				//关闭总中断
	MOV			TH0,#0D8H		//定时10ms
	MOV			TL0,#0F0H
	INC			30H
	MOV			A,30H
	CJNE		A,#100,OUT
	MOV			30H,#00H		//定时器超时一百次，达到64h即一秒
	INC 		R0				//计数器+1
	DEC			31H
	DEC			32H
	

/***********四种状态互相转变********************/
JMP5SN:
	CJNE 		R0,#31,JMP30EW
	MOV 		P1,#11010110B	//110东西红'101南北黄'10人行南北红
	CLR 		P3.0	  		//10人行东西红
	SETB 		P3.1
	MOV			31H,#5	
	LCALL		CALCU
	SJMP 		OUT
JMP30EW:
	CJNE 		R0,#36,JMP5EW
	MOV 		P1,#01111010B	//011东西绿'110南北红'10人行南北红
	SETB 		P3.0	  		//01人行东西绿
	CLR 		P3.1
	MOV			31H,#30
	MOV			32H,#25
	LCALL		CALCU
	SJMP 		OUT
JMP5EW:
	CJNE 		R0,#66,JMP30SN
	MOV 		P1,#10111010B	//101东西黄'110南北红'10人行南北红
	SETB 		P3.0	  		//01人行东西绿
	CLR 		P3.1
	MOV			32H,#5
	LCALL		CALCU
	SJMP 		OUT
JMP30SN:
	CJNE 		R0,#71,OUT
	MOV 		R0,#1
	MOV 		P1,#11001101B	//110东西红'011南北绿'01人行南北绿
	CLR 		P3.0	  		//10人行东西红
	SETB 		P3.1
	MOV			31H,#30
	MOV			32H,#35			
	LCALL		CALCU
	SJMP 		OUT

OUT:
	SETB		EA				//开总中断
	POP			ACC
	POP			PSW				//保护状态字寄存器
	RETI

D04MS:							//延时0.4ms
	MOV 		R3,#2
D1:			 
	MOV 		R2,#10
D2:
	DJNZ 		R2,D2
	DJNZ 		R3,D1
	RET

TABLE:	DB		3FH,06H,5BH,4FH,66H,6DH,7DH,07H	//共阴极0~9
		DB		7FH,6FH
		DB		00H
/*TABLE:	DB		3FH,06H,5BH,4FH,66H,6DH,7DH,07H	//共阴极0~F
			DB		7FH,6FH,77H,7CH,39H,5EH,79H,71H
			DB		00H*/
			
/*TABLE:	DB 		0C0H,0F9H,0A4H,0B0H,99H,92H,82H,0F8H//共阴极0~9
			DB		80H,90H
			DB 		0FFH*/	   

/*TABLE:	DB 		0C0H,0F9H,0A4H,0B0H,99H,92H,82H,0F8H//共阴极0~F
			DB		80H,90H,88H,83H,0C6H,0A1H,86H,8EH
			DB 		0FFH*/
			
		END