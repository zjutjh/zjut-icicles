//连接单片机的 JP10 到动态数码管的 J12
//分别连接单片机的 P20、P21、P22 到 74HC138 模块的 A、B、C。 
//分别连接单片机的 P34、P35、P36、P37 到AD/DA模块的 DI、CS、CLK、DO。
//将NE555 模块的跳线帽 J11 跳开，跳线帽安装在其中一根上，以免丢掉。
ORG 0000H
LJMP MAIN

ORG 0200H
MAIN:
	MOV		SP,#60H
	ACALL 	AD_CHANGE
	
	MOV 	DPTR,#D_table		//动态显示四位
	
	MOV		A,#00H				//38译码器动态显示四位
	MOV 	A,R0				//放个位	
	MOVC 	A,@A+DPTR
	MOV 	P2,#03
	MOV 	P0,A
	ACALL 	Delay

	MOV 	A,#00H
	MOV 	A,R1				//放十位	
	MOVC 	A,@A+DPTR
	MOV 	P2,#02
	MOV 	P0,A
	ACALL 	Delay

	MOV 	A,#00H
	MOV 	A,R2				//放百位	
	MOVC 	A,@A+DPTR
	MOV 	P2,#01
	MOV 	P0,A
	ACALL 	Delay

	MOV 	A,#00H
	MOV 	A,R3				//放千位
	MOVC 	A,@A+DPTR
	MOV		P2,#00
	MOV 	P0,A
	ACALL 	Delay
	
	MOV 	P0,#00H
	LJMP 	MAIN	
	
AD_CHANGE:      			   //获取AD转换结果，共12位，串行读入单片机内，用SPI总线
	INC 	R7				
DELL: 
	DJNZ 	R6,DELL
	CJNE 	R7,#0FFH,RETURN
	MOV 	R7,#00H
	MOV 	R6,#0FFH
	
	MOV 	R0,#0d4H  			//R0控制字  0X94或0XB4电位，0XD4热敏，0XA4光敏
	CLR 	P3.5				//片选CS为低电平，选中XPT2046
	CLR 	P3.6				//时钟脚I/O CLOCK位低电平
	MOV 	R2,#08H	 			//设置循环读入的次数为8
	MOV 	A,R0				//下一次转换的命令从R0送入A
LOOP0:
	RLC		A
	MOV		P3.4,C
	CLR 	P3.6
	NOP	
	SETB	P3.6
	NOP
	DJNZ 	R2,LOOP0
	MOV 	A,#00H
	MOV 	R2,#04H
	NOP
	NOP
	NOP
	NOP
LOOP1:
	MOV 	C,P3.7			//读入上一次的转换结果中的1位
	RRC 	A				//带进位位的循环右移
	
	SETB	P3.6			//一个CLK时钟
	NOP
	CLR 	P3.6
	NOP
	
	DJNZ 	R2,LOOP1		//是否完成8次转换结果读入和命令输出？未完成则继续
	RRC 	A
	RRC 	A
	RRC 	A
	RRC 	A
	MOV 	R1,A			//R1存高4位数据
	MOV 	A,#00H			//A清“0”
	MOV 	R2,#08H			//设置R2循环次数为8，为移入8位数据准备
LOOP2:
	MOV 	C,P3.7			//读入上一次的转换结果中的1位
	RRC 	A				//带进位位的循环右移

	CLR 	P3.6			//一个CLK时钟
	NOP
	SETB 	P3.6
	NOP
	
	DJNZ 	R2,LOOP2		//是否完成8次转换结果读入和命令输出？未完成则继续
	
	SWAP 	A			
	MOV 	R0,A 			//R0存低8位
	SETB 	P3.6
	LJMP 	DATA_HEX_DEC
RETURN:
	RET
	
DATA_HEX_DEC:				//将获取的12位2进制数转换为十进制存在R3 R2 R1 R0；千位 百位 十位 个位里
//	R0-->TL0
//  R1-->TH0
	MOV 	A,R1
	ANL 	A,#0FH
	MOV 	R1,A
    CLR 	A
	
    MOV 	R2,A          	//先清零
    MOV 	R3,A
    MOV 	R4,A
    MOV 	R5,#16      	//共转换十六位数	 
LOOP:
    CLR 	C
    MOV 	A,R0          	//从待转换数的高端移出一位到Cy
    RLC 	A
    MOV 	R0,A
	
    MOV 	A,R1
    RLC 	A
    MOV 	R1,A
	
    MOV 	A,R4           	//送到BCD码的低端
    ADDC 	A,R4           	//带进位的自身相加，相当于左移一位
    DA 		A              	//十进制调整，变成BCD码
    MOV 	R4,A
	
    MOV 	A,R3
    ADDC 	A,R3
    DA 		A
    MOV 	R3,A
	
    MOV 	A,R2
    ADDC 	A,R2
    MOV 	R2,A
	
    DJNZ 	R5,LOOP
//已经把TH1 TL1中的数字，转换成BCD码，送到了R2 R3 R4
//*************************************************//
//分别存入 R3 R2 R1 R0；千位 百位 十位 个位
    MOV		A,R4
    MOV 	B,#16
    DIV 	AB
    MOV 	R1,A
    MOV 	R0,B

    MOV 	A,R2
    MOV 	R4,A

    MOV 	A,R3
    MOV 	B,#16
    DIV 	AB
    MOV 	R3,A
    MOV 	R2,B
    RET 
	
Delay:				//定时0.4ms
    MOV 	30H,#10
DEL0: 
	MOV 	31H,#1
DEL1: 
	MOV 	32H,#20
DEL2: 
	DJNZ 	32H,DEL2
    DJNZ 	31H,DEL1
    DJNZ 	30H,DEL0
	  
	RET  
	
D_table:
    DB 3FH,06H,5BH,4FH,66H,6DH,7DH,07H,7FH,6FH	//共阴极0~9
END
