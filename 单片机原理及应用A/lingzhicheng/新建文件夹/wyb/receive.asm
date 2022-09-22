	ORG 0000H
	LJMP MAIN
	ORG 0023H
	LJMP RECE_STR
	ORG 1000H
MAIN:
	MOV SP,#60H		
	MOV TMOD,#20H 	//定时器1方式2
	MOV TH1,#0F4H	//波特率2400
	MOV TL1,#0F4H
	MOV PCON,#80H	//SMOD为1,波特率加倍
	SETB TR1
	MOV SCON,#50H
	MOV IE,#90H
	MOV R0,#30H
	MOV R7,#0
	MOV R6,#0
HERE:
	SJMP HERE
/*	MOV 29H,#1
	MOV 30H,#0C0H

LOOP:
	MOV A,29H
	MOV R5,29H
	JZ LOOP
	MOV	R1,#30H
	LCALL KEY1;跳转至KEY1
	LJMP LOOP;重复循环
KEY1:
	MOV	P2,#0FFH;P2口全写入1
	MOV	A,P2;读入八个按键的状态，有键按下则为0
	CPL	A;全部取反，有键按下则为1
	JZ KEY1;如果累加器A全为0，则无键按下，跳转至KEY1重新开始
	LCALL D10ms;去抖动
	MOV	A,P2;读入八个按键的状态，有键按下则为0
	CPL	A;全部取反，有键按下则为1
	JZ KEY1;如果累加器A全为0，则无键按下，跳转至KEY1重新开始
	JB ACC.7,PK0
	RET
PK0:
	LCALL PKEY0
	LJMP KEY1
		
PKEY0:
	MOV	P0,@R1
	INC R1
	JNB	P2.0,PKEY0
	LCALL D10ms
	JNB	P2.0,PKEY0
	DJNZ R5,K_RETURN
	MOV R1,#30H
	MOV R5,29H
K_RETURN:
	RET


D10ms:
	MOV	R3,#25
D1:
	MOV	R4,#200
	DJNZ R4,$
	DJNZ R3,D1
	RET	  */
			

RECE_STR:
	PUSH ACC
	PUSH PSW
	MOV P1,#1		//测试点
	CLR RI
	MOV A,SBUF		 //接收呼叫信号
	CJNE A,#06H,SEND15
SEND00:
	MOV A,#00H		 //允许发送
	MOV SBUF,A
	LJMP HERE_RE
SEND15:
	MOV P1,#2		//测试点
	MOV A,#15H		 //不允许发送
	MOV SBUF,A
	LJMP RETURN
HERE_RE:	
	JNB TI,HERE_RE	 
	CLR TI		 
	MOV P1,#4		//测试点
RECE_LEN:
	JNB RI,RECE_LEN		//接收数据长度
	CLR RI		
	MOV A,SBUF			//查询接收长度
	MOV R7,A
	MOV 29H,R7
	INC R6
	MOV P1,#8		//测试点

RECE_DATA:
	JNB RI,RECE_DATA		//接收数据
	CLR RI		
	MOV A,SBUF			//查询接收数据
	MOV @R0,A
	INC R0
	INC R6
	DJNZ R7,RECE_DATA
	MOV P1,#16		//测试点

RECE_YN:
	JNB RI,RECE_YN		//接收校验位
	CLR RI		
	MOV A,SBUF			//查询接收校验位
	SUBB A,R6
	JNZ SENDF0
SEND0F:
	MOV A,#0FH		 //发送正确
	MOV SBUF,A
	LJMP HERE_END
SENDF0:
	MOV P1,#32		//测试点
	MOV A,#0F0H		 //发送不正确
	MOV SBUF,A
	LJMP RETURN
HERE_END:
	JNB TI,HERE_END	 
	CLR TI
	MOV P1,#64		//测试点
	LJMP RETURN		 

RETURN:
	POP PSW
	POP ACC
	RETI

END 