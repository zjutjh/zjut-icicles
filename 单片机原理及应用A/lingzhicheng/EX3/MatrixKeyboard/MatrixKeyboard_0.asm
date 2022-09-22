		ORG		0000H
		AJMP	START
		ORG 	0100H
START:	MOV		SP,#60H
		MOV		DPTR,#TABLE
		LCALL	KEY2
		LJMP	START
		
KEY2:	LCALL 	KS;调用键盘检测子程序ks
		JNZ		K1;无键按下则A为0，有键按下则跳转至K1
		LCALL	D10ms
		RET
K1:		LCALL	D10ms;去抖动10ms
		LCALL	KS;再次检测
		JNZ		K2;仍然有键按下则跳转K2，否则回到KEY2重新开始第一遍检测
		AJMP	KEY2
		
K2:		MOV		R2,#0EFH;1110'1111暂存进R2，列P1.4为0
		MOV 	R4,#00H;0列号送入R4暂存
K3:		MOV		P1,R2;将列扫描值送入P1口，P1.4为0
L0:		JB		P1.0,L1;判0行线电平，P1.0为1则无按下，跳转至L1检测1行
		MOV		A,#00H;若检测出，则将0行首键盘号送进ACC
		AJMP	LK;跳转至LK计算行号+列号
L1:		JB  	P1.1,L2
		MOV		A,#04H
		AJMP	LK
L2:		JB		P1.2,L3
		MOV		A,#08H
		AJMP	LK
L3:		JB		P1.3,NEXT;若0~3行均无检测出，跳入NEXT，准备检测下一列
		MOV		A,#0CH
		AJMP	LK
		
NEXT:	INC		R4;列号加1
		MOV		A,R2;将R2扫描值送入A
		JNB		ACC.7,KEY2;判断A的最高位是否为0，即是已经0111’0000，若是则已扫描完，则返回KEY2
		RL		A;若不是，则A左移一位，扫描下一列
		MOV		R2,A;下一列的A返还给R2
		AJMP	K3;回到K3开始扫描下一列
		
LK:		ADD		A,R4;列号R4，行号ACC
		MOVC	A,@A+DPTR
		MOV		P2,A
		ACALL	K4
		
K4:		LCALL	KS;检测是否键已经松开
		JNZ		K4
		LCALL	D10ms
		JNZ		K4
		LCALL	KEY2
		
KS:		MOV		P1,#0FH
		MOV		A,P1
		XRL		A,#0FH
		RET
		
D10ms:	MOV		R7,#25
D1:		MOV		R6,#200
		DJNZ	R6,$
		DJNZ	R7,D1
		RET
		
TABLE:	DB 		0C0H,0F9H,0A4H,0B0H,99H,92H,82H,0F8H
		DB		80H,90H,88H,83H,0C6H,0A1H,86H,8EH
		DB		0FFH
		END