	ORG		0000H
	LJMP	MAIN
	ORG	  	0100H
MAIN:
	MOV		SP,#60H
	CLR		C
	MOV		R2,#50H
	MOV		51H,#00H;和的字节数清零
	MOV		R0,#30H
	MOV 	R1,#40H
	LCALL	BCD_ADD_BYTES
	SJMP	$
	
BCD_ADD_BYTES:
	MOV		A,@R0;取被加数
	ADDC	A,@R1;求和
	DA		A;十进制调整
	MOV	    @R0,A;保存
	INC		51H;字节增1
	INC		R0;地址增1
	INC 	R1
	DJNZ    50H	,BCD_ADD_BYTES;所有字节未加完继续，否则向下执行
	JC		NEXT;和的最高字节有进位则转移next
	RET
	
NEXT:
	INC 	51H
	MOV 	@R0,#01H
	RET
	
	END