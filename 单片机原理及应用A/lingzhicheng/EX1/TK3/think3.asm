ORG 0000H

LJMP MAIN
ORG 0050H
	MAIN:
		MOV SP,#60H
		MOV DPTR,#0200H
		MOV R0,#40H
		MOV R2,#00H
		MOV R3,#00H
	F1:
		MOV A,R2
		MOVC A,@A+DPTR
		MOV @R0,A
		SUBB A,R3
		JC F2
		
		MOV A,R2
		MOVC A,@A+DPTR
		MOV R3,A;如果没有借位则把最大数暂存R3
	F2:	
		CLR C;循环十六次，先找出最大数，并先存入40H--4FH
		INC R0
		INC R2
		CJNE R2,#10H,F1
		
		MOV R1,#40H
	PAIXU:
		MOV R0,#40H
		MOV A,@R0
		MOV R3,A
	F3:	
		INC R0
		MOV A,@R0
		SUBB A,R3
		JNC F4;若无借位，则高位比低位大，无需交换
		MOV A,@R0;若有借位，交换操作
		MOV R2,A
		MOV A,R3
		MOV @R0,A
		DEC R0
		MOV A,R2
		MOV @R0,A
		INC R0
		CLR C
		CJNE R0,#4FH,F3
	F4:
		MOV A,@R0
		MOV R3,A
		CLR C
		CJNE R0,#4FH,F3
		
		INC R1
		CJNE R1,#4FH,PAIXU
		
		SJMP $
		END	
		
		
		