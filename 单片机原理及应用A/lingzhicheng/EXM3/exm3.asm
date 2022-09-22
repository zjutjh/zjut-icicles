    ORG     0000H   
    LJMP    MAIN
    
    ORG     0003H   //INT0
    AJMP    INTT0
    
    ORG     0013H   //INT1
    AJMP    INTT1
    
    ORG     0100H
MAIN:
    MOV     SP,#60H
    MOV     DPTR,#0500H
    MOV     R1,#40H       //放到40H~4FH
    MOV     R3,#16        //转移16个
    LCALL   LOOP
    
    SETB    EX0     //允许中断
	SETB    IT0     //跳沿触发
	SETB    EX1
	SETB    IT1
    SETB    EA      //开总中断，或者用IE设置
    MOV		IP,#01H //中断优先级设置
    MOV	    P3,#0FFH
    MOV     P0,#00H
    
SHOW:
    MOV		A,#00H				//38译码器动态显示左起第1个
	MOV 	A,R2
	MOVC 	A,@A+DPTR    
	MOV 	P2,#00
	MOV 	P0,A
	ACALL 	DELAY
    
    MOV		A,#00H				//38译码器动态显示左起第2个
	MOV 	A,R1	
	MOVC 	A,@A+DPTR        
	MOV 	P2,#01
	MOV 	P0,A
	ACALL 	DELAY
    
    MOV		A,#00H				//38译码器动态显示左起第3个
	MOV 	A,R0	
	MOVC 	A,@A+DPTR        
	MOV 	P2,#02
	MOV 	P0,A
	ACALL 	DELAY

	LJMP 	SHOW	

    SJMP    $

LOOP:
    MOV     A,#00H		
    MOVC    A,@A+DPTR
    MOV     @R1,A
    INC     R1
    INC     DPTR
    DJNZ    R3,LOOP     //完成16个数据转移
    MOV     R2,#00H
    MOV     R3,#00H
    
PAIXU:
	MOV     R0,#40H
	MOV     A,@R0
	MOV     R3,A
F1:	
	INC     R0
	MOV     A,@R0
	SUBB    A,R3
	JNC     F2      //若无借位，则高位比低位大，无需交换
	MOV     A,@R0   //若有借位，交换操作
	MOV     R2,A
	MOV     A,R3
	MOV     @R0,A
	DEC     R0
	MOV     A,R2
	MOV     @R0,A
	INC     R0
	CLR     C
	CJNE    R0,#4FH,F1
F2:
	MOV     A,@R0
	MOV     R3,A
	CLR     C
	CJNE    R0,#4FH,F1
	INC     R1
	CJNE    R1,#4FH,PAIXU
    
    MOV     R0,#4FH     //存最大数R0 最小数R1
    MOV     A,@R0
    MOV     R0,A
    MOV     R1,#40H
    MOV     A,@R1
    MOV     R1,A
    RET

INTT0:
    CLR     EA
    PUSH    ACC
    SETB    EA
    
    MOV	    A,P3
    CPL	    A
    JZ      RETURN
    LCALL   D10ms
    MOV	    A,P3
    CPL	    A   
    JZ      RETURN
    MOV     R3,#0FFH
    JB      ACC.2,Pkey1
    
    CLR     EA
    POP     ACC
    SETB    EA
    RETI



INTT1:
    CLR     EA
    PUSH    ACC
    SETB    EA
    
    MOV	    A,P3
    CPL	    A
    JZ      RETURN
    LCALL   D10ms
    MOV	    A,P3
    CPL	    A   
    JZ      RETURN
    MOV     R3,#0FFH
    JB      ACC.3,Pkey4
    
    CLR     EA
    POP     ACC
    SETB    EA
    RETI

RETURN:
    POP     ACC
    RETI

Pkey1:
    MOV     DPTR,#TABLE
    MOV     R1,#00H
    MOV 	A,R1
	ANL 	A,#0FH
	MOV 	R1,A
    CLR 	A
	
    MOV 	R2,A          	//先清零
    MOV 	R3,A
    MOV 	R4,A
    MOV 	R5,#16      	//共转换十六位数	 
LOOP1:
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
	
    DJNZ 	R5,LOOP1
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
    
    SJMP    RETURN
    
Pkey4:
    MOV     DPTR,#TABLE
    MOV     R1,#00H
    MOV 	A,R1
	ANL 	A,#0FH
	MOV 	R1,A
    CLR 	A
	
    MOV 	R2,A          	//先清零
    MOV 	R3,A
    MOV 	R4,A
    MOV 	R5,#16      	//共转换十六位数	 
LOOP4:
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
	
    DJNZ 	R5,LOOP4
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
    
    DJNZ    R3,Pkey4
    SJMP    RETURN



DELAY:				//定时0.4ms
    MOV     30H,#10
DEL0:MOV    31H,#1
DEL1:MOV    32H,#20
DEL2: 
    DJNZ    32H,DEL2
    DJNZ    31H,DEL1
    DJNZ    30H,DEL0
    RET
    
D10ms:	MOV	    32H,#25
D1:	    MOV	    31H,#200
        DJNZ    31H,$
        DJNZ    32H,D1
        RET
        
        
ORG 0500H
DB 22H,55H,0A0H,11H,0C0H,99H,12H,0B0H,44H,0F1H,77H,33H,0EFH,66H,88H,0D0H


//共阴极0~F
TABLE:	DB		3FH,06H,5BH,4FH,66H,6DH,7DH,07H	
	DB		7FH,6FH,77H,7CH,39H,5EH,79H,71H
	DB		00H
    
END