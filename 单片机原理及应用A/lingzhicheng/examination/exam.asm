    ORG     0000H
    LJMP    MAIN
    ORG     0003H   //INT0
    AJMP    INTT0
    ORG     0100H
MAIN:
    MOV     DPTR,#2000H
    MOV     R0,DPL
    MOV     R1,DPH
    MOV     R2,#00H     
    MOV     R3,#15        //转移15个
    MOV     R6,#0
    MOV     R7,#0
    MOV     DPTR,#0500H
    ACALL   LOOP
    MOV     PSW,#00H
    SETB    EX0     //允许中断
    SETB    IT0     //跳沿触发
    SETB    EA
    MOV     IP,#01H //中断优先级设置
    MOV	    P3,#0FFH


SHOW:
    MOV     P2,R7
    SJMP    $


LOOP:
    MOV     A,#00H		
    PUSH    DPL
    PUSH    DPH
    MOVC    A,@A+DPTR
    MOV     DPL,R0
    MOV     DPH,R1
    MOVX    @DPTR,A
    INC     DPTR
    MOV     R0,DPL
    MOV     R1,DPH
    POP     DPH
    POP     DPL
    INC     DPTR
    DJNZ    R3,LOOP    //完成数据转移
    MOV     R3,#15  
    MOV     DPTR,#2000H

COUNT:
    MOV     B,#2
    MOVX    A,@DPTR
    INC     DPTR
    DIV     AB
    MOV     A,B
    JZ      COUNT_ODD

COUNT_EVEN:
    INC     R6
    DJNZ    R3,COUNT
    LJMP    WAIT

COUNT_ODD:
    INC     R7
    DJNZ    R3,COUNT
    LJMP    WAIT


INTT0:
    CLR     EA
    PUSH    PSW
    PUSH    ACC
    SETB    EA
    
    MOV	    A,P3
    CPL	    A
    JZ      RETURN
    LCALL   D10ms
    MOV	    A,P3
    CPL	    A   
    JZ      RETURN
    MOV     R4,A
    MOV     A,R7
    MOV     B,#10
    DIV     AB
    MOV     R0,A
    MOV     R1,B
    MOV     A,R4
    MOV     R3,#0FFH
    JB      ACC.2,Pkey0
    
    CLR     EA
    POP     ACC
    POP     PSW
    SETB    EA
    RETI

Pkey0:
    MOV     DPTR,#TABLE
    
    MOV		A,#00H				//38译码器动态显示左起第1个
	MOV 	A,R0
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
    
    DJNZ    R3,Pkey0
    SJMP    RETURN
    
RETURN:
    RETI
    
WAIT:
    RET
    
DELAY:				//定时0.4ms
    MOV     30H,#10
DEL0:MOV    31H,#1
DEL1:MOV    32H,#20
DEL2: 
    DJNZ    32H,DEL2
    DJNZ    31H,DEL1
    DJNZ    30H,DEL0

D10ms:	MOV	    32H,#25
D1:	    MOV	    31H,#200
        DJNZ    31H,$
        DJNZ    32H,D1
        RET
    
    
ORG 0500H
DB 21H,55H,60H,11H,0C0H,98H,00H,0B0H,44H,50H,77H,33H,0E0H,66H,88H


//共阴极0~9
TABLE:
    DB      3FH,06H,5BH,4FH,66H,6DH,7DH,07H	
    DB      7FH,6FH
    DB		00H
        
    END