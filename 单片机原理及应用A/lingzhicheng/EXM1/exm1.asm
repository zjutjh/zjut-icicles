    ORG     0000H
    LJMP    MAIN
    ORG     0100H
MAIN:
    MOV     DPTR,#0500H
    MOV     R1,#40H     
    MOV     R3,#16        //转移15个

LOOP:
    MOV     A,#00H		
    MOVC    A,@A+DPTR
    MOV     @R1,A
    INC     R1
    INC     DPTR
    DJNZ    R3,LOOP    //完成数据转移
    SJMP    $

ORG 0500H
DB 22H,55H,0A0H,11H,0C0H,99H,12H,0B0H,44H,0F1H,77H,33H,0EFH,66H,88H,0D0H
    
END