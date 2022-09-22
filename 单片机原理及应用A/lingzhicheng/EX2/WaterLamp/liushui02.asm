led     EQU     P0	   
        ORG     0000H
        LJMP    MAIN
        ORG     0050H
MAIN:
        MOV     SP,#60H	
LOOP:
		MOV     A,#01H
	    MOV	    R0,#08H			 
LEFTROTATE:
        MOV     led,A
		LCALL   DELAY1S	         
		RL      A                
		DJNZ    R0,LEFTROTATE

 	    MOV	    R0,#08H			
RIGHTROTATE:
		RR      A              
        MOV     led,A
		LCALL   DELAY1S         
		DJNZ    R0,RIGHTROTATE

		SJMP    LOOP


DELAY1S:  				 	     
	    MOV R7,#0A7H
DL1:
	    MOV R6,#0ABH
DL0:
	    MOV R5,#10H
	    DJNZ R5,$
	    DJNZ R6,DL0
	    DJNZ R7,DL1
	    NOP
	    RET
 
        END

