X			EQU			4						; Define a name X=4
n			EQU			5						; Define a name n=5

			AREA		Example4,CODE,READONLY	; Pseudo-instruction,code snippet name
			ENTRY								; Program entry point
			CODE32								; 32-bit ARM instructions
			
START		LDR			SP,=0x4003F00			; Load 0x4003F00 to SP
			LDR			R0,=X					; Load X to R0
			LDR			R1,=n					; Load n to R1
			BL			POW						; Jump to POW, and copy the address of the next instruction to the R14
		
HATL		B			HATL					; Jump in place 

POW		
			STMFD		SP!,{R1-R12,LR}			; STM:Bulk loading   FD:Full decrement 
			MOVS		R2,R1					; S:Affect CPSR 
		
			MOVEQ		R0,#1					; If Z=1,then 1->R0
			BEQ			POW_END					; If Z=1,jump to POW_END
			CMP			R2,#1					; Compare R2 with 1
			BEQ			POW_END					; If R2=1,jump to POW_END
			
			MOV			R1,R0					; (else) R0->R1
			SUB			R2,R2,#1				; R2=R2-1
POW_L1		BL			D0_MUL					; Jump to D0_MUL, and copy the address
			SUBS		R2,R2,#1				; R2=R2-1, affect CPSR
			BNE			POW_L1					; If Z=0, jump to POW_L1
		
POW_END		LDMFD		SP!,{R1-R12,PC}    		; LDM:Bulk storage 
		
D0_MUL		MUL			R0,R1,R0				; R0=R1*R0
			MOV			PC,LR					; LR->PC
			END									; End of the source program