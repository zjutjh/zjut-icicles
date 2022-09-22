COUNT	EQU		0X4003100				; Define a name 
		AREA	Example2,CODE,READONLY  ; Pseudo-instruction,code snippet name
		ENTRY							; Program entry point
		CODE32							; 32-bit ARM instructions
START	LDR		R1,=COUNT				; Load 32-bit immediate data COUNT
		MOV		R0,#0					; 0->R0
		STR		R0,[R1]					; Store the data of the R0 register to the storage unit pointed to by R1 (offset is 0) 
		
LOOP	LDR		R1,=COUNT				; Repeat the initialization operation 
		LDR		R0,[R1]					; Load R1 to R0 
		ADD		R0,R0,#1				; R0=R0+1
		CMP		R0,#10					; R0 is compared with 10, and set the relevant flag bit  
		MOVHS	R0,#0					; If >=, then 0->R0
		STR		R0,[R1]					; The repeat operation
		
		B		LOOP					; Jump to the LOOP label
		
		END								; End of the source program