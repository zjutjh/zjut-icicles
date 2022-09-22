		AREA Example,CODE,READONLY		; Pseudo-instruction，code snippet name 
		ENTRY							; Program entry point 
		CODE32							; 32-bit ARM instructions 
START	MOV		R0,#15					; START is a label,15->R0 
		MOV		R1,#8					; 8->R1 
		ADDS	R0,R0,R1				; R0=R0+R1
		
		B		START					; Jump to the START label 
		
		END								; End of the source program  