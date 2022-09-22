		EXPORT  Add
        AREA	AddC,CODE,READONLY
        ENTRY
        CODE32
Add     ADD 	R0,R0,R1
        MOV		PC,LR
        END