		ORG		0000H
		LJMP	START
		ORG		0023H
		LJMP	IF_06H
		ORG		1000H
			
START:
		MOV		SP,#60H
		MOV		TMOD,#20H		//定时器0，定时器工作模式2工作
		MOV		TH1,#0FAH		//模式2自动重装载的初值
		MOV		TH1,#0FAH
		SETB 	TR1		 		//开始计时
		MOV		SCON,#50H		//方式1,允许接收-->4.8kbit/s
		MOV		PCON,#00H		//SMOD=0
		MOV		IE,#90H
		MOV		R0,30H			//存放发送的数据块首地址
		MOV		R7,#00H			//存放发送的数据块长度
		MOV		R6,#00H			//校验和，长度字节与数字字节的累加和
		
HERE:
		SJMP	HERE
		
IF_06H:
		PUSH	ACC
		PUSH	PSW
		CLR		RI				//清除接收完毕RI标志位
		MOV		A,SBUF			//核对握手信号是不是06H
		CJNE	A,#06H,TX_15H	//如果是06H则发送00H应答，否则发送15H拒绝

TX_00H:
		MOV		A,#00H			
		MOV		SBUF,A			//发送出00H表示可以接收数据
		LJMP	HERE_RE
		
TX_15H:
		MOV		A,#15H			//发送15H表示不可以接收
		MOV		SBUF,A
		LJMP	RETURN
			
HERE_RE:
		JNB		TI,HERE_RE		//等待发送完毕，发送完毕准备接收
		CLR		TI
		
HAVE1:
		JNB		RI,HAVE1		//等待接收数据长度
		CLR		RI
		MOV		A,SBUF
		MOV		R7,A			//R7存数据长度
		MOV		R6,A			//R6存校验和

HAVE2:
		JNB		RI,HAVE2		//等待接收正式数据		
		CLR		RI
		MOV		A,SBUF
		MOV		@R0,A
		MOV		A,R6
		ADD		A,@R0
		MOV		R6,A
		INC		@R0
		DJNZ	R7,HAVE2		//如果没接收完完20个数据，继续接收
		
RX_CHECK:						//接收校验和
		JNB		RI,RX_CHECK
		CLR		RI		
		MOV		A,SBUF
		MOV		39H,A
		CJNE	A,39H,TX_ERR	//如果校验正确，则继续向下ok，错误则发送#F0H
		
TX_OK:							//校验正确，发送0FH
		MOV		A,#0FH
		MOV		SBUF,A
		LJMP	HERE_END
			
TX_ERR:							//校验错误，发送F0H
		MOV		A,#0F0H
		MOV		SBUF,A			
		
HERE_END:						//等待发送完毕
		JNB TI,HERE_END	 	
		CLR TI

RETURN:
		POP		PSW
		POP		ACC
		RETI

		END
			