		ORG		0000H
		LJMP	START
		ORG		1000H
			
START:
		MOV		SP,#60H
		MOV		TMOD,#20H		//定时器0，定时器工作模式2工作
		MOV		TH1,#0FAH		//模式2自动重装载的初值
		MOV		TH1,#0FAH
		SETB 	TR1		 		//开始计时
		MOV		SCON,#50H		//方式1工作,允许接收-->4.8kbit/s
		MOV		PCON,#00H		//SMOD=0
		MOV		R0,30H			//存放发送的数据块首地址
		MOV		R7,#20H			//存放发送的数据块长度
		MOV		R6,#00H			//校验和，长度字节与数字字节的累加和
		
TX_ACK:	
		MOV		A,#06H			//发送06H询问是否可以接收数据
		MOV		SBUF,A
		
WAIT1:	
		JNB		TI,WAIT1		//等待发送完一个字节
		CLR		TI				//清除发送完毕TI标志位

RX_YES:	
		JNB		RI,RX_YES		//等待乙机回答是否可以接收
		CLR		RI				//清除接收完毕RI标志位
		
NEXT1:	
		MOV		A,SBUF			//接收到乙机发送过来的ACK
		CJNE	A,#00H,TX_ACK  	//若为00H则表示可以接收数据，往下启动发送，否则再次询问
		
TX_LENGTH:
		MOV		A,R7			//先发送字节长度数R7
		MOV		SBUF,A
		
WAIT2:
		JNB		TI,WAIT2		//等待数据发送完毕
		CLR		TI				//清除发送完毕TI标志位
		MOV		R6,A			//增加校验和

TX_NEWS:						//查询发送数据
		MOV		A,@R0
		MOV		SBUF,A
		
TX_NEWS_WAIT:
		JNB		TI,TX_NEWS_WAIT
		CLR		TI				//清除发送完毕TI标志位
		MOV		A,R6
		ADD		A,@R0
		MOV		R6,A
		INC		@R0
		DJNZ	R7,TX_NEWS		//如果没发送完32个数据，继续发送
		
TX_CHECK:
		MOV		A,R6
		ADD		A,@R0
		MOV		R6,A
		MOV		A,R6			//发送校验位
		MOV		SBUF,A
		
WAIT3:							//等待发送完校验位
		JNB		TI,WAIT3
		CLR		TI
		
WAIT4:							//等待接收完校验位
		JNB		RI,WAIT4
		CLR		RI
		
IF_0FH:
		MOV		A,SBUF			//将接收到校验是否正确的恢复存到累加器
		CJNE	A,#0FH,START

HERE:	
		SJMP	HERE

		END
		
