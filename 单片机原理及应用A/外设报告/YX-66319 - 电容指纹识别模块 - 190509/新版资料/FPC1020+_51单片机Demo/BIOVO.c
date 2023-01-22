/*******************************************************************************
**文件名：BIOVO.c 
**说明：指纹芯片数据处理
**编程人员：李可洽
**编写时间：2013.04.07
*******************************************************************************/
#include "config.h"
#define UART_BUF_LEN  8
#define BUF_N 	8

unsigned char rBuf[UART_BUF_LEN];          //接收返回信息
unsigned char tBuf[UART_BUF_LEN];          //发送命令或者数据
unsigned char g_ucUartRxEnd ;              //接收返回信息结束标志
unsigned char g_ucUartRxLen ;             //接收返回信息长度
unsigned char l_ucFPID;                  //用户编号

extern unsigned char p;  //  串口接收地址


/*******************************************************************      
**功能：延时程序
**参数：
**返回：无                                                                
*******************************************************************/      
                                                                           
void Delay_ms(unsigned int ms)                                                         
{                                                                          
		unsigned int i=0,j=0;
		for(i=0;i<ms;i++)
		for(j=0;j<123;j++);
}
/*******************************************************************************
**功能：等待数据包发送完成
**参数：
**返回：无
*******************************************************************************/
unsigned char WaitFpData(void)
{
  unsigned char i;
  
  for(i=200; i>0; i--)//等待指纹芯片返回
  {
    Delay_ms(40);
    if(g_ucUartRxEnd)break;
  }  
  if(i==0)return FALSE;//指纹芯片没有返回
  else return TRUE;
}

/*******************************************************************************
**功能: 计算校验值
**参数: 要发送的数据指针地址
**返回: 校验值
*******************************************************************************/
unsigned char CmdGenCHK(unsigned char wLen,unsigned char *ptr)
{
	unsigned char i,temp = 0;
	
	for(i = 0; i < wLen; i++)
	{
		temp ^= *(ptr + i);
	}
	return temp;
}

/*******************************************************************************
**功能: 发送控制指纹芯片指令
**参数: wLen 数据长度
        cpPara 发送的数据
**返回：void
*******************************************************************************/
void UART_SendPackage(unsigned char wLen,unsigned char *ptr)
{
  unsigned int i=0,len=0;
 
  tBuf[0] = DATA_START;//指令包
  for(i = 0; i < wLen; i++)      // data in packet 
  {
    tBuf[1+i] = *(ptr+i);
  } 
  
  tBuf[wLen + 1] = CmdGenCHK(wLen, ptr);         //Generate checkout data
  tBuf[wLen + 2] = DATA_END;
  len = wLen + 3;
	
  g_ucUartRxEnd = 0;
  g_ucUartRxLen = len ;
	
	UartSend(tBuf,len);
}
 
/*******************************************************************************
**功能：返回信息处理
**参数： cmd 不同命令不同处理
**返回：处理结果
*******************************************************************************/
unsigned char Check_Package(unsigned char cmd)
{
    unsigned char flag = FALSE;
	   if(!WaitFpData()) return flag; //等待接收返回信息
     p = 0 ;
    if(g_ucUartRxEnd)
      g_ucUartRxEnd = 0;//清数据包接收标志
    else 
      return flag;
    
  if(rBuf[0] != DATA_START)return flag;
	if(rBuf[1] != cmd)return flag;
	if(rBuf[6] != CmdGenCHK(g_ucUartRxLen - 3, &rBuf[1]))return flag;
	switch(cmd)
	{
	case CMD_ENROLL1:
	case CMD_ENROLL2:
	case CMD_ENROLL3:
		if(ACK_SUCCESS == rBuf[4])flag = TRUE;
		else if(ACK_USER_EXIST == rBuf[4])
		{
//		Spk_HaveUser();
			Delay_ms(1500);
		}
		break;
	case CMD_DELETE:  //删除指定编号指纹
	case CMD_CLEAR:    //清空所有指纹
	case CMD_IDENTIFY:  //1:1比对
		if(ACK_SUCCESS == rBuf[4])flag = TRUE;
		break;
	case CMD_USERNUMB:  //取用户总数
		if(ACK_SUCCESS == rBuf[4])
		{
			flag = TRUE;
			l_ucFPID = rBuf[3];
		}
		break;
	case CMD_SEARCH:   //1:N比对
		if((1 == rBuf[4])||(2 == rBuf[4])||(3 == rBuf[4]))
		{
			flag = TRUE;
			l_ucFPID = rBuf[3];
		}
		break;
	default:
		break;
	}
     
    return flag;
}

/*******************************************************************************
**功能：以CharBuffer1 或CharBuffer2 中的特征文件搜索整个或部分指纹库
**参数：
**返回：无
*******************************************************************************/
void FP_Search(void)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_SEARCH;          //1:N比对
  *(buf+1) = 0x00;
  *(buf+2) = 0x00;
  *(buf+3) = 0x00;
  *(buf+4) = 0x00;

  UART_SendPackage(5, buf);
}

/*******************************************************************************
**功能：清空 flash 指纹库
**参数：
**返回：无
*******************************************************************************/
void FP_Clear(void)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_CLEAR;
  *(buf+1) = 0x00;
  *(buf+2) = 0x00;
  *(buf+3) = 0x00;
  *(buf+4) = 0x00;

  UART_SendPackage(5, buf);
}

/*******************************************************************************
**功能：删除指定编号指纹
**参数：u_id
**返回：void
*******************************************************************************/
void FP_Delete(unsigned int u_id)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_DELETE;
  *(buf+1) = u_id>>8;
  *(buf+2) = u_id&0xff;
  *(buf+3) = 0x00;
  *(buf+4) = 0x00;
  UART_SendPackage(5, buf);
}

/*******************************************************************************
**功能：1:1比对
**参数：u_id
**返回：void
*******************************************************************************/
void FP_Identify(unsigned int u_id)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_IDENTIFY;
  *(buf+1) = u_id>>8;
  *(buf+2) = u_id&0xff;
  *(buf+3) = 0x00;
  *(buf+4) = 0x00;
  UART_SendPackage(5, buf);
}

/*******************************************************************************
**注册指纹
**输入两次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
void Enroll_Step1(unsigned int u_id)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_ENROLL1;
  *(buf+1) = u_id>>8;
  *(buf+2) = u_id&0xff;
  *(buf+3) = 1;
  *(buf+4) = 0x00;

  UART_SendPackage(5, buf);
}

/*******************************************************************************
**注册指纹
**输入两次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
void Enroll_Step2(unsigned int u_id)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_ENROLL2;
  *(buf+1) = u_id>>8;
  *(buf+2) = u_id&0xff;
  *(buf+3) = 1;
  *(buf+4) = 0x00;

  UART_SendPackage(5, buf);
}

/*******************************************************************************
**注册指纹
**输入三次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
void Enroll_Step3(unsigned int u_id)
{
  unsigned char buf[BUF_N];
  
  *buf = CMD_ENROLL3;
  *(buf+1) = u_id>>8;
  *(buf+2) = u_id&0xff;
  *(buf+3) = 1;
  *(buf+4) = 0x00;

  UART_SendPackage(5, buf);
}

/*******************************************************************************
**注册指纹
**输入三次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Enroll(unsigned int u_id)
{
	Enroll_Step1(u_id);
	if(FALSE == Check_Package(CMD_ENROLL1))return FALSE;
	Delay_ms(100);
	
	Enroll_Step2(u_id);
	if(FALSE == Check_Package(CMD_ENROLL2))return FALSE;
	Delay_ms(100);
	
	Enroll_Step3(u_id);
	return Check_Package(CMD_ENROLL3);
}

/*******************************************************************************
**清空指纹
**
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Clear(void)
{
	FP_Clear();
//	if(FALSE == WaitFpData())return FALSE;
	return Check_Package(CMD_CLEAR);
}

/*******************************************************************************
**删除指定指纹
**
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Delete(unsigned int u_id)
{
	FP_Delete(u_id);
//	if(FALSE == WaitFpData())return FALSE;
	return Check_Package(CMD_DELETE);
}
/*******************************************************************************
**读取用户总数
**
**参数
*******************************************************************************/
unsigned char Finger_Read(void)
{
	 unsigned char buf[BUF_N];
  
  *buf = CMD_USERNUMB;
  *(buf+1) = 0x00;
  *(buf+2) = 0x00;
  *(buf+3) = 0x00;
  *(buf+4) = 0x00;
  UART_SendPackage(5, buf);
	return Check_Package(CMD_USERNUMB);
}
/*******************************************************************************
**读取用户总数
**
**参数
*******************************************************************************/
unsigned char Finger_Search(void)
{
	 FP_Search();
	return Check_Package(CMD_SEARCH);
}








