#include "sgp30.h"
#include "delay.h"
#include "usart.h"
//IO方向设置
#define sgp30SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define sgp30SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
//IO操作
#define sgp30IIC_SCL   PBout(8) //SCL
#define sgp30IIC_SDA   PBout(9) //SDA
#define sgp30READ_SDA  PBin(9)  //输入SDA

//IIC初始化
static void SGP30IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOH时钟
    
    //PB8,9初始化设置
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //快速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    sgp30IIC_SDA=1;
    sgp30IIC_SCL=1;  
}

//产生IIC起始信号
static void SGP30IIC_Start(void)
{
	sgp30SDA_OUT();     //sda线输出
	sgp30IIC_SDA=1;	  	  
	sgp30IIC_SCL=1;
	delay_us(4);
 	sgp30IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	sgp30IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
static void SGP30IIC_Stop(void)
{
	sgp30SDA_OUT();//sda线输出
	sgp30IIC_SCL=0;
	sgp30IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	sgp30IIC_SCL=1; 
	sgp30IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 SGP30IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	sgp30SDA_IN();      //SDA设置为输入  
	sgp30IIC_SDA=1;delay_us(1);	   
	sgp30IIC_SCL=1;delay_us(1);	 
	while(sgp30READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			SGP30IIC_Stop();
			return 1;
		}
	}
	sgp30IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
static void SGP30IIC_Ack(void)
{
	sgp30IIC_SCL=0;
	sgp30SDA_OUT();
	sgp30IIC_SDA=0;
	delay_us(2);
	sgp30IIC_SCL=1;
	delay_us(2);
	sgp30IIC_SCL=0;
}
//不产生ACK应答		    
static void SGP30IIC_NAck(void)
{
	sgp30IIC_SCL=0;
	sgp30SDA_OUT();
	sgp30IIC_SDA=1;
	delay_us(2);
	sgp30IIC_SCL=1;
	delay_us(2);
	sgp30IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void SGP30IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	sgp30SDA_OUT(); 	    
    sgp30IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        sgp30IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		sgp30IIC_SCL=1;
		delay_us(2); 
		sgp30IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static u8 SGP30IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	sgp30SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        sgp30IIC_SCL=0; 
        delay_us(2);
		    sgp30IIC_SCL=1;
        receive<<=1;
        if(sgp30READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        SGP30IIC_NAck();//发送nACK
    else
        SGP30IIC_Ack(); //发送ACK   
    return receive;
}


//初始化IIC总线，查找SGP30
void SGP30_Init(void)
{	
	 SGP30IIC_Init();		//初始化IIC总线
	
	//查看SGP30的序列号，确认已与总线相连
	SGP30IIC_Start();
	SGP30IIC_Send_Byte(0x58<<1);
	if(SGP30IIC_Wait_Ack()==1){}
	else{}
	SGP30IIC_Send_Byte(0x36);
	if(SGP30IIC_Wait_Ack()==1){}
	else{}
	SGP30IIC_Send_Byte(0x82);
	if(SGP30IIC_Wait_Ack()==1){}
	else{}
	SGP30IIC_Stop();
}

//初始化SGP30的空气质量检测功能
void SGP30_Start_Measure(void)
{	
	//发送Init_air_quality命令
	SGP30IIC_Start();
	SGP30IIC_Send_Byte(0x58<<1);
	if(SGP30IIC_Wait_Ack()==1){}
	else{}
	SGP30IIC_Send_Byte(0x20);
	if(SGP30IIC_Wait_Ack()==1){}
	else{}
	SGP30IIC_Send_Byte(0x03);
	if(SGP30IIC_Wait_Ack()==1){}
	else{}
	SGP30IIC_Stop();
	
	//delay_ms(15000);		//需要时间间隔进行初始化
	
	//发送Measure_air_quality命令
	SGP30IIC_Start();
	SGP30IIC_Send_Byte(0x58<<1);
	SGP30IIC_Wait_Ack();
	SGP30IIC_Send_Byte(0x20);
	SGP30IIC_Wait_Ack();
	SGP30IIC_Send_Byte(0x08);
	SGP30IIC_Wait_Ack();
	SGP30IIC_Stop();
}



void Read_Air_Quality(u16 *co2, u16 *tvoc)
{
	u8 co2_high = 0;
	u8 co2_low = 0;
	u8 tvoc_high = 0;
	u8 tvoc_low = 0;
	
	//发送Measure_air_quality命令
	SGP30IIC_Start();
	SGP30IIC_Send_Byte(0x58<<1);
	SGP30IIC_Wait_Ack();
	SGP30IIC_Send_Byte(0x20);
	SGP30IIC_Wait_Ack();
	SGP30IIC_Send_Byte(0x08);
  SGP30IIC_Wait_Ack();
	
	delay_ms(500);		//需要时间间隔进行检测
	
	//发送读地址并读取两个16位的浓度值及相应的8位CRC检查值
	SGP30IIC_Start();
	SGP30IIC_Send_Byte((0x58<<1)|0x01);     //read
	SGP30IIC_Wait_Ack();
	co2_high = SGP30IIC_Read_Byte(1);
	co2_low = SGP30IIC_Read_Byte(1);
	SGP30IIC_Read_Byte(1);
	tvoc_high = SGP30IIC_Read_Byte(1);
	tvoc_low = SGP30IIC_Read_Byte(1);
	SGP30IIC_Read_Byte(0);
	SGP30IIC_Stop();
	
	    *co2 = (co2_high<<8)|co2_low;
 	    *tvoc = (tvoc_high<<8)|tvoc_low;

}
/*
//读取氢气与乙醇气体浓度
void Read_Air_Quality_2(u16 *h2, u16 *ethanol)
{
	u8 h2_high = 0;
	u8 h2_low = 0;
	u8 ethanol_high = 0;
	u8 ethanol_low = 0;
	
	//发送Measure_air_quality命令
	IIC_Start();
	IIC_Send_Byte(0x58<<1);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x50);
	IIC_Wait_Ack();

	
	delay_ms(500);		//需要时间间隔进行检测
	
	//发送读地址并读取两个16位的浓度值及相应的8位CRC检查值
	IIC_Start();
	IIC_Send_Byte((0x58<<1)|0x01);     //read
	IIC_Wait_Ack();
	h2_high = IIC_Read_Byte(1);
	h2_low = IIC_Read_Byte(1);
  IIC_Read_Byte(1);
	ethanol_high = IIC_Read_Byte(1);
	ethanol_low = IIC_Read_Byte(1);
	IIC_Read_Byte(0);
	IIC_Stop();
	
	    *h2 = (h2_high<<8)|h2_low;
 	    *ethanol = (ethanol_high<<8)|ethanol_low;
			
}*/


