#include "value.h"
#include "stdlib.h"
#include "string.h"
Data_dev DataInfo; //数据信息结构体

_data_dev data;

//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}
//数据向前移动一位
void DataMoveForward(u16 dat[])
{
	u16 i=0;
	for(i=0;i<Dnum-1;i++)
	{
	  dat[i]=dat[i+1];
	}
	
}
//分析接收到的串口信息
// 
//buf:接收到的GPS数据缓冲区首地址
u8 data_Analysis(  u8 *buf)
{
	u8 *p1,dx,i,ID;			 
	u8 posx;     
	u32 temp;	   
	  
	u16 datN,Lt;
	p1=(u8*)strstr((const char *)buf,"data");     //" 查找到data字符串
	
 if(DataInfo.mode==0&&DataInfo.RevAdrr>0)//WiFi模式 且不是全接收模式
 { 
	 ID=*(p1+4)-0x30;
  if(DataInfo.RevAdrr!=ID)
		return 0; //接收数据域要接收的数据ID不一致退出
 }
	
	for(i=0;i<Dtype;i++)
	{
		posx=NMEA_Comma_Pos(p1,i+1);								//第i个数据
		if(posx!=0XFF)
		{
			
			if(i==2)//Mq2
			   temp=MQ2_get(NMEA_Str2num(p1+posx,&dx));
//			else if(i==6)//Mq7
//			   temp=MQ7_get(NMEA_Str2num(p1+posx,&dx));
			else 	 
			   temp=NMEA_Str2num(p1+posx,&dx); 
			
			
			Lt=i;//第1~9个数据
			if(DataInfo.Revnum[Lt]<Dnum)//判断接收的数据是否达到上限 未到达上限
			{
				datN=DataInfo.Revnum[Lt]; //将要接收的第几个字节数据 赋值给N
				DataInfo.DataBuff[Lt][datN]=temp;//赋值
				DataInfo.Revnum[Lt]++; //数据数量加一
				
			}
			else
			{
			 DataMoveForward(DataInfo.DataBuff[Lt]);	//数据向前移动一位，讲新的数据赋值给数组最后一位
			 datN=Dnum-1;		
			 DataInfo.DataBuff[Lt][datN]=temp;//赋值
			}	 
		}	  
	}  
	return 1;
}
//曲线参数设置
void QxPra(u8 ch)
{ 
	DataInfo.DtypeSel=ch;//当前选择的哪个数据类型
  switch(ch)
	{
	  case 0://  温度  
		   DataInfo.Yxs=0.5;//Y轴 系数 该值调整后Y轴坐标上限为100
		break;
		 case 1:// 湿度 
		   DataInfo.Yxs=0.5;//Y轴 系数 该值调整后Y轴坐标上限为100
		break;
		 case 2:// MQ2(甲烷)、 
		   DataInfo.Yxs=30;//Y轴 系数 该值调整后Y轴坐标上限为6000
		break;
		case 3:// pm2.5、 
		   DataInfo.Yxs=0.5;//Y轴 系数 该值调整后Y轴坐标上限为100
		break;
	  case 4:// 、pm1.0 
		   DataInfo.Yxs=0.5;//Y轴 系数 该值调整后Y轴坐标上限为100
		break;
	  case 5:// PM10 
		   DataInfo.Yxs=0.5;//Y轴 系数 该值调整后Y轴坐标上限为100
		break;
	  case 6:// MQ7(co) 
		   DataInfo.Yxs=0.5;//Y轴 系数 该值调整后Y轴坐标上限为100
		break;
		case 7:// co2 
		   DataInfo.Yxs=15;//Y轴 系数 该值调整后Y轴坐标上限为3000
		break;
	  case 8:// TVOC
		   DataInfo.Yxs=15;//Y轴 系数 该值调整后Y轴坐标上限为3000
		break; 
	}


}

//报警值参数设置
void BjPra(u8 ch)
{ 
	DataInfo.DtypeSel=ch;//当前选择的哪个数据类型
	DataInfo.Atemp=DataInfo.Avalue[DataInfo.DtypeSel];
  switch(ch)
	{
	  case 0://  温度  
		   DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
		 case 1:// 湿度 
		  DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
		 case 2:// MQ2(甲烷)
		  DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
		case 3:// pm2.5、 
		   DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
	  case 4:// 、pm1.0 
		    DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
	  case 5:// PM10 
		   DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
	  case 6:// MQ7(co) 
		    DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
		case 7:// co2 
		    DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break;
	  case 8:// TVOC
		    DataInfo.LAn=5;//  //  按键持续处于按压状态 数据追加的最大值
		break; 
	}


}

// 报警值比较函数
void BjzBj(void)
{u16 i=0;
	DataInfo.Af=0;
  for(i=0;i<Dtype;i++)
	{
	  if(DataInfo.DataBuff[i][DataInfo.Revnum[i]-1]>DataInfo.Avalue[i])//数组更新的最后一个值大于报警值
		{ 
		   DataInfo.Af|=1<<i;
		} 
	}
  if(DataInfo.Af>0)
		PCF8574_WriteBit(BEEP_IO,0);	//控制蜂鸣器 开
	else 
		PCF8574_WriteBit(BEEP_IO,1);	//控制蜂鸣器 关


}






