#ifndef _VALUE_H
#define _VALUE_H
#include "sys.h"
#include "usart3.h"	 
#include "MQ.h"
#include "malloc.h"
#include "tftlcd.h"
#include "ltdc.h"
#include "tftlcd.h"
#include "touch.h" 
#include "24cxx.h"
#include "pcf8574.h"
#include "PMS5003.h"
#include "dht11.h"
#include "sgp30.h"
#include "adc.h"
#include "common.h"

#define Dnum     400    //曲线显示缓存的最大数组

#define Dtype    9     //处理的数据类型 温度、湿度、MQ2(甲烷)、pm2.5、pm1.0、PM10、MQ7(co)、co2、TVOC

#define adrr    200   //报警值存储地址
typedef struct 
{
	 
	u8  update;                  //数据更新标志 
	u16 Revnum[Dtype];           //已接收到的数据数量
  u16 DataBuff[Dtype][Dnum];	 //数据缓存数组
	
	//曲线相关
	u8  qxdate;   //曲线需要更新
	u8  DtypeSel;// 数据类型选择
	float Yxs;   //Y轴系数 用于调整 Y轴坐标值  
	
	//报警	 
	s16 Avalue[Dtype];          // 报警值数据
	u16 Af;                     // 报警值 标志 0-8 位对应0-8的数据类型 
	s16 Atemp;                  //  报警暂存值  
	u16 LAn;                    //  按键持续处于按压状态 数据追加的最大值
	
	//
	u8 mode;// 设备工作在哪个模式上  0 wifi  1采集模式
	u8 wifiF;//是否建立连接
	u32 Wificnt;
	u8  Wifidat[100];
	u8 RevAdrr;// 0全部接收 1-9接收对应的id终端设备
}Data_dev;  
extern Data_dev DataInfo; //数据信息结构体

typedef struct
{
	_pms_dev *pms;
	dht22_dev *dht22;
	adc_dev *adc;
	sgp30_dev *sgp;
	u8 level;
}_data_dev;

extern _data_dev data; 



u8 data_Analysis(  u8 *buf);//数据分析
//曲线参数设置
void QxPra(u8 ch);
 //报警值参数设置
void BjPra(u8 ch);
// 报警值比较函数
void BjzBj(void);
#endif


