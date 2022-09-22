#ifndef _PMS5003_H
#define _PMS5003_H
#include "sys.h"
#include "stdio.h"
#include "usart.h"

typedef struct 
{
	u16 length;      //帧长度
	u16 data1;       //PM1.0浓度(CF=1,标准颗粒物) 单位ug/m3
	u16 data2;       //PM2.5浓度(CF=1,标准颗粒物) 单位ug/m3
	u16 data3;       //PM10 浓度(CF=1,标准颗粒物) 单位ug/m3
	u16 data4;       //PM1.0浓度(大气环境下) 单位ug/m3
	u16 data5;       //PM2.5浓度(大气环境下) 单位ug/m3
	u16 data6;       //PM10 浓度(大气环境下) 单位ug/m3
	u16 data7;       //0.1升空气中直径在0.3um以上颗粒物个数
	u16 data8;       //0.1升空气中直径在0.5um以上颗粒物个数
	u16 data9;       //0.1升空气中直径在1.0um以上颗粒物个数
	u16 data10;      //0.1升空气中直径在2.5um以上颗粒物个数
	u16 data11;      //0.1升空气中直径在5.0um以上颗粒物个数
	u16 data12;      //0.1升空气中直径在10um以上颗粒物个数
	u8 id;           //版本号
	u8 error;        //错误代码
	u16 checksum;    //校验和
	u8 isUpdate;
}_pms_dev;



u8 Proces_Data(u8 buf[],_pms_dev* pms);


#endif
