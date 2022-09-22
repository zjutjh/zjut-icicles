#ifndef __SGP30_H
#define __SGP30_H

#include <myiic.h>
#include <sys.h>	

typedef struct
{
	//u16 co2,tvoc,h2,ethanol;
	u16 co2,tvoc;
	u8 isUpdate;
}sgp30_dev;



void SGP30_Init(void);
void SGP30_Start_Measure(void);
//void Read_Air_Quality(u8 *co2_high, u8 *co2_low, u8 *tvoc_high, u8 *tvoc_low);
void Read_Air_Quality(u16 *co2, u16 *tvoc);
//void Read_Air_Quality_2(u16 *h2, u16 *ethanol);

#endif
