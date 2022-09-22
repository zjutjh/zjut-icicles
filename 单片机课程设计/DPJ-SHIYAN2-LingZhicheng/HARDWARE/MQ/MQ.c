#include "MQ.h"

//CH4浓度转换函数
u16 MQ2_get(u16 mq2)
{
	  u16 ppm;
    float RS,R0;   //RS为传感器在不同浓度中的电阻值，R0表示传感器在洁净空气中的电阻值
	  float ratio;    //ratio=RS/R0;
	  R0=(5000.0/300.0-1.0)*1000.0;
	  RS=(5000.0/(mq2*1.0)-1.0)*1000.0;
	  ratio=RS/R0;
	  if(ratio>1)
		{ppm=100;}
		if((ratio>0.5f)&&(ratio<=1.0f))
		{ppm=(-200)*ratio+300;}
		if((ratio>0.32f)&&(ratio<=0.5f))
		{ppm=(-556)*ratio+478;}
		if((ratio>0.25f)&&(ratio<=0.32f))
		{ppm=(-2860)*ratio+1215;}
		if((ratio>0.2f)&&(ratio<=0.25f))
		{ppm=(-4000)*ratio+1500;}
		if((ratio>0.17f)&&(ratio<=0.2f))
		{ppm=(-10000)*ratio+2700;}
		if((ratio>0.11f)&&(ratio<=0.17f))
		{ppm=(-16666)*ratio+3833;}
		if((ratio>0.087f)&&(ratio<=0.11f))
		{ppm=(-43500)*ratio+6785;}
		if((ratio>0.072f)&&(ratio<=0.087f))
		{ppm=(-66666)*ratio+8800;}
		if((ratio>0.065f)&&(ratio<=0.072f))
		{ppm=(-143000)*ratio+14300;}
		if((ratio>0.052f)&&(ratio<=0.065f))
		{ppm=(-154000)*ratio+15000;}
		if((ratio>0.043f)&&(ratio<=0.052f))
		{ppm=(-333333)*ratio+24333;}
		if(ratio<=0.043f)
		{ppm=10000;}
		return ppm;
}


//CO浓度转换函数
u16 MQ7_get(u16 mq7)
{
	  u16 ppm;
    float RS,R0;   //RS为传感器在不同浓度中的电阻值，R0表示传感器在洁净空气中的电阻值
	  float ratio;    //ratio=RS/R0;
	  R0=(5000.0/300.0-1.0)*1000.0;
	  RS=(5000.0/(mq7*1.0)-1.0)*1000.0;
	  ratio=RS/R0;
	  if(ratio>1)
		{ppm=0;}
	  if((ratio>0.25f)&&(ratio<=1.0f))
		{ppm=(-13.3)*ratio+13.3;}
		if((ratio>0.17f)&&(ratio<=0.25f))
		{ppm=(-124)*ratio+41;}
		if((ratio>0.14f)&&(ratio<=0.17f))
		{ppm=(-333)*ratio+77;}
		if((ratio>0.115f)&&(ratio<=0.14f))
		{ppm=(-400)*ratio+86;}
		if((ratio>0.095f)&&(ratio<=0.14f))
		{ppm=(-500)*ratio+98;}
		if((ratio>0.08f)&&(ratio<=0.095f))
		{ppm=(-1333)*ratio+177;}
		if((ratio>0.065f)&&(ratio<=0.08f))
		{ppm=(-2000)*ratio+230;}
		if((ratio>0.042f)&&(ratio<=0.065f))
		{ppm=(-4350)*ratio+382;}
		if((ratio>0.036f)&&(ratio<=0.042f))
		{ppm=(-16666)*ratio+900;}
		if((ratio>0.031f)&&(ratio<=0.036f))
		{ppm=(-20000)*ratio+1020;}
		if((ratio>0.028f)&&(ratio<=0.031f))
		{ppm=(-33333)*ratio+1430;}
		if((ratio>0.02f)&&(ratio<=0.028f))
		{ppm=(-62500)*ratio+2250;}
		if(ratio<=0.02f)
		{ppm=0;}
		return ppm;
}
