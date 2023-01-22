/*******************************************************************************
**文件名：BIOVO.h 
**说明：指纹模块操作
**编程人员：
**编写时间：2014.6.26
*******************************************************************************/

#ifndef _BIOVO_H_
#define _BIOVO_H_

#define TRUE  0x01
#define FALSE  0x00

#define DATA_START			0xf5//数据包开始
#define DATA_END			0xf5//数据包结束

#define CMD_ENROLL1  		0x01//添加指纹步骤一
#define CMD_ENROLL2  		0x02//添加指纹步骤二
#define CMD_ENROLL3  		0x03//添加指纹步骤三
#define CMD_DELETE  		0x04//删除指定编号指纹
#define CMD_CLEAR  			0x05//清空所有指纹
#define CMD_USERNUMB  		0x09//取用户总数
#define CMD_IDENTIFY  		0x0b//1:1比对
#define CMD_SEARCH  		0x0c//1:N比对

#define ACK_SUCCESS  		0x00	//操作成功
#define ACK_FAIL	  		0x01	//操作失败
#define ACK_FULL	  		0x04	//指纹数据库已满
#define ACK_NOUSER   		0x05	//无此用户
#define ACK_USER_EXIST 		0x07 	//用户已存在
#define ACK_TIMEOUT  		0x08	//采集超时


void Delay_ms(unsigned int ms)  ; 
/*******************************************************************************
**注册指纹
**输入两次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Enroll(unsigned int u_id);

/*******************************************************************************
**验证指纹
**输入两次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Identify(void);

/*******************************************************************************
**清空指纹
**输入两次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Clear(void);

/*******************************************************************************
**删除指定指纹
**输入两次指纹注册一个指纹模板
**参数：UserID 指纹号
*******************************************************************************/
unsigned char Finger_Delete(unsigned int u_id);

unsigned char Finger_Read(void);

unsigned char Finger_Search(void);
#endif

