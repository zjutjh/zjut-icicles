#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#define DEF_GPIO_DIR_OUT  0x01
#define DEF_GPIO_DIR_IN   0x02
#define DEF_GPIO_SET_DATA 0x03
#define DEF_GPIO_CLR_DATA 0x04
#define DEF_GPIO_GET_DATA 0x05
void main(int argc,char * argv[])
{
	FILE *fd;
	fd=open("/dev/dm365_gpio_experiment",O_RDWR,0);
	int gpio;
	int cmd;
	gpio=atoi(argv[1]);
	if(argc==1)
	printf("please input arg\n");
	else if(argc==2)
		ioctl(fd,DEF_GPIO_GET_DATA,gpio);
	else
	{
		cmd=atoi(argv[2]);
		switch(cmd)
		{
			case 0:ioctl(fd,DEF_GPIO_CLR_DATA,gpio);break;
			case 1:ioctl(fd,DEF_GPIO_SET_DATA,gpio);break;
			case 2:ioctl(fd,DEF_GPIO_DIR_IN,gpio);break;
			case 3:ioctl(fd,DEF_GPIO_DIR_OUT,gpio);break;
			default: printf("wrong\n");
		}
	}
}

