#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
int main(int argc, char **argv)
{
    int fd,ret;
	unsigned char values[0];
    fd = open("/dev/SY", O_RDWR);
	if (fd < 0)
	{
		printf("can't open /dev/SY\n");
		return -1;
	}
    ret= read(fd,values,sizeof(unsigned char));
    if (ret >= 0){
	      printf("reading data is OK \n");
	}
	else{
		  printf("read data is fialed \n",ret);
	}
	printf("SY address = 0x%x\n",values[0]);
	return 0;
}
