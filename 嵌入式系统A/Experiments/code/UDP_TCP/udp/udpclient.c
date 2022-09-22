nclude <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#define MAX_SIZE 1024
#define PORT 7050
#define HOST_ADDR "192.168.1.51"       //根据组队的服务器地址更改
int main(int argc,char **argv)
{
int sockfd,buflen;
char buf1[300],buf2[300];
struct sockaddr_in server,client;
socklen_t client_length=sizeof(client);
if(argc!=2)
{
  fprintf(stderr,"Usage:%s hostname\a\n",argv[0]);
  exit(1);
}
sockfd=socket(AF_INET,SOCK_DGRAM,0);
server.sin_family=AF_INET;
server.sin_port = htons(PORT);
server.sin_addr.s_addr = inet_addr(argv[1]);
for(;;)
{
printf ("Enter a line: ");
gets (buf1);//从stdin流中读取字符串，直至接受到换行符
buflen = strlen (buf1);
if (buflen == 0)
break;
sendto(sockfd, buf1, buflen + 1, 0,(struct sockaddr *)&server,sizeof(server));
if(recvfrom(sockfd, buf2, sizeof (buf2), 0,(struct sockaddr *)&client,&client_length)==-1)
{
	printf("recvfrom() error\n"); 
	exit(1); 
}
printf("Received line: %s\n", buf2);
}
close(sockfd);
return 0;
}

