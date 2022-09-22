nclude <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#define PORT 7050
int main(void)
{
int sockfd,pktlen;
char buf[300],buf1[300];
struct sockaddr_in server;
struct sockaddr_in client;
sockfd=socket(AF_INET,SOCK_DGRAM,0);
memset ((char *)&server, sizeof(server), 0); //将已开辟内存空间 server 的全部字节的值设为值0.类似于bzero
server.sin_family = AF_INET;
server.sin_port = htons(PORT);//端口号
server.sin_addr.s_addr = INADDR_ANY;//设置网络地址,INADDR_ANY表示机器的IP地址
bind(sockfd,(struct sockaddr *)&server,sizeof(struct sockaddr_in));
for (;;) {
/*recv接受client发送的数据,recv函数仅仅是copy数据，真正的接收数据是协议来完成的）， 第一个参数指定接收端套接字描述符；第二个参数指明一个缓冲区，该缓冲区用来存放recv函数接收到的数据；第三个参数指明buf的长度
recv函数返回其实际copy的字节数*/
socklen_t len=sizeof(struct sockaddr_in);
pktlen = recvfrom (sockfd, buf, sizeof (buf), 0,(struct sockaddr *)&client, &len);
if (pktlen == 0)
break;
printf ("Received line: %s\n", buf);
printf ("Enter a line: ");
fgets(buf1,300,stdin);
/*并不是send把ns的发送缓冲中的数据传到连接的另一端的，而是协议传的，send仅仅是把buf中的数据copy到ns的发送缓冲区的剩余空间里
返回实际copy的字节数*/
sendto (sockfd, buf1,sizeof(buf1), 0,(struct sockaddr*)&client,len);
}
close(sockfd);
}

