
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <string.h>

#include <netinet/in.h>

#include <errno.h>

#define PORT 7777

main ()

{

struct sockaddr_in client, server;// 客户端地址信息 本机地址信息
socklen_t namelen;
int s, ns,  pktlen;//s：监听socket ns：数据传输socket namelen:client的地址长度 pktlen:传送数据的字节数

char buf[400];

char buf3[200];

s=socket(AF_INET, SOCK_STREAM, 0); //创建连接的SOCKET,s为socket描述符

// 初始化服务器地址

memset ((char *)&server, sizeof(server), 0); //将已开辟内存空间 server 的全部字节的值设为值0.类似于bzero

server.sin_family = AF_INET;

server.sin_port = htons(PORT);//端口号

server.sin_addr.s_addr = INADDR_ANY;//设置网络地址,INADDR_ANY表示机器的IP地址

//server需要在listen之前绑定一个大家都知道的地址,就是刚刚初始化好的ip+端口号 
bind(s, (struct sockaddr *)&server, sizeof(server));

listen(s,1);//侦听客户端请求,i为socket可以排队链接的最大个数

/*接受client请求,s为server的描述符(即监听socket描述符),第二个参数即指针client的协议地址,第三个参数代表地址长度

返回值ns是一个全新的描述符,是数据传输socket,代表与返回客户的tcp连接*/

namelen = sizeof (client);

ns = accept (s, (struct sockaddr *)&client, &namelen);

//开始进行网络I/O

for (;;) {

/*recv接受client发送的数据,recv函数仅仅是copy数据，真正的接收数据是协议来完成的）， 第一个参数指定接收端套接字描述符；第二个参数指明一个缓冲区，该缓冲区用来存放recv函数接收到的数据；第三个参数指明buf的长度

recv函数返回其实际copy的字节数*/

pktlen = recv (ns, buf, sizeof (buf), 0);

if (pktlen == 0)

break;

printf ("Received line: %s\n", buf);

printf ("Enter a line: ");

gets(buf3);

/*并不是send把ns的发送缓冲中的数据传到连接的另一端的，而是协议传的，send仅仅是把buf中的数据copy到ns的发送缓冲区的剩余空间里

返回实际copy的字节数*/

send (ns, buf3,sizeof(buf3), 0);

}

close(ns);

close(s);

}
