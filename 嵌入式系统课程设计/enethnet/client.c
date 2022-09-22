/*  
-*- coding: utf-8
-*- @Time : 2021/9/21 14:23
-*- @Author : lingz
-*- @Software: CLion
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <time.h>
#include <string.h>
#include <arpa/inet.h>

#define PORT 1115
#define MAXDATASIZE 100
#define REMOTE_IP "127.0.0.1"

struct message
{
    char from_name[20];
    char to_name[20];
    char mess[1024];
    char time_now[25];
    //connect:flag=0,trans mess:flag=1
    int flag;
};
struct message recv_buf;
struct message send_buf;
char enter_name[20];
char sign[3]="\n";
int fd;
pthread_t tid;
// ident=0 means receive mess, ident=1 means send mess.
int ident=0;
int count=0;

// Process timestamps.
char *client_get_time(){
    time_t time_now;
    time(&time_now);
    return(ctime(&time_now));
}

void pthread_recv(void* ptr)
{
    while(1)
    {
    	if ((recv(fd,&recv_buf,sizeof(struct message),0)) == -1){
            // Determine if you can connect to the service.
            printf("recv() error\n");
        	exit(1);
        }
	    if(ident==0 && recv_buf.flag!=0){ 
            printf("%s",recv_buf.time_now);
            printf("%s",recv_buf.mess);
            printf("-------------------------------------------------------\n");
	        bzero(&recv_buf,sizeof(struct message));
        }
    }
}

void multi_chat(){
    if(count==0){
        // Create a process.
        pthread_create(&tid,NULL,pthread_recv,NULL);count++;
    }
    system("clear");
    printf("\t Welcome to our chat room.\n");
    printf("\t (UserName:%s, enter \"exit\" to return)\n", send_buf.from_name);
    printf("-------------------------------------------------------\n");
    char str[]="Enter chat room.\n";
    strcpy(send_buf.mess,str);
    send_buf.flag=1;
    // If to_name is 0, then everyone can receive the message.
    strcpy(send_buf.to_name,"0");
    getchar();
    send(fd,&send_buf,sizeof(struct message),0);
    while(1){
        // Process line breaks.
        char s[3];
	    strcpy(s,sign);
        memset(send_buf.mess,0,sizeof(send_buf.mess));
        gets(send_buf.mess);
        strncpy(send_buf.time_now, client_get_time(), 25);
        printf("Message sent, %s\n", send_buf.time_now);
	    printf("\n");
	    strcat(send_buf.mess,s);
        if(strncmp(send_buf.mess,"exit",4)==0){
            char str[]="Exit chat room, bye~\n";
            strcpy(send_buf.mess,str);
            send_buf.flag=1;
            strcpy(send_buf.to_name,"0");
            send(fd,&send_buf,sizeof(struct message),0);
            printf("You have exited the chat room, bye~\n");
            sleep(1);
            ident=1;
            break;
        }
        // Consolidate messages and send them.
        send(fd,&send_buf,sizeof(struct message),0);
    }
}

void private_chat(){ 
    system("clear");
    printf("\t Welcome to private chat.\n");
    printf("\t (UserName:%s, enter \"exit\" to return)\n", send_buf.from_name);
    printf("-------------------------------------------------------\n");
    printf("\n Private chat people number:");
    int pri_num = 0;
    scanf("%d", &pri_num);
    // Create a dynamic two-dimensional array.
    char (*enter_name_array)[20] = (char(*)[20])malloc(sizeof(char) * pri_num*20);
    printf("\n Receiver name:");
    for(int i=0;i<pri_num;i++) scanf("%s", enter_name_array[i]);
    // Process extra space.
    getchar();
    printf("-------------------------------------------------------\n");
    while(1){
        char s[3];
        strcpy(s,sign);
        memset(send_buf.mess,0,sizeof(send_buf.mess));
        printf(" Enter message: ");
        gets(send_buf.mess);
        if(strncmp(send_buf.mess,"exit",4)==0){
            printf("\n You have exited private chat, bye~\n");
            sleep(1);
            ident=1;
            break;
        }
        // Loop send private messages.
        for(int i=0;i<pri_num;i++){
            strncpy(send_buf.time_now, client_get_time(), 25);
            strcpy(send_buf.to_name, enter_name_array[i]);
            strcat(send_buf.mess,s);
            strcpy(send_buf.from_name,enter_name);
            send_buf.flag=1;        
            send(fd,&send_buf,sizeof(struct message),0);
        }    
        printf(" Send private message successfully!\n\n");
        sleep(1);
    }
    // Release the dynamic array.
    free(enter_name_array);
}

int main(){
    system("clear");
    int  numbytes;
    char buf[MAXDATASIZE];
    struct hostent *he;
    struct sockaddr_in server;
    if ((fd=socket(AF_INET, SOCK_STREAM, 0))==-1){
    	printf("Socket() error------> TCP socket creation failed!\n");
    	exit(1);
    }

    bzero(&server,sizeof(server));

    printf("\t Linux chat room system.\n");
    printf( "PLease enter UserName：");
    scanf("%s",enter_name);
    getchar();

    // Set IPv4、PORT and address, if address isn't set, will default to 127.0.0.1
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr.s_addr = inet_addr(REMOTE_IP);
    bzero(&send_buf,sizeof(struct message));
    strcpy(send_buf.from_name,enter_name);
    if(connect(fd, (struct sockaddr *)&server,sizeof(struct sockaddr))==-1){  
    	printf("Connect() error------> Cannot connet the server!\n"); 
    	exit(1); 
    }

    // No message is sent at initialization, so flag=0.
    send_buf.flag=0; 
    strcpy(send_buf.to_name,"0");
    send(fd,&send_buf,sizeof(struct message),0);
    printf("Connect success!\n");
    sleep(1);
    
    char command;
    while (1){  
	system("clear");
	printf("\n----------------------Command Bar----------------------\n");
   	printf("1.Chat room\n");
    printf("2.Send private message\n");
    printf("3.Exit\n");
    printf("-------------------------------------------------------\n");
    printf("Tips:Private message receiver should be in the chat room first.\n");
	printf("Enter your command:");
    scanf("%c",&command);
    switch(command){   
        case '1':ident=0;multi_chat();break;
        case '2':private_chat();break;
        case '3':exit(1);break;
        default :break;
    }
    }
    return(0);
    close(fd);  
 }
