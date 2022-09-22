/*  
-*- coding: utf-8
-*- @Time : 2021/9/21 14:25
-*- @Author : lingz
-*- @Software: CLion
*/

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <time.h>

#define PORT 1115    
#define BACKLOG 1
#define Max 10
#define MAXSIZE 1024

struct user
{
   char name[20];
   int socket_fd;
};

struct message
{
    char from_name[20];
    char to_name[20];
    char mess[1024];
    char time_now[25];
    int flag;
};
struct message mess_buf;
struct user user_array[Max];
int user_num=0;
char inf[1024];
char private[1024]="Private:";
char sign[2]=":";
char sendbuf[1024];

int SendToClient(int fd,int Size);

// Process timestamps.
char *get_time(){
    time_t time_now;
    time(&time_now);
    return(ctime(&time_now));
}

void *pthread_service(void* sfd){
    int fd=*(int *)sfd;
    while(1){
	    char temp_name[30];
	    char temp_sign[2];
	    strcpy(temp_sign,sign);
        int numbytes;
        int i;
        numbytes=recv(fd,&mess_buf,sizeof(struct message),0);
        if(numbytes>0){
           if(mess_buf.flag==0){
               // If you do not send a message, the server displays all current users.
	            for(i=0;i<Max;i++){
                    if(fd==user_array[i].socket_fd){
                        strcpy(user_array[i].name,mess_buf.from_name);               
                    }
                }
	        }
            else if(mess_buf.flag==1){
                // Fill the message field.
                strcpy(temp_name,mess_buf.from_name);
	        	strcpy(temp_sign,sign);
	       	    strcpy(inf,mess_buf.mess);
	       	    strcat(temp_name,temp_sign);
	       	    strcat(sendbuf,temp_name);
	       	    strcat(sendbuf,inf);
		        strcpy(mess_buf.mess,sendbuf);
	        }
        }
	    else if(numbytes<=0){
            // Disconnect.
	        for(i=0;i<Max;i++){
                if(fd==user_array[i].socket_fd){
                    user_array[i].socket_fd=0;               
                }
            }
            printf("Numbytes=%d\n",numbytes);
            printf("Exit! Socket fd=%d\n",fd);
	        user_num--;
            break;
	    }
        printf("Receive message from %s,size=%d,flag:%d\n",mess_buf.from_name,numbytes,mess_buf.flag);
        if(mess_buf.flag==0 || mess_buf.flag==1){SendToClient(fd,numbytes);}
        bzero(mess_buf.mess,sizeof(mess_buf.mess));
        bzero(temp_name,sizeof(temp_name));
	    bzero(temp_sign,sizeof(temp_sign));
	    bzero(sendbuf,sizeof(sendbuf));
    }
    close(fd);
}


int SendToClient(int fd,int Size){
    // Data is received from the client, processed and forwarded to the user who needs to be delivered.
    int i;
    strncpy(mess_buf.time_now, get_time(), 25);
    if(strcmp(mess_buf.to_name,"0")==0){
        // If to_name ==0, all client can receive.
        for(i=0;i<user_num;i++){
            printf("user_array[%d].socket_fd=%d, name=%s\n", i, user_array[i].socket_fd, user_array[i].name);
            if((user_array[i].socket_fd!=0)&&(user_array[i].socket_fd!=fd)){
                send(user_array[i].socket_fd,&mess_buf,sizeof(struct message),0); 
                printf("%s send public message to %s\n", mess_buf.from_name, user_array[i].name);
            }
        }  
    }
    else{
        // Only some people can receive.
        for(i=0;i<user_num;i++){
	        if(strcmp(mess_buf.to_name,user_array[i].name)==0){
		        strcat(private,mess_buf.mess);
		        strcpy(mess_buf.mess,private);
		        send(user_array[i].socket_fd,&mess_buf,sizeof(struct message),0); 
		        strcpy(private,"Priavte:");
                printf("%s send  private message to %s\n", mess_buf.from_name, user_array[i].name);
	        }
	    }
    }
    return 0;
}

int  main(){
    system("clear");
    int listenfd, connectfd;
    struct sockaddr_in server;
    struct sockaddr_in client; 
    int sin_size;
    sin_size=sizeof(struct sockaddr_in);
    int fd;

    // Create socket, IPv4, TCP
    if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Creating socket failed.");
        exit(1);
    }

    int opt = SO_REUSEADDR;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bzero(&server,sizeof(server));

    // Set IPv4、PORT and address, INADDR_ANY means all addresses are accessible.
    server.sin_family=AF_INET;
    server.sin_port=htons(PORT);
    server.sin_addr.s_addr = htonl(INADDR_ANY);


    if (bind(listenfd, (struct sockaddr *)&server, sizeof(struct sockaddr)) == -1) { 
    	perror("Bind error.");
    	exit(1);
    }

    if(listen(listenfd,BACKLOG) == -1){  
    	perror("listen() error.\n"); 
    	exit(1); 
    }

    printf("Server start successfully, waiting for client....\n");

    while(1)
    {
        if ((fd = accept(listenfd,(struct sockaddr *)&client,&sin_size))==-1) {
        	perror("accept() error\n"); 
        	exit(1); 
        }

        if(user_num>=Max){
            printf("Warning:Reach the maximum of client number.\n");
            close(fd);
        }

        for(int i=0;i<Max;i++){
            if(user_array[i].socket_fd==0){
                user_array[i].socket_fd=fd;
                break;
            }
        }

        pthread_t tid;
        pthread_create(&tid,NULL,(void*)pthread_service,&fd);
        user_num=user_num+1;
    }
    close(listenfd);            
}
