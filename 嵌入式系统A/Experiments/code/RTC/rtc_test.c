#include <ctype.h>
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#if 1
typedef struct struct_tag_TimeInfor
{
        unsigned char year;
        unsigned char month;
        unsigned char day;
        unsigned char week;
        unsigned char hour;
        unsigned char min;
        unsigned char sec;
}TimeInfo;
#endif

typedef struct struct_tag_alarmInfor
{
        unsigned char enable;
        unsigned char pending;
		struct struct_tag_TimeInfor t_time;
}AlarmInfo;

int get_alarmtime(AlarmInfo time)
{ 
       
//           FILE *fp;
            int fd,retval;
            struct rtc_wkalrm rtc_tm;
//            fp = fopen("rtc.txt","a");
            fd = open ("/dev/rtc0", O_RDONLY);
             if (fd ==  -1) {
            perror("/dev/rtc0");
            exit(errno);
            }
          retval = ioctl(fd, RTC_ALM_READ, &rtc_tm);
          //time.enabled=rtc_tm.enabled
		  //time.pending=rtc_tm.pending
		  time.t_time.year=rtc_tm.time.tm_year;
          time.t_time.month=rtc_tm.time.tm_mon;
          time.t_time.day=rtc_tm.time.tm_mday;
          time.t_time.week=rtc_tm.time.tm_wday;
          time.t_time.hour=rtc_tm.time.tm_hour;
          time.t_time.min=rtc_tm.time.tm_min;
          time.t_time.sec=rtc_tm.time.tm_sec;
          fprintf(stdout, "Current ALARM date/time is \n %d-%d-%d, %02d:%02d:%02d.\n",
                            time.t_time.year + 1900, time.t_time.month + 1, time.t_time.day,time.t_time.hour, time.t_time.min, time.t_time.sec);
          close(fd);
          return 0;
}

int set_alarmtime(AlarmInfo time)
{
        int i,fd,retval;
        int b[6];
        char *rtime[6];
        char *message[]={"first is year\n"
                         "second is month\n"
                         "third is day\n"
                         "fourth is hour\n"
                         "fifth is minute\n"
                         "sixth is second\n"};
                        struct rtc_wkalrm alarm;
                        fd = open ("/dev/rtc0", O_RDWR);
                        if (fd ==  -1) 
                        {
                            perror("/dev/rtc0");
                            exit(errno);
                                                }
                        fprintf(stdout,*message);
                        for(i=0;i<6;i++)
                        {
                             rtime[i]=(char *)malloc(sizeof(char)*10);
                             scanf("%s\n \n",rtime[i]);
                                                }
                         for(i=0;i<6;i++)
                         b[i]=atoi(rtime[i]);        
                         time.t_time.year=b[0]-1900;            //year
                         time.t_time.month=b[1]-1;              //month
                         time.t_time.day=b[2];                  //day
                         time.t_time.hour=b[3];                 //hour
                         time.t_time.min=b[4];                  //minute
                         time.t_time.sec=b[5];               //second
                        
                  /*     if(time.t_time.year<2000||time.t_time.year>2099)
                        {
                           printf("Please input the correct year, the year should be in the scope of 2000~2099!\n");
                           exit(1);
                                              }    */

                         if(time.t_time.month<1||time.t_time.month>12)
                        {
                           printf("Please input the correct mouth, the mouth should be in the scope of 1~12!\n");
                           exit(1);
                                                }
                         if(time.t_time.day<1||time.t_time.day>31)
                        {
                           printf("Please input the correct days, the days should be in the scope of 1~31!\n");
                           exit(1);
                                               }
                         else if(time.t_time.month==4||time.t_time.month==6||time.t_time.month==9||time.t_time.month==11)
                                {
                                            if(time.t_time.day<1||time.t_time.day>30)
                                               {
                                                  printf("Please input the correct days, the days should be in the scope of 1~30!\n");
                                                  exit(1);
                                                }
                                 }
                         else if(time.t_time.month==2)
                                {
                                    if((time.t_time.year%4==0)&&(time.t_time.year%100!=0)||(time.t_time.year%400==0))
                                       {
                                           if(time.t_time.day<1||time.t_time.day>29)
                                              {
                                                  printf("Please input the correct days, the days should be in the scope of 1~29!\n");
                                                  exit(1);
                                               }
                                        }
                                    else
                                       {
                                           if(time.t_time.day<1||time.t_time.day>28)
                                              {
                                                  printf("Please input the correct days, the days should be in the scope of 1~28!\n");
                                                  exit(1);
                                              }
                                         }
                                        
                                  }
                         if(time.t_time.hour<0||time.t_time.hour>23)
                           {
                                   printf("Please input the correct hour, the hour should be in the scope of 0~23!\n");
                                   exit(1);
                           }
                         if(time.t_time.min<0||time.t_time.min>59)
                           {
                                   printf("Please input the correct minute, the minute should be in the scope of 0~60!\n");
                                   exit(1);
                            }
                         if(time.t_time.sec<0||time.t_time.sec>59)
                           {
                                  printf("Please input the correct second, the second should be in the scope of 0~60!\n");
                                  exit(1);
                          }
                            alarm.time.tm_year=time.t_time.year;
                            alarm.time.tm_mon=time.t_time.month;
                            alarm.time.tm_mday=time.t_time.day;
                            alarm.time.tm_hour=time.t_time.hour;
                            alarm.time.tm_min=time.t_time.min;
                            alarm.time.tm_sec=time.t_time.sec;
                            retval = ioctl(fd, RTC_ALM_SET, &alarm);
                            if (retval == -1) 
                             {   
                                 perror("ioctl");
                                 exit(errno);
                             }
                            close(fd);
                 //           system("/bin/busybox hwclock --hctosys");
                            return 0;
                        }

int get_curtime(TimeInfo time)
{ 
       
//           FILE *fp;
            int fd,retval;
            struct rtc_time rtc_tm;
//            fp = fopen("rtc.txt","a");
            fd = open ("/dev/rtc0", O_RDONLY);
             if (fd ==  -1) {
            perror("/dev/rtc0");
            exit(errno);
            }
          retval = ioctl(fd, RTC_RD_TIME, &rtc_tm);
          time.year=rtc_tm.tm_year;
          time.month=rtc_tm.tm_mon;
          time.day=rtc_tm.tm_mday;
          time.week=rtc_tm.tm_wday;
          time.hour=rtc_tm.tm_hour;
          time.min=rtc_tm.tm_min;
          time.sec=rtc_tm.tm_sec;
          fprintf(stdout, "Current RTC date/time is \n %d-%d-%d, %02d:%02d:%02d.\n",
                            time.year + 1900, time.month + 1, time.day,time.hour, time.min, time.sec);
          close(fd);
          return 0;
}


int set_curtime(TimeInfo time)
{
        int i,fd,retval;
        int b[6];
        char *rtime[6];
        char *message[]={"first is year\n"
                         "second is month\n"
                         "third is day\n"
                         "fourth is hour\n"
                         "fifth is minute\n"
                         "sixth is second\n"};
                        struct rtc_time rtc_tm;
                        fd = open ("/dev/rtc0", O_RDWR);
                        if (fd ==  -1) 
                        {
                            perror("/dev/rtc0");
                            exit(errno);
                                                }
                        fprintf(stdout,*message);
                        for(i=0;i<6;i++)
                        {
                             rtime[i]=(char *)malloc(sizeof(char)*10);
                             scanf("%s\n \n",rtime[i]);
                                                }
                         for(i=0;i<6;i++)
                                    b[i]=atoi(rtime[i]);        
                         time.year=b[0]-1900;            //year
                         time.month=b[1]-1;              //month
                         time.day=b[2];                  //day
                         time.hour=b[3];                 //hour
                         time.min=b[4];                  //minute
                         time.sec=b[5];               //second
                        
 /*                        if(time.year<2000||time.year>2099)
                        {
                           printf("Please input the correct year, the year should be in the scope of 2000~2099!\n");
                           exit(1);
                                              }
*/
                         if(time.month<1||time.month>12)
                        {
                           printf("Please input the correct mouth, the mouth should be in the scope of 1~12!\n");
                           exit(1);
                                                }
                         if(time.day<1||time.day>31)
                        {
                           printf("Please input the correct days, the days should be in the scope of 1~31!\n");
                           exit(1);
                                               }
                         else if(time.month==4||time.month==6||time.month==9||time.month==11)
                                {
                                            if(time.day<1||time.day>30)
                                               {
                                                  printf("Please input the correct days, the days should be in the scope of 1~30!\n");
                                                  exit(1);
                                                }
                                 }
                         else if(time.month==2)
                                {
                                    if((time.year%4==0)&&(time.year%100!=0)||(time.year%400==0))
                                       {
                                           if(time.day<1||time.day>29)
                                              {
                                                  printf("Please input the correct days, the days should be in the scope of 1~29!\n");
                                                  exit(1);
                                               }
                                        }
                                    else
                                       {
                                           if(time.day<1||time.day>28)
                                              {
                                                  printf("Please input the correct days, the days should be in the scope of 1~28!\n");
                                                  exit(1);
                                              }
                                         }
                                        
                                  }
                         if(time.hour<0||time.hour>23)
                           {
                                   printf("Please input the correct hour, the hour should be in the scope of 0~23!\n");
                                   exit(1);
                           }
                         if(time.min<0||time.min>59)
                           {
                                   printf("Please input the correct minute, the minute should be in the scope of 0~60!\n");
                                   exit(1);
                            }
                         if(time.sec<0||time.sec>59)
                           {
                                  printf("Please input the correct second, the second should be in the scope of 0~60!\n");
                                  exit(1);
                          }
                            rtc_tm.tm_year=time.year;
                            rtc_tm.tm_mon=time.month;
                            rtc_tm.tm_mday=time.day;
                            rtc_tm.tm_hour=time.hour;
                            rtc_tm.tm_min=time.min;
                            rtc_tm.tm_sec=time.sec;
                            retval = ioctl(fd, RTC_SET_TIME, &rtc_tm);
                            if (retval == -1) 
                             {   
                                 perror("ioctl");
                                 exit(errno);
                             }
                            close(fd);
                 //           system("/bin/busybox hwclock --hctosys");
                            return 0;
                        }
                        
int main() 
{
        TimeInfo p;
        AlarmInfo a;
        int choice;
        fprintf(stdout,"Now you can choose\n1 to select get_curtime\n2 to select set_curtime\n3 to get_alarmtime\n");
        
        //fprintf(stdout,"input any digital  without zero to get time\n");
       // fprintf(stdout,"Your will get time:\n");
        scanf("%d",&choice);
        switch(choice)
        {  
         case 1 : 
       //        set_curtime(p); 
               get_curtime(p);
               break;
         case 2 :
               set_curtime(p);
               get_curtime(p);
               break;
         case 3 :  
			   get_alarmtime(a);
			   break; 
/*		 case 4 :
			  set_alarmtime(a);
              get_alarmtime(a);       */
         }

       // if (choice)
       // get_curtime(p);

        return 0;
}
