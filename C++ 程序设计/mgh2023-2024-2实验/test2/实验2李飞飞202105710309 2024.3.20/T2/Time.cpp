//Time类实现
#include "Time.hpp"
#include <iostream>
using namespace std;

void Time::showTime(){
    cout<<hour<<":"<<minute<<":"<<second;
}

void Time::setTime(){
    do{  
      cin>>hour>>minute>>second;
    }
    while(hour<0||hour>24|| minute<0||minute>59|| second<0||second>59);
}

double Time::diff(const Time& T){//注意常引用对形参T的约束
    //免费一小时
    long d=normalize()-T.normalize()-60;
    if(d<0) return 0;
    int h=d/60,m=d%60;
    if(m<15) return h;
    if(m>=15&&m<30) return h+0.5;
    if(m>=30&&m<60) return h+1;
}

long Time::normalize()const{
  return hour*60+minute;
}
