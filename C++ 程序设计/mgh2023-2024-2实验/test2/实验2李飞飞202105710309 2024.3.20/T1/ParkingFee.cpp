//Time类的应用
#include "Time.hpp"
#include <iostream>
using namespace std;

double parkingFee(double); //根据停车时长收费，收费标准参考课程材料的停车场计费标准
int main(){
   Time arriveTime,leaveTime;
   double parkingTime,Fee;

   cout<<"Please set the arrivetime(24 hour format):";
   arriveTime.setTime(); 
  
   cout<<"Please set leavetime(24 hour format):";
   leaveTime.setTime();  

   parkingTime=leaveTime.diff(arriveTime);
   Fee=parkingFee(parkingTime);
   cout<<"The Parking time from ";
   arriveTime.showTime();
   cout<<" to ";
   leaveTime.showTime();
   cout<<endl;

   cout<<"The parking fee is:"<<Fee<<endl;
   return   0;
}
double parkingFee(double t) {
	return t * 4;
}
