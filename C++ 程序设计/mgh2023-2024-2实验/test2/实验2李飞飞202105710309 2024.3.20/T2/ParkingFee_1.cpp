//Time类的应用

#include "Date.hpp" //+补充Date类的应用

#include "Time.hpp"
#include <iostream>
#include<cmath>
using namespace std;

double parkingFee(double day,double mint); //根据停车时长收费,需调整为根据真实的停车时长（可能包含日期差，时间差)收费
int main(){  
	Date arriveDate,leaveDate;
	Time arriveTime, leaveTime;
	double parkingTime, Fee;
	double parkingDate;

	cout << "Please set the arriveDate:";
	arriveDate.setDate();
	cout << "Please set the arrivetime(24 hour format):";
	arriveTime.setTime();

	cout << "Please set the leaveDate:";
	leaveDate.setDate();
	cout << "Please set leavetime(24 hour format):";
	leaveTime.setTime();

	parkingTime = leaveTime.diff(arriveTime);
	//补充计算停车的日期差
	parkingDate = leaveDate.diff(arriveDate);

	Fee=parkingFee(parkingDate,parkingTime); //更新原有的收费计算

	cout << "The Parking time from ";
	arriveDate.showDate();
	cout << " ";
	arriveTime.showTime();
	cout << " to ";
	leaveDate.showDate();
	cout << " ";
	leaveTime.showTime();
	cout << endl;

	cout << "The parking fee is:" << Fee << endl;
	return   0;
}
double parkingFee(double day, double mint) {
	double ans = 0;
	//计算分钟贡献
	ans += min(30.0, 4 * mint);
	//计算日期贡献
	ans += day * 30;
	if (day > 365)return ans * 0.8;
	if (day > 30)return ans * 0.9;
	return ans;
}
