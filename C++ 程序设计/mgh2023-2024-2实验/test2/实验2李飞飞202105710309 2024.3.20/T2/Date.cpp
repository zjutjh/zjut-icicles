//Date类实现
#include "Date.hpp"
#include <iostream>
using namespace std;
int mp[13] = { 0,31,28,31,30,31,30,31,31,30,31,30,31 };
void Date::showDate(){
    cout<<year<<":"<< month <<":"<< day;
}

void Date::setDate(){
    do{  
      cin>> year >> month >> day;
    } while (year < 0 || year>2024 || month < 0 || month>12 || day<0 ||
        day>((mp[month] + (month == 2 && (year % 400 == 0 || (year % 4 == 0 && (year % 100))) ? 0 : 1))));
}

double Date::diff(const Date& T){
    long d = normalize() - T.normalize();
    if (d < 0) return -1; // 如果差值为负数，则返回-1
    return d; // 返回日期差值
}

long Date::normalize()const{
    long totalDays = day; // 将天数加到总天数中
    // 计算年份和月份对应的天数
    for (int i = 1; i < month; ++i) {
        totalDays += mp[i];
    }
    // 如果是闰年并且当前月份大于2月，则2月多加一天
    if (month > 2 && ((year % 400 == 0) || (year % 4 == 0 && year % 100 != 0))) {
        totalDays += 1;
    }
    // 计算年份对应的天数
    totalDays += (year - 1) * 365 + (year - 1) / 4 - (year - 1) / 100 + (year - 1) / 400;
    return totalDays;
}
