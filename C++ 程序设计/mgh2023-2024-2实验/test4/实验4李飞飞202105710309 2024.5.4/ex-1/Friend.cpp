#include "Friend.hpp"
Clock::Clock(int h, int m, int s) :hour(h), minute(m), second(s) {}
void Clock::display(const Date& d) {
	//引用Date类对象中的私有数据
	cout << d.getMonth() << "/" << d.getDay() << "/" << d.getYear() << endl; 
	//引用本类对象中的私有数据
	cout << hour << ":" << minute << ":" << second << endl;
}
Date::Date(int m, int d, int y) :month(m),day(d),year(y){}
