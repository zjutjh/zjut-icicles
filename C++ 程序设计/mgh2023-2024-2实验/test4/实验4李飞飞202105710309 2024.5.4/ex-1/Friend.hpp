#include <iostream>
using namespace std;
class Date;
class Clock{
public:
	Clock(int, int, int);
	void display(const Date&);
private:
	int hour;
	int minute;
	int second;
};
class Date{
public:
	Date(int, int, int);
	int getYear() const { return year; }
	int getMonth() const { return month; }
	int getDay() const { return day; }

private:
	int month;
	int day;
	int year;
};
