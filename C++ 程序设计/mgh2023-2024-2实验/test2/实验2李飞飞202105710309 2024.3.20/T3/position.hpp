#pragma once
#include<cstring>
#include<iostream>
#include<cmath>
using namespace std;
class position{
public:
	position();
	~position();
	void set(double x=0,double y=0);
	void show()const;
	//判断第几象限
	string quadrant()const;
	double distance()const;
	double distance(const position &other)const;
	double slope()const;
	double slope(const position& other)const;
	void move(double distX,double distY=0);
private:
	double x;
	double y;
};
double distance(const position& other) {
	return other.distance();
}
double distance(const position& a, const position& b) {
	return a.distance(b);
}
position::position(){
	x = 0, y = 0;
}

position::~position(){}

inline void position::set(double x, double y){
	this->x = x; this->y = y;
}

inline void position::show()const {
	cout << "横坐标：" << x << " 纵坐标：" << y<<endl;
}

string position::quadrant()const {
	if (x == 0 && y == 0)return "原点";
	if (x == 0)return "y轴上";
	if (y == 0)return "x轴上";
	if (x > 0 && y > 0) return "一象限";
	if (x < 0 && y < 0) return "三象限";
	if (x > 0 && y < 0) return "四象限";
	if (x < 0 && y > 0) return "二象限";
}

inline double position::distance()const {
	return sqrt(x * x + y * y);
}

inline double position::distance(const position& other)const {
	return sqrt((x-other.x)* (x - other.x) + (y - other.y) * (y - other.y));
}

inline double position::slope()const {
	return y/x;
}

inline double position::slope(const position& other)const {
	return (other.y-y)/(other.x-x);
}

inline void position::move(double distX, double distY){
	x += distX; y += distY;
}
