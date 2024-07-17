#include "DateTime.h"
#include<cmath>
#include<iostream>
using namespace std;
double parkingFee(double hour) {
	//获取天数
	int day = ((int)(floor(hour))) / (24);
	hour -= day * 24;
	double ans = 0;
	//计算分钟贡献
	ans += min(30.0, 4 * hour);
	//计算日期贡献
	ans += day * 30;
	if (day > 365)return ans * 0.8;
	if (day > 30)return ans * 0.9;
	return ans;
}
int main() {
	DateTime a, b;
	cout << "请输入开始时间：";
	a.setDateTime();
	cout << "请输入离开时间：";
	b.setDateTime();
	double mints = b.diff(a);
	cout << "起止时间：";
	a.showDateTime();
	cout<< "到";
	b.showDateTime();
	cout<< endl;
	cout << "费用：" << parkingFee(mints);
	return 0;
}