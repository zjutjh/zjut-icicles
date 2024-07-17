//myString.hpp
#pragma once
#include <iostream>
#include <cstring>
using namespace std;
class myString {
public:
	//根据测试程序写构造函数原型
	myString();//默认构造函数

	myString(const myString& a); //拷贝构造函数

	myString(const char* str); //构造函数

	myString(int length, const char c); //构造函数

	myString(const char* s, int index, int length); //构造函数

	~myString(); //析构函数

	void display() const;//显示字符串

	void input(); // 输入字符串

	int len() const;//求字符串长
	//补充下标重载运算
	char& operator[](int index);

	//运算符重载
	myString& operator=(const myString& a); //赋值重载

	bool operator!=(const myString& a);//字符串不等于比较

	bool operator==(const myString& a);//字符串等于比较

	bool operator>(const myString& a); //字符串大于比较

	myString operator+(const myString& a); //字符串拼接

private:
	char* str;
	int size;
};

//考虑为myString添加输入输出流重载
//重载<<运算符
ostream& operator<<(ostream& out, const myString& a);
//重载>>运算符
istream& operator>>(istream& in, myString& a);
