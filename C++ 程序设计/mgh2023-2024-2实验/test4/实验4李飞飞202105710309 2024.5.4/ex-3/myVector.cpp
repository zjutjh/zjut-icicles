//myVector.cpp
#include"myVector.hpp"
#include <iostream>
using namespace std;

myVector::myVector(unsigned n, int value) {
	for (int i = 0; i < n; i++)
		data[i] = value;
	size = n;
}

myVector::myVector(const myVector& obj) {
	for (int i = 0; i < obj.size; i++)
		data[i] = obj.data[i];
	size = obj.size;
}

//赋值重载
myVector& myVector::operator=(const myVector& right) {
	for (int i = 0; i < right.size; i++)
		data[i] = right.data[i];
	size = right.get_size();
	return (*this);
}

//下标运算
int& myVector::operator[](unsigned index) {
	if (index < 0 || index >= size) {
		cout << "下标越界\n";
		exit(1);
	}
	return data[index];
}
//调整容量
void myVector::set_size(unsigned newsize) {
	size = newsize;
}
//获取容量
int myVector::get_size()const {
	return size;
}

myVector myVector::operator-() {
	myVector mv(this->size, 0);
	for (int i = 0; i < size; i++)
		mv.data[i] = data[size - i - 1];
	return mv;
}

void myVector::sort(){
for (int i = 0; i < size; i++) {
	for (int j = 0; j < size - i - 1; j++) {
		if (data[j] > data[j + 1]) {
			int temp = data[j];
			data[j] = data[j + 1];
			data[j + 1] = temp;
		}
	}
}
}

void myVector::display() const {
	for (int i = 0; i < size; i++) {
		if (i == 0) cout << data[i];
		else cout << ',' << data[i];
	}
	cout << endl;
}
myVector& myVector::operator++() {
	for (int i = 0; i < size; i++)
		data[i] += 1;
	return (*this);
}
myVector myVector::operator++(int) {
	myVector mv(*this);
	for (int i = 0; i < size; i++)
		data[i] += 1;
	return mv;
}

myVector  operator+(const myVector& left, const  myVector& right) {
	myVector mv;
	int sum = 0;
	int flag;
	for (int i = 0; i < left.get_size(); i++) {
		flag = 1;
		for (int j = 0; j < sum; j++)
			if (left.data[i] == mv.data[j])
				flag = 0;
		if (flag) {
			mv.data[sum] = left.data[i];
			sum++;
		}
	}
	for (int i = 0; i < right.get_size(); i++) {
		flag = 1;
		for (int j = 0; j < sum; j++)
			if (right.data[i] == mv.data[j])
				flag = 0;
		if (flag) {
			mv.data[sum] = right.data[i];
			sum++;
		}
	}
	mv.set_size(sum);
	return mv;
}

myVector  operator-(const myVector& left, const myVector& right) {//表示求left和right的差集
	myVector mv;
	int sum = 0, flag;
	for (int i = 0; i < left.get_size(); i++) {
		flag = 1;
		for (int j = 0; j < right.get_size(); j++)
			if (left.data[i] == right.data[j])
				flag = 0;
		if (flag) {
			mv.data[sum] = left.data[i];
			sum++;
		}
	}
	mv.set_size(sum);
	return mv;
}

ostream& operator<<(ostream& out, const myVector& vec) {
	for (int i = 0; i < vec.size; i++)
		if (i == 0) out << vec.data[i];
		else out << ',' << vec.data[i];
	return out;
}
istream& operator>>(istream& in, myVector& vec) {
	for (int i = 0; i < 10; i++)
		in >> vec.data[i];
	return in;
}
