#pragma once
#include "Student.hpp"
#include<iostream>
using namespace std;
Student::Student(string id, string name, int age, int grade, string password, int cnt, int* data) {
	studentId = id;
	courseName = name;
	this->age = age;
	studentGreade = grade;
	this->password = password;
	next = nullptr;
	selectedCourseCnt = cnt;
	mySelectedCourses = new int[50];//限选50门
	for (int i = 0; i < cnt; i++)
		mySelectedCourses[i] = data[i];
}
Student::Student(const Student& stu1) {
	studentId = stu1.studentId;
	courseName = stu1.courseName;
	age = stu1.age;
	studentGreade = stu1.studentGreade;
	password = stu1.password;
	next = nullptr;
	selectedCourseCnt = stu1.selectedCourseCnt;
	mySelectedCourses = new int[50];
	int i;
	for (i = 0; i < selectedCourseCnt; i++) {
		mySelectedCourses[i] = stu1.mySelectedCourses[i];
	}
}
Student::Student(string id, string name, int age, int grade, string password) {
	studentId = id;
	courseName = name;
	this->age = age;
	studentGreade = grade;
	this->password = password;
	next = nullptr;
	selectedCourseCnt = 0;
	mySelectedCourses = new int[50];//限选50门
}
Student::~Student() {
	delete[]mySelectedCourses;
}

void Student::add(int temp) {
	mySelectedCourses[selectedCourseCnt] = temp;
	selectedCourseCnt++;
}
void Student::showStudentInfo() {
	cout << "学号" << studentId << endl;
	cout << "名字" << courseName << endl;
	cout << "年龄" << age << endl;
	cout << "入学年份" << studentGreade << endl;
}

int Student::getSelectedCoursePlaceById(int id) {
	for (int i = 0; i < selectedCourseCnt; i++)
		if (mySelectedCourses[i] == id)
			return i;
	return -1;
}
void Student::cancelCourseByPlaceId(int id) {
	for (int i = id; i < selectedCourseCnt - 1; i++)
		mySelectedCourses[i] = mySelectedCourses[i + 1];
	selectedCourseCnt--;
}

int Student::chooseCourse(int id,bool choiceable) {
	int option = 2;
	//没选过
	if (getSelectedCoursePlaceById(id) == -1) {
		cout << "是否要添加这门课,是请输入1，否请输入-1" << endl;
		while (option != 1 && option != -1) {
			cin >> option;
			if (option == 1 || option == -1) {
				if (option == 1) {
					if(choiceable)
						add(id);
					else {
						cout << "无法选择，已达最大可选人数！\n";
						return 0;
					}
				}
				cout << "操作成功" << endl;
				return 1;
			}
		}
	}
	//选过了
	else {
		cout << "您已选了这门课，是否要取消，是请输入1，否请输入-1" << endl;
		while (option != 1 && option != -1) {
			cin >> option;
			if (option == 1 || option == -1) {
				if (option == 1){
					cancelCourseByPlaceId(getSelectedCoursePlaceById(id));
				}
				cout << "操作成功" << endl;
				return -1;
			}
		}
	}

}

void Student::setPassword() {
	cout << "请输入新的密码:";
	cin >> password;
	cout << "成功" << endl;
}
void Student::setData(string id, string courseName, int age, int studentGreade, string password) {
	studentId = id;
	this->courseName = courseName;
	this->age = age;
	this->studentGreade = studentGreade;
	this->password = password;
}

string Student::getid() { return studentId; }

string Student::getCourseName() { return courseName; }

int Student::getAge() { return age; }

int Student::getGreade() { return studentGreade; }

string Student::getPassword() { return password; }

int Student::getSelectedCourseCnt() { return selectedCourseCnt; }