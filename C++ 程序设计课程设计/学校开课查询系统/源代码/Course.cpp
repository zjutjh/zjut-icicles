#include "Course.hpp"
#include<iostream>
using namespace std;
Course::Course(int id, string name, string teacherName, double credit, string type, string department, int cnt, int maxium){
	courseId = id;
	courseName = name;
	lecturerName = teacherName;
	this->credit = credit;
	courseType = type;
	this->department = department;
	next = nullptr;
	selectedCnt = cnt;
	maxCnt = maxium;
}
Course::~Course(){}

bool  Course::studentChooseable() {
	return maxCnt != selectedCnt;
}
void Course::updateSelectedCnt(int x) {
	selectedCnt += x;
}
void Course::showStudentInfo(){
	cout << courseId << ' ' << courseName << ' ' << lecturerName << ' ' << credit << ' '
		<< courseType << ' ' << department << ' ' << "已选" << selectedCnt << "人" << ' '
		<< "最多可选" << maxCnt << "人" << endl;
}
void Course::updateCourseInfo(string name, string teacher, double s, string type, string dep){
	courseName = name; lecturerName = teacher; credit = s;
	courseType = type; department = dep;
}
Course& Course::operator=(Course& now){
	courseId = now.courseId;
	courseName = now.courseName;
	lecturerName = now.lecturerName;
	credit = now.credit;
	courseType = now.courseType;
	department = now.department;
	selectedCnt = now.selectedCnt;
	maxCnt = now.maxCnt;
	return *this;
}

int Course::getNumber() { return courseId; }

string Course::getCourseName() { return courseName; }

string Course::getLecturerName() { return lecturerName; }

double Course::getCredit() { return credit; }

string Course::getDepartment() { return department; }

string Course::getcourseType() { return courseType; }

int Course::getSelectedCnt() { return selectedCnt; }

int Course::getMaxCnt() { return maxCnt; }



