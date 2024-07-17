#include "StudentList.hpp"
#include<iostream>
#include<fstream>
StudentList::StudentList() {
	size = 0;
	head = nullptr;
}
StudentList::~StudentList() {}

void StudentList::addStudent(Student& student) {
	Student* now = new Student(student);
	if (head == nullptr) {
		head = now; size++; 
		return;
	}
	now->next = head->next;
	head->next = now;
	size++; 
	return;

}
Student* StudentList::login() {
	string sno, password;
	cout << "请输入学号:";cin >> sno;
	cout << "请输入密码:";cin >> password;
	Student* p = head;
	while (p != nullptr) {
		if (p->getid() == sno && p->getPassword() == password) {
			cout << "登陆成功" << endl;
			return p;
		}
		p = p->next;
	}
	cout << "登陆失败" << endl;
	return nullptr;
}

void StudentList::write() {
	ofstream outstu("Student.txt");
	Student* p = head;
	while (p != nullptr) {
		outstu << p->getid() << " " << p->getCourseName() << " " << p->getAge()
			<< " " << p->getGreade() << " " << p->getPassword() << " " << p->getSelectedCourseCnt() << " ";
		//录入选课记录
		for (int i = 0; i < p->getSelectedCourseCnt(); i++)
			outstu << p->mySelectedCourses[i] << " ";
		outstu << endl;
		p = p->next;
	}
	outstu.close();
	cout << "学生列表信息保存成功" << endl;
}

void StudentList::deletecourseId(int id) {
	int m;
	Student* p = head;
	while (p != nullptr) {
		if (p->getSelectedCoursePlaceById(id) != -1) {
			m = p->getSelectedCoursePlaceById(id);
			p->cancelCourseByPlaceId(m);
		}
		p = p->next;
	}
}
Student* StudentList::findStudentById(string id) {
	Student* p = head;
	while (p->getid() != id && p->next != nullptr)
		p = p->next;
	if (p!=nullptr&&p->getid() == id)return p;
	else  cout << "查无此人" << endl;
	return nullptr;
}
void StudentList::updateStudentInfo(string studentId) {
	string id, name, password;
	int age, grade;
	Student* p2 = findStudentById(studentId);
	if (p2 == nullptr)return;
	cout << "准备要修改该项";
	p2->showStudentInfo();
	cout << "请输入学号，姓名，年龄，入学时间，密码" << endl;
	cin >> id >> name >> age >> grade >> password;
	p2->setData(id, name, age, grade, password);
	cout << "操作成功" << endl;
	return;

}
void StudentList::deleteStudentByName(string name) {
	Student* p2 = findStudentById(name);
	if (p2 == nullptr)return;
	cout << "准备要删除该项";
	p2->showStudentInfo();
	Student* pre = head;
	Student* p = head;
	if (p->getid() == name) {
		head = p->next;
		pre->~Student();
		return;
	}
	p = p->next;
	while (p != nullptr) {
		if (p->getid() == name) {
			pre->next = p->next;
			p->~Student();
			return;
		}
		p = p->next;
		pre = p->next;
	}
}
