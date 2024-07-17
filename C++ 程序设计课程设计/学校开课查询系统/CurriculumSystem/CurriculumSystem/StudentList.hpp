#pragma once
#include"Student.hpp"
class StudentList {
public:
	StudentList();
	virtual ~StudentList();
	//头插法添加学生
	void addStudent(Student& stu2);
	//学生登录
	Student* login();
	//信息录入文件
	void write();
	//根据学号查找学生
	Student* findStudentById(string id);
	//取消id为id的课的选课记录
	void deletecourseId(int id);
	//根据学号更新学生信息
	void updateStudentInfo(string id);
	//根据学生姓名删除学生
	void deleteStudentByName(string id);//删除特定学生
protected:
	Student* head;
	int size;
};
