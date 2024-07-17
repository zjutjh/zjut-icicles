#pragma once
#include<string>
using namespace std;
class Student {
public:
	Student(string, string, int, int, string, int, int*);
	Student(string, string, int, int, string);
	virtual ~Student();
	void showStudentInfo();//展示个人信息
	void add(int temp);//将课程编号编入mySelectedCourses末尾
	Student(const Student& student);
	Student* next;
	//选课或取消已选的课
	int chooseCourse(int id, bool choiceable);
	//根据课程号获取自己选次课的序号，没有返回-1
	int getSelectedCoursePlaceById(int id);
	//取消选课
	void cancelCourseByPlaceId(int id);
	int* mySelectedCourses;//存放学生所选课程的学生编号

	//getter setter
	string getid();//得到学号
	string getCourseName();
	int getAge();
	int getGreade();
	string getPassword();
	int getSelectedCourseCnt();
	void setPassword();//修改密码
	void setData(string id, string na, int a, int ey, string pa);
protected:
	string studentId;//学号
	string courseName;
	string password;//密码
	int age;
	int studentGreade;//入学年份
	int selectedCourseCnt;//已选课科目数量
};
