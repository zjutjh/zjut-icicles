#pragma once
#include<string>
using namespace std;
class Course {
public:
	Course* next;
	Course() {};
	Course(int id, string name, string teacherName, double credit, string type, string department, int cnt, int s);
	virtual ~Course();
	void showStudentInfo();
	void updateCourseInfo(string, string, double, string, string);
	Course& operator =(Course& c);
	bool studentChooseable();
	void updateSelectedCnt(int x);

	//getter setter
	int getNumber();
	string getCourseName();
	string getLecturerName();
	double getCredit();
	string getcourseType();
	string getDepartment();
	int getSelectedCnt();
	int getMaxCnt();

protected:
	int courseId;//课程编号
	string courseName;
	string lecturerName;//任课教师
	double credit;//学分
	string courseType;//课程性质
	string department;//开课院系
	int selectedCnt;//已选该课的人数
	int maxCnt;//该科可选的最大人数
};
