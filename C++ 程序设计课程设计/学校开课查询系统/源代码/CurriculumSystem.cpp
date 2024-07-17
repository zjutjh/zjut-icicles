#include <iostream>
#include<fstream>
#include"CourseList.hpp"
#include"StudentList.hpp"
#include"Manager.hpp"
using namespace std;
void init();
void backToMenu(int x);
void showMainMenu();
void showSubQueryMenu();
void showSubStudentMenu();
void showSubManagerMenu();
void readyforBack();
void divider();
void subMenu1(bool subMenuRunning);
void subMenu2(bool subMenuRunning);
void subMenu3(bool subMenuRunning);
StudentList studentList;
CourseList courseList;
Manager manager;
int main() {
	init();
	bool continueRunning = true;
	while (continueRunning) {
		bool subMenuRunning = true;
		showMainMenu();
		int mainMenuId; cin >> mainMenuId;
		system("cls");
		switch (mainMenuId) {
		case 1:
			subMenu1(subMenuRunning);
			break;
		case 2: {
			subMenu2(subMenuRunning);
			break;
		}
		case 3:
			subMenu3(subMenuRunning);
			break;
		case -1:
			continueRunning = false; break;
		default:
			cout << "输入错误" << endl;
			break;
		}
	}
	cout << "数据保存中。。。" << endl;
	courseList.write();
	studentList.write();
	cout << "数据保存完成~~~" << endl;
	cout << "期待下次使用~";
	return 0;

}

void init() {
	//初始化课程与学生链表
	ifstream studentFile("Student.txt");
	ifstream courseFile("Course.txt");
	ifstream managerFile("Manager.txt");

	//初始学生链表
	string id, name, password;
	int age, grade, cnt, data[50];
	while (studentFile >> id >> name >> age >> grade >> password >> cnt) {
		for (int i = 0; i < cnt; i++)
			studentFile >> data[i];
		Student temp(id, name, age, grade, password, cnt, data);
		studentList.addStudent(temp);
	}
	studentFile.close();

	//初始课程链表
	int courseId, maxium;
	string  teacherName, type, department;
	double credit;
	while (courseFile >> courseId >> name >> teacherName >> credit >> type >> department >> cnt >> maxium) {
		Course courseTemp(courseId, name, teacherName, credit, type, department, cnt, maxium);
		courseList.add(courseTemp);
	}
	courseFile.close();

	string accont;
	managerFile >> accont >> password;
	manager = Manager(accont, password);
	managerFile.close();
}

void subMenu1(bool subMenuRunning) {
	while (subMenuRunning) {
		showSubQueryMenu();
		int queryMenuSubId; cin >> queryMenuSubId;
		system("cls");
		switch (queryMenuSubId) {
		case 1: {
			courseList.show();
			courseList.sortCourseMenu();
			courseList.show();
			cout << "输入任意数字返回" << endl;
			int st; cin >> st;;
			system("cls");
		}
			  break;
		case 2:
			courseList.findCourseMenu();
			system("cls");
			break;
		case -1:
			subMenuRunning = false;
			break;
		}
	}
}
void subMenu2(bool subMenuRunning) {
	Student* curstudent = studentList.login();
	if (curstudent == nullptr) return;
	while (subMenuRunning) {
		showSubStudentMenu();
		int studentMenuId; cin >> studentMenuId;
		switch (studentMenuId) {
		case 1:
			curstudent->showStudentInfo();
			readyforBack();
			break;
		case 2:
			for (int j = 0; j < curstudent->getSelectedCourseCnt(); j++) {
				courseList.findByCourseId(curstudent->mySelectedCourses[j]);
			}
			readyforBack();
			break;
		case 3:
			courseList.findCourseMenu();
			readyforBack();
			break;
		case 4: {
			cout << "请输入编号:";
			int temp; cin >> temp;
			int t = curstudent->chooseCourse(temp, courseList.findByCourseId(temp)->studentChooseable());
			courseList.findByCourseId(temp)->updateSelectedCnt(t);
			readyforBack();
			break;
		}
		case 5:
			curstudent->setPassword();
			readyforBack();
			break;
		case -1:
			subMenuRunning = false;
			break;
		}
		system("cls");
	}
	system("cls");
}
void subMenu3(bool subMenuRunning) {
	if (!manager.login())return;
	while (subMenuRunning) {
		showSubManagerMenu();
		int managerMenuId; cin >> managerMenuId;
		system("cls");
		switch (managerMenuId) {
		case 1:
			courseList.show();
			readyforBack();
			break;
		case 2:
			courseList.findCourseMenu();
			cout << "1.修改课程信息" << endl;
			cout << "2.删除课程信息" << endl;
			cout << "请选择（1/2,输入-1返回上层结构）：" << endl;
			int tempMenuId; cin >> tempMenuId;
			while (tempMenuId != -1) {
				int temp;
				if (tempMenuId == 1) {
					cout << "请输入要修改的课程的编号";
					cin >> temp;
					Course* tempCourse = courseList.findByCourseId(temp);
					if (tempCourse == nullptr) {
						cout << "没有查询到此课程" << endl;
						break;
					}
					cout << "准备要修改该项";
					tempCourse->showStudentInfo();
					courseList.updateCourseInfo(tempCourse);
					cout << "操作成功" << endl;
				}
				else if (tempMenuId == 2) {
					cout << "请输入要删除的课程的编号!";
					cin >> temp;
					Course* tempCourse = courseList.findByCourseId(temp);
					if (tempCourse == nullptr) {
						cout << "没有查询到此课程!" << endl;
						break;
					}
					cout << "准备删除该项";
					courseList.findByCourseId(temp)->showStudentInfo();
					studentList.deletecourseId(temp);//删除所有学生中关联的信息；
					courseList.removeCourse(temp);
					cout << "操作成功" << endl;
				}
				else {
					cout << "输入有误!" << endl;
				}
				readyforBack();
				break;
			}
			break;
		case 3: {
			int courseId;
			string teacherName, type, department, name;
			double credit;
			cout << "请输入新课程的编号，名字，任课教师，学分，课程性质，开课学院" << endl;
			cin >> courseId >> name >> teacherName >> credit >> type >> department;
			Course courseTemp(courseId, name, teacherName, credit, type, department, 0, 50);
			courseList.add(courseTemp);
			cout << "操作成功" << endl;
			readyforBack();
			break;
		}
		case 4:
			manager.setPassword();
			readyforBack();
			break;
		case 5: {
			cout << "请输入要修改的学生学号";
			string studentId; cin >> studentId;
			studentList.updateStudentInfo(studentId);
			readyforBack();
			break;
		}
		case 6: {
			cout << "请输入要删除的学生学号";
			string studentId; cin >> studentId;
			studentList.deleteStudentByName(studentId);
			readyforBack();
			break;
		}
		case 7: {
			string studentId;
			string id, name, password;
			int age, grade;
			cout << "请输入新学生的学号，姓名，年龄，入学时间，密码" << endl;
			cin >> id >> name >> age >> grade >> password;
			Student studentTemp(id, name, age, grade, password);
			studentList.addStudent(studentTemp);
			readyforBack();
			break;
		}
		case -1:
			subMenuRunning = false;
			break;
		default:
			cout << "输入有误！" << endl;
		}
	}
}
void showMainMenu() {
	cout << "    -------------------------------------------------\n";
	cout << "    |           欢迎使用开课查询系统                |" << endl;
	cout << "    |           1.查询开课信息                      |" << endl;
	cout << "    |           2.学生登录                          |" << endl;
	cout << "    |           3.管理员登录                        |" << endl;
	cout << "    ------------------------------------------------\n";
	cout << "               请选择（1/2/3,-1为结束）：";
}
void showSubQueryMenu() {
	divider();
	cout << "1.显示所有课程信息与排序" << endl;
	cout << "2.查找" << endl;
	backToMenu(2);
}
void showSubStudentMenu() {
	divider();
	cout << "1.查看个人信息" << endl;
	cout << "2.显示已选课程" << endl;
	cout << "3.查询开课列表" << endl;
	cout << "4.按课程编号选课/退课" << endl;
	cout << "5.修改密码" << endl;
	backToMenu(5);
}
void showSubManagerMenu() {
	divider();
	cout << "1.显示所有课程信息" << endl;
	cout << "2.查找课程,查找之后可进行修改，删除操作" << endl;
	cout << "3.添加课程" << endl;
	cout << "4.修改密码" << endl;
	cout << "5.修改学生" << endl;
	cout << "6.删除学生" << endl;
	cout << "7.添加学生" << endl;
	backToMenu(7);
}
void readyforBack() {
	string temp;
	cout << "任意输入返回";
	cin >> temp; system("cls");
}

void divider() {
	cout << "------------------分割线------------------------\n";
}

void backToMenu(int x) {
	cout << "请选择（";
	bool flag = false;
	for (int i = 1; i <= x; i++) {
		if (flag)cout << "/";
		flag = true;
		cout << i;
	}
	cout << ", 输入-1返回上层结构）：" << endl;
}