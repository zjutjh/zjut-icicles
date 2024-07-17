//Student.hpp
#include<iostream> 
using namespace std;
float score_sum = 0.0f;
int stu_sum = 0;
class student {
public:
	student(int n, int a, float s) :num(n), age(a), score(s)
	{  //每创建一个对象加上总分和人数 
		score_sum += s;
		stu_sum++;
	}
private:
	int num;
	int age;
	float score;
};
float AverageScore() {
	if (stu_sum == 0) return 0;
	return score_sum / stu_sum;
}
int main() {
	student s1(1, 20, 83.5), s2(2, 20, 90), s3(3, 20, 100);
	cout << AverageScore() << endl;
}

//class Student {
//public:
//	Student(int n, int a, float s) :num(n), age(a), score(s) {  }
//	void total();
//	static float average();
//private:
//	int num;
//	int age;
//	float score;
//	static float sum;//总分 
//	static int count;//人数 
//};
//void Student::total(){
//	sum += score;
//	count++;
//}
//float Student::average(){
//	return (sum / count);
//}

//Student.cpp
//#include<iostream>
//#include"Student.hpp"
//using namespace std;
//float Student::sum = 0;
//int Student::count = 0;
//int main(){
//	Student stud[3] = { Student(1001,18,70), Student(1002,19,78), Student(1003,18,98) };
//	for (int i = 0; i < 3; i++){
//		stud[i].total();
//	}
//	cout << "average = " << Student::average() << endl;
//}
