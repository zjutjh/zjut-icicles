#include "student1.hpp"//#include "student2.hpp"
#include <iostream>
using namespace std;

int main()
{
	Student stu1, stu2("Jennie Mao"), stu3(stu2), stu4("John Smith");
	cout << "stu1:" << stu1.GetName() << endl;
	cout << "stu2:" << stu2.GetName() << endl;
	cout << "stu3:" << stu3.GetName() << endl;
	cout << "stu4:" << stu4.GetName() << endl;

	stu1 = stu4;

	cout << "stu1:" << stu1.GetName() << endl;

	stu1.ChangeName("Peter");
	cout << "stu1:" << stu1.GetName() << endl;

	stu3.ChangeName("Tom");
	cout << "stu3:" << stu3.GetName() << endl;

	return 0;
}
