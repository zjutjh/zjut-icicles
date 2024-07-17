#include "Student1.hpp"
Student::Student() {
	name[0] = '\0';
}
Student::Student(const char* n){
	int i;
	for (i = 0; n[i] != '\0'; i++)
		name[i] = n[i];
	name[i] = '\0';
}
const char* Student::GetName(){ //返回名字
	return name;
}
void Student::ChangeName(const char* n){
	//改名
	int i;
	for (i = 0; n[i] != '\0'; i++)
		name[i] = n[i];
	name[i] = '\0';
}