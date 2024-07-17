#include"student2.hpp"
#include<iostream>
#include<cstring>
Student::Student(){
	name = new char[1];
	name[0] = '\0';
}

Student::Student(const char* n){
	int i = 0;
	for (; n[i] != '\0'; i++){
	}
	name = new char[i + 1];
	for (int j = 0; j < i; j++){
		name[j] = n[j];
	}
	name[i] = '\0';
}
Student::Student(const Student& other){
	int i = 0;
	for (; other.name[i] != '\0'; i++){
	}
	name = new char[i + 1];
	for (int j = 0; j < i; j++){
		name[j] = other.name[j];
	}
	name[i] = '\0';
}
Student::~Student(){
	delete[]name;
}
void Student::operator=(const Student& right){
	int i = 0;
	for (; right.name[i] != '\0'; i++){
	}
	name = new char[i + 1];
	for (int j = 0; j < i; j++){
		name[j] = right.name[j];
	}
	name[i] = '\0';
}
void Student::ChangeName(const char* n){
	delete[]name;
	int i = 0;
	for (; n[i] != '\0'; i++){
	}
	name = new char[i + 1];
	for (int j = 0; j < i; j++){
		name[j] = n[j];
	}
	name[i] = '\0';
}
char* Student::GetName(){
	return name;
}