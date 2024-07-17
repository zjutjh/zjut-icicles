#include "Manager.hpp"
#include"string"
#include<iostream>
Manager::Manager(string account, string password){
	this->account = account;
	this->password = password;
}

Manager::~Manager(){}
bool Manager::login(){
	string account, password;
	cout << "ÇëÊäÈëÕËºÅ: "; cin >> account;
	cout << "ÇëÊäÈëÃÜÂë: "; cin >> password;
	if (this->account == account && this->password == password){
		cout << "µÇÂ½³É¹¦" << endl;
		return true;
	}
	cout << "µÇÂ½Ê§°Ü" << endl;
	return false;
}
void Manager::setPassword(){
	cout << "ÇëÊäÈëÐÂµÄÃÜÂë";
	cin >> password;
	cout << "ÐÞ¸Ä³É¹¦" << endl;
}
