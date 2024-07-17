#pragma once
#include<string>
using namespace std;
class Manager {
public:
	Manager(string, string);
	Manager() {};
	virtual ~Manager();
	bool login();
	void setPassword();
protected:
	string account;
	string password;
};
