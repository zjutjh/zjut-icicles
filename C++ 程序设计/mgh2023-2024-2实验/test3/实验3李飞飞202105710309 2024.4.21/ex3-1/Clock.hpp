#pragma once
#include <iostream>
using namespace std;
class Clock {
public:
	Clock(int hv = 1, int mv = 1, int sv = 1) :h(hv), m(mv), s(sv) {
		cout << "I\'m a clock:" << h << ":" << m << ":" << s << endl;
	}
	Clock(const Clock& other) {
		h = other.h;
		m = other.m;
		s = other.s;
		cout << "I\'m a copy clock:" << h << ":" << m << ":" << s << endl;
	}
	void reset() {
		h = 0;
		m = 0;
		s = 0;
	}
	~Clock() {
		cout << "A clock says: Bye-bye." << h << ":" << m << ":" << s << endl;
	}
private:
	int h, m, s;
};
