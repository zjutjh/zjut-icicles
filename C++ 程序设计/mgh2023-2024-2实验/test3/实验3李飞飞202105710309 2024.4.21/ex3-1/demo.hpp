#pragma once
#include <iostream>
#include "box.hpp"
#include "clock.hpp"
using namespace std;

class Demo {
public:
	Demo(int xv = 0, int hv = 0, int mv = 0, int sv = 0) :x(xv, mv, sv), y(hv, mv, sv) {
		cout << "Demo-Constructor1." << endl;
	}
	Demo(const box& abox, const Clock& aclock) :y(aclock), x(abox) {
		cout << "Demo-Constructor2." << endl;
	}
	Demo(const Demo& other) {
		cout << "Demo-Copy Constructor." << endl;
	}
	Clock getclock() { return y; }
	box getbox() { return x; }
	~Demo() {
		cout << "Demo-Destructor." << endl;
	}
public:
	box x;
	Clock y;
};
#pragma once
