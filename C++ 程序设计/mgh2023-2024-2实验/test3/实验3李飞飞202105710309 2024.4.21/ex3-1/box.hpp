#pragma once
#include <iostream>
using namespace std;

class box {
public:
	box(int hv = 10, int wv = 10, int lv = 10) :h(hv), w(wv), len(lv) {
		cout << "I\'m a box:" << h << "-" << w << "-" << len << endl;
	}
	box(const box& other) {
		h = other.h;
		w = other.w;
		len = other.len;
		cout << "I\'m a copy box:" << h << "-" << w << ":" << len << endl;
	}
	void reset() {
		h = 10;
		w = 10;
		len = 10;
	}

	~box() {
		cout << "A box says: Bye-bye." << h << "-" << w << "-" << len << endl;
	}
private:
	int h, w, len;
};
