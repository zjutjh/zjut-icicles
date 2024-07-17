#include<iostream>

using namespace std;

class xclass {

public:

	xclass() :x(1) { cout << "cons()" << x << endl; }

	xclass(int xv) :x(xv) { cout << "cons(int)" << x << endl; }

	xclass(const xclass& obj) :x(obj.x) { cout << "copy cons" << endl; }

	int get()const { return x; }

	void set(int xv) { x = xv; }

	~xclass() { cout << "bye" << x << endl; }

private:

	int x;

};//----------------------------------------------

xclass fun(int x) {

	xclass a(x);

	return a;

}//-----------------------------------------------

int main() {

	xclass a;

	for (int i = 4; i < 6; i++) {

		fun(i).set(i + 1);

	}

	xclass b(3);

	return 0;

}