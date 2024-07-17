#include<iostream>
#include "Clock.hpp"
#include "demo.hpp"
using namespace std;
void fun1(Clock xclock) {
	xclock.reset();
}
Demo fun2(Demo xDemo) {
	xDemo.getclock().reset();
	cout << "    ppppp   \n";
	return xDemo;
}
int main()
{
	Clock  aclock(8, 34, 45), bclock(aclock);
	box  abox, bbox(20, 30);
	Demo ademo(1, 2, 3, 4), bdemo(ademo), cdemo(abox, aclock), ddemo(3, 4);
	cout << "    KJJJJJJ   \n";
	Clock* pclock;
	box* pbox;
	Demo* pDemo;
	pclock = new Clock(6, 7, 8);
	cout << "    tttttttt   \n";
	pbox = new box(cdemo.getbox());
	cout << "    tttttttt   \n";
	pDemo = new Demo[2]{ Demo(6,7,8,9),Demo(bbox,bclock) };
	cout << "    KJJJJJJ   \n";
	fun1(*pclock);
	cout << "    KJJJJJJ   \n";
	fun2(pDemo[1]);
	cout << "    KJJJJJJ   \n";
	delete pclock;
	delete[]pDemo;
	delete pbox;
	return 0;
}
