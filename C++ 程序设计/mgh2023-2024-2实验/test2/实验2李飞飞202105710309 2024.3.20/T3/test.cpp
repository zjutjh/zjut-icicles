#include "position.hpp"
#include <iostream>
using namespace std;

int main() {
	position a, b, c, d, e;
	a.set(5, 15);
	a.show();//输出(5,15)
	cout << a.quadrant() << endl; //输出a的象限 
	b.set(-4.5, 6.7);
	b.show();
	cout << b.quadrant() << endl; //输出b的象限
	c.set(-10, -100);
	c.show();
	cout << c.quadrant() << endl; //输出c的象限
	d.set(20.5, 5.5);
	d.show();
	cout << d.quadrant() << endl; //输出b的象限
	e.set();//默认为原点
	e.show();
	cout << distance(a,b) << endl;
	cout << distance(c) << endl;//默认求与原点的距离
	cout << a.slope() << endl;//与原点构成直线的斜率
	cout << a.slope(d) << endl;//与d构成直线的斜率
	a.move(3);//沿x轴平移
	a.show();
	b.move(-4, 5);
	b.show();
	c.move(0, 6);//沿y轴平移
	c.show();
	return 0;
}
