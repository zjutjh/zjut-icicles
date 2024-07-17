#include "position.hpp"
#include <iostream>
using namespace std;
int main() {
	position a; //默认坐标位置为原点
	position b(4, 5), c(b), d(-2, -7), e(1); //b的位置为x轴值4，y轴值5
	position* px;
	cout << "K1 = " << position::K() << endl;
	px = new position(-3.5, 9.8);
	(*px).show(); //在屏幕上输出 (-3.5,9.8)
	cout << "K2 = " << a.K() << endl;
	px->set(99, -48); //重新设置点
	cout << "K3 = " << position::K() << endl;
	delete px;
	cout << "K4 = " << b.K() << endl;
	px = new position[3]{ position(5,15),position(-4.5,16.7),position(-10.8) };
	cout << "b:"; b.show();
	cout << "c:"; c.show();
	c.set(-10, -100);
	cout << "c:"; c.show();
	e.set();//默认为原点
	cout << "e:"; e.show();
	cout << a.distance() << endl;
	cout << c.distance() << endl;//默认求与原点的距离
	cout << a.slope() << endl; //与原点构成直线的斜率
	cout << a.slope(d) << endl;  //与d构成直线的斜率
	a.move(3);//沿x轴平移
	cout << "a:"; a.show();
	b.move(-4, 5);
	cout << "b:"; b.show();
	c.move(0, 6.3);//沿y轴平移
	cout << "c:"; c.show();
	cout << "K5 = " << position::K() << endl;
	delete[]px;
	cout << "K6 = " << position::K() << endl;
	return 0;
}
