//测试程序：
#include<iostream>
#include "linkedList.hpp"
using namespace std;
int main()
{
    linked_list a1, a2, b, c;
    dataType data;
    //正向和逆向建链测试
    //输入2 ,6, 7, 3, 5, 9,12, 4 ,0     [2 6 7 3 5 9 12 4 0
    while (cin >> data) {
        if (data == 0) break; //输入0结束
        a1.add_front(data);
        a2.add_tail(data);
    }
    a1.display();
    a2.display();
    //链表转置测试
    //输入2 ,16, 3, 8, 15, 4, 9, 7 ,0    [2 16 3 8 15 4 9 7 0
    while (cin >> data) {
        if (data == 0) break; //输入0结束
        b.add_tail(data);
    }
    b.display();
    b.reverse();
    b.display();

    c=a1+b; //测试集合并
    c.display();
    c=a1-b; //测试集合差
    c.display();
    c=a1.intersectionSet(b); //测试集合交
    c.display();
    a1.sort(); //测试升序排序
    a1.display();

    //思考需要降序排序如何做？
    b.add_tail(16); b.add_tail(3); b.display();
    b.delete_repeat();                  b.display();
    node* pos = b.find(15);
    b.add_pos_after(18, pos);   b.display();
    b.add_pos_before(23, pos); b.display();
    b.delete_pos_after(pos);     b.display();
    b.delete_pos_before(pos);  b.display();
    b.Delete(8);                         b.display();

    return 0;
}
