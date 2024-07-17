//list_demo1.cpp
//l、链表的基本构建和访问
#include<iostream>
#include "LinkedList.hpp" //包含node的定义 和实验内容1的 1）-8）的链表工作
int main() {
	node* head1, * tail1;
	node* head2, * tail2;
	dataType xdata;

	head1 = NULL;
	//正向和逆向建链测试   
	head1 = buildListForward();//输入1 2 3 4 5 6 7 8 9正向构建链表
	cout << "The result of buildListForward:";
	display(head1);//测试输出1,2,3,4,5,6,7,8,9	

	head2 = buildListReverse();//输入1 2 3 4 5 6 7 8 9逆向构建链表
	cout << "The result of buildListReverse:";
	display(head2);//测试输出9,8,7,6,5,4,3,2,1

	tail1 = findListTail(head1);//找到表1的表尾
	tail2 = findListTail(head2);//找到表2的表尾

	add2tail(head1, tail1, 11);
	add2head(head1, tail1, 33);
	display(head1);//测试输出33,1,2,3,4,5,6,7,8,9,11

	add2tail(head2, tail2, 22);
	add2head(head2, tail2, 44);
	display(head2);//测试输出44,9,8,7,6,5,4,3,2,1,22

	//回收表2
	deleteList(head2);
	tail2 = NULL;//同步更新表尾

	display(head2);//无输出

	add2tail(head2, tail2, 22);
	add2tail(head2, tail2, 33);
	add2head(head2, tail2, 44);
	add2head(head2, tail2, 55);
	add2tail(head2, tail2, 66);
	display(head2);//测试输出55,44,22,33,66

	//回收表1
	deleteList(head1);
	tail1 = NULL;//同步更新表尾

	//回收表2
	deleteList(head2);
	tail2 = NULL;//同步更新表尾

	//正向构建：逐结点添加在尾
	while (cin >> xdata) {//输入22  96  47  13  15   19  12  24  0
		if (xdata == 0) break;//输入0结束
		add2tail(head1, tail1, xdata);
	}
	display(head1);//测试输出22,96,47,13,15,19,12,24

	//逆向构建：逐结点添加在首
	while (cin >> xdata) {//输入22  96  47  13  15   19  12  24  0
		if (xdata == 0) break;//输入0结束
		add2head(head2, tail2, xdata);
	}
	display(head2);//测试输出24,12,19,15,13,47,96,22

	node* p, * pre;
	search(head1, 15, p, pre);
	if (p) {
		if (pre) cout << pre->data;//目标的前驱
		cout << ',' << p->data;//目标
		if (p->next) cout << ',' << p->next->data; //目标的后继
		cout << endl;
	}

	//添加新结点在查找点后
	p->next = new node(77, p->next);
	//添加新结点在查找点前
	pre->next = new node(88, p);
	display(head1);//测试输出22,96,47,13,88,15,77,19,12,24

	//删除15
	search(head1, 15, p, pre);
	pre->next = p->next;
	delete p;
	display(head1);//测试输出22,96,47,13,88,77,19,12,24

	search(head1, 99, p, pre);
	if (!p) cout << "99 is not in the list head1";

	deleteList(head1);
	deleteList(head2);

	return 0;
}

