//list_demo2.cpp
//2、链表的进阶工作

#include<iostream>
#include "LinkedList.hpp" //包含node的定义 和实验内容1的 1）-8）的链表工作 + 实验内容2 的1)-8)的链表工作


int main() {
	node* head1;
	node* head2;
	dataType xdata;

	head1 = NULL;
	//正向和逆向建链测试   
	head1 = buildListForward();//输入1 2 3 4 5 6 7 8 9正向构建链表
	cout << "The result of buildListForward:";
	display(head1);//测试输出1,2,3,4,5,6,7,8,9

	node* p, * pre;
	search(head1, 5, p, pre);//查找15
	addAfterP(head1, p, 11);//添加在15之前
	addBeforeP(head1, pre, p, 22);//添加在15之后
	display(head1);//测试输出1,2,3,4,22,5,11,6,7,8,9

	search(head1, 10, p, pre);//查找10
	addAfterP(head1, p, 33);//p为空，添加在表首
	addBeforeP(head1, pre, p, 44);//p为空，添加在表首
	display(head1);//测试输出44,33,1,2,3,4,22,5,11,6,7,8,9

	search(head1, 44, p, pre);//查找44
	addAfterP(head1, p, 55);//添加在44之后
	addBeforeP(head1, pre, p, 66);//添加在44之前
	display(head1);//测试输出66,44,55,33,1,2,3,4,22,5,11,6,7,8,9

	search(head1, 44, p, pre);//查找44
	deleteP(head1, pre, p);
	display(head1);//测试输出66,55,33,1,2,3,4,22,5,11,6,7,8,9

	search(head1, 66, p, pre);//查找66
	deleteP(head1, pre, p);
	display(head1);//测试输出55,33,1,2,3,4,22,5,11,6,7,8,9

	reverseList(head1);//逆置链表
	display(head1);//测试输出9,8,7,6,11,5,22,4,3,2,1,33,55

	head2 = copyList(head1);//备份head1

	selectSortList(head1);
	display(head1);//排序输出1,2,3,4,5,6,7,8,9,11,22,33,55

	display(head2);//测试输出9,8,7,6,11,5,22,4,3,2,1,33,55
	bubbleSortList(head2);
	display(head2);//排序输出1,2,3,4,5,6,7,8,9,11,22,33,55

	while (cin >> xdata) {//依次输入-10 4 10  100  56  4 44 0
		if (xdata == 0) break;
		add2OrderedList(head1, xdata);
	}
	display(head1);//-10,1,2,3,4,4,4,5,6,7,8,9,10,11,22,33,44,55,56,100

	search(head1, -10, p, pre);//查找-10
	deleteP(head1, pre, p);//删除-10
	display(head1);//输出：1,2,3,4,4,4,5,6,7,8,9,10,11,22,33,44,55,56,100

	search(head1, 44, p, pre);//查找44
	deleteP(head1, pre, p);//删除44
	display(head1);//输出：1,2,3,4,4,4,5,6,7,8,9,10,11,22,33,55,56,100

	search(head1, 22, p, pre);//查找22
	addAfterP(head1, p, 4);
	addBeforeP(head1, pre, p, 4);
	display(head1);//输出：1,2,3,4,4,4,5,6,7,8,9,10,11,4,22,4,33,55,56,100

	search(head1, 1, p, pre);//查找1
	addAfterP(head1, p, 4);
	addBeforeP(head1, pre, p, 4);
	display(head1);//输出：4,1,4,2,3,4,4,4,5,6,7,8,9,10,11,4,22,4,33,55,56,100

	deleteAllValues(head1, 4);
	display(head1);//输出：1,2,3,5,6,7,8,9,10,11,22,33,55,56,100

	deleteList(head1);
	deleteList(head2);

	return 0;
}