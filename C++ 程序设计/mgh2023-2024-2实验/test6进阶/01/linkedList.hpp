#pragma once
#include <iostream>

typedef int dataType;
class node {
public:
	node(dataType d, node* ptr = nullptr) { data = d; next = ptr; }
	dataType data;
	node* next;
};

class linked_list {
public:
	linked_list();
	linked_list(const linked_list& other);
	linked_list& operator=(const linked_list& right);
	int list_size() const;
	//集合并交叉也可以考虑设计为类外函数
	linked_list operator+(const linked_list& right); //链表集合并
	linked_list operator-(const linked_list& right);//链表集合差
	linked_list intersectionSet(const linked_list& right); //链表集合交
	node* find(dataType value); //查找
	bool  find(dataType value, node*& pre, node*& p);
	//考虑修改：bool  find(dataType value,node*& pre,node*&p);
	//找到返回真：p为目标点，pre为前驱点; 找不到返回假：p和pre均为nullptr
	void add_front(dataType value); //添加到首
	void add_tail(dataType value);//添加到尾
	void add_pos_after(dataType value, node* pos); //添加到指定位置之后
	void add_pos_before(dataType value, node* pos);//添加到指定位置之前
	void Delete(dataType value);//删除指定值
	void delete_pos_after(node* pos);//删除指定位置之后
	void delete_pos_before(node* pos);//删除指定位置之前
	void delete_all (dataType value);//删除所有重复值
	void delete_repeat();
	void reverse();//逆置链表
	void sort();//升序排列当前链表   
	void display();//遍历链表
	~linked_list();
private:
	node* head, * tail;
	int size; //若使用哨兵链，则可使用哨兵的数据域代替size
}; //可以考虑设计带哨兵的链表，哨兵结点的信息可以代替size 存储表中结点的个数
