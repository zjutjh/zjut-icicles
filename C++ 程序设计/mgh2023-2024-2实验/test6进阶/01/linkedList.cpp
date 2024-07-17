#pragma once
#include<iostream>
#include"linkedList.hpp"
using namespace std;

linked_list::linked_list()
{
	head = tail = nullptr; size = 0;
}   //初始化构造函数

linked_list::linked_list(const linked_list& other)
{
    head = tail = nullptr;
    size = other.size;
    node* p = other.head;
    while (p != nullptr)
    {
        add_tail(p->data);
        p = p->next;
    }
}

linked_list& linked_list::operator=(const linked_list& right)//赋值运算符重载
{
    if (this == &right) return *this; // 如果自我赋值，则直接返回

    // 删除原有元素
    node* p = head;
    while (p != nullptr)
    {
        node* q = p->next;
        delete p;
        p = q;
    }
    head = tail = nullptr;
    size = 0;
    // 复制头和尾指针
    head = tail = nullptr;
    // 复制元素个数
    size = right.size;
    // 遍历右链表
    for (node* p = right.head; p != nullptr; p = p->next)
    {
        add_tail(p->data); // 在尾添加元素
    }
    return *this; // 返回本对象
}

int linked_list::list_size() const
{
    return size;
}

//集合并交叉也可以考虑设计为类外函数
//linked_list linked_list::operator+(const linked_list& right); //链表集合并
//linked_list linked_list::operator-(const linked_list& right);//链表集合差
//linked_list linked_list::intersectionSet(const linked_list& right); //链表集合交

node* linked_list::find(dataType value)//查找
{
    node* p = head;
    while (p != nullptr && p->data != value)
    {
        p = p->next;
    }
    return p;
}

bool linked_list::find(dataType value, node*& pre, node*& p)
{
    p = head;
    while (p != nullptr && p->data != value)
    {
        pre = p;
        p = p->next;
    }
    if (p == nullptr) return false;
    else return true;
}
//考虑修改：bool  find(dataType value,node*& pre,node*&p);
//找到返回真：p为目标点，pre为前驱点; 找不到返回假：p和pre均为nullptr
void linked_list::add_front(dataType value) //添加到首
{
    node* newNode = new node(value);
    newNode->next = head;
    if (head == nullptr) tail = newNode;
    head = newNode;
    size++;
}

void linked_list::add_tail(dataType value)//添加到尾
{
    node* newNode = new node(value);
    if (tail == nullptr) head = newNode;
    else tail->next = newNode;
    tail = newNode;
    size++;
}

void linked_list::add_pos_after(dataType value, node* pos) //添加到指定位置之后
{
    node* newNode = new node(value);
    newNode->next = pos->next;
    pos->next = newNode;
    if (newNode->next == nullptr) tail = newNode;
    size++;
}

void linked_list::add_pos_before(dataType value, node* pos)
{//添加到指定位置之前
    node* newNode = new node(value);
    if (head == pos)
    {
        newNode->next = head;
        head = newNode;
    }
    else
    {
        newNode->next = pos;
        node* pre = nullptr;
        node* p = head;
        while (p != pos)
        {
            pre = p;
            p = p->next;
        }
        pre->next = newNode;
    }
    size++;
}

//删除指定值
void linked_list::Delete(dataType value)
{
    node* pre = nullptr, * p = head;
    while (p != nullptr && p->data != value)
    {
        pre = p;
        p = p->next;
    }
    if (p == nullptr) return;
    if (pre == nullptr) head = p->next;
    else pre->next = p->next;
    delete p;
    size--;
}

void linked_list::delete_pos_after(node* pos)//删除指定位置之后
{
    if (pos == tail) return ;
    else
    {
        node* pre = pos;
        node* p = pre->next;
        pre->next = p->next;
        delete p;
        size--;
        
    }
}

void linked_list::delete_pos_before(node* pos)//删除指定位置之前
{
    if (pos == head) return ;
    else
    {
        node* pre = head;
        node* p = pre->next;
        while (p->next != pos)
        {
            p = p->next;
            pre = pre->next;
        }
        pre->next = p->next;
        delete p;
        size--;
    }
}

void linked_list::delete_all(dataType value)//删除所有重复值
{
    node* p = head;
    while (p != nullptr)
    {
        node* q = p->next;
        if (p->data == value)
        {
            if (p == head) head = q;
            else p->next = q->next;
            delete p;
            size--;
        }
        p = q;
    }
}

void linked_list::delete_repeat()
{
    node* q = head;
    while (q != nullptr)
    {
        node* p = q->next;
        node* pre = q;
        while (p != nullptr)
        {
            if (p->data == q->data)
            {
                pre->next = p->next;
                delete p;
                size--;
                p = pre->next;
            }
            else
            {
                pre = p;
                p = p->next;
            }
        }
        
        q = q->next;
    }
    tail = q;
}

void linked_list::reverse()//逆置链表
{
    node* p = head, * q = nullptr, * r = nullptr;
    while (p != nullptr)
    {
        r = q;
        q = p;
        p = p->next;
        q->next = r;
    }
    tail = head;
    head = q;
}

void linked_list::sort()//升序排列当前链表   
{
    node* p = head;
    dataType temp;
    while (p != nullptr)
    {
        node* q = p->next;
        while (q != nullptr)
        {
            if (p->data > q->data)
            {
                temp = p->data;
                p->data = q->data;
                q->data = temp;
            }
            q = q->next;
        }
        p = p->next;
    }
}

void linked_list::display()//遍历链表
{
    node* p = head;
    while (p != nullptr)
    {
        cout << p->data << " ";
        p = p->next;
    }
    cout << endl;
}


linked_list::~linked_list()
{
    node* p = head;
    while (p != nullptr)
    {
        node* q = p->next;
        delete p;
        p = q;
    }
    head = tail = nullptr;
    size = 0;
}




linked_list linked_list::operator+(const linked_list& right)//链表集合并
{
    linked_list result;
    node* p = head;
    while (p != nullptr)
    {
        result.add_tail(p->data);
        p = p->next;
    }
    p = right.head;
    while (p != nullptr)
    {
        result.add_tail(p->data);
        p = p->next;
    }
    return result;
}
linked_list linked_list::operator-(const linked_list& right)//链表集合差
{
    linked_list result;
    node* p = head;
    while (p != nullptr)
    {
        result.add_tail(p->data);
        p = p->next;
    }
    p = right.head;
    while (p != nullptr)
    {
        result.Delete(p->data);
        p = p->next;
    }
    return result;
}
linked_list linked_list::intersectionSet(const linked_list& right) //链表集合交
{
    linked_list result;
    node* p = head;
    while (p != nullptr)
    {
        result.add_tail(p->data);
        p = p->next;
    }
    p = right.head;
    while (p != nullptr)
    {
        result.find(p->data, result.head, result.tail);
        p = p->next;
    }
    return result;
}