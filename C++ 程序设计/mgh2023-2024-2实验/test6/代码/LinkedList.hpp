#pragma once
typedef int dataType; //处理的数据类型以int为例
using namespace std;
class node {
public:
	node(dataType d, node* ptr = nullptr) { data = d; next = ptr; }
	dataType data;
	node* next;
};
node* buildListForward()//从键盘输入若干数据，正向构建链表，返回表首。
{
    dataType xdata;//接受输入的临时变量
    cin >> xdata;
    node* head = new node(xdata), * tail = head, * Nnode; //建立第一个结点，表首和表尾指示同一个结点
    char c;
    while ((cin >> xdata).get(c)) //使用 cin 构建循环
    {
        Nnode = new node(xdata); //创建新结点空间
        tail->next = Nnode; //将新结点挂接在表尾
        tail = Nnode; //新结点成为新表尾
        if (c == '\n') break;
    }
    return head;
}
node* buildListReverse()//从键盘输入若干数据，逆向构建链表，返回表首。
{
    dataType xdata;
    node* head = NULL, * Nnode; //建立第一个结点，表首和表尾指示同一个结点
    char c;
    while ((cin >> xdata).get(c)) //使用 cin 构建循环，当输入为非 dataType 类型或者 Ctrl+Z 结束
    {
        Nnode = new node(xdata); //创建新结点空间
        Nnode->next = head;
        head = Nnode; //新结点成为新表首
        if (c == '\n') break;
    }
    return head;
}
void display(node* xhead)//根据表首依次访问链表结点数据，数据输出使用逗号分隔。
{
    node* p = xhead; //定义指针变量p指向表首
    if (p != NULL)
    {
        cout << p->data;
        p = p->next;
        while (p != NULL)//当p为空时，链表访问结束
        {
            cout << "," << p->data;
            p = p->next;//p指向下一个结点
        }
        cout << endl;
    }

}
void search(node* xhead, dataType value, node*& p, node*& pre)//根据目标值value，在给定表首的链表中查找value的位置p以及前驱点pre
{
    p = xhead; //定义指针变量p指向表首
    pre = NULL;//定义指针变量pre指向空结点
    while (p != NULL && p->data != value)//当p为空或者p所指数据不等于目标值时，查找结束
    {
        pre = p;//将p前驱结点赋值给pre
        p = p->next;//p指向下一个结点
    }
}
void deleteList(node*& xhead)//根据给定的表首回收链表的所有结点
{
    node* p = xhead, * q; //定义指针变量p指向表首，q为临时指针
    while (p != NULL)//当p为空时，链表访问结束
    {
        q = p;//将p结点赋值给q
        p = p->next;//p指向下一个结点
        delete q;//回收q所指结点空间
    }
    xhead = NULL;//同步更新表首
}
void add2tail(node*& xhead, node*& xtail, dataType value)//根据给定的value，申请新结点并将其添加到链表的尾部
{
    node* newNode = new node(value);
    if (!xhead) {
        xhead = newNode;
        xtail = newNode;
    }
    else {
        xtail->next = newNode;
        xtail = newNode;
    }
}
void add2head(node*& xhead, node*& xtail, dataType value)//根据给定的value，申请新结点并将其添加到链表的首部
{
    node* Nnode = new node(value); //创建新结点空间
    Nnode->next = xhead;
    xhead = Nnode;//新结点成为新表首
}
node* findListTail(node* head)//根据表首找到链表的最后一个结点（尾部）并返回
{
    node* p = head, * q = NULL; //定义指针变量p指向表首，q为临时指针
    while (p != NULL)//当p为空时，链表访问结束
    {
        q = p;//将p结点赋值给q
        p = p->next;//p指向下一个结点
    }
    return q;//返回表尾
}



void addAfterP(node*& xhead, node* p, dataType value)
{
    node* Nnode = new node(value);
    if (p == NULL)
    {
        Nnode->next = xhead;
        xhead = Nnode;
    }
    else
    {
        Nnode->next = p->next;
        p->next = Nnode;
    }
}
void addBeforeP(node*& xhead, node* pre, node* p, dataType value)
{
    node* Nnode = new node(value, p);
    if (p == NULL)
    {
        Nnode->next = xhead;
        xhead = Nnode;
    }
    else
    {
        if (pre == NULL)
        {
            xhead = Nnode;
        }
        else  pre->next = Nnode;
    }
}
void deleteP(node*& xhead, node* pre, node* p)
{
    if (p == xhead)
    {
        xhead = p->next;
        delete p;
    }
    else
    {
        pre->next = p->next;
        delete p;
        p = pre->next;
    }
}
void add2OrderedList(node*& xhead, dataType value)
{
    node* Nnode = new node(value);
    if (xhead == NULL)
    {
        xhead = Nnode;
    }
    else if (xhead->next == NULL)
    {
        if (xhead->data <= value)
        {
            xhead->next = Nnode;
        }
        else
        {
            Nnode->next = xhead;
            xhead = Nnode;
        }
    }
    else if (xhead->data > value)
    {
        Nnode->next = xhead;
        xhead = Nnode;
    }
    else
    {
        node* pre = xhead;
        node* p = xhead->next;
        while (p != NULL)
        {
            if (p->data > value)
            {
                Nnode->next = p;
                pre->next = Nnode;
                break;
            }
            p = p->next;
            pre = pre->next;
        }
        pre->next = Nnode;
    }
}
void reverseList(node*& xhead)
{
    if (xhead != NULL && xhead->next != NULL)
    {
        node* p = xhead->next;
        node* pre = xhead;
        node* s = p->next;
        while (p != NULL)
        {
            p->next = pre;
            pre = p;
            p = s;
            if (s != NULL) s = s->next;

        }
        xhead->next = NULL;
        xhead = pre;
    }
}
node* copyList(node* xhead)
{
    if (xhead != NULL)
    {
        node* head = new node(xhead->data);
        node* tail = head;
        node* p = xhead->next;
        while (p != NULL)
        {
            node* Nnode = new node(p->data);
            tail->next = Nnode;
            tail = Nnode;
            p = p->next;
        }
        return head;
    }
    return NULL;
}
void selectSortList(node* xhead)
{
    if (xhead != NULL)
    {
        node* p, * pre = xhead;

        while (pre->next != NULL)
        {
            node* min = pre;
            p = pre->next;
            while (p != NULL)
            {
                if (p->data < min->data)
                {
                    min = p;
                }
                p = p->next;
            }
            dataType d = pre->data;
            pre->data = min->data;
            min->data = d;
            pre = pre->next;
        }
    }
}
void bubbleSortList(node* xhead)
{
    if (xhead != NULL)
    {
        node* p, * pt;
        node* pre = xhead;
        while (pre != NULL)
        {
            p = xhead;
            pt = p->next;
            while (pt != NULL)
            {
                if (p->data > pt->data)
                {
                    dataType d = p->data;
                    p->data = pt->data;
                    pt->data = d;
                }
                p = pt;
                pt = pt->next;
            }
            pre = pre->next;
        }

    }
}
void deleteAllValues(node*& xhead, dataType value)
{
    if (xhead != NULL)
    {
        node* p = xhead;
        node* pt = xhead->next;
        while (xhead->data == value)
        {
            delete p;
            xhead = pt;
            if (xhead == NULL) break;
            p = pt;
            pt = pt->next;
        }

        while (pt != NULL)
        {
            if (pt->data == value)
            {
                p->next = pt->next;
                delete pt;
                pt = p->next;
            }
            else
            {
                p = p->next;
                pt = pt->next;
            }
        }
    }
}
