
/*
// 实验参考代码
*/

#include <stdio.h>
#include <stdlib.h>



/*-----------------------------------------------------------------------------------------------*/
// 题目：单链表置逆
/*-----------------------------------------------------------------------------------------------*/


/*
// 单链表结点定义
*/
typedef struct node
{
    int data;
    struct node* next;
}NODE;



/*
// 尾部插入结点建立含头结点的单链表函数
*/
NODE* create_nodes_list()
{
    NODE* H = NULL;
    NODE* s, * r = NULL;
    int ret, data;
    char exit_flag;

    H = (NODE*)malloc(sizeof(NODE));
    if (H == NULL)
    {
        return NULL;
    }

    /*初始化头结点参数*/
    H->data = 0;
    H->next = NULL;

    r = H;

    printf("请输入单链表的数据域,直接输入q加回车结束创建\n");
    while (1)
    {
        ret = scanf_s("%d", &data);
        scanf_s("%c", &exit_flag);

        if (ret == 1)
        {
            s = (NODE*)malloc(sizeof(NODE));
            if (s == NULL)
            {
                break;
            }
            s->data = data;
            s->next = NULL; /*新创建结点加在队尾，初始化指针域为空*/

            r->next = s;

            /*r指向队尾的结点*/
            r = s;
        }
        else if (exit_flag == 'q')
        {
            printf("创建一元多项式链表结束\n");
            break;
        }
        else
        {
            printf("输入参数错误，请按要求输入数值，结束创建请直接输入q加回车\n");
        }

        fflush(stdin);
    }

    return H;
}


/*
// 遍历打印带头结点的链表中所有数据域
*/
void print_nodes_list(NODE* H)
{
    NODE* p;

    if (H == NULL)
    {
        return;
    }

    p = H->next;

    printf("单链表的数据域值如下：\n");

    // TODO（1分）
    while (p)
    {
        printf("%d ", p->data);
        p = p->next;
    }

    printf("\n");
}



/*
// 销毁带头结点的链表函数
*/
void destroy_nodes_list(NODE* H)
{
    NODE* t, * p;

    if (H == NULL)
    {
        return;
    }

    p = H;

    // TODO（1分）
    t = p->next;
    while (t)
    {
        delete p;
        p = t;
        t = p->next;
    }
}



/*
// 单链表置逆
*/
void reverse_list(NODE* H)
{

    // TODO（3分）: 实现单链表逆置
    if (H == NULL)
    {
        return;
    }
    NODE* p = H->next;

    int data[100], i = -1;

    while (p)
    {
        i++;
        data[i] = p->data;
        p = p->next;
    }
    p = H->next;
    while (p)
    {
        p->data = data[i];
        i--;
        p = p->next;
    }

    return;
}



/*
// 单链表置逆测试函数
*/
void test_reverse_list()
{
    NODE* pa = NULL;

    /*创建链表A*/
    printf("\n");
    printf("创建链表\n");
    pa = create_nodes_list();

    /*遍历输出创建的链表*/
    printf("\n");
    printf("逆置之前的链表内容:\n");
    print_nodes_list(pa);

    /*逆置链表*/
    reverse_list(pa);

    /*打印输出逆置之后的链表*/
    printf("\n");
    printf("逆置之后的链表内容：\n");
    print_nodes_list(pa);

    /*销毁链表*/
    destroy_nodes_list(pa);

    getchar();
}


void main()
{
    /*测试单链表逆置*/
    test_reverse_list();

}

