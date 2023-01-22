
/*
// 第三次实验题目1参考代码
*/

#include <stdio.h>
#include <stdlib.h>


//栈最大长度
#define MAXSIZE  1024


/*
// 完成栈类型为char的基本函数定义
*/

//栈元素类型为字符的类型
typedef char datatype;

//栈信息结构体
typedef struct {
    datatype data[MAXSIZE];
    int  top;
} SeqStack;


//初始化栈
SeqStack* Init_SeqStack()
{
    SeqStack* s;
    s = (SeqStack*)malloc(sizeof(SeqStack));
    if (NULL == s)
    {
        return NULL;
    }
    s->top = -1;
    return s;
}


//判空栈
int Empty_SeqStack(SeqStack* s)
{
    if (-1 == s->top)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


//入栈
int Push_SeqStack(SeqStack* s, datatype x)
{
    // TODO: 待完善（0.5分）
    if (s->top == MAXSIZE)
    {
        return 0;
    }
    s->data[s->top + 1] = x;
    s->top++;

    return 0;
}


//出栈
int Pop_SeqStack(SeqStack* s, datatype* x)
{
    // TODO: 待完善（0.5分）
    if (Empty_SeqStack(s))
    {
        return 0;
    }
    *x = s->data[s->top];
    s->data[s->top] = ' ';
    s->top--;
    return 0;
}


//取栈顶元素
int Top_SeqStack(SeqStack* s, datatype* x)
{
    // TODO: 待完善（0.5分）
    if (s == NULL)
    {
        return 0;
    }
    *x = s->data[s->top];

    return 0;
}


//销毁栈
int Destroy_Stack(SeqStack* s)
{
    if (NULL == s)
    {
        return 0;
    }
    else
    {
        free(s);
        return 1;
    }
}


/*-----------------------------------------------------------------------------------------------*/
// 题1：检测输入括号匹配状况
/*-----------------------------------------------------------------------------------------------*/

void check_brackets()
{
    SeqStack* S;
    datatype input, topdata;
    int ret = 0;

    printf("括号检测测试程序开始:\n");


    //初始化栈    
    //TODO（0.5分）
    S = Init_SeqStack();
    printf("支持的括号包括：%c, %c, %c, %c, %c, %c \n", '(', ')', '[', ']', '{', '}');
    printf("请输入运算表达式： \n");
    while (1)
    {
        input = getchar();

        /*对输入的字符逐个进行解析*/
        if (input == '#')
        {
            //检测到结束字符，跳出循环
            break;
        }
        else
        {
            // TODO: 根据input的值，进行判别处理。（2分）
            if (input == '(' || input == '[' || input == '{')
            {
                Push_SeqStack(S, input);
            }
            else if (input == ')' || input == ']' || input == '}')
            {
                Top_SeqStack(S, &topdata);
                if (topdata == input - 1 || topdata == input - 2)
                {
                    Pop_SeqStack(S, &topdata);
                }
                else
                {
                    ret = 1;
                    break;
                }
            }
        }
    }


    /*用Empty_SeqStack()函数判别括号是否正确匹配*/
    //TODO（0.5分）
    if (ret == 1 || Empty_SeqStack(S) == 0)
    {
        printf("运算表达式括号不匹配!!!\n");
    }
    else
    {
        printf("运算表达式括号匹配\n");
    }

    //销毁栈
    //TODO（0.5分）
    Destroy_Stack(S);

    fflush(stdin);
    printf("运算符的括号检测结束，请按任意键结束测试\n");
    getchar();

    return;
}


void main()
{
    check_brackets();
}

