
/*
// 第三次实验题2参考代码
*/

#include <stdio.h>
#include <stdlib.h>


//栈最大长度
#define MAXSIZE  1024


//TODO: 从题1拷贝  datatype和SeqStack的定义，以及已完善的栈操作函数
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
    if (s == NULL || s->top == -1)
    {
        return 0;
    }
    *x = s->data[s->top];

    return 1;
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

/*
// 完成栈类型为char的基本函数定义
*/

/*-----------------------------------------------------------------------------------------------*/
// 题2：表达式求值
/*-----------------------------------------------------------------------------------------------*/


//栈元素类型为整数的类型
typedef int datatype2;

//栈信息结构体
typedef struct {
    datatype2 data[MAXSIZE];
    int  top;
} SeqStack2;


//初始化栈
SeqStack2* Init_SeqStack2()
{
    SeqStack2* s;
    s = (SeqStack2*)malloc(sizeof(SeqStack2));
    if (NULL == s)
    {
        return NULL;
    }
    s->top = -1;
    return s;
}


//判空栈
int Empty_SeqStack2(SeqStack2* s)
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
int Push_SeqStack2(SeqStack2* s, datatype2 x)
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
int Pop_SeqStack2(SeqStack2* s, datatype2* x)
{
    // TODO: 待完善（0.5分）
    if (s->top == -1)
    {
        return 0;
    }

    s->data[s->top] = 0;
    *x = s->data[s->top];
    s->top--;
    return 0;
}


//取栈顶元素
int Top_SeqStack2(SeqStack2* s, datatype2* x)
{
    // TODO: 待完善（0.5分）
    if (s->top == -1 || s == NULL)
    {
        return 0;
    }

    *x = s->data[s->top];
    return 0;
}


//销毁栈
int Destroy_Stack2(SeqStack2* s)
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


/*
// 利用算术操作类型op_type实现第一操作数data1和第二操作数data2之间的算术运算
*/
int operation(int data1, int data2, char op_type)
{
    int value;

    if (op_type == '+')
    {
        value = data1 + data2;
    }
    else if (op_type == '-')
    {
        value = data1 - data2;
    }
    else if (op_type == '*')
    {
        value = data1 * data2;
    }
    else if (op_type == '/')
    {
        value = data1 / data2;
    }

    printf("%d %c %d = %d\n", data1, op_type, data2, value);
    return value;
}

/*
// 获取运算符优先级的函数
*/
int get_level(char op_type)
{
    if (op_type == '*' || op_type == '/')
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
// 表达式求值，待完善
*/
void caculate_expression()
{
    SeqStack* S1;
    SeqStack2* S2;
    char input, op;
    int ret1, ret2, ret3;
    int value, cnt;
    int data1, data2;
    int err_flag;
    char buf[32];

    printf("表达式计算测试程序开始:\n");

    //初始化栈1，用于保存操作符    
    S1 = Init_SeqStack();
    if (NULL == S1)
    {
        printf("操作符顺序栈创建失败\n");
        return;
    }

    //初始化栈2，用于保存操作数    
    S2 = Init_SeqStack2();
    if (NULL == S2)
    {
        printf("操作数顺序栈创建失败\n");
        return;
    }

    printf("请输入运算表达式： \n");

    buf[0] = 0;
    cnt = 0;

    while (1)
    {
        input = getchar();
        //printf("%c \n", input);

        /*对输入的字符逐个进行解析*/
        if ('#' == input) /* 收到结束符 */
        {
            // 完成操作数从字符到数字的转换
            if (cnt > 0)  /*如果在#输入前有数字字符输入，则将数字字符转换为数值，并压栈*/
            {
                buf[cnt] = 0;
                value = atoi(buf);
                Push_SeqStack2(S2, value);
                buf[0] = 0;
                cnt = 0;
            }

            //TODO: 进行算术运算处理（1.5分）
            while (!Empty_SeqStack(S1))
            {

                Top_SeqStack2(S2, &data1);
                Pop_SeqStack2(S2, &ret2);
                Top_SeqStack2(S2, &data2);
                Pop_SeqStack2(S2, &ret2);

                Top_SeqStack(S1, &op);
                Push_SeqStack2(S2, operation(data2, data1, op));
                Pop_SeqStack(S1, &op);
            }
            break;
        }
        else if (input >= '0' && input <= '9')  /*检测到操作数字符，先将连续的字符保存到数组buf中*/
        {
            buf[cnt] = input;
            cnt++;
        }
        else if ('+' == input || '-' == input || '*' == input || '/' == input)  /*检测到操作符号*/
        {
            // 完成将操作数从字符到数字的转换并压栈
            if (cnt > 0)
            {
                buf[cnt] = 0;
                value = atoi(buf);
                Push_SeqStack2(S2, value);
                buf[0] = 0;
                cnt = 0;
            }

            if (Top_SeqStack(S1, &op) == 1)  /*存在已入栈的操作符*/
            {
                // TODO: 根据当前输入运算符和栈顶运算符之间的优先级比较情况，处理算术运算（1.5分）
                if (get_level(op) == 1)
                {
                    Top_SeqStack2(S2, &data1);
                    Pop_SeqStack2(S2, &ret2);
                    Top_SeqStack2(S2, &data2);
                    Pop_SeqStack2(S2, &ret2);
                    Push_SeqStack2(S2, operation(data2, data1, op));

                    Pop_SeqStack(S1, &op);
                    Push_SeqStack(S1, input);
                }
                else
                {
                    Push_SeqStack(S1, input);
                }


            }
            else
            {
                /*将第一个操作符压栈*/
                Push_SeqStack(S1, input);
            }
        }
        else if (input == ' ')
        {
            if (cnt > 0) /*如果在空格前有数字字符输入，则将数字字符转换为数值，并压栈*/
            {
                buf[cnt] = 0;
                value = atoi(buf);
                Push_SeqStack2(S2, value);
                buf[0] = 0;
                cnt = 0;
            }
            continue; /*输入为空格，继续输入*/
        }
        else
        {
            printf("输入字符错误,请输入数字、加减乘除运算符或空格，并以#号键加回车结束\n");
            break;
        }

    }

    Top_SeqStack2(S2, &ret3);
    Pop_SeqStack2(S2, &ret1);

    if (Empty_SeqStack(S1) && Empty_SeqStack2(S2))
    {
        // TODO: 打印输出运算表达式计算得到的数值（0.5分）
        printf("最后答案 = %d\n", ret3);

    }
    else
    {
        printf("运算表达式不完整!!!\n");
    }

    //销毁栈
    Destroy_Stack(S1);
    Destroy_Stack2(S2);

    fflush(stdin);
    printf("表达式计算结束，请按任意键结束测试\n");
    getchar();

}


void main()
{
    caculate_expression();
}

