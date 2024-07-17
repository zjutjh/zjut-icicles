//myVector.hpp
#include <iostream>
using namespace std;
const int CAPACITY = 1000;
class myVector {
public:
    //构造系列
    myVector(unsigned n = 10, int value = 0); //用指定值value初始化n个单元  ,n<=CAPACITY
    myVector(const myVector& obj);//拷贝构造
    //赋值重载
    myVector& operator=(const myVector& right);
    //下标运算
    int& operator[](unsigned index);
    //调整容量
    void set_size(unsigned newsize); //调整容量到newsize ,newsize<=CAPACITY
    //获取容量
    int get_size()const;

    //也可以用下面两个函数代替
//void expand(unsigned delta);//调整容量到size+delta
//void subtract(unsigned delta);//调整容量到size-delta
    myVector operator-(); //返回元素逆置存放的向量，即将原向量元素首尾交换的结果，注意原向量保持不变。

    void sort();//升序排序      

    void display() const;//从0开始显示向量元素  ,以逗号分隔每个单元值
    myVector& operator++(); //前置增量重载，将vector内每个单元值增加1，并返回修改后的vector
    myVector operator++(int);//后置增量重载，将vector内每个单元值增加1，并返回修改前的vector
    //输入输出流重载
    friend ostream& operator<<(ostream& out, const myVector& vec);
    friend istream& operator>>(istream& in, myVector& vec);

    friend myVector  operator+(const myVector& left, const  myVector& right);
    friend myVector  operator-(const myVector& left, const myVector& right);
    
private:
    int data[CAPACITY];
    int size; //有效单元个数
};

myVector  operator+(const myVector& left, const  myVector& right);
//表示求复杂可以求left和right的并集, 并集元素个数不超过CAPACITY

myVector  operator-(const myVector& left, const myVector& right);
//表示求left和right的差集

ostream& operator<<(ostream& out, const myVector& vec); //输出流重载。从0开始显示向量元素，以逗号分隔每个单元值
istream& operator>>(istream& in, myVector& vec);   //输入流重载            