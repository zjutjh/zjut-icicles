#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

using namespace std; 
int main() {
    // 初始化随机数种子
    srand(static_cast<unsigned int>(time(nullptr)));

    // 第一组数据
    int data1[10];
    for (int i = 0; i < 10; ++i) 
	{
        data1[i] = rand() % 100; // 假设范围是0-99
    }

    // 写入data1.txt
    ofstream file1("data1.txt");
    for (int i = 0; i < 10; ++i) 
	{
        file1 << data1[i] << endl;
    }
    file1.close();

    // 第二组数据
    int data2[10];
    for (int i = 0; i < 10; ++i) 
	{
        data2[i] = rand() % 100;
    }

    // 写入data2.txt
    ofstream file2("data2.txt");
    for (int i = 0; i < 10; ++i) 
	{
        file2 << data2[i] << endl;
    }
    file2.close();

    return 0;
}
