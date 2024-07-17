#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace std;
int main() {
    vector<int> a;
    int number;

    // 读取data1.txt
    ifstream file1("data1.txt");
    while (file1 >> number) {
        a.push_back(number);
    }
    file1.close();

    // 读取data2.txt
    ifstream file2("data2.txt");
    while (file2 >> number) {
        a.push_back(number);
    }
    file2.close();

    // 排序数组
    sort(a.begin(), a.end());

    // 输出到屏幕
    cout << "Array: ";
    for (int i = 0; i < a.size(); ++i) {
        cout << a[i] << " ";
    }
    cout << endl;

    // 写入data3.txt
    ofstream file3("data3.txt");
    for (int i = 0; i < a.size(); ++i) {
        file3 << a[i] << endl;
    }
    file3.close();

    return 0;
}
