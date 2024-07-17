#include<iostream>
#include<ctime>
#include<cstdlib>
using namespace std;
#define ll long long
#define endl '\n'
ll n, m, k;
//随机生成
void InitMatrix(int** &p, int n, int m) {
    p = new int* [n];
    for (int i = 0; i < n; i++)
        p[i] = new int[m];
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            //防止乘法爆int
            p[i][j] = rand() % 10000;
}
//展示
inline void Display(int** p, int n, int m) {
    cout << "展示一个矩阵：" << endl;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++)
            cout << p[i][j] << " ";
        cout << endl;
    }
    cout << endl;
}
//摧毁
void DestroyArray(int** p, int n) {
    for (int i = 0; i < n; i++)
        delete[]p[i];
    delete[]p;
}
//矩阵乘法，生成n*k矩阵
int** Multiply(int** pa, int** pb, int n, int m, int k) {
    int** p3=nullptr;
    InitMatrix(p3, n, k);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < k; j++) {
            p3[i][j] = 0;
            for (int t = 0; t < m; t++)
                p3[i][j] += pa[i][t] * pb[t][j];
        }
    return p3;
}
void solve() {
    cin >> n >> m >> k;
    int** p1=nullptr, ** p2=nullptr;
    InitMatrix(p1, n, m); InitMatrix(p2, m, k);
    Display(p1, n, m); Display(p2, m, k);
    int** p3 = Multiply(p1, p2, n, m, k);
    Display(p3, n, k);
    DestroyArray(p1, n); DestroyArray(p2, m);
    DestroyArray(p3, n);
}
int main() {
    srand(time(0));
    int T = 1;
    //    cin>>T;
    while (T--) solve();
    return 0;
}