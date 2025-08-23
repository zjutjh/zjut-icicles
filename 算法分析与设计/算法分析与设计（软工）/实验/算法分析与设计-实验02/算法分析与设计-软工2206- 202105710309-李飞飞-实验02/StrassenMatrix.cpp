#include<iostream>
#include<cstring>
#include<algorithm>
#include<cmath>
#include<queue>
#include<set>
#include <time.h>
#include<ctime>
#include<cstdlib>
using namespace std;
#define ll long long
#define endl '\n'
// #define int long long
ll n, m;
set<pair<int**, int>> st;

//展示n*n
inline void Display(int** p, int n);
//摧毁
void DestroyArray(int** p, int n);
//随机生成n*n
void InitMatrix(int**& p, int n, bool fill);
//普通矩阵乘法，生成n*n矩阵
int** Multiply(int** pa, int** pb, int n);

//象限检查
int check(int& x, int& y, int& k);
//矩阵加法
int** StrassenAdd(int** pa, int** pb, int n);
//矩阵减法
int** StrassenSub(int** pa, int** pb, int n);
//Strassen矩阵乘法
int** StrassenMultiply(int** pa, int** pb, int n);

void solve() {
    while (1) {
        //非2的整此幂或非方阵只需要填充0直至到达最近整次幂就行
        //不影响算法本身，所以这里不做处理，直接要求必须2^k
        cout << "请输入方阵维数n 要求必须为2^k\n";
        cin >> n;
        if (!(n & n - 1))break;
    }

    //初始答案
    int** A, ** B;
    InitMatrix(A, n, true); InitMatrix(B, n, true);

    freopen("matriaA.txt", "w", stdout);//打开写入
    //展示A,B
    cout << "矩阵A:\n";
    Display(A, n);
    fclose(stdout);

    freopen("matriaB.txt", "w", stdout);//打开写入
    cout << "矩阵B:\n";
    Display(B, n);
    fclose(stdout);


    //计时
    clock_t start, end;
    //展示普通矩阵乘法
    freopen("Multiply.txt", "w", stdout);//打开写入
    start = clock();
    int** D = Multiply(A, B, n);
    end = clock();
    cout << "Strassen矩阵乘法结果:\n";
    cout << "用时 = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
    Display(D, n);
    fclose(stdout);

    //展示Strassen
    freopen("StrassenMultiply.txt", "w", stdout);//打开写入
    start = clock();
    int** C = StrassenMultiply(A, B, n);
    end = clock();
    cout << "Strassen矩阵乘法结果:\n";
    cout << "用时 = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
    Display(C, n);
    fclose(stdout);

    //回收内存，因为我很懒，所以放一起回收了
    for (auto t : st)
        DestroyArray(t.first, t.second);
}
signed main() {
    srand(time(0));
    solve();
    return 0;
}

int check(int& x, int& y, int& k) {
    if (x < k && y < k)return 1;
    else if (x >= k && y >= k)return 4;
    else if (x < k && y >= k)return 2;
    return 3;
}

//Strassen矩阵乘法
int** StrassenMultiply(int** pa, int** pb, int n) {
    int nowSize = n >> 1;
    //初始化答案
    int** ans = nullptr;
    InitMatrix(ans, n, false);

    if (n == 1) {
        ans[0][0] = pa[0][0] * pb[0][0];
        return ans;
    }

    //初始化参数A,B,C
    int** A11, ** A12, ** A21, ** A22;
    int** B11, ** B12, ** B21, ** B22;
    int** C11, ** C12, ** C21, ** C22;
    InitMatrix(A11, nowSize, false); InitMatrix(A12, nowSize, false); InitMatrix(A21, nowSize, false); InitMatrix(A22, nowSize, false);
    InitMatrix(B11, nowSize, false); InitMatrix(B12, nowSize, false); InitMatrix(B21, nowSize, false); InitMatrix(B22, nowSize, false);
    //获取A的四个子矩阵
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            int t = check(i, j, nowSize);
            if (t == 1)
                A11[i][j] = pa[i][j];
            else if (t == 2)
                A12[i][j - nowSize] = pa[i][j];
            else if (t == 3)
                A21[i - nowSize][j] = pa[i][j];
            else A22[i - nowSize][j - nowSize] = pa[i][j];

        }
    //获取B的四个子矩阵
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            int t = check(i, j, nowSize);
            if (t == 1)
                B11[i][j] = pb[i][j];
            else if (t == 2)
                B12[i][j - nowSize] = pb[i][j];
            else if (t == 3)
                B21[i - nowSize][j] = pb[i][j];
            else B22[i - nowSize][j - nowSize] = pb[i][j];

        }

    //初始化参数S
    int** S1, ** S2, ** S3;
    int** S4, ** S5, ** S6;
    int** S7, ** S8, ** S9, ** S10;
    S1 = StrassenSub(B12, B22, nowSize);
    S2 = StrassenAdd(A11, A12, nowSize);
    S3 = StrassenAdd(A21, A22, nowSize);
    S4 = StrassenSub(B21, B11, nowSize);
    S5 = StrassenAdd(A11, A22, nowSize);
    S6 = StrassenAdd(B11, B22, nowSize);
    S7 = StrassenSub(A12, A22, nowSize);
    S8 = StrassenAdd(B21, B22, nowSize);
    S9 = StrassenSub(A11, A21, nowSize);
    S10 = StrassenAdd(B11, B12, nowSize);

    //初始化参数P
    int** P1, ** P2, ** P3, ** P4, ** P5, ** P6, ** P7;
    P1 = StrassenMultiply(A11, S1, nowSize);
    P2 = StrassenMultiply(S2, B22, nowSize);
    P3 = StrassenMultiply(S3, B11, nowSize);
    P4 = StrassenMultiply(A22, S4, nowSize);
    P5 = StrassenMultiply(S5, S6, nowSize);
    P6 = StrassenMultiply(S7, S8, nowSize);
    P7 = StrassenMultiply(S9, S10, nowSize);

    //计算答案分矩阵
    int** t1, ** t2;
    t1 = StrassenAdd(P5, P4, nowSize), t2 = StrassenSub(P2, P6, nowSize);
    C11 = StrassenSub(t1, t2, nowSize);
    C12 = StrassenAdd(P1, P2, nowSize);
    C21 = StrassenAdd(P3, P4, nowSize);
    int** t3, ** t4;
    t3 = StrassenAdd(P5, P1, nowSize), t4 = StrassenAdd(P3, P7, nowSize);
    C22 = StrassenSub(t3, t4, nowSize);

    //汇总答案并返回
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            int t = check(i, j, nowSize);
            if (t == 1)
                ans[i][j] = C11[i][j];
            else if (t == 2)
                ans[i][j] = C12[i][j - nowSize];
            else if (t == 3)
                ans[i][j] = C21[i - nowSize][j];
            else ans[i][j] = C22[i - nowSize][j - nowSize];
        }
    return ans;
}

//矩阵加法
int** StrassenAdd(int** pa, int** pb, int n) {
    int** ans = nullptr;
    InitMatrix(ans, n, false);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            ans[i][j] = pa[i][j] + pb[i][j];
    return ans;
}
//矩阵减法
int** StrassenSub(int** pa, int** pb, int n) {
    int** ans = nullptr;
    InitMatrix(ans, n, false);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            ans[i][j] = pa[i][j] - pb[i][j];
    return ans;
}


//普通矩阵乘法，生成n*n矩阵
int** Multiply(int** pa, int** pb, int n) {
    int** p3 = nullptr;
    int m = n, k = n;
    InitMatrix(p3, n, false);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < k; j++) {
            p3[i][j] = 0;
            for (int t = 0; t < m; t++)
                p3[i][j] += pa[i][t] * pb[t][j];
        }
    return p3;
}
//摧毁
void DestroyArray(int** p, int n) {
    for (int i = 0; i < n; i++)
        delete[]p[i];
    delete[]p;
}
//展示n*n
inline void Display(int** p, int n) {
    cout << "展示一个矩阵：" << endl;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            cout << p[i][j] << " ";
        cout << endl;
    }
    cout << endl;
}
//随机生成n*n
void InitMatrix(int**& p, int n, bool fill) {
    p = new int* [n];
    for (int i = 0; i < n; i++)
        p[i] = new int[n];
    st.insert({ p,n });
    if (!fill)return;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            //防止乘法爆int
            // p[i][j] =3;
            p[i][j] = rand() % 100;
}