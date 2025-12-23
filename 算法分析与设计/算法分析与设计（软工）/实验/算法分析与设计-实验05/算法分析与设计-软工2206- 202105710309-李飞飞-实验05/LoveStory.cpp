#include<iostream>
#include<cstring>
#include<algorithm>
#include<cmath>
#include<queue>
#include<set>
using namespace std;
#define ll long long
#define endl '\n'
#define pb push_back
#define inf 0x3f3f3f3f
const int N = 5e2 + 10;
ll n, m, needVisited;
int xa, xb, ya, yb;
int mp[N][N];
int dx[8] = { -1,-1,0,1,1,1,0,-1 }, dy[8] = { 0,1,1,1,0,-1,-1,-1 };
int minAns = inf, cntAns;
vector<int> path;
void print() {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++)
            cout << mp[i][j] << " ";
        cout << endl;
    }
    cout << endl;
}
//判断是否有解 防止退化到最劣
void floorfill(int x, int y) {
    for (int i = 0; i < 8; i++) {
        int a = dx[i] + x, b = dy[i] + y;
        if (a < 0 || b < 0 || a >= n || b >= m)continue;
        if (mp[a][b])continue;
        mp[a][b] = -2;
        floorfill(a, b);
    }
}
//上一个位置 步数 2^上次方向 路径记录 当前访问
void dfs(int x, int y, int cnt, int lastDirection, int num,
    multiset<pair<int, int>>& log, set<pair<int, int>>& nowVisited) {
    if (x == ya && y == yb) {
        if (nowVisited.size() != needVisited)return;
        //可能等于或小于
        //由于走一步代价不一定增加，所以存在更新最优的情况
        if (cnt < minAns) {
            path.clear();
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++)
                    path.pb(mp[i][j]);
            minAns = cnt;
            cntAns = 1;
        }
        else cntAns++;
    }
    for (int i = 0; i < 8; i++) {
        int a = x + dx[i], b = y + dy[i];
        if (a < 0 || b < 0 || a >= n || b >= m)continue;
        //前面floorfill时把所有位置涂成-2了
        if (mp[a][b] != -2)continue;
        //如果没转向&后为真
        int nowCnt = cnt + !(lastDirection & (1 << i));
        //最优性剪枝
        if (nowCnt > minAns)continue;

        log.insert({ a,b });
        nowVisited.insert({ a,b });
        mp[a][b] = num;

        dfs(a, b, nowCnt, 1 << i, num + 1, log, nowVisited);
        mp[a][b] = -2;
        //恢复现场
        log.erase(log.find({ a,b }));
        if (log.find({ a,b }) == log.end())
            nowVisited.erase({ a,b });
    }
}
void solve() {
    int k;
    cin >> n >> m >> k;
    for (int i = 0; i < k; i++) {
        int a, b; cin >> a >> b;
        mp[a - 1][b - 1] = -1;
    }
    cin >> xa >> xb >> ya >> yb;
    xa--; xb--; ya--; yb--;
    needVisited = n * m - k;
    mp[ya][yb] = -2;
    //检查除去公主与障碍后是否联通
    floorfill(xa, xb);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            if (!mp[i][j]) {
                cout << "No Solution!";
                return;
            }

    multiset<pair<int, int>> log;
    set<pair<int, int>> nowVisited;
    mp[xa][xb] = 1;
    log.insert({xa,xb});
    nowVisited.insert({xa,xb});
    dfs(xa, xb, 0, (1 << 9) - 1, 2, log, nowVisited);
    cout << minAns << endl << cntAns << endl;
    for (int i = 1; i <= path.size(); i++) {
        cout << path[i - 1] << " ";
        if (i % m == 0)cout << endl;
    }
}
int main() {
    freopen("input.txt", "r", stdin);//打开读入
    freopen("output.txt", "w", stdout);//打开写入
    solve();
    fclose(stdin);
    fclose(stdout);
    return 0;
}