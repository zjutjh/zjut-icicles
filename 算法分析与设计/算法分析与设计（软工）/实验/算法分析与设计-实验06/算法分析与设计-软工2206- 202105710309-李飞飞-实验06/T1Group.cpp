#include<iostream>
#include<cstring>
#include<algorithm>
#include<cmath>
#include<queue>
using namespace std;
#define ll long long
#define endl '\n'
#define cnt1(x) __builtin_popcount(x)
const int N = 5e3 + 10;
int n, m;
bool mp[N][N];
ll best = -1e9, ans;
struct node {
    //在搜第x个点
    int x;
    //当前状态压缩后已经选取的点的情况
    ll now;
    //上界
    int up;
    //优先队列根据上界比较
    bool operator <(const node other)const {
        return up < other.up;
    }
};
void bfs() {
    priority_queue<node> q;
    //初始上界为全集
    q.push({ 0,0,n });
    while (q.size()) {
        auto t = q.top(); q.pop();
        //最优性剪枝 上界小于最优
        if (t.up <= best)continue;
        //如果是叶子节点
        if (t.x == n) {
            if (cnt1(t.now) > best) {
                ans = t.now;
                best = cnt1(t.now);
            }
            continue;
        }
        //不选t.x情况
        node st = t;
        st.x++; st.up--;
        q.push(st);
        //选了t.x的情况：上界不变,x++,now&上当前位置并判断合法性
        st = t;
        st.x++; st.now|=(1 << t.x);
        bool flag = true;
        for (int i = 0; i < n; i++) {
            if ((t.now >> i & 1) && !mp[i][t.x]) {
                flag = false;
                break;
            }
        }
        if (flag)q.push(st);
    }

}
void solve() {
    cin >> n >> m;
    for (int i = 0; i < m; i++) {
        int a, b; cin >> a >> b;
        a--; b--;
        mp[a][b] = mp[b][a] = 1;
    }
    bfs();
    cout << "最大团有: " << best << "个节点\n";
    cout<<"分别为: ";
    int t = 1;
    while (ans) {
        if (ans & 1)cout << t << " ";
        ans >>= 1;
        t++;
    }
}
int main() {
    solve();
    return 0;
}