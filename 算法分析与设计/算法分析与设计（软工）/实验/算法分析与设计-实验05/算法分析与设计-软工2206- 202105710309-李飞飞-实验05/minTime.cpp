#include<iostream>
#include<cstring>
#include<queue>
using namespace std;
#define ll long long
#define endl '\n'
#define inf 0x3f3f3f3f
ll n,m,ans=inf;
vector<ll> q,w;
void dfs(int x){
    if(x==n){
        ll now=-inf;
        for(auto t:q)now=max(now,t);
        //因为有剪枝所以这里一定更优秀
        ans=now;
        return;
    }
    for(int i=0;i<m;i++){
        //最优性剪枝
        if(q[i]+w[x]>=ans)continue;
        q[i]+=w[x];
        dfs(x+1);
        q[i]-=w[x];
    }
}
void solve(){
    //n件事 m人
    cin>>n>>m;
    q.resize(m,0);
    w.resize(n);
    for(auto &t:w)cin>>t;
    dfs(0);
    cout<<"最短时间消耗: "<<ans;
}
int main(){
    solve();
    return 0;
}