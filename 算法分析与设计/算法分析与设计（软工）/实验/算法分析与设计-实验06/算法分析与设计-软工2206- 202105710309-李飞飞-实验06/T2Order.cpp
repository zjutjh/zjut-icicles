#include<iostream>
#include<algorithm>
#include<queue>
using namespace std;
#define ll long long
#define endl '\n'
#define pb push_back
ll n,m;
vector<int> time;
double s;
int best=1e9;
struct node{
    //当前在搜索的任务
    int k;
    //最大等待时间
    int cost;
    //每个机器实际代价
    vector<int> q;
    //下界 最大等待时间＋剩余任务总时间/m
    double down;
    double leftTotalTime;
    //为了优先队列从小到大，这里反转比较符
    bool operator <(const node other)const{
        return down>other.down;
    }
};
node get(node &x,int p){
    node now=x;
    //更新要下次要搜索的位置
    now.k=x.k+1;
    //将任务k给p号人
    now.q[p]+=time[x.k];
    //更新实际代价
    now.cost=max(now.cost,now.q[p]);
    //更新总时间
    now.leftTotalTime-=time[x.k];
    //更新下界
    now.down=now.leftTotalTime/m;
    return now;
};
void bfs(){
    priority_queue<node> q;
    vector<int> st(m);
    q.push({0,0,st,s/m,s});
    while (q.size()){
        auto t=q.top();
        q.pop();
        //下界大于当前最优，直接剪枝
        if(t.down>best)continue;
        //从0个人开始试图填充任务k
        for(int i=0;i<m;i++){
            auto now=get(t,i);
            //如果时叶子节点则更新全局答案，不入队
            if(now.k==n){
                best=min(best,now.cost);
            }
            //下界小于最优，入队尝试更新
            else if(now.down<best)q.push(now);
        }
    }
    
}
void solve(){
    cin>>n>>m;
    time.resize(n);
    for(int i=0;i<n;i++){
        cin>>time[i];
        s+=time[i];
    }
    bfs();
    cout<<best;
}
int main(){
    solve();
    return 0;
}