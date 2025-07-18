#include<iostream>
#include<algorithm>
#include<queue>
using namespace std;
#define ll long long
#define endl '\n'
#define pb push_back
#define fast ios::sync_with_stdio(false);cin.tie(0);cout.tie(0);
const int N=5e5+10;
ll n,m;
int ma,maCnt,serachRound; //最大约数,最大约数个数
vector<ll> ans;
int primes[9] = {2, 3, 5, 7, 11, 13, 17, 19, 23};
//当前素数编号 上个素数的个数 当前数 约数之和
void dfs(int u, int last, int p, int sum) {
    serachRound++;
    if(p>=m&&(sum>maCnt||(sum==maCnt))){
        if(sum>maCnt){
            maCnt=sum;
            ans.clear();
            ans.pb(p);
        }
        else ans.pb(p);
    }
    //可行性剪枝
    if (u==9) return;
    //最优性剪枝,不得高于上一个的频次
    for (int i = 1; i <= last; ++ i) {
        if (1ll*p * primes[u] > n) return;
        p *= primes[u];
        dfs(u + 1, i, p, sum * (i + 1));
    }
}
void solve(){
    cin>>m>>n;
    dfs(0, 30, 1, 1);
    cout<<"程序一共搜索了"<<serachRound<<"轮"<<endl;
    for(auto t:ans)cout<<t<<" ";
}
int main(){
    freopen("input.txt", "r", stdin);//打开读入
    freopen("output.txt", "w", stdout);//打开写入
    // fast
    solve();
    fclose(stdin);
    fclose(stdout);
   return 0;
}
