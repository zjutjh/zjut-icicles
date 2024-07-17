#include<iostream>
#include<algorithm>
#include<queue>
using namespace std;
const int N=5e5+10;
int n,m;
void solve(){
    cin>>n>>m;
    vector<int> q(n),t(m,0);
    for(auto &t:q)cin>>t;
    sort(q.begin(),q.end());
    double ans=0;
    for(int i=0;i<n;i++){
        t[i%m]+=q[i];
        ans+=t[i%m];
    }
    cout<<ans/n;
}
int main(){
   int T=1;
   while(T--) solve();
   return 0;
}