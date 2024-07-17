#include<iostream>
#include<cstring>
#include<algorithm>
#include<cmath>
#include<queue>
using namespace std;
#define ll long long
#define endl '\n'
#define pb push_back
const int N=5e5+10;
ll n,m;
int d[N],st[N];
int e[N],ne[N],h[N],idx,w[N];
void add(int a,int b,int k){
    e[idx]=b,ne[idx]=h[a],h[a]=idx++;
    w[idx-1]=k;
}
void spfa(){
    memset(d,0x3f,sizeof d);
    int q[N],hh=0,tt=0;
    q[0]=0,d[0]=0;
    while(hh<=tt){
        int t=q[hh++];
        st[t]=0;//出队
        for(int i=h[t];i!=-1;i=ne[i]){
            int j=e[i];
            if(d[j]>d[t]+w[i]){//更新
                d[j]=d[t]+w[i];
                if(!st[j]){//入队
                    q[++tt]=j;
                    st[j]=1;
                }
            }
        }
    }
}
void spfaSolve(){
    cin>>n;
    memset(h,-1,sizeof(h));
    for(int i=0;i<n;i++)
        for(int j=i+1;j<n;j++){
            int t;cin>>t;
            add(i,j,t);
        }
    spfa();
    cout<<"最小代价: "<<d[n-1];
}
void solve(){
    cin>>n;
    //初始化代价q[i][j]为i上j下的代价
    //f[i]为0~i最小代价
    vector<int> q[n],f(n,1e9);
    f[0]=0;
    for(int i=0;i<n;i++){
        q[i].reserve(n);
        for(int j=i+1;j<n;j++)
        cin>>q[i][j];
    }
    for(int i=0;i<n;i++)
        for(int j=0;j<i;j++)
            f[i]=min(f[i],f[j]+q[j][i]);
    cout<<f[n-1];
}
int main(){
    freopen("input.txt", "r", stdin);//打开读入
    freopen("output.txt", "w", stdout);//打开写入
    spfaSolve();
    fclose(stdin);
    fclose(stdout);
    return 0;
}