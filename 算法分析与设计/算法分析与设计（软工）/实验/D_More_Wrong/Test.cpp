#include<iostream>
#include<cstring>
#include<algorithm>
#include<vector>
#include<set>
using namespace std;
#define ll long long
#define endl '\n'
#define pb push_back
#define inf 0x3f3f3f3f
const int N=1e5+10;
ll n,m;
int h=-1,e[N],r[N],l[N],idx=2;
int w[N];
//将w的x号元素加入k右边
//0和1为左右哨兵
void addr(int k,int x){
    e[idx]=x,r[idx]=r[k],l[idx]=k,l[r[k]]=idx,r[k]=idx++;
}
void addl(int k,int x){
    addr(l[k],x);
}
void remove(int k){
    r[l[k]]=r[k];
    l[r[k]]=l[k];
}
void show(){
    for(int i=r[0];i!=1;i=r[i])
        cout<<e[i]<<" ";
    cout<<endl;
}
void init(int x){
    w[0]=-inf;w[1]=inf;
    r[0]=1,l[1]=0;
    //随机初始化w
    for(int i=2;i<x+2;i++)
        addr(0,rand()%1000);
    cout<<"随机的初始化数组:\n";
    show();
    sort(e+2,e+x+2,[](int x,int y){
        return x>y;
    });
    cout<<"排序后的链表:\n";
    show();
}
//随机k个数中最接近x的数
int Shewood(int x){
    int k=sqrt(n);
    set<int> st;
    while(st.size()<k)
        //每次随机插入一个下标用于寻找,直至有k个不同的
        st.insert(rand()%n+2);
    int now=inf,p=-1;
    //找一个最接近的小标返回
    for(auto t:st)
        if(abs(e[t]-x)<now){
            now=abs(e[t]-x);
            p=t;
        }
    return p;
}
int findNext(int x){
    int begin=Shewood(x);
    if(e[begin]<x){
        while(e[begin]<x)
            begin=r[begin];
    }
    else{
        while(e[begin]>x)
            begin=l[begin];
    }
    return e[r[begin]];
}
int findPre(int x){
    int begin=Shewood(x);
    if(e[begin]<x){
        while(e[begin]<x)
            begin=r[begin];
    }
    else{
        while(e[begin]>x)
            begin=l[begin];
    }
    return e[l[begin]];
}
int findMin(){
    //最接近0且不能找到哨兵
    int begin=Shewood(0);
    while(l[begin]!=0)
        begin=l[begin];
    return e[begin];
}
int findMax(){
    //最接近元素上界且不能找到哨兵
    int begin=Shewood(1000);
    while(r[begin]!=1)
        begin=r[begin];
    return e[begin];
}
void solve(){
    cin>>n;
    //初始化
    init(n);
    //———————Shewood算法———————//
    //随机挑选一个数来找前驱后继
    int t=rand()%n+2;
    cout<<e[t]<<"的前驱"<<findPre(e[t])<<endl;
    cout<<e[t]<<"的后继"<<findNext(e[t])<<endl;
    cout<<"最小值"<<findMin()<<endl;
    cout<<"最大值"<<findMax()<<endl;
}
int main(){
    solve();
    return 0;
}