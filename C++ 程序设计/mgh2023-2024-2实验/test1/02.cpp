#include<iostream>
#include<algorithm>
#include<unordered_set>
#include<ctime>
#include<cstdlib>
using namespace std;
#define ll long long
#define endl '\n'
#define fast ios::sync_with_stdio(false);cin.tie(0);cout.tie(0);
ll n,m;
int *p;
int get(int &k){
    int ans=0;
    ans+=k%10;
    ans+=k/100%10*10;
    return ans;
}
//初始化数组为n个不同元素
void InitArray(int n){
    p=new int[n];
    unordered_set<int> st;
    while (st.size()!=n){
        int k=rand();
        st.insert(k);
    }
    int now=0;
    for(auto t:st)
        p[now++]=t;
}
//打印数组
void Display(int *p,int n){
    for(int i=0;i<n;i++){
        cout<<p[i]<<" ";
        if(n%20&&n)cout<<endl;
    }
}
//排序数组
void SortArray(int *p,int n){
    sort(p,p+n,[](int a,int b){
        int sa=get(a),sb=get(b);
        return sa>sb;
    });
}
//销毁数组
void DestroyArray(int *p){
    delete []p;
    p=nullptr;
}
void solve(){
    cin>>n;
    p=new int[n];
    for(int i=0;i<n;i++)cin>>p[i];
    SortArray(p,n);
    Display(p,n);
    DestroyArray(p);
}
int main(){
    srand(time(0));
    fast
    int T=1;
    cin>>T;
    while(T--) solve();
    return 0;
}