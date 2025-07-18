#include<iostream>
#include<string>
#include<queue>
using namespace std;
int n,m;
int main(){
    string num;
    cin>>num>>m;
    n=num.length();
    vector<int> q(n+1);
    for(int i=0;i<n;i++)
        q[i+1]=num[i]-'0';
    //需要保留的总数 当前坐标
    int rest=n-m,t=1;
    bool flag=0;
    while(rest--){
        int p=t;
        //保证剩余的数还够
        for(int i=t;i<=t+m;++i)
            if(q[p]>q[i]) p=i;
        //已经不用考虑前导0
        if(q[p]) flag=1;
        if(flag) cout<<q[p];
        m-=p-t; t=p+1;
    }
    if(!flag)cout<<0;
    return 0;
}