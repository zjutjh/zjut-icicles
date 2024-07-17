#include<iostream>
#include<cstring>
#include<algorithm>
#include<queue>
using namespace std;
#define ll long long
#define endl '\n'
const int N=1010;
ll n,m,f[N];
int main(){
    cin>>n;
    f[0]=1;
    //等价于从体积为1~n的物品无限选,恰好填满容量n的完全背包
    //复杂度N^2
    for(int i=1;i<=n;i++)
        for(int j=i;j<=n;j++)
            f[j]=(f[j]+f[j-i]);
            
    cout<<f[n];
    return 0;
}
