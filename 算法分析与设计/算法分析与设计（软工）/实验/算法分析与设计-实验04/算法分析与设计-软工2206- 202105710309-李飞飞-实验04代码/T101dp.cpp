#include<iostream>
using namespace std;
const int N=1010;
int n,m,f[N];
int main(){
    cin>>n>>m;
    while(n--){
        int v,w;
        cin>>v>>w;
        for(int i=m;i>=v;i--)
            f[i]=max(f[i],f[i-v]+w);
    }
    cout<<f[m];
    return 0;
}