#include<iostream>
#include<cstring>
#define ll long long
using namespace std;
const int N=310;
int n;
ll s[N],f[N][N],g[N][N];
int main(){
    freopen("input.txt", "r", stdin);//打开读入
    freopen("output.txt", "w", stdout);//打开写入
    cin>>n;
    //前缀和
    for(int i=1;i<=n;i++){
        cin>>s[i];
        s[i]+=s[i-1];
    }
    memset(f,0x3f,sizeof f);
    for(int len=1;len<=n;len++){
        for(int l=1;l+len-1<=n;l++){
            int r=l+len-1;
            if(len==1){
                f[l][r]=g[l][r]=0;
                continue;
            }
            for(int i=l;i<r;i++){
                f[l][r]=min(f[l][r],f[l][i]+f[i+1][r]+s[r]-s[l-1]);
                g[l][r]=max(g[l][r],g[l][i]+g[i+1][r]+s[r]-s[l-1]);
            }
        }
    }
    cout<<"最大分数: "<<g[1][n]<<endl;
    cout<<"最小分数: "<<f[1][n];
    fclose(stdin);
    fclose(stdout);
    return 0;
}