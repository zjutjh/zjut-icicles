#include<iostream>
using namespace std;
#define ll long long
#define endl '\n'
void swap1(int a,int b){
    //无法交换
}
void swap2(int &a,int &b){
    int t=a;
    a=b;b=t;
}
void swap3(int *a,int *b){
    int t=*a;
    *a=*b;
    *b=t;
}
void solve(){
    int a,b;
    cin>>a>>b;
    cout<<a<<" "<<b<<endl;
    swap3(&a,&b);
    cout<<a<<" "<<b;
}
int main(){
    solve();
    return 0;
}