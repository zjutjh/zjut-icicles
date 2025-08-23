#include<iostream>
#include<vector>
#include<algorithm>
using namespace std;
int n,m;
int main(){
    cin>>n>>m;
    vector<pair<int,int>> q(n);
    for(auto &t:q){
        int a,b;
        cin>>a>>b;
        t={a,b};
    }
    sort(q.begin(),q.end(),[](pair<int,int> &a,pair<int,int> &b)->bool{
        return 1.0*a.second/a.first>1.0*b.second/b.first;
    });
    cout<<"排序后结果: \n";
    for(auto t:q)
        cout<<"体积: "<<t.first<<" 价值: "<<t.second<<endl;
    double value=0;
    for(auto t:q){
        if(m>=t.first){
            m-=t.first;value+=t.second;
        }
        else{
            value+=1.0*t.second/t.first*m;
            break;
        }
    }
    cout<<"最优大价值: "<<value;
    return 0;
}