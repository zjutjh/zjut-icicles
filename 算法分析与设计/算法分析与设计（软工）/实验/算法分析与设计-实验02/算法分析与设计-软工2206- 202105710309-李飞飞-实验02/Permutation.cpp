#include<iostream>
#include<cstring>
#include<algorithm>
#include<stack>
#include<vector>
#include<unordered_set>
using namespace std;
#define ll long long
#define endl '\n'
const int N=1010;
ll n,m;
// unordered_set<string> st;
vector<string> st;
bool used[N];
string input;
stack<char> now;
void dfs(int cnt){
    if(cnt==n){
        string nowString;
        auto t=now;
        while(t.size()){
            nowString+=t.top();
            t.pop();
        }
        // st.insert(nowString);
        st.push_back(nowString);
        return;
    }
    unordered_set<char> sst;
    for(int i=0;i<n;i++)
        if(!used[i]&&!sst.count(input[i])){
            sst.insert(input[i]);
            used[i]=1;
            now.push(input[i]);
            dfs(cnt+1);
            used[i]=0;
            now.pop();
        }
}
int main(){
    cin>>n>>input;
    dfs(0);
    for(auto t:st)cout<<t<<endl;
    cout<<st.size();
    return 0;
}
