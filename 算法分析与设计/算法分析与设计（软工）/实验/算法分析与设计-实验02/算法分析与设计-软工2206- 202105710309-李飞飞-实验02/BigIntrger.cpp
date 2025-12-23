#include<iostream>
#include<vector>
#include<algorithm>
#include<cstring>
using namespace std;
#define ll long long
#define endl '\n'
#define pb push_back
#define all(v) v.begin(), v.end()
//高精度
bool cmp(vector<int> &a,vector<int> &b){
    if(a.size()!=b.size())return a.size()>b.size();//位数大直接返回
    for(int i=a.size()-1;i>=0;i--)
        if(a[i]!=b[i])return a[i]>b[i];//逐一比较大小
    return true;//相等
}
vector<int> sub(vector<int> &a,vector<int> &b){
    int t=0;
    vector<int> ans;
    for(int i=0;i<a.size();i++){
        t+=a[i];
        if(i<b.size())t-=b[i];
        ans.push_back((t+10)%10);
        if(t<0)t=-1;
        else t=0;
    }
    while(ans.size()>1&&!ans.back())ans.pop_back();
    return ans;
}
vector<int> add(vector<int> a,vector<int> b){
    vector<int> ans;
    int t=0,k=max(a.size(),b.size());
    for(int i=0;i<k;i++){
        if(a.size()>i)t+=a[i];
        if(b.size()>i)t+=b[i];
        ans.push_back(t%10);
        t/=10;
    }
    if(t)ans.push_back(t);
    return ans;
}
vector<int> mul(vector<int> &a,int b){
    int t=0;
    vector<int> ans;
    for(int i=0;i<a.size();i++){
        t+=b*a[i];
        ans.push_back(t%10);
        t/=10;
    }
    if(t) ans.push_back(t);
    while(ans.size()>1&&!ans.back())ans.pop_back();
    return ans;
}
vector<int> div(vector<int> &a,int b,int &r){//a被除数 b除数 r余数
    vector<int> ans;
    for(int i=a.size()-1;i>=0;i--){
        r=r*10+a[i];
        ans.push_back(r/b);
        r%=b;
    }
    reverse(ans.begin(),ans.end());
    while(ans.size()>1&&!ans.back())ans.pop_back();
    return ans;
}

class Integer{
private:
public:
    bool isPositive;
    vector<int> value;
    int pow;
    Integer(vector<int> value);
    Integer(string value);
    ~Integer();

    //转为int
    int intify(vector<int> q)
    //int转为Integer
    Integer integerify(int x);

    //改变符号
    Integer modiIsPositive();
    //提升幂次 基础幂次次数 幂次的幂次次数
    void improvePow(int base,int pow);
    
    //乘一个int
    Integer intMultiply(int x);
    //除一个int
    Integer intDivision(int x);

    //统一幂次
    void alignPow(Integer &x);
    //加一个大整数(可以负数)
    Integer conquerAdd(Integer x);
    //两个数三段分治乘法
    Integer divideMultiply(Integer a,Integer b);
};


//乘一个int
Integer Integer::intMultiply(int x){
    Integer ans;
    //处理幂次与符号
    ans.pow=this->pow;
    ans.isPositive=this->isPositive;
    if(x<0)ans.modiIsPositive();
    //有效数值
    ans.value=mul(this->value,x);
    return ans;
}
//除一个int,保证够除(位数小的分治时直接转int算)
Integer Integer::intDivision(int x){
    Integer ans;
    ans.pow=pow;
    ans.isPositive=isPositive;
    ans.value=div(value,x,1);
    return ans;
}


//统一幂次
void Integer::alignPow(Integer &x){
    if(pow==x.pow)return;
    if(pow>x.pow){
        int t=pow-x.pow;
        vector<int> k(t,0);
        value=k+value;
        pow-=t;
    }
    else{
        int t=x.pow-pow;
        vector<int> k(t,0);
        x.value=k+x.value;
        x.pow-=t;
    }
}
// 加一个大整数(可以负数)
Integer Integer::conquerAdd(Integer x){
    Integer ans;
    alignPow(x);
    ans.pow=pow;
    //同正负
    if(this->isPositive==x.isPositive){
        ans.value=add(value,x.value);
        ans.isPositive=isPositive;
    }
    else{
        if(cmp(value,x.value)){
            ans.value=sub(value,x.value);
            if(isPositive)ans.isPositive=true;
            else ans.isPositive=false;
        }
        else{
            ans.value=sub(x.value,value);
            if(x.isPositive)ans.isPositive=true;
            else ans.isPositive=false;
        }
    }
    return ans;
}
//两个数三段分治乘法
Integer Integer::divideMultiply(Integer a,Integer b){
    if(a.)
}


//改变符号
Integer Integer::modiIsPositive(){
    this->isPositive=!this->isPositive;
}
//提升幂次 基础幂次次数 幂次的幂次次数
void Integer::improvePow(int base,int pow){
    this.pow+=base*pow;
}

//转化类型
int Integer::intify(vector<int> q){
    int ans=0;
    reverse(all(q));
    for(auto t:q){
        ans*=10;
        ans+=(t-'0');
    }
    return ans;
}
Integer Integer::integerify(int x){
    Integer ans;
    ans.pow=0;
    ans.isPositive=true;
    if(x<0){
        ans.modiIsPositive();
        x*=-1;
    }
    while(x){
        ans.value.pb(x%10);
        x/=10;
    }
    return ans;
}


//构造与析构
Integer::Integer(vector<int> value){
    this->value=value;
}
Integer::Integer(string value){
    for(int i=value.size()-1;i>=0;i--)
        this->value.push_back(value[i]-'0');
}
Integer::~Integer(){
}
