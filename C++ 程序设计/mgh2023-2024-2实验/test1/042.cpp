#include<iostream>
#include <vector>
#include <algorithm>
#include <numeric>
using namespace std;
#define ll long long
#define endl '\n'
struct student{
   int id;
   char name[20];
   int scores[3];
};
void inputStu(student *stu){
    cin>>stu->id>>stu->name;
    for(int i=0;i<3;i++)
    cin>>stu->scores[i];
}
void inputStu(student &stu){
    cin>>stu.id;
    cin>>stu.name;
    for(int i=0;i<3;i++)
    cin>>stu.scores[i];
}
void outputStu(const student stu){
    cout<<"id:"<<stu.id<<"  学生姓名："<<stu.name<<endl;
    cout<<"三门课成绩：";
    cout<<stu.scores[0]<<" "<<stu.scores[1]<<" "<<stu.scores[2]<<endl;
}
void outputStu(const student *stu){
    cout<<"id:"<<stu->id<<"  学生姓名："<<stu->name<<endl;
    cout<<"三门课成绩：";
    cout<<stu->scores[0]<<" "<<stu->scores[1]<<" "<<stu->scores[2]<<endl;
}
void outputStu(const student &stu){
    cout<<"id:"<<stu.id<<"  学生姓名："<<stu.name<<endl;
    cout<<"三门课成绩：";
    cout<<stu.scores[0]<<" "<<stu.scores[1]<<" "<<stu.scores[2]<<endl;
}
//第一个是最高分获得者 后面三个分别是三门课最高分
//引用形参传vector,里面装的对象
vector<student*> fun(vector<student> &q){
    //分别表示高分获得者 三门课最高分获得者
    student *ma=nullptr;
    student *ma1=nullptr,*ma2=nullptr,*ma3=nullptr;
    //引用形参操作每个对象
    for(auto &t:q){
        int sum=accumulate(t.scores,t.scores+3,0);
        if(ma==nullptr||sum>accumulate(ma->scores,ma->scores+3,0))
            ma=&t;
        if(ma1==nullptr||ma1->scores[0]<t.scores[0])
            ma1=&t;
        if(ma2==nullptr||ma2->scores[1]<t.scores[1])
            ma2=&t;
        if(ma3==nullptr||ma3->scores[2]<t.scores[2])
            ma3=&t;
    }
    vector<student*> ans({ma,ma1,ma2,ma3});
    return ans;
}
void solve(){
    const int n=3;
    vector<student> aclass(n);
    for(auto &t:aclass)
        inputStu(&t);
    // for(auto &t:aclass)
    //     outputStu(&t);
    auto result=fun(aclass);
    for(auto t:result)
        outputStu(t);
}
int main(){
    solve();
    return 0;
}