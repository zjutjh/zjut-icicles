#include <iostream>
#include <cstring>
using namespace std;
class Name{
public:
	//1)根据使用给出构造函数的声明
	Name(const char *s="X");
	//2)给出拷贝构造函数的声明
	Name(const Name& obj);
	//3)给出赋值重载函数的声明
	Name& operator=(const Name& obj);
	//4)给出下标重载声明
	char& operator[](int index);
	//5)给出姓名等于比较重载声明
	bool operator==(const Name& obj) const;
	//6)给出求平均姓名长度的成员函数AveLen声明
	static double AveLen();
	//7)给出输入流重载的友元声明
	friend istream& operator>>(istream& in,Name& obj);
	//8)给出其他必要的成员函数声明
	~Name();
	operator char* ();
	friend ostream& operator<<(ostream& out,const Name& obj);
	
	void display(ostream& out) const
	{    out<<name<<endl;   }
	
	static int count; //姓名个数统计
	static int sum;
private:
    char *name;
    int size;   
};

istream& operator>>(istream& in,Name& obj);
//9)给出输出流重载声明
ostream& operator<<(ostream& out,const Name& obj);

//10)给出count初始化定义
int Name::count=0;
int Name::sum=0;

int main(){
    Name per1("Chen Jiaqi"),per2("Han Ying"),per3(per1),per4; 
    //默认的名字为X
    Name *person=new Name("Mike Li");
    cout<<per1<<","<<per2<<","<<per3<<","<<per4<<endl;
    per4=(*person);
    cout<<per4<<endl;
    cin>>per4;
    cout<<per4<<endl;
    cout<<Name::count<<endl;
    cout<<Name::sum<<endl;
    cout<<Name::AveLen()<<endl;
    per2[4]='P';
    cout<<per2<<endl;
    if(per1==per3)
    cout<<"equal"<<endl; 
    else 
    cout<<"not equal"<<endl;
    if(strcmp("Chen Jiami",per3)>=0) 
    cout<<"Bigger"<<endl;
    else 
    cout<<"Smaller"<<endl;
    delete person;
    cout<<Name::AveLen()<<endl;
    return 0;
}

//1)构造函数
Name::Name(const char *s){
	name = new char[strlen(s)+1];
	size = strlen(s);
	strcpy(name,s);
	count++;
    sum += size;
}


//2)给出拷贝构造函数的声明
Name::Name(const Name& obj){
    this->name=new char[obj.size+1];
    strcpy(this->name,obj.name);
    this->size=obj.size;
    count++;
    sum += size;
}
//3)给出赋值重载函数的声明
Name& Name::operator=(const Name& obj){
	sum -= size;
    delete[] name;
    name=new char[obj.size+1];
    strcpy(name,obj.name);
    size=obj.size; 
    sum += size;
    return *this;
}
//4)给出下标重载声明
char& Name::operator[](int index){
    if(index<0||index>=size)
    {
    	cout<<"下标越界\n";
    	exit(1);
	}
    else 
    	return name[index];
}
//5)给出姓名等于比较重载声明
bool Name::operator==(const Name& obj) const{
    return strcmp(this->name,obj.name)==0;
}
//6)给出求平均姓名长度的成员函数AveLen声明
double Name::AveLen(){
    return 1.0*sum/count;
}
//7)给出输入流重载的友元声明
istream& operator>>(istream& in,Name& obj){
    char *temp=new char[128];
    in.getline(temp,128);
    delete[] obj.name;
    Name::sum -= obj.size;
    
    obj.size=strlen(temp);
    obj.name=new char[obj.size+1];
    strcpy(obj.name,temp);
    Name::sum += obj.size;
    return in;
}
//8)给出其他必要的成员函数声明
Name::~Name(){
	sum -= size;
    delete[] name;
    count--;
}
Name::operator char* ()
{
	return name;
}

ostream& operator<<(ostream& out,const Name& obj){
    out<<obj.name;
    return out;
}
