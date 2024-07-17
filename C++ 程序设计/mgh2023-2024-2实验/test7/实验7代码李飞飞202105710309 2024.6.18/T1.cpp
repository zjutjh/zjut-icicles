#include<iostream>
using namespace std;
class A { //基类
public:
   A(int v1=0,int v2=0,int v3=0):a(v1),b(v2),c(v3){ }
   void F1(){cout<< "F1"<<a<<" "<<b<<" "<<c<<endl;}
   int a;
protected:
   void F2( ) {cout<<"F2"<<a<<" "<<b<<" "<<c<<endl;}
   int b;
private:
   void F3(){cout<<"F3"<<a<<" "<<b<<" "<<c<<endl;}
   int c;
};

class B: public A{
public:
   //B的构造函数缺失
   B(int v1=0,int v2=0,int v3=0,int v4=0,int v5=0,int v6=0):A(v1,v2,v3),Ba(v4),Bb(v5),Bc(v6){}
   void F4( ) {cout<<"F4"<<Ba<<" "<<Bb<<" "<<Bc<<endl;}
   int Ba;
protected:
   void F5( ) {cout<<"F5"<<Ba<<" "<<Bb<<" "<<Bc<<endl;}
   int Bb;
private:
   void F6(){cout<<"F6"<<Ba<<" "<<Bb<<" "<<Bc<<endl;}
   int Bc;
};
class C: protected  B{
public:
   //C的构造函数缺失
   C(int v1=0,int v2=0,int v3=0,int v4=0,int v5=0,int v6=0,int v7=0,int v8=0):B(v1,v2,v3,v4,v5,v6),Ca(v7),Cb(v8){}
   void F7(){cout<<"F7"<<Ba<<" "<<Bb <<endl;}
   void F8(){cout<<"F8"<<Ca<<" "<<Cb <<endl;}
   int Ca;
private:
   int Cb;
};

//测试主函数
int main()
{
   A Aobj1, Aobj2(1,2,3);
   B Bobj1,Bobj2(1,2,3,4,5);
   C Cobj1,Cobj2(1,2,3,4,5,6);
   
   // 展示类A的成员
   	cout<<"Aobj1:\n";
   	Aobj1.F1();
   	cout<<"Aobj1.a "<<Aobj1.a<<endl;
   	cout<<"Aobj2:\n";
   	Aobj2.F1();
    cout<<"Aobj2.a "<<Aobj2.a<<endl;
    
   // 展示类B的成员
   cout<<"Bobj1:\n";
   Bobj1.F1();
   Bobj1.F4();
   cout<<"Bobj1.a "<<Bobj1.a<<endl;
   cout<<"Bobj1.Ba "<<Bobj1.Ba<<endl;
   cout<<"Bobj2:\n";
   Bobj2.F1();
   Bobj2.F4();
   cout<<"Bobj2.a "<<Bobj2.a<<endl;
   cout<<"Bobj2.Ba "<<Bobj2.Ba<<endl;
    
	// 展示类C的成员
	cout<<"Cobj1:\n";
	Cobj1.F7();
  	Cobj1.F8();
  	cout<<"Cobj1.Ca: "<<Cobj1.Ca<<endl;
	cout<<"Cobj2:\n";
	Cobj2.F7();
	Cobj2.F8();
	cout<<"Cobj2.Ca: "<<Cobj2.Ca<<endl;

   
   return 0;
}

