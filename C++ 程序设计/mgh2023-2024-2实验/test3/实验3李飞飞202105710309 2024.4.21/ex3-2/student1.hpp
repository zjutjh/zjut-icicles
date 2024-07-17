//student1.hpp
class Student {
public:
	Student();//考虑这里为什么不缺省无参构造?
	Student(const char* n);
	//Student(const Student & other);
	//Student& operator=(const Student& right);  
	//~Student( );
	const char* GetName();  //获取名字
	void  ChangeName(const char* n);  //改名
private:
	char name[20];
};
