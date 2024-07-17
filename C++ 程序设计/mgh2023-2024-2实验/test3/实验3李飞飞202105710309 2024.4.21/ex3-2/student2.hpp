//student2.hpp
class Student {
public:
	Student();
	Student(const char* n);
	Student(const Student& other);
	void operator=(const Student& right);
	//Student& operator=(const Student& right);
	~Student();
	char* GetName(); //获取名字
	void  ChangeName(const char* n); //改名
private:
	char* name;
};
