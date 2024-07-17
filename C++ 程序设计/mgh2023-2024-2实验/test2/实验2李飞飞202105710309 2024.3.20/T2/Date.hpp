//Date类声明,即为课程材料中的clock类
class  Date{
public:
    void showDate();
    void setDate();
   double diff(const Date& T); //注意常引用对形参T的约束
private:
    long normalize() const; //注意const在此的必要性
    int year;
    int month;
    int day;
};
