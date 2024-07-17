//Time类声明,即为课程材料中的clock类
class  Time{
public:
    void showTime();
    void setTime();
   double diff(const Time& T); //注意常引用对形参T的约束
private:
    long normalize() const; //注意const在此的必要性
    int hour;
    int minute;
    int second;
};
