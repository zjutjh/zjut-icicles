#pragma once
//DateTime类声明,即为课程材料中的clock类
class  DateTime {
public:
    void showDateTime();
    void setDateTime();
    double diff(const DateTime& T); //注意常引用对形参T的约束
private:
    long normalize() const; //注意const在此的必要性
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
};
