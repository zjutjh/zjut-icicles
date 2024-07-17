#pragma once
#include"Course.hpp"
using namespace std;
class CourseList{
    public:
        CourseList();
        virtual ~CourseList();
        //尾插法
        void add(Course &cou);
        //按编号查找
        Course *findByCourseId(int id);
        //按名字查找
        void findByCourseName(string n);
        //按院系查找
        void findByCourseDepartment(string f);
        //按教师名字查找
        void findByLecturer(string t);
        //查找
        void findCourseMenu();
        //展示所有课程信息
        void show();
        //写入文件
        void write();
        //按学分排序
        void sortByCourseCredit();
        //按开课学院排序
        void sortByCourseDepartment();

        void sortCourseMenu();//排序
        //修改信息
        void updateCourseInfo(Course *target);
        //删除课程
        void removeCourse(int id);
    protected:
        Course *head;
        int size;

};
