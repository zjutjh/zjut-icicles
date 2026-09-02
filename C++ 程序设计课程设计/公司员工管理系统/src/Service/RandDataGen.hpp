#pragma once
#include "../Model/Employee.hpp"
#include <vector>
class RandomDataGenerator {
public:
    static std::vector<Employee> generateRandomEmployees(int count);

private:
    static const std::vector<std::string> firstNames;
    static const std::vector<std::string> lastNames;
    static const std::vector<std::string> ethnicities;
    static const std::vector<std::string> majors;
    static const std::vector<std::string> jobTitles;
    static const std::vector<std::string> departments;
    static const std::vector<std::string> positionTitles;
};

inline const std::vector<std::string> RandomDataGenerator::firstNames = {
    "张", "李", "王", "赵", "刘", "陈", "杨", "黄", "上官", "欧阳"};
inline const std::vector<std::string> RandomDataGenerator::lastNames = {
    "伟", "芳", "书灵", "锦浩", "辰希", "怀清", "东臻", "梓雯", "凯霖", "宸旭"};
inline const std::vector<std::string> RandomDataGenerator::ethnicities = {
    "汉族", "汉族", "汉族", "汉族", "汉族", "维吾尔族", "土家族", "彝族", "蒙古族", "藏族"};
inline const std::vector<std::string> RandomDataGenerator::majors = {
    "计算机科学与技术", "软件工程", "信息安全", "电子信息工程", "通信工程", "自动化", "土木工程", "建筑学", "环境设计"};
inline const std::vector<std::string> RandomDataGenerator::jobTitles = {
    "助理工程师", "工程师", "高级工程师", "教授级高级工程师"};
inline const std::vector<std::string> RandomDataGenerator::departments = {
    "人事部", "技术部", "后勤部", "信息部"};
inline const std::vector<std::string> RandomDataGenerator::positionTitles = {
    "普通员工", "工程师", "团队领导", "部门领导", "公司领导"};