#pragma once
#include "Date.hpp"
#include <ctime>
#include <iostream>
#include <string>

enum class EducationalLevel : int {
    Unknown,
    None,
    Primary,
    Junior,
    Secondary,
    Vocational,
    Associate,
    Bachelor,
    Master,
    Doctorate,
    Postdoctorate
};

struct Employee {
    std::string id;
    std::string name;
    EducationalLevel eduLevel;
    Date birthday;
    std::string ethnicity;
    std::string major;
    std::string jobTitle;
    std::string department;
    std::string positionTitle;
};

inline std::ostream &operator<<(std::ostream &os, const Employee &e) {
    char sep = '\x1e';
    os << (e.id.size()) << sep << e.id << sep
       << (e.name.size()) << sep << e.name << sep
       << static_cast<int>(e.eduLevel) << sep
       << e.birthday << sep
       << (e.ethnicity.size()) << sep << e.ethnicity << sep
       << (e.major.size()) << sep << e.major << sep
       << (e.jobTitle.size()) << sep << e.jobTitle << sep
       << (e.department.size()) << sep << e.department << sep
       << (e.positionTitle.size()) << sep << e.positionTitle << sep;
    return os;
}

inline void readStringBlock(std::istream &is, std::string &str) {
    size_t nextReadSize;
    char drop;
    is >> nextReadSize;
    str.resize(nextReadSize);
    is >> drop;
    is.read(&str[0], nextReadSize);
    is >> drop;
}

inline std::istream &operator>>(std::istream &is, Employee &e) {
    size_t nextReadSize;
    char drop;
    readStringBlock(is, e.id);
    readStringBlock(is, e.name);
    is >> reinterpret_cast<int &>(e.eduLevel);
    is >> drop;
    is >> e.birthday;
    is >> drop;
    readStringBlock(is, e.ethnicity);
    readStringBlock(is, e.major);
    readStringBlock(is, e.jobTitle);
    readStringBlock(is, e.department);
    readStringBlock(is, e.positionTitle);
    return is;
}
