#include <iostream>
#include <string>
class Date {
    friend std::ostream &operator<<(std::ostream &os, const Date &date);
    friend std::istream &operator>>(std::istream &is, Date &date);

public:
    Date(int year = 2000, int month = 1, int day = 1) : year(year), month(month), day(day) {}
    bool fromString(std::string str, std::string format = "%Y-%m-%d");
    std::string toString(std::string format = "y-m-d") const;

private:
    bool isValid(int y, int m, int d) const;
    int year;
    int month;
    int day;
};

inline std::ostream &operator<<(std::ostream &os, const Date &date) {
    os << date.year << "-" << (date.month < 10 ? "0" : "") << date.month << "-"
       << (date.day < 10 ? "0" : "") << date.day;
    return os;
}

inline std::istream &operator>>(std::istream &is, Date &date) {
    char sep1, sep2;
    is >> date.year >> sep1 >> date.month >> sep2 >> date.day;
    return is;
}

inline bool Date::fromString(std::string str, std::string format) {
    int tempY = 0, tempM = 0, tempD = 0;

    // 生成动态 sscanf 格式字符串
    std::string scanFormat = "";
    for (char c : format) {
        if (c == 'y' || c == 'm' || c == 'd') {
            scanFormat += "%d";
        } else {
            scanFormat += c;
        }
    }

    // 确定 y, m, d 在格式中的顺序
    int *targets[3];
    int count = 0;
    for (char c : format) {
        if (c == 'y')
            targets[count++] = &tempY;
        else if (c == 'm')
            targets[count++] = &tempM;
        else if (c == 'd')
            targets[count++] = &tempD;
    }

    if (sscanf(str.c_str(), scanFormat.c_str(), targets[0], targets[1], targets[2]) == 3) {
        if (isValid(tempY, tempM, tempD)) {
            this->year = tempY;
            this->month = tempM;
            this->day = tempD;
            return true;
        }
    }
    return false;
}

inline std::string Date::toString(std::string format) const {
    std::string result;
    for (char c : format) {
        if (c == 'y') {
            result += std::to_string(year);
        } else if (c == 'm') {
            result += (month < 10 ? "0" : "") + std::to_string(month);
        } else if (c == 'd') {
            result += (day < 10 ? "0" : "") + std::to_string(day);
        } else {
            result += c;
        }
    }
    return result;
}

// 基础日期合法性校验
inline bool Date::isValid(int y, int m, int d) const {
    if (m < 1 || m > 12 || d < 1 || d > 31)
        return false;
    if (m == 2) {
        bool isLeap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
        return d <= (isLeap ? 29 : 28);
    }
    if (m == 4 || m == 6 || m == 9 || m == 11)
        return d <= 30;
    return true;
}