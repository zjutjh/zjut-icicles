// Book.cpp

#include "Book.h"

// 序列化方法实现
std::string Book::serialize() const {
    return bookID + "," + title + "," + author + "," + publishDate + "," + (isBorrowed ? "1" : "0");
}

// 反序列化方法实现
Book Book::deserialize(const std::string& data) {
    std::string id, title, author, date;
    bool borrowed = false;
    size_t pos = 0, prev = 0;
    int field = 0;

    while ((pos = data.find(',', prev)) != std::string::npos) {
        std::string token = data.substr(prev, pos - prev);
        switch (field++) {
        case 0: id = token; break;
        case 1: title = token; break;
        case 2: author = token; break;
        case 3: date = token; break;
        }
        prev = pos + 1;
    }
    // 最后一个字段
    if (field == 4) {
        borrowed = (data.substr(prev) == "1");
    }

    return Book(id, title, author, date, borrowed);
}
/*
// 操作符 < 重载，用于在 AVL 树中比较 Book 对象
bool Book::operator<(const Book& other) const {
    return bookID < other.bookID;
}

// 操作符 == 重载，用于比较 Book 对象是否相等
bool Book::operator==(const Book& other) const {
    return bookID == other.bookID;
}
*/

