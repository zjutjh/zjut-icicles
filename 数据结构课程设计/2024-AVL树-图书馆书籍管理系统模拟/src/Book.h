// Book.h
#ifndef BOOK_H
#define BOOK_H

#include <string>

class Book {
public:
    Book() = default;
    Book(const std::string& bookID, const std::string& title, const std::string& author,
         const std::string& publishDate, bool isBorrowed)
        : bookID(bookID), title(title), author(author),
        publishDate(publishDate), isBorrowed(isBorrowed) {}

    // 访问器
    std::string getBookID() const { return bookID; }
    std::string getTitle() const { return title; }
    std::string getAuthor() const { return author; }
    std::string getPublishDate() const { return publishDate; }
    bool getIsBorrowed() const { return isBorrowed; }

    // 设置器
    void setBookID(const std::string& id) { bookID = id; }
    void setTitle(const std::string& t) { title = t; }
    void setAuthor(const std::string& a) { author = a; }
    void setPublishDate(const std::string& date) { publishDate = date; }
    void setIsBorrowed(bool borrowed) { isBorrowed = borrowed; }

    // 序列化方法
    std::string serialize() const;

    // 反序列化方法
    static Book deserialize(const std::string& data);

private:
    std::string bookID;
    std::string title;
    std::string author;
    std::string publishDate;
    bool isBorrowed;
};

#endif // BOOK_H
