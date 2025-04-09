#ifndef LIBRARYMANAGER_H
#define LIBRARYMANAGER_H

#include "Book.h"
#include "AVLTree.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <MyUnorderedMap.h>
class LibraryManager {
public:
    LibraryManager();
    ~LibraryManager();

    bool addBook(const Book& book);
    bool removeBook(const std::string& bookID);

    Book* searchByBookID(const std::string& bookID) const;
    std::vector<Book*> searchBooks(const std::string& query) const; // 新增方法

    std::vector<std::pair<Book*,int>> getAllBooks() const;
    void inOrderTraverse(AVLNode<std::string, Book*>* node, std::vector<std::pair<Book*,int>>& booksAndHeight) const;

    AVLNode<std::string, Book*>* getBookIDTreeRoot() const;
    std::vector<Book*> getAllBooksOld() const;

private:
    AVLTree<std::string, Book*> bookIDTree;
    AVLTree<std::string, Book*> titleTree;
    AVLTree<std::string, Book*> authorTree;

    // 倒排索引：关键词 -> 书籍列表
    //std::unordered_map<std::string, std::vector<Book*>> invertedIndex;
    MyUnorderedMap<std::string, std::vector<Book*>> invertedIndex;

    void inOrderTraverseHelper(AVLNode<std::string, Book*>* node, std::vector<std::pair<Book*,int>>& booksAndHeight) const;
    void loadBooksFromFile(const std::string& filename = "books.txt");
    void addToIndex(const Book& book);
    void removeFromIndex(const Book& book);

    // 辅助函数
    std::vector<std::string> tokenize(const std::string& text) const;
    std::string toLower(const std::string& str) const; // 声明 toLower 函数

};

#endif // LIBRARYMANAGER_H
