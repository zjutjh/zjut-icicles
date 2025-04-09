#include "LibraryManager.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include "MyUnorderedMap.h" // 确保路径正确，使用双引号

LibraryManager::LibraryManager() {
    // 在构造函数中加载图书信息
    loadBooksFromFile();
}

LibraryManager::~LibraryManager() {
    // 通过 bookIDTree 中的所有节点删除 Book 对象

    std::vector<std::pair<Book*,int>> allBooks = getAllBooks();
    for (auto book : allBooks) {
        delete book.first;
    }
}

bool LibraryManager::addBook(const Book& book) {
    // 检查图书编号是否已存在
    if (bookIDTree.search(book.getBookID()) != nullptr) {
        return false; // 图书编号重复
    }

    // 创建一个动态分配的 Book 对象
    Book* newBook = new Book(book);

    // 插入到所有 AVL 树中
    bookIDTree.insert(newBook->getBookID(), newBook);
    titleTree.insert(newBook->getTitle(), newBook);
    authorTree.insert(newBook->getAuthor(), newBook);
    addToIndex(*newBook); // 更新倒排索引

    return true; // 添加成功
}

bool LibraryManager::removeBook(const std::string& bookID) {
    // 通过 bookID 查找书籍
    Book* book = bookIDTree.search(bookID);
    if (!book) {
        return false; // 未找到书籍，返回 false
    }

    // 从所有 AVL 树中移除
    bookIDTree.remove(bookID);
    titleTree.remove(book->getTitle());
    authorTree.remove(book->getAuthor());

    removeFromIndex(*book); // 更新倒排索引

    // 删除 Book 对象
    delete book;
    return true; // 删除成功，返回 true
}

Book* LibraryManager::searchByBookID(const std::string& bookID) const {
    return bookIDTree.search(bookID);
}

std::vector<std::string> LibraryManager::tokenize(const std::string& text) const {
    std::vector<std::string> tokens;
    std::istringstream iss(text);
    std::string word;
    while (iss >> word) {
        // 去除标点符号
        word.erase(std::remove_if(word.begin(), word.end(), ::ispunct), word.end());
        // 转换为小写
        word = toLower(word);
        if (!word.empty()) {
            tokens.push_back(word);
        }
    }
    return tokens;
}

std::string LibraryManager::toLower(const std::string& str) const {
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return lowerStr;
}

// 实现 searchBooks
std::vector<Book*> LibraryManager::searchBooks(const std::string& query) const {
    std::vector<std::string> tokens = tokenize(query);
    MyUnorderedMap<Book*, int> bookMatchCount;

    for (const auto& token : tokens) {
        const std::vector<Book*>* books_ptr = invertedIndex.find(token);
        if (books_ptr != nullptr) {
            const std::vector<Book*>& books = *books_ptr;
            for (auto book : books) {
                int* count_ptr = bookMatchCount.find(book);
                if (count_ptr != nullptr) {
                    (*count_ptr)++;
                } else {
                    bookMatchCount.insert(book, 1);
                }
            }
        }
    }

    // 获取所有匹配的书籍及其匹配次数
    std::vector<std::pair<Book*, int>> matchedBooks;
    std::vector<std::pair<Book*, int>> allPairs = bookMatchCount.getAllKeyValuePairs();
    matchedBooks.reserve(allPairs.size());

    for (const auto& pair : allPairs) {
        matchedBooks.emplace_back(pair);
    }

    // 按匹配次数从高到低排序
    std::sort(matchedBooks.begin(), matchedBooks.end(),
              [](const std::pair<Book*, int>& a, const std::pair<Book*, int>& b) -> bool {
                  return a.second > b.second;
              });

    // 提取排序后的书籍
    std::vector<Book*> sortedBooks;
    sortedBooks.reserve(matchedBooks.size());
    for (const auto& pair : matchedBooks) {
        sortedBooks.push_back(pair.first);
    }

    return sortedBooks;
}

// 获取所有图书
std::vector<std::pair<Book*,int>> LibraryManager::getAllBooks() const {
    std::vector<std::pair<Book*,int>> booksAndHeight;
    inOrderTraverseHelper(bookIDTree.getRoot(), booksAndHeight);
    return booksAndHeight;
}
std::vector<Book*> LibraryManager::getAllBooksOld() const {
    std::vector<std::pair<Book*,int>> booksAndHeight;
    inOrderTraverseHelper(bookIDTree.getRoot(), booksAndHeight);
    std::vector<Book*> books;
    for (const auto& pair : booksAndHeight) {
        books.push_back(pair.first);  // pair.first 是 Book* 类型
    }
    return books;
}


// 公开中序遍历方法
void LibraryManager::inOrderTraverse(AVLNode<std::string, Book*>* node, std::vector<std::pair<Book*,int>>& booksAndHeight) const {
    inOrderTraverseHelper(node, booksAndHeight);
}

// 实现 getter 函数
AVLNode<std::string, Book*>* LibraryManager::getBookIDTreeRoot() const {
    return bookIDTree.getRoot();
}

// 辅助函数：中序遍历收集所有节点
void LibraryManager::inOrderTraverseHelper(AVLNode<std::string, Book*>* node, std::vector<std::pair<Book*,int>>& booksAndHeight) const {
    if (!node)
        return;
    inOrderTraverseHelper(node->left, booksAndHeight);
    booksAndHeight.push_back(std::pair<Book*, int>(node->value, node->height));
    inOrderTraverseHelper(node->right, booksAndHeight);
}

// 加载图书信息从文件
void LibraryManager::loadBooksFromFile(const std::string& filename) {
    std::ifstream inFile(filename);

    if (!inFile.is_open()) {
        // 文件不存在或无法打开，可能是第一次运行
        std::cout << "无法打开文件 " << filename << " 进行加载。可能是第一次运行程序。" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(inFile, line)) {
        if (line.empty())
            continue;

        std::istringstream iss(line);
        std::string bookID, title, author, publishDate, isBorrowedStr;

        // 每行的格式为：bookID,title,author,publishDate,isBorrowed
        if (std::getline(iss, bookID, ',') &&
            std::getline(iss, title, ',') &&
            std::getline(iss, author, ',') &&
            std::getline(iss, publishDate, ',') &&
            std::getline(iss, isBorrowedStr)) {

            bool isBorrowed = (isBorrowedStr == "1") ? true : false;

            // 创建 Book 对象并添加到库
            Book book(bookID, title, author, publishDate, isBorrowed);
            addBook(book);
        }
    }
    inFile.close();
}

// 将书名和作者分词并添加到倒排索引
// 实现 addToIndex
void LibraryManager::addToIndex(const Book& book) {
    auto addWordsToIndex = [&](const std::string& text) {
        std::istringstream iss(text);
        std::string word;
        while (iss >> word) {
            word = toLower(word);
            // 去除标点符号
            word.erase(std::remove_if(word.begin(), word.end(), ::ispunct), word.end());
            // 使用 operator[] 访问或插入
            invertedIndex[word].push_back(const_cast<Book*>(&book));
        }
    };

    addWordsToIndex(book.getTitle());
    addWordsToIndex(book.getAuthor());
}

void LibraryManager::removeFromIndex(const Book& book) {
    auto removeWordsFromIndex = [&](const std::string& text) {
        std::istringstream iss(text);
        std::string word;
        while (iss >> word) {
            word = toLower(word);
            // 去除标点符号
            word.erase(std::remove_if(word.begin(), word.end(), ::ispunct), word.end());
            try {
                // 使用 at() 方法
                std::vector<Book*>& books = const_cast<std::vector<Book*>&>(invertedIndex.at(word));
                // 使用 std::remove 移除特定书籍指针
                books.erase(std::remove(books.begin(), books.end(), &book), books.end());

                if (books.empty()) {
                    invertedIndex.erase(word);
                }
            } catch (const std::out_of_range& e) {
                // 如果键不存在，忽略
            }
        }
    };

    removeWordsFromIndex(book.getTitle());
    removeWordsFromIndex(book.getAuthor());
}
