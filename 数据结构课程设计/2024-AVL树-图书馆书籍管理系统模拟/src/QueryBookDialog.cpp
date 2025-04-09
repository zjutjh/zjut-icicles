// QueryBookDialog.cpp

#include "QueryBookDialog.h"
#include "ui_QueryBookDialog.h"
#include <QMessageBox>
#include <sstream>

QueryBookDialog::QueryBookDialog(QWidget *parent, LibraryManager* libraryManager)
    : QDialog(parent),
    ui(new Ui::QueryBookDialog),
    libraryManager(libraryManager)
{
    ui->setupUi(this);
    this->setWindowTitle("查询图书信息");

    // 连接搜索按钮的点击事件到槽函数
    //connect(ui->searchButton, &QPushButton::clicked, this, &QueryBookDialog::on_searchButton_clicked);
}

QueryBookDialog::~QueryBookDialog()
{
    delete ui;
}

// 搜索按钮槽函数
void QueryBookDialog::on_searchButton_clicked()
{
    performSearch();
}

// 执行搜索操作
void QueryBookDialog::performSearch()
{
    QString bookID = ui->bookIDLineEdit->text().trimmed();
    QString keywords = ui->keywordLineEdit->text().trimmed(); // 获取关键词输入
    QString borrowedStatus = ui->borrowedStatusComboBox->currentText();

    if (bookID.isEmpty() && keywords.isEmpty() && borrowedStatus == "全部") {
        QMessageBox::warning(this, "输入错误", "请至少输入一个搜索条件！");
        return;
    }

    std::vector<Book*> filteredBooks;

    // 如果输入了图书编号，优先根据图书编号搜索
    if (!bookID.isEmpty()) {
        Book* book = libraryManager->searchByBookID(bookID.toStdString());
        if (book)
            filteredBooks.push_back(book);
    }
    // 否则，使用关键词进行模糊搜索
    else if (!keywords.isEmpty()) {
        std::vector<Book*> keywordResults = libraryManager->searchBooks(keywords.toStdString());
        filteredBooks = keywordResults;
    }
    // 如果仅输入了借阅状态
    else {
        if (borrowedStatus == "全部") {
            filteredBooks = libraryManager->getAllBooksOld();
        }
    }

    // 根据借阅状态筛选
    if (borrowedStatus != "全部") {
        std::vector<Book*> temp;
        for (auto book : filteredBooks) {
            if ((borrowedStatus == "已借出" && book->getIsBorrowed()) ||
                (borrowedStatus == "可借" && !book->getIsBorrowed()))
                temp.push_back(book);
        }
        filteredBooks = temp;
    }

    // 显示搜索结果
    if (!filteredBooks.empty()) {
        std::ostringstream oss;
        for (const auto& book : filteredBooks) {
            oss << "图书编号: " << book->getBookID()
                << ", 书名: " << book->getTitle()
                << ", 作者: " << book->getAuthor()
                << ", 出版日期: " << book->getPublishDate()
                << ", 借阅状态: " << (book->getIsBorrowed() ? "已借出" : "可借") << "\n";
        }
        ui->resultTextEdit->setPlainText(QString::fromStdString(oss.str()));
    } else {
        QMessageBox::information(this, "查询结果", "未找到符合条件的图书。");
        ui->resultTextEdit->clear();
    }
}
