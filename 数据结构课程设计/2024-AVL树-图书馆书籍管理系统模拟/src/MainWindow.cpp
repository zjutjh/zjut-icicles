// MainWindow.cpp
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "EditBookDialog.h"
#include <QMessageBox>
#include <QInputDialog> // 包含 QInputDialog
#include <QCloseEvent>  // 包含 QCloseEvent
#include <fstream>
#include <sstream>
#include "QueryBookDialog.h"
#include <QDebug> // 包含 QDebug 用于调试

// 构造函数
MainWindow::MainWindow(QWidget *parent, LibraryManager* libraryManager)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    libraryManager(libraryManager)
{
    ui->setupUi(this);
    connect(ui->showAllBooksButton, &QPushButton::clicked, [this]() { showAllBooks(ui->allBooksTextEdit); });
    connect(ui->updateBookStatusButton, &QPushButton::clicked, this, &MainWindow::updateBookStatus);
    // 不需要手动连接槽函数，因为使用了自动连接
}

// 析构函数
MainWindow::~MainWindow()
{
    delete ui;
}

// 重写 closeEvent 方法，在窗口关闭时保存图书信息
void MainWindow::closeEvent(QCloseEvent *event)
{
    saveBooksToFile();
    event->accept(); // 确保事件继续传播
}

// 添加图书按钮槽函数
void MainWindow::on_addBookButton_clicked()
{
    qDebug() << "Add Book button clicked";
    EditBookDialog dialog(this, libraryManager, true);
    dialog.exec();
}

// 移除图书按钮槽函数
void MainWindow::on_removeBookButton_clicked()
{
    qDebug() << "Remove Book button clicked";
    // 弹出输入对话框让用户输入要删除的图书编号
    bool ok;
    QString bookID = QInputDialog::getText(this, tr("删除图书"),
                                           tr("请输入要删除的图书编号："), QLineEdit::Normal,
                                           "", &ok);
    if (!ok || bookID.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "图书编号不能为空！");
        return;
    }

    // 确认删除操作
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "确认删除", "确定要删除图书编号为 " + bookID + " 的图书吗？",
                                  QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No) {
        return; // 用户取消删除
    }
    if(bookID == "root"){
        libraryManager->removeBook(libraryManager->getBookIDTreeRoot()->key);
        return;
    }
    // 执行删除操作
    bool deletionSuccess = libraryManager->removeBook(bookID.toStdString());

    if (deletionSuccess) {
        QMessageBox::information(this, "成功", "图书已删除！");
    } else {
        QMessageBox::warning(this, "删除失败", "未找到指定的图书编号，删除失败！");
    }
}

// 搜索图书按钮槽函数
void MainWindow::on_searchBookButton_clicked()
{
    // 打开查询对话框
    QueryBookDialog dialog(this, libraryManager);
    dialog.exec();
}

// 保存图书信息到文件
void MainWindow::saveBooksToFile()
{
    QString fileName = "books.txt";
    std::ofstream outFile(fileName.toStdString());

    if (!outFile.is_open()) {
        QMessageBox::warning(this, "错误", "无法打开文件进行保存！");
        return;
    }

    qDebug() << "开始保存图书信息到" << fileName;

    // 使用正确的模板参数
    std::function<void(AVLNode<std::string, Book*>*)> inOrderSave = [&](AVLNode<std::string, Book*>* node) {
        if (node == nullptr)
            return;
        inOrderSave(node->left);
        std::string serialized = node->value->serialize();
        outFile << serialized << "\n";
        qDebug() << "保存图书：" << QString::fromStdString(serialized);
        inOrderSave(node->right);
    };

    inOrderSave(libraryManager->getBookIDTreeRoot());

    outFile.close();
    QMessageBox::information(this, "成功", "图书信息已保存！");
}

// 显示所有图书
void MainWindow::showAllBooks(QTextEdit* textEdit)
{
    std::vector<std::pair<Book*,int>> allBooks = libraryManager->getAllBooks();

    std::ostringstream oss;
    Book* book;
    for (const auto& bookAndHeight : allBooks) {
        book = bookAndHeight.first;
        oss << "图书编号: " << book->getBookID()
            << ", height: " << bookAndHeight.second
            << ", 书名: " << book->getTitle()
            << ", 作者: " << book->getAuthor()
            << ", 出版日期: " << book->getPublishDate()
            << ", 借阅状态: " << (book->getIsBorrowed() ? "已借出" : "可借") << "\n";
    }
    textEdit->setPlainText(QString::fromStdString(oss.str()));
}

// 更新借阅状态按钮槽函数
void MainWindow::updateBookStatus()
{
    qDebug() << "Update Book Status button clicked";
    // 弹出对话框让用户输入图书编号
    bool ok;
    QString bookID = QInputDialog::getText(this, tr("更新借阅状态"),
                                           tr("请输入图书编号："), QLineEdit::Normal,
                                           "", &ok);
    if (!ok || bookID.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "图书编号不能为空！");
        return;
    }

    // 搜索图书
    Book* book = libraryManager->searchByBookID(bookID.toStdString());
    if (!book) {
        QMessageBox::warning(this, "未找到", "未找到指定的图书编号！");
        return;
    }

    // 弹出对话框让用户选择新的借阅状态
    QString newStatus = QInputDialog::getItem(this, tr("更新状态"),
                                              tr("选择新的借阅状态："),
                                              QStringList() << "已借出" << "可借",
                                              0, false, &ok);
    if (!ok || newStatus.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "借阅状态不能为空！");
        return;
    }

    // 更新借阅状态
    if (newStatus == "已借出") {
        book->setIsBorrowed(true);
    } else {
        book->setIsBorrowed(false);
    }

    QMessageBox::information(this, "成功", "借阅状态已更新！");
}
