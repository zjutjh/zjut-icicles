// MainWindow.h
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTextEdit>
#include "LibraryManager.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // 构造函数
    MainWindow(QWidget *parent = nullptr, LibraryManager* libraryManager = nullptr);
    // 析构函数
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override; // 重写 closeEvent 方法

private slots:
    void on_addBookButton_clicked();
    void on_removeBookButton_clicked();
    void on_searchBookButton_clicked();
    void showAllBooks(QTextEdit* textEdit);
    void updateBookStatus(); // 声明 updateBookStatus 槽函数

private:
    Ui::MainWindow *ui;
    LibraryManager* libraryManager;
    void saveBooksToFile(); // 声明 saveBooksToFile 方法
};
#endif // MAINWINDOW_H
