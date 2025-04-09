// EditBookDialog.cpp

#include "EditBookDialog.h"
#include "ui_EditBookDialog.h"
#include "LibraryManager.h"
#include <QMessageBox>

// 构造函数
EditBookDialog::EditBookDialog(QWidget *parent, LibraryManager* libraryManager, bool isNewBook)
    : QDialog(parent),
    ui(new Ui::EditBookDialog),
    libraryManager(libraryManager),
    isNewBook(isNewBook)
{
    ui->setupUi(this);
    this->setWindowTitle(isNewBook ? "添加图书" : "编辑图书");

    if (!isNewBook) {
        // 加载现有图书信息
        // 假设有一个方法获取图书信息
    }

    // 连接信号槽
    //connect(ui->saveButton, &QPushButton::clicked, this, &EditBookDialog::on_saveButton_clicked);
    connect(ui->bookIDLineEdit, &QLineEdit::textChanged, this, &EditBookDialog::validateBookID);
}

// 析构函数
EditBookDialog::~EditBookDialog()
{
    delete ui;
}

// 验证图书编号是否已存在（仅在添加新书时）
void EditBookDialog::validateBookID()
{
    if (isNewBook) {
        QString bookID = ui->bookIDLineEdit->text().trimmed();
        if (!bookID.isEmpty()) {
            Book* existingBook = libraryManager->searchByBookID(bookID.toStdString());
            if (existingBook) {
                ui->bookIDLineEdit->setStyleSheet("border: 1px solid red");
                // 可以添加更多提示，如 QLabel 显示错误信息
            } else {
                ui->bookIDLineEdit->setStyleSheet("");
            }
        }
    }
}

// 保存按钮槽函数
void EditBookDialog::on_saveButton_clicked()
{
    // 收集输入数据
    QString bookID = ui->bookIDLineEdit->text().trimmed();
    QString title = ui->titleLineEdit->text().trimmed();
    QString author = ui->authorLineEdit->text().trimmed();
    QString publishDate = ui->publishDateEdit->date().toString("yyyy-MM-dd");

    bool isBorrowed = ui->borrowedCheckBox->isChecked();

    if (bookID.isEmpty() || title.isEmpty() || author.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "请填写所有必填字段！");
        return;
    }

    Book updatedBook(bookID.toStdString(), title.toStdString(), author.toStdString(), publishDate.toStdString(), isBorrowed);

    if (isNewBook) {
        // 验证图书编号是否已存在
        Book* existingBook = libraryManager->searchByBookID(bookID.toStdString());
        if (existingBook) {
            QMessageBox::warning(this, "重复编号", "图书编号已存在！");
            return;
        }
        libraryManager->addBook(updatedBook);
    } else {
        // 编辑现有图书
        // 假设有方法根据 bookID 更新图书信息
        libraryManager->removeBook(bookID.toStdString());
        libraryManager->addBook(updatedBook);
    }

    QMessageBox::information(this, "成功", "图书信息已保存！");
    accept();
}
