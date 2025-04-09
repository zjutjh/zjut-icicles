// UserManagementDialog.cpp

#include "UserManagementDialog.h"
#include "ui_UserManagementDialog.h"
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDebug>

UserManagementDialog::UserManagementDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::UserManagementDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("用户管理系统");
}

UserManagementDialog::~UserManagementDialog()
{
    delete ui;
}

void UserManagementDialog::on_addUserButton_clicked()
{
    QString username = ui->addUsernameLineEdit->text().trimmed();
    QString password = ui->addPasswordLineEdit->text().trimmed();

    if (username.isEmpty() || password.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "用户名和密码不能为空！");
        return;
    }

    bool success = addUser(username, password);
    if (success) {
        QMessageBox::information(this, "成功", "用户已添加！");
        ui->addUsernameLineEdit->clear();
        ui->addPasswordLineEdit->clear();
    } else {
        QMessageBox::warning(this, "失败", "用户名已存在！");
    }
}

void UserManagementDialog::on_queryUsersButton_clicked()
{
    QStringList users = getAllUsers();
    if (users.isEmpty()) {
        ui->usersTextEdit->setPlainText("没有用户记录。");
    } else {
        QString displayText;
        for (const QString &user : users) {
            displayText += user + "\n";
        }
        ui->usersTextEdit->setPlainText(displayText);
    }
}

void UserManagementDialog::on_deleteUserButton_clicked()
{
    QString username = ui->deleteUsernameLineEdit->text().trimmed();

    if (username.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "用户名不能为空！");
        return;
    }

    bool success = deleteUser(username);
    if (success) {
        QMessageBox::information(this, "成功", "用户已删除！");
        ui->deleteUsernameLineEdit->clear();
    } else {
        QMessageBox::warning(this, "失败", "未找到指定的用户名！");
    }
}

// 添加新用户到users.txt
bool UserManagementDialog::addUser(const QString &username, const QString &password)
{
    QFile file("users.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        // 如果文件不存在，创建一个新文件
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "错误", "无法打开用户文件！");
            return false;
        }
    }

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty())
            continue;

        QStringList parts = line.split(",");
        if (parts.size() < 1)
            continue;

        QString existingUsername = parts.at(0).trimmed();
        if (username == existingUsername) {
            file.close();
            return false; // 用户名已存在
        }
    }
    file.close();

    // 追加新用户
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开用户文件进行写入！");
        return false;
    }

    QTextStream out(&file);
    out << username << "," << password << "\n";
    file.close();
    return true;
}

// 从users.txt获取所有用户
QStringList UserManagementDialog::getAllUsers()
{
    QStringList users;

    QFile file("users.txt");
    if (!file.exists()) {
        // 没有用户记录
        return users;
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开用户文件进行读取！");
        return users;
    }

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty())
            continue;

        QStringList parts = line.split(",");
        if (parts.size() != 2)
            continue;

        QString username = parts.at(0).trimmed();
        QString password = parts.at(1).trimmed();

        users << QString("用户名: %1, 密码: %2").arg(username, password);
    }

    file.close();
    return users;
}

// 从users.txt删除用户
bool UserManagementDialog::deleteUser(const QString &username)
{
    QFile file("users.txt");
    if (!file.exists()) {
        return false; // 用户文件不存在
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开用户文件进行读取！");
        return false;
    }

    QStringList userLines;
    QTextStream in(&file);
    bool found = false;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty())
            continue;

        QStringList parts = line.split(",");
        if (parts.size() < 1)
            continue;

        QString existingUsername = parts.at(0).trimmed();
        if (username != existingUsername) {
            userLines << line;
        } else {
            found = true;
        }
    }

    file.close();

    if (!found) {
        return false; // 未找到要删除的用户
    }

    // 写回文件
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开用户文件进行写入！");
        return false;
    }

    QTextStream out(&file);
    for (const QString &userLine : userLines) {
        out << userLine << "\n";
    }

    file.close();
    return true;
}
