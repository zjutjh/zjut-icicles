// LoginDialog.cpp

#include "LoginDialog.h"
#include "ui_LoginDialog.h"
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDebug>

LoginDialog::LoginDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LoginDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("登录");

    // 设置默认登录类型为“用户”
    ui->loginTypeComboBox->setCurrentIndex(1);
}

LoginDialog::~LoginDialog()
{
    delete ui;
}

void LoginDialog::on_loginButton_clicked()
{
    QString loginType = ui->loginTypeComboBox->currentText().trimmed();
    QString username = ui->usernameLineEdit->text().trimmed();
    QString password = ui->passwordLineEdit->text().trimmed();

    if (username.isEmpty() || password.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "用户名和密码不能为空！");
        return;
    }

    if (authenticate(loginType, username, password)) {
        QMessageBox::information(this, "登录成功", "欢迎，" + username + "！");
        // 存储用户类型
        m_isAdmin = (loginType == "管理员");
        accept(); // 关闭对话框并返回 QDialog::Accepted
    } else {
        QMessageBox::warning(this, "登录失败", "用户名或密码错误！");
    }
}

void LoginDialog::on_cancelButton_clicked()
{
    reject(); // 关闭对话框并返回 QDialog::Rejected
}

bool LoginDialog::authenticate(const QString &loginType, const QString &username, const QString &password)
{
    QString fileName;

    if (loginType == "管理员") {
        fileName = "admins.txt";
    } else if (loginType == "用户") {
        fileName = "users.txt";
    } else {
        QMessageBox::warning(this, "错误", "未知的登录类型！");
        return false;
    }

    QFile file(fileName);
    if (!file.exists()) {
        // 如果管理员文件不存在，创建一个默认管理员账户
        if (loginType == "管理员") {
            if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream out(&file);
                out << "admin,admin123\n"; // 默认管理员账户
                file.close();
                qDebug() << "Created default admin account.";
            } else {
                QMessageBox::warning(this, "错误", "无法创建管理员文件！");
                return false;
            }
        } else if (loginType == "用户") {
            // 如果用户文件不存在，创建空文件
            if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                file.close();
                qDebug() << "Created empty users file.";
            } else {
                QMessageBox::warning(this, "错误", "无法创建用户文件！");
                return false;
            }
        }
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开文件进行读取！");
        return false;
    }

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty())
            continue;

        QStringList parts = line.split(",");
        if (parts.size() != 2)
            continue;

        QString fileUsername = parts.at(0).trimmed();
        QString filePassword = parts.at(1).trimmed();

        if (username == fileUsername && password == filePassword) {
            file.close();
            return true;
        }
    }

    file.close();
    return false;
}

// Getter for admin status
bool LoginDialog::isAdmin() const
{
    return m_isAdmin;
}
