// LoginDialog.h
#ifndef LOGINDIALOG_H
#define LOGINDIALOG_H

#include <QDialog>

namespace Ui {
class LoginDialog;
}

class LoginDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LoginDialog(QWidget *parent = nullptr);
    ~LoginDialog();

    bool isAdmin() const; // 是否为管理员

private slots:
    void on_loginButton_clicked();
    void on_cancelButton_clicked();

private:
    Ui::LoginDialog *ui;
    bool authenticate(const QString &loginType, const QString &username, const QString &password);
    bool m_isAdmin = false; // 是否为管理员
};

#endif // LOGINDIALOG_H
