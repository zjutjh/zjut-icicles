// UserManagementDialog.h

#ifndef USERMANAGEMENTDIALOG_H
#define USERMANAGEMENTDIALOG_H

#include <QDialog>

namespace Ui {
class UserManagementDialog;
}

class UserManagementDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UserManagementDialog(QWidget *parent = nullptr);
    ~UserManagementDialog();

private slots:
    void on_addUserButton_clicked();
    void on_queryUsersButton_clicked();
    void on_deleteUserButton_clicked();

private:
    Ui::UserManagementDialog *ui;

    // 用户管理功能
    bool addUser(const QString &username, const QString &password);
    QStringList getAllUsers();
    bool deleteUser(const QString &username);
};

#endif // USERMANAGEMENTDIALOG_H
