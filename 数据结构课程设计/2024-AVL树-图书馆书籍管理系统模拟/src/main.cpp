// main.cpp

#include <QApplication>
#include "MainWindow.h"
#include "LoginDialog.h"
#include "UserManagementDialog.h"
#include "LibraryManager.h"
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    LibraryManager libraryManager;

    LoginDialog loginDialog;
    if (loginDialog.exec() == QDialog::Accepted) {
        bool isAdmin = loginDialog.isAdmin();
        if (isAdmin) {

            bool continueSelection = true;
            while (continueSelection) {
                // 创建自定义的消息框
                QMessageBox msgBox;
                msgBox.setWindowTitle("选择系统");
                msgBox.setText("请选择要进入的系统：");

                // 创建自定义按钮
                QPushButton *userMgmtButton = msgBox.addButton("用户管理", QMessageBox::AcceptRole);
                QPushButton *bookMgmtButton = msgBox.addButton("图书管理", QMessageBox::RejectRole);
                QPushButton *exitButton = msgBox.addButton("退出", QMessageBox::DestructiveRole);

                // 设置消息框图标（可选）
                msgBox.setIcon(QMessageBox::Question);

                // 显示消息框并等待用户选择
                msgBox.exec();

                if (msgBox.clickedButton() == userMgmtButton) {
                    qDebug() << "管理员选择了用户管理系统";
                    // 选择用户管理系统
                    UserManagementDialog userMgmtDialog;
                    userMgmtDialog.exec();
                }
                else if (msgBox.clickedButton() == bookMgmtButton) {
                    qDebug() << "管理员选择了图书管理系统";
                    // 选择图书管理系统
                    MainWindow w(nullptr, &libraryManager);
                    w.show();
                    return a.exec();
                }
                else {
                    qDebug() << "管理员选择退出，程序退出";
                    // 退出程序
                    continueSelection = false;
                    return 0;
                }
            }

        } else {
            // 普通用户登录，直接进入图书管理系统
            qDebug() << "普通用户登录，进入图书管理系统";
            MainWindow w(nullptr, &libraryManager);
            w.show();
            return a.exec();
        }
    }

    return 0;
}
