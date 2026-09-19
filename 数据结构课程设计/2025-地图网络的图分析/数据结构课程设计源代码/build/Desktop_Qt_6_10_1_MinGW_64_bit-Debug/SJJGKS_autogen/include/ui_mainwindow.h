/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.10.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLabel *label;
    QLabel *label_2;
    QLineEdit *sourceEdit;
    QLineEdit *targetEdit;
    QPushButton *runButton;
    QPushButton *runButton_2;
    QTextEdit *outputEdit;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        label = new QLabel(centralwidget);
        label->setObjectName("label");
        label->setGeometry(QRect(210, 70, 40, 12));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(320, 70, 40, 12));
        sourceEdit = new QLineEdit(centralwidget);
        sourceEdit->setObjectName("sourceEdit");
        sourceEdit->setGeometry(QRect(150, 140, 113, 20));
        targetEdit = new QLineEdit(centralwidget);
        targetEdit->setObjectName("targetEdit");
        targetEdit->setGeometry(QRect(290, 140, 113, 20));
        runButton = new QPushButton(centralwidget);
        runButton->setObjectName("runButton");
        runButton->setGeometry(QRect(180, 190, 56, 18));
        runButton_2 = new QPushButton(centralwidget);
        runButton_2->setObjectName("runButton_2");
        runButton_2->setGeometry(QRect(310, 190, 56, 18));
        outputEdit = new QTextEdit(centralwidget);
        outputEdit->setObjectName("outputEdit");
        outputEdit->setGeometry(QRect(440, 70, 104, 66));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 18));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "\350\265\267\347\202\271ID", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "\347\273\210\347\202\271ID", nullptr));
        sourceEdit->setText(QString());
        runButton->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        runButton_2->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
