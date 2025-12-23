#-------------------------------------------------
#
# Project created by QtCreator 2024-06-03T16:55:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ex2
TEMPLATE = app

# 使编译器在使用弃用的 Qt 功能时发出警告
DEFINES += QT_DEPRECATED_WARNINGS

# 使用 C++11 标准
CONFIG += c++11

# 添加源文件
SOURCES += \
    AVLTree.cpp \
    Book.cpp \
    EditBookDialog.cpp \
    LoginDialog.cpp \
    MainWindow.cpp \
    QueryBookDialog.cpp \
    librarymanager.cpp \
    main.cpp \
    usermanagementdialog.cpp

# 添加头文件
HEADERS += \
    AVLTree.h \
    Book.h \
    EditBookDialog.h \
    MainWindow.h \
    MyUnorderedMap.h \
    QueryBookDialog.h \
    include/AVLNode.hpp \
    librarymanager.h \
    logindialog.h \
    usermanagementdialog.h

# 添加 UI 文件
FORMS += \
    EditBookDialog.ui \
    LoginDialog.ui \
    MainWindow.ui \
    QueryBookDialog.ui \
    UserManagementDialog.ui

# 默认的部署规则
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# 添加包含路径
INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/include
