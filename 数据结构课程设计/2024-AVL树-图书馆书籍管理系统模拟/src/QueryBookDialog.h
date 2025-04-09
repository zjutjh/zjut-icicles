// QueryBookDialog.h
#ifndef QUERYBOOKDIALOG_H
#define QUERYBOOKDIALOG_H

#include <QDialog>
#include "LibraryManager.h"

namespace Ui {
class QueryBookDialog;
}

class QueryBookDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QueryBookDialog(QWidget *parent = nullptr, LibraryManager* libraryManager = nullptr);
    ~QueryBookDialog();

private slots:
    void on_searchButton_clicked();
    void performSearch();

private:
    Ui::QueryBookDialog *ui;
    LibraryManager* libraryManager;
};

#endif // QUERYBOOKDIALOG_H
