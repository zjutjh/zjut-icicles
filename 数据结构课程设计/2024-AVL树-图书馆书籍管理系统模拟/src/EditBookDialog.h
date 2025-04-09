// EditBookDialog.h
#ifndef EDITBOOKDIALOG_H
#define EDITBOOKDIALOG_H

#include <QDialog>
#include "ui_EditBookDialog.h"
#include "LibraryManager.h"

namespace Ui {
class EditBookDialog;
}

class EditBookDialog : public QDialog
{
    Q_OBJECT

public:
    explicit EditBookDialog(QWidget *parent = nullptr, LibraryManager* libraryManager = nullptr, bool isNewBook = false);
    ~EditBookDialog();

private slots:
    void on_saveButton_clicked();
    void validateBookID();

private:
    Ui::EditBookDialog *ui;
    LibraryManager* libraryManager;
    bool isNewBook;
};

#endif // EDITBOOKDIALOG_H
