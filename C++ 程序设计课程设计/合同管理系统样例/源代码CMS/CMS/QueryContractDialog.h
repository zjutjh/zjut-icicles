#ifndef QUERYCONTRACTDIALOG_H
#define QUERYCONTRACTDIALOG_H

#include <QDialog>
#include <QTextEdit>
#include "ReadFile.hpp"
#include "./include/ContractList.hpp"

namespace Ui {
class QueryContractDialog;
}

class QueryContractDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QueryContractDialog(QWidget *parent, ContractList* contractList) ;
    ~QueryContractDialog();

private slots:
    void on_searchButton_clicked();

private:
    Ui::QueryContractDialog *ui;
    void performSearch();
    ContractList* contractList;
};

#endif // QUERYCONTRACTDIALOG_H
