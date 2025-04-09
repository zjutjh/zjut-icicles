#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "EditContractDialog.h"
#include "ui_EditContractDialog.h"
#include <QMessageBox>
#include <QRegExpValidator>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include "EditContractDialog.h"
#include "QueryContractDialog.h"
#include "ReadFile.hpp"
#include "./include/ContractList.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_importContractButton_click();
    void on_showAllContractsButton_click();
    void on_deleteContractButton_click();
    void on_modifyContractButton_click();
    void on_showExpiredContractsButton_click();
    void on_searchContractsButton_click();
    void on_countContractsByMonthButton_click();

private:
    Ui::MainWindow *ui;
    ContractList contractList;
    void showAllContracts(QTextEdit* textEdit);
    void showExpiredContracts(QTextEdit* textEdit);
    bool isContractNumberExist(const std::string &contractNumber);
    void getContractsCountByMonth(QTextEdit* textEdit);
    void importContractFromFile();
    void createNewContract();
    bool isContractDataValid(const Contract &contract);
    bool isNumber(const std::string &str);
    bool isValidDate(const std::string &dateStr);

};

#endif // MAINWINDOW_H
