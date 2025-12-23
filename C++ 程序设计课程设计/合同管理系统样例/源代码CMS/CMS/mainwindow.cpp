#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include "EditContractDialog.h"
#include "QueryContractDialog.h"
#include "ReadFile.hpp"
#include "./include/ContractList.hpp"

#include <QMap>
#include <QDate>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    contractList.loadFromFile("contracts.csv");

    connect(ui->importContractButton, &QPushButton::clicked, this, &MainWindow::on_importContractButton_click);
    connect(ui->showAllContractsButton, &QPushButton::clicked, this, &MainWindow::on_showAllContractsButton_click);
    connect(ui->deleteContractButton, &QPushButton::clicked, this, &MainWindow::on_deleteContractButton_click);
    connect(ui->modifyContractButton, &QPushButton::clicked, this, &MainWindow::on_modifyContractButton_click);
    connect(ui->showExpiredContractsButton, &QPushButton::clicked, this, &MainWindow::on_showExpiredContractsButton_click);
    connect(ui->searchContractsButton, &QPushButton::clicked, this, &MainWindow::on_searchContractsButton_click);
    connect(ui->countContractsByMonthButton, &QPushButton::clicked, this, &MainWindow::on_countContractsByMonthButton_click);

}

MainWindow::~MainWindow() {
    contractList.saveToFile("contracts.csv");
    delete ui;
}



QDate stringToDate(const std::string& dateString) {
    return QDate::fromString(QString::fromStdString(dateString), "yyyy-MM-dd");
}

void MainWindow::showExpiredContracts(QTextEdit* textEdit)
{
    textEdit->clear();
    QDate currentDate = QDate::currentDate();

    Node* current = contractList.getHead();
    while (current != nullptr) {
        QDate endDate = stringToDate(current->contract.endDate);
        if (endDate.isValid() && endDate < currentDate) {
            current->contract.display(textEdit);
        }
        current = current->next;
    }
}

QString getYearMonth(const QDate& date) {
    return date.toString("yyyy-MM");
}

void MainWindow::getContractsCountByMonth(QTextEdit* textEdit)
{
    textEdit->clear();
    QMap<QString, int> monthCountMap;

    Node* current = contractList.getHead();
    while (current != nullptr) {
        QDate signingDate = stringToDate(current->contract.signingDate);
        if (signingDate.isValid()) {
            QString yearMonth = getYearMonth(signingDate);
            monthCountMap[yearMonth]++;
        }
        current = current->next;
    }

    for (auto it = monthCountMap.begin(); it != monthCountMap.end(); ++it) {
        textEdit->append(it.key() + ": " + QString::number(it.value()));
    }
}



void MainWindow::on_modifyContractButton_click() {
    QString contractNumber = QInputDialog::getText(this, tr("修改合同"), tr("请输入合同编号:"));
    if (!contractNumber.isEmpty()) {
        Contract* contract = contractList.findContract(contractNumber.toStdString());

        if (contract != nullptr) {
            EditContractDialog dialog(this,&contractList, false);
            dialog.loadContract(*contract);

            if (dialog.exec() == QDialog::Accepted) {
                Contract updatedContract = dialog.getUpdatedContract();
                *contract = updatedContract;
                contractList.saveToFile("contracts.csv");
                QMessageBox::information(this, tr("成功"), tr("合同修改成功"));
            }
        } else {
            QMessageBox::warning(this, tr("错误"), tr("合同不存在"));
        }
    }
}

bool MainWindow::isContractNumberExist(const std::string &contractNumber) {
    return contractList.findContract(contractNumber) != nullptr;
}
bool MainWindow::isContractDataValid(const Contract &contract)
{
    bool valid = true;

    // 验证合同编号
    if (contract.contractNumber.empty() || !isNumber(contract.contractNumber)) {
        valid = false;
    }

    // 验证日期格式
    if (!isValidDate(contract.signingDate) ||
        !isValidDate(contract.startDate) ||
        !isValidDate(contract.endDate)) {
        valid = false;
    }

    // 验证日期顺序
    QDate signingDate = stringToDate(contract.signingDate);
    QDate startDate = stringToDate(contract.startDate);
    QDate endDate = stringToDate(contract.endDate);

    if (!(signingDate.isValid() && startDate.isValid() && endDate.isValid() &&
          signingDate <= startDate && startDate <= endDate)) {
        valid = false;
    }

    // 验证金额
    if (contract.amount.empty() || !isNumber(contract.amount)) {
        valid = false;
    }

    return valid;
}

bool MainWindow::isNumber(const std::string &str)
{
    return !str.empty() && std::all_of(str.begin(), str.end(), ::isdigit);
}

bool MainWindow::isValidDate(const std::string &dateStr)
{
    QString date = QString::fromStdString(dateStr);
    if (date.length() != 10) {
        return false;
    }

    for (int i = 0; i < date.length(); ++i) {
        if (i == 4 || i == 7) {
            if (date[i] != '-') {
                return false;
            }
        } else {
            if (!date[i].isDigit()) {
                return false;
            }
        }
    }

    int year = date.mid(0, 4).toInt();
    int month = date.mid(5, 2).toInt();
    int day = date.mid(8, 2).toInt();

    if (month < 1 || month > 12) {
        return false;
    }

    // 检查每个月的天数
    int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    // 检查闰年
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        daysInMonth[1] = 29;
    }

    if (day < 1 || day > daysInMonth[month - 1]) {
        return false;
    }

    return true;
}

void MainWindow::importContractFromFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("导入合同"), "", tr("Text Files (*.txt);;All Files (*)"));
    if (!fileName.isEmpty()) {
        Contract contract;
        contract.readContent(fileName.toStdString());
        if (!isContractDataValid(contract)) {
            QMessageBox::warning(this, tr("数据错误"), tr("合同数据无效，请检查合同文件"));
            return;
        }

        if (isContractNumberExist(contract.contractNumber)) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, tr("覆盖合同"),
                                          tr("已经存在编号为 %1 的合同，是否覆盖原有合同？").arg(QString::fromStdString(contract.contractNumber)),
                                          QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::No) {
                return;
            } else {
                contractList.deleteContract(contract.contractNumber); // 删除原有合同
            }
        }

        contractList.addContract(contract); // 添加新合同
        contractList.saveToFile("contracts.csv"); // 保存到文件
        QMessageBox::information(this, tr("成功"), tr("合同导入成功"));
    }
}

void MainWindow::createNewContract()
{
    Contract newContract;
    EditContractDialog dialog(this,&contractList, true);
    dialog.loadContract(newContract);

    if (dialog.exec() == QDialog::Accepted) {
        Contract updatedContract = dialog.getUpdatedContract();

        if (isContractNumberExist(updatedContract.contractNumber)) {
            contractList.deleteContract(updatedContract.contractNumber); // 删除原有合同
        }

        contractList.addContract(updatedContract); // 添加新合同
        contractList.saveToFile("contracts.csv"); // 保存到文件
        QMessageBox::information(this, tr("成功"), tr("合同创建成功"));
    }
}
void MainWindow::on_importContractButton_click()
{
    QStringList options;
    options << tr("从文件导入") << tr("新建合同");

    bool ok;
    QString choice = QInputDialog::getItem(this, tr("选择操作"),
                                           tr("请选择要执行的操作："),
                                           options, 0, false, &ok);
    if (ok && !choice.isEmpty()) {
        if (choice == tr("从文件导入")) {
            importContractFromFile();
        } else if (choice == tr("新建合同")) {
            createNewContract();
        }
    }
}
void MainWindow::showAllContracts(QTextEdit* textEdit)
{
    contractList.displayAllContracts(textEdit);
}
void MainWindow::on_showAllContractsButton_click()
{
    ui->outputTextEdit->clear();
    showAllContracts(ui->outputTextEdit);
}

void MainWindow::on_deleteContractButton_click()
{
    QString contractNumber = QInputDialog::getText(this, tr("删除合同"), tr("请输入合同编号:"));
    if (!contractNumber.isEmpty()) {
        if (isContractNumberExist(contractNumber.toStdString())) {
            contractList.deleteContract(contractNumber.toStdString());
            contractList.saveToFile("contracts.csv");
            QMessageBox::information(this, tr("成功"), tr("合同删除成功"));
        } else {
            QMessageBox::warning(this, tr("错误"), tr("合同编号 %1 不存在").arg(contractNumber));
        }
    }
}


void MainWindow::on_showExpiredContractsButton_click()
{
    ui->outputTextEdit->clear();
    showExpiredContracts(ui->outputTextEdit);
}

void MainWindow::on_searchContractsButton_click()
{
    QueryContractDialog dialog(this, &contractList);
    dialog.exec();
}
void MainWindow::on_countContractsByMonthButton_click()
{
    ui->outputTextEdit->clear();
    getContractsCountByMonth(ui->outputTextEdit);
}
