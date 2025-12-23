#include "QueryContractDialog.h"
#include "ui_QueryContractDialog.h"
#include <QMessageBox>
//#include "./lib/sqlite3.h"
#include "./include/ReadFile.hpp"
#include "./include/ContractList.hpp"
QueryContractDialog::QueryContractDialog(QWidget *parent, ContractList* contractList) :
    QDialog(parent),
    ui(new Ui::QueryContractDialog),
   contractList(contractList)
{
    ui->setupUi(this);
    connect(ui->searchButton, &QPushButton::clicked, this, &QueryContractDialog::on_searchButton_clicked);
}

QueryContractDialog::~QueryContractDialog()
{
    delete ui;
}

void QueryContractDialog::on_searchButton_clicked()
{
    performSearch();
}

void QueryContractDialog::performSearch() {
    int searchType = ui->searchTypeComboBox->currentIndex();
    QString keyword = ui->keywordLineEdit->text();
    int searchMethod = ui->searchMethodComboBox->currentIndex();

    if (keyword.isEmpty()) {
        QMessageBox::warning(this, tr("输入错误"), tr("请输入查询关键词"));
        return;
    }

    ui->resultTextEdit->clear();

    std::string keywordStr = keyword.toStdString();


    Node* current = contractList->getHead();
    while (current != nullptr) {
        bool match = false;
        if (searchMethod == 0) { // 精确查询
            if (searchType == 0 && current->contract.partyA == keywordStr) {
                match = true;
            } else if (searchType == 1 && current->contract.partyB == keywordStr) {
                match = true;
            } else if (searchType == 2 && (current->contract.partyA == keywordStr || current->contract.partyB == keywordStr)) {
                match = true;
            }
        } else { // 模糊查询
            if (searchType == 0 && current->contract.partyA.find(keywordStr) != std::string::npos) {
                match = true;
            } else if (searchType == 1 && current->contract.partyB.find(keywordStr) != std::string::npos) {
                match = true;
            } else if (searchType == 2 && (current->contract.partyA.find(keywordStr) != std::string::npos || current->contract.partyB.find(keywordStr) != std::string::npos)) {
                match = true;
            }
        }

        if (match) {
            current->contract.display(ui->resultTextEdit);
        }
        current = current->next;//指针current沿着next部分穿越整个列表
    }
}
