#include "EditContractDialog.h"
#include "ui_EditContractDialog.h"
#include <QMessageBox>
#include <QDate>

EditContractDialog::EditContractDialog(QWidget *parent, ContractList* contractList, bool isNewContract) :
    QDialog(parent),
    ui(new Ui::EditContractDialog),
    contractList(contractList),
    isNewContract(isNewContract),
    dateValidationErrorShown(false),
    contractNumberValidationErrorShown(false)
{
    ui->setupUi(this);

    // 连接信号到槽函数
    connect(ui->contractNumberLineEdit, &QLineEdit::editingFinished, this, &EditContractDialog::validateContractNumber);
    connect(ui->startDateLineEdit, &QLineEdit::editingFinished, this, &EditContractDialog::validateDate);
    connect(ui->endDateLineEdit, &QLineEdit::editingFinished, this, &EditContractDialog::validateDate);
    connect(ui->signingDateLineEdit, &QLineEdit::editingFinished, this, &EditContractDialog::validateDate);
    connect(ui->amountLineEdit, &QLineEdit::editingFinished, this, &EditContractDialog::validateAmount);
    connect(ui->saveButton, &QPushButton::clicked, this, &EditContractDialog::saveContract);
}

EditContractDialog::~EditContractDialog()
{
    delete ui;
}

void EditContractDialog::loadContract(const Contract &contract)
{
    ui->contractNumberLineEdit->setText(QString::fromStdString(contract.contractNumber));
    ui->partyALineEdit->setText(QString::fromStdString(contract.partyA));
    ui->partyBLineEdit->setText(QString::fromStdString(contract.partyB));
    ui->contentTextEdit->setPlainText(QString::fromStdString(contract.content));
    ui->startDateLineEdit->setText(QString::fromStdString(contract.startDate));
    ui->endDateLineEdit->setText(QString::fromStdString(contract.endDate));
    ui->amountLineEdit->setText(QString::fromStdString(contract.amount));
    ui->signingDateLineEdit->setText(QString::fromStdString(contract.signingDate));
}

Contract EditContractDialog::getUpdatedContract() const
{
    Contract contract;
    contract.contractNumber = ui->contractNumberLineEdit->text().toStdString();
    contract.partyA = ui->partyALineEdit->text().toStdString();
    contract.partyB = ui->partyBLineEdit->text().toStdString();
    contract.content = ui->contentTextEdit->toPlainText().toStdString();
    contract.startDate = ui->startDateLineEdit->text().toStdString();
    contract.endDate = ui->endDateLineEdit->text().toStdString();
    contract.amount = ui->amountLineEdit->text().toStdString();
    contract.signingDate = ui->signingDateLineEdit->text().toStdString();
    return contract;
}



void EditContractDialog::validateDate()
{
    QLineEdit *dateLineEdit = qobject_cast<QLineEdit*>(sender());
    QString dateText = dateLineEdit->text();

    if (!isValidDate(dateText)) {
        if (!dateValidationErrorShown) {
            QMessageBox::warning(this, tr("输入错误"), tr("日期格式(YYYY-MM-DD)错误\n或日期无效"));
            dateValidationErrorShown = true;
        }
        dateLineEdit->setFocus();
    } else {
        dateValidationErrorShown = false;
    }
}

bool EditContractDialog::isValidDate(const QString &date)
{
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



bool EditContractDialog::validateContractNumber()
{
    QString contractNumber = ui->contractNumberLineEdit->text();
    bool ok;
    int number = contractNumber.toInt(&ok);
    if (contractNumber.isEmpty() || !ok) {
        QMessageBox::warning(this, tr("输入错误"), tr("合同编号必须为数字"));
        ui->contractNumberLineEdit->setFocus();
        return false;
    }
    if (isNewContract && contractList && contractList->findContract(contractNumber.toStdString())) {
        if (!contractNumberValidationErrorShown) {
            QMessageBox::warning(this, tr("输入错误"), tr("合同编号 %1 已经存在，请使用不同的合同编号").arg(contractNumber));
            contractNumberValidationErrorShown = true;
        }
        ui->contractNumberLineEdit->setFocus();
            return false;
    }
    return true;
}

bool EditContractDialog::validateAmount()
{
    QString amount = ui->amountLineEdit->text();
    bool ok;
    int value = amount.toInt(&ok);
    if (amount.isEmpty() || !ok) {
        QMessageBox::warning(this, tr("输入错误"), tr("金额必须为数字"));
        ui->amountLineEdit->setFocus();
        return false;
    }
    return true;
}
void EditContractDialog::saveContract()
{
    if (ui->contractNumberLineEdit->text().isEmpty() ||
        ui->partyALineEdit->text().isEmpty() ||
        ui->partyBLineEdit->text().isEmpty() ||
        ui->startDateLineEdit->text().isEmpty() ||
        ui->endDateLineEdit->text().isEmpty() ||
        ui->amountLineEdit->text().isEmpty() ||
        ui->signingDateLineEdit->text().isEmpty()) {
        QMessageBox::warning(this, tr("输入错误"), tr("所有字段都必须填写"));
        return;
    }

    if (!isValidDate(ui->startDateLineEdit->text()) ||
        !isValidDate(ui->endDateLineEdit->text()) ||
        !isValidDate(ui->signingDateLineEdit->text())) {
        QMessageBox::warning(this, tr("输入错误"), tr("日期格式(YYYY-MM-DD)错误\n或日期无效"));
        return;
    }

    QDate signingDate = QDate::fromString(ui->signingDateLineEdit->text(), "yyyy-MM-dd");
    QDate startDate = QDate::fromString(ui->startDateLineEdit->text(), "yyyy-MM-dd");
    QDate endDate = QDate::fromString(ui->endDateLineEdit->text(), "yyyy-MM-dd");

    if (signingDate.isValid() && startDate.isValid() && endDate.isValid()) {
        if (signingDate > startDate || startDate > endDate) {
            QMessageBox::warning(this, tr("输入错误"), tr("日期时间顺序不对，请重新编辑"));
            return;
        }
    } else {
        QMessageBox::warning(this, tr("输入错误"), tr("日期格式无效，请重新编辑"));
        return;
    }

    if (!validateContractNumber() || !validateAmount()) {
        return;
    }

    accept();
}


