#ifndef EDITCONTRACTDIALOG_H
#define EDITCONTRACTDIALOG_H

#include <QDialog>
#include "./include/ContractList.hpp"

namespace Ui {
class EditContractDialog;
}

class EditContractDialog : public QDialog
{
    Q_OBJECT

public:
    explicit EditContractDialog(QWidget *parent = nullptr, ContractList* contractList = nullptr, bool isNewContract = false);
    ~EditContractDialog();
    void loadContract(const Contract &contract);
    Contract getUpdatedContract() const;

private slots:
    bool validateContractNumber();
    void validateDate();
    bool validateAmount();
    void saveContract();

private:
    Ui::EditContractDialog *ui;
    ContractList* contractList;
    bool isNewContract;
    bool dateValidationErrorShown;
    bool contractNumberValidationErrorShown;
    bool isValidDate(const QString &date);
};

#endif // EDITCONTRACTDIALOG_H
