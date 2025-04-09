#ifndef CONTRACT_HPP
#define CONTRACT_HPP

#include <string>
#include <QTextEdit>
class Contract {
public:
    std::string contractNumber = "N/A";
    std::string partyA = "N/A";
    std::string partyB = "N/A";
    std::string content = "N/A";
    std::string startDate = "N/A";
    std::string endDate = "N/A";
    std::string amount = "N/A";
    std::string signingDate = "N/A";

    void cleanFields();
    void readContent(const std::string& fileName);
    void exportToFile(const std::string& filePath) const;
    void modify(const std::string& contractNumber);
    void display(QTextEdit* textEdit) const;

private:
    std::string cleanString(const std::string& str) const;
    std::string extractValue(const std::string& content, const std::string& key) const;
};

void saveContractToDatabase(const Contract& contract);
void showAllContracts(QTextEdit* textEdit);
void searchContracts(QTextEdit* textEdit);
void deleteContract(const std::string& contractNumber);
void showExpiredContracts(QTextEdit* textEdit);
bool isContractNumberExist(const std::string &contractNumber);
void getContractsCountByMonth(QTextEdit* textEdit);
#endif // CONTRACT_HPP
