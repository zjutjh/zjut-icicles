#ifndef CONTRACTLIST_HPP
#define CONTRACTLIST_HPP

#include "Node.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class ContractList {
private:
    Node* head;

public:
    ContractList() : head(nullptr) {}
    ~ContractList();

    void addContract(const Contract& contract);
    void deleteContract(const std::string& contractNumber);
    Contract* findContract(const std::string& contractNumber);
    void displayAllContracts(QTextEdit* textEdit) const;
    void loadFromFile(const std::string& fileName);
    void saveToFile(const std::string& fileName) const;
    Node* getHead() const;
    std::vector<std::string> split(const std::string& s, char delimiter) const;
    void displayAllContractsSortedBySigningDate(QTextEdit* textEdit) const;
    void sortBySigningDate();
};

#endif // CONTRACTLIST_HPP
