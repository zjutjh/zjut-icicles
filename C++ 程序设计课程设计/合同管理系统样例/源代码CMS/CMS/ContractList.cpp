#include "ContractList.hpp"

ContractList::~ContractList() {
    Node* current = head;
    while (current != nullptr) {
        Node* toDelete = current;
        current = current->next;
        delete toDelete;
    }
}

void ContractList::addContract(const Contract& contract) {
    Node* newNode = new Node(contract);
    newNode->next = head;
    head = newNode;
}

void ContractList::deleteContract(const std::string& contractNumber) {
    Node* current = head;
    Node* prev = nullptr;

    while (current != nullptr && current->contract.contractNumber != contractNumber) {
        prev = current;
        current = current->next;
    }

    if (current == nullptr) {
        std::cerr << "Contract not found: " << contractNumber << std::endl;
        return;
    }

    if (prev == nullptr) {
        head = current->next;
    } else {
        prev->next = current->next;
    }

    delete current;
}

Contract* ContractList::findContract(const std::string& contractNumber) {
    Node* current = head;

    while (current != nullptr) {
        if (current->contract.contractNumber == contractNumber) {
            return &current->contract;
        }
        current = current->next;
    }

    return nullptr;
}
#include<QDate>
void ContractList::displayAllContracts(QTextEdit* textEdit) const {
    std::vector<Node*> nodes;
    Node* current = head;
    while (current != nullptr) {
        nodes.push_back(current);
        current = current->next;
    }

    std::sort(nodes.begin(), nodes.end(), [](Node* a, Node* b) {
        QDate dateA = QDate::fromString(QString::fromStdString(a->contract.signingDate), "yyyy-MM-dd");
        QDate dateB = QDate::fromString(QString::fromStdString(b->contract.signingDate), "yyyy-MM-dd");
        return dateA < dateB;
    });

    textEdit->clear();
    for (Node* node : nodes) {
        node->contract.display(textEdit);
    }
}
Node* ContractList::getHead() const {
    return head;
}
void ContractList::loadFromFile(const std::string& fileName) {
    std::ifstream file(fileName);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> tokens = split(line, '\\'); // 使用反斜杠作为分隔符
        if (tokens.size() >= 8) {
            Contract contract;
            contract.contractNumber = tokens[0];
            contract.partyA = tokens[1];
            contract.partyB = tokens[2];
            contract.content = tokens[3];
            contract.startDate = tokens[4];
            contract.endDate = tokens[5];
            contract.amount = tokens[6];
            contract.signingDate = tokens[7];
            addContract(contract);
        }
    }

    file.close();
}
std::vector<std::string> ContractList::split(const std::string& s, char delimiter) const {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

void ContractList::saveToFile(const std::string& fileName) const {
    std::ofstream file(fileName);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return;
    }

    Node* current = head;

    while (current != nullptr) {
        const Contract& contract = current->contract;
        file << contract.contractNumber << "\\"
             << contract.partyA << "\\"
             << contract.partyB << "\\"
             << contract.content << "\\"
             << contract.startDate << "\\"
             << contract.endDate << "\\"
             << contract.amount << "\\"
             << contract.signingDate << "\n";
        current = current->next;
    }

    file.close();
}



