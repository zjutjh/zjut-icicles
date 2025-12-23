#include "./include/ReadFile.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <locale>
#include <codecvt>
#include <QTextEdit>
//#include "./lib/sqlite3.h"
#include <QTextEdit>
#include "./include/ContractList.hpp"

#include <QDate>
std::wstring string_to_wstring(const std::string& str) {
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    return converter.from_bytes(str);
}

std::string wstring_to_string(const std::wstring& wstr) {
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    return converter.to_bytes(wstr);
}

std::string Contract::cleanString(const std::string& str) const {
    std::wstring wide_str = string_to_wstring(str);

    std::wstring cleaned;
    wchar_t ch1 = L' ', ch2 = L':', ch3 = L'：', ch4 = L'(', ch5 = L')', ch6 = L'（', ch7 = L'）';
    for (wchar_t ch : wide_str)  {
        if (ch == ch1 || ch == ch2 || ch == ch3 || ch == ch4 || ch == ch5 || ch == ch6 || ch == ch7) {
            continue;
        }
        cleaned += ch;
    }
    return wstring_to_string(cleaned);
}

std::string Contract::extractValue(const std::string& content, const std::string& key) const {
    std::string::size_type startPos = content.find(key);
    if (startPos == std::string::npos) {
        return "N/A";
    }
    startPos += key.length();
    std::string::size_type endPos = content.find('\n', startPos);
    if (endPos == std::string::npos) {
        endPos = content.length();
    }
    return content.substr(startPos, endPos - startPos);
}

void Contract::cleanFields() {
    contractNumber = cleanString(contractNumber);
    partyA = cleanString(partyA);
    partyB = cleanString(partyB);
    startDate = cleanString(startDate);
    endDate = cleanString(endDate);
    amount = cleanString(amount);
    signingDate = cleanString(signingDate);
}

void Contract::readContent(const std::string& fileName) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string fileContent = buffer.str();
    file.close();


    fileContent = cleanString(fileContent);

    contractNumber = extractValue(fileContent, "编号");
    partyA = extractValue(fileContent, "甲方");
    partyB = extractValue(fileContent, "乙方");
    content = extractValue(fileContent, "内容");
    startDate = extractValue(fileContent, "开始时间");
    endDate = extractValue(fileContent, "结束时间");
    amount = extractValue(fileContent, "金额");
    signingDate = extractValue(fileContent, "日期");


    cleanFields();
}


void Contract::display(QTextEdit* textEdit) const {
    textEdit->append(QString::fromStdString("合同编号: " + contractNumber));
    textEdit->append(QString::fromStdString("甲方: " + partyA));
    textEdit->append(QString::fromStdString("乙方: " + partyB));
    textEdit->append(QString::fromStdString("内容: " + content));
    textEdit->append(QString::fromStdString("开始时间: " + startDate));
    textEdit->append(QString::fromStdString("结束时间: " + endDate));
    textEdit->append(QString::fromStdString("金额: " + amount));
    textEdit->append(QString::fromStdString("签订日期: " + signingDate));
    textEdit->append("-------------------------");
}

