#include "Database.hpp"

std::vector<Employee> Database::loadEmployees() {
    std::ifstream inFile(dbDir + "data.dat", std::ios::binary);
    std::vector<Employee> employees{};
    size_t metaSize = 0;
    inFile >> metaSize;
    inFile.seekg(metaSize, std::ios::cur);
    char starter = false;
    inFile >> starter;
    while (starter == '\x1f') {
        starter = false; // 旧 C++ 标准在文件尾不会覆盖旧数据，需要手动重置
        Employee e;
        inFile >> e;
        if (inFile) {
            employees.push_back(e);
        }
        inFile >> starter;
    }
    return employees;
}

bool Database::saveData(const std::vector<Employee> &employees) {
    std::filesystem::create_directories(dbDir);
    std::ofstream outFile(dbDir + "data.dat", std::ios::binary);
    if (!outFile) {
        return false;
    }
    size_t metaSize = 0;
    outFile << metaSize;
    for (const Employee &e : employees) {
        outFile << '\x1f' << e;
    }
    return true;
}
